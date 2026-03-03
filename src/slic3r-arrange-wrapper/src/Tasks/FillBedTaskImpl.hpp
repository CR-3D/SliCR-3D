///|/ Copyright (c) Prusa Research 2023 Tomáš Mészáros @tamasmeszaros
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef FILLBEDTASKIMPL_HPP
#define FILLBEDTASKIMPL_HPP

#include <algorithm>
#include <cmath>
#include <limits>

#include <boost/log/trivial.hpp>

#include <libslic3r/ClipperUtils.hpp>

#include <arrange/NFP/NFPArrangeItemTraits.hpp>

#include <arrange-wrapper/Tasks/FillBedTask.hpp>
#include <arrange-wrapper/SceneBuilder.hpp>

namespace Slic3r { namespace arr2 {

template<class ArrItem>
static void collect_prototype_extruders(FillBedTask<ArrItem> &task,
                                        const Scene &scene,
                                        const ObjectID &prototype_geometry_id)
{
    task.prototype_extruders.clear();

    auto slicer_model = dynamic_cast<const ArrangeableSlicerModel*>(&scene.model());
    if (slicer_model == nullptr)
        return;

    const Model &model = slicer_model->get_model();
    auto it = std::find_if(model.objects.begin(), model.objects.end(),
                           [&prototype_geometry_id](const ModelObject *obj) {
                               return obj != nullptr && obj->id() == prototype_geometry_id;
                           });
    if (it == model.objects.end() || *it == nullptr)
        return;

    for (const ModelVolume *mv : (*it)->volumes) {
        if (mv == nullptr || !mv->is_model_part())
            continue;
        const int extruder_id = mv->extruder_id();
        const int area_id = extruder_id > 0 ? extruder_id - 1 : 0;
        if (area_id >= 0)
            task.prototype_extruders.insert(uint16_t(area_id));
    }

    // If we cannot resolve the tool assignment, assume default tool 0.
    // This keeps left-side placement available instead of blocking both sides.
    if (task.prototype_extruders.empty())
        task.prototype_extruders.insert(uint16_t(0));
}

template<class ArrItem>
int calculate_items_needed_to_fill_bed(const ExtendedBed &bed,
                                       const ArrItem &prototype_item,
                                       size_t prototype_count,
                                       const std::vector<ArrItem> &fixed)
{
    double poly_area  = fixed_area(prototype_item);

    auto area_sum_fn = [&](double s, const auto &itm) {
        return s + (get_bed_index(itm) == get_bed_constraint(prototype_item)) * fixed_area(itm);
    };

    double unsel_area = std::accumulate(fixed.begin(),
                                        fixed.end(),
                                        0.,
                                        area_sum_fn);

    double fixed_area = unsel_area + prototype_count * poly_area;
    double bed_area   = 0.;

    visit_bed([&bed_area] (auto &realbed) { bed_area = area(realbed); }, bed);

    // This is the maximum number of items,
    // the real number will always be close but less.
    auto needed_items = static_cast<int>(
        std::ceil((bed_area - fixed_area) / poly_area));

    return needed_items;
}

template<class ArrItem>
void extract(FillBedTask<ArrItem> &task,
             const Scene &scene,
             const ArrangeableToItemConverter<ArrItem> &itm_conv)
{
    task.prototype_item = {};

    auto selected_ids = scene.selected_ids();

    if (selected_ids.empty())
        return;

    std::set<ObjectID> selected_objects = selected_geometry_ids(scene);

    if (selected_objects.size() != 1)
        return;

    ObjectID prototype_geometry_id = *(selected_objects.begin());
    collect_prototype_extruders(task, scene, prototype_geometry_id);

    auto set_prototype_item = [&task, &itm_conv](const Arrangeable &arrbl) {
        if (arrbl.is_printable())
            task.prototype_item = itm_conv.convert(arrbl);
    };

    scene.model().visit_arrangeable(selected_ids.front(), set_prototype_item);

    if (!task.prototype_item)
        return;
    if (task.prototype_extruders.empty())
        task.prototype_extruders.insert(uint16_t(0));

    // Workaround for missing items when arranging the same geometry only:
    // Injecting a number of items but with slightly shrinked shape, so that
    // they can fill the emerging holes.
    ArrItem prototype_item_shrinked;
    scene.model().visit_arrangeable(selected_ids.front(),
        [&prototype_item_shrinked, &itm_conv](const Arrangeable &arrbl) {
            if (arrbl.is_printable())
                prototype_item_shrinked = itm_conv.convert(arrbl, -SCALED_EPSILON);
        });

    const int bed_constraint{*get_bed_constraint(*task.prototype_item)};
    if (bed_constraint != get_bed_index(*task.prototype_item)) {
        return;
    }

    set_bed_index(*task.prototype_item, Unarranged);

    auto collect_task_items = [&prototype_geometry_id, &task,
                               &itm_conv, &bed_constraint](const Arrangeable &arrbl) {
        try {
            if (arrbl.bed_constraint() == bed_constraint) {
                if (arrbl.geometry_id() == prototype_geometry_id) {
                    if (arrbl.is_printable()) {
                        auto itm = itm_conv.convert(arrbl);
                        raise_priority(itm);
                        task.selected.emplace_back(std::move(itm));
                    }
                } else {
                    auto itm = itm_conv.convert(arrbl, -SCALED_EPSILON);
                    task.unselected.emplace_back(std::move(itm));
                }
            }
        } catch (const EmptyItemOutlineError &ex) {
            BOOST_LOG_TRIVIAL(error)
                << "ObjectID " << std::to_string(arrbl.id().id) << ": " << ex.what();
        }
    };

    scene.model().for_each_arrangeable(collect_task_items);

    int needed_items = calculate_items_needed_to_fill_bed(task.bed,
                                                          *task.prototype_item,
                                                          task.selected.size(),
                                                          task.unselected);

    task.selected_existing_count = task.selected.size();
    task.selected.reserve(task.selected.size() + needed_items);
    std::fill_n(std::back_inserter(task.selected), needed_items,
                *task.prototype_item);

    // Add as many filler items as there are needed items. Most of them will
    // be discarded anyways.
    std::fill_n(std::back_inserter(task.selected_fillers), needed_items,
                prototype_item_shrinked);
}

template<class ArrItem>
static void apply_bed_exclude_areas(FillBedTask<ArrItem> &task, const Scene &scene)
{
    const std::vector<Polygons> &exclude_areas = scene.bed_exclude_areas();
    if (exclude_areas.empty())
        return;

    if (task.prototype_extruders.empty())
        task.prototype_extruders.insert(uint16_t(0));

    Polygons blocked;
    for (uint16_t extruder_id : task.prototype_extruders)
        if (size_t(extruder_id) < exclude_areas.size())
            append(blocked, exclude_areas[size_t(extruder_id)]);

    if (blocked.empty())
        return;

    static constexpr double exclude_safety_margin_mm = 0.20;
    const coord_t exclude_safety_margin = scale_(exclude_safety_margin_mm);
    if (exclude_safety_margin > 0)
        blocked = offset(blocked, exclude_safety_margin);

    constexpr int exclude_obstacle_priority = std::numeric_limits<int>::min();
    const int bed_idx = task.prototype_item && get_bed_constraint(*task.prototype_item).has_value() ?
                        *get_bed_constraint(*task.prototype_item) : 0;

    for (const Polygon &poly : blocked) {
        if (poly.points.size() < 3 || std::abs(poly.area()) <= 0.)
            continue;
        ArrItem obstacle;
        set_shape(obstacle, ExPolygons{ ExPolygon{ poly } });
        // Mark as synthetic exclusion geometry so arrangement post-processing
        // can treat it differently from real fixed objects.
        set_priority(obstacle, exclude_obstacle_priority);
        set_bed_index(obstacle, bed_idx);
        set_bed_constraint(obstacle, bed_idx);
        task.unselected.emplace_back(std::move(obstacle));
    }
}


template<class ArrItem>
std::unique_ptr<FillBedTask<ArrItem>> FillBedTask<ArrItem>::create(
    const Scene &sc, const ArrangeableToItemConverter<ArrItem> &converter)
{
    auto task = std::make_unique<FillBedTask<ArrItem>>();

    task->settings.set_from(sc.settings());

    task->bed = get_corrected_bed(sc.bed(), converter);

    extract(*task, sc, converter);
    apply_bed_exclude_areas(*task, sc);

    return task;
}

template<class ArrItem>
std::unique_ptr<FillBedTaskResult> FillBedTask<ArrItem>::process_native(
    Ctl &ctl)
{
    auto result = std::make_unique<FillBedTaskResult>();

    if (!prototype_item)
        return result;

    result->prototype_id = retrieve_id(*prototype_item).value_or(ObjectID{});

    class FillBedCtl: public ArrangerCtl<ArrItem>
    {
        ArrangeTaskCtl &parent;
        FillBedTask &self;
        bool do_stop = false;

    public:
        FillBedCtl(ArrangeTaskCtl &p, FillBedTask &slf) : parent{p}, self{slf} {}

        void update_status(int remaining) override
        {
            parent.update_status(remaining);
        }

        bool was_canceled() const override
        {
            return parent.was_canceled() || do_stop;
        }

        void on_packed(ArrItem &itm) override
        {
            // Stop at the first filler that is not on the physical bed
            do_stop = get_bed_index(itm) == -1 && get_priority(itm) == 0;
        }

    } subctl(ctl, *this);

    auto arranger = Arranger<ArrItem>::create(settings);

    arranger->arrange(selected, unselected, bed, subctl);

    auto unsel_cpy = unselected;
    for (const auto &itm : selected) {
        unsel_cpy.emplace_back(itm);
    }

    arranger->arrange(selected_fillers, unsel_cpy, bed, FillBedCtl{ctl, *this});

    auto arranged_range = Range{selected.begin(),
                                selected.begin() + selected_existing_count};

    result->add_arranged_items(arranged_range);

    auto to_add_range = Range{selected.begin() + selected_existing_count,
                              selected.end()};

    for (auto &itm : to_add_range) {
        if (get_bed_index(itm) == get_bed_constraint(itm))
            result->add_new_item(itm);
    }

    for (auto &itm : selected_fillers)
        if (get_bed_index(itm) == get_bed_constraint(itm))
            result->add_new_item(itm);

    return result;
}

} // namespace arr2
} // namespace Slic3r

#endif // FILLBEDTASKIMPL_HPP
