///|/ Copyright (c) Prusa Research 2023 Tomáš Mészáros @tamasmeszaros
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef ARRANGETASK_IMPL_HPP
#define ARRANGETASK_IMPL_HPP

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <set>

#include <boost/log/trivial.hpp>

#include <libslic3r/SVG.hpp>
#include <libslic3r/ClipperUtils.hpp>

#include <arrange-wrapper/Tasks/ArrangeTask.hpp>
#include <arrange-wrapper/Items/ArrangeItem.hpp>
#include <arrange-wrapper/SceneBuilder.hpp>

namespace Slic3r { namespace arr2 {

template<class ArrItem>
static std::set<uint16_t> collect_scene_extruders(const Scene &scene)
{
    std::set<uint16_t> used_extruders;

    auto slicer_model = dynamic_cast<const ArrangeableSlicerModel*>(&scene.model());
    if (slicer_model == nullptr) {
        used_extruders.insert(uint16_t(0));
        return used_extruders;
    }

    // If the user is arranging a selection, respect only the selected geometry.
    // This avoids activating exclusion zones from unrelated objects/tools.
    const std::set<ObjectID> selected_objects = selected_geometry_ids(scene);
    const bool filter_to_selected = ! selected_objects.empty();

    const Model &model = slicer_model->get_model();
    for (const ModelObject *obj : model.objects) {
        if (obj == nullptr)
            continue;
        if (filter_to_selected && selected_objects.find(obj->id()) == selected_objects.end())
            continue;
        for (const ModelVolume *mv : obj->volumes) {
            if (mv == nullptr || !mv->is_model_part())
                continue;
            const int extruder_id = mv->extruder_id();
            const int area_id = extruder_id > 0 ? extruder_id - 1 : 0;
            if (area_id >= 0)
                used_extruders.insert(uint16_t(area_id));
        }
    }

    if (used_extruders.empty())
        used_extruders.insert(uint16_t(0));

    return used_extruders;
}

template<class ArrItem>
static void apply_bed_exclude_areas(ArrangeTask<ArrItem> &task, const Scene &scene)
{
    const std::vector<Polygons> &exclude_areas = scene.bed_exclude_areas();
    if (exclude_areas.empty())
        return;

    const std::set<uint16_t> used_extruders = collect_scene_extruders<ArrItem>(scene);

    Polygons blocked;
    for (size_t area_idx = 0; area_idx < exclude_areas.size(); ++ area_idx)
        if (used_extruders.find(uint16_t(area_idx)) != used_extruders.end())
            append(blocked, exclude_areas[area_idx]);

    if (blocked.empty())
        return;

    static constexpr double exclude_safety_margin_mm = 0.20;
    const coord_t exclude_safety_margin = scale_(exclude_safety_margin_mm);
    if (exclude_safety_margin > 0)
        blocked = offset(blocked, exclude_safety_margin);

    constexpr int exclude_obstacle_priority = std::numeric_limits<int>::min();
    auto append_obstacles = [&blocked](std::vector<ArrItem> &fixed_items, int bed_idx) {
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
            fixed_items.emplace_back(std::move(obstacle));
        }
    };

    std::set<int> bed_indices;
    auto collect_bed_idx = [&bed_indices](const ArrItem &itm) {
        if (auto c = get_bed_constraint(itm); c.has_value())
            bed_indices.insert(*c);
        else if (const int b = get_bed_index(itm); b >= 0)
            bed_indices.insert(b);
    };
    for (const ArrItem &itm : task.printable.selected)   collect_bed_idx(itm);
    for (const ArrItem &itm : task.printable.unselected) collect_bed_idx(itm);
    for (const ArrItem &itm : task.unprintable.selected) collect_bed_idx(itm);
    for (const ArrItem &itm : task.unprintable.unselected) collect_bed_idx(itm);
    if (bed_indices.empty())
        bed_indices.insert(0);

    for (int bed_idx : bed_indices) {
        append_obstacles(task.printable.unselected, bed_idx);
        append_obstacles(task.unprintable.unselected, bed_idx);
    }
}

// Prepare the selected and unselected items separately. If nothing is
// selected, behaves as if everything would be selected.
template<class ArrItem>
void extract_selected(ArrangeTask<ArrItem> &task,
                      const ArrangeableModel &mdl,
                      const ArrangeableToItemConverter<ArrItem> &itm_conv)
{
    // Go through the objects and check if inside the selection
    mdl.for_each_arrangeable(
        [&task, &itm_conv](const Arrangeable &arrbl) {
            bool selected = arrbl.is_selected();
            bool printable = arrbl.is_printable();

            try {
                auto itm = itm_conv.convert(arrbl, selected ? 0 : -SCALED_EPSILON);

                auto &container_parent = printable ? task.printable :
                                                 task.unprintable;

                auto &container = selected ?
                                       container_parent.selected :
                                       container_parent.unselected;

                container.emplace_back(std::move(itm));
            } catch (const EmptyItemOutlineError &ex) {
                BOOST_LOG_TRIVIAL(error)
                    << "ObjectID " << std::to_string(arrbl.id().id) << ": " << ex.what();
            }
        });
}

template<class ArrItem>
std::unique_ptr<ArrangeTask<ArrItem>> ArrangeTask<ArrItem>::create(
    const Scene &sc, const ArrangeableToItemConverter<ArrItem> &converter)
{
    auto task = std::make_unique<ArrangeTask<ArrItem>>();

    task->settings.set_from(sc.settings());
    // Product requirement: Arrange always centers the placement on the bed.
    task->settings
        .set_arrange_strategy(ArrangeSettingsView::asPullToCenter)
        .set_xl_alignment(ArrangeSettingsView::xlpCenter);

    task->bed = get_corrected_bed(sc.bed(), converter);
    extract_selected(*task, sc.model(), converter);
    apply_bed_exclude_areas(*task, sc);

    return task;
}

// Remove all items on the physical bed (not occupyable for unprintable items)
// and shift all items to the next lower bed index, so that arrange will think
// that logical bed no. 1 is the physical one
template<class ItemCont>
void prepare_fixed_unselected(ItemCont &items, int shift)
{
    for (auto &itm : items)
        set_bed_index(itm, get_bed_index(itm) - shift);

    items.erase(std::remove_if(items.begin(), items.end(),
                               [](auto &itm) { return !is_arranged(itm); }),
                items.end());
}

inline int find_first_empty_bed(const std::vector<int>& bed_indices,
                                int starting_from = 0) {
    int ret = starting_from;

    for (int idx : bed_indices) {
        if (idx == ret) {
            ret++;
        } else if (idx > ret) {
            break;
        }
    }

    return ret;
}

template<class ArrItem>
std::unique_ptr<ArrangeTaskResult>
ArrangeTask<ArrItem>::process_native(Ctl &ctl)
{
    auto result = std::make_unique<ArrangeTaskResult>();

    auto arranger = Arranger<ArrItem>::create(settings);

    class TwoStepArrangeCtl: public Ctl
    {
        Ctl &parent;
        ArrangeTask &self;
    public:
        TwoStepArrangeCtl(Ctl &p, ArrangeTask &slf) : parent{p}, self{slf} {}

        void update_status(int remaining) override
        {
            parent.update_status(remaining + self.unprintable.selected.size());
        }

        bool was_canceled() const override { return parent.was_canceled(); }

    } subctl{ctl, *this};

    arranger->arrange(printable.selected, printable.unselected, bed, subctl);

    std::vector<int> printable_bed_indices =
        get_bed_indices(crange(printable.selected), crange(printable.unselected));

    // If there are no printables, leave the physical bed empty
    static constexpr int SearchFrom = 1;

    // Unprintable items should go to the first logical (!) bed not containing
    // any printable items
    int first_empty_bed = find_first_empty_bed(printable_bed_indices, SearchFrom);

    prepare_fixed_unselected(unprintable.unselected, first_empty_bed);

    arranger->arrange(unprintable.selected, unprintable.unselected, bed, ctl);

    result->add_items(crange(printable.selected));

    for (auto &itm : unprintable.selected) {
        if (is_arranged(itm)) {
            int bedidx = get_bed_index(itm) + first_empty_bed;
            arr2::set_bed_index(itm, bedidx);
        }

        result->add_item(itm);
    }

    return result;
}

} // namespace arr2
} // namespace Slic3r

#endif //ARRANGETASK_IMPL_HPP
