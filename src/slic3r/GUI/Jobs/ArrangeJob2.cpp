///|/ Copyright (c) Prusa Research 2023 Tomáš Mészáros @tamasmeszaros
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "ArrangeJob2.hpp"

#include <numeric>
#include <iterator>

#include <libslic3r/Model.hpp>
#include <libslic3r/TriangleMeshSlicer.hpp>
#include <libslic3r/Geometry/ConvexHull.hpp>

#include <libslic3r/SLAPrint.hpp>
#include <libslic3r/Print.hpp>

#include <slic3r/GUI/Plater.hpp>
#include <slic3r/GUI/GLCanvas3D.hpp>
#include <slic3r/GUI/GUI_App.hpp>

#include <boost/log/trivial.hpp>

namespace Slic3r { namespace GUI {

class GUISelectionMask: public arr2::SelectionMask {
    const Selection *m_sel;

public:
    explicit GUISelectionMask(const Selection *sel) : m_sel{sel} {}

    bool is_wipe_tower_selected(int wipe_tower_index) const override
    {
        const GLVolume *volume{GUI::get_selected_gl_volume(*m_sel)};
        if (volume != nullptr && volume->wipe_tower_bed_index == wipe_tower_index) {
            return true;
        }
        return false;
    }

    std::vector<bool> selected_objects() const override
    {
        auto selmap = m_sel->get_object_idxs();

        std::vector<bool> ret(m_sel->get_model()->objects.size(), false);

        for (auto sel : selmap) {
            ret[sel] = true;
        }

        return ret;
    }

    std::vector<bool> selected_instances(int obj_id) const override
    {
        auto objcnt = static_cast<int>(m_sel->get_model()->objects.size());
        auto icnt   = obj_id < objcnt ?
                          m_sel->get_model()->objects[obj_id]->instances.size() :
                          0;

        std::vector<bool> ret(icnt, false);

        auto selmap = m_sel->get_content();
        auto objit = selmap.find(obj_id);

        if (objit != selmap.end() && obj_id < objcnt) {
            ret = std::vector<bool>(icnt, false);
            for (auto sel : objit->second) {
                ret[sel] = true;
            }
        }

        return ret;
    }
};

class BedSelectionMask: public arr2::SelectionMask {
    const int m_bed_index;
    std::vector<std::vector<bool>> m_selected_instances;
    std::vector<bool> m_selected_objects;

public:
    explicit BedSelectionMask(const int bed_index, const ModelObjectPtrs &objects, const std::set<ObjectID> &instances_on_bed):
        m_bed_index{bed_index},
        m_selected_instances(get_selected_instances(objects, instances_on_bed)),
        m_selected_objects(get_selected_objects(this->m_selected_instances))
        {}

    bool is_wipe_tower_selected(int wipe_tower_index) const override
    {
        return wipe_tower_index == m_bed_index;
    }

    std::vector<bool> selected_objects() const override
    {
        return this->m_selected_objects;
    }

    std::vector<bool> selected_instances(int obj_id) const override {
        return this->m_selected_instances[obj_id];
    }

private:
    static std::vector<bool> get_selected_objects(
        const std::vector<std::vector<bool>> &selected_instances
    ) {
        std::vector<bool> result;

        std::transform(
            selected_instances.begin(),
            selected_instances.end(),
            std::back_inserter(result),
            [&](const std::vector<bool> &object_selected_instances) {
                return std::any_of(
                    object_selected_instances.begin(),
                    object_selected_instances.end(),
                    [](const bool is_selected){ return is_selected; }
                );
            }
        );

        return result;
    }

    std::vector<bool> get_selected_instances(
        const ModelObject &object,
        const std::set<ObjectID> &instances_on_bed
    ) {
        std::vector<bool> result;
        std::transform(
            object.instances.begin(),
            object.instances.end(),
            std::back_inserter(result),
            [&](const ModelInstance *instance) {
                const auto instance_bed_index{instances_on_bed.find(instance->id())};
                if(instance_bed_index != instances_on_bed.end()) {
                    return true;
                }
                return false;
            }
        );
        return result;
    }

    std::vector<std::vector<bool>> get_selected_instances(
        const ModelObjectPtrs &objects,
        const std::set<ObjectID> &instances_on_bed
    ) {
        std::vector<std::vector<bool>> result;
        std::transform(
            objects.begin(),
            objects.end(),
            std::back_inserter(result),
            [&](const ModelObject *object){
                return get_selected_instances(*object, instances_on_bed);
            }
        );
        return result;
    }
};

static Polygon get_wtpoly(const GLCanvas3D::WipeTowerInfo &wti)
{

    auto bb = scaled(wti.bounding_box());
    Polygon poly = Polygon({
        {bb.min},
        {bb.max.x(), bb.min.y()},
        {bb.max},
        {bb.min.x(), bb.max.y()}
    });

    poly.rotate(wti.rotation());
    poly.translate(scaled(wti.pos()));

    return poly;
}

// Wipe tower logic based on GLCanvas3D::WipeTowerInfo implements the Arrangeable
// interface with this class:
class ArrangeableWT: public arr2::ArrangeableWipeTowerBase
{
    BoundingBox m_xl_bb;
    Vec2d m_orig_tr;
    double m_orig_rot;

public:
    explicit ArrangeableWT(const ObjectID                  &oid,
                           const GLCanvas3D::WipeTowerInfo &wti,
                           std::function<bool(int)>            sel_pred,
                           const BoundingBox                xl_bb = {})
        : arr2::ArrangeableWipeTowerBase{oid, get_wtpoly(wti), wti.bed_index(), std::move(sel_pred)}
        , m_orig_tr{wti.pos()}
        , m_orig_rot{wti.rotation()}
        , m_xl_bb{xl_bb}
    {}

    // Rotation is disabled for wipe tower in arrangement
    void transform(const Vec2d &transl, double /*rot*/) override
    {
        GLCanvas3D::WipeTowerInfo::apply_wipe_tower(m_orig_tr + transl, m_orig_rot, this->bed_index);
    }

    void imbue_data(arr2::AnyWritable &datastore) const override
    {
        // For XL printers, there is a requirement that the wipe tower
        // needs to be placed right beside the extruders which reside at the
        // top edge of the bed.
        if (m_xl_bb.defined) {
            Vec2crd xl_center = m_xl_bb.center();
            datastore.write("sink", Vec2crd{xl_center.x(), 2 * m_xl_bb.max.y()});
        }

        arr2::ArrangeableWipeTowerBase::imbue_data(datastore);
    }
};

// Now the wipe tower handler implementation for GLCanvas3D::WipeTowerInfo
// This is what creates the ArrangeableWT when the arrangement requests it.
// An object of this class is installed into the arrangement Scene.
struct WTH : public arr2::WipeTowerHandler
{
    GLCanvas3D::WipeTowerInfo wti;
    ObjectID oid;
    std::function<bool(int)> sel_pred;
    BoundingBox xl_bb;

    WTH(const ObjectID &objid,
        const GLCanvas3D::WipeTowerInfo &w,
        std::function<bool(int)> sel_predicate = [](int){ return false; })
        : wti(w), oid{objid}, sel_pred{std::move(sel_predicate)}
    {}

    template<class Self, class Fn>
    static void visit_(Self &&self, Fn &&fn)
    {
        ArrangeableWT wta{self.oid, self.wti, self.sel_pred, self.xl_bb};
        fn(wta);
    }

    void visit(std::function<void(arr2::Arrangeable &)> fn) override
    {
        visit_(*this, fn);
    }

    void visit(std::function<void(const arr2::Arrangeable &)> fn) const override
    {
        visit_(*this, fn);
    }

    void set_selection_predicate(std::function<bool(int)> pred) override
    {
        sel_pred = std::move(pred);
    }

    ObjectID get_id() const override {
        return this->oid;
    }
};

arr2::SceneBuilder build_scene(Plater &plater, ArrangeSelectionMode mode)
{
    arr2::SceneBuilder builder;

    const int current_bed{s_multiple_beds.get_active_bed()};
    if (mode == ArrangeSelectionMode::SelectionOnly) {
        auto sel = std::make_unique<GUISelectionMask>(&plater.get_selection());
        builder.set_selection(std::move(sel));
    } else if (mode == ArrangeSelectionMode::CurrentBedSelectionOnly) {
        arr2::BedConstraints constraints;
        for (const ModelObject *object : plater.model().objects) {
            for (const ModelInstance *instance : object->instances) {
                constraints.insert({instance->id(), current_bed});
            }
        }
        builder.set_bed_constraints(std::move(constraints));

        auto gui_selection{std::make_unique<GUISelectionMask>(&plater.get_selection())};
        builder.set_selection(std::move(gui_selection));
    } else if (mode == ArrangeSelectionMode::CurrentBedFull) {
        std::set<ObjectID> instances_on_bed;
        arr2::BedConstraints constraints;
        for (const auto &instance_bed : s_multiple_beds.get_inst_map()) {
            if (instance_bed.second == current_bed) {
                instances_on_bed.emplace(instance_bed.first);
                constraints.emplace(instance_bed);
            }
        }
        builder.set_bed_constraints(std::move(constraints));

        auto bed_selection{std::make_unique<BedSelectionMask>(
            current_bed,
            plater.model().objects,
            instances_on_bed
        )};
        builder.set_selection(std::move(bed_selection));
    }

    builder.set_arrange_settings(plater.canvas3D()->get_arrange_settings_view());

    const auto wipe_tower_infos = plater.canvas3D()->get_wipe_tower_infos();

    std::vector<AnyPtr<arr2::WipeTowerHandler>> handlers;

    for (const auto &info : wipe_tower_infos) {
        if (info) {
            auto handler{std::make_unique<WTH>(wipe_tower_instance_id(info.bed_index()), info)};
            if (plater.config() && is_XL_printer(*plater.config())) {
                handler->xl_bb = bounding_box(get_bed_shape(*plater.config()));
            }
            handlers.push_back(std::move(handler));
        }
    }

    if (plater.config()) {
        const BedsGrid::Gap gap{s_multiple_beds.get_bed_gap()};
        builder.set_bed(*plater.config(), gap);
    }

    builder.set_wipe_tower_handlers(std::move(handlers));

    builder.set_model(plater.model());

    if (plater.printer_technology() == ptSLA)
        builder.set_sla_print(&plater.active_sla_print());
    else
        builder.set_fff_print(&plater.active_fff_print());

    return builder;
}

FillBedJob2::FillBedJob2(arr2::Scene &&scene, const Callbacks &cbs) : Base(std::move(scene), _u8L("Filling bed"), cbs) {}

ArrangeJob2::ArrangeJob2(arr2::Scene &&scene, const Callbacks &cbs) : Base(std::move(scene), _u8L("Arranging"), cbs) {}

}} // namespace Slic3r
