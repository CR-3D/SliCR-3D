#include "MultipleBeds.hpp"

#include "BuildVolume.hpp"
#include "Model.hpp"

#include <cassert>

namespace Slic3r {

MultipleBeds s_multiple_beds;
bool s_reload_preview_after_switching_beds = false;
bool s_beds_just_switched = false;



Vec3d MultipleBeds::get_bed_translation(int id) const
{
    // The x value is bed gap as multiples of the actual printable area bounding box,
    // so it can be matched to how the old slicer arranged things (in SceneBuilder.cpp).
    // The y value is a multiple of the larger of printable area BB and bed model BB -
    // this is to make sure that the bed models do not overlap.
    const double bed_gap_x = 2./10;
    const double bed_gap_y = 2./10;

    if (id == 0)
        return Vec3d::Zero();
    int x = 0;
    int y = 0;
    if (m_layout_linear)
        x = id;
    else {
        // Grid layout.
        ++id;
        int a = 1;
        while ((a+1)*(a+1) < id)
            ++a;
        id = id - a*a;
        x=a;
        y=a;
        if (id <= a)
            y = id-1;
        else
            x=id-a-1;
    }
    return Vec3d(x * m_build_volume_bb.size().x() * (1. + bed_gap_x), y * m_build_volume_bb.size().y() * (1. + bed_gap_y), 0.);
}







void MultipleBeds::clear_inst_map()
{
    m_inst_to_bed.clear();
}

void MultipleBeds::set_instance_bed(ObjectID id, int bed_idx)
{
    assert(bed_idx < get_max_beds());
    m_inst_to_bed[id] = bed_idx;
}

void MultipleBeds::inst_map_updated()
{
    int max_bed_idx = 0;
    for (const auto& [obj_id, bed_idx] : m_inst_to_bed)
        max_bed_idx = std::max(max_bed_idx, bed_idx);

    if (m_number_of_beds != max_bed_idx + 1) {
        m_number_of_beds = max_bed_idx + 1;
        m_active_bed = m_number_of_beds - 1;
        request_next_bed(false);
    }
    if (m_active_bed >= m_number_of_beds)
        m_active_bed = m_number_of_beds - 1;
}

void MultipleBeds::request_next_bed(bool show)
{
    m_show_next_bed = (get_number_of_beds() < get_max_beds() ? show : false);
}

void MultipleBeds::set_active_bed(int i)
{
    assert(i < get_max_beds());
    if (i<m_number_of_beds)
        m_active_bed = i;
}

void MultipleBeds::move_active_to_first_bed(Model& model, const BuildVolume& build_volume, bool to_or_from) const
{
    static std::vector<std::pair<Vec3d, bool>> old_state;
    size_t i = 0;
    assert(! to_or_from || old_state.empty());

    for (ModelObject* mo : model.objects) {
        for (ModelInstance* mi : mo->instances) {
            if (to_or_from) {
                old_state.resize(i+1);
                old_state[i] = std::make_pair(mi->get_offset(), mi->printable);
                if (this->is_instance_on_active_bed(mi->id()))
                    mi->set_offset(mi->get_offset() - get_bed_translation(get_active_bed()));
                else
                    mi->printable = false;
            } else {
                mi->set_offset(old_state[i].first);
                mi->printable = old_state[i].second;
            }
            ++i;
        }
    }
    if (! to_or_from)
        old_state.clear();
}



bool MultipleBeds::is_instance_on_active_bed(ObjectID id) const
{
    auto it = m_inst_to_bed.find(id);
    return (it != m_inst_to_bed.end() && it->second == m_active_bed);
}



bool MultipleBeds::is_glvolume_on_thumbnail_bed(const Model& model, int obj_idx, int instance_idx) const
{
    if (obj_idx < 0 || instance_idx < 0 || obj_idx >= int(model.objects.size()) || instance_idx >= int(model.objects[obj_idx]->instances.size()))
        return false;

    auto it = m_inst_to_bed.find(model.objects[obj_idx]->instances[instance_idx]->id());
    if (it == m_inst_to_bed.end())
        return false;
    return (m_bed_for_thumbnails_generation < 0 || it->second == m_bed_for_thumbnails_generation);
}



bool MultipleBeds::rearrange_linear_to_grid_if_possible(Model& model, const BuildVolume& build_volume)
{
    m_layout_linear = true;
    int old_number_of_beds = m_number_of_beds;
    m_number_of_beds = get_max_beds();
    Slic3r::ScopeGuard guard([this]() { m_layout_linear = false; });
    model.update_print_volume_state(build_volume);
    m_number_of_beds = old_number_of_beds;

    int max_bed = 0;

    std::map<ObjectID, std::pair<ModelInstance*, int>> id_to_ptr_and_bed;
    for (ModelObject* mo : model.objects) {
        for (ModelInstance* mi : mo->instances) {
            auto it = m_inst_to_bed.find(mi->id());
            if (it == m_inst_to_bed.end()) {
                // An instance is outside. Do not rearrange anything,
                // that could create collisions.
                return false;
            }
            id_to_ptr_and_bed[mi->id()] = std::make_pair(mi, it->second);
            max_bed = std::max(max_bed, it->second);
        }
    }

    if (max_bed == 0) {
        // All instances are on the first bed. No need to do anything.
        return false;
    }
    m_number_of_beds = max_bed + 1;
    assert(m_number_of_beds <= get_max_beds());

    // All instances are on some bed, at least two are used.
    for (auto& [oid, mi_and_bed] : id_to_ptr_and_bed) {
        auto& [mi, bed_idx] = mi_and_bed;
        mi->set_offset(mi->get_offset() - get_bed_translation(bed_idx));
    }

    m_layout_linear = false;
    for (auto& [oid, mi_and_bed] : id_to_ptr_and_bed) {
        auto& [mi, bed_idx] = mi_and_bed;
        mi->set_offset(mi->get_offset() + get_bed_translation(bed_idx));
    }
    model.update_print_volume_state(build_volume);
    return true;
}



}

