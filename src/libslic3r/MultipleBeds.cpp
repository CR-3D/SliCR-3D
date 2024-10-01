#include "MultipleBeds.hpp"

#include "BuildVolume.hpp"
#include "Model.hpp"

#include <cassert>

namespace Slic3r {

MultipleBeds s_multiple_beds;


Vec3d MultipleBeds::get_bed_translation(int id) const
{
    // TODO: Arrange defines this in LogicalBedGap in SceneBuilder.cpp
    // TODO: It should be defined as multiple of bed size.

    if (id == 0)
        return Vec3d::Zero();
    int x = 0;
    int y = 0;
#if 0
    // Linear layout
    x = id;
#else
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
#endif
    return 300. * Vec3d(x, y, 0.);
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

}
