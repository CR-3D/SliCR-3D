#include "Layer.hpp"
#include "ClipperUtils.hpp"
#include "ClipperZUtils.hpp"
#include "Print.hpp"
#include "Fill/Fill.hpp"
#include "ShortestPath.hpp"
#include "SVG.hpp"
#include "BoundingBox.hpp"
#include "ExtrusionEntity.hpp"

#include <boost/log/trivial.hpp>

namespace Slic3r {

Layer::~Layer()
{
    this->lower_layer = this->upper_layer = nullptr;
    for (LayerRegion *region : m_regions)
        delete region;
    m_regions.clear();
}

// Test whether whether there are any slices assigned to this layer.
bool Layer::empty() const
{
    for (const LayerRegion *layerm : m_regions)
        if (layerm != nullptr && ! layerm->slices().empty())
            // Non empty layer.
            return false;
    return true;
}

LayerRegion* Layer::add_region(const PrintRegion *print_region)
{
    m_regions.emplace_back(new LayerRegion(this, print_region));
    return m_regions.back();
}

// merge all regions' slices to get islands
void Layer::make_slices()
{
    ExPolygons slices;
    if (m_regions.size() == 1) {
        // optimization: if we only have one region, take its slices
        slices = to_expolygons(m_regions.front()->slices().surfaces);
    } else {
        Polygons slices_p;
        for (LayerRegion *layerm : m_regions)
            polygons_append(slices_p, to_polygons(layerm->slices().surfaces));
        slices = union_safety_offset_ex(slices_p);
    }
    
    this->lslices.clear();
    this->lslices.reserve(slices.size());
    
    // prepare ordering points
    Points ordering_points;
    ordering_points.reserve(slices.size());
    for (const ExPolygon &ex : slices)
        ordering_points.push_back(ex.contour.first_point());
    
    // sort slices
    std::vector<Points::size_type> order = chain_points(ordering_points);
    
    // populate slices vector
    for (size_t i : order)
        this->lslices.emplace_back(std::move(slices[i]));
}

static inline bool layer_needs_raw_backup(const Layer *layer)
{
    return ! (layer->regions().size() == 1 && (layer->id() > 0 || layer->object()->config().first_layer_size_compensation.value == 0));
}

void Layer::backup_untyped_slices()
{
    if (layer_needs_raw_backup(this)) {
        for (LayerRegion *layerm : m_regions)
            layerm->raw_slices = to_expolygons(layerm->slices().surfaces);
    } else {
        assert(m_regions.size() == 1);
        m_regions.front()->raw_slices.clear();
    }
}

void Layer::restore_untyped_slices()
{
    if (layer_needs_raw_backup(this)) {
        for (LayerRegion *layerm : m_regions)
            layerm->m_slices.set(layerm->raw_slices, stPosInternal | stDensSparse);
    } else {
        assert(m_regions.size() == 1);
        m_regions.front()->m_slices.set(this->lslices, stPosInternal | stDensSparse);
    }
}

// Similar to Layer::restore_untyped_slices()
// To improve robustness of detect_surfaces_type() when reslicing (working with typed slices), see GH issue #7442.
// Only resetting layerm->slices if Slice::extra_perimeters is always zero or it will not be used anymore
// after the perimeter generator.
// there is now much more things used by prepare_infill, i'm not confident on this optimisation.
void Layer::restore_untyped_slices_no_extra_perimeters()
{
    restore_untyped_slices();
//    if (layer_needs_raw_backup(this)) {
//        for (LayerRegion *layerm : m_regions)
//        	if (! layerm->region().config().extra_perimeters.value)
//            	layerm->slices.set(layerm->raw_slices, stPosInternal | stDensSparse);
//    } else {
//    	assert(m_regions.size() == 1);
//    	LayerRegion *layerm = m_regions.front();
//    	// This optimization is correct, as extra_perimeters are only reused by prepare_infill() with multi-regions.
//        //if (! layerm->region().config().extra_perimeters.value)
//        	layerm->slices.set(this->lslices, stPosInternal | stDensSparse);
//    }
}

ExPolygons Layer::merged(float offset_scaled) const
{
    assert(offset_scaled >= 0.f);
    // If no offset is set, apply EPSILON offset before union, and revert it afterwards.
    float offset_scaled2 = 0;
    if (offset_scaled == 0.f) {
        offset_scaled  = float(  EPSILON);
        offset_scaled2 = float(- EPSILON);
    }
    Polygons polygons;
	for (LayerRegion *layerm : m_regions) {
		const PrintRegionConfig &config = layerm->region().config();
		// Our users learned to bend Slic3r to produce empty volumes to act as subtracters. Only add the region if it is non-empty.
		if (config.bottom_solid_layers > 0 || config.top_solid_layers > 0 || config.fill_density > 0. || config.perimeters > 0)
			append(polygons, offset(layerm->slices().surfaces, offset_scaled));
	}
    ExPolygons out = union_ex(polygons);
	if (offset_scaled2 != 0.f)
		out = offset_ex(out, offset_scaled2);
    return out;
}

// Here the perimeters are created cummulatively for all layer regions sharing the same parameters influencing the perimeters.
// The perimeter paths and the thin fills (ExtrusionEntityCollection) are assigned to the first compatible layer region.
// The resulting fill surface is split back among the originating regions.
void Layer::make_perimeters()
{
    BOOST_LOG_TRIVIAL(trace) << "Generating perimeters for layer " << this->id();
    
    // keep track of regions whose perimeters we have already generated
    std::vector<unsigned char> done(m_regions.size(), false);
    
    for (LayerRegionPtrs::iterator layerm = m_regions.begin(); layerm != m_regions.end(); ++ layerm) {
      if ((*layerm)->slices().empty()) {
          (*layerm)->perimeters.clear();
          (*layerm)->fills.clear();
          (*layerm)->ironings.clear();
          (*layerm)->thin_fills.clear();
      } else {
        size_t region_id = layerm - m_regions.begin();
        if (done[region_id])
            continue;
        BOOST_LOG_TRIVIAL(trace) << "Generating perimeters for layer " << this->id() << ", region " << region_id;
        done[region_id] = true;
        const PrintRegionConfig &config = (*layerm)->region().config();
        
        // find compatible regions
        LayerRegionPtrs layerms;
        layerms.push_back(*layerm);
        for (LayerRegionPtrs::const_iterator it = layerm + 1; it != m_regions.end(); ++it) {
            LayerRegion* other_layerm = *it;
            const PrintRegionConfig &other_config = other_layerm->region().config();
            if (other_layerm->slices().empty()) continue;
            /// !!! add here the settings you want to be added in the per-object menu.
            /// if you don't do that, objects will share the same region, and the same settings.
            if (config.perimeter_extruder           == other_config.perimeter_extruder
                && config.perimeters                == other_config.perimeters
                && config.external_perimeter_extrusion_width == other_config.external_perimeter_extrusion_width
                && config.external_perimeter_overlap == other_config.external_perimeter_overlap
                && config.external_perimeter_speed == other_config.external_perimeter_speed // it os mandatory? can't this be set at gcode.cpp?
                && config.external_perimeters_first == other_config.external_perimeters_first
                && config.external_perimeters_hole  == other_config.external_perimeters_hole
                && config.external_perimeters_nothole == other_config.external_perimeters_nothole
                && config.external_perimeters_vase == other_config.external_perimeters_vase
                && config.extra_perimeters_odd_layers == other_config.extra_perimeters_odd_layers
                && config.extra_perimeters_overhangs == other_config.extra_perimeters_overhangs
                && config.gap_fill_enabled          == other_config.gap_fill_enabled
                && ((config.gap_fill_speed          == other_config.gap_fill_speed) || !config.gap_fill_enabled)
                && config.gap_fill_last             == other_config.gap_fill_last
                && config.gap_fill_flow_match_perimeter == other_config.gap_fill_flow_match_perimeter
                && config.gap_fill_extension         == other_config.gap_fill_extension
                && config.gap_fill_max_width         == other_config.gap_fill_max_width
                && config.gap_fill_min_area         == other_config.gap_fill_min_area
                && config.gap_fill_min_length         == other_config.gap_fill_min_length
                && config.gap_fill_min_width         == other_config.gap_fill_min_width
                && config.gap_fill_overlap          == other_config.gap_fill_overlap
                && config.infill_dense              == other_config.infill_dense
                && config.infill_dense_algo         == other_config.infill_dense_algo
                && config.no_perimeter_unsupported_algo == other_config.no_perimeter_unsupported_algo
                && (this->id() == 0 || config.only_one_perimeter_first_layer == other_config.only_one_perimeter_first_layer)
                && config.only_one_perimeter_top    == other_config.only_one_perimeter_top
                && config.only_one_perimeter_top_other_algo == other_config.only_one_perimeter_top_other_algo
                && config.overhangs_width_speed     == other_config.overhangs_width_speed
                && config.overhangs_width           == other_config.overhangs_width
                && config.overhangs_reverse         == other_config.overhangs_reverse
                && config.overhangs_reverse_threshold == other_config.overhangs_reverse_threshold
                && config.perimeter_extrusion_width == other_config.perimeter_extrusion_width
                && config.perimeter_loop            == other_config.perimeter_loop
                && config.perimeter_loop_seam       == other_config.perimeter_loop_seam
                && config.perimeter_overlap         == other_config.perimeter_overlap
                && config.perimeter_reverse == other_config.perimeter_reverse
                && config.perimeter_speed           == other_config.perimeter_speed // it os mandatory? can't this be set at gcode.cpp?
                && config.print_extrusion_multiplier == other_config.print_extrusion_multiplier
                && config.small_perimeter_speed     == other_config.small_perimeter_speed
                && config.small_perimeter_min_length == other_config.small_perimeter_min_length
                && config.small_perimeter_max_length == other_config.small_perimeter_max_length
                && config.thin_walls                == other_config.thin_walls
                && config.thin_walls_min_width      == other_config.thin_walls_min_width
                && config.thin_walls_overlap        == other_config.thin_walls_overlap
                && config.thin_perimeters           == other_config.thin_perimeters
                && config.thin_perimeters_all       == other_config.thin_perimeters_all
                && config.thin_walls_speed          == other_config.thin_walls_speed
                && config.infill_overlap            == other_config.infill_overlap
                && config.perimeter_loop            == other_config.perimeter_loop

                && config.fuzzy_skin                == other_config.fuzzy_skin
                && config.fuzzy_skin_thickness      == other_config.fuzzy_skin_thickness
                && config.fuzzy_skin_point_dist     == other_config.fuzzy_skin_point_dist
                ) {
                layerms.push_back(other_layerm);
                done[it - m_regions.begin()] = true;
            }
        }
        
        if (layerms.size() == 1) {  // optimization
            (*layerm)->fill_surfaces.surfaces.clear();
            (*layerm)->make_perimeters((*layerm)->slices(), &(*layerm)->fill_surfaces);
            (*layerm)->fill_expolygons = to_expolygons((*layerm)->fill_surfaces.surfaces);
        } else {
            SurfaceCollection new_slices;
            // Use the region with highest infill rate, as the make_perimeters() function below decides on the gap fill based on the infill existence.
            LayerRegion *layerm_config = layerms.front();
            {
                // group slices (surfaces) according to number of extra perimeters
                std::map<unsigned short, Surfaces> slices;  // extra_perimeters => [ surface, surface... ]
                for (LayerRegion *layerm : layerms) {
                    for (const Surface &surface : layerm->slices().surfaces)
                        slices[surface.extra_perimeters].emplace_back(surface);
                    if (layerm->region().config().fill_density > layerm_config->region().config().fill_density)
                        layerm_config = layerm;
                    //clean list as only the layerm_config will have these ones recomputed
                    layerm->perimeters.clear();
                    layerm->thin_fills.clear();
                    layerm->fill_no_overlap_expolygons.clear();
                }
                // merge the surfaces assigned to each group
                for (std::pair<const unsigned short,Surfaces> &surfaces_with_extra_perimeters : slices)
                    new_slices.append(/*union_ex(surfaces_with_extra_perimeters.second, true) same as -?>*/offset_ex(surfaces_with_extra_perimeters.second, ClipperSafetyOffset), surfaces_with_extra_perimeters.second.front());
            }
            
            // make perimeters
            SurfaceCollection fill_surfaces;
            this->m_object->print()->throw_if_canceled();
            layerm_config->make_perimeters(new_slices, &fill_surfaces);

            // assign fill_surfaces to each LayerRegion
            if (!fill_surfaces.surfaces.empty()) { 
                for (LayerRegionPtrs::iterator l = layerms.begin(); l != layerms.end(); ++l) {
                    // Separate the fill surfaces.
                    ExPolygons slices = to_expolygons((*l)->slices().surfaces);
                    ExPolygons expp = intersection_ex(to_expolygons(fill_surfaces.surfaces), slices);
                    (*l)->fill_expolygons = expp;
                    (*l)->fill_no_overlap_expolygons = (layerm_config)->fill_no_overlap_expolygons;
                    //(*l)->perimeters = (layerm_config)->perimeters;
                    //(*l)->thin_fills = (layerm_config)->thin_fills;
                    (*l)->fill_surfaces.clear();
                    for (Surface &surf: fill_surfaces.surfaces) {
                        ExPolygons exp = intersection_ex(ExPolygons{ surf.expolygon }, slices);
                        (*l)->fill_surfaces.append(std::move(exp), surf);
                    }
                }
            }
        }
      }
    }
    BOOST_LOG_TRIVIAL(trace) << "Generating perimeters for layer " << this->id() << " - Done";
}

void Layer::make_milling_post_process() {
    if (this->object()->print()->config().milling_diameter.empty()) return;

    BOOST_LOG_TRIVIAL(trace) << "Generating milling_post_process for layer " << this->id();

    // keep track of regions whose perimeters we have already generated
    std::vector<unsigned char> done(m_regions.size(), false);

    for (LayerRegionPtrs::iterator layerm = m_regions.begin(); layerm != m_regions.end(); ++layerm) {
        if ((*layerm)->slices().empty()) {
            (*layerm)->milling.clear();
        } else {
            size_t region_id = layerm - m_regions.begin();
            if (done[region_id])
                continue;
            BOOST_LOG_TRIVIAL(trace) << "Generating milling_post_process for layer " << this->id() << ", region " << region_id;
            done[region_id] = true;
            const PrintRegionConfig& config = (*layerm)->region().config();

            // find compatible regions
            LayerRegionPtrs layerms;
            layerms.push_back(*layerm);
            for (LayerRegionPtrs::const_iterator it = layerm + 1; it != m_regions.end(); ++it) {
                LayerRegion* other_layerm = *it;
                const PrintRegionConfig& other_config = other_layerm->region().config();
                if (other_layerm->slices().empty()) continue;
                /// !!! add here the settings you want to be added in the per-object menu.
                /// if you don't do that, objects will share the same region, and the same settings.
                if (config.milling_post_process == other_config.milling_post_process
                    && config.milling_extra_size == other_config.milling_extra_size
                    && (config.milling_after_z == other_config.milling_after_z ||
                        this->bottom_z() > std::min(config.milling_after_z.get_abs_value(this->object()->print()->config().milling_diameter.values[0]),
                            other_config.milling_after_z.get_abs_value(this->object()->print()->config().milling_diameter.values[0])))) {
                    layerms.push_back(other_layerm);
                    done[it - m_regions.begin()] = true;
                }
            }

            if (layerms.size() == 1) {  // optimization
                (*layerm)->make_milling_post_process((*layerm)->slices());
            } else {
                SurfaceCollection new_slices;
                // Use a region if you ned a specific settgin, be sure to choose the good one, curtly there is o need.
                LayerRegion* layerm_config = layerms.front();
                {
                    // group slices (surfaces) according to number of extra perimeters
                    std::map<unsigned short, Surfaces> slices;  // extra_perimeters => [ surface, surface... ]
                    for (LayerRegion* layerm : layerms) {
                        for (const Surface& surface : layerm->slices().surfaces)
                            slices[surface.extra_perimeters].emplace_back(surface);
                        layerm->milling.clear();
                    }
                    // merge the surfaces assigned to each group
                    for (std::pair<const unsigned short, Surfaces>& surfaces_with_extra_perimeters : slices)
                        new_slices.append(union_safety_offset_ex(to_expolygons(surfaces_with_extra_perimeters.second)), surfaces_with_extra_perimeters.second.front());
                }

                // make perimeters
                layerm_config->make_milling_post_process(new_slices);

            }
        }
    }
    BOOST_LOG_TRIVIAL(trace) << "Generating milling_post_process for layer " << this->id() << " - Done";
}

void Layer::export_region_slices_to_svg(const char *path) const
{
    BoundingBox bbox;
    for (const LayerRegion *region : m_regions)
        for (const Surface &surface : region->slices().surfaces)
            bbox.merge(get_extents(surface.expolygon));
    Point legend_size = export_surface_type_legend_to_svg_box_size();
    Point legend_pos(bbox.min(0), bbox.max(1));
    bbox.merge(Point(std::max(bbox.min(0) + legend_size(0), bbox.max(0)), bbox.max(1) + legend_size(1)));

    SVG svg(path, bbox);
    const float transparency = 0.5f;
    for (const LayerRegion *region : m_regions)
        for (const Surface &surface : region->slices().surfaces)
            svg.draw(surface.expolygon, surface_type_to_color_name(surface.surface_type), transparency);
    export_surface_type_legend_to_svg(svg, legend_pos);
    svg.Close(); 
}

// Export to "out/LayerRegion-name-%d.svg" with an increasing index with every export.
void Layer::export_region_slices_to_svg_debug(const char *name) const
{
    static size_t idx = 0;
    this->export_region_slices_to_svg(debug_out_path("Layer-slices-%s-%d.svg", name, idx ++).c_str());
}

// used by Layer::build_up_down_graph()
static void connect_layer_slices(
    Layer                                           &below,
    Layer                                           &above,
    const ClipperLib_Z::PolyTree                    &polytree,
    const std::vector<std::pair<coord_t, coord_t>>  &intersections,
    const coord_t                                    offset_below,
    const coord_t                                    offset_above
#ifndef NDEBUG
    , const coord_t                                  offset_end
#endif // NDEBUG
    )
{
    class Visitor {
    public:
        Visitor(const std::vector<std::pair<coord_t, coord_t>> &intersections, 
            Layer &below, Layer &above, const coord_t offset_below, const coord_t offset_above
#ifndef NDEBUG
            , const coord_t offset_end
#endif // NDEBUG
            ) :
            m_intersections(intersections), m_below(below), m_above(above), m_offset_below(offset_below), m_offset_above(offset_above)
#ifndef NDEBUG
            , m_offset_end(offset_end) 
#endif // NDEBUG
            {}

        void visit(const ClipperLib_Z::PolyNode &polynode)
        {
#ifndef NDEBUG
            auto assert_intersection_valid = [this](int i, int j) {
                assert(i < j);
                assert(i >= m_offset_below);
                assert(i < m_offset_above);
                assert(j >= m_offset_above);
                assert(j < m_offset_end);
                return true;
            };
#endif // NDEBUG
            if (polynode.Contour.size() >= 3) {
                // If there is an intersection point, it should indicate which contours (one from layer below, the other from layer above) intersect.
                // Otherwise the contour is fully inside another contour.
                auto [i, j] = this->find_top_bottom_contour_ids_strict(polynode);
                bool found = false;
                if (i < 0 && j < 0) {
                    // This should not happen. It may only happen if the source contours had just self intersections or intersections with contours at the same layer.
                    // We may safely ignore such cases where the intersection area is meager.
                    double a = ClipperLib_Z::Area(polynode.Contour);
                    if (a < sqr(scaled<double>(0.001))) {
                        // Ignore tiny overlaps. They are not worth resolving.
                    } else {
                        // We should not ignore large cases. Try to resolve the conflict by a majority of references.
                        std::tie(i, j) = this->find_top_bottom_contour_ids_approx(polynode);
                        // At least top or bottom should be resolved.
                        assert(i >= 0 || j >= 0);
                    }
                }
                if (j < 0) {
                    if (i < 0) {
                        // this->find_top_bottom_contour_ids_approx() shoudl have made sure this does not happen.
                        assert(false);
                    } else {
                        assert(i >= m_offset_below && i < m_offset_above);
                        i -= m_offset_below;
                        j = this->find_other_contour_costly(polynode, m_above, j == -2);
                        found = j >= 0;
                    }
                } else if (i < 0) {
                    assert(j >= m_offset_above && j < m_offset_end);
                    j -= m_offset_above;
                    i = this->find_other_contour_costly(polynode, m_below, i == -2);
                    found = i >= 0;
                } else {
                    assert(assert_intersection_valid(i, j));
                    i -= m_offset_below;
                    j -= m_offset_above;
                    assert(i >= 0 && i < m_below.lslices_ex.size());
                    assert(j >= 0 && j < m_above.lslices_ex.size());
                    found = true;
                }
                if (found) {
                    assert(i >= 0 && i < m_below.lslices_ex.size());
                    assert(j >= 0 && j < m_above.lslices_ex.size());
                    // Subtract area of holes from the area of outer contour.
                    double area = ClipperLib_Z::Area(polynode.Contour);
                    for (int icontour = 0; icontour < polynode.ChildCount(); ++ icontour)
                        area -= ClipperLib_Z::Area(polynode.Childs[icontour]->Contour);
                    // Store the links and area into the contours.
                    LayerSlice::Links &links_below = m_below.lslices_ex[i].overlaps_above;
                    LayerSlice::Links &links_above = m_above.lslices_ex[j].overlaps_below;
                    LayerSlice::Link key{ j };
                    auto it_below = std::lower_bound(links_below.begin(), links_below.end(), key, [](auto &l, auto &r){ return l.slice_idx < r.slice_idx; });
                    if (it_below != links_below.end() && it_below->slice_idx == j) {
                        it_below->area += area;
                    } else {
                        auto it_above = std::lower_bound(links_above.begin(), links_above.end(), key, [](auto &l, auto &r){ return l.slice_idx < r.slice_idx; });
                        if (it_above != links_above.end() && it_above->slice_idx == i) {
                            it_above->area += area;
                        } else {
                            // Insert into one of the two vectors.
                            bool take_below = false;
                            if (links_below.size() < LayerSlice::LinksStaticSize)
                                take_below = false;
                            else if (links_above.size() >= LayerSlice::LinksStaticSize) {
                                size_t shift_below = links_below.end() - it_below;
                                size_t shift_above = links_above.end() - it_above;
                                take_below = shift_below < shift_above;
                            }
                            if (take_below)
                                links_below.insert(it_below, { j, float(area) });
                            else
                                links_above.insert(it_above, { i, float(area) });
                        }
                    }
                }
            }
            for (int i = 0; i < polynode.ChildCount(); ++ i)
                for (int j = 0; j < polynode.Childs[i]->ChildCount(); ++ j)
                    this->visit(*polynode.Childs[i]->Childs[j]);
        }

    private:
        // Find the indices of the contour below & above for an expolygon created as an intersection of two expolygons, one below, the other above.
        // Returns -1 if there is no point on the intersection refering bottom resp. top source expolygon.
        // Returns -2 if the intersection refers to multiple source expolygons on bottom resp. top layers.
        std::pair<int32_t, int32_t> find_top_bottom_contour_ids_strict(const ClipperLib_Z::PolyNode &polynode) const
        {
            // If there is an intersection point, it should indicate which contours (one from layer below, the other from layer above) intersect.
            // Otherwise the contour is fully inside another contour.
            int32_t i = -1, j = -1;
            auto process_i = [&i, &j](coord_t k) {
                if (i == -1)
                    i = k;
                else if (i >= 0) {
                    if (i != k) {
                        // Error: Intersection contour contains points of two or more source bottom contours.
                        i = -2;
                        if (j == -2)
                            // break
                            return true;
                    }
                } else
                    assert(i == -2);
                return false;
            };
            auto process_j = [&i, &j](coord_t k) {
                if (j == -1)
                    j = k;
                else if (j >= 0) {
                    if (j != k) {
                        // Error: Intersection contour contains points of two or more source top contours.
                        j = -2;
                        if (i == -2)
                            // break
                            return true;
                    }
                } else
                    assert(j == -2);
                return false;
            };
            for (int icontour = 0; icontour <= polynode.ChildCount(); ++ icontour) {
                const ClipperLib_Z::Path &contour = icontour == 0 ? polynode.Contour : polynode.Childs[icontour - 1]->Contour;
                if (contour.size() >= 3) {
                    for (const ClipperLib_Z::IntPoint &pt : contour)
                        if (coord_t k = pt.z(); k < 0) {
                            const auto &intersection = m_intersections[-k - 1];
                            assert(intersection.first <= intersection.second);
                            if (intersection.first < m_offset_above ? process_i(intersection.first) : process_j(intersection.first))
                                goto end;
                            if (intersection.second < m_offset_above ? process_i(intersection.second) : process_j(intersection.second))
                                goto end;
                        } else if (k < m_offset_above ? process_i(k) : process_j(k))
                            goto end;
                }
            }
        end:
            return { i, j };
        }

        // Find the indices of the contour below & above for an expolygon created as an intersection of two expolygons, one below, the other above.
        // This variant expects that the source expolygon assingment is not unique, it counts the majority.
        // Returns -1 if there is no point on the intersection refering bottom resp. top source expolygon.
        // Returns -2 if the intersection refers to multiple source expolygons on bottom resp. top layers.
        std::pair<int32_t, int32_t> find_top_bottom_contour_ids_approx(const ClipperLib_Z::PolyNode &polynode) const
        {
            // 1) Collect histogram of contour references.
            struct HistoEl {
                int32_t id;
                int32_t count;
            };
            std::vector<HistoEl> histogram;
            {
                auto increment_counter = [&histogram](const int32_t i) {
                    auto it = std::lower_bound(histogram.begin(), histogram.end(), i, [](auto l, auto r){ return l.id < r; });
                    if (it == histogram.end() || it->id != i)
                        histogram.insert(it, HistoEl{ i, int32_t(1) });
                    else
                        ++ it->count;
                };
                for (int icontour = 0; icontour <= polynode.ChildCount(); ++ icontour) {
                    const ClipperLib_Z::Path &contour = icontour == 0 ? polynode.Contour : polynode.Childs[icontour - 1]->Contour;
                    if (contour.size() >= 3) {
                        for (const ClipperLib_Z::IntPoint &pt : contour)
                            if (coord_t k = pt.z(); k < 0) {
                                const auto &intersection = m_intersections[-k - 1];
                                assert(intersection.first <= intersection.second);
                                increment_counter(intersection.first);
                                increment_counter(intersection.second);
                            } else
                                increment_counter(k);
                    }
                }
                assert(! histogram.empty());
            }
            int32_t i = -1;
            int32_t j = -1;
            if (! histogram.empty()) {
                // 2) Split the histogram to bottom / top.
                auto mid          = std::upper_bound(histogram.begin(), histogram.end(), m_offset_above, [](auto l, auto r){ return l < r.id; });
                // 3) Sort the bottom / top parts separately.
                auto bottom_begin = histogram.begin();
                auto bottom_end   = mid;
                auto top_begin    = mid;
                auto top_end      = histogram.end();
                std::sort(bottom_begin, bottom_end, [](auto l, auto r) { return l.count > r.count; });
                std::sort(top_begin,    top_end,    [](auto l, auto r) { return l.count > r.count; });
                double i_quality = 0;
                double j_quality = 0;
                if (bottom_begin != bottom_end) {
                    i = bottom_begin->id;
                    i_quality = std::next(bottom_begin) == bottom_end ? std::numeric_limits<double>::max() : double(bottom_begin->count) / std::next(bottom_begin)->count;
                }
                if (top_begin != top_end) {
                    j = top_begin->id;
                    j_quality = std::next(top_begin) == top_end ? std::numeric_limits<double>::max() : double(top_begin->count) / std::next(top_begin)->count;
                }
                // Expected to be called only if there are duplicate references to be resolved by the histogram.
                assert(i >= 0 || j >= 0);
                assert(i_quality < std::numeric_limits<double>::max() || j_quality < std::numeric_limits<double>::max());
                if (i >= 0 && i_quality < j_quality) {
                    // Force the caller to resolve the bottom references the costly but robust way.
                    assert(j >= 0);
                    // Twice the number of references for the best contour.
                    assert(j_quality >= 2.);
                    i = -2;
                } else if (j >= 0) {
                    // Force the caller to resolve the top reference the costly but robust way.
                    assert(i >= 0);
                    // Twice the number of references for the best contour.
                    assert(i_quality >= 2.);
                    j = -2;
                }

            }
            return { i, j };
        }

        static int32_t find_other_contour_costly(const ClipperLib_Z::PolyNode &polynode, const Layer &other_layer, bool other_has_duplicates)
        {
            if (! other_has_duplicates) {
                // The contour below is likely completely inside another contour above. Look-it up in the island above.
                Point pt(polynode.Contour.front().x(), polynode.Contour.front().y());
                for (int i = int(other_layer.lslices_ex.size()) - 1; i >= 0; -- i)
                    if (other_layer.lslices_ex[i].bbox.contains(pt) && other_layer.lslices[i].contains(pt))
                        return i;
                // The following shall not happen now as the source expolygons are being shrunk a bit before intersecting,
                // thus each point of each intersection polygon should fit completely inside one of the original (unshrunk) expolygons.
                assert(false);
            }
            // The comment below may not be valid anymore, see the comment above. However the code is used in case the polynode contains multiple references 
            // to other_layer expolygons, thus the references are not unique.
            //
            // The check above might sometimes fail when the polygons overlap only on points, which causes the clipper to detect no intersection.
            // The problem happens rarely, mostly on simple polygons (in terms of number of points), but regardless of size!
            // example of failing link on two layers, each with single polygon without holes.
            // layer A = Polygon{(-24931238,-11153865),(-22504249,-8726874),(-22504249,11477151),(-23261469,12235585),(-23752371,12727276),(-25002495,12727276),(-27502745,10227026),(-27502745,-12727274),(-26504645,-12727274)}
            // layer B = Polygon{(-24877897,-11100524),(-22504249,-8726874),(-22504249,11477151),(-23244827,12218916),(-23752371,12727276),(-25002495,12727276),(-27502745,10227026),(-27502745,-12727274),(-26504645,-12727274)}
            // note that first point is not identical, and the check above picks (-24877897,-11100524) as the first contour point (polynode.Contour.front()).
            // that point is sadly slightly outisde of the layer A, so no link is detected, eventhough they are overlaping "completely"
            Polygons contour_poly{ Polygon{ClipperZUtils::from_zpath(polynode.Contour)} };
            BoundingBox contour_aabb{contour_poly.front().points};
            int32_t i_largest = -1;
            double  a_largest = 0;
            for (int i = int(other_layer.lslices_ex.size()) - 1; i >= 0; -- i)
                if (contour_aabb.overlap(other_layer.lslices_ex[i].bbox))
                    // it is potentially slow, but should be executed rarely
                    if (Polygons overlap = intersection(contour_poly, other_layer.lslices[i]); ! overlap.empty()) {
                        if (other_has_duplicates) {
                            // Find the contour with the largest overlap. It is expected that the other overlap will be very small.
                            double a = area(overlap);
                            if (a > a_largest) {
                                a_largest = a;
                                i_largest = i;
                            }
                        } else {
                            // Most likely there is just one contour that overlaps, however it is not guaranteed.
                            i_largest = i;
                            break;
                        }
                    }
            assert(i_largest >= 0);
            return i_largest;
        }

        const std::vector<std::pair<coord_t, coord_t>> &m_intersections;
        Layer                                          &m_below;
        Layer                                          &m_above;
        const coord_t                                   m_offset_below;
        const coord_t                                   m_offset_above;
#ifndef NDEBUG
        const coord_t                                   m_offset_end;
#endif // NDEBUG
    } visitor(intersections, below, above, offset_below, offset_above
#ifndef NDEBUG
        , offset_end
#endif // NDEBUG
    );

    for (int i = 0; i < polytree.ChildCount(); ++ i)
        visitor.visit(*polytree.Childs[i]);

#ifndef NDEBUG
    // Verify that only one directional link is stored: either from bottom slice up or from upper slice down.
    for (int32_t islice = 0; islice < below.lslices_ex.size(); ++ islice) {
        LayerSlice::Links &links1 = below.lslices_ex[islice].overlaps_above;
        for (LayerSlice::Link &link1 : links1) {
            LayerSlice::Links &links2 = above.lslices_ex[link1.slice_idx].overlaps_below;
            assert(! std::binary_search(links2.begin(), links2.end(), link1, [](auto &l, auto &r){ return l.slice_idx < r.slice_idx; }));
        }
    }
    for (int32_t islice = 0; islice < above.lslices_ex.size(); ++ islice) {
        LayerSlice::Links &links1 = above.lslices_ex[islice].overlaps_below;
        for (LayerSlice::Link &link1 : links1) {
            LayerSlice::Links &links2 = below.lslices_ex[link1.slice_idx].overlaps_above;
            assert(! std::binary_search(links2.begin(), links2.end(), link1, [](auto &l, auto &r){ return l.slice_idx < r.slice_idx; }));
        }
    }
#endif // NDEBUG

    // Scatter the links, but don't sort them yet.
    for (int32_t islice = 0; islice < int32_t(below.lslices_ex.size()); ++ islice)
        for (LayerSlice::Link &link : below.lslices_ex[islice].overlaps_above)
            above.lslices_ex[link.slice_idx].overlaps_below.push_back({ islice, link.area });
    for (int32_t islice = 0; islice < int32_t(above.lslices_ex.size()); ++ islice)
        for (LayerSlice::Link &link : above.lslices_ex[islice].overlaps_below)
            below.lslices_ex[link.slice_idx].overlaps_above.push_back({ islice, link.area });
    // Sort the links.
    for (LayerSlice &lslice : below.lslices_ex)
        std::sort(lslice.overlaps_above.begin(), lslice.overlaps_above.end(), [](const LayerSlice::Link &l, const LayerSlice::Link &r){ return l.slice_idx < r.slice_idx; });
    for (LayerSlice &lslice : above.lslices_ex)
        std::sort(lslice.overlaps_below.begin(), lslice.overlaps_below.end(), [](const LayerSlice::Link &l, const LayerSlice::Link &r){ return l.slice_idx < r.slice_idx; });
}

void LayerRegion::remove_nonplanar_slices(SurfaceCollection topNonplanar) {
    Surfaces layerm_slices_surfaces(m_slices.surfaces);

    //save previously detected nonplanar surfaces
    SurfaceCollection polyNonplanar;
    for(Surface s : m_slices.surfaces) {
        if (s.is_nonplanar()) {
            polyNonplanar.surfaces.push_back(s);
        }
    }

    // clear internal surfaces
    m_slices.clear();

    // append internal surfaces again without the found topNonplanar surfaces
    m_slices.append(
        diff_ex(
            union_ex(layerm_slices_surfaces), 
            topNonplanar.surfaces, 
            ApplySafetyOffset::No),
                    stPosInternal
    );
}

void LayerRegion::append_top_nonplanar_slices(SurfaceCollection topNonplanar) {
    m_slices.append(std::move(topNonplanar));
}


void Layer::build_up_down_graph(Layer& below, Layer& above)
{
    coord_t             paths_below_offset = 0;
    ClipperLib_Z::Paths paths_below = ClipperZUtils::expolygons_to_zpaths(below.lslices, paths_below_offset);
    coord_t             paths_above_offset = paths_below_offset + coord_t(below.lslices.size());
    ClipperLib_Z::Paths paths_above = ClipperZUtils::expolygons_to_zpaths(above.lslices, paths_above_offset);
#ifndef NDEBUG
    coord_t             paths_end = paths_above_offset + coord_t(above.lslices.size());
#endif // NDEBUG

    ClipperLib_Z::Clipper  clipper;
    ClipperLib_Z::PolyTree result;
    ClipperZUtils::ClipperZIntersectionVisitor::Intersections intersections;
    ClipperZUtils::ClipperZIntersectionVisitor visitor(intersections);
    clipper.ZFillFunction(visitor.clipper_callback());
    clipper.AddPaths(paths_below, ClipperLib_Z::ptSubject, true);
    clipper.AddPaths(paths_above, ClipperLib_Z::ptClip, true);
    clipper.Execute(ClipperLib_Z::ctIntersection, result, ClipperLib_Z::pftNonZero, ClipperLib_Z::pftNonZero);

    connect_layer_slices(below, above, result, intersections, paths_below_offset, paths_above_offset
#ifndef NDEBUG
        , paths_end
#endif // NDEBUG
        );
}

// used by Layer::build_up_down_graph()
// Shrink source polygons one by one, so that they will be separated if they were touching
// at vertices (non-manifold situation).
// Then convert them to Z-paths with Z coordinate indicating index of the source expolygon.
[[nodiscard]] static ClipperLib_Z::Paths expolygons_to_zpaths_shrunk(const ExPolygons &expolygons, coord_t isrc)
{
    size_t num_paths = 0;
    for (const ExPolygon &expolygon : expolygons)
        num_paths += expolygon.num_contours();

    ClipperLib_Z::Paths out;
    out.reserve(num_paths);

    ClipperLib::Paths           contours;
    ClipperLib::Paths           holes;
    ClipperLib::Clipper         clipper;
    ClipperLib::ClipperOffset   co;
    ClipperLib::Paths           out2;

    // Top / bottom surfaces must overlap more than 2um to be chained into a Z graph.
    // Also a larger offset will likely be more robust on non-manifold input polygons.
    static constexpr const float delta = scaled<float>(0.001);
    co.MiterLimit = scaled<double>(3.);
// Use the default zero edge merging distance. For this kind of safety offset the accuracy of normal direction is not important.
//    co.ShortestEdgeLength = delta * ClipperOffsetShortestEdgeFactor;
//    static constexpr const double accept_area_threshold_ccw = sqr(scaled<double>(0.1 * delta));
    // Such a small hole should not survive the shrinkage, it should grow over 
//    static constexpr const double accept_area_threshold_cw  = sqr(scaled<double>(0.2 * delta));

    for (const ExPolygon &expoly : expolygons) {
        contours.clear();
        co.Clear();
        co.AddPath(expoly.contour.points, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
        co.Execute(contours, - delta);
//        size_t num_prev = out.size();
        if (! contours.empty()) {
            holes.clear();
            for (const Polygon &hole : expoly.holes) {
                co.Clear();
                co.AddPath(hole.points, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
                // Execute reorients the contours so that the outer most contour has a positive area. Thus the output
                // contours will be CCW oriented even though the input paths are CW oriented.
                // Offset is applied after contour reorientation, thus the signum of the offset value is reversed.
                out2.clear();
                co.Execute(out2, delta);
                append(holes, std::move(out2));
            }
            // Subtract holes from the contours.
            if (! holes.empty()) {
                clipper.Clear();
                clipper.AddPaths(contours, ClipperLib::ptSubject, true);
                clipper.AddPaths(holes, ClipperLib::ptClip, true);
                contours.clear();
                clipper.Execute(ClipperLib::ctDifference, contours, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
            }
            for (const auto &contour : contours) {
                bool accept = true;
                // Trying to get rid of offset artifacts, that may be created due to numerical issues in offsetting algorithm
                // or due to self-intersections in the source polygons.
                //FIXME how reliable is it? Is it helpful or harmful? It seems to do more harm than good as it tends to punch holes
                // into existing ExPolygons.
#if 0
                if (contour.size() < 8) {
                    // Only accept contours with area bigger than some threshold.
                    double a = ClipperLib::Area(contour);
                    // Polygon has to be bigger than some threshold to be accepted.
                    // Hole to be accepted has to have an area slightly bigger than the non-hole, so it will not happen due to rounding errors,
                    // that a hole will be accepted without its outer contour.
                    accept = a > 0 ? a > accept_area_threshold_ccw : a < - accept_area_threshold_cw;
                }
#endif
                if (accept) {
                    out.emplace_back();
                    ClipperLib_Z::Path &path = out.back();
                    path.reserve(contour.size());
                    for (const Point &p : contour)
                        path.push_back({ p.x(), p.y(), isrc });
                }
            }
        }
#if 0 // #ifndef NDEBUG
        // Test whether the expolygons in a single layer overlap.
        Polygons test;
        for (size_t i = num_prev; i < out.size(); ++ i)
            test.emplace_back(ClipperZUtils::from_zpath(out[i]));
        Polygons outside = diff(test, to_polygons(expoly));
        if (! outside.empty()) {
            BoundingBox bbox(get_extents(expoly));
            bbox.merge(get_extents(test));
            SVG svg(debug_out_path("expolygons_to_zpaths_shrunk-self-intersections.svg").c_str(), bbox);
            svg.draw(expoly, "blue");
            svg.draw(test, "green");
            svg.draw(outside, "red");
        }
        assert(outside.empty());
#endif // NDEBUG
        ++ isrc;
    }

    return out;
}

void Layer::export_region_fill_surfaces_to_svg(const char *path) const
{
    BoundingBox bbox;
    for (const LayerRegion *region : m_regions)
        for (const Surface &surface : region->slices().surfaces)
            bbox.merge(get_extents(surface.expolygon));
    Point legend_size = export_surface_type_legend_to_svg_box_size();
    Point legend_pos(bbox.min(0), bbox.max(1));
    bbox.merge(Point(std::max(bbox.min(0) + legend_size(0), bbox.max(0)), bbox.max(1) + legend_size(1)));

    SVG svg(path, bbox);
    const float transparency = 0.5f;
    for (const LayerRegion *region : m_regions)
        for (const Surface &surface : region->slices().surfaces)
            svg.draw(surface.expolygon, surface_type_to_color_name(surface.surface_type), transparency);
    export_surface_type_legend_to_svg(svg, legend_pos);
    svg.Close();
}

// Export to "out/LayerRegion-name-%d.svg" with an increasing index with every export.
void Layer::export_region_fill_surfaces_to_svg_debug(const char *name) const
{
    static size_t idx = 0;
    this->export_region_fill_surfaces_to_svg(debug_out_path("Layer-fill_surfaces-%s-%d.svg", name, idx ++).c_str());
}

void SupportLayer::simplify_support_extrusion_path() {
    const PrintConfig& print_config = this->object()->print()->config();
    const bool spiral_mode = print_config.spiral_vase;
    const bool enable_arc_fitting = print_config.arc_fitting && !spiral_mode;
    coordf_t scaled_resolution = scale_d(print_config.resolution.value);
    if (scaled_resolution == 0) scaled_resolution = enable_arc_fitting ? SCALED_EPSILON * 2 : SCALED_EPSILON;

    SimplifyVisitor visitor{ scaled_resolution , enable_arc_fitting, &print_config.arc_fitting_tolerance };
    this->support_fills.visit(visitor);
}

BoundingBox get_extents(const LayerRegion &layer_region)
{
    BoundingBox bbox;
    const Surfaces& srfs = layer_region.slices().surfaces;
    if (!srfs.empty()) {
        bbox = get_extents(srfs.front());
        for (auto it = srfs.cbegin() + 1; it != srfs.cend(); ++it)
            bbox.merge(get_extents(*it));
    }
    return bbox;
}

BoundingBox get_extents(const LayerRegionPtrs &layer_regions)
{
    BoundingBox bbox;
    if (!layer_regions.empty()) {
        bbox = get_extents(*layer_regions.front());
        for (auto it = layer_regions.begin() + 1; it != layer_regions.end(); ++it)
            bbox.merge(get_extents(**it));
    }
    return bbox;
}

}
