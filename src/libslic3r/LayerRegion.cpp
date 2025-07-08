///|/ Copyright (c) Prusa Research 2016 - 2023 Vojtěch Bubník @bubnikv, Pavel Mikuš @Godrak, Lukáš Matěna @lukasmatena, Lukáš Hejl @hejllukas
///|/ Copyright (c) Slic3r 2014 - 2016 Alessandro Ranellucci @alranel
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "ExPolygon.hpp"
#include "Flow.hpp"
#include "Layer.hpp"
#include "BridgeDetector.hpp"
#include "ClipperUtils.hpp"
#include "Geometry.hpp"
#include "Milling/MillingPostProcess.hpp"
#include "PerimeterGenerator.hpp"
#include "Print.hpp"
#include "Surface.hpp"
#include "BoundingBox.hpp"
#include "SVG.hpp"
#include "Algorithm/RegionExpansion.hpp"

#include <algorithm>
#include <string>
#include <map>

#include <boost/log/trivial.hpp>

namespace Slic3r {

void LayerRegion::clear() {
    this->m_perimeters.clear();
    this->m_millings.clear();
    this->m_unsupported_bridge_edges.clear();
    this->m_fill_surfaces.clear();
    this->m_fills.clear();
    this->m_ironings.clear();
    this->m_thin_fills.clear();
    this->m_fill_expolygons.clear();
    this->m_fill_expolygons_bboxes.clear();
    this->m_fill_expolygons_composite.clear();
    this->m_fill_expolygons_composite_bboxes.clear();
    this->m_fill_no_overlap_expolygons.clear();
}

Flow LayerRegion::flow(FlowRole role) const
{
    return this->flow(role, m_layer->height);
}

Flow LayerRegion::flow(FlowRole role, double layer_height) const
{
    return m_region->flow(*m_layer->object(), role, layer_height, m_layer->id());
}

// Average diameter of nozzles participating on extruding this region.
coordf_t LayerRegion::bridging_height_avg() const
{
    const PrintRegionConfig& region_config = this->region().config();
    if (region_config.bridge_type == BridgeType::btFromNozzle) {
        const PrintConfig& print_config = this->layer()->object()->print()->config();
        return region().nozzle_dmr_avg(print_config) * sqrt(region_config.bridge_flow_ratio.get_abs_value(1));
    } else if (region_config.bridge_type == BridgeType::btFromHeight) {
        return this->layer()->height;
    } else if (region_config.bridge_type == BridgeType::btFromFlow) {
        return this->bridging_flow(FlowRole::frInfill).height();
    }
    throw Slic3r::InvalidArgument("Unknown BridgeType");
}

Flow LayerRegion::bridging_flow(FlowRole role, BridgeType force_type) const
{
    const PrintRegion       &region         = this->region();
    const PrintRegionConfig &region_config  = region.config();
    const PrintObject       &print_object   = *this->layer()->object();
        // The old Slic3r way (different from all other slicers): Use rounded extrusions.
        // Get the configured nozzle_diameter for the extruder associated to the flow role requested.
        // Here this->extruder(role) - 1 may underflow to MAX_INT, but then the get_at() will follback to zero'th element, so everything is all right.
    float nozzle_diameter = float(print_object.print()->config().nozzle_diameter.get_at(region.extruder(role, *this->layer()->object()) - 1));
    BridgeType bridge_type = force_type == BridgeType::btNone ? region_config.bridge_type : force_type;

    float bridge_width;
    float bridge_height;

    if (bridge_type == BridgeType::btFromFlow) {
        Flow reference_flow = flow(role);
        float diameter = sqrt(4 * reference_flow.mm3_per_mm() / PI);
        bridge_width = diameter;
        bridge_height = diameter;
    }
    else if (bridge_type == BridgeType::btFromHeight) {
        bridge_height = m_layer->height;
        bridge_width = float(sqrt(region_config.bridge_flow_ratio.get_abs_value(1.)) * nozzle_diameter);
    }
    else {
        bridge_height = nozzle_diameter;
        bridge_width  = float(sqrt(region_config.bridge_flow_ratio.get_abs_value(1.)) * nozzle_diameter);
    }

    return Flow::bridging_flow( bridge_width, bridge_height, nozzle_diameter);
    
}

// Fill in layerm->m_fill_surfaces by trimming the layerm->slices by layerm->fill_expolygons.
void LayerRegion::slices_to_fill_surfaces_clipped()
{
    // Collect polygons per surface type.
   std::array<std::vector<const Surface*>, size_t(stCount)> by_surface;
    for (const Surface &surface : this->slices())
        by_surface[size_t(surface.surface_type)].emplace_back(&surface);
    // Trim surfaces by the fill_boundaries.
    m_fill_surfaces.surfaces.clear();
   for (size_t surface_type = 0; surface_type < size_t(posCount); ++ surface_type) {
        const std::vector<const Surface*> &this_surfaces = by_surface[surface_type];
        if (! this_surfaces.empty())
            m_fill_surfaces.append(intersection_ex(this_surfaces, this->fill_expolygons()), SurfaceType(surface_type));
    }
}

// Produce perimeter extrusions, gap fill extrusions and fill polygons for input slices.
void LayerRegion::make_perimeters(
    // Input slices for which the perimeters, gap fills and fill expolygons are to be generated.
    const SurfaceCollection                                &slices,
    // Ranges of perimeter extrusions and gap fill extrusions per suface, referencing
    // newly created extrusions stored at this LayerRegion.
    std::vector<std::pair<ExtrusionRange, ExtrusionRange>> &perimeter_and_gapfill_ranges,
    // All fill areas produced for all input slices above.
    ExPolygons                                             &fill_expolygons,
    // Ranges of fill areas above per input slice.
    std::vector<ExPolygonRange>                            &fill_expolygons_ranges)
{
    m_perimeters.clear();
    m_thin_fills.clear();

    perimeter_and_gapfill_ranges.reserve(perimeter_and_gapfill_ranges.size() + slices.size());
    // There may be more expolygons produced per slice, thus this reserve is conservative.
    fill_expolygons.reserve(fill_expolygons.size() + slices.size());
    fill_expolygons_ranges.reserve(fill_expolygons_ranges.size() + slices.size());

    const PrintConfig       &print_config  = this->layer()->object()->print()->config();
    const PrintRegionConfig &region_config = this->region().config();
    // This needs to be in sync with PrintObject::_slice() slicing_mode_normal_below_layer!
    bool spiral_vase = print_config.spiral_vase &&
        //FIXME account for raft layers.
        (this->layer()->id() >= size_t(region_config.bottom_solid_layers.value) &&
         this->layer()->print_z >= region_config.bottom_solid_min_thickness - EPSILON);

    //this is a factory, the content will be copied into the PerimeterGenerator
    PerimeterGenerator::Parameters params(
        this->layer(),
        this->flow(frPerimeter),
        this->flow(frExternalPerimeter),
        this->bridging_flow(frPerimeter),
        this->flow(frSolidInfill),
        region_config,
        this->layer()->object()->config(),
        print_config,
        spiral_vase,
        (region_config.perimeter_generator.value == PerimeterGeneratorType::Arachne) //use_arachne
    );
    

    // perimeter bonding set.
    if (params.perimeter_flow.spacing_ratio() == 1
        && params.ext_perimeter_flow.spacing_ratio() == 1
        && params.config.external_perimeters_first
        && params.object_config.perimeter_bonding.value > 0) {
        params.infill_gap = (1 - params.object_config.perimeter_bonding.get_abs_value(1)) * params.get_ext_perimeter_spacing();
        params.ext_perimeter_spacing2 -= params.infill_gap;
    }

    const ExPolygons *lower_slices = this->layer()->lower_layer ? &this->layer()->lower_layer->lslices() : nullptr;
    const ExPolygons *upper_slices = this->layer()->upper_layer ? &this->layer()->upper_layer->lslices() : nullptr;
    
    for (const Surface &surface : slices) {
        size_t perimeters_begin = m_perimeters.size();
        size_t gap_fills_begin = m_thin_fills.size();
        size_t fill_expolygons_begin = fill_expolygons.size();

        PerimeterGenerator::PerimeterGenerator g{params};
        g.throw_if_canceled = [this]() { this->layer()->object()->print()->throw_if_canceled(); };
        g.process(
            // input:
            surface, lower_slices, slices, upper_slices,
            // output:
                // Loops with the external thin walls
            &m_perimeters,
                // Gaps without the thin walls
            &m_thin_fills,
                // Infills without the gap fills
            fill_expolygons,
                // mask for "no overlap" area
            m_fill_no_overlap_expolygons
        );

        for(auto *peri : this->m_perimeters.entities()) assert(!peri->empty());

        perimeter_and_gapfill_ranges.emplace_back(
            ExtrusionRange{ uint32_t(perimeters_begin), uint32_t(m_perimeters.size()) }, 
            ExtrusionRange{ uint32_t(gap_fills_begin),  uint32_t(m_thin_fills.size()) });
        fill_expolygons_ranges.emplace_back(ExtrusionRange{ uint32_t(fill_expolygons_begin), uint32_t(fill_expolygons.size()) });
    }
}

void LayerRegion::make_milling_post_process(const SurfaceCollection& slices) {
    MillingPostProcess mill(// input:
        &slices,
        (this->layer()->lower_layer != nullptr) ? &this->layer()->lower_layer->lslices() : nullptr,
        this->region().config(),
        this->layer()->object()->config(),
        this->layer()->object()->print()->config()
    );
    m_millings = mill.process(this->layer());
}


// Extract surfaces of given type from surfaces, extract fill (layer) thickness of one of the surfaces.
static ExPolygons fill_surfaces_extract_expolygons(Surfaces &surfaces, std::initializer_list<SurfaceType> surface_types, double &thickness)
{
    size_t cnt = 0;
    for (const Surface &surface : surfaces)
        if (std::find(surface_types.begin(), surface_types.end(), surface.surface_type) != surface_types.end()) {
            ++cnt;
            thickness = surface.thickness;
        }
    if (cnt == 0)
        return {};

    ExPolygons out;
    out.reserve(cnt);
    for (Surface &surface : surfaces)
        if (std::find(surface_types.begin(), surface_types.end(), surface.surface_type) != surface_types.end())
            out.emplace_back(std::move(surface.expolygon));
    return out;
}

// Cache for detecting bridge orientation and merging regions with overlapping expansions.
struct Bridge {
    ExPolygon expolygon;
    uint32_t group_id;
    std::vector<Algorithm::RegionExpansionEx>::const_iterator bridge_expansion_begin;
};

// Group the bridge surfaces by overlaps.
uint32_t group_id(std::vector<Bridge> &bridges, uint32_t src_id) {
    uint32_t group_id = bridges[src_id].group_id;
    while (group_id != src_id) {
        src_id = group_id;
        group_id = bridges[src_id].group_id;
    }
    bridges[src_id].group_id = group_id;
    return group_id;
};

std::vector<Bridge> get_grouped_bridges(
    ExPolygons&& bridge_expolygons,
    const std::vector<Algorithm::RegionExpansionEx>& bridge_expansions
) {
    using namespace Algorithm;

    std::vector<Bridge> result;
    {
        result.reserve(bridge_expansions.size());
        uint32_t group_id = 0;
        using std::move_iterator;
        for (ExPolygon& expolygon : bridge_expolygons)
            result.push_back({ std::move(expolygon), group_id ++, bridge_expansions.end() });
    }


    // Detect overlaps of bridge anchors inside their respective shell regions.
    // bridge_expansions are sorted by boundary id and source id.
    for (auto expansion_iterator = bridge_expansions.begin(); expansion_iterator != bridge_expansions.end();) {
        auto boundary_region_begin = expansion_iterator;
        auto boundary_region_end = std::find_if(
            next(expansion_iterator),
            bridge_expansions.end(),
            [&](const RegionExpansionEx& expansion){
                return expansion.boundary_id != expansion_iterator->boundary_id;
            }
        );

        // Cache of bboxes per expansion boundary.
        std::vector<BoundingBox> bounding_boxes;
        bounding_boxes.reserve(std::distance(boundary_region_begin, boundary_region_end));
        std::transform(
            boundary_region_begin,
            boundary_region_end,
            std::back_inserter(bounding_boxes),
            [](const RegionExpansionEx& expansion){
                return get_extents(expansion.expolygon.contour);
            }
        );

        // For each bridge anchor of the current source:
        for (;expansion_iterator != boundary_region_end; ++expansion_iterator) {
            auto candidate_iterator = std::next(expansion_iterator);
            for (;candidate_iterator != boundary_region_end; ++candidate_iterator) {
                const BoundingBox& current_bounding_box{
                    bounding_boxes[expansion_iterator - boundary_region_begin]
                };
                const BoundingBox& candidate_bounding_box{
                    bounding_boxes[candidate_iterator - boundary_region_begin]
                };
                if (
                    expansion_iterator->src_id != candidate_iterator->src_id
                    && current_bounding_box.overlap(candidate_bounding_box)
                    // One may ignore holes, they are irrelevant for intersection test.
                    && !intersection(expansion_iterator->expolygon.contour, candidate_iterator->expolygon.contour).empty()
                ) {
                    // The two bridge regions intersect. Give them the same (lower) group id.
                    uint32_t id  = group_id(result, expansion_iterator->src_id);
                    uint32_t id2 = group_id(result, candidate_iterator->src_id);
                    if (id < id2)
                        result[id2].group_id = id;
                    else
                        result[id].group_id = id2;
                }
            }
        }
    }
    return result;
}

Surfaces merge_bridges(
    std::vector<Bridge>& bridges,
    const std::vector<Algorithm::RegionExpansionEx>& bridge_expansions,
    const float closing_radius
) {
    for (auto it = bridge_expansions.begin(); it != bridge_expansions.end(); ) {
        bridges[it->src_id].bridge_expansion_begin = it;
        uint32_t src_id = it->src_id;
        for (++ it; it != bridge_expansions.end() && it->src_id == src_id; ++ it) ;
    }

    Surfaces result;
    for (uint32_t bridge_id = 0; bridge_id < uint32_t(bridges.size()); ++ bridge_id) {
        if (group_id(bridges, bridge_id) == bridge_id) {
            // Head of the group.
            Polygons bridge_group;
            Polygons expansions;
            for (uint32_t bridge_id2 = bridge_id; bridge_id2 < uint32_t(bridges.size()); ++ bridge_id2) {
                if (group_id(bridges, bridge_id2) == bridge_id) {
                    append(bridge_group, to_polygons(std::move(bridges[bridge_id2].expolygon)));
                    auto it_bridge_expansion = bridges[bridge_id2].bridge_expansion_begin;
                    assert(it_bridge_expansion == bridge_expansions.end() || it_bridge_expansion->src_id == bridge_id2);
                    for (; it_bridge_expansion != bridge_expansions.end() && it_bridge_expansion->src_id == bridge_id2; ++ it_bridge_expansion)
                        append(expansions, to_polygons(it_bridge_expansion->expolygon));
                }
            }
            append(bridge_group, expansions);

            //NOTE: The current regularization of the shells can create small unasigned regions in the object (E.G. benchy)
            // without the following closing operation, those regions will stay unfilled and cause small holes in the expanded surface.
            // look for narrow_ensure_vertical_wall_thickness_region_radius filter.
            ExPolygons merged_bridges = closing_ex(bridge_group, closing_radius);
            // without safety offset, artifacts are generated (GH #2494)
            // union_safety_offset_ex(acc)

            for (ExPolygon &bridge_expolygon : merged_bridges) {
                const Lines lines{to_lines(diff_pl(to_polylines(bridge_expolygon), expand(expansions, float(SCALED_EPSILON))))};
                auto [bridging_dir, unsupported_dist] = detect_bridging_direction(lines, to_polygons(bridge_expolygon));
                Surface surface{ stPosBottom | stModBridge, std::move(bridge_expolygon) };
                surface.bridge_angle = M_PI + std::atan2(bridging_dir.y(), bridging_dir.x());
                result.push_back(std::move(surface));
            }
        }
    }
    return result;
}

struct ExpansionResult {
    Algorithm::WaveSeeds anchors;
    std::vector<Algorithm::RegionExpansionEx> expansions;
};

ExpansionResult expand_expolygons(
    const ExPolygons& expolygons,
    std::vector<ExpansionZone>& expansion_zones
) {
    using namespace Algorithm;
    WaveSeeds bridge_anchors;
    std::vector<RegionExpansionEx> bridge_expansions;

    unsigned processed_bridges_count = 0;
    for (ExpansionZone& expansion_zone : expansion_zones) {
        WaveSeeds seeds{wave_seeds(
            expolygons,
            expansion_zone.expolygons,
            expansion_zone.parameters.tiny_expansion,
            true
        )};
        std::vector<RegionExpansionEx> expansions{propagate_waves_ex(
            seeds,
            expansion_zone.expolygons,
            expansion_zone.parameters
        )};

        for (WaveSeed &seed : seeds)
            seed.boundary += processed_bridges_count;
        for (RegionExpansionEx &expansion : expansions)
            expansion.boundary_id += processed_bridges_count;

        expansion_zone.expanded_into = ! expansions.empty();

        append(bridge_anchors, std::move(seeds));
        append(bridge_expansions, std::move(expansions));

        processed_bridges_count += expansion_zone.expolygons.size();
    }
    return {bridge_anchors, bridge_expansions};
}

// Extract bridging surfaces from "surfaces", expand them into "shells" using expansion_params,
// detect bridges.
// Trim "shells" by the expanded bridges.
// only used by the new process_external_surfaces
Surfaces expand_bridges_detect_orientations(
    Surfaces &surfaces,
    std::vector<ExpansionZone>& expansion_zones,
    const float closing_radius
)
{
    using namespace Slic3r::Algorithm;

    double thickness;
    ExPolygons bridge_expolygons = fill_surfaces_extract_expolygons(surfaces, {stPosBottom | stModBridge}, thickness);
    if (bridge_expolygons.empty())
        return {};

    // Calculate bridge anchors and their expansions in their respective shell region.
    ExpansionResult expansion_result{expand_expolygons(
        bridge_expolygons,
        expansion_zones
    )};

    std::vector<Bridge> bridges{get_grouped_bridges(
        std::move(bridge_expolygons),
        expansion_result.expansions
    )};
    bridge_expolygons.clear();

    std::sort(expansion_result.anchors.begin(), expansion_result.anchors.end(), Algorithm::lower_by_src_and_boundary);

    // Merge the groups with the same group id, produce surfaces by merging source overhangs with their newly expanded anchors.
    std::sort(expansion_result.expansions.begin(), expansion_result.expansions.end(), [](auto &l, auto &r) {
        return l.src_id < r.src_id || (l.src_id == r.src_id && l.boundary_id < r.boundary_id);
    });
    Surfaces out{merge_bridges(bridges, expansion_result.expansions, closing_radius)};

    // Clip by the expanded bridges.
    for (ExpansionZone& expansion_zone : expansion_zones)
        if (expansion_zone.expanded_into)
            expansion_zone.expolygons = diff_ex(expansion_zone.expolygons, out);
    return out;
}

// Extract bridging surfaces from "surfaces", expand them into "shells" using expansion_params.
// Trim "shells" by the expanded bridges.
Surfaces expand_merge_surfaces(
    Surfaces &surfaces,
    SurfaceType surface_type,
    std::vector<ExpansionZone>& expansion_zones,
    const float closing_radius,
    const double bridge_angle
)
{
    using namespace Slic3r::Algorithm;

    double thickness;
    ExPolygons src = fill_surfaces_extract_expolygons(surfaces, {surface_type}, thickness);
    if (src.empty())
        return {};

    unsigned processed_expolygons_count = 0;
    std::vector<RegionExpansion> expansions;
    for (ExpansionZone& expansion_zone : expansion_zones) {
        std::vector<RegionExpansion> zone_expansions = propagate_waves(src, expansion_zone.expolygons, expansion_zone.parameters);
        expansion_zone.expanded_into = !zone_expansions.empty();

        for (RegionExpansion &expansion : zone_expansions)
            expansion.boundary_id += processed_expolygons_count;

        processed_expolygons_count += expansion_zone.expolygons.size();
        append(expansions, std::move(zone_expansions));
    }

    std::vector<ExPolygon> expanded = merge_expansions_into_expolygons(std::move(src), std::move(expansions));
    //NOTE: The current regularization of the shells can create small unasigned regions in the object (E.G. benchy)
    // without the following closing operation, those regions will stay unfilled and cause small holes in the expanded surface.
    // look for narrow_ensure_vertical_wall_thickness_region_radius filter.
    expanded = closing_ex(expanded, closing_radius);
    // Trim the zones by the expanded expolygons.
    for (ExpansionZone& expansion_zone : expansion_zones)
        if (expansion_zone.expanded_into)
            expansion_zone.expolygons = diff_ex(expansion_zone.expolygons, expanded);

    Surface templ{ surface_type, {} };
    templ.bridge_angle = bridge_angle;
    Surfaces out;
    out.reserve(expanded.size());
    for (auto &expoly : expanded)
        out.emplace_back(templ, std::move(expoly));
    return out;
}

//#define EXTERNAL_SURFACES_OFFSET_PARAMETERS ClipperLib::jtMiter, 3.
//#define EXTERNAL_SURFACES_OFFSET_PARAMETERS ClipperLib::jtMiter, 1.5
#define EXTERNAL_SURFACES_OFFSET_PARAMETERS ClipperLib::jtSquare, 0.

size_t get_island_idx(const Polygon &contour,
                      const std::vector<BoundingBox> &bboxes,
                      const ExPolygons &fill_boundaries) {
    assert(bboxes.size() == fill_boundaries.size());
    std::vector<size_t> candidates;
    for (size_t idx = 0; idx < bboxes.size(); ++idx) {
        if (bboxes[idx].contains(contour.front()) && bboxes[idx].contains(contour.points[contour.size() / 2])) {
            candidates.push_back(idx);
        }
    }
    assert(!candidates.empty());
    if (candidates.size() > 1) {
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (!bboxes[candidates[i]].contains(contour.points)) {
                candidates.erase(candidates.begin() + i);
                --i;
            }
        }
    }
    assert(!candidates.empty());
    // note: fill_boundaries don't overlap, you only need to test one point.
    if (candidates.size() > 1) {
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (!fill_boundaries[candidates[i]].contains(contour.front())) {
                candidates.erase(candidates.begin() + i);
                --i;
            }
        }
    }
    if (candidates.size() < 0) {
        //failed becasue of some epsilon, try with another point
        for (size_t idx = 0; idx < bboxes.size(); ++idx) {
            if (bboxes[idx].contains(contour.points[1])) {
                candidates.push_back(idx);
            }
        }
        if (candidates.size() > 1) {
            for (size_t i = 0; i < candidates.size(); ++i) {
                if (!fill_boundaries[candidates[i]].contains(contour.points[1])) {
                    candidates.erase(candidates.begin() + i);
                    --i;
                }
            }
        }
    }
    if (candidates.size() < 0) {
        //failed because of some margins, try with shrunk polygon
        const Polygons contours_shrunk = offset(contour, -scale_t(0.05));
        if (!contours_shrunk.empty()) {
            const Polygon &contour_shrunk = contours_shrunk.front();
            for (size_t idx = 0; idx < bboxes.size(); ++idx) {
                if (bboxes[idx].contains(contour_shrunk.front())) {
                    candidates.push_back(idx);
                }
            }
            if (candidates.size() > 1) {
                for (size_t i = 0; i < candidates.size(); ++i) {
                    if (!fill_boundaries[candidates[i]].contains(contour_shrunk.front())) {
                        candidates.erase(candidates.begin() + i);
                        --i;
                    }
                }
            }
        }
    }
    assert(candidates.size() == 1);
    return candidates.size() == 1 ? candidates.front() : -1;
}

 LayerRegion::process_external_surfaces(const Layer *lower_layer, const Polygons *lower_layer_covered) {
    using namespace Slic3r::Algorithm;

#ifdef SLIC3R_DEBUG_SLICE_PROCESSING
    export_region_fill_surfaces_to_svg_debug("4_process_external_surfaces-initial");
#endif /* SLIC3R_DEBUG_SLICE_PROCESSING */

    // Width of the perimeters.
    float shell_width = 0;
    float expansion_min = 0;
    if (int num_perimeters = this->region().config().perimeters; num_perimeters > 0) {
        Flow external_perimeter_flow = this->flow(frExternalPerimeter);
        Flow perimeter_flow          = this->flow(frPerimeter);
        shell_width  = 0.5f * external_perimeter_flow.scaled_width() + external_perimeter_flow.scaled_spacing();
        shell_width += perimeter_flow.scaled_spacing() * (num_perimeters - 1);
        expansion_min = perimeter_flow.scaled_spacing();
    } else {
        // TODO: Maybe there is better solution when printing with zero perimeters, but this works reasonably well, given the situation
        shell_width   = float(SCALED_EPSILON);
        expansion_min = float(SCALED_EPSILON);;
    }

    // Scaled expansions of the respective external surfaces.
    float                           expansion_top           = shell_width * sqrt(2.);
    float                           expansion_bottom        = expansion_top;
    float                           expansion_bottom_bridge = expansion_top;
    // Expand by waves of expansion_step size (expansion_step is scaled), but with no more steps than max_nr_expansion_steps.
    static constexpr const float    expansion_step          = scaled<float>(0.1);
    // Don't take more than max_nr_steps for small expansion_step.
    static constexpr const size_t   max_nr_expansion_steps  = 5;
    // Radius (with added epsilon) to absorb empty regions emering from regularization of ensuring, viz  const float narrow_ensure_vertical_wall_thickness_region_radius = 0.5f * 0.65f * min_perimeter_infill_spacing;
    const float closing_radius = 0.55f * 0.65f * 1.05f * this->flow(frSolidInfill).scaled_spacing();

    // Expand the top / bottom / bridge surfaces into the shell thickness solid infills.
    double     layer_thickness;
    ExPolygons shells = union_ex(fill_surfaces_extract_expolygons(m_fill_surfaces.surfaces, { stInternalSolid }, layer_thickness));
    ExPolygons sparse = union_ex(fill_surfaces_extract_expolygons(m_fill_surfaces.surfaces, { stInternal }, layer_thickness));
    ExPolygons top_expolygons = union_ex(fill_surfaces_extract_expolygons(m_fill_surfaces.surfaces, { stTop }, layer_thickness));
    const auto expansion_params_into_sparse_infill = RegionExpansionParameters::build(expansion_min, expansion_step, max_nr_expansion_steps);
    const auto expansion_params_into_solid_infill  = RegionExpansionParameters::build(expansion_bottom_bridge, expansion_step, max_nr_expansion_steps);

    std::vector<ExpansionZone> expansion_zones{
        ExpansionZone{std::move(shells), expansion_params_into_solid_infill},
        ExpansionZone{std::move(sparse), expansion_params_into_sparse_infill},
        ExpansionZone{std::move(top_expolygons), expansion_params_into_solid_infill},
    };

    SurfaceCollection bridges;
    {
        BOOST_LOG_TRIVIAL(trace) << "Processing external surface, detecting bridges. layer" << this->layer()->print_z;
        const double custom_angle = this->region().config().bridge_angle.value;
        bridges.surfaces = custom_angle > 0 ?
            expand_merge_surfaces(m_fill_surfaces.surfaces, stBottomBridge, expansion_zones, closing_radius, Geometry::deg2rad(custom_angle)) :
            expand_bridges_detect_orientations(m_fill_surfaces.surfaces, expansion_zones, closing_radius);
        BOOST_LOG_TRIVIAL(trace) << "Processing external surface, detecting bridges - done";
#if 0
        {
            static int iRun = 0;
            bridges.export_to_svg(debug_out_path("bridges-after-grouping-%d.svg", iRun++), true);
        }
#endif
    }

    m_fill_surfaces.remove_types({ stTop });
    {
        Surface top_templ(stTop, {});
        top_templ.thickness = layer_thickness;
        m_fill_surfaces.append(std::move(expansion_zones.back().expolygons), top_templ);
    }

    expansion_zones.pop_back();

    expansion_zones.at(0).parameters = RegionExpansionParameters::build(expansion_bottom, expansion_step, max_nr_expansion_steps);
    Surfaces bottoms = expand_merge_surfaces(m_fill_surfaces.surfaces, stBottom, expansion_zones, closing_radius);

    expansion_zones.at(0).parameters = RegionExpansionParameters::build(expansion_top, expansion_step, max_nr_expansion_steps);
    Surfaces tops = expand_merge_surfaces(m_fill_surfaces.surfaces, stTop, expansion_zones, closing_radius);

//    m_fill_surfaces.remove_types({ stBottomBridge, stBottom, stTop, stInternal, stInternalSolid });
    m_fill_surfaces.clear();
    unsigned zones_expolygons_count = 0;
    for (const ExpansionZone& zone : expansion_zones)
        zones_expolygons_count += zone.expolygons.size();
    reserve_more(m_fill_surfaces.surfaces, zones_expolygons_count + bridges.size() + bottoms.size() + tops.size());
    {
        Surface solid_templ(stInternalSolid, {});
        solid_templ.thickness = layer_thickness;
        m_fill_surfaces.append(std::move(expansion_zones[0].expolygons), solid_templ);
    }
    {
        Surface sparse_templ(stInternal, {});
        sparse_templ.thickness = layer_thickness;
        m_fill_surfaces.append(std::move(expansion_zones[1].expolygons), sparse_templ);
    }
    m_fill_surfaces.append(std::move(bridges.surfaces));
    m_fill_surfaces.append(std::move(bottoms));
    m_fill_surfaces.append(std::move(tops));

#ifdef SLIC3R_DEBUG_SLICE_PROCESSING
    export_region_fill_surfaces_to_svg_debug("4_process_external_surfaces-final");
#endif /* SLIC3R_DEBUG_SLICE_PROCESSING */
}

void LayerRegion::prepare_fill_surfaces()
{
#ifdef SLIC3R_DEBUG_SLICE_PROCESSING
    export_region_slices_to_svg_debug("2_prepare_fill_surfaces-initial");
    export_region_fill_surfaces_to_svg_debug("2_prepare_fill_surfaces-initial");
#endif /* SLIC3R_DEBUG_SLICE_PROCESSING */ 

    /*  Note: in order to make the psPrepareInfill step idempotent, we should never
        alter fill_surfaces boundaries on which our idempotency relies since that's
        the only meaningful information returned by psPerimeters. */
    
    bool spiral_vase = this->layer()->object()->print()->config().spiral_vase;

    // if no solid layers are requested, turn top/bottom surfaces to internal
    // For Lightning infill, infill_only_where_needed is ignored because both
    // do a similar thing, and their combination doesn't make much sense.
    if (! spiral_vase && this->region().config().top_solid_layers == 0) {
        for (Surface &surface : m_fill_surfaces)
            if (surface.is_top())
                surface.surface_type = /*this->layer()->object()->config().infill_only_where_needed && this->region().config().fill_pattern != ipLightning ? stInternalVoid :*/ stInternal;
    }
    if (this->region().config().bottom_solid_layers == 0) {
        for (Surface &surface : m_fill_surfaces)
            if (surface.is_bottom()) // (surface.surface_type == stBottom)
                surface.surface_type = stInternal;
    }

    // turn too small internal regions into solid regions according to the user setting
    if (! spiral_vase && this->region().config().fill_density.value > 0) {
        // scaling an area requires two calls!
        double min_area = scale_(scale_(this->region().config().solid_infill_below_area.value));
        for (Surface &surface : m_fill_surfaces)
            if (surface.surface_type == stInternal && surface.area() <= min_area)
                surface.surface_type = stInternalSolid;
    }

#ifdef SLIC3R_DEBUG_SLICE_PROCESSING
    export_region_slices_to_svg_debug("2_prepare_fill_surfaces-final");
    export_region_fill_surfaces_to_svg_debug("2_prepare_fill_surfaces-final");
#endif /* SLIC3R_DEBUG_SLICE_PROCESSING */
}



double LayerRegion::infill_area_threshold() const
{
    double ss = this->flow(frSolidInfill).scaled_spacing();
    return ss*ss;
}

void LayerRegion::trim_surfaces(const Polygons &trimming_polygons)
{
#ifndef NDEBUG
    for (const Surface &surface : this->slices()) {
        assert(surface.surface_type == (stPosInternal | stDensSparse));
        surface.expolygon.assert_valid();
    }
#endif /* NDEBUG */
    coordf_t scaled_resolution = std::max(SCALED_EPSILON, scale_t(this->layer()->object()->print()->config().resolution.value));
    this->m_slices.set(ensure_valid(intersection_ex(this->slices().surfaces, trimming_polygons)/*, scaled_resolution*/), stPosInternal | stDensSparse);
    for(auto &srf : this->m_slices) srf.expolygon.assert_valid();
}

void LayerRegion::elephant_foot_compensation_step(const float elephant_foot_compensation_perimeter_step, const Polygons &trimming_polygons)
{
#ifndef NDEBUG
    for (const Surface &surface : this->slices()) {
        assert(surface.surface_type == (stPosInternal | stDensSparse));
        surface.expolygon.assert_valid();
    }
#endif /* NDEBUG */
    assert(elephant_foot_compensation_perimeter_step >= 0);
    Polygons tmp = intersection(this->slices().surfaces, trimming_polygons);
    append(tmp, diff(this->slices().surfaces, opening(this->slices().surfaces, elephant_foot_compensation_perimeter_step)));
    this->m_slices.set(union_ex(tmp), stPosInternal | stDensSparse);
    for(auto &srf : this->m_slices) srf.expolygon.assert_valid();
}

void LayerRegion::export_region_slices_to_svg(const char *path) const
{
    BoundingBox bbox;
    for (const Surface& surface : this->slices())
        bbox.merge(get_extents(surface.expolygon));
    Point legend_size = export_surface_type_legend_to_svg_box_size();
    Point legend_pos(bbox.min(0), bbox.max(1));
    bbox.merge(Point(std::max(bbox.min(0) + legend_size(0), bbox.max(0)), bbox.max(1) + legend_size(1)));

    SVG svg(path, bbox);
    const float transparency = 0.5f;
    for (const Surface &surface : this->slices())
        svg.draw(surface.expolygon, surface_type_to_color_name(surface.surface_type, 0.9f), transparency);
    for (const Surface &surface : this->fill_surfaces())
        svg.draw(to_polylines(surface.expolygon), surface_type_to_color_name(surface.surface_type), scale_t(0.1));
    export_surface_type_legend_to_svg(svg, legend_pos);
    svg.Close();
}

// Export to "out/LayerRegion-name-%d.svg" with an increasing index with every export.
void LayerRegion::export_region_slices_to_svg_debug(const char *name) const
{
    static std::map<std::string, size_t> idx_map;
    size_t &idx = idx_map[name];
    this->export_region_slices_to_svg(debug_out_path("LayerRegion-slices-%s-%d.svg", name, idx ++).c_str());
}

void LayerRegion::export_region_fill_surfaces_to_svg(const char *path) const
{
    BoundingBox bbox;
    for (const Surface &surface : this->fill_surfaces())
        bbox.merge(get_extents(surface.expolygon));
    Point legend_size = export_surface_type_legend_to_svg_box_size();
    Point legend_pos(bbox.min(0), bbox.max(1));
    bbox.merge(Point(std::max(bbox.min(0) + legend_size(0), bbox.max(0)), bbox.max(1) + legend_size(1)));

    SVG svg(path, bbox);
    const float transparency = 0.5f;
    for (const Surface &surface : this->fill_surfaces()) {
        svg.draw(surface.expolygon, surface_type_to_color_name(surface.surface_type), transparency);
        svg.draw_outline(surface.expolygon, "black", "blue", scale_(0.05)); 
    }
    export_surface_type_legend_to_svg(svg, legend_pos);
    svg.Close();
}

// Export to "out/LayerRegion-name-%d.svg" with an increasing index with every export.
void LayerRegion::export_region_fill_surfaces_to_svg_debug(const char *name) const
{
    static std::map<std::string, size_t> idx_map;
    size_t &idx = idx_map[name];
    this->export_region_fill_surfaces_to_svg(debug_out_path("LayerRegion-fill_surfaces-%s-%d.svg", name, idx ++).c_str());
}

void LayerRegion::simplify_extrusion_entity()
{

    const PrintConfig& print_config = this->layer()->object()->print()->config();
    const bool spiral_mode = print_config.spiral_vase;
    ArcFittingType enable_arc_fitting = print_config.arc_fitting.value;
    if (spiral_mode)
        enable_arc_fitting = ArcFittingType::Disabled;
    coordf_t scaled_resolution = scale_d(print_config.resolution.value);
    if (enable_arc_fitting != ArcFittingType::Disabled) {
        scaled_resolution = scale_d(print_config.arc_fitting_resolution.get_abs_value(std::max(EPSILON, unscaled(scaled_resolution))));
    }
    if (scaled_resolution == 0) scaled_resolution = enable_arc_fitting != ArcFittingType::Disabled ? SCALED_EPSILON * 2 : SCALED_EPSILON;
    scaled_resolution = std::max(double(SCALED_EPSILON), scaled_resolution);

	//Ligne 652:     SimplifyVisitor(coordf_t scaled_resolution, ArcFittingType use_arc_fitting, const ConfigOptionFloatOrPercent *arc_fitting_tolearance)
    //call simplify for all paths
    Slic3r::SimplifyVisitor visitor{ scaled_resolution , enable_arc_fitting, &print_config.arc_fitting_tolerance, enable_arc_fitting != ArcFittingType::Disabled ? SCALED_EPSILON * 2 : SCALED_EPSILON };
    this->m_perimeters.visit(visitor);
    this->m_fills.visit(visitor);
    this->m_ironings.visit(visitor);
    this->m_millings.visit(visitor);
}

}
 
