#include "ExtrusionProcessor.hpp"
#include <string>

namespace Slic3r { namespace ExtrusionProcessor {

//
//class CalculateAndSliptOverhangingExtrusionsVisitor : public ExtrusionVisitorConst {
//public:


ExtrusionPaths calculate_and_split_overhanging_extrusions(const ExtrusionPath                             &path,
                                                          const AABBTreeLines::LinesDistancer<Linef>      &unscaled_prev_layer,
                                                          const AABBTreeLines::LinesDistancer<CurledLine> &prev_layer_curled_lines,
                                                          const double max_width)
{
    if (!path.role().is_overhang()) {
        return { path };
    }
    //TODO: 'split' lines if the dist of each point is between 0 and max_width, with a max length of path.width/2

    std::vector<ExtendedPoint>           extended_points = estimate_points_properties<true, true, true, true>(path.polyline.to_polyline().points,
                                                                                                    unscaled_prev_layer, path.width());
    std::vector<std::pair<float, float>> calculated_distances(extended_points.size());

    for (size_t i = 0; i < extended_points.size(); i++) {
        const ExtendedPoint &curr = extended_points[i];
        const ExtendedPoint &next = extended_points[i + 1 < extended_points.size() ? i + 1 : i];

        // The following code artifically increases the distance to provide slowdown for extrusions that are over curled lines
        float        proximity_to_curled_lines = 0.0;
        const double dist_limit                = 10.0 * path.width();
        {
            Vec2d middle       = 0.5 * (curr.position + next.position);
            auto  line_indices = prev_layer_curled_lines.all_lines_in_radius(Point::new_scale(middle), scale_(dist_limit));
            if (!line_indices.empty()) {
                double len = (next.position - curr.position).norm();
                // For long lines, there is a problem with the additional slowdown. If by accident, there is small curled line near the middle
                // of this long line
                //  The whole segment gets slower unnecesarily. For these long lines, we do additional check whether it is worth slowing down.
                // NOTE that this is still quite rough approximation, e.g. we are still checking lines only near the middle point
                // TODO maybe split the lines into smaller segments before running this alg? but can be demanding, and GCode will be huge
                if (len > 8) {
                    Vec2d dir   = Vec2d(next.position - curr.position) / len;
                    Vec2d right = Vec2d(-dir.y(), dir.x());

                    Polygon box_of_influence = {
                        scaled(Vec2d(curr.position + right * dist_limit)),
                        scaled(Vec2d(next.position + right * dist_limit)),
                        scaled(Vec2d(next.position - right * dist_limit)),
                        scaled(Vec2d(curr.position - right * dist_limit)),
                    };

                    double projected_lengths_sum = 0;
                    for (size_t idx : line_indices) {
                        const CurledLine &line   = prev_layer_curled_lines.get_line(idx);
                        Lines             inside = intersection_ln({{line.a, line.b}}, {box_of_influence});
                        if (inside.empty())
                            continue;
                        double projected_length = abs(dir.dot(unscaled(Vec2d((inside.back().b - inside.back().a).cast<double>()))));
                        projected_lengths_sum += projected_length;
                    }
                    if (projected_lengths_sum < 0.4 * len) {
                        line_indices.clear();
                    }
                }

                for (size_t idx : line_indices) {
                    const CurledLine &line                 = prev_layer_curled_lines.get_line(idx);
                    float             distance_from_curled = unscaled(line_alg::distance_to(line, Point::new_scale(middle)));
                    float proximity = (1.0 - (distance_from_curled / dist_limit)) * (1.0 - (distance_from_curled / dist_limit)) *
                                      (line.curled_height / (path.height() * 10.0f)); // max_curled_height_factor from SupportSpotGenerator
                    proximity_to_curled_lines = std::max(proximity_to_curled_lines, proximity);
                }
            }
        }
        calculated_distances[i].first  = std::max(curr.distance, next.distance);
        calculated_distances[i].second = proximity_to_curled_lines;
    }

    auto set_between_0_and_1 = [max_width](float val) -> float {
        return std::min(1.f, std::max(0.f, val) / float(max_width));
    };

    ExtrusionPaths      result;
    ExtrusionAttributes new_attrs = path.attributes();
    new_attrs.overhang_attributes = std::optional<OverhangAttributes>(
        {set_between_0_and_1(calculated_distances[0].first), 
        set_between_0_and_1(calculated_distances[0].first), 
        calculated_distances[0].second});
    result.emplace_back(new_attrs);
    result.back().polyline.append(Point::new_scale(extended_points[0].position));
    size_t sequence_start_index = 0;
    for (size_t i = 1; i < extended_points.size(); i++) {
        if (!result.back().polyline.back().coincides_with_epsilon(Point::new_scale(extended_points[i].position))) {
            result.back().polyline.append(Point::new_scale(extended_points[i].position));
            result.back().overhang_attributes_mutable()->end_distance_from_prev_layer =  set_between_0_and_1(extended_points[i].distance);
        }
        if (std::abs(calculated_distances[sequence_start_index].first - calculated_distances[i].first) < 0.001 * max_width &&
            std::abs(calculated_distances[sequence_start_index].second - calculated_distances[i].second) < 0.001) {
            // do not start new path, the attributes are similar enough
            // NOTE: a larger tolerance may be applied here. However, it makes the gcode preview much less smooth
            // (But it has very likely zero impact on the print quality.)
        } else if (i + 1 < extended_points.size()) { // do not start new path if this is last point!
            // start new path, parameters differ
            //set all between 0 and 1
            new_attrs.overhang_attributes->start_distance_from_prev_layer = set_between_0_and_1(calculated_distances[i].first);
            new_attrs.overhang_attributes->end_distance_from_prev_layer   = set_between_0_and_1(calculated_distances[i].first);
            //already between 0 and 1
            new_attrs.overhang_attributes->proximity_to_curled_lines      = calculated_distances[i].second;
            sequence_start_index                                          = i;
            if (result.back().size() > 1) {
                result.emplace_back(new_attrs);
                result.back().polyline.append(Point::new_scale(extended_points[i].position));
            } else {
                assert(result.back().size() == 1);
                assert(result.back().polyline.back().coincides_with_epsilon(Point::new_scale(extended_points[i].position)));
            }
        }
    }
    // started new path, but there is noting after that.
    if (result.back().size() == 1) {
        //delete it
        result.pop_back();
    }
#ifdef _DEBUG
    for (auto &path : result) {
        assert(path.attributes().overhang_attributes.has_value());
        assert(path.attributes().overhang_attributes->start_distance_from_prev_layer >= 0 &&
               path.attributes().overhang_attributes->start_distance_from_prev_layer <= 1);
    }
    assert(is_approx(result.front().first_point(), path.first_point()));
    assert(is_approx(result.back().last_point(), path.last_point()));
    Point last_pt = result.front().last_point();
    for (size_t idx_path = 1; idx_path < result.size() ; ++idx_path) {
        ExtrusionPath &path = result[idx_path];
        assert(path.polyline.size() >= 2);
        assert(path.first_point().coincides_with_epsilon(last_pt));
        if (!path.polyline.get_point(1).coincides_with_epsilon(last_pt)) {
            path.polyline.set_front(last_pt);
        }
        for (size_t idx_pt = 1; idx_pt < path.size(); ++idx_pt)
            assert(!path.polyline.get_point(idx_pt - 1).coincides_with_epsilon(path.polyline.get_point(idx_pt)));
        last_pt = path.last_point();
    }
#endif
    // avoid precision loss
    result.front().polyline.set_front(path.first_point());
    result.back().polyline.set_back(path.last_point());
    return result;
};

ExtrusionEntityCollection calculate_and_split_overhanging_extrusions(const ExtrusionEntityCollection            *ecc,
                                                                     const AABBTreeLines::LinesDistancer<Linef> &unscaled_prev_layer,
                                                                     const AABBTreeLines::LinesDistancer<CurledLine> &prev_layer_curled_lines,
                                                                     const double max_width)
{
    ExtrusionEntityCollection result{};
    result.set_can_sort_reverse(ecc->can_sort(), ecc->can_reverse());
    for (const auto *e : ecc->entities()) {
        if (auto *col = dynamic_cast<const ExtrusionEntityCollection *>(e)) {
            result.append(calculate_and_split_overhanging_extrusions(col, unscaled_prev_layer, prev_layer_curled_lines, max_width));
        } else if (auto *loop = dynamic_cast<const ExtrusionLoop *>(e)) {
#ifdef _DEBUG
    Point last_pt = loop->last_point();
    for (const ExtrusionPath &path : loop->paths) {
        assert(path.polyline.size() >= 2);
        assert(path.first_point() == last_pt);
        for (size_t idx = 1; idx < path.size(); ++idx)
            assert(!path.polyline.get_point(idx - 1).coincides_with_epsilon(path.polyline.get_point(idx)));
        last_pt = path.last_point();
    }
#endif
            ExtrusionLoop new_loop = *loop;
            new_loop.paths.clear();
            for (const ExtrusionPath &p : loop->paths) {
                auto paths = calculate_and_split_overhanging_extrusions(p, unscaled_prev_layer, prev_layer_curled_lines, max_width);
                assert(p.first_point() == paths.front().first_point());
                assert(p.last_point() == paths.back().last_point());
                new_loop.paths.insert(new_loop.paths.end(), paths.begin(), paths.end());
            }
            result.append(std::move(new_loop));
        } else if (auto *mp = dynamic_cast<const ExtrusionMultiPath *>(e)) {
            ExtrusionMultiPath new_mp = *mp;
            new_mp.paths.clear();
            for (const ExtrusionPath &p : mp->paths) {
                auto paths = calculate_and_split_overhanging_extrusions(p, unscaled_prev_layer, prev_layer_curled_lines, max_width);
                assert(p.first_point() == paths.front().first_point());
                assert(p.last_point() == paths.back().last_point());
                new_mp.paths.insert(new_mp.paths.end(), paths.begin(), paths.end());
            }
            result.append(std::move(new_mp));
        } else if (auto *p = dynamic_cast<const ExtrusionPath *>(e)) {
            result.append(calculate_and_split_overhanging_extrusions(*p, unscaled_prev_layer, prev_layer_curled_lines, max_width));
        } else if (auto *mp = dynamic_cast<const ExtrusionMultiPath3D *>(e)) {
            ExtrusionMultiPath3D new_mp = *mp;
            result.append(std::move(new_mp)); //TODO split
        } else if (auto *mp = dynamic_cast<const ExtrusionPath3D *>(e)) {
            ExtrusionPath3D new_mp = *mp;
            result.append(std::move(new_mp)); //TODO split
        } else {
            throw Slic3r::InvalidArgument("Unknown extrusion entity type");
        }
    }
    return result;
};


std::pair<float,float> calculate_overhang_speed(const ExtrusionAttributes &attributes,
                              const FullPrintConfig     &config,
                              size_t                     extruder_id)
{
    assert(attributes.overhang_attributes.has_value());
    if(!attributes.overhang_attributes.has_value())
        return {-1, -1};
    float speed_ratio = 0; // 0: overhangs speed, 1= periemter/externalperiemter speed.
    float fan_speed = -1;
        //= {{100, ConfigOptionFloatOrPercent{default_speed, false}}};
    if (config.overhangs_dynamic_speed.is_enabled()) {
        GraphData graph = config.overhangs_dynamic_speed.value;
        // ensure it start at 0%, and ensure it ends at 100%
        if (graph.graph_points[graph.begin_idx].x() != 0) {
            graph.graph_points.insert(graph.graph_points.begin() + graph.begin_idx, {0,0});
            graph.end_idx++;
        }
        if (graph.graph_points[graph.end_idx - 1].x() != 100) {
            graph.graph_points.insert(graph.graph_points.begin() + graph.end_idx, {100,100});
            graph.end_idx++;
        }
        graph.graph_points[graph.begin_idx].x() = 0;
        graph.graph_points[graph.end_idx - 1].y() = 100;
        //interpolate
        assert(attributes.overhang_attributes->start_distance_from_prev_layer >= 0 &&
               attributes.overhang_attributes->start_distance_from_prev_layer <= 1);
        assert(attributes.overhang_attributes->end_distance_from_prev_layer >= 0 &&
               attributes.overhang_attributes->end_distance_from_prev_layer <= 1);
        float extrusion_ratio   = std::min(
                     graph.interpolate(100 - 100 * attributes.overhang_attributes->start_distance_from_prev_layer),
                     graph.interpolate(100 - 100 * attributes.overhang_attributes->end_distance_from_prev_layer));
        assert(attributes.width * attributes.overhang_attributes->proximity_to_curled_lines >= 0 &&
               attributes.width * attributes.overhang_attributes->proximity_to_curled_lines <= 1);
        float curled_extrusion_ratio = graph.interpolate(100 - 100 * attributes.overhang_attributes->proximity_to_curled_lines);
        //float curled_base_speed = interpolate_speed(speed_sections, attributes.width * attributes.overhang_attributes->proximity_to_curled_lines);
        speed_ratio       = std::min(extrusion_ratio, curled_extrusion_ratio) / 100.0;
        assert(speed_ratio >= 0 && speed_ratio <= 1);
        //overhangs_with_speeds = {{0, config.overhang_speed_0},
        //                         {25, config.overhang_speed_1},
        //                         {50, config.overhang_speed_2},
        //                         {75, config.overhang_speed_3},
        //                         {100, ConfigOptionFloatOrPercent{default_speed, false}}};
    }

    std::vector<std::pair<int, ConfigOptionInts>> overhang_with_fan_speeds = {{100, ConfigOptionInts{0}}};
    if (config.overhangs_dynamic_fan_speed.is_enabled(extruder_id) && attributes.overhang_attributes->start_distance_from_prev_layer > 0 && attributes.overhang_attributes->end_distance_from_prev_layer > 0) {
        GraphData graph = config.overhangs_dynamic_fan_speed.get_at(extruder_id);
        //interpolate
        assert(attributes.overhang_attributes->start_distance_from_prev_layer >= 0 &&
               attributes.overhang_attributes->start_distance_from_prev_layer <= 1);
        assert(attributes.overhang_attributes->end_distance_from_prev_layer >= 0 &&
               attributes.overhang_attributes->end_distance_from_prev_layer <= 1);
        fan_speed = std::min(
                     graph.interpolate(100 - 100 * attributes.overhang_attributes->start_distance_from_prev_layer),
                     graph.interpolate(100 - 100 * attributes.overhang_attributes->end_distance_from_prev_layer));
        //float fan_speed = std::min(interpolate_speed(fan_speed_sections, attributes.overhang_attributes->start_distance_from_prev_layer),
        //                       interpolate_speed(fan_speed_sections, attributes.overhang_attributes->end_distance_from_prev_layer));
        assert(fan_speed >= 0 && fan_speed <= 100);
        //overhangs_with_speeds = {{0, config.overhang_speed_0},
        //                         {25, config.overhang_speed_1},
        //                         {50, config.overhang_speed_2},
        //                         {75, config.overhang_speed_3},
        //                         {100, ConfigOptionFloatOrPercent{default_speed, false}}};
        //overhang_with_fan_speeds = {{0, config.overhang_fan_speed_0},
        //                            {25, config.overhang_fan_speed_1},
        //                            {50, config.overhang_fan_speed_2},
        //                            {75, config.overhang_fan_speed_3},
        //                            {100, ConfigOptionInts{0}}};
    }

    //float                  speed_base = external_perim_reference_speed > 0 ? external_perim_reference_speed : default_speed;
    //std::map<float, float> speed_sections;
    //for (size_t i = 0; i < overhangs_with_speeds.size(); i++) {
    //    float distance = attributes.width * (1.0 - (overhangs_with_speeds[i].first / 100.0));
    //    float speed    = overhangs_with_speeds[i].second.percent ? (speed_base * overhangs_with_speeds[i].second.value / 100.0) :
    //                                                               overhangs_with_speeds[i].second.value;
    //    if (speed < EPSILON)
    //        speed = speed_base;
    //    speed_sections[distance] = speed;
    //}

    //std::map<float, float> fan_speed_sections;
    //for (size_t i = 0; i < overhang_with_fan_speeds.size(); i++) {
    //    float distance               = attributes.width * (1.0 - (overhang_with_fan_speeds[i].first / 100.0));
    //    float fan_speed              = overhang_with_fan_speeds[i].second.get_at(extruder_id);
    //    fan_speed_sections[distance] = fan_speed;
    //}

    //auto interpolate_speed = [](const std::map<float, float> &values, float distance) {
    //    auto upper_dist = values.lower_bound(distance);
    //    if (upper_dist == values.end()) {
    //        return values.rbegin()->second;
    //    }
    //    if (upper_dist == values.begin()) {
    //        return upper_dist->second;
    //    }

    //    auto  lower_dist = std::prev(upper_dist);
    //    float t          = (distance - lower_dist->first) / (upper_dist->first - lower_dist->first);
    //    return (1.0f - t) * lower_dist->second + t * upper_dist->second;
    //};

    //float fan_speed = std::min(interpolate_speed(fan_speed_sections, attributes.overhang_attributes->start_distance_from_prev_layer),
    //                           interpolate_speed(fan_speed_sections, attributes.overhang_attributes->end_distance_from_prev_layer));

    //if (!config.enable_dynamic_overhang_speeds) {
    //    final_speed = -1;
    //}
    //if (!config.enable_dynamic_fan_speeds.get_at(extruder_id)) {
    //    fan_speed = -1;
    //}

    //return {final_speed, fan_speed};
    return {speed_ratio, fan_speed};
}

}} // namespace Slic3r::ExtrusionProcessor
