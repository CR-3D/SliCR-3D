#include "Wipe.hpp"
#include "../GCode.hpp"

#include <string_view>

#include <Eigen/Geometry>

using namespace std::string_view_literals;

namespace Slic3r::GCode {

void Wipe::init(const PrintConfig &config, const GCodeWriter &writer, const std::vector<uint16_t> &extruders)
{
    this->reset_path();

    // Calculate maximum wipe length to accumulate by the wipe cache.
    // Paths longer than wipe_xy should never be needed for the wipe move.
    double wipe_xy = 0;
    const bool multimaterial = extruders.size() > 1;
    for (uint16_t id : extruders) // != writer.extruders() ?
        if (config.wipe.get_at(id)) {
            // Wipe length to extrusion ratio.
            const double xy_to_e = this->calc_xy_to_e_ratio(writer, id);
            wipe_xy              = std::max(wipe_xy, writer.gcode_config().retract_length.get_at(id) / xy_to_e);
            if (multimaterial)
                wipe_xy = std::max(wipe_xy, writer.gcode_config().retract_length_toolchange.get_at(id) / xy_to_e);
        }

    if (wipe_xy == 0)
        this->disable();
    else
        this->enable(wipe_xy);
}

void Wipe::set_path(const Path &path, bool is_loop) {
    assert(path.empty() || path.size() > 1);
    this->reset_path();
    if (this->is_enabled() && path.size() > 1)
        m_path = path;
    m_path_is_loop = is_loop;
}

void Wipe::set_path(Path &&path, bool is_loop) {
    assert(path.empty() || path.size() > 1);
    this->reset_path();
    if (this->is_enabled() && path.size() > 1)
        m_path = std::move(path);
    m_path_is_loop = is_loop;
}

void Wipe::set_path(const ExtrusionPaths &paths, bool reversed, bool is_loop)
{
    this->reset_path();
    m_path_is_loop = is_loop;

    if (this->is_enabled() && ! paths.empty()) {
        coord_t wipe_len_max_scaled = scaled(m_wipe_len_max);
        if (reversed) {
            m_path = paths.back().as_polyline().get_arc();
            Geometry::ArcWelder::reverse(m_path);
            int64_t len = Geometry::ArcWelder::estimate_path_length(m_path);
            for (auto it = std::next(paths.rbegin()); len < wipe_len_max_scaled && it != paths.rend(); ++ it) {
                if (it->role().is_bridge())
                    break; // Do not perform a wipe on bridges.
                assert(it->size() >= 2);
                assert(m_path.back().point == it->last_point());
                if (m_path.back().point != it->last_point())
                    // ExtrusionMultiPath is interrupted in some place. This should not really happen.
                    break;
                ArcPolyline polyline = it->as_polyline();
                len += Geometry::ArcWelder::estimate_path_length(polyline.get_arc());
                m_path.insert(m_path.end(), polyline.get_arc().rbegin() + 1, polyline.get_arc().rend());
            }
        } else {
            m_path = std::move(paths.front().as_polyline().get_arc());
            int64_t len = Geometry::ArcWelder::estimate_path_length(m_path);
            for (auto it = std::next(paths.begin()); len < wipe_len_max_scaled && it != paths.end(); ++ it) {
                if (it->role().is_bridge())
                    break; // Do not perform a wipe on bridges.
                assert(it->size() >= 2);
                assert(m_path.back().point == it->first_point());
                if (m_path.back().point != it->first_point())
                    // ExtrusionMultiPath is interrupted in some place. This should not really happen.
                    break;
                ArcPolyline polyline = it->as_polyline();
                len += Geometry::ArcWelder::estimate_path_length(polyline.get_arc());
                m_path.insert(m_path.end(), polyline.get_arc().begin() + 1, polyline.get_arc().end());
            }
        }
    }

    assert(m_path.empty() || m_path.size() > 1);
}

std::pair<double, bool> Wipe::calc_wipe_speed(const GCodeWriter &writer)
{
    double wipe_speed = writer.gcode_config().get_computed_value("travel_speed") * 0.8;
    bool use_wipe_speed = false;
    if (writer.tool_is_extruder() && writer.gcode_config().wipe_speed.get_at(writer.tool()->id()) > 0) {
        wipe_speed = writer.gcode_config().wipe_speed.get_at(writer.tool()->id());
        use_wipe_speed = true;
    }
    return {wipe_speed, use_wipe_speed};
}

std::string Wipe::wipe(GCodeGenerator &gcodegen, bool toolchange)
{
    std::string     gcode;
    const Tool &extruder = *gcodegen.writer().tool();
    static constexpr const std::string_view wipe_retract_comment = "wipe and retract"sv;
    const bool use_firmware_retract = gcodegen.writer().gcode_config().use_firmware_retraction.value;

    if (!gcodegen.writer().tool_is_extruder())
        return "";

    // Remaining quantized retraction length.
    double retract_length = extruder.retract_length();
    if (toolchange) {
        retract_length = extruder.retract_length_toolchange();
    } else if (gcodegen.writer().print_region_config() && gcodegen.writer().print_region_config()->print_retract_length.is_enabled()) {
        retract_length = gcodegen.writer().print_region_config()->print_retract_length.value;
    }
    retract_length = extruder.retract_to_go(retract_length);
    double nozzle_diameter = extruder.id() < 0 ? 0.4 : gcodegen.config().nozzle_diameter.get_at(extruder.id());
    const double xy_to_e    = this->calc_xy_to_e_ratio(gcodegen.writer(), extruder.id());
    /*const*/ double wipe_total_length = std::max(retract_length, gcodegen.config().wipe_min.get_abs_value(extruder.id(), retract_length / xy_to_e));
    if ( wipe_total_length > 0 && this->has_path()) {
        // if wipe_min == 0, then don't try to go farther than the current path (else, set it to 100%)
        if (gcodegen.config().wipe_min.get_at(extruder.id()).value == 0) {
            wipe_total_length = std::min(wipe_total_length, Geometry::ArcWelder::path_length<coordf_t>(this->path()));
        }
        double wipe_length = wipe_total_length;
        /*const*/double lift_length = extruder.id() < 0 ? 0 : gcodegen.config().wipe_lift_length.get_abs_value(extruder.id(), wipe_length);
        lift_length = std::min(lift_length, wipe_total_length);
        double no_lift_length = wipe_total_length - lift_length;
        assert(no_lift_length >= 0);
        const double lift = extruder.id() < 0 ? 0 : gcodegen.config().wipe_lift.get_abs_value(extruder.id(), gcodegen.layer()->height);
        const double lift_per_mm = lift / lift_length;
        const double initial_z = gcodegen.writer().get_position().z();
        double current_z = initial_z;
        const double final_z = gcodegen.writer().get_position().z() + lift;
        auto         wipe_linear = [&gcode, &gcodegen, &retract_length, &wipe_length, &no_lift_length, &current_z, xy_to_e, use_firmware_retract, lift_per_mm, final_z]
        (const Vec2d &prev_quantized, Vec2d &p, bool &done)->bool { //return false if point is used, true if need to be recalled again
            bool partial_segment = false;
            Vec2d  p_quantized = gcodegen.writer().get_default_gcode_formatter().quantize(p);
            if (p_quantized == prev_quantized) {
                p = prev_quantized; // keep old prev
                return partial_segment;
            }
            double segment_length = (p_quantized - prev_quantized).norm();
            // Compute a min dist between point, to avoid going under the precision.
            double precision = pow(10, -gcodegen.config().gcode_precision_xyz.value) * 1.5;
            {
                precision = std::max(precision, (EPSILON * 10));
                if (gcodegen.config().resolution.value > 0) {
                    precision = std::max(precision, gcodegen.config().resolution.value);
                }
                if (segment_length < precision) {
                    p = prev_quantized; // keep old prev
                    return partial_segment;
                }
            }
            if (wipe_length < precision) {
                //we can stop here
                p = prev_quantized; // keep old prev
                wipe_length = 0;
                done = true;
                return false;
            }
            //reduce length if no_lift_length in the middle
            if (no_lift_length + precision / 2 < segment_length && no_lift_length > precision) {
                // Shorten the segment.
                p = p_quantized = gcodegen.writer().get_default_gcode_formatter().quantize(
                    Vec2d(prev_quantized + (p - prev_quantized) * (no_lift_length / segment_length)));
                segment_length = (p_quantized - prev_quantized).norm();
                partial_segment = true;
                assert(is_approx(no_lift_length, segment_length, 0.01));
                no_lift_length = EPSILON *2;
            }
            //reduce length if wipe_length in the middle
            if (wipe_length + precision / 2 < segment_length) {
                // Shorten the segment.
                p = p_quantized = gcodegen.writer().get_default_gcode_formatter().quantize(
                    Vec2d(prev_quantized + (p - prev_quantized) * (wipe_length / segment_length)));
                segment_length = (p_quantized - prev_quantized).norm();
                partial_segment = true;
                assert(is_approx(wipe_length, segment_length, 0.01));
            }
            // Quantize E axis as it is to be extruded as a whole segment.
            double dE = 0;
            if (retract_length > 0) {
                dE = gcodegen.writer().get_default_gcode_formatter().quantize_e(xy_to_e * segment_length);
                if (dE > retract_length - EPSILON) {
                    if (dE > retract_length + EPSILON) {
                        // Shorten the segment.
                        p_quantized = gcodegen.writer().get_default_gcode_formatter().quantize(
                            Vec2d(prev_quantized + (p - prev_quantized) * (retract_length / dE)));
                        segment_length = (p_quantized - prev_quantized).norm();
                        partial_segment = true;
                        // test if the shorten segment isn't too short
                        if (p_quantized == prev_quantized) {
                            if (!use_firmware_retract) {
                                // add it as missing extrusion to push through as soon as possible.
                                gcodegen.writer().add_de_delayed(retract_length);
                            }
                            // too small to print, ask for a retry (with no retract_length left)
                            retract_length = 0;
                            return true;
                        }
                    }
                    dE = retract_length;
                }
            }
            p = p_quantized;
            assert(p.x() != gcodegen.writer().get_position().x() || p.y() != gcodegen.writer().get_position().y());
            if (lift_per_mm == 0 || no_lift_length > EPSILON) {
                if (dE > 0) {
                    gcode += gcodegen.writer().extrude_to_xy(p, use_firmware_retract ? 0 : -dE, wipe_retract_comment);
                } else {
                    gcode += gcodegen.writer().travel_to_xy(p, 0,  wipe_retract_comment);
                }
            } else {
                current_z = std::min(final_z, current_z + segment_length * lift_per_mm);
                Vec3d p3d(p.x(), p.y(), current_z);
                if (dE > 0) {
                    gcode += gcodegen.writer().extrude_to_xyz(p3d, use_firmware_retract ? 0 : -dE, wipe_retract_comment);
                } else {
                    gcode += gcodegen.writer().travel_to_xyz(p3d, false, 0,  wipe_retract_comment);
                }
            }
            retract_length -= dE;
            wipe_length -= segment_length;
            no_lift_length -= segment_length;
            done = wipe_length < EPSILON;
            assert(wipe_length > -0.1);
            return partial_segment;
        };
        auto         wipe_arc = [this, &gcode, &gcodegen, &retract_length, &wipe_length, &no_lift_length, &current_z, xy_to_e, use_firmware_retract, lift_per_mm, final_z, &wipe_linear]
                (const Vec2d &prev_quantized, Vec2d &p,  const Slic3r::Geometry::ArcWelder::Segment &seg/*double radius_in, const bool ccw, */, bool &done)->bool { //return false if point is used, true if need to be recalled again
            const double radius_in = unscaled(seg.radius);
            const bool ccw = seg.ccw();
            bool partial_segment = false;
            Vec2d  p_quantized = gcodegen.writer().get_default_gcode_formatter().quantize(p);
            if (p_quantized == prev_quantized) {
                p = prev_quantized; // keep old prev
                return partial_segment;
            }
            // Use the exact radius for calculating the IJ values, no quantization.
            double radius = radius_in;
            if (radius == 0) {
                // Degenerated arc after quantization. Process it as if it was a line segment.
                return wipe_linear(prev_quantized, p, done);
            }
            Vec2d  center = Geometry::ArcWelder::arc_center(prev_quantized.cast<double>(), p_quantized.cast<double>(), double(radius), ccw);
            float  angle  = Geometry::ArcWelder::arc_angle(prev_quantized.cast<double>(), p_quantized.cast<double>(), double(radius));
            assert(angle > 0);
            double segment_length = angle * std::abs(radius);
            if (wipe_length < EPSILON * 10) {
                //we can stop here
                p = prev_quantized; // keep old prev
                wipe_length = 0;
                done = true;
                return false;
            }
            // reduce length if no_lift_length in the middle
            if (no_lift_length + EPSILON * 10 < segment_length && no_lift_length > EPSILON * 10) {
                // Shorten the segment. Recalculate the arc from the unquantized end coordinate.
                center = Geometry::ArcWelder::arc_center(prev_quantized.cast<double>(), p.cast<double>(), double(radius), ccw);
                angle = Geometry::ArcWelder::arc_angle(prev_quantized.cast<double>(), p.cast<double>(), double(radius));
                segment_length = angle * std::abs(radius);
                Vec2d new_p_quantized = gcodegen.writer().get_default_gcode_formatter().quantize(
                        Vec2d(center + Eigen::Rotation2D((ccw ? angle : -angle) * (no_lift_length / segment_length)) * (prev_quantized - center)));
                segment_length = no_lift_length;
                no_lift_length = EPSILON *2;
                partial_segment = true;
            }
            // reduce length if wipe_length in the middle
            if (wipe_length + EPSILON * 10 < segment_length) {
                // Shorten the segment. Recalculate the arc from the unquantized end coordinate.
                center = Geometry::ArcWelder::arc_center(prev_quantized.cast<double>(), p.cast<double>(), double(radius), ccw);
                angle = Geometry::ArcWelder::arc_angle(prev_quantized.cast<double>(), p.cast<double>(), double(radius));
                segment_length = angle * std::abs(radius);
                Vec2d new_p_quantized = gcodegen.writer().get_default_gcode_formatter().quantize(
                        Vec2d(center + Eigen::Rotation2D((ccw ? angle : -angle) * (wipe_length / segment_length)) * (prev_quantized - center)));
                segment_length = wipe_length;
                partial_segment = true;
            }
            double dE = 0;
            if(retract_length > 0){
                dE = gcodegen.writer().get_default_gcode_formatter().quantize_e(xy_to_e * segment_length);
                if (dE > retract_length - EPSILON) {
                    if (dE > retract_length + EPSILON) {
                        // Shorten the segment. Recalculate the arc from the unquantized end coordinate.
                        center = Geometry::ArcWelder::arc_center(prev_quantized.cast<double>(), p.cast<double>(), double(radius), ccw);
                        angle = Geometry::ArcWelder::arc_angle(prev_quantized.cast<double>(), p.cast<double>(), double(radius));
                        segment_length = angle * std::abs(radius);
                        partial_segment = true;
                        dE = xy_to_e * segment_length;
                        p_quantized = gcodegen.writer().get_default_gcode_formatter().quantize(
                                Vec2d(center + Eigen::Rotation2D((ccw ? angle : -angle) * (retract_length / dE)) * (prev_quantized - center)));
                    }
                    dE   = retract_length;
                }
                assert(dE > 0);
            }
            {
                // Calculate quantized IJ circle center offset.
                Vec2d ij = gcodegen.writer().get_default_gcode_formatter().quantize(Vec2d(center - prev_quantized));
                if (ij == Vec2d::Zero()) {
                    // Degenerated arc after quantization. Process it as if it was a line segment.
                    return wipe_linear(prev_quantized, p, done) || partial_segment;
                }
                // use quantized version
                p = p_quantized;
                // The arc is valid.
                if (lift_per_mm == 0 || no_lift_length > EPSILON) {
                    if (dE > 0) {
                        gcode += gcodegen.writer().extrude_arc_to_xy(p, ij, use_firmware_retract ? 0 : -dE, ccw, wipe_retract_comment);
                    } else {
                        gcode += gcodegen.writer().travel_arc_to_xy(p, ij, ccw, 0/*speed*/, wipe_retract_comment);
                    }
                } else {
                    current_z = std::min(final_z, current_z + segment_length * lift_per_mm);
                    Vec3d p3d(p.x(), p.y(), current_z);
                    gcode += gcodegen.writer().extrude_arc_to_xyz(p3d, ij, use_firmware_retract ? 0 : -dE, ccw, wipe_retract_comment);
                }
            }
            retract_length -= dE;
            wipe_length -= segment_length;
            no_lift_length -= segment_length;
            done = wipe_length < EPSILON;
            //assert(wipe_length > -0.1);
            return partial_segment;
        };
        // Start with the current position, which may be different from the wipe path start in case of loop clipping.
        Vec2d prev = gcodegen.point_to_gcode_quantized(gcodegen.last_pos_defined() ? gcodegen.last_pos() : path().front().point);
#ifdef _DEBUG
        for (size_t i = 1; i < path().size(); ++i) {
            assert(!path()[i - 1].point.coincides_with_epsilon(path()[i].point));
        }
#endif
        auto iterate_path = [&]() {
            Vec2d p;
            auto end = this->path().end();
            size_t idx = 0;
            for (auto it = this->path().begin(); it != end; ++it) {
                p = gcodegen.point_to_gcode(it->point + m_offset);
                // wipe_xxx check itself if prev == p (with quantization)
                bool done = false;
                while (it->linear() ? wipe_linear(prev, p, done) : wipe_arc(prev, p, *it/*unscaled<double>(it->radius), it->ccw()*/, done)) {
                    prev = p;
                    if(done) return;
                    p = gcodegen.point_to_gcode(it->point + m_offset);
                }
                // wipe has updated p into quantized-prev point for next loop
                prev = p;
                if(done) return;
                idx++;
            }
        };
        ArcPolyline arcpoly(this->path());
        iterate_path();

        // if the current path is very short, then use the boundaries (if any).
        if (wipe_length > wipe_total_length / 5 && m_boundaries && !m_boundaries->empty() && gcodegen.config().wipe_min.get_at(extruder.id()).value != 0) {
            Point start = gcodegen.gcode_to_point(prev);
            //TODO: optimisatin if it takes too long.
            // imo, it shouldn't be triggered that often.
            //choose the right one
            ExPolygon my_boundary;
            for (const ExPolygon &boundary : *m_boundaries) {
                BoundingBox bb = get_extents(boundary.contour);
                if (bb.contains(start)) {
                    if (boundary.contains(start)) {
                        // worth it?
                        coordf_t arc_path_length = Geometry::ArcWelder::path_length<coordf_t>(this->path());
                        if (boundary.contour.length() < arc_path_length * 2) {
                            // no
                            break;
                        }
                        coord_t offset = scale_t(nozzle_diameter * 1.5);
                        ExPolygons offseted_boundary = offset_ex(boundary, -offset);
                        ensure_valid(offseted_boundary, std::max(SCALED_EPSILON, scale_t(nozzle_diameter) / 10));
                        if (offseted_boundary.size() > 1) {
                            for (auto &expoly : offseted_boundary) {
                                if (expoly.contains(start)) {
                                    my_boundary = expoly;
                                    break;
                                }
                            }
                        } else if (!offseted_boundary.empty()) {
                            my_boundary = offseted_boundary.front();
                        }
                        // try again with lower offset, if it's worth
                        if (my_boundary.empty()) {
                            offset = scale_t(nozzle_diameter * 0.7);
                            offseted_boundary = offset_ex(boundary, -offset);
                            ensure_valid(offseted_boundary, std::max(SCALED_EPSILON, scale_t(nozzle_diameter) / 10));
                            if (offseted_boundary.size() > 1) {
                                for (auto &expoly : offseted_boundary) {
                                    if (expoly.contains(start)) {
                                        my_boundary = expoly;
                                        break;
                                    }
                                }
                            } else if (!offseted_boundary.empty()) {
                                my_boundary = offseted_boundary.front();
                            }
                        }
                        // still use the boundary if the current path is really too short
                        if (my_boundary.empty() && arc_path_length < nozzle_diameter * 10) {
                            my_boundary = boundary;
                        }
                        break;
                    }
                }
            }
            if (!my_boundary.empty()) {
                // go to nearest point in contour
                Polyline poly;
                coordf_t max_dist_sqr = start.distance_to_square(my_boundary.contour.front());
                size_t best_idx = 0;
                for (size_t idx = 1; idx < my_boundary.contour.size(); idx++) {
                    coordf_t dist_sqr = start.distance_to_square(my_boundary.contour.points[idx]);
                    if (max_dist_sqr > dist_sqr) {
                        max_dist_sqr = dist_sqr;
                        best_idx = idx;
                    }
                }
                bool travel_ok = false;
                Polylines test_cross = intersection_pl(Polyline({start, my_boundary.contour.points[best_idx]}),
                                                        my_boundary);
                if (test_cross.empty()) {
                    travel_ok = max_dist_sqr < sqr(nozzle_diameter * 2);
                } else if (test_cross.size() == 1) {
                    travel_ok =true;
                    poly = my_boundary.contour.split_at_index(best_idx);
                } else {
                    // get the nearest hole (if nearest than contour)
                    size_t best_id_hole = -1;
                    for (size_t id_hole = 0; id_hole < my_boundary.holes.size(); id_hole++) {
                        for (size_t idx = 0; idx < my_boundary.holes[id_hole].size(); idx++) {
                            coordf_t dist_sqr = start.distance_to_square(my_boundary.holes[id_hole].points[idx]);
                            if (max_dist_sqr > dist_sqr) {
                                max_dist_sqr = dist_sqr;
                                best_idx = idx;
                                best_id_hole = id_hole;
                            }
                        }
                    }
                    if (best_id_hole != size_t(-1)) {
                        poly = my_boundary.holes[best_id_hole].split_at_index(best_idx);
                    } else {
                        poly = my_boundary.contour.split_at_index(best_idx);
                    }
                    travel_ok =true;
                }
                if (travel_ok) {
                    assert(!poly.empty());
                    // if useful use it, else discard and use path
                    if (poly.length() > Geometry::ArcWelder::path_length<coordf_t>(this->path())) {
                        // iterate on it
                        auto iterate_polyline = [&]() {
                            Vec2d p;
                            auto end = poly.points.end();
                            for (auto it = poly.points.begin(); it != end; ++it) {
                                p = gcodegen.point_to_gcode(*it + m_offset);
                                // wipe_xxx check itself if prev == p (with quantization)
                                bool done = false;
                                while (wipe_linear(prev, p, done)) {
                                    prev = p;
                                    if(done) return;
                                    p = gcodegen.point_to_gcode(*it + m_offset);
                                }
                                // wipe has updated p into quantized-prev point for next loop
                                prev = p;
                                if(done) return;
                            }
                        };
                        iterate_polyline();
                        while (wipe_length > 0) {
                            poly.reverse();
                            iterate_polyline();
                        }
                    }
                }
            }
        }
        
        // if not enough, then use the same path again (in reverse if not a loop)
        size_t max_iteration = 3;
        while (wipe_length > 0 && gcodegen.config().wipe_min.get_at(extruder.id()).value != 0 && max_iteration > 0) {
            if (!m_path_is_loop) {
                Geometry::ArcWelder::reverse(m_path);
            }
            ArcPolyline arcpoly2(this->path());
            iterate_path();
            max_iteration--;
        }

        // set new current point in gcodegen
        auto pq = gcodegen.writer().get_default_gcode_formatter().quantize(prev);
        assert(prev == gcodegen.writer().get_default_gcode_formatter().quantize(prev));
        gcodegen.set_last_pos(gcodegen.gcode_to_point(prev));

        // set extra z as lift, so we don't start to extrude in mid-air.
        if (lift_per_mm != 0) {
            gcodegen.writer().set_lift(gcodegen.writer().get_position().z() - initial_z);
        }
    }

    // Prevent wiping again on the same path.
    this->reset_path();
    if (!gcode.empty()) {
        std::pair<double, bool> wipe_speed = this->calc_wipe_speed(gcodegen.writer());
        // Delayed emitting of a wipe start tag.
        gcode = std::string(";") + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Start) + std::string("\n")
        // Delayed emitting of a wipe speed.
            + gcodegen.writer().set_speed_mm_s(wipe_speed.first, gcodegen.config().gcode_comments ? wipe_speed.second? "wipe_speed"sv : "travel_speed * 0.8"sv : ""sv , gcodegen.enable_cooling_markers() ? ";_WIPE"sv : ""sv)
            + gcode;
        // add tag for processor
        gcode += ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_End) + "\n";
    }
    return gcode;
}


// Returns true if the smooth path is longer than a threshold.
bool longer_than(const ExtrusionPaths &paths, double length)
{
    for (const ExtrusionPath &path : paths) {
        ArcPolyline polyline = path.as_polyline();
        for (auto it = std::next(polyline.get_arc().begin()); it != polyline.get_arc().end(); ++it) {
            length -= Geometry::ArcWelder::segment_length<double>(*std::prev(it), *it);
            if (length < 0)
                return true;
        }
    }
    return length < 0;
}


// 
// Length of a smooth path.
//
std::optional<Point> sample_path_point_at_distance_from_start(const ExtrusionPaths &paths, double distance)
{
    if (distance >= 0) {
        for (const ExtrusionPath &path : paths) {
            ArcPolyline polyline = path.as_polyline();
            auto it  = polyline.get_arc().begin();
            auto end = polyline.get_arc().end();
            Point prev_point = it->point;
            for (++ it; it != end; ++ it) {
                Point point = it->point;
                if (it->linear()) {
                    // Linear segment
                    Vec2d  v    = (point - prev_point).cast<double>();
                    double lsqr = v.squaredNorm();
                    if (lsqr > sqr(distance))
                        return std::make_optional<Point>(prev_point + Point::round(v * (distance / sqrt(lsqr))));
                    distance -= sqrt(lsqr);
                } else {
                    // Circular segment
                    float angle = Geometry::ArcWelder::arc_angle(prev_point.cast<float>(), point.cast<float>(), it->radius);
                    double len = std::abs(it->radius) * angle;
                    if (len > distance) {
                        // Rotate the segment end point in reverse towards the start point.
                        return std::make_optional<Point>(prev_point.rotated(- angle * (distance / len), Point::round(
                            Geometry::ArcWelder::arc_center(prev_point.cast<float>(), point.cast<float>(), it->radius, it->ccw()).cast<double>())));
                    }
                    distance -= len;
                }
                if (distance < 0)
                    return std::make_optional<Point>(point);
                prev_point = point;
            }
        }
    }
    // Failed.
    return {};
}

std::optional<Point> sample_path_point_at_distance_from_end(const ExtrusionPaths &paths, double distance)
{
    if (distance >= 0) {
        for (const ExtrusionPath& path : paths) {
            ArcPolyline polyline = path.as_polyline();
            auto it = polyline.get_arc().begin();
            auto end = polyline.get_arc().end();
            Point prev_point = it->point;
            for (++it; it != end; ++it) {
                Point point = it->point;
                if (it->linear()) {
                    // Linear segment
                    Vec2d  v = (point - prev_point).cast<double>();
                    double lsqr = v.squaredNorm();
                    if (lsqr > sqr(distance))
                        return std::make_optional<Point>(prev_point + Point::round(v * (distance / sqrt(lsqr))));
                    distance -= sqrt(lsqr);
                }
                else {
                    // Circular segment
                    float angle = Geometry::ArcWelder::arc_angle(prev_point.cast<float>(), point.cast<float>(), it->radius);
                    double len = std::abs(it->radius) * angle;
                    if (len > distance) {
                        // Rotate the segment end point in reverse towards the start point.
                        return std::make_optional<Point>(prev_point.rotated(-angle * (distance / len), Point::round(
                            Geometry::ArcWelder::arc_center(prev_point.cast<float>(), point.cast<float>(), it->radius, it->ccw()).cast<double>())));
                    }
                    distance -= len;
                }
                if (distance < 0)
                    return std::make_optional<Point>(point);
                prev_point = point;
            }
        }
    }
    // Failed.
    return {};
}

// Make a little move inwards before leaving loop after path was extruded,
// thus the current extruder position is at the end of a path and the path
// may not be closed in case the loop was clipped to hide a seam.
// note: prusaslicer method. I don't use it as the current algorithm is quite much more complex. Also couldn't find a way to trigger it with a test project in ps 2.8.
std::optional<Point> wipe_hide_seam(const ExtrusionPaths &paths, bool is_hole, double wipe_length)
{
    assert(! paths.empty());
    assert(paths.front().size() >= 2);
    assert(paths.back().size() >= 2);

    // Heuristics for estimating whether there is a chance that the wipe move will fit inside a small perimeter
    // or that the wipe move direction could be calculated with reasonable accuracy.
    if (longer_than(paths, 2.5 * wipe_length)) {
        // The print head will be moved away from path end inside the island.
        Point p_current = paths.back().last_point();//paths.back().path.back().point;
        Point p_next = paths.front().first_point();//paths.front().path.front().point;
        Point p_prev;
        {
            // Is the seam hiding gap large enough already?
            double l = wipe_length - (p_next - p_current).cast<double>().norm();
            if (l > 0) {
                // Not yet.
                std::optional<Point> n = sample_path_point_at_distance_from_start(paths, l);
                assert(n);
                if (!n) {
                    // Wipe move cannot be calculated, the loop is not long enough. This should not happen due to the
                    // longer_than() test above.
                    return {};
                }
            }
            if (std::optional<Point> p = sample_path_point_at_distance_from_end(paths, wipe_length); p)
                p_prev = *p;
            else
                // Wipe move cannot be calculated, the loop is not long enough. This should not happen due to the longer_than() test above.
                return {};
        }
        // Detect angle between last and first segment.
        // The side depends on the original winding order of the polygon (left for contours, right for holes).
        double angle_inside = angle_ccw(p_next - p_current, p_prev - p_current);
        assert(angle_inside >= -M_PI && angle_inside <= M_PI);
        // 3rd of this angle will be taken, thus make the angle monotonic before interpolation.
        if (is_hole) {
            if (angle_inside > 0)
                angle_inside -= 2.0 * M_PI;
        } else {
            if (angle_inside < 0)
                angle_inside += 2.0 * M_PI;
        }
        // Rotate the forward segment inside by 1/3 of the wedge angle.
        auto v_rotated = Eigen::Rotation2D(angle_inside) * (p_next - p_current).cast<double>().normalized();
        return std::make_optional<Point>(p_current + Point::round(v_rotated * wipe_length));
    }

    return {};
}

// Superslicer methods
//
//void Wipe::append(const Point &p)
//{
//    assert(this->path.empty() || !this->path.last_point().coincides_with_epsilon(p));
//    this->path.append(p);
//}
//
//void Wipe::append(const Polyline &poly)
//{
//    assert(!poly.empty());
//    if (!this->path.empty() && path.last_point().coincides_with_epsilon(poly.first_point())) {
//        int copy_start_idx = 0;
//        while (copy_start_idx < poly.size() && poly.points[copy_start_idx].distance_to(this->path.last_point()) < SCALED_EPSILON) {
//            copy_start_idx++;
//        }
//        if (copy_start_idx >= poly.size())
//            return;
//        assert(!this->path.last_point().coincides_with_epsilon(poly.points[copy_start_idx]));
//        this->path.append(poly.points.begin() + copy_start_idx, poly.points.end());
//    } else {
//        this->path.append(poly);
//    }
//}
//
//void Wipe::set(const Polyline &p)
//{
//    path = p;
//}
} // namespace Slic3r::GCode
