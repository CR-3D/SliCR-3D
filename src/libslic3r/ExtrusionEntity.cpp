#include "ExtrusionEntity.hpp"
#include "ExtrusionEntityCollection.hpp"
#include "ExPolygonCollection.hpp"
#include "ClipperUtils.hpp"
#include "Extruder.hpp"
#include "Flow.hpp"
#include <cmath>
#include <limits>
#include <sstream>
#include "Utils.hpp"

#define L(s) (s)

namespace Slic3r {

static const double slope_inner_outer_wall_gap = 0.4;

//// extrusion entity visitor
void ExtrusionVisitor::use(ExtrusionPath &path) { default_use(path); };
void ExtrusionVisitor::use(ExtrusionPath3D &path3D) { default_use(path3D); }
void ExtrusionVisitor::use(ExtrusionMultiPath &multipath) { default_use(multipath); }
void ExtrusionVisitor::use(ExtrusionMultiPath3D &multipath3D) { default_use(multipath3D); }
void ExtrusionVisitor::use(ExtrusionLoop &loop) { default_use(loop); }
void ExtrusionVisitor::use(ExtrusionEntityCollection &collection) { default_use(collection); }

void ExtrusionVisitorConst::use(const ExtrusionPath &path) { default_use(path); }
void ExtrusionVisitorConst::use(const ExtrusionPath3D &path3D) { default_use(path3D); }
void ExtrusionVisitorConst::use(const ExtrusionMultiPath &multipath) { default_use(multipath); }
void ExtrusionVisitorConst::use(const ExtrusionMultiPath3D &multipath3D) { default_use(multipath3D); }
void ExtrusionVisitorConst::use(const ExtrusionLoop &loop) { default_use(loop); }
void ExtrusionVisitorConst::use(const ExtrusionEntityCollection &collection) { default_use(collection); }
    
void
ExtrusionPath::intersect_expolygons(const ExPolygonCollection &collection, ExtrusionEntityCollection* retval) const
{
    this->_inflate_collection(intersection_pl(Polylines{ this->polyline.as_polyline() }, collection.expolygons), retval);
}

void ExtrusionPath::subtract_expolygons(const ExPolygonCollection &collection, ExtrusionEntityCollection* retval) const
{
    this->_inflate_collection(diff_pl(Polylines{ this->polyline.as_polyline() }, collection.expolygons), retval);
}

void ExtrusionPath::clip_end(coordf_t distance)
{
    this->polyline.clip_end(distance);
}

void ExtrusionPath::simplify(coordf_t tolerance, bool with_fitting_arc, double fitting_arc_tolerance)
{
    this->polyline.simplify(tolerance, with_fitting_arc, fitting_arc_tolerance);
}

void ExtrusionPath3D::simplify(coordf_t tolerance, bool with_fitting_arc, double fitting_arc_tolerance)
{
    //TODO: simplify but only for sub-path with same zheight.
    //if (with_fitting_arc) {
    //    this->polyline.simplify(tolerance, with_fitting_arc, fitting_arc_tolerance);
    //}
}

double ExtrusionPath::length() const
{
    return this->polyline.length();
}

void ExtrusionPath::_inflate_collection(const Polylines &polylines, ExtrusionEntityCollection* collection) const
{
    ExtrusionEntitiesPtr to_add;
    for (const Polyline &polyline : polylines)
        to_add.emplace_back(new ExtrusionPath(PolylineOrArc{ polyline }, *this));
    collection->append(std::move(to_add));
}

void ExtrusionPath::polygons_covered_by_width(Polygons &out, const float scaled_epsilon) const
{
    polygons_append(out, offset(this->polyline.as_polyline(), double(scale_(this->width/2)) + scaled_epsilon));
}

void ExtrusionPath::polygons_covered_by_spacing(Polygons &out, const float spacing_ratio, const float scaled_epsilon) const
{
    // Instantiating the Flow class to get the line spacing.
    // Don't know the nozzle diameter, setting to zero. It shall not matter it shall be optimized out by the compiler.
    bool bridge = is_bridge(this->role()) || (this->width * 4 < this->height);
    assert(! bridge || this->width == this->height);
    //TODO: check BRIDGE_FLOW here
    auto flow = bridge 
        ? Flow::bridging_flow(this->width, 0.f) 
        : Flow::new_from_width(this->width, 0.f, this->height, spacing_ratio);
    polygons_append(out, offset(this->polyline.as_polyline(), 0.5f * float(flow.scaled_spacing()) + scaled_epsilon));
}

bool ExtrusionLoop::make_clockwise()
{
    bool was_ccw = this->polygon().is_counter_clockwise();
    if (was_ccw) this->reverse();
    return was_ccw;
}

bool ExtrusionLoop::make_counter_clockwise()
{
    bool was_cw = this->polygon().is_clockwise();
    if (was_cw) this->reverse();
    return was_cw;
}

void ExtrusionLoop::reverse()
{
    for (ExtrusionPath &path : this->paths)
        path.reverse();
    std::reverse(this->paths.begin(), this->paths.end());
}

Polygon ExtrusionLoop::polygon() const
{
    Polygon polygon;
    for (const ExtrusionPath &path : this->paths) {
        // for each polyline, append all points except the last one (because it coincides with the first one of the next polyline)
        polygon.points.insert(polygon.points.end(), path.polyline.get_points().begin(), path.polyline.get_points().end()-1);
    }
    return polygon;
}

PolylineOrArc ExtrusionLoop::as_polyline() const {
    PolylineOrArc polyline;
    for (const ExtrusionPath& path : this->paths) {
        polyline.append(path.as_polyline());
    }
    return polyline;
}

double ExtrusionLoop::length() const
{
    double len = 0;
    for (const ExtrusionPath &path : this->paths)
        len += path.polyline.length();
    return len;
}

ExtrusionRole ExtrusionLoop::role() const
{
    if (this->paths.empty())
        return erNone;
    ExtrusionRole role = this->paths.front().role();
    for (const ExtrusionPath &path : this->paths)
        if (role != path.role()) {
            return erMixed;
        }
    return role;
}

void ExtrusionLoop::clip_end(double distance, ExtrusionPaths *paths) const
{
    *paths = this->paths;

    while (distance > 0 && !paths->empty()) {
        ExtrusionPath &last = paths->back();
        double         len  = last.length();
        if (len <= distance) {
            paths->pop_back();
            distance -= len;
        } else {
            last.polyline.clip_end(distance);
            break;
        }
    }
}

bool ExtrusionLoop::split_at_vertex(const Point &point, const double scaled_epsilon)
{
    for (ExtrusionPaths::iterator path = this->paths.begin(); path != this->paths.end(); ++path) {
        if (int idx = path->polyline.find_point(point, scaled_epsilon); idx != -1) {
            if (this->paths.size() == 1) {
                // just change the order of points
                PolylineOrArc p1, p2;
                path->polyline.split_at_index(idx, &p1, &p2);
                if (p1.is_valid() && p2.is_valid()) {
                    p2.append(std::move(p1));
                    path->polyline.swap(p2); // swap points & fitting result
                }
            } else {
                // new paths list starts with the second half of current path
                ExtrusionPaths new_paths;
                PolylineOrArc p1, p2;
                path->polyline.split_at_index(idx, &p1, &p2);
                new_paths.reserve(this->paths.size() + 1);
                {
                    ExtrusionPath p = *path;
                    p.polyline.swap(p2);
                    if (p.polyline.is_valid()) new_paths.push_back(p);
                }

                // then we add all paths until the end of current path list
                new_paths.insert(new_paths.end(), path + 1, this->paths.end());  // not including this path

                // then we add all paths since the beginning of current list up to the previous one
                new_paths.insert(new_paths.end(), this->paths.begin(), path);  // not including this path

                // finally we add the first half of current path
                {
                    ExtrusionPath p = *path;
                    p.polyline.swap(p1);
                    if (p.polyline.is_valid()) new_paths.push_back(p);
                }
                // we can now override the old path list with the new one and stop looping
                this->paths = std::move(new_paths);
            }
            return true;
        }
    }
    // The point was not found.
    return false;
}

ExtrusionLoop::ClosestPathPoint ExtrusionLoop::get_closest_path_and_point(const Point &point, bool prefer_non_overhang) const
{
    // Find the closest path and closest point belonging to that path. Avoid overhangs, if asked for.
    ClosestPathPoint out { 0, 0 };
    double           min2 = std::numeric_limits<double>::max();
    ClosestPathPoint best_non_overhang { 0, 0 };
    double           min2_non_overhang = std::numeric_limits<double>::max();
    for (const ExtrusionPath& path : this->paths) {
        std::pair<int, Point> foot_pt_ = foot_pt(path.polyline.get_points(), point);
        double d2 = (foot_pt_.second - point).cast<double>().squaredNorm();
        if (d2 < min2) {
            out.foot_pt     = foot_pt_.second;
            out.path_idx    = &path - &this->paths.front();
            out.segment_idx = foot_pt_.first;
            min2            = d2;
            }
        if (prefer_non_overhang && !is_bridge(path.role()) && d2 < min2_non_overhang) {
            best_non_overhang.foot_pt     = foot_pt_.second;
            best_non_overhang.path_idx    = &path - &this->paths.front();
            best_non_overhang.segment_idx = foot_pt_.first;
            min2_non_overhang             = d2;
        }
    }
    if (prefer_non_overhang && min2_non_overhang != std::numeric_limits<double>::max()) {
        // Only apply the non-overhang point if there is one.
        out = best_non_overhang;
    }
    return out;
}

ExtrusionLoopSloped::ExtrusionLoopSloped(ExtrusionPaths&   original_paths,
                                         double            seam_gap,
                                         double            slope_min_length,
                                         double            slope_max_segment_length,
                                         double            start_slope_ratio,
                                         ExtrusionLoopRole role)
    : ExtrusionLoop(role)
{
    // create slopes
    const auto add_slop = [this, slope_max_segment_length, seam_gap](const ExtrusionPath& path, const Polyline& poly,
                                                                          double ratio_begin, double ratio_end) {
        if (poly.empty()) {
            return;
        }

        // Ensure `slope_max_segment_length`
        Polyline detailed_poly;
        {
            detailed_poly.append(poly.first_point());

            // Recursively split the line into half until no longer than `slope_max_segment_length`
            const std::function<void(const Line&)> handle_line = [slope_max_segment_length, &detailed_poly, &handle_line](const Line& line) {
                if (line.length() <= slope_max_segment_length) {
                    detailed_poly.append(line.b);
                } else {
                    // Then process left half
                    handle_line({line.a, line.midpoint()});
                    // Then process right half
                    handle_line({line.midpoint(), line.b});
                }
            };

            for (const auto& l : poly.lines()) {
                handle_line(l);
            }
        }

        starts.emplace_back(detailed_poly, path, ExtrusionPathSloped::Slope{ratio_begin, ratio_begin},
                                    ExtrusionPathSloped::Slope{ratio_end, ratio_end});

        if (is_approx(ratio_end, 1.) && seam_gap > 0) {
            // Remove the segments that has no extrusion
            const auto seg_length = detailed_poly.length();
            if (seg_length > seam_gap) {
                // Split the segment and remove the last `seam_gap` bit
                const Polyline orig = detailed_poly;
                Polyline       tmp;
                orig.split_at_length(seg_length - seam_gap, &detailed_poly, &tmp);

                ratio_end = lerp(ratio_begin, ratio_end, (seg_length - seam_gap) / seg_length);
                assert(1. - ratio_end > EPSILON);
            } else {
                // Remove the entire segment
                detailed_poly.clear();
            }
        }
        if (!detailed_poly.empty()) {
            ends.emplace_back(detailed_poly, path, ExtrusionPathSloped::Slope{1., 1. - ratio_begin},
                                      ExtrusionPathSloped::Slope{1., 1. - ratio_end});
        }
    };

    double remaining_length = slope_min_length;

    ExtrusionPaths::iterator path        = original_paths.begin();
    double                   start_ratio = start_slope_ratio;
    for (; path != original_paths.end() && remaining_length > 0; ++path) {
        const double path_len = unscale_(path->length());
        if (path_len > remaining_length) {
            // Split current path into slope and non-slope part
            Polyline slope_path;
            Polyline flat_path;
            path->polyline.split_at_length(scale_(remaining_length), &slope_path, &flat_path);

            add_slop(*path, slope_path, start_ratio, 1);
            start_ratio = 1;

            paths.emplace_back(std::move(flat_path), *path);
            remaining_length = 0;
        } else {
            remaining_length -= path_len;
            const double end_ratio = lerp(1.0, start_slope_ratio, remaining_length / slope_min_length);
            add_slop(*path, path->polyline, start_ratio, end_ratio);
            start_ratio = end_ratio;
        }
    }
    assert(remaining_length <= 0);
    assert(start_ratio == 1.);

    // Put remaining flat paths
    paths.insert(paths.end(), path, original_paths.end());
}

std::vector<const ExtrusionPath*> ExtrusionLoopSloped::get_all_paths() const {
    std::vector<const ExtrusionPath*> r;
    r.reserve(starts.size() + paths.size() + ends.size());
    for (const auto& p : starts) {
        r.push_back(&p);
    }
    for (const auto& p : paths) {
        r.push_back(&p);
    }
    for (const auto& p : ends) {
        r.push_back(&p);
    }

    return r;
}

void ExtrusionLoopSloped::clip_slope(double distance, bool inter_perimeter)
{

    this->clip_end(distance);
    this->clip_front(distance*2);
}

void ExtrusionLoopSloped::clip_end(const double distance)
{
    double clip_dist = distance;
    std::vector<ExtrusionPathSloped> &ends_slope = this->ends;
    while (clip_dist > 0 && !ends_slope.empty()) {
        ExtrusionPathSloped &last_path = ends_slope.back();
        double len = last_path.length();
        if (len <= clip_dist) {
            ends_slope.pop_back();
            clip_dist -= len;
        } else {
            last_path.polyline.clip_end(clip_dist);
            break;
        }
    }
}

void ExtrusionLoopSloped::clip_front(const double distance)
{
    double clip_dist = distance;
    if (this->role() == erPerimeter)
        clip_dist = scale_(this->slope_path_length()) * slope_inner_outer_wall_gap;

    std::vector<ExtrusionPathSloped> &start_slope = this->starts;

    Polyline front_inward;
    while (distance > 0 && !start_slope.empty()) {
        ExtrusionPathSloped &first_path = start_slope.front();
        double len = first_path.length();
        if (len <= clip_dist) {
            start_slope.erase(start_slope.begin());
            clip_dist -= len;
        } else {
            first_path.polyline.reverse();
            first_path.polyline.clip_end(clip_dist);
            first_path.polyline.reverse();
            break;
        }
    }
}

double ExtrusionLoopSloped::slope_path_length() {
    double total_length = 0.0;
    for (ExtrusionPathSloped start_ep : this->starts) {
        total_length += unscale_(start_ep.length());
    }
    return total_length;
}

// Orca: This function is used to check if the loop is smooth(continuous) or not. 
// TODO: the main logic is largly copied from the calculate_polygon_angles_at_vertices function in SeamPlacer file. Need to refactor the code in the future.
bool ExtrusionLoop::is_smooth(double angle_threshold, double min_arm_length) const
{
    // go through all the points in the loop and check if the angle between two segments(AB and BC) is less than the threshold
    size_t idx_prev = 0;
    size_t idx_curr = 0;
    size_t idx_next = 0;

    float distance_to_prev = 0;
    float distance_to_next = 0;

    const auto _polygon = polygon();
    const Points& points = _polygon.points;

    std::vector<float> lengths{};
    for (size_t point_idx = 0; point_idx < points.size() - 1; ++point_idx) {
        lengths.push_back((unscale(points[point_idx]) - unscale(points[point_idx + 1])).norm());
    }
    lengths.push_back(std::max((unscale(points[0]) - unscale(points[points.size() - 1])).norm(), 0.1));

    // push idx_prev far enough back as initialization
    while (distance_to_prev < min_arm_length) {
        idx_prev = Slic3r::prev_idx_modulo(idx_prev, points.size());
        distance_to_prev += lengths[idx_prev];
    }

    for (size_t _i = 0; _i < points.size(); ++_i) {
        // pull idx_prev to current as much as possible, while respecting the min_arm_length
        while (distance_to_prev - lengths[idx_prev] > min_arm_length) {
            distance_to_prev -= lengths[idx_prev];
            idx_prev = Slic3r::next_idx_modulo(idx_prev, points.size());
        }

        // push idx_next forward as far as needed
        while (distance_to_next < min_arm_length) {
            distance_to_next += lengths[idx_next];
            idx_next = Slic3r::next_idx_modulo(idx_next, points.size());
        }

        // Calculate angle between idx_prev, idx_curr, idx_next.
        const Point& p0 = points[idx_prev];
        const Point& p1 = points[idx_curr];
        const Point& p2 = points[idx_next];
        const auto a = angle(p0 - p1, p2 - p1);
        if (a > 0 ? a < angle_threshold : a > -angle_threshold) {
            return false;
        }

        // increase idx_curr by one
        float curr_distance = lengths[idx_curr];
        idx_curr++;
        distance_to_prev += curr_distance;
        distance_to_next -= curr_distance;
    }

    return true;
}


// Splitting an extrusion loop, possibly made of multiple segments, some of the segments may be bridging.
void ExtrusionLoop::split_at(const Point &point, bool prefer_non_overhang, const double scaled_epsilon)
{
    if (this->paths.empty())
        return;
    
    auto [path_idx, segment_idx, p] = get_closest_path_and_point(point, prefer_non_overhang);
    
    // Snap p to start or end of segment_idx if closer than scaled_epsilon.
    {
        const Point *p1 = this->paths[path_idx].polyline.get_points().data() + segment_idx;
        const Point *p2 = p1;
        ++ p2;
        double d2_1 = (point - *p1).cast<double>().squaredNorm();
        double d2_2 = (point - *p2).cast<double>().squaredNorm();
        const double thr2 = scaled_epsilon * scaled_epsilon;
        if (d2_1 < d2_2) {
            if (d2_1 < thr2)
                p = *p1;
        } else {
            if (d2_2 < thr2) 
                p = *p2;
        }
    }
    
    // now split path_idx in two parts
    const ExtrusionPath &path = this->paths[path_idx];
    ExtrusionPath p1(path.role(), path.mm3_per_mm, path.width, path.height, can_reverse());
    ExtrusionPath p2(path.role(), path.mm3_per_mm, path.width, path.height, can_reverse());
    path.polyline.split_at(p, &p1.polyline, &p2.polyline);
    
    if (this->paths.size() == 1) {
        if (!p1.polyline.is_valid()) {
            this->paths.front().polyline.swap(p2.polyline);
        } else if (!p2.polyline.is_valid()) {
            this->paths.front().polyline.swap(p1.polyline);
        } else {
            p2.polyline.append(std::move(p1.polyline));
            this->paths.front().polyline.swap(p2.polyline);
        }
    } else {
        // install the two paths
        this->paths.erase(this->paths.begin() + path_idx);
        if (p2.polyline.is_valid() && p2.polyline.length() > 0) this->paths.insert(this->paths.begin() + path_idx, p2);
        if (p1.polyline.is_valid() && p1.polyline.length() > 0) this->paths.insert(this->paths.begin() + path_idx, p1);
    }
    
    // split at the new vertex
    this->split_at_vertex(p, 0.);
}

ExtrusionPaths clip_end(ExtrusionPaths& paths, coordf_t distance)
{
    ExtrusionPaths removed;
    
    while (distance > 0 && !paths.empty()) {
        ExtrusionPath& last = paths.back();
        removed.push_back(last);
        double len = last.length();
        if (len <= distance) {
            paths.pop_back();
            distance -= len;
        } else {
            last.polyline.clip_end(distance);
            removed.back().polyline.clip_start(removed.back().polyline.length() - distance);
            break;
        }
    }
    std::reverse(removed.begin(), removed.end());
    return removed;
}

bool ExtrusionLoop::has_overhang_point(const Point &point) const
{
    for (const ExtrusionPath &path : this->paths) {
        int pos = path.polyline.find_point(point);
        if (pos != -1) {
            // point belongs to this path
            // we consider it overhang only if it's not an endpoint
            return (is_bridge(path.role()) && pos > 0 && pos != (int)(path.polyline.size())-1);
        }
    }
    return false;
}

void ExtrusionLoop::polygons_covered_by_width(Polygons &out, const float scaled_epsilon) const
{
    for (const ExtrusionPath &path : this->paths)
        path.polygons_covered_by_width(out, scaled_epsilon);
}

void ExtrusionLoop::polygons_covered_by_spacing(Polygons &out, const float spacing_ratio, const float scaled_epsilon) const
{
    for (const ExtrusionPath &path : this->paths)
        path.polygons_covered_by_spacing(out, spacing_ratio, scaled_epsilon);
}

std::string ExtrusionEntity::role_to_string(ExtrusionRole role)
{
    switch (role) {
        case erNone                         : return L("Unknown");
        case erPerimeter                    : return L("Internal perimeter");
        case erExternalPerimeter            : return L("External perimeter");
        case erOverhangPerimeter            : return L("Overhang perimeter");
        case erInternalInfill               : return L("Internal infill");
        case erSolidInfill                  : return L("Solid infill");
        case erTopSolidInfill               : return L("Top solid infill");
        case erIroning                      : return L("Ironing");
        case erBridgeInfill                 : return L("Bridge infill");
        case erInternalBridgeInfill         : return L("Internal bridge infill");
        case erThinWall                     : return L("Thin wall");
        case erGapFill                      : return L("Gap fill");
        case erSkirt                        : return L("Skirt");
        case erSupportMaterial              : return L("Support material");
        case erSupportMaterialInterface     : return L("Support material interface");
        case erWipeTower                    : return L("Wipe tower");
        case erMilling                      : return L("Mill");
        case erCustom                       : return L("Custom");
        case erMixed                        : return L("Mixed");
        case erTravel                       : return L("Travel");
        default                             : assert(false);
    }

    return "";
}


ExtrusionRole ExtrusionEntity::string_to_role(const std::string_view role)
{
    if (role == L("Perimeter") || role == L("Internal perimeter"))
        return erPerimeter;
    else if (role == L("External perimeter"))
        return erExternalPerimeter;
    else if (role == L("Overhang perimeter"))
        return erOverhangPerimeter;
    else if (role == L("Internal infill"))
        return erInternalInfill;
    else if (role == L("Solid infill"))
        return erSolidInfill;
    else if (role == L("Top solid infill"))
        return erTopSolidInfill;
    else if (role == L("Ironing"))
        return erIroning;
    else if (role == L("Bridge infill"))
        return erBridgeInfill;
    else if (role == L("Internal bridge infill"))
        return erInternalBridgeInfill;
    else if (role == L("Thin wall"))
        return erThinWall;
    else if (role == L("Gap fill"))
        return erGapFill;
    else if (role == L("Skirt") || role == L("Skirt/Brim")) // "Skirt" is for backward compatibility with 2.3.1 and earlier
        return erSkirt;
    else if (role == L("Support material"))
        return erSupportMaterial;
    else if (role == L("Support material interface"))
        return erSupportMaterialInterface;
    else if (role == L("Wipe tower"))
        return erWipeTower;
    else if (role == L("Mill"))
        return erMilling;
    else if (role == L("Custom"))
        return erCustom;
    else if (role == L("Mixed"))
        return erMixed;
    else
        return erNone;
}


std::string role_to_code(ExtrusionRole role)
{
    switch (role) {
        case erNone                         : return L("None");
        case erPerimeter                    : return L("IPeri");
        case erExternalPerimeter            : return L("EPeri");
        case erOverhangPerimeter            : return L("OPeri");
        case erInternalInfill               : return L("IFill");
        case erSolidInfill                  : return L("SFill");
        case erTopSolidInfill               : return L("TFill");
        case erIroning                      : return L("Iron");
        case erBridgeInfill                 : return L("EBridge");
        case erInternalBridgeInfill         : return L("IBridge");
        case erThinWall                     : return L("ThinW");
        case erGapFill                      : return L("GFill");
        case erSkirt                        : return L("Skirt");
        case erSupportMaterial              : return L("Supp");
        case erSupportMaterialInterface     : return L("SuppI");
        case erWipeTower                    : return L("WTower");
        case erMilling                      : return L("Mill");
        case erCustom                       : return L("Custom");
        case erMixed                        : return L("Mixed");
        case erTravel                       : return L("Travel");
        default                             : assert(false);
    }

    return "";
}


std::string looprole_to_code(ExtrusionLoopRole looprole)
{
    std::string code;
    if(elrDefault == (looprole & elrDefault))
        code += std::string("D");
    if(elrInternal == (looprole & elrInternal))
        code += std::string("Int");
    if(elrSkirt == (looprole & elrSkirt))
        code += std::string("Skirt");
    if(elrHole == (looprole & elrHole))
        code += std::string("Hole");
    if(elrVase == (looprole & elrVase))
        code += std::string("Vase");
    if(elrFirstLoop == (looprole & elrFirstLoop))
        code += std::string("First");

    return code;
}

void ExtrusionPrinter::use(const ExtrusionPath &path) { 
    ss << (json?"\"":"") << "ExtrusionPath" << (path.can_reverse()?"":"Oriented") << (json?"_":":") << role_to_code(path.role()) << (json?"\":":"") << "[";
    for (int i = 0; i < path.polyline.size(); i++) {
        if (i != 0) ss << ",";
        double x = (mult * (path.polyline.get_points()[i].x()));
        double y = (mult * (path.polyline.get_points()[i].y()));
        ss << std::fixed << "["<<(trunc>0?(int(x*trunc))/double(trunc):x) << "," << (trunc>0?(int(y*trunc))/double(trunc):y) <<"]";
    }
    ss << "]";
}
void ExtrusionPrinter::use(const ExtrusionPath3D &path3D) {
    ss << (json?"\"":"") << "ExtrusionPath3D" << (path3D.can_reverse()?"":"Oriented") << (json?"_":":") << role_to_code(path3D.role()) << (json?"\":":"") << "[";
    for (int i = 0; i < path3D.polyline.size();i++){
        if (i != 0) ss << ",";
        double x = (mult * (path3D.polyline.get_points()[i].x()));
        double y = (mult * (path3D.polyline.get_points()[i].y()));
        double z = (path3D.z_offsets.size() > i ? mult * (path3D.z_offsets[i]) : -1);
        ss << std::fixed << "[" << (trunc>0?(int(x*trunc))/double(trunc):x) << "," << (trunc>0?(int(y*trunc))/double(trunc):y) << "," << (trunc>0?(int(z*trunc))/double(trunc):z) << "]";
    }
    ss << "]";
}
void ExtrusionPrinter::use(const ExtrusionMultiPath &multipath) {
    ss << (json?"\"":"") << "ExtrusionMultiPath" << (multipath.can_reverse()?"":"Oriented") << (json?"_":":") << role_to_code(multipath.role()) << (json?"\":":"") << "{";
    for (int i = 0; i < multipath.paths.size(); i++) {
        if (i != 0) ss << ",";
        multipath.paths[i].visit(*this);
    }
    ss << "}";
}
void ExtrusionPrinter::use(const ExtrusionMultiPath3D &multipath3D) {
    ss << (json?"\"":"") << "multipath3D" << (multipath3D.can_reverse()?"":"Oriented") << (json?"_":":") << role_to_code(multipath3D.role()) << (json?"\":":"") << "{";
    for (int i = 0; i < multipath3D.paths.size(); i++) {
        if (i != 0) ss << ",";
        multipath3D.paths[i].visit(*this);
    }
    ss << "}";
}
void ExtrusionPrinter::use(const ExtrusionLoop &loop) { 
    ss << (json?"\"":"") << "ExtrusionLoop" << (json?"_":":") << role_to_code(loop.role())<<"_" << looprole_to_code(loop.loop_role()) << (json?"\":":"") << "{";
    if(!loop.can_reverse()) ss << (json?"\"":"") << "oriented" << (json?"\":":"=") << "true,";
    for (int i = 0; i < loop.paths.size(); i++) {
        if (i != 0) ss << ",";
        loop.paths[i].visit(*this);
    }
    ss << "}";
}
void ExtrusionPrinter::use(const ExtrusionEntityCollection &collection) {
    ss << (json?"\"":"") << "ExtrusionEntityCollection" << (json?"_":":") << role_to_code(collection.role()) << (json?"\":":"") << "{";
    if(!collection.can_sort()) ss << (json?"\"":"") << "no_sort" << (json?"\":":"=") << "true,";
    if(!collection.can_reverse()) ss << (json?"\"":"") << "oriented" << (json?"\":":"=") << "true,";
    for (int i = 0; i < collection.entities().size(); i++) {
        if (i != 0) ss << ",";
        collection.entities()[i]->visit(*this);
    }
    ss << "}";
}


void ExtrusionLength::default_use(const ExtrusionEntity &entity) { dist += entity.length(); };
void ExtrusionLength::use(const ExtrusionEntityCollection &collection)
{
    for (int i = 0; i < collection.entities().size(); i++) { collection.entities()[i]->visit(*this); }
}

double ExtrusionVolume::get(const ExtrusionEntityCollection &coll)
{
    for (const ExtrusionEntity *entity : coll.entities()) entity->visit(*this);
    return volume;
}

void ExtrusionModifyFlow::set(ExtrusionEntityCollection &coll)
{
    for (ExtrusionEntity *entity : coll.entities()) entity->visit(*this);
}

void ExtrusionVisitorRecursiveConst::use(const ExtrusionMultiPath &multipath)
{
    for (const ExtrusionPath &path : multipath.paths) { path.visit(*this); }
}
void ExtrusionVisitorRecursiveConst::use(const ExtrusionMultiPath3D &multipath3D)
{
    for (const ExtrusionPath3D &path3D : multipath3D.paths) { path3D.visit(*this); }
}
void ExtrusionVisitorRecursiveConst::use(const ExtrusionLoop &loop)
{
    for (const ExtrusionPath &path : loop.paths) { path.visit(*this); }
}
void ExtrusionVisitorRecursiveConst::use(const ExtrusionEntityCollection &collection)
{
    for (const ExtrusionEntity *entity : collection.entities()) { entity->visit(*this); }
}
void ExtrusionVisitorRecursive::use(ExtrusionMultiPath &multipath)
{
    for (ExtrusionPath &path : multipath.paths) { path.visit(*this); }
}
void ExtrusionVisitorRecursive::use(ExtrusionMultiPath3D &multipath3D)
{
    for (ExtrusionPath3D &path3D : multipath3D.paths) { path3D.visit(*this); }
}
void ExtrusionVisitorRecursive::use(ExtrusionLoop &loop)
{
    for (ExtrusionPath &path : loop.paths) { path.visit(*this); }
}
void ExtrusionVisitorRecursive::use(ExtrusionEntityCollection &collection)
{
    for (ExtrusionEntity *entity : collection.entities()) { entity->visit(*this); }
}

void HasRoleVisitor::use(const ExtrusionMultiPath &multipath)
{
    for (const ExtrusionPath &path : multipath.paths) {
        path.visit(*this);
        if (found)
            return;
    }
}
void HasRoleVisitor::use(const ExtrusionMultiPath3D &multipath3D)
{
    for (const ExtrusionPath3D &path3D : multipath3D.paths) {
        path3D.visit(*this);
        if (found)
            return;
    }
}
void HasRoleVisitor::use(const ExtrusionLoop &loop)
{
    for (const ExtrusionPath &path : loop.paths) {
        path.visit(*this);
        if (found)
            return;
    }
}
void HasRoleVisitor::use(const ExtrusionEntityCollection &collection)
{
    for (const ExtrusionEntity *entity : collection.entities()) {
        entity->visit(*this);
        if (found)
            return;
    }
}
bool HasRoleVisitor::search(const ExtrusionEntity &entity, HasRoleVisitor &&visitor)
{
    entity.visit(visitor);
    return visitor.found;
}
bool HasRoleVisitor::search(const ExtrusionEntitiesPtr &entities, HasRoleVisitor &&visitor)
{
    for (ExtrusionEntity *ptr : entities) {
        ptr->visit(visitor);
        if (visitor.found)
            return true;
    }
    return visitor.found;
}


//class ExtrusionTreeVisitor : ExtrusionVisitor {
//public:
//    //virtual void use(ExtrusionEntity &entity) { assert(false); };
//    virtual void use(ExtrusionPath &path) override { const ExtrusionPath &constpath = path;  use(constpath); };
//    virtual void use(ExtrusionPath3D &path3D) override { const ExtrusionPath3D &constpath3D = path3D;  use(constpath3D); };
//    virtual void use(ExtrusionMultiPath &multipath) override { const ExtrusionMultiPath &constmultipath = multipath;  use(constmultipath); };
//    virtual void use(ExtrusionMultiPath3D &multipath3D) override { const ExtrusionMultiPath3D &constmultipath3D = multipath3D;  use(constmultipath3D); };
//    virtual void use(ExtrusionLoop &loop) override { const ExtrusionLoop &constloop = loop;  use(constloop); };
//    virtual void use(ExtrusionEntityCollection &collection) { const ExtrusionEntityCollection &constcollection = collection;  use(constcollection); };
//    virtual void use(const ExtrusionPath &path) override { assert(false); };
//    virtual void use(const ExtrusionPath3D &path3D) override { assert(false); };
//    virtual void use(const ExtrusionMultiPath &multipath) override { assert(false); };
//    virtual void use(const ExtrusionMultiPath3D &multipath3D) { assert(false); };
//    virtual void use(const ExtrusionLoop &loop) override { assert(false); };
//    virtual void use(const ExtrusionEntityCollection &collection) { assert(false); };
//    virtual void use_default(ExtrusionEntity &entity) { const ExtrusionEntity &constentity = entity;  use_default(constentity); };
//    virtual void use_default(const ExtrusionEntity &entity) {};
//
//};

}
