///|/ Copyright (c) Prusa Research 2020 - 2023 Vojtěch Bubník @bubnikv, Lukáš Matěna @lukasmatena, Pavel Mikuš @Godrak
///|/ Copyright (c) SuperSlicer 2023 Remi Durand @supermerill
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "SeamPlacer.hpp"

#include "Color.hpp"
#include "Polygon.hpp"
#include "PrintConfig.hpp"
#include "tbb/parallel_for.h"
#include "tbb/blocked_range.h"
#include "tbb/parallel_reduce.h"
#include <boost/log/trivial.hpp>
#include <algorithm>
#include <queue>
#include <random>
#include <tuple>

#include "libslic3r/AABBTreeLines.hpp"
#include "libslic3r/KDTreeIndirect.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"

#include "libslic3r/Geometry/Curves.hpp"
#include "libslic3r/ShortEdgeCollapse.hpp"
#include "libslic3r/TriangleSetSampling.hpp"

#include "libslic3r/Utils.hpp"

#include <atomic>

//#define DEBUG_FILES

#ifdef DEBUG_FILES
#include <boost/nowide/cstdio.hpp>
#include <SVG.hpp>
#endif

namespace Slic3r {

namespace SeamPlacerImpl {

template<typename T> int sgn(T val) { return int(T(0) < val) - int(val < T(0)); }

// base function: ((e^(((1)/(x^(2)+1)))-1)/(e-1))
// checkout e.g. here: https://www.geogebra.org/calculator
float gauss(float value, float mean_x_coord, float mean_value, float falloff_speed) {
    float shifted = value - mean_x_coord;
    float denominator = falloff_speed * shifted * shifted + 1.0f;
    float exponent = 1.0f / denominator;
    return mean_value * (std::exp(exponent) - 1.0f) / (std::exp(1.0f) - 1.0f);
}

float compute_angle_penalty(float ccw_angle) {
    // This function is used:
    // ((ℯ^(((1)/(x^(2)*3+1)))-1)/(ℯ-1))*1+((1)/(2+ℯ^(-x)))
    // looks scary, but it is gaussian combined with sigmoid,
    // so that concave points have much smaller penalty over convex ones
    // https://github.com/prusa3d/PrusaSlicer/tree/master/doc/seam_placement/corner_penalty_function.png
    return gauss(ccw_angle, 0.0f, 1.0f, 3.0f) + 1.0f / (2 + std::exp(-ccw_angle));
}

/// Coordinate frame
class Frame
{
public:
    Frame() {
        mX = Vec3f(1, 0, 0);
        mY = Vec3f(0, 1, 0);
        mZ = Vec3f(0, 0, 1);
    }

    Frame(const Vec3f &x, const Vec3f &y, const Vec3f &z) : mX(x), mY(y), mZ(z) {}

    void set_from_z(const Vec3f &z) {
        mZ = z.normalized();
        Vec3f tmpZ = mZ;
        Vec3f tmpX = (std::abs(tmpZ.x()) > 0.99f) ? Vec3f(0, 1, 0) : Vec3f(1, 0, 0);
        mY = (tmpZ.cross(tmpX)).normalized();
        mX = mY.cross(tmpZ);
    }

    Vec3f to_world(const Vec3f &a) const { return a.x() * mX + a.y() * mY + a.z() * mZ; }

    Vec3f to_local(const Vec3f &a) const { return Vec3f(mX.dot(a), mY.dot(a), mZ.dot(a)); }

    const Vec3f &binormal() const { return mX; }

    const Vec3f &tangent() const { return mY; }

    const Vec3f &normal() const { return mZ; }

private:
    Vec3f mX, mY, mZ;
};

Vec3f sample_sphere_uniform(const Vec2f &samples) {
    float term1 = 2.0f * float(PI) * samples.x();
    float term2 = 2.0f * sqrt(samples.y() - samples.y() * samples.y());
    return {cos(term1) * term2, sin(term1) * term2, 1.0f - 2.0f * samples.y()};
}

Vec3f sample_hemisphere_uniform(const Vec2f &samples) {
    float term1 = 2.0f * float(PI) * samples.x();
    float term2 = 2.0f * sqrt(samples.y() - samples.y() * samples.y());
    return {cos(term1) * term2, sin(term1) * term2, abs(1.0f - 2.0f * samples.y())};
}

Vec3f sample_power_cosine_hemisphere(const Vec2f &samples, float power) {
    float term1 = 2.f * float(PI) * samples.x();
    float term2 = pow(samples.y(), 1.f / (power + 1.f));
    float term3 = sqrt(1.f - term2 * term2);

    return Vec3f(cos(term1) * term3, sin(term1) * term3, term2);
}

// raycast evrything and store the weight, or set everything to 1 if 'deactivate'
std::vector<float> raycast_visibility(const AABBTreeIndirect::Tree<3, float> &raycasting_tree,
                                      const indexed_triangle_set &triangles,
                                      const TriangleSetSamples &samples,
                                      size_t negative_volumes_start_index,
                                      bool deactivate) {
    BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: raycast visibility of " << samples.positions.size() << " samples over "
                             << triangles.indices.size() << " triangles: end";

    // prepare uniform samples of a hemisphere
    float step_size = 1.0f / SeamPlacer::sqr_rays_per_sample_point;
    std::vector<Vec3f> precomputed_sample_directions(SeamPlacer::sqr_rays_per_sample_point *
                                                     SeamPlacer::sqr_rays_per_sample_point);
    for (size_t x_idx = 0; x_idx < SeamPlacer::sqr_rays_per_sample_point; ++x_idx) {
        float sample_x = x_idx * step_size + step_size / 2.0;
        for (size_t y_idx = 0; y_idx < SeamPlacer::sqr_rays_per_sample_point; ++y_idx) {
            size_t dir_index = x_idx * SeamPlacer::sqr_rays_per_sample_point + y_idx;
            float sample_y = y_idx * step_size + step_size / 2.0;
            precomputed_sample_directions[dir_index] = sample_hemisphere_uniform({sample_x, sample_y});
        }
    }

    bool model_contains_negative_parts = negative_volumes_start_index < triangles.indices.size();

    std::vector<float> result(samples.positions.size());
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, result.size()),
        [&triangles, &precomputed_sample_directions, model_contains_negative_parts, negative_volumes_start_index,
         &raycasting_tree, &result, &samples, deactivate](tbb::blocked_range<size_t> r) {
            // Maintaining hits memory outside of the loop, so it does not have to be reallocated for each query.
            std::vector<igl::Hit> hits;
            for (size_t s_idx = r.begin(); s_idx < r.end(); ++s_idx) {
                result[s_idx] = 1.0f;
                if (deactivate) {
                    continue;
                }
                constexpr float decrease_step = 1.0f /
                    (SeamPlacer::sqr_rays_per_sample_point * SeamPlacer::sqr_rays_per_sample_point);

                const Vec3f &center = samples.positions[s_idx];
                const Vec3f &normal = samples.normals[s_idx];
                // apply the local direction via Frame struct - the local_dir is with respect to +Z being forward
                Frame f;
                f.set_from_z(normal);

                for (const auto &dir : precomputed_sample_directions) {
                    Vec3f final_ray_dir = (f.to_world(dir));
                    if (!model_contains_negative_parts) {
                        igl::Hit hitpoint;
                        // FIXME: This AABBTTreeIndirect query will not compile for float ray origin and
                        // direction.
                        Vec3d final_ray_dir_d = final_ray_dir.cast<double>();
                        Vec3d ray_origin_d = (center + normal * 0.01f).cast<double>(); // start above surface.
                        bool hit = AABBTreeIndirect::intersect_ray_first_hit(triangles.vertices, triangles.indices,
                                                                             raycasting_tree, ray_origin_d,
                                                                             final_ray_dir_d, hitpoint);
                        if (hit && its_face_normal(triangles, hitpoint.id).dot(final_ray_dir) <= 0) {
                            result[s_idx] -= decrease_step;
                        }
                    } else { // TODO improve logic for order based boolean operations - consider order of volumes
                        bool casting_from_negative_volume = samples.triangle_indices[s_idx] >=
                            negative_volumes_start_index;

                        Vec3d ray_origin_d = (center + normal * 0.01f).cast<double>(); // start above surface.
                        if (casting_from_negative_volume) { // if casting from negative volume face, invert direction,
                                                            // change start pos
                            final_ray_dir = -1.0 * final_ray_dir;
                            ray_origin_d = (center - normal * 0.01f).cast<double>();
                        }
                        Vec3d final_ray_dir_d = final_ray_dir.cast<double>();
                        bool some_hit = AABBTreeIndirect::intersect_ray_all_hits(triangles.vertices,
                                                                                 triangles.indices, raycasting_tree,
                                                                                 ray_origin_d, final_ray_dir_d, hits);
                        if (some_hit) {
                            int counter = 0;
                            // NOTE: iterating in reverse, from the last hit for one simple reason: We know the state
                            // of the ray at that point;
                            //  It cannot be inside model, and it cannot be inside negative volume
                            for (int hit_index = int(hits.size()) - 1; hit_index >= 0; --hit_index) {
                                Vec3f face_normal = its_face_normal(triangles, hits[hit_index].id);
                                if (hits[hit_index].id >= int(negative_volumes_start_index)) { // negative volume hit
                                    counter -= sgn(
                                        face_normal.dot(final_ray_dir)); // if volume face aligns with ray dir, we are
                                                                         // leaving negative space
                                    // which in reverse hit analysis means, that we are entering negative space :) and
                                    // vice versa
                                } else {
                                    counter += sgn(face_normal.dot(final_ray_dir));
                                }
                            }
                            if (counter == 0) {
                                result[s_idx] -= decrease_step;
                            }
                        }
                    }
                }
            }
        });

    BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: raycast visibility of " << samples.positions.size() << " samples over "
                             << triangles.indices.size() << " triangles: end";

    return result;
}

std::vector<float> calculate_polyline_angles_at_vertices(const PolylineWithEnd &polyline,
                                                         const std::vector<float> &lengths,
                                                         float min_arm_length) {
    std::vector<float> result(polyline.size());
    assert(polyline.size() - 1 == lengths.size());

    if (polyline.size() == 1) {
        result[0] = 0.0f;
    }

    size_t idx_prev = 0;
    size_t idx_curr = 0;
    size_t idx_next = 0;
    size_t idx_start = 0;
    size_t idx_end = 0;

    float distance_to_prev = 0;
    float distance_to_next = 0;

    bool is_polygon = polyline.front() == polyline.back();

    // if polygon : start at 0
    if (is_polygon) {
        idx_start = 0;
        idx_end = polyline.size() - 1; // last point is same as first, don't bother.
        // push idx_prev far enough back as initialization
        while (distance_to_prev < min_arm_length) {
            idx_prev = Slic3r::prev_idx_modulo(idx_prev, lengths.size());
            distance_to_prev += lengths[idx_prev];
        }
    } else {
        // line, start & end are hardcoded
        idx_start = 1;
        idx_end = polyline.size() - 1;
        result.front() = polyline.endpoints.first ? -PI / 2 : 0;
        result.back() = polyline.endpoints.second ? -PI / 2 : 0;
    }

    for (size_t _i = idx_start; _i < idx_end; ++_i) {
        // pull idx_prev to current as much as possible, while respecting the min_arm_length
        while (distance_to_prev - lengths[idx_prev] > min_arm_length && idx_prev < lengths.size()) {
            distance_to_prev -= lengths[idx_prev];
            // if polygon, circle, if line, then stop at the end.
            if (is_polygon) {
                idx_prev = Slic3r::next_idx_modulo(idx_prev, lengths.size());
            } else {
                idx_prev++;
            }
        }

        // push idx_next forward as far as needed
        while (distance_to_next < min_arm_length && idx_next < lengths.size()) {
            distance_to_next += lengths[idx_next];
            // if polygon, circle, if line, then stop at the end.
            if (is_polygon) {
                idx_next = Slic3r::next_idx_modulo(idx_next, lengths.size());
            } else {
                idx_next++;
            }
        }

        // Calculate angle between idx_prev, idx_curr, idx_next.
        const Point &p0 = polyline.points[idx_prev];
        const Point &p1 = polyline.points[idx_curr];
        const Point &p2 = polyline.points[idx_next];
        result[idx_curr] = float(angle_ccw(p1 - p0, p2 - p1));
        if (!is_polygon && polyline.direction == PolylineWithEnd::PolyDir::BOTH) {
            result[idx_curr] = std::abs(result[idx_curr]);
        }

        // increase idx_curr by one
        float curr_distance = lengths[idx_curr];
        idx_curr++;
        distance_to_prev += curr_distance;
        distance_to_next -= curr_distance;
    }

    // polygon: repeat angle
    if (is_polygon) {
        result.back() = result.front();
    }

    return result;
}

struct CoordinateFunctor
{
    const std::vector<Vec3f> *coordinates;
    CoordinateFunctor(const std::vector<Vec3f> *coords) : coordinates(coords) {}
    CoordinateFunctor() : coordinates(nullptr) {}

    const float &operator()(size_t idx, size_t dim) const { return coordinates->operator[](idx)[dim]; }
};

// structure to store global information about the model - occlusion hits, enforcers, blockers
struct GlobalModelInfo
{
    TriangleSetSamples mesh_samples;
    std::vector<float> mesh_samples_visibility;
    CoordinateFunctor mesh_samples_coordinate_functor;
    KDTreeIndirect<3, float, CoordinateFunctor> mesh_samples_tree{CoordinateFunctor{}};
    float mesh_samples_radius;

    indexed_triangle_set enforcers;
    indexed_triangle_set blockers;
    AABBTreeIndirect::Tree<3, float> enforcers_tree;
    AABBTreeIndirect::Tree<3, float> blockers_tree;

    bool has_custom_seam_modifier{false};

    bool is_enforced(const Vec3f &position, float radius) const {
        if (enforcers.empty()) {
            return false;
        }
        float radius_sqr = radius * radius;
        return AABBTreeIndirect::is_any_triangle_in_radius(enforcers.vertices, enforcers.indices, enforcers_tree,
                                                           position, radius_sqr);
    }

    bool is_blocked(const Vec3f &position, float radius) const {
        if (blockers.empty()) {
            return false;
        }
        float radius_sqr = radius * radius;
        return AABBTreeIndirect::is_any_triangle_in_radius(blockers.vertices, blockers.indices, blockers_tree,
                                                           position, radius_sqr);
    }

    float calculate_point_visibility(const Vec3f &position) const {
        std::vector<size_t> points = find_nearby_points(mesh_samples_tree, position, mesh_samples_radius);
        if (points.empty()) {
            return 1.0f;
        }

        auto compute_dist_to_plane = [](const Vec3f &position, const Vec3f &plane_origin, const Vec3f &plane_normal) {
            Vec3f orig_to_point = position - plane_origin;
            return std::abs(orig_to_point.dot(plane_normal));
        };

        float total_weight = 0;
        float total_visibility = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            size_t sample_idx = points[i];

            Vec3f sample_point = this->mesh_samples.positions[sample_idx];
            Vec3f sample_normal = this->mesh_samples.normals[sample_idx];

            float weight = mesh_samples_radius - compute_dist_to_plane(position, sample_point, sample_normal);
            weight += (mesh_samples_radius - (position - sample_point).norm());
            total_visibility += weight * mesh_samples_visibility[sample_idx];
            total_weight += weight;
        }

        return total_visibility / total_weight;
    }

#ifdef DEBUG_FILES
    void debug_export(const indexed_triangle_set &obj_mesh) const {
        indexed_triangle_set divided_mesh = obj_mesh;
        Slic3r::CNumericLocalesSetter locales_setter;

        {
            auto filename = debug_out_path("visiblity.obj");
            FILE *fp = boost::nowide::fopen(filename.c_str(), "w");
            if (fp == nullptr) {
                BOOST_LOG_TRIVIAL(error) << "stl_write_obj: Couldn't open " << filename << " for writing";
                return;
            }

            for (size_t i = 0; i < divided_mesh.vertices.size(); ++i) {
                float visibility = calculate_point_visibility(divided_mesh.vertices[i]);
                Vec3f color = value_to_rgbf(0.0f, 1.0f, visibility);
                fprintf(fp, "v %f %f %f  %f %f %f\n", divided_mesh.vertices[i](0), divided_mesh.vertices[i](1),
                        divided_mesh.vertices[i](2), color(0), color(1), color(2));
            }
            for (size_t i = 0; i < divided_mesh.indices.size(); ++i)
                fprintf(fp, "f %d %d %d\n", divided_mesh.indices[i][0] + 1, divided_mesh.indices[i][1] + 1,
                        divided_mesh.indices[i][2] + 1);
            fclose(fp);
        }

        {
            auto filename = debug_out_path("visiblity_samples.obj");
            FILE *fp = boost::nowide::fopen(filename.c_str(), "w");
            if (fp == nullptr) {
                BOOST_LOG_TRIVIAL(error) << "stl_write_obj: Couldn't open " << filename << " for writing";
                return;
            }

            for (size_t i = 0; i < mesh_samples.positions.size(); ++i) {
                float visibility = mesh_samples_visibility[i];
                Vec3f color = value_to_rgbf(0.0f, 1.0f, visibility);
                fprintf(fp, "v %f %f %f  %f %f %f\n", mesh_samples.positions[i](0), mesh_samples.positions[i](1),
                        mesh_samples.positions[i](2), color(0), color(1), color(2));
            }
            fclose(fp);
        }
    }
#endif
};

// Extract perimeter polylines of the given layer
PolylineWithEnds extract_perimeter_polylines(const Layer *layer,
                                             const SeamPosition configured_seam_preference,
                                             std::vector<const LayerRegion *> &corresponding_regions_out,
                                             PerimeterGeneratorType perimeter_type) {
    PolylineWithEnds polylines;
    class PerimeterCopy : public ExtrusionVisitorConst
    {
        PolylineWithEnds *polylines;
        std::vector<const LayerRegion *> &m_corresponding_regions_out;
        const LayerRegion *current_layer_region;
        SeamPosition configured_seam_preference;
        PerimeterGeneratorType perimeter_type;

    public:
        bool also_overhangs = false;
        bool also_thin_walls = false;
        PerimeterCopy(std::vector<const LayerRegion *> &regions_out,
                      PolylineWithEnds *polys,
                      SeamPosition configured_seam,
                      PerimeterGeneratorType perimeter_type)
            : m_corresponding_regions_out(regions_out)
            , configured_seam_preference(configured_seam)
            , polylines(polys)
            , perimeter_type(perimeter_type) {}
        virtual void default_use(const ExtrusionEntity &entity){};
        virtual void use(const ExtrusionPath &path) override {
            if (perimeter_type == PerimeterGeneratorType::Arachne && path.role() != ExtrusionRole::ThinWall) {
                // path.polygons_covered_by_width(*polygons, SCALED_EPSILON);
                assert(m_corresponding_regions_out.size() == polylines->size());
                polylines->emplace_back(path.polyline.to_polyline().points, true, true,
                                        PolylineWithEnd::PolyDir::BOTH); // TODO: more point for arcs
                assert(path.polyline.front() != path.polyline.back());
                assert(path.polyline.size() > 1);
                // while (m_corresponding_regions_out->size() < polylines->size()) {
                m_corresponding_regions_out.push_back(current_layer_region);
                //}
            }
        }
        virtual void use(const ExtrusionLoop &loop) override {
            bool is_ccw = loop.polygon().is_counter_clockwise();
            if ((configured_seam_preference == spAllRandom && !loop.paths.empty() &&
                 loop.paths.front().role().is_perimeter()) ||
                (also_thin_walls && loop.role() == ExtrusionRole::ThinWall)) {
                Points pts;
                loop.collect_points(pts);
                pts.push_back(pts.front()); // polygon
                assert(m_corresponding_regions_out.size() == polylines->size());
                assert(pts.size() > 1);
                polylines->emplace_back(std::move(pts), true, false,
                                        is_ccw ? PolylineWithEnd::PolyDir::CCW : PolylineWithEnd::PolyDir::CW);
                m_corresponding_regions_out.push_back(current_layer_region);
                return;
            } else {
                PolylineWithEnds polys;
                size_t count_paths_collected = 0;
                bool previous_collected = false;
                bool current_collected = false;
                for (const ExtrusionPath &path : loop.paths) {
                    current_collected = false;
                    if (path.role() == ExtrusionRole::ExternalPerimeter) {
                        if (!previous_collected)
                            polys.emplace_back(false, false,
                                               is_ccw ? PolylineWithEnd::PolyDir::CCW : PolylineWithEnd::PolyDir::CW);
                        path.collect_points(polys.back().points);
                        assert(polys.back().size() > 1);
                        count_paths_collected++;
                        current_collected = true;
                    }
                    if (path.role().is_overhang() &&
                        also_overhangs) { // TODO find a way to search for external overhangs only
                        if (!previous_collected)
                            polys.emplace_back(false, false,
                                               is_ccw ? PolylineWithEnd::PolyDir::CCW : PolylineWithEnd::PolyDir::CW);
                        path.collect_points(polys.back().points);
                        assert(polys.back().size() > 1);
                        count_paths_collected++;
                        current_collected = true;
                    }
                    // if (path.role() == ExtrusionRole::erThinWall && also_thin_walls) {
                    //    path.collect_points(p); // TODO: 2.7: reactivate when it's possible to distinguish between
                    //    thinwalltravel & thinextrusions
                    // // currently, only looking for thinwall-only loop
                    //}
                    previous_collected = current_collected;
                }

                if (!polys.empty()) { // for random seam alignment, extract all perimeters
                    if (count_paths_collected == loop.paths.size()) {
                        assert(polys.size() == 1);
                        assert(polys.front().first_point() == polys.front().last_point());
                    }
                    assert(m_corresponding_regions_out.size() == polylines->size());
                    append(*polylines, std::move(polys));
                    while (m_corresponding_regions_out.size() < polylines->size()) {
                        m_corresponding_regions_out.push_back(current_layer_region);
                    }
                }
            }
        }
        virtual void use(const ExtrusionMultiPath &collection) override {
            if (perimeter_type == PerimeterGeneratorType::Arachne) {
                for (size_t idx = 0; idx < collection.size(); idx++) {
                    const ExtrusionPath &path = collection.paths[idx];
                    assert(m_corresponding_regions_out.size() == polylines->size());
                    polylines->emplace_back(path.polyline.to_polyline().points, idx == 0 ? true : false,
                                            idx + 1 < collection.size() ? false : true,
                                            PolylineWithEnd::PolyDir::BOTH); // TODO: more points for arcs
                    assert(path.polyline.front() != path.polyline.back());
                    assert(path.polyline.size() > 1);
                    m_corresponding_regions_out.push_back(current_layer_region);
                }
            }
        }
        virtual void use(const ExtrusionEntityCollection &collection) override {
            for (const ExtrusionEntity *entity : collection.entities()) {
                entity->visit(*this);
            }
        }
        void set_current_layer_region(const LayerRegion *set) { current_layer_region = set; }
    } visitor(corresponding_regions_out, &polylines, configured_seam_preference, perimeter_type);

    for (const LayerRegion *layer_region : layer->regions()) {
        for (const ExtrusionEntity *ex_entity : layer_region->perimeters()) {
            visitor.set_current_layer_region(layer_region);
            assert(ex_entity->is_collection()); // collection of inner, outer, and overhang perimeters
            ex_entity->visit(visitor);
            if (polylines.empty()) {
                // maybe only thin walls?
                visitor.also_thin_walls = true;
                ex_entity->visit(visitor);
                if (polylines.empty()) {
                    // can happen if the external is fully an overhang
                    visitor.also_overhangs = true;
                    ex_entity->visit(visitor);
                    visitor.also_overhangs = false;
                    if (polylines.empty()) {
                        // shouldn't happen
                        assert(ex_entity->role() == ExtrusionRole::ThinWall); // no loops
                        // ex_entity->visit(visitor);
                        // what to do in this case?
                        Points pts;
                        ex_entity->collect_points(pts);
                        assert(pts.front() != pts.back());
                        polylines.emplace_back(std::move(pts), true, pts.front() != pts.back(),
                                               PolylineWithEnd::PolyDir::BOTH);
                        corresponding_regions_out.push_back(layer_region);
                    }
                }
                visitor.also_thin_walls = false;
            }
        }
    }
    if (polylines.empty()) { // If there are no perimeter polylines/polygons for whatever reason (disabled perimeters
                             // .. ) insert dummy point
        // it is easier than checking everywhere if the layer is not emtpy, no seam will be placed to this layer anyway
        polylines.emplace_back(Points{/*Point { 0, 0 }*/}, true, true, PolylineWithEnd::PolyDir::BOTH);
        corresponding_regions_out.push_back(nullptr);
    }
    assert(corresponding_regions_out.size() == polylines.size());
    return polylines;
}

// Insert SeamCandidates created from perimeter polygons/polylines in to the result vector.
// Compute its type (Enfrocer,Blocker), angle, and position
// each SeamCandidate also contains pointer to shared Perimeter structure representing the polyline
// if Custom Seam modifiers are present, oversamples the polyline if necessary to better fit user intentions
void process_perimeter_polylines(const PolylineWithEnd &orig_polyline,
                                 float z_coord,
                                 const LayerRegion *region,
                                 const GlobalModelInfo &global_model_info,
                                 PrintObjectSeamData::LayerSeams &result) {
    if (orig_polyline.size() <= 1) {
        return;
    }
    PolylineWithEnd polyline = orig_polyline;
    bool is_polygon = polyline.first_point() == polyline.last_point();
    assert(!is_polygon || polyline.direction != PolylineWithEnd::PolyDir::BOTH);
    if (is_polygon && polyline.direction == PolylineWithEnd::PolyDir::CW) {
        polyline.reverse();
        assert(polyline.direction == PolylineWithEnd::PolyDir::CCW);
    }
    float angle_arm_len = region != nullptr ? region->flow(FlowRole::frExternalPerimeter).nozzle_diameter() : 0.5f;

    std::vector<float> lengths{};
    for (size_t point_idx = 0; point_idx < polyline.size() - 1; ++point_idx) {
        lengths.push_back((unscale(polyline.points[point_idx]) - unscale(polyline.points[point_idx + 1])).norm());
    }
    std::vector<float> polyline_angles = calculate_polyline_angles_at_vertices(polyline, lengths, angle_arm_len);

    result.perimeters.push_back({});
    Perimeter &perimeter = result.perimeters.back();

    std::queue<Vec3f> orig_polyline_points{};
    for (size_t index = 0; index < polyline.size(); ++index) {
        Vec2f unscaled_p = unscale(polyline.points[index]).cast<float>();
        orig_polyline_points.emplace(unscaled_p.x(), unscaled_p.y(), z_coord);
    }
    std::queue<Vec3f> oversampled_points{};
    size_t orig_angle_index = 0;
    perimeter.start_index = result.points.size();
    perimeter.flow_width = region != nullptr ? region->flow(FlowRole::frExternalPerimeter).width() : 0.0f;
    bool some_point_enforced = false;
    while (!orig_polyline_points.empty() || !oversampled_points.empty()) {
        EnforcedBlockedSeamPoint type = EnforcedBlockedSeamPoint::Neutral;
        Vec3f position;
        float local_ccw_angle = 0;
        bool orig_point = false;
        if (!oversampled_points.empty()) {
            position = oversampled_points.front();
            oversampled_points.pop();
        } else {
            position = orig_polyline_points.front();
            orig_polyline_points.pop();
            if (orig_polyline.direction == PolylineWithEnd::PolyDir::BOTH) {
                local_ccw_angle = polyline_angles[orig_angle_index];
            } else if (orig_polyline.direction == PolylineWithEnd::PolyDir::CW) {
                local_ccw_angle = -polyline_angles[orig_angle_index];
            } else {
                assert(orig_polyline.direction == PolylineWithEnd::PolyDir::CCW);
                local_ccw_angle = polyline_angles[orig_angle_index];
            }
            orig_angle_index++;
            orig_point = true;
        }

        if (global_model_info.is_enforced(position, perimeter.flow_width)) {
            type = EnforcedBlockedSeamPoint::Enforced;
        }

        if (global_model_info.is_blocked(position, perimeter.flow_width)) {
            type = EnforcedBlockedSeamPoint::Blocked;
        }
        some_point_enforced = some_point_enforced || type == EnforcedBlockedSeamPoint::Enforced;

        if (orig_point && !orig_polyline_points.empty()) {
            Vec3f pos_of_next = orig_polyline_points.front();
            Line line(scaled(Vec2f(position.head<2>())), scaled(Vec2f(pos_of_next.head<2>())));
            float distance_to_next = (position - pos_of_next).norm();
            if (global_model_info.is_enforced((position + pos_of_next) / 2, distance_to_next / 2)) {
                Vec3f vec_to_next = (pos_of_next - position).normalized();
                float step_size = SeamPlacer::enforcer_oversampling_distance;
                float step = step_size;
                while (step < distance_to_next) {
                    oversampled_points.push(position + vec_to_next * step);
                    step += step_size;
                }
            }
        }

        result.points.emplace_back(position, perimeter, local_ccw_angle, type);
    }

    perimeter.end_index = result.points.size();
    assert(!is_polygon ||
           result.points[perimeter.end_index - 1].position == result.points[perimeter.start_index].position);

    if (some_point_enforced) {
        // We will patches of enforced points (patch: continuous section of enforced points), choose
        // the longest patch, and select the middle point or sharp point (depending on the angle)
        // this point will have high priority on this perimeter
        size_t perimeter_size = perimeter.end_index - perimeter.start_index;
        const auto next_index = [&](size_t idx) {
            return perimeter.start_index + Slic3r::next_idx_modulo(idx - perimeter.start_index, perimeter_size);
        };

        assert(perimeter.start_index + 2 <= perimeter.end_index); // at least 2 elt
        std::vector<size_t> patches_starts_ends;
        for (size_t i = perimeter.start_index; i < perimeter.end_index - 1; ++i) {
            if (result.points[i].type != EnforcedBlockedSeamPoint::Enforced &&
                result.points[i + 1].type == EnforcedBlockedSeamPoint::Enforced) {
                patches_starts_ends.push_back(i + 1);
            }
            if (result.points[i].type == EnforcedBlockedSeamPoint::Enforced &&
                result.points[i + 1].type != EnforcedBlockedSeamPoint::Enforced) {
                patches_starts_ends.push_back(i + 1);
            }
        }
        // also add start & end that can't be found for polyline
        if (!is_polygon && result.points[perimeter.start_index].type == EnforcedBlockedSeamPoint::Enforced) {
            patches_starts_ends.insert(patches_starts_ends.begin(), perimeter.start_index);
        }
        if (!is_polygon && result.points[perimeter.end_index - 1].type == EnforcedBlockedSeamPoint::Enforced) {
            patches_starts_ends.push_back(perimeter.end_index);
        }
        // If patches_starts_ends are empty, it means that the whole perimeter is enforced.. don't do anything in that
        // case, as there is no center of a circumference
        if (!patches_starts_ends.empty()) {
            // if the first point in the patches is not enforced, it marks a patch end. in that case, put it to the
            // end and start on next
            // to simplify the processing
            assert(patches_starts_ends.size() % 2 == 0);
            bool start_on_second = false;
            if (result.points[patches_starts_ends[0]].type != EnforcedBlockedSeamPoint::Enforced) {
                start_on_second = true;
                patches_starts_ends.push_back(patches_starts_ends[0]);
            }
            // now pick the longest patch
            std::pair<size_t, size_t> longest_patch{0, 0};
            auto patch_len = [perimeter_size](const std::pair<size_t, size_t> &start_end) {
                if (start_end.second < start_end.first) {
                    return start_end.first + (perimeter_size - start_end.second);
                } else {
                    return start_end.second - start_end.first;
                }
            };
            for (size_t patch_idx = start_on_second ? 1 : 0; patch_idx < patches_starts_ends.size(); patch_idx += 2) {
                std::pair<size_t, size_t> current_patch{patches_starts_ends[patch_idx],
                                                        patches_starts_ends[patch_idx + 1]};
                if (patch_len(longest_patch) < patch_len(current_patch)) {
                    longest_patch = current_patch;
                }
            }
            std::vector<size_t> viable_points_indices;
            std::vector<size_t> large_angle_points_indices;
            assert(longest_patch.first >= perimeter.start_index && longest_patch.first <= perimeter.end_index);
            assert(longest_patch.second >= perimeter.start_index && longest_patch.second <= perimeter.end_index);
            assert(is_polygon || longest_patch.first <= longest_patch.second);
            for (size_t point_idx = longest_patch.first; point_idx != longest_patch.second;
                 point_idx = is_polygon ? next_index(point_idx) : (1 + point_idx)) {
                size_t viable_points_indices_count = viable_points_indices.size();
                viable_points_indices.push_back(point_idx);
                if (std::abs(result.points[point_idx].local_ccw_angle) > SeamPlacer::sharp_angle_snapping_threshold) {
                    large_angle_points_indices.push_back(point_idx);
                }
            }
            assert(viable_points_indices.size() > 0);
            if (large_angle_points_indices.empty()) {
                size_t central_idx = viable_points_indices[viable_points_indices.size() / 2];
                result.points[central_idx].central_enforcer = true;
            } else {
                size_t central_idx = large_angle_points_indices.size() / 2;
                result.points[large_angle_points_indices[central_idx]].central_enforcer = true;
            }
        }
    }
}

// Get index of previous and next perimeter point of the layer. Because SeamCandidates of all polygons of the given
// layer are sequentially stored in the vector, each perimeter contains info about start and end index. These vales
// are used to deduce index of previous and next neigbour in the corresponding perimeter.
std::pair<size_t, size_t> find_previous_and_next_perimeter_point(const std::vector<SeamCandidate> &perimeter_points,
                                                                 size_t point_index) {
    const SeamCandidate &current = perimeter_points[point_index];
    int prev = point_index -
        1; // for majority of points, it is true that neighbours lie behind and in front of them in the vector
    int next = point_index + 1;

    if (point_index == current.perimeter.start_index) {
        // if point_index is equal to start, it means that the previous neighbour is at the end
        prev = current.perimeter.end_index;
    }

    if (point_index == current.perimeter.end_index - 1) {
        // if point_index is equal to end, than next neighbour is at the start
        next = current.perimeter.start_index;
    }

    assert(prev >= 0);
    assert(next >= 0);
    return {size_t(prev), size_t(next)};
}

// Computes all global model info - transforms object, performs raycasting
void compute_global_occlusion(GlobalModelInfo &result,
                              const PrintObject *po,
                              std::function<void(void)> throw_if_canceled) {
    BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: gather occlusion meshes: start";
    auto obj_transform = po->trafo_centered();
    indexed_triangle_set triangle_set;
    indexed_triangle_set negative_volumes_set;
    // add all parts
    for (const ModelVolume *model_volume : po->model_object()->volumes) {
        if (model_volume->type() == ModelVolumeType::MODEL_PART ||
            model_volume->type() == ModelVolumeType::NEGATIVE_VOLUME) {
            auto model_transformation = model_volume->get_matrix();
            indexed_triangle_set model_its = model_volume->mesh().its;
            its_transform(model_its, model_transformation);
            if (model_volume->type() == ModelVolumeType::MODEL_PART) {
                its_merge(triangle_set, model_its);
            } else {
                its_merge(negative_volumes_set, model_its);
            }
        }
    }
    throw_if_canceled();

    BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: gather occlusion meshes: end";

    BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: decimate: start";
    its_short_edge_collpase(triangle_set, SeamPlacer::fast_decimation_triangle_count_target);
    its_short_edge_collpase(negative_volumes_set, SeamPlacer::fast_decimation_triangle_count_target);

    size_t negative_volumes_start_index = triangle_set.indices.size();
    its_merge(triangle_set, negative_volumes_set);
    its_transform(triangle_set, obj_transform);
    BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: decimate: end";

    BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: Compute visibility sample points: start";

    result.mesh_samples = sample_its_uniform_parallel(SeamPlacer::raycasting_visibility_samples_count, triangle_set);
    result.mesh_samples_coordinate_functor = CoordinateFunctor(&result.mesh_samples.positions);
    result.mesh_samples_tree = KDTreeIndirect<3, float, CoordinateFunctor>(result.mesh_samples_coordinate_functor,
                                                                           result.mesh_samples.positions.size());

    // The following code determines search area for random visibility samples on the mesh when calculating visibility
    // of each perimeter point number of random samples in the given radius (area) is approximately poisson
    // distribution to compute ideal search radius (area), we use exponential distribution (complementary distr to
    // poisson) parameters of exponential distribution to compute area that will have with probability="probability"
    // more than given number of samples="samples"
    float probability = 0.9f;
    float samples = 4;
    float density = SeamPlacer::raycasting_visibility_samples_count / result.mesh_samples.total_area;
    // exponential probability distrubtion function is : f(x) = P(X > x) = e^(l*x) where l is the rate parameter (computed
    // as 1/u where u is mean value) probability that sampled area A with S samples contains more than samples count:
    //  P(S > samples in A) = e^-(samples/(density*A));   express A:
    float search_area = samples / (-logf(probability) * density);
    float search_radius = sqrt(search_area / PI);
    result.mesh_samples_radius = search_radius;

    BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: Compute visiblity sample points: end";
    throw_if_canceled();

    BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: Mesh sample raidus: " << result.mesh_samples_radius;

    BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: build AABB tree: start";
    auto raycasting_tree = AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(triangle_set.vertices,
                                                                                       triangle_set.indices);

    throw_if_canceled();
    BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: build AABB tree: end";
    bool has_seam_visibility = po->config().seam_visibility.value &&
        po->config().seam_position.value == SeamPosition::spCost;
    result.mesh_samples_visibility = raycast_visibility(raycasting_tree, triangle_set, result.mesh_samples,
                                                        negative_volumes_start_index, !has_seam_visibility);
    throw_if_canceled();
#ifdef DEBUG_FILES
    result.debug_export(triangle_set);
#endif
}

void gather_enforcers_blockers(GlobalModelInfo &result, const PrintObject *po) {
    BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: build AABB trees for raycasting enforcers/blockers: start";

    auto obj_transform = po->trafo_centered();

    for (const ModelVolume *mv : po->model_object()->volumes) {
        if (mv->is_seam_painted()) {
            auto model_transformation = obj_transform * mv->get_matrix();

            indexed_triangle_set enforcers = mv->seam_facets.get_facets(*mv, EnforcerBlockerType::ENFORCER);
            its_transform(enforcers, model_transformation);
            its_merge(result.enforcers, enforcers);

            indexed_triangle_set blockers = mv->seam_facets.get_facets(*mv, EnforcerBlockerType::BLOCKER);
            its_transform(blockers, model_transformation);
            its_merge(result.blockers, blockers);
        }
    }

    result.enforcers_tree = AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(result.enforcers.vertices,
                                                                                        result.enforcers.indices);
    result.blockers_tree = AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(result.blockers.vertices,
                                                                                       result.blockers.indices);

    BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: build AABB trees for raycasting enforcers/blockers: end";
}

struct SeamComparator
{
    SeamPosition setup;
    float angle_importance = 1.f;
    float travel_importance = 1.f;
    float visibility_importance = 1.f;
    Point seam_mod_pos;
    explicit SeamComparator(SeamPosition setup, const PrintObject &po) : setup(setup) {
        if (setup == spCustom)
            setup = spCost;
        if (setup == spNearest) {
            // setup = spCost;
            angle_importance = SeamPlacer::angle_importance_nearest;
        } else if (setup == spAligned || setup == spExtremlyAligned) {
            angle_importance = SeamPlacer::angle_importance_aligned;
        } else {
            travel_importance = (float) po.config().seam_travel_cost.get_abs_value(1.f);
            angle_importance = (float) po.config().seam_angle_cost.get_abs_value(1.f);
        }
        visibility_importance = (po.config().seam_visibility.value &&
                                 po.config().seam_position.value == SeamPosition::spCost) ?
            1.f :
            0.f;
    }

    // Standard comparator, must respect the requirements of comparators (e.g. give same result on same inputs) for
    // sorting usage should return if a is better seamCandidate than b
    bool is_first_better(const SeamCandidate &a,
                         const SeamCandidate &b,
                         const Vec2f &preffered_location = Vec2f{0.0f, 0.0f}) const {
        if ((setup == SeamPosition::spAligned || setup == SeamPosition::spExtremlyAligned) &&
            a.central_enforcer != b.central_enforcer) {
            return a.central_enforcer;
        }

        // Blockers/Enforcers discrimination, top priority
        if (a.type != b.type) {
            return a.type > b.type;
        }

        // avoid overhangs
        if ((a.overhang > a.perimeter.flow_width / 4 && b.overhang == 0.0f) ||
            (b.overhang > b.perimeter.flow_width / 4 && a.overhang == 0.0f)) {
            return a.overhang < b.overhang;
        }

        // prefer hidden points (more than 0.5 mm inside)
        if (a.embedded_distance < -0.5f && b.embedded_distance > -0.5f) {
            return true;
        }
        if (b.embedded_distance < -0.5f && a.embedded_distance > -0.5f) {
            return false;
        }

        if (setup == SeamPosition::spRear && a.position.y() != b.position.y()) {
            return a.position.y() > b.position.y();
        }

        float distance_penalty_a = 0.0f;
        float distance_penalty_b = 0.0f;
        if (setup == spNearest || setup == spCost || setup == spCustom) {
            distance_penalty_a = 1.0f - gauss((a.position.head<2>() - preffered_location).norm(), 0.0f, 1.0f, 0.005f);
            distance_penalty_b = 1.0f - gauss((b.position.head<2>() - preffered_location).norm(), 0.0f, 1.0f, 0.005f);
        }

        // the penalites are kept close to range [0-1.x] however, it should not be relied upon
        float penalty_a = 2 * a.overhang / a.perimeter.flow_width + visibility_importance * a.visibility +
            angle_importance * compute_angle_penalty(a.local_ccw_angle) + travel_importance * distance_penalty_a;
        float penalty_b = 2 * b.overhang / b.perimeter.flow_width + visibility_importance * b.visibility +
            angle_importance * compute_angle_penalty(b.local_ccw_angle) + travel_importance * distance_penalty_b;

        return penalty_a < penalty_b;
    }

    // Comparator used during alignment. If there is close potential aligned point, it is compared to the current
    // seam point of the perimeter, to find out if the aligned point is not much worse than the current seam
    // Also used by the random seam generator.
    bool is_first_not_much_worse(const SeamCandidate &a, const SeamCandidate &b) const {
        // Blockers/Enforcers discrimination, top priority
        if ((setup == SeamPosition::spAligned || setup == SeamPosition::spExtremlyAligned) &&
            a.central_enforcer != b.central_enforcer) {
            // Prefer centers of enforcers.
            return a.central_enforcer;
        }

        if (a.type == EnforcedBlockedSeamPoint::Enforced) {
            return true;
        }

        if (a.type == EnforcedBlockedSeamPoint::Blocked) {
            return false;
        }

        if (a.type != b.type) {
            return a.type > b.type;
        }

        // avoid overhangs
        if ((a.overhang > 0.0f || b.overhang > 0.0f) &&
            abs(a.overhang - b.overhang) > (0.1f * a.perimeter.flow_width)) {
            return a.overhang < b.overhang;
        }

        // prefer hidden points (more than 0.5 mm inside)
        if (a.embedded_distance < -0.5f && b.embedded_distance > -0.5f) {
            return true;
        }
        if (b.embedded_distance < -0.5f && a.embedded_distance > -0.5f) {
            return false;
        }

        if (setup == SeamPosition::spRandom || setup == SeamPosition::spAllRandom) {
            return true;
        }

        if (setup == SeamPosition::spRear) {
            return a.position.y() + SeamPlacer::seam_align_score_tolerance * 5.0f > b.position.y();
        }

        float penalty_a = a.overhang + a.visibility + angle_importance * compute_angle_penalty(a.local_ccw_angle);
        float penalty_b = b.overhang + b.visibility + angle_importance * compute_angle_penalty(b.local_ccw_angle);

        return penalty_a <= penalty_b || penalty_a - penalty_b < SeamPlacer::seam_align_score_tolerance;
    }

    bool are_similar(const SeamCandidate &a, const SeamCandidate &b) const {
        return is_first_not_much_worse(a, b) && is_first_not_much_worse(b, a);
    }
};

#ifdef DEBUG_FILES
void debug_export_points(const std::vector<PrintObjectSeamData::LayerSeams> &layers,
                         const BoundingBox &bounding_box,
                         const SeamComparator &comparator) {
    for (size_t layer_idx = 0; layer_idx < layers.size(); ++layer_idx) {
        std::string angles_file_name = debug_out_path(("angles_" + std::to_string(layer_idx) + ".svg").c_str());
        SVG angles_svg{angles_file_name, bounding_box};
        float min_vis = 0;
        float max_vis = min_vis;

        float min_weight = std::numeric_limits<float>::min();
        float max_weight = min_weight;

        for (const SeamCandidate &point : layers[layer_idx].points) {
            Vec3i32 color = value_to_rgbi(-PI, PI, point.local_ccw_angle);
            std::string fill = "rgb(" + std::to_string(color.x()) + "," + std::to_string(color.y()) + "," +
                std::to_string(color.z()) + ")";
            angles_svg.draw(scaled(Vec2f(point.position.head<2>())), fill);
            min_vis = std::min(min_vis, point.visibility);
            max_vis = std::max(max_vis, point.visibility);

            min_weight = std::min(min_weight, -compute_angle_penalty(point.local_ccw_angle));
            max_weight = std::max(max_weight, -compute_angle_penalty(point.local_ccw_angle));
        }

        std::string visiblity_file_name = debug_out_path(("visibility_" + std::to_string(layer_idx) + ".svg").c_str());
        SVG visibility_svg{visiblity_file_name, bounding_box};
        std::string weights_file_name = debug_out_path(("weight_" + std::to_string(layer_idx) + ".svg").c_str());
        SVG weight_svg{weights_file_name, bounding_box};
        std::string overhangs_file_name = debug_out_path(("overhang_" + std::to_string(layer_idx) + ".svg").c_str());
        SVG overhangs_svg{overhangs_file_name, bounding_box};

        for (const SeamCandidate &point : layers[layer_idx].points) {
            Vec3i32 color = value_to_rgbi(min_vis, max_vis, point.visibility);
            std::string visibility_fill = "rgb(" + std::to_string(color.x()) + "," + std::to_string(color.y()) + "," +
                std::to_string(color.z()) + ")";
            visibility_svg.draw(scaled(Vec2f(point.position.head<2>())), visibility_fill);

            Vec3i32 weight_color = value_to_rgbi(min_weight, max_weight,
                                                 -compute_angle_penalty(point.local_ccw_angle));
            std::string weight_fill = "rgb(" + std::to_string(weight_color.x()) + "," +
                std::to_string(weight_color.y()) + "," + std::to_string(weight_color.z()) + ")";
            weight_svg.draw(scaled(Vec2f(point.position.head<2>())), weight_fill);

            Vec3i32 overhang_color = value_to_rgbi(-0.5, 0.5, std::clamp(point.overhang, -0.5f, 0.5f));
            std::string overhang_fill = "rgb(" + std::to_string(overhang_color.x()) + "," +
                std::to_string(overhang_color.y()) + "," + std::to_string(overhang_color.z()) + ")";
            overhangs_svg.draw(scaled(Vec2f(point.position.head<2>())), overhang_fill);
        }
    }
}
#endif

// Pick best seam point based on the given comparator
void pick_seam_point(std::vector<SeamCandidate> &perimeter_points,
                     size_t start_index,
                     const SeamComparator &comparator) {
    size_t end_index = perimeter_points[start_index].perimeter.end_index;

    size_t seam_index = start_index;
    for (size_t index = start_index; index < end_index; ++index) {
        if (comparator.is_first_better(perimeter_points[index], perimeter_points[seam_index])) {
            seam_index = index;
        }
    }
    perimeter_points[start_index].perimeter.seam_index = seam_index;
}

size_t pick_nearest_seam_point_index(const std::vector<SeamCandidate> &perimeter_points,
                                     size_t start_index,
                                     const Vec2f &preffered_location,
                                     const PrintObject &po) {
    size_t end_index = perimeter_points[start_index].perimeter.end_index;
    SeamComparator comparator{po.config().seam_position.value /*spNearest*/, po};

    size_t seam_index = start_index;
    for (size_t index = start_index; index < end_index; ++index) {
        if (comparator.is_first_better(perimeter_points[index], perimeter_points[seam_index], preffered_location)) {
            seam_index = index;
        }
    }
    return seam_index;
}

// picks random seam point uniformly, respecting enforcers blockers and overhang avoidance.
void pick_random_seam_point(const std::vector<SeamCandidate> &perimeter_points,
                            size_t start_index,
                            const PrintObject &po) {
    SeamComparator comparator{spRandom, po};

    // algorithm keeps a list of viable points and their lengths. If it finds a point
    // that is much better than the viable_example_index (e.g. better type, no overhang; see is_first_not_much_worse)
    // then it throws away stored lists and starts from start
    // in the end, the list should contain points with same type (Enforced > Neutral > Blocked) and also only those
    // which are not big overhang.
    size_t viable_example_index = start_index;
    size_t end_index = perimeter_points[start_index].perimeter.end_index;
    struct Viable
    {
        // Candidate seam point index.
        size_t index;
        float edge_length;
        Vec3f edge;
    };
    std::vector<Viable> viables;

    const Vec3f pseudornd_seed = perimeter_points[viable_example_index].position;
    float rand = std::abs(sin(pseudornd_seed.dot(Vec3f(12.9898f, 78.233f, 133.3333f))) * 43758.5453f);
    rand = rand - (int) rand;

    for (size_t index = start_index; index < end_index; ++index) {
        if (comparator.are_similar(perimeter_points[index], perimeter_points[viable_example_index])) {
            // index ok, push info into viables
            Vec3f edge_to_next{perimeter_points[index == end_index - 1 ? start_index : index + 1].position -
                               perimeter_points[index].position};
            float dist_to_next = edge_to_next.norm();
            viables.push_back({index, dist_to_next, edge_to_next});
        } else if (comparator.is_first_not_much_worse(perimeter_points[viable_example_index],
                                                      perimeter_points[index])) {
            // index is worse then viable_example_index, skip this point
        } else {
            // index is better than viable example index, update example, clear gathered info, start again
            // clear up all gathered info, start from scratch, update example index
            viable_example_index = index;
            viables.clear();

            Vec3f edge_to_next = (perimeter_points[index == end_index - 1 ? start_index : index + 1].position -
                                  perimeter_points[index].position);
            float dist_to_next = edge_to_next.norm();
            viables.push_back({index, dist_to_next, edge_to_next});
        }
    }

    // now pick random point from the stored options
    float len_sum = std::accumulate(viables.begin(), viables.end(), 0.0f,
                                    [](const float acc, const Viable &v) { return acc + v.edge_length; });
    float picked_len = len_sum * rand;

    size_t point_idx = 0;
    while (picked_len - viables[point_idx].edge_length > 0) {
        picked_len = picked_len - viables[point_idx].edge_length;
        point_idx++;
    }

    Perimeter &perimeter = perimeter_points[start_index].perimeter;
    perimeter.seam_index = viables[point_idx].index;
    perimeter.final_seam_position = perimeter_points[perimeter.seam_index].position +
        viables[point_idx].edge.normalized() * picked_len;
    perimeter.finalized = true;
}

} // namespace SeamPlacerImpl

// Parallel process and extract each perimeter polygon of the given print object.
// Gather SeamCandidates of each layer into vector and build KDtree over them
// Store results in the SeamPlacer variables m_seam_per_object
void SeamPlacer::gather_seam_candidates(const PrintObject *po,
                                        const SeamPlacerImpl::GlobalModelInfo &global_model_info,
                                        SeamPosition configured_seam_preference) {
    using namespace SeamPlacerImpl;
    PrintObjectSeamData &seam_data = m_seam_per_object.emplace(po, PrintObjectSeamData{}).first->second;
    seam_data.layers.resize(po->layer_count());

    // use an antomic idx instead of the range, to avoid a thread being very late because it's on the difficult layers.
    std::atomic_size_t next_layer_idx(0);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, po->layers().size()),
                      [po, &global_model_info, &seam_data, configured_seam_preference,
                       &next_layer_idx](tbb::blocked_range<size_t> r) {
                          // for (size_t layer_idx = r.begin(); layer_idx < r.end(); ++layer_idx) {
                          for (size_t layer_idx = next_layer_idx++; layer_idx < po->layers().size();
                               layer_idx = next_layer_idx++) {
                              PrintObjectSeamData::LayerSeams &layer_seams = seam_data.layers[layer_idx];
                              const Layer *layer = po->get_layer(layer_idx);
                              auto unscaled_z = layer->slice_z;
                              std::vector<const LayerRegion *> regions;
                              // NOTE corresponding region ptr may be null, if the layer has zero perimeters
                              PolylineWithEnds polygons_and_lines =
                                  extract_perimeter_polylines(layer, configured_seam_preference, regions,
                                                              po->config().perimeter_generator.value);
                              for (size_t poly_index = 0; poly_index < polygons_and_lines.size(); ++poly_index) {
                                  process_perimeter_polylines(polygons_and_lines[poly_index], unscaled_z,
                                                              regions[poly_index], global_model_info, layer_seams);
                              }
                              auto functor = SeamCandidateCoordinateFunctor{layer_seams.points};
                              seam_data.layers[layer_idx].points_tree =
                                  std::make_unique<PrintObjectSeamData::SeamCandidatesTree>(functor,
                                                                                            layer_seams.points.size());
                          }
                      });
}

void SeamPlacer::calculate_candidates_visibility(const PrintObject *po,
                                                 const SeamPlacerImpl::GlobalModelInfo &global_model_info) {
    using namespace SeamPlacerImpl;

    std::vector<PrintObjectSeamData::LayerSeams> &layers = m_seam_per_object[po].layers;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, layers.size()),
                      [&layers, &global_model_info](tbb::blocked_range<size_t> r) {
                          for (size_t layer_idx = r.begin(); layer_idx < r.end(); ++layer_idx) {
                              for (auto &perimeter_point : layers[layer_idx].points) {
                                  perimeter_point.visibility = global_model_info.calculate_point_visibility(
                                      perimeter_point.position);
                              }
                          }
                      });
}

void SeamPlacer::calculate_overhangs_and_layer_embedding(const PrintObject *po) {
    using namespace SeamPlacerImpl;
    using PerimeterDistancer = AABBTreeLines::LinesDistancer<Linef>;

    std::vector<PrintObjectSeamData::LayerSeams> &layers = m_seam_per_object[po].layers;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, layers.size()), [po, &layers](tbb::blocked_range<size_t> r) {
        std::unique_ptr<PerimeterDistancer> prev_layer_distancer;
        if (r.begin() > 0) { // previous layer exists
            prev_layer_distancer = std::make_unique<PerimeterDistancer>(
                to_unscaled_linesf(po->layers()[r.begin() - 1]->lslices));
        }

        for (size_t layer_idx = r.begin(); layer_idx < r.end(); ++layer_idx) {
            size_t regions_with_perimeter = 0;
            for (const LayerRegion *region : po->layers()[layer_idx]->regions()) {
                if (region->perimeters().size() > 0) {
                    regions_with_perimeter++;
                }
            };
            bool should_compute_layer_embedding = regions_with_perimeter > 1;
            std::unique_ptr<PerimeterDistancer> current_layer_distancer = std::make_unique<PerimeterDistancer>(
                to_unscaled_linesf(po->layers()[layer_idx]->lslices));

            for (SeamCandidate &perimeter_point : layers[layer_idx].points) {
                Vec2f point = Vec2f{perimeter_point.position.head<2>()};
                if (prev_layer_distancer.get() != nullptr) {
                    perimeter_point.overhang = prev_layer_distancer->distance_from_lines<true>(point.cast<double>());
                    // Seams: overhangs: don't remove overhang_angle_threshold, it seems to create artifacts.
                    // supermerill/SuperSlicer#4217
                    // + 0.6f * perimeter_point.perimeter.flow_width
                    // - tan(SeamPlacer::overhang_angle_threshold)
                    //         * po->layers()[layer_idx]->height;
                    perimeter_point.overhang = perimeter_point.overhang < 0.0f ? 0.0f : perimeter_point.overhang;
                }

                if (should_compute_layer_embedding) { // search for embedded perimeter points (points hidden inside
                                                      // the print ,e.g. multimaterial join, best position for seam)
                    perimeter_point.embedded_distance = current_layer_distancer->distance_from_lines<true>(
                                                            point.cast<double>()) +
                        0.6f * perimeter_point.perimeter.flow_width;
                }
            }

            prev_layer_distancer.swap(current_layer_distancer);
        }
    });
}

// Estimates, if there is good seam point in the layer_idx which is close to last_point_pos
// uses comparator.is_first_not_much_worse method to compare current seam with the closest point
// (if current seam is too far away )
// If the current chosen stream is close enough, it is stored in seam_string. returns true and updates last_point_pos
// If the closest point is good enough to replace current chosen seam, it is stored in potential_string_seams, returns
// true and updates last_point_pos Otherwise does nothing, returns false Used by align_seam_points().
std::optional<std::pair<size_t, size_t>> SeamPlacer::find_next_seam_in_layer(
    const std::vector<PrintObjectSeamData::LayerSeams> &layers,
    const Vec3f &projected_position,
    const size_t layer_idx,
    const float max_distance,
    const SeamPlacerImpl::SeamComparator &comparator) const {
    using namespace SeamPlacerImpl;
    std::vector<size_t> nearby_points_indices = find_nearby_points(*layers[layer_idx].points_tree, projected_position,
                                                                   max_distance);

    if (nearby_points_indices.empty()) {
        return {};
    }

    size_t best_nearby_point_index = nearby_points_indices[0];
    size_t nearest_point_index = nearby_points_indices[0];

    // Now find best nearby point, nearest point, and corresponding indices
    for (const size_t &nearby_point_index : nearby_points_indices) {
        const SeamCandidate &point = layers[layer_idx].points[nearby_point_index];
        if (point.perimeter.finalized) {
            continue; // skip over finalized perimeters, try to find some that is not finalized
        }
        if (comparator.is_first_better(point, layers[layer_idx].points[best_nearby_point_index],
                                       projected_position.head<2>()) ||
            layers[layer_idx].points[best_nearby_point_index].perimeter.finalized) {
            best_nearby_point_index = nearby_point_index;
        }
        if ((point.position - projected_position).squaredNorm() <
                (layers[layer_idx].points[nearest_point_index].position - projected_position).squaredNorm() ||
            layers[layer_idx].points[nearest_point_index].perimeter.finalized) {
            nearest_point_index = nearby_point_index;
        }
    }

    const SeamCandidate &best_nearby_point = layers[layer_idx].points[best_nearby_point_index];
    const SeamCandidate &nearest_point = layers[layer_idx].points[nearest_point_index];

    if (nearest_point.perimeter.finalized) {
        // all points are from already finalized perimeter, skip
        return {};
    }

    // from the nearest_point, deduce index of seam in the next layer
    const SeamCandidate &next_layer_seam = layers[layer_idx].points[nearest_point.perimeter.seam_index];

    // First try to pick central enforcer if any present
    if (next_layer_seam.central_enforcer &&
        (next_layer_seam.position - projected_position).squaredNorm() < sqr(3 * max_distance)) {
        return {std::pair<size_t, size_t>{layer_idx, nearest_point.perimeter.seam_index}};
    }

    // First try to align the nearest, then try the best nearby
    if (comparator.is_first_not_much_worse(nearest_point, next_layer_seam)) {
        return {std::pair<size_t, size_t>{layer_idx, nearest_point_index}};
    }
    // If nearest point is not good enough, try it with the best nearby point.
    if (comparator.is_first_not_much_worse(best_nearby_point, next_layer_seam)) {
        return {std::pair<size_t, size_t>{layer_idx, best_nearby_point_index}};
    }

    return {};
}

// get the nearests points from layers above & below. stop when the seam_align_tolerable_dist_factor don't allow to
// jump to a point,
std::vector<std::pair<size_t, size_t>> SeamPlacer::find_seam_string(
    const PrintObject *po,
    std::pair<size_t, size_t> start_seam,
    const SeamPlacerImpl::SeamComparator &comparator) const {
    const std::vector<PrintObjectSeamData::LayerSeams> &layers = m_seam_per_object.find(po)->second.layers;
    int layer_idx = start_seam.first;

    // initialize searching for seam string - cluster of nearby seams on previous and next layers
    int next_layer = layer_idx + 1;
    int step = 1;
    std::pair<size_t, size_t> prev_point_index = start_seam;
    std::vector<std::pair<size_t, size_t>> seam_string{start_seam};

    auto reverse_lookup_direction = [&]() {
        step = -1;
        prev_point_index = start_seam;
        next_layer = layer_idx - 1;
    };

    while (next_layer >= 0) {
        if (next_layer >= int(layers.size())) {
            reverse_lookup_direction();
            if (next_layer < 0) {
                break;
            }
        }
        float max_nozzle_diam = 0;
        for (size_t region_id = 0; region_id < po->num_printing_regions(); ++region_id) {
            max_nozzle_diam = std::max(max_nozzle_diam,
                                       (float) po->print()->config().nozzle_diameter.get_at(
                                           po->printing_region(region_id).config().perimeter_extruder.value - 1));
        }
        float max_distance = SeamPlacer::seam_align_tolerable_dist_factor * max_nozzle_diam *
            layers[start_seam.first].points[start_seam.second].perimeter.flow_width;
        Vec3f prev_position = layers[prev_point_index.first].points[prev_point_index.second].position;
        Vec3f projected_position = prev_position;
        projected_position.z() = float(po->get_layer(next_layer)->slice_z);

        std::optional<std::pair<size_t, size_t>> maybe_next_seam = find_next_seam_in_layer(layers, projected_position,
                                                                                           next_layer, max_distance,
                                                                                           comparator);

        if (maybe_next_seam.has_value()) {
            // For old macOS (pre 10.14), std::optional does not have .value() method, so the code is using
            // operator*() instead.
            seam_string.push_back(maybe_next_seam.operator*());
            prev_point_index = seam_string.back();
            // String added, prev_point_index updated
        } else {
            if (step == 1) {
                reverse_lookup_direction();
                if (next_layer < 0) {
                    break;
                }
            } else {
                break;
            }
        }
        next_layer += step;
    }
    return seam_string;
}

// clusters already chosen seam points into strings across multiple layers, and then
// aligns the strings via polynomial fit
// Does not change the positions of the SeamCandidates themselves, instead stores
// the new aligned position into the shared Perimeter structure of each perimeter
// Note that this position does not necesarilly lay on the perimeter.
void SeamPlacer::align_seam_points(const PrintObject *po, const SeamPlacerImpl::SeamComparator &comparator) {
    using namespace SeamPlacerImpl;

    // Prepares Debug files for writing.
#ifdef DEBUG_FILES
    Slic3r::CNumericLocalesSetter locales_setter;
    auto clusters_f = debug_out_path("seam_clusters.obj");
    FILE *clusters = boost::nowide::fopen(clusters_f.c_str(), "w");
    if (clusters == nullptr) {
        BOOST_LOG_TRIVIAL(error) << "stl_write_obj: Couldn't open " << clusters_f << " for writing";
        return;
    }
    auto aligned_f = debug_out_path("aligned_clusters.obj");
    FILE *aligns = boost::nowide::fopen(aligned_f.c_str(), "w");
    if (aligns == nullptr) {
        BOOST_LOG_TRIVIAL(error) << "stl_write_obj: Couldn't open " << clusters_f << " for writing";
        return;
    }
#endif

    // gather vector of all seams on the print_object - pair of layer_index and seam__index within that layer
    const std::vector<PrintObjectSeamData::LayerSeams> &layers = m_seam_per_object[po].layers;
    std::vector<std::pair<size_t, size_t>> seams;
    for (size_t layer_idx = 0; layer_idx < layers.size(); ++layer_idx) {
        const std::vector<SeamCandidate> &layer_perimeter_points = layers[layer_idx].points;
        size_t current_point_index = 0;
        while (current_point_index < layer_perimeter_points.size()) {
            seams.emplace_back(layer_idx, layer_perimeter_points[current_point_index].perimeter.seam_index);
            current_point_index = layer_perimeter_points[current_point_index].perimeter.end_index;
        }
    }

    // sort them before alignment. Alignment is sensitive to initializaion, this gives it better chance to choose
    // something nice
    if (comparator.setup != spExtremlyAligned) {
        std::stable_sort(seams.begin(), seams.end(),
                         [&comparator, &layers](const std::pair<size_t, size_t> &left,
                                                const std::pair<size_t, size_t> &right) {
                             return comparator.is_first_better(layers[left.first].points[left.second],
                                                               layers[right.first].points[right.second]);
                         });
    }

    // spExtremlyAligned: don't move unless it's an enforcer / Sphere
    if (comparator.setup == spExtremlyAligned && seams.size() > 1) {
        std::vector<std::vector<Perimeter *>> layer2seams;
        size_t seam_idx = 0;
        size_t current_layer_idx = 0;
        // fisrt layer: keep that.
        layer2seams.emplace_back();
        while (seams[seam_idx].first == 0 && seam_idx < seams.size()) {
            layer2seams.back().push_back(&layers[0].points[seams[seam_idx].second].perimeter);
            layer2seams.back().back()->finalized = true;
            // iterate
            seam_idx++;
        }
        // next layer: get the nearest point form previous layer, and project it onto the current perimeter.
        for (; seam_idx < seams.size(); seam_idx++) {
            while (current_layer_idx < seams[seam_idx].first) {
                layer2seams.emplace_back();
                current_layer_idx++;
            }
            const std::vector<SeamPlacerImpl::SeamCandidate> &points = layers[seams[seam_idx].first].points;
            const SeamCandidate &current = points[seams[seam_idx].second];
            Perimeter &perimeter = points[seams[seam_idx].second].perimeter;
            layer2seams.back().push_back(&perimeter);
            if (current.type == EnforcedBlockedSeamPoint::Enforced) {
                //// TODO: align as much as possible inside the enforcement
                perimeter.seam_index = seams[seam_idx].second;
                perimeter.final_seam_position = current.position;
                perimeter.finalized = true;
            } else if (current.type == EnforcedBlockedSeamPoint::Sphere) {
                // keep as-is
            } else if (current_layer_idx > 0 && !layer2seams[current_layer_idx - 1].empty()) {
                // TODO: remove seam blocker
                Point nearest_point;
                Vec3f nearest_old_point;
                size_t nearest_pt_idx;
                size_t next_pt_idx;
                double nearest_sqr_dist = std::numeric_limits<double>::max();
                for (const Perimeter *lower_peri : layer2seams[current_layer_idx - 1]) {
                    // old point
                    Point lower_pt{scale_t(lower_peri->final_seam_position.x()),
                                   scale_t(lower_peri->final_seam_position.y())};
                    // for each segment
                    for (int i = perimeter.start_index; i < perimeter.end_index - 1; i++) {
                        Point pt = lower_pt.projection_onto(Point{scale_t(points[i].position.x()),
                                                                  scale_t(points[i].position.y())},
                                                            Point{scale_t(points[i + 1].position.x()),
                                                                  scale_t(points[i + 1].position.y())});
                        double dist_sqr = pt.distance_to_square(lower_pt);
                        if (dist_sqr < nearest_sqr_dist) {
                            nearest_sqr_dist = dist_sqr;
                            nearest_point = pt;
                            nearest_pt_idx = i;
                            next_pt_idx = i + 1;
                            nearest_old_point = lower_peri->final_seam_position;
                        }
                    }
                    // test last segment
                    {
                        Point pt = lower_pt
                                       .projection_onto(Point{scale_t(points[perimeter.end_index - 1].position.x()),
                                                              scale_t(points[perimeter.end_index - 1].position.y())},
                                                        Point{scale_t(points[perimeter.start_index].position.x()),
                                                              scale_t(points[perimeter.start_index].position.y())});
                        double dist_sqr = pt.distance_to_square(lower_pt);
                        if (dist_sqr < nearest_sqr_dist) {
                            nearest_sqr_dist = dist_sqr;
                            nearest_point = pt;
                            nearest_pt_idx = perimeter.end_index - 1;
                            next_pt_idx = perimeter.start_index;
                            nearest_old_point = lower_peri->final_seam_position;
                        }
                    }
                }

                if (nearest_sqr_dist < std::numeric_limits<double>::max()) {
                    perimeter.seam_index = nearest_pt_idx;
                    perimeter.final_seam_position.x() = unscaled(nearest_point.x());
                    perimeter.final_seam_position.y() = unscaled(nearest_point.y());
                    perimeter.finalized = true;
                }
            }
        }
        return;
    }

    // align the seam points - start with the best, and check if they are aligned, if yes, skip, else start alignment
    // Keeping the vectors outside, so with a bit of luck they will not get reallocated after couple of for loop iterations.
    std::vector<std::pair<size_t, size_t>> seam_string;
    std::vector<std::pair<size_t, size_t>> alternative_seam_string;
    std::vector<Vec2f> observations;
    std::vector<float> observation_points;
    std::vector<float> weights;

    int global_index = 0;
    while (global_index < int(seams.size())) {
        size_t layer_idx = seams[global_index].first;
        size_t seam_index = seams[global_index].second;
        global_index++;
        const std::vector<SeamCandidate> &layer_perimeter_points = layers[layer_idx].points;
        if (layer_perimeter_points[seam_index].perimeter.finalized) {
            // This perimeter is already aligned, skip seam
            continue;
        } else {
            seam_string = this->find_seam_string(po, {layer_idx, seam_index}, comparator);
            size_t step_size = 1 + seam_string.size() / 20;
            for (size_t alternative_start = 0; alternative_start < seam_string.size();
                 alternative_start += step_size) {
                size_t start_layer_idx = seam_string[alternative_start].first;
                size_t seam_idx =
                    layers[start_layer_idx].points[seam_string[alternative_start].second].perimeter.seam_index;
                alternative_seam_string = this->find_seam_string(po,
                                                                 std::pair<size_t, size_t>(start_layer_idx, seam_idx),
                                                                 comparator);
                if (alternative_seam_string.size() > seam_string.size()) {
                    seam_string = std::move(alternative_seam_string);
                }
            }
            if (seam_string.size() < seam_align_minimum_string_seams) {
                // string NOT long enough to be worth aligning, skip
                continue;
            }

            // String is long enough, all string seams and potential string seams gathered, now do the alignment
            // sort by layer index
            std::sort(seam_string.begin(), seam_string.end(),
                      [](const std::pair<size_t, size_t> &left, const std::pair<size_t, size_t> &right) {
                          return left.first < right.first;
                      });

            // repeat the alignment for the current seam, since it could be skipped due to alternative path being aligned.
            global_index--;

            // gather all positions of seams and their weights
            observations.resize(seam_string.size());
            observation_points.resize(seam_string.size());
            weights.resize(seam_string.size());

            auto angle_3d = [](const Vec3f &a, const Vec3f &b) {
                return std::abs(acosf(a.normalized().dot(b.normalized())));
            };

            auto angle_weight = [](float angle) { return 1.0f / (0.1f + compute_angle_penalty(angle)); };

            // gather points positions and weights
            float total_length = 0.0f;
            Vec3f last_point_pos = layers[seam_string[0].first].points[seam_string[0].second].position;
            for (size_t index = 0; index < seam_string.size(); ++index) {
                const SeamCandidate &current = layers[seam_string[index].first].points[seam_string[index].second];
                float layer_angle = 0.0f;
                if (index > 0 && index < seam_string.size() - 1) {
                    layer_angle = angle_3d(
                        current.position -
                            layers[seam_string[index - 1].first].points[seam_string[index - 1].second].position,
                        layers[seam_string[index + 1].first].points[seam_string[index + 1].second].position -
                            current.position);
                }
                observations[index] = current.position.head<2>();
                observation_points[index] = current.position.z();
                weights[index] = angle_weight(current.local_ccw_angle);
                float curling_influence = layer_angle > 2.0 * std::abs(current.local_ccw_angle) ? -0.8f : 1.0f;
                if (current.type == EnforcedBlockedSeamPoint::Enforced) {
                    curling_influence = 1.0f;
                    weights[index] += 3.0f;
                }
                total_length += curling_influence * (last_point_pos - current.position).norm();
                last_point_pos = current.position;
            }

            if (comparator.setup == spRear) {
                total_length *= 0.3f;
            }

            // Curve Fitting
            size_t number_of_segments = std::max(size_t(1),
                                                 size_t(std::max(0.0f, total_length) /
                                                        SeamPlacer::seam_align_mm_per_segment));
            auto curve = Geometry::fit_cubic_bspline(observations, observation_points, weights, number_of_segments);

            // Do alignment - compute fitted point for each point in the string from its Z coord, and store the
            // position into Perimeter structure of the point; also set flag aligned to true
            for (size_t index = 0; index < seam_string.size(); ++index) {
                const auto &pair = seam_string[index];
                float t = std::min(1.0f,
                                   std::pow(std::abs(layers[pair.first].points[pair.second].local_ccw_angle) /
                                                SeamPlacer::sharp_angle_snapping_threshold,
                                            3.0f));
                if (layers[pair.first].points[pair.second].type == EnforcedBlockedSeamPoint::Enforced) {
                    t = std::max(0.4f, t);
                }

                Vec3f current_pos = layers[pair.first].points[pair.second].position;
                Vec2f fitted_pos = curve.get_fitted_value(current_pos.z());

                // interpolate between current and fitted position, prefer current pos for large weights.
                Vec3f final_position = t * current_pos + (1.0f - t) * to_3d(fitted_pos, current_pos.z());

                Perimeter &perimeter = layers[pair.first].points[pair.second].perimeter;
                perimeter.seam_index = pair.second;
                perimeter.final_seam_position = final_position;
                perimeter.finalized = true;
            }

#ifdef DEBUG_FILES
            auto randf = []() { return float(rand()) / float(RAND_MAX); };
            Vec3f color{randf(), randf(), randf()};
            for (size_t i = 0; i < seam_string.size(); ++i) {
                auto orig_seam = layers[seam_string[i].first].points[seam_string[i].second];
                fprintf(clusters, "v %f %f %f %f %f %f \n", orig_seam.position[0], orig_seam.position[1],
                        orig_seam.position[2], color[0], color[1], color[2]);
            }

            color = Vec3f{randf(), randf(), randf()};
            for (size_t i = 0; i < seam_string.size(); ++i) {
                const Perimeter &perimeter = layers[seam_string[i].first].points[seam_string[i].second].perimeter;
                fprintf(aligns, "v %f %f %f %f %f %f \n", perimeter.final_seam_position[0],
                        perimeter.final_seam_position[1], perimeter.final_seam_position[2], color[0], color[1],
                        color[2]);
            }
#endif
        }
    }

#ifdef DEBUG_FILES
    fclose(clusters);
    fclose(aligns);
#endif
}

void SeamPlacer::init(const Print &print, std::function<void(void)> throw_if_canceled_func) {
    using namespace SeamPlacerImpl;
    m_seam_per_object.clear();
    this->external_perimeters_first = print.default_region_config().external_perimeters_first;

    for (size_t obj_idx = 0; obj_idx < print.objects().size(); ++obj_idx) {
        const PrintObject *po = print.objects()[obj_idx];
        print.set_status(int((obj_idx * 100) / print.objects().size()),
                         ("Computing seam visibility areas: object %s / %s"),
                         {std::to_string(obj_idx + 1), std::to_string(print.objects().size())},
                         PrintBase::SlicingStatus::SECONDARY_STATE);
        throw_if_canceled_func();
        SeamPosition configured_seam_preference = po->config().seam_position.value;
        SeamComparator comparator{configured_seam_preference, *po};

        {
            GlobalModelInfo global_model_info{};
            gather_enforcers_blockers(global_model_info, po);
            throw_if_canceled_func();
            if (configured_seam_preference == spAligned || configured_seam_preference == spExtremlyAligned ||
                configured_seam_preference == spNearest || configured_seam_preference == spCost ||
                configured_seam_preference == spCustom) {
                compute_global_occlusion(global_model_info, po, throw_if_canceled_func);
            }
            throw_if_canceled_func();
            BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: gather_seam_candidates: start";
            gather_seam_candidates(po, global_model_info, configured_seam_preference);
            BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: gather_seam_candidates: end";
            throw_if_canceled_func();
            if (configured_seam_preference == spAligned || configured_seam_preference == spExtremlyAligned ||
                configured_seam_preference == spNearest || configured_seam_preference == spCost ||
                configured_seam_preference == spCustom) {
                BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: calculate_candidates_visibility : start";
                calculate_candidates_visibility(po, global_model_info);
                BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: calculate_candidates_visibility : end";
            }
        } // destruction of global_model_info (large structure, no longer needed)
        throw_if_canceled_func();
        BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: calculate_overhangs and layer embdedding : start";
        calculate_overhangs_and_layer_embedding(po);
        BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: calculate_overhangs and layer embdedding: end";
        throw_if_canceled_func();
        if (configured_seam_preference != spNearest && configured_seam_preference != spCost &&
            configured_seam_preference != spCustom) { // For spNearest, the seam is picked in the place_seam method
                                                      // with actual nozzle position information
            BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: pick_seam_point : start";
            // pick seam point
            std::vector<PrintObjectSeamData::LayerSeams> &layers = m_seam_per_object[po].layers;
            tbb::parallel_for(tbb::blocked_range<size_t>(0, layers.size()),
                              [&layers, configured_seam_preference, comparator, po](tbb::blocked_range<size_t> r) {
                                  for (size_t layer_idx = r.begin(); layer_idx < r.end(); ++layer_idx) {
                                      std::vector<SeamCandidate> &layer_perimeter_points = layers[layer_idx].points;
                                      for (size_t current = 0; current < layer_perimeter_points.size();
                                           current = layer_perimeter_points[current].perimeter.end_index)
                                          if (configured_seam_preference == spRandom ||
                                              configured_seam_preference == SeamPosition::spAllRandom)
                                              pick_random_seam_point(layer_perimeter_points, current, *po);
                                          else
                                              pick_seam_point(layer_perimeter_points, current, comparator);
                                  }
                              });
            BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: pick_seam_point : end";
        }
        throw_if_canceled_func();
        if (configured_seam_preference == spAligned || configured_seam_preference == spExtremlyAligned ||
            configured_seam_preference == spRear) {
            BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: align_seam_points : start";
            align_seam_points(po, comparator);
            BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: align_seam_points : end";
        }

#ifdef DEBUG_FILES
        debug_export_points(m_seam_per_object[po].layers, po->bounding_box(), comparator);
#endif
    }
}

static constexpr float MINIMAL_POLYGON_SIDE = scaled<float>(0.2f);
std::tuple<bool, std::optional<Vec3f>> get_seam_from_modifier(const Layer &layer,
                                                              const ExtrusionLoop &loop,
                                                              const uint16_t print_object_instance_idx,
                                                              const Point &last_po,
                                                              const PrintObject *po) {
    bool has_custom_seam_modifier = false;
    if (print_object_instance_idx < po->instances().size()) {
        for (ModelVolume *v : po->model_object()->volumes) {
            if (v->is_seam_position()) {
                has_custom_seam_modifier = true;
                break;
            }
        }
    }
    if (has_custom_seam_modifier) {
        Polygon polygon = loop.polygon();
        polygon.densify(MINIMAL_POLYGON_SIDE);
        bool was_clockwise = polygon.make_counter_clockwise();
        // Look for all lambda-seam-modifiers below current z, choose the highest one
        ModelVolume *v_lambda_seam = nullptr;
        Vec3d lambda_pos;
        double lambda_z = 0;
        double lambda_dist = 0;
        double lambda_radius = 0;
        double max_lambda_radius = 0;
        // get model_instance (like from po->model_object()->instances, but we don't have the index for that array)
        const ModelInstance *model_instance = po->instances()[print_object_instance_idx].model_instance;
        for (ModelVolume *v : po->model_object()->volumes) {
            if (v->is_seam_position()) {
                // xy in object coordinates, z in plater coordinates
                // created/moved shpere have offset in their transformation, and loaded ones have their loaded
                // transformation in the source transformation.
                Vec3d test_lambda_pos =
                    model_instance->transform_vector((v->get_transformation() * v->source.transform).get_offset(),
                                                     false);
                // remove shift, as we used the transform_vector(.., FALSE). that way, we have a correct z vs the
                // layer height, and same for the x and y vs polygon.
                test_lambda_pos.x() -= unscaled(po->instances()[print_object_instance_idx].shift.x());
                test_lambda_pos.y() -= unscaled(po->instances()[print_object_instance_idx].shift.y());

                double test_lambda_z = std::abs(layer.print_z - test_lambda_pos.z());
                Point xy_lambda(scale_(test_lambda_pos.x()), scale_(test_lambda_pos.y()));
                Point nearest = polygon.point_projection(xy_lambda).first;
                Vec3d polygon_3dpoint{unscaled(nearest.x()), unscaled(nearest.y()), (double) layer.print_z};
                double test_lambda_dist = (polygon_3dpoint - test_lambda_pos).norm();
                double sphere_radius = po->model_object()->instance_bounding_box(0, true).size().x() / 2;
                max_lambda_radius = std::max(max_lambda_radius, sphere_radius);

                // use this one if the first or nearer (in z, or in xy if same z)
                if (v_lambda_seam == nullptr || (lambda_z > test_lambda_z) ||
                    (lambda_z == test_lambda_z && lambda_dist > test_lambda_dist)) {
                    v_lambda_seam = v;
                    lambda_pos = test_lambda_pos;
                    lambda_radius = sphere_radius;
                    lambda_dist = test_lambda_dist;
                    lambda_z = test_lambda_z;
                }
            }
        }

        if (v_lambda_seam != nullptr) {
            // Found, get the center point and apply rotation and scaling of Model instance. Continues to spAligned if
            // not found or Weight set to Zero. Weight is set by user and stored in the radius of the sphere
            float seam_mod_weight = 1.f + 100 * (lambda_radius / max_lambda_radius);
            if (seam_mod_weight > 0.0) {
                return std::tuple<bool, std::optional<Vec3f>>{true,
                                                              std::optional{Vec3f{(float) lambda_pos.x(),
                                                                                  (float) lambda_pos.y(),
                                                                                  (float) layer.slice_z}}};
            }
        }
    }
    return std::tuple<bool, std::optional<Vec3f>>{false, std::nullopt};
}

Point SeamPlacer::place_seam(const Layer *layer,
                             const ExtrusionLoop &loop,
                             const uint16_t print_object_instance_idx,
                             const Point &last_pos) const {
    using namespace SeamPlacerImpl;
    const PrintObject *po = layer->object();
    // Must not be called with supprot layer.
    assert(dynamic_cast<const SupportLayer *>(layer) == nullptr);
    // Object layer IDs are incremented by the number of raft layers.
    assert(layer->id() >= po->slicing_parameters().raft_layers());
    const size_t layer_index = layer->id() - po->slicing_parameters().raft_layers();
    const double unscaled_z = layer->slice_z;

    // FIXME?: not working on arcs
    auto get_next_loop_point = [&loop](ExtrusionLoop::ClosestPathPoint current) {
        current.segment_idx += 1;
        if (current.segment_idx >= loop.paths[current.path_idx].polyline.size()) {
            current.path_idx = next_idx_modulo(current.path_idx, loop.paths.size());
            current.segment_idx = 0;
        }
        current.foot_pt = loop.paths[current.path_idx].polyline.get_point(current.segment_idx);
        return current;
    };

    const PrintObjectSeamData::LayerSeams &layer_perimeters = m_seam_per_object.find(layer->object())
                                                                  ->second.layers[layer_index];

    // Find the closest perimeter in the SeamPlacer to this loop.
    // Repeat search until two consecutive points of the loop are found, that result in the same closest_perimeter
    // This is beacuse with arachne, T-Junctions may exist and sometimes the wrong perimeter was chosen
    size_t closest_perimeter_point_index = 0;
    { // local space for the closest_perimeter_point_index
        Perimeter *closest_perimeter = nullptr;
        ExtrusionLoop::ClosestPathPoint closest_point{0, 0, loop.paths[0].polyline.front()};
        size_t points_count = std::accumulate(loop.paths.begin(), loop.paths.end(), 0,
                                              [](size_t acc, const ExtrusionPath &p) {
                                                  return acc + p.polyline.size();
                                              });
        for (size_t i = 0; i < points_count; ++i) {
            Vec2f unscaled_p = unscaled<float>(closest_point.foot_pt);
            closest_perimeter_point_index = find_closest_point(*layer_perimeters.points_tree.get(),
                                                               to_3d(unscaled_p, float(unscaled_z)));
            if (closest_perimeter != &layer_perimeters.points[closest_perimeter_point_index].perimeter) {
                closest_perimeter = &layer_perimeters.points[closest_perimeter_point_index].perimeter;
                closest_point = get_next_loop_point(closest_point);
            } else {
                break;
            }
        }
    }

    Vec3f seam_position;
    size_t seam_index;
    if (const Perimeter &perimeter = layer_perimeters.points[closest_perimeter_point_index].perimeter;
        perimeter.finalized) {
        seam_position = perimeter.final_seam_position;
        seam_index = perimeter.seam_index;
    } else {
        seam_index = (po->config().seam_position.value == spNearest || po->config().seam_position.value == spCost ||
                      po->config().seam_position.value == spCustom) ?
            pick_nearest_seam_point_index(layer_perimeters.points, perimeter.start_index, unscaled<float>(last_pos),
                                          *po) :
            perimeter.seam_index;
        seam_position = layer_perimeters.points[seam_index].position;
    }

    Point seam_point = Point::new_scale(seam_position.x(), seam_position.y());

    auto [has_seam_mod, seam_mod_pos] = get_seam_from_modifier(*layer, loop, print_object_instance_idx, last_pos, po);
    if (has_seam_mod && seam_mod_pos.has_value()) {
        seam_point = Point::new_scale(seam_mod_pos->x(), seam_mod_pos->y());
    } else {
        if (loop.role() == ExtrusionRole::Perimeter) { // Hopefully inner perimeter
            const SeamCandidate &perimeter_point = layer_perimeters.points[seam_index];
            ExtrusionLoop::ClosestPathPoint projected_point = loop.get_closest_path_and_point(seam_point, false);
            // determine depth of the seam point.
            float depth = (float) unscale(Point(seam_point - projected_point.foot_pt)).norm();
            float beta_angle = cos(perimeter_point.local_ccw_angle / 2.0f);
            size_t index_of_prev = seam_index == perimeter_point.perimeter.start_index ?
                perimeter_point.perimeter.end_index - 1 :
                seam_index - 1;
            size_t index_of_next = seam_index == perimeter_point.perimeter.end_index - 1 ?
                perimeter_point.perimeter.start_index :
                seam_index + 1;

            if ((seam_position - perimeter_point.position).squaredNorm() < depth && // seam is on perimeter point
                perimeter_point.local_ccw_angle < -EPSILON                          // In concave angles
            ) { // In this case, we are at internal perimeter, where the external perimeter has seam in concave angle.
                // We want to align
                // the internal seam into the concave corner, and not on the perpendicular projection on the closest
                // edge (which is what the split_at function does)
                Vec2f dir_to_middle = ((perimeter_point.position - layer_perimeters.points[index_of_prev].position)
                                           .head<2>()
                                           .normalized() +
                                       (perimeter_point.position - layer_perimeters.points[index_of_next].position)
                                           .head<2>()
                                           .normalized()) *
                    0.5;
                depth = 1.4142 * depth / beta_angle;
                // There are some nice geometric identities in determination of the correct depth of new seam point.
                // overshoot the target depth, in concave angles it will correctly snap to the corner; TODO: find out
                // why such big overshoot is needed.
                Vec2f final_pos = perimeter_point.position.head<2>() + depth * dir_to_middle;
                projected_point = loop.get_closest_path_and_point(Point::new_scale(final_pos.x(), final_pos.y()),
                                                                  false);
            } else { // not concave angle, in that case the nearest point is the good candidate
                // but for staggering, we also need to recompute depth of the inner perimter, because in convex corners,
                // the distance is larger than layer width we want the perpendicular depth, not distance to nearest point
                depth = depth * beta_angle / 1.4142;
            }

            seam_point = projected_point.foot_pt;
            // lastly, for internal perimeters, do the staggering if requested
            if (po->config().staggered_inner_seams && loop.length() > 0.0) {
                // fix depth, it is sometimes strongly underestimated
                depth = std::max(loop.paths[projected_point.path_idx].width(), depth);

                while (depth > 0.0f) {
                    auto next_point = get_next_loop_point(projected_point);
                    Vec2f a = unscale(projected_point.foot_pt).cast<float>();
                    Vec2f b = unscale(next_point.foot_pt).cast<float>();
                    float dist = (a - b).norm();
                    if (dist > depth) {
                        Vec2f final_pos = a + (b - a) * depth / dist;
                        next_point.foot_pt = Point::new_scale(final_pos.x(), final_pos.y());
                    }
                    depth -= dist;
                    projected_point = next_point;
                }
                seam_point = projected_point.foot_pt;
            }
            /*{
                static int isaqsdsdfsdfqzfn = 0;
                std::stringstream stri;
                stri << layer->id() << "_split_seam_" << isaqsdsdfsdfqzfn++ << ".svg";
                SVG svg(stri.str());
                for(auto& path : loop.paths)
                    svg.draw(path.polyline, "blue");
                svg.draw(seam_point, "red");
                svg.Close();
            }*/
        }
    }
    return seam_point;
}

} // namespace Slic3r