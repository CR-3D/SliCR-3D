///|/ Copyright (c) Prusa Research 2019 - 2023 Enrico Turri @enricoturri1966, Filip Sykala @Jony01, Vojtěch Bubník @bubnikv, Lukáš Matěna @lukasmatena
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_3DBed_hpp_
#define slic3r_3DBed_hpp_

#include "GLTexture.hpp"
#include "3DScene.hpp"
#include "CoordAxes.hpp"
#include "MeshUtils.hpp"

#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/ExPolygon.hpp"

#include <tuple>
#include <array>

namespace Slic3r {
namespace GUI {

class GLCanvas3D;

class Bed3D
{
public:
    enum class Type : unsigned char
    {
        // The print bed model and texture are available from some printer preset.
        System,
        // The print bed model is unknown, thus it is rendered procedurally.
        Custom
    };

private:
    BuildVolume m_build_volume;
    Type m_type{ Type::Custom };
    // m_texture_filename can be relative or absolute
    std::string m_texture_filename;
    // absolute path for m_texture_filename
    boost::filesystem::path m_texture_path;
    bool m_texture_with_grid = false;
    // m_model_filename can be relative or absolute
    std::string m_model_filename;
    // absolute path for m_model_filename
    boost::filesystem::path m_model_path;
    // Print volume bounding box exteded with axes and model.
    BoundingBoxf3 m_extended_bounding_box;
    // Print bed polygon
    ExPolygon m_contour;
    // Slightly expanded print bed polygon, for collision detection.
    Polygon m_polygon;
    Pointfs m_exclude_area;
    Pointfs m_exclude_areas;
    GLModel m_exclude_triangles;

    GLModel m_triangles;
    GLModel m_gridlines;
    GLModel m_gridlines_big;
    GLModel m_gridlines_small;
    GLModel m_contourlines;
    mutable GLTexture m_texture;
    ColorRGBA m_model_color{ 0.235f, 0.235f, 0.235f, 1.0f };
    ColorRGBA m_grid_color{ 0.9f, 0.9f, 0.9f, 0.6f };

    // temporary texture shown until the main texture has still no levels compressed
    GLTexture m_temp_texture;
    PickingModel m_model;
    Vec3d m_model_offset{ Vec3d::Zero() };
    CoordAxes m_axes;

    float m_scale_factor{ 1.0f };

public:
    Bed3D();
    ~Bed3D() = default;

    // Update print bed model from configuration.
    // Return true if the bed shape changed, so the calee will update the UI.
    //FIXME if the build volume max print height is updated, this function still returns zero
    // as this class does not use it, thus there is no need to update the UI.
    bool set_shape(const Pointfs& bed_shape, const double max_print_height, const std::string& custom_texture, const std::string& custom_model, bool force_as_custom = false);
    bool set_shape(const Pointfs& shape, const Pointfs& exclude_areas, Vec2d position, const std::string& custom_texture, const std::string& custom_model, bool force_as_custom);
    void generate_exclude_polygon(ExPolygon &exclude_polygon);
    void calc_exclude_triangles(const ExPolygon &poly);
    void render_exclude_area(bool force_default_color);
    bool check_outside(int obj_id, int instance_id, BoundingBoxf3* bounding_box);
    bool preprocess_exclude_areas(arrangement::ArrangePolygons& unselected, int num_plates, float inflation);

    // Build volume geometry for various collision detection tasks.
    const BuildVolume& build_volume() const { return m_build_volume; }

    // Was the model provided, or was it generated procedurally?
    Type get_type() const { return m_type; }
    // Was the model generated procedurally?
    bool is_custom() const { return m_type == Type::Custom; }

    // Bounding box around the print bed, axes and model, for rendering.
    const BoundingBoxf3& extended_bounding_box() const { return m_extended_bounding_box; }

    // Check against an expanded 2d bounding box.
    //FIXME shall one check against the real build volume?
    bool contains(const Point& point) const;
    Point point_projection(const Point& point) const;

    void render(GLCanvas3D& canvas, const Transform3d& view_matrix, const Transform3d& projection_matrix, bool bottom, float scale_factor, bool show_texture);
    void render_axes();
    void render_for_picking(GLCanvas3D& canvas, const Transform3d& view_matrix, const Transform3d& projection_matrix, bool bottom, float scale_factor);

    Pointfs get_exclude_area() { return m_exclude_areas; }


private:
    // Calculate an extended bounding box from axes and current model for visualization purposes.
    BoundingBoxf3 calc_extended_bounding_box() const;
    void init_triangles();
    void init_gridlines();
    void init_contourlines();
    static std::tuple<Type, std::string, std::string, bool> detect_type(const Pointfs& shape);
    void render_internal(GLCanvas3D& canvas, const Transform3d& view_matrix, const Transform3d& projection_matrix, bool bottom, float scale_factor,
        bool show_texture, bool picking);
    void render_system(GLCanvas3D& canvas, const Transform3d& view_matrix, const Transform3d& projection_matrix, bool bottom, bool show_texture);
    void render_texture(bool bottom, GLCanvas3D& canvas, const Transform3d& view_matrix, const Transform3d& projection_matrix);
    void render_model(const Transform3d& view_matrix, const Transform3d& projection_matrix);
    void render_custom(GLCanvas3D& canvas, const Transform3d& view_matrix, const Transform3d& projection_matrix, bool bottom, bool show_texture, bool picking);
    void render_default(bool bottom, bool picking, bool show_texture, const Transform3d& view_matrix, const Transform3d& projection_matrix);
    void render_contour(const Transform3d& view_matrix, const Transform3d& projection_matrix);
    void render_grid(bool bottom, bool has_model);
    
    void register_raycasters_for_picking(const GLModel::Geometry& geometry, const Transform3d& trafo);
};

} // GUI
} // Slic3r

#endif // slic3r_3DBed_hpp_
