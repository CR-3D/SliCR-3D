///|/ Copyright (c) Prusa Research 2019 - 2023 Enrico Turri @enricoturri1966, Vojtěch Bubník @bubnikv, Filip Sykala @Jony01, Lukáš Matěna @lukasmatena, Oleksandra Iushchenko @YuSanka
///|/ Copyright (c) 2022 Michael Kirsch
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "libslic3r/libslic3r.h"

#include "3DBed.hpp"

#include "libslic3r/Polygon.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/GCode/PostProcessor.hpp"
#include "libslic3r/Geometry/Circle.hpp"
#include "libslic3r/Tesselate.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/Arrange.hpp"

#include "GUI.hpp"
#include "GUI_App.hpp"
#include "GLCanvas3D.hpp"
#include "Plater.hpp"
#include "Camera.hpp"

#include <GL/glew.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/locale/generator.hpp>
#include <boost/log/trivial.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>

static const float GROUND_Z = -0.02f;
static const Slic3r::ColorRGBA DEFAULT_MODEL_COLOR             = Slic3r::ColorRGBA::DARK_GRAY();
static const Slic3r::ColorRGBA PICKING_MODEL_COLOR             = Slic3r::ColorRGBA::BLACK();
static const Slic3r::ColorRGBA DEFAULT_SOLID_GRID_COLOR        = { 0.9f, 0.9f, 0.9f, 1.0f };
static const Slic3r::ColorRGBA DEFAULT_TRANSPARENT_GRID_COLOR  = { 0.9f, 0.9f, 0.9f, 0.6f };
static const Slic3r::ColorRGBA HEIGHT_LIMIT_TOP_COLOR		= { 0.6f, 0.6f, 1.0f, 1.0f };
static const Slic3r::ColorRGBA HEIGHT_LIMIT_BOTTOM_COLOR	= { 0.4f, 0.4f, 1.0f, 1.0f };

static const float GROUND_Z_GRIDLINE = -0.26f;
static const float GRABBER_X_FACTOR = 0.20f;
static const float GRABBER_Y_FACTOR = 0.03f;
static const float GRABBER_Z_VALUE = 0.5f;

namespace Slic3r {
namespace GUI {

Bed3D::Bed3D()
{
    this->m_model_color = DEFAULT_MODEL_COLOR;
    this->m_grid_color = DEFAULT_TRANSPARENT_GRID_COLOR;
    m_selected = false;
    {
        //try to load color from ui file
        boost::property_tree::ptree tree_colors;
        boost::filesystem::path path_colors = Slic3r::GUI::get_app_config()->layout_config_path() / "colors.ini";
        try {
            boost::nowide::ifstream ifs;
            ifs.imbue(boost::locale::generator()("en_US.UTF-8"));
            ifs.open(path_colors.string());
            boost::property_tree::read_ini(ifs, tree_colors);

            std::string color_code = tree_colors.get<std::string>("Gui_plater");
            if (color_code.length() > 5) {
                wxColour color;
                color.Set((color_code[0] == '#') ? color_code : ("#" + color_code));
                this->m_model_color.r(color.Red() / 256.f);
                this->m_model_color.g(color.Green() / 256.f);
                this->m_model_color.b(color.Blue() / 256.f);
            }
            color_code = tree_colors.get<std::string>("Gui_plater_grid");
            if (color_code.length() > 5) {
                wxColour color;
                color.Set((color_code[0] == '#') ? color_code : ("#" + color_code));
                this->m_grid_color.r(color.Red() / 256.f);
                this->m_grid_color.g(color.Green() / 256.f);
                this->m_grid_color.b(color.Blue() / 256.f);
            }
        }
        catch (const std::ifstream::failure& err) {
            BOOST_LOG_TRIVIAL(warning) << "The color file cannot be loaded. Path: '"<< path_colors.string()<<"'. Reason: " << err.what() ;
        }
        catch (const std::runtime_error& err) {
            BOOST_LOG_TRIVIAL(warning) << "Fail loading the color file Path: '"<< path_colors.string()<<"'. Reason: " << err.what() ;
        }
    }
}

bool init_model_from_poly(GLModel &model, const ExPolygon &poly, float z)
{
    if (poly.empty())
        return false;

    const std::vector<Vec2f> triangles = triangulate_expolygon_2f(poly, NORMALS_UP);
    if (triangles.empty() || triangles.size() % 3 != 0)
        return false;

    GLModel::Geometry init_data;
    init_data.format = { GLModel::Geometry::EPrimitiveType::Triangles, GLModel::Geometry::EVertexLayout::P3T2 };
    init_data.reserve_vertices(triangles.size());
    init_data.reserve_indices(triangles.size() / 3);

    Vec2f min = triangles.front();
    Vec2f max = min;
    for (const Vec2f &v : triangles) {
        min = min.cwiseMin(v).eval();
        max = max.cwiseMax(v).eval();
    }

    const Vec2f size = max - min;
    if (size.x() <= 0.0f || size.y() <= 0.0f)
        return false;

    Vec2f inv_size = size.cwiseInverse();
    inv_size.y() *= -1.0f;

    // vertices + indices
    unsigned int vertices_counter = 0;
    for (const Vec2f &v : triangles) {
        const Vec3f p = {v.x(), v.y(), z};
        init_data.add_vertex(p, (Vec2f)(v - min).cwiseProduct(inv_size).eval());
        ++vertices_counter;
        if (vertices_counter % 3 == 0)
            init_data.add_triangle(vertices_counter - 3, vertices_counter - 2, vertices_counter - 1);
    }

    model.init_from(std::move(init_data));

    return true;
}



bool Bed3D::set_shape(const Pointfs& bed_shape, const double max_print_height, const std::string& custom_texture, const std::string& custom_model, bool force_as_custom)
{
    auto check_texture = [](const std::string& texture) {
        boost::system::error_code ec; // so the exists call does not throw (e.g. after a permission problem)
        return !texture.empty() && (boost::algorithm::iends_with(texture, ".png") || boost::algorithm::iends_with(texture, ".svg")) && boost::filesystem::exists(Slic3r::find_full_path(texture), ec);
    };

    auto check_model = [](const std::string& model) {
        boost::system::error_code ec;
        return !model.empty() && boost::algorithm::iends_with(model, ".stl") && boost::filesystem::exists(Slic3r::find_full_path(model), ec);
    };

    Type type;
    std::string model;
    std::string texture;
    if (force_as_custom)
        type = Type::Custom;
    else {
        auto [new_type, system_model, system_texture, system_with_grid] = detect_type(bed_shape);
        type = new_type;
        model = system_model;
        texture = system_texture;
        m_texture_with_grid = system_with_grid;
    }

    std::string texture_filename = custom_texture.empty() ? texture : custom_texture;
    if (! texture_filename.empty() && ! check_texture(texture_filename)) {
        BOOST_LOG_TRIVIAL(error) << "Unable to load bed texture: " << texture_filename;
        texture_filename.clear();
    }

    std::string model_filename = custom_model.empty() ? model : custom_model;
    if (! model_filename.empty() && ! check_model(model_filename)) {
        BOOST_LOG_TRIVIAL(error) << "Unable to load bed model: " << model_filename;
        model_filename.clear();
    }

    
    if (m_build_volume.bed_shape() == bed_shape && m_build_volume.max_print_height() == max_print_height && m_type == type && m_texture_filename == texture_filename && m_model_filename == model_filename)
        // No change, no need to update the UI.
        return false;

    m_type = type;
    m_build_volume = BuildVolume { bed_shape, max_print_height };
    m_texture_filename = texture_filename;
    m_texture_path = Slic3r::find_full_path(m_texture_filename, m_texture_filename);
    m_model_filename = model_filename;
    m_model_path = Slic3r::find_full_path(m_model_filename, m_model_filename);
    m_extended_bounding_box = this->calc_extended_bounding_box();

    m_contour = ExPolygon(Polygon::new_scale(bed_shape));
    const BoundingBox bbox = m_contour.contour.bounding_box();
    if (!bbox.defined)
        throw RuntimeError(std::string("Invalid bed shape"));
    m_polygon = offset(m_contour.contour, (float)bbox.radius() * 1.7f, jtRound, scale_(0.5)).front();

    m_triangles.reset();
    m_gridlines.reset();
    m_gridlines_big.reset();
    m_gridlines_small.reset();
    m_contourlines.reset();
    m_texture.reset();
    m_model.reset();

    // Set the origin and size for rendering the coordinate system axes.
    m_axes.set_origin({ 0.0, 0.0, static_cast<double>(GROUND_Z) });
    m_axes.set_stem_length(0.1f * static_cast<float>(m_build_volume.bounding_volume().max_size()));

    // unregister from picking
    wxGetApp().plater()->canvas3D()->remove_raycasters_for_picking(SceneRaycaster::EType::Bed);

    // Let the calee to update the UI.
    return true;
}

// Orca: Exclude Areas Rendering

//check whether instance is outside the plate or not
bool Bed3D::check_outside(int obj_id, int instance_id, BoundingBoxf3* bounding_box)
{
	bool outside = true;
 
 Model& model = wxGetApp().plater_->model();
 ModelObject* object = model.objects[obj_id];
	ModelInstance* instance = object->instances[instance_id];

	BoundingBoxf3 instance_box = bounding_box? *bounding_box: object->instance_convex_hull_bounding_box(instance_id);
	Polygon hull = instance->convex_hull_2d();
	BoundingBoxf3 plate_box = get_plate_box();
   
	if (instance_box.max.z() > plate_box.min.z())
		plate_box.min.z() += instance_box.min.z(); // not considering outsize if sinking

	if (plate_box.contains(instance_box))
	{
		if (m_exclude_bounding_box.size() > 0)
		{
			Polygon hull = instance->convex_hull_2d();
			int index;
			for (index = 0; index < m_exclude_bounding_box.size(); index ++)
			{
				Polygon p = m_exclude_bounding_box[index].polygon(true);  // instance convex hull is scaled, so we need to scale here
				if (intersection({ p }, { hull }).empty() == false)
				{
					break;
				}
			}
			if (index >= m_exclude_bounding_box.size())
				outside = false;
		}
		else
			outside = false;
	}

	return outside;
}

void Bed3D::render_exclude_area(bool force_default_color) {
	if (force_default_color) //for thumbnail case
		return;

	ColorRGBA select_color{ 0.765f, 0.7686f, 0.7686f, 1.0f };
	ColorRGBA unselect_color{ 0.9f, 0.9f, 0.9f, 1.0f };
	//ColorRGBA default_color{ 0.9f, 0.9f, 0.9f, 1.0f };

	// draw exclude area
	glsafe(::glDepthMask(GL_FALSE));

	if (m_selected) {
		glsafe(::glColor4fv(select_color.data()));
	}
	else {
		glsafe(::glColor4fv(unselect_color.data()));
	}

	m_exclude_triangles.set_color(m_selected ? select_color : unselect_color);
    m_exclude_triangles.render();
	glsafe(::glDepthMask(GL_TRUE));
}

void Bed3D::calc_exclude_triangles(const ExPolygon &poly)
{
    m_exclude_triangles.reset();

    if (!init_model_from_poly(m_exclude_triangles, poly, GROUND_Z))
		BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << ":Unable to create exclude triangles\n";
}

void Bed3D::generate_exclude_polygon(ExPolygon &exclude_polygon)
{
	auto compute_exclude_points = [&exclude_polygon](Vec2d& center, double radius, double start_angle, double stop_angle, int count)
	{
		double angle_steps;
		angle_steps = (stop_angle - start_angle) / (count - 1);
		for(int j = 0; j < count; j++ )
		{
			double angle = start_angle + j * angle_steps;
			double x = center(0) + ::cos(angle) * radius;
			double y = center(1) + ::sin(angle) * radius;
			exclude_polygon.contour.append({ scale_(x), scale_(y) });
		}
	};

	int points_count = 8;
	if (m_exclude_area.size() == 4)
	{
			//rectangle case
			for (int i = 0; i < 4; i++)
			{
				const Vec2d& p = m_exclude_area[i];
				Vec2d center;
				double start_angle, stop_angle, radius;
				switch (i) {
					case 0:
						radius = 5.f;
						center(0) = p(0) + radius;
						center(1) = p(1) + radius;
						start_angle = PI;
						stop_angle = 1.5 * PI;
						compute_exclude_points(center, radius, start_angle, stop_angle, points_count);
						break;
					case 1:
						exclude_polygon.contour.append({ scale_(p(0)), scale_(p(1)) });
						break;
					case 2:
						radius = 3.f;
						center(0) = p(0) - radius;
						center(1) = p(1) - radius;
						start_angle = 0;
						stop_angle = 0.5 * PI;
						compute_exclude_points(center, radius, start_angle, stop_angle, points_count);
						break;
					case 3:
						exclude_polygon.contour.append({ scale_(p(0)), scale_(p(1)) });
						break;
				}
			}
	}
	else {
		for (const Vec2d& p : m_exclude_area) {
			exclude_polygon.contour.append({ scale_(p(0)), scale_(p(1)) });
		}
	}
}

bool Bed3D::preprocess_exclude_areas(arrangement::ArrangePolygons& unselected, float inflation) {
	bool added = false;

	if (m_exclude_areas.size() > 0)
	{
		//has exclude areas
 // Plater *plate = wxGetApp().plater_;

		for (int index = 0; index < this->m_exclude_bounding_box.size(); index ++)
		{
			Polygon ap({
				{scaled(this->m_exclude_bounding_box[index].min.x()), scaled(this->m_exclude_bounding_box[index].min.y())},
				{scaled(this->m_exclude_bounding_box[index].max.x()), scaled(this->m_exclude_bounding_box[index].min.y())},
				{scaled(this->m_exclude_bounding_box[index].max.x()), scaled(this->m_exclude_bounding_box[index].max.y())},
				{scaled(this->m_exclude_bounding_box[index].min.x()), scaled(this->m_exclude_bounding_box[index].max.y())}
				});

				arrangement::ArrangePolygon ret;
				ret.poly.contour = ap;
				ret.translation  = Vec2crd(0, 0);
				ret.rotation     = 0.0f;
				ret.is_virt_object = true;
				ret.height      = 1;
				ret.name = "ExcludedRegion" + std::to_string(index);
				ret.inflation = inflation;

				unselected.emplace_back(std::move(ret));

			added = true;
		}
	}

	return added;
}

bool Bed3D::set_shape(const Pointfs& shape,
                   const Pointfs& exclude_areas,
                   const double max_print_height,
                   const std::string& custom_texture,
                   const std::string& custom_model,
                   bool force_as_custom)
{
    Pointfs new_shape, new_exclude_areas;
	m_raw_shape = shape;

	if (m_exclude_area == new_exclude_areas) {
		BOOST_LOG_TRIVIAL(info) << "Plate same shape, skip directly";
		return false;
	}

	if (m_exclude_area != new_exclude_areas)
	{
		m_exclude_area = std::move(new_exclude_areas);

		calc_bounding_boxes();

		ExPolygon logo_poly;
		generate_logo_polygon(logo_poly);

		ExPolygon poly;

		generate_print_polygon(poly);
		calc_triangles(poly);

		ExPolygon exclude_poly;

		generate_exclude_polygon(exclude_poly);
		calc_exclude_triangles(exclude_poly);

		const BoundingBox& pp_bbox = poly.contour.bounding_box();
		calc_gridlines(poly, pp_bbox);
	}

	calc_height_limit();

	return true;
}

void Bed3D::calc_triangles(const ExPolygon &poly)
{
    m_triangles.reset();

    if (!init_model_from_poly(m_triangles, poly, GROUND_Z))
		BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << ":Unable to create plate triangles\n";
}

void Bed3D::render_height_limit(Bed3D::HeightLimitMode mode)
{
/*
	if (m_print && m_print->config().print_sequence == PrintSequence::ByObject && mode != HEIGHT_LIMIT_NONE)
	{
		// draw lower limit
		glsafe(::glLineWidth(3.0f * m_scale_factor));
        m_height_limit_common.set_color(HEIGHT_LIMIT_BOTTOM_COLOR);
        m_height_limit_common.render();

		if ((mode == HEIGHT_LIMIT_BOTTOM) || (mode == HEIGHT_LIMIT_BOTH)) {
			glsafe(::glLineWidth(3.0f * m_scale_factor));
            m_height_limit_bottom.set_color(HEIGHT_LIMIT_BOTTOM_COLOR);
            m_height_limit_bottom.render();
		}

		// draw upper limit
		if ((mode == HEIGHT_LIMIT_TOP) || (mode == HEIGHT_LIMIT_BOTH)){
            glsafe(::glLineWidth(3.0f * m_scale_factor));
            m_height_limit_top.set_color(HEIGHT_LIMIT_TOP_COLOR);
            m_height_limit_top.render();
		}
	}
 */
}

void Bed3D::calc_height_limit() {
    m_height_limit_common.reset();
    m_height_limit_bottom.reset();
    m_height_limit_top.reset();
/*
	Lines3 bottom_h_lines, top_lines, top_h_lines, common_lines;
	int shape_count = m_shape.size();
	float first_z = 0.02f;
	for (int i = 0; i < shape_count; i++) {
		auto &cur_p = m_shape[i];
		Vec3crd p1(scale_(cur_p.x()), scale_(cur_p.y()), scale_(first_z));
		Vec3crd p2(scale_(cur_p.x()), scale_(cur_p.y()), scale_(m_height_to_rod));
		Vec3crd p3(scale_(cur_p.x()), scale_(cur_p.y()), scale_(m_height_to_lid));

		common_lines.emplace_back(p1, p2);
		top_lines.emplace_back(p2, p3);

		Vec2d next_p;
		if (i < (shape_count - 1)) {
			next_p = m_shape[i+1];

		}
		else {
			next_p = m_shape[0];
		}
		Vec3crd p4(scale_(cur_p.x()), scale_(cur_p.y()), scale_(m_height_to_rod));
		Vec3crd p5(scale_(next_p.x()), scale_(next_p.y()), scale_(m_height_to_rod));
		bottom_h_lines.emplace_back(p4, p5);

		Vec3crd p6(scale_(cur_p.x()), scale_(cur_p.y()), scale_(m_height_to_lid));
		Vec3crd p7(scale_(next_p.x()), scale_(next_p.y()), scale_(m_height_to_lid));
		top_h_lines.emplace_back(p6, p7);
	}
	//std::copy(bottom_lines.begin(), bottom_lines.end(), std::back_inserter(bottom_h_lines));
	std::copy(top_lines.begin(), top_lines.end(), std::back_inserter(top_h_lines));

	if (!init_model_from_lines(m_height_limit_common, common_lines))
		BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << "Unable to create height limit bottom lines\n";

	if (!init_model_from_lines(m_height_limit_bottom, bottom_h_lines))
		BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << "Unable to create height limit bottom lines\n";

	if (!init_model_from_lines(m_height_limit_top, top_h_lines))
		BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << "Unable to create height limit top lines\n";
  */
}

void Bed3D::calc_gridlines(const ExPolygon& poly, const BoundingBox& pp_bbox) {
    m_gridlines.reset();
    m_gridlines_bolder.reset();

	Polylines axes_lines, axes_lines_bolder;
	int count = 0;
	int step  = 10;
	// Orca: use 500 x 500 bed size as baseline.
    const Point grid_counts = pp_bbox.size() / ((coord_t) scale_(step * 50));
    // if the grid is too dense, we increase the step
    if (grid_counts.minCoeff() > 1) {
        step = static_cast<int>(grid_counts.minCoeff() + 1) * 10;
    }
    for (coord_t x = pp_bbox.min(0); x <= pp_bbox.max(0); x += scale_(step)) {
		Polyline line;
		line.append(Point(x, pp_bbox.min(1)));
		line.append(Point(x, pp_bbox.max(1)));

		if ( (count % 5) == 0 )
			axes_lines_bolder.push_back(line);
		else
			axes_lines.push_back(line);
		count ++;
	}
	count = 0;
	for (coord_t y = pp_bbox.min(1); y <= pp_bbox.max(1); y += scale_(step)) {
		Polyline line;
		line.append(Point(pp_bbox.min(0), y));
		line.append(Point(pp_bbox.max(0), y));
		axes_lines.push_back(line);

		if ( (count % 5) == 0 )
			axes_lines_bolder.push_back(line);
		else
			axes_lines.push_back(line);
		count ++;
	}

	// clip with a slightly grown expolygon because our lines lay on the contours and may get erroneously clipped
	Lines gridlines = to_lines(intersection_pl(axes_lines, offset(poly, (float)SCALED_EPSILON)));
	Lines gridlines_bolder = to_lines(intersection_pl(axes_lines_bolder, offset(poly, (float)SCALED_EPSILON)));

	// append bed contours
	Lines contour_lines = to_lines(poly);
	std::copy(contour_lines.begin(), contour_lines.end(), std::back_inserter(gridlines));

/*
	if (!init_model_from_lines(m_gridlines, gridlines, GROUND_Z_GRIDLINE))
		BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << "Unable to create bed grid lines\n";

	if (!init_model_from_lines(m_gridlines_bolder, gridlines_bolder, GROUND_Z_GRIDLINE))
		BOOST_LOG_TRIVIAL(error) << __FUNCTION__ << "Unable to create bed grid lines\n";
  */
  
}

void Bed3D::generate_print_polygon(ExPolygon &print_polygon)
{
	auto compute_points = [&print_polygon](Vec2d& center, double radius, double start_angle, double stop_angle, int count)
	{
		double angle_steps;
		angle_steps = (stop_angle - start_angle) / (count - 1);
		for(int j = 0; j < count; j++ )
		{
			double angle = start_angle + j * angle_steps;
			double x = center(0) + ::cos(angle) * radius;
			double y = center(1) + ::sin(angle) * radius;
			print_polygon.contour.append({ scale_(x), scale_(y) });
		}
	};
}


void Bed3D::generate_logo_polygon(ExPolygon &logo_polygon)
{
	if (m_shape.size() == 4)
	{
        bool is_bbl_vendor = false;

        //rectangle case
		for (int i = 0; i < 4; i++)
		{
			const Vec2d& p = m_shape[i];
			if ((i  == 0) || (i  == 1)) {
                logo_polygon.contour.append({scale_(p(0)), scale_(is_bbl_vendor ? p(1) - 12.f : p(1))});
            }
			else {
				logo_polygon.contour.append({ scale_(p(0)), scale_(p(1)) });
			}
		}
	}
	else {
		for (const Vec2d& p : m_shape) {
			logo_polygon.contour.append({ scale_(p(0)), scale_(p(1)) });
		}
	}
}

void Bed3D::calc_bounding_boxes() const {
	BoundingBoxf3* bounding_box = const_cast<BoundingBoxf3*>(&m_bounding_box);
	*bounding_box = BoundingBoxf3();
   
	for (const Vec2d& p : m_shape) {
		bounding_box->merge({ p(0), p(1), 0.0 });
	}

	BoundingBoxf3* extended_bounding_box = const_cast<BoundingBoxf3*>(&m_extended_bounding_box);
	*extended_bounding_box = m_bounding_box;

	double half_x = bounding_box->size().x() * GRABBER_X_FACTOR;
	double half_y = bounding_box->size().y() * 1.0f * GRABBER_Y_FACTOR;
	double half_z = GRABBER_Z_VALUE;
	Vec3d center(bounding_box->center().x(), bounding_box->min(1) -half_y, GROUND_Z);
	m_grabber_box.min = Vec3d(center.x() - half_x, center.y() - half_y, center.z() - half_z);
	m_grabber_box.max = Vec3d(center.x() + half_x, center.y() + half_y, center.z() + half_z);
	m_grabber_box.defined = true;
	extended_bounding_box->merge(m_grabber_box);

    //calc exclude area bounding box
    m_exclude_bounding_box.clear();
    BoundingBoxf3 exclude_bb;
    for (int index = 0; index < m_exclude_area.size(); index ++) {
		const Vec2d& p = m_exclude_area[index];

		if (index % 4 == 0)
			exclude_bb = BoundingBoxf3();

		exclude_bb.merge({ p(0), p(1), 0.0 });

		if (index % 4 == 3)
		{
			exclude_bb.max(2) = m_depth;
			exclude_bb.min(2) = GROUND_Z;
			m_exclude_bounding_box.emplace_back(exclude_bb);
		}
	}
}



bool Bed3D::contains(const Point& point) const
{
    return m_polygon.contains(point);
}

Point Bed3D::point_projection(const Point& point) const
{
    return m_polygon.point_projection(point).first;
}

void Bed3D::render(GLCanvas3D& canvas, const Transform3d& view_matrix, const Transform3d& projection_matrix, bool bottom, float scale_factor, bool show_texture)
{
    render_internal(canvas, view_matrix, projection_matrix, bottom, scale_factor, show_texture, false);
}

void Bed3D::render_for_picking(GLCanvas3D& canvas, const Transform3d& view_matrix, const Transform3d& projection_matrix, bool bottom, float scale_factor)
{
    render_internal(canvas, view_matrix, projection_matrix, bottom, scale_factor, false, true);
}

void Bed3D::render_internal(GLCanvas3D& canvas, const Transform3d& view_matrix, const Transform3d& projection_matrix, bool bottom, float scale_factor,
    bool show_texture, bool picking)
{
    m_scale_factor = scale_factor;

    glsafe(::glEnable(GL_DEPTH_TEST));

    m_model.model.set_color(picking ? PICKING_MODEL_COLOR : DEFAULT_MODEL_COLOR);

    switch (m_type)
    {
    case Type::System: { render_system(canvas, view_matrix, projection_matrix, bottom, show_texture); break; }
    default:
    case Type::Custom: { render_custom(canvas, view_matrix, projection_matrix, bottom, show_texture, picking); break; }
    }

    glsafe(::glDisable(GL_DEPTH_TEST));
}

// Calculate an extended bounding box from axes and current model for visualization purposes.
BoundingBoxf3 Bed3D::calc_extended_bounding_box() const
{
    BoundingBoxf3 out { m_build_volume.bounding_volume() };
    const Vec3d size = out.size();
    // ensures that the bounding box is set as defined or the following calls to merge() will not work as intented
    if (size.x() > 0.0 && size.y() > 0.0 && !out.defined)
        out.defined = true;
    // Reset the build volume Z, we don't want to zoom to the top of the build volume if it is empty.
    out.min.z() = 0.0;
    out.max.z() = 0.0;
    // extend to origin in case origin is off bed
    out.merge(m_axes.get_origin());
    // extend to contain axes
    out.merge(m_axes.get_origin() + m_axes.get_total_length() * Vec3d::Ones());
    out.merge(out.min + Vec3d(-m_axes.get_tip_radius(), -m_axes.get_tip_radius(), out.max.z()));
    // extend to contain model, if any
    BoundingBoxf3 model_bb = m_model.model.get_bounding_box();
    if (model_bb.defined) {
        model_bb.translate(m_model_offset);
        out.merge(model_bb);
    }
    return out;
}

void Bed3D::init_triangles()
{
    if (m_triangles.is_initialized())
        return;

    if (m_contour.empty())
        return;

    const std::vector<Vec2f> triangles = triangulate_expolygon_2f(m_contour, NORMALS_UP);
    if (triangles.empty() || triangles.size() % 3 != 0)
        return;

    GLModel::Geometry init_data;
    init_data.format = { GLModel::Geometry::EPrimitiveType::Triangles, GLModel::Geometry::EVertexLayout::P3T2 };
    init_data.reserve_vertices(triangles.size());
    init_data.reserve_indices(triangles.size() / 3);

    Vec2f min = triangles.front();
    Vec2f max = min;
    for (const Vec2f& v : triangles) {
        min = min.cwiseMin(v).eval();
        max = max.cwiseMax(v).eval();
    }

    const Vec2f size = max - min;
    if (size.x() <= 0.0f || size.y() <= 0.0f)
        return;

    Vec2f inv_size = size.cwiseInverse();
    inv_size.y() *= -1.0f;

    // vertices + indices
    unsigned int vertices_counter = 0;
    for (const Vec2f& v : triangles) {
        const Vec3f p = { v.x(), v.y(), GROUND_Z };
        init_data.add_vertex(p, (Vec2f)(v - min).cwiseProduct(inv_size).eval());
        ++vertices_counter;
        if (vertices_counter % 3 == 0)
            init_data.add_triangle(vertices_counter - 3, vertices_counter - 2, vertices_counter - 1);
    }

    if (m_model.model.get_filename().empty() && m_model.mesh_raycaster == nullptr)
        // register for picking
        register_raycasters_for_picking(init_data, Transform3d::Identity());

    m_triangles.init_from(std::move(init_data));
    m_triangles.set_color(m_model_color);
}

void Bed3D::init_gridlines()
{
    if (m_gridlines.is_initialized())
        return;

    if (m_contour.empty())
        return;

    const BoundingBox& bed_bbox = m_contour.contour.bounding_box();

    Polylines axes_lines;
    Polylines axes_lines_big;
    Polylines axes_lines_small;
    coord_t step = scale_t(5);
    while (bed_bbox.radius() > step * 100) {
        step *= 10;
    }
    for (coord_t x = bed_bbox.min.x(), idx= 0; x <= bed_bbox.max.x(); x += step, idx++) {
        Polyline line;
        line.append(Point(x, bed_bbox.min.y()));
        line.append(Point(x, bed_bbox.max.y()));
        if (idx % 10 == 0)
            axes_lines_big.push_back(line);
        else if(idx%2==1)
            axes_lines_small.push_back(line);
        else
            axes_lines.push_back(line);
    }
    for (coord_t y = bed_bbox.min.y(), idx = 0; y <= bed_bbox.max.y(); y += step, idx++) {
        Polyline line;
        line.append(Point(bed_bbox.min.x(), y));
        line.append(Point(bed_bbox.max.x(), y));
        if (idx % 10 == 0)
            axes_lines_big.push_back(line);
        else if (idx % 2 == 1)
            axes_lines_small.push_back(line);
        else
            axes_lines.push_back(line);
    }

    // clip with a slightly grown expolygon because our lines lay on the contours and may get erroneously clipped
    Polygons contour_offset = offset(m_contour, float(SCALED_EPSILON));
    Lines gridlines = to_lines(intersection_pl(axes_lines, contour_offset));
    Lines gridlines_big = to_lines(intersection_pl(axes_lines_big, contour_offset));
    Lines gridlines_small = to_lines(intersection_pl(axes_lines_small, contour_offset));

    // append bed contours
    Lines contour_lines = to_lines(m_contour);
    std::copy(contour_lines.begin(), contour_lines.end(), std::back_inserter(gridlines));

    auto createGrid = [](const Lines &grid_lines, GLModel &model_to_fill){
	    GLModel::Geometry init_data;
	    init_data.format = { GLModel::Geometry::EPrimitiveType::Lines, GLModel::Geometry::EVertexLayout::P3 };
	    init_data.reserve_vertices(2 * grid_lines.size());
	    init_data.reserve_indices(2 * grid_lines.size());

	    for (const Slic3r::Line& l : grid_lines) {
	        init_data.add_vertex(Vec3f(unscale<float>(l.a.x()), unscale<float>(l.a.y()), GROUND_Z));
	        init_data.add_vertex(Vec3f(unscale<float>(l.b.x()), unscale<float>(l.b.y()), GROUND_Z));
	        const unsigned int vertices_counter = (unsigned int)init_data.vertices_count();
	        init_data.add_line(vertices_counter - 2, vertices_counter - 1);
	    }

	    model_to_fill.init_from(std::move(init_data));
    };
    createGrid(gridlines, m_gridlines);
    createGrid(gridlines_big, m_gridlines_big);
    createGrid(gridlines_small, m_gridlines_small);
}

void Bed3D::init_contourlines()
{
    if (m_contourlines.is_initialized())
        return;

    if (m_contour.empty())
        return;

    const Lines contour_lines = to_lines(m_contour);

    GLModel::Geometry init_data;
    init_data.format = { GLModel::Geometry::EPrimitiveType::Lines, GLModel::Geometry::EVertexLayout::P3 };
    init_data.reserve_vertices(2 * contour_lines.size());
    init_data.reserve_indices(2 * contour_lines.size());

    for (const Slic3r::Line& l : contour_lines) {
        init_data.add_vertex(Vec3f(unscale<float>(l.a.x()), unscale<float>(l.a.y()), GROUND_Z));
        init_data.add_vertex(Vec3f(unscale<float>(l.b.x()), unscale<float>(l.b.y()), GROUND_Z));
        const unsigned int vertices_counter = (unsigned int)init_data.vertices_count();
        init_data.add_line(vertices_counter - 2, vertices_counter - 1);
    }

    m_contourlines.init_from(std::move(init_data));
    m_contourlines.set_color({ 1.0f, 1.0f, 1.0f, 0.5f });
}

// Try to match the print bed shape with the shape of an active profile. If such a match exists,
// return the print bed model.
std::tuple<Bed3D::Type, std::string, std::string, bool> Bed3D::detect_type(const Pointfs& shape)
{
    auto bundle = wxGetApp().preset_bundle.get();
    if (bundle != nullptr && bundle->printers.size() > bundle->printers.get_selected_idx()) {
        const Preset* curr = &bundle->printers.get_selected_preset();
        while (curr != nullptr) {
            if (curr->config.has("bed_shape")) {
                if (shape == dynamic_cast<const ConfigOptionPoints*>(curr->config.option("bed_shape"))->get_values()) {
                    std::string model_filename = PresetUtils::system_printer_bed_model(*curr);
                    std::string texture_filename = PresetUtils::system_printer_bed_texture(*curr);
                    if (!model_filename.empty() && !texture_filename.empty())
                        return { Type::System, model_filename, texture_filename, PresetUtils::system_printer_model(*curr)->bed_with_grid };
                    else if(!model_filename.empty())
                        return { Type::System, model_filename, {}, true };
                    else if(!texture_filename.empty())
                        return { Type::System, {}, texture_filename, PresetUtils::system_printer_model(*curr)->bed_with_grid};
                }
            }

            curr = bundle->printers.get_preset_parent(*curr);
        }
    }

    return { Type::Custom, {}, {}, false };
}

void Bed3D::render_axes()
{
    if (m_build_volume.valid())
        m_axes.render(Transform3d::Identity(), 0.25f);
}

void Bed3D::render_grid(bool bottom, bool has_model)
{
    // draw grid
    ColorRGBA grid_color = m_grid_color;
    if (has_model && !bottom)
        grid_color = DEFAULT_SOLID_GRID_COLOR;
    else if (bottom)
        grid_color = DEFAULT_TRANSPARENT_GRID_COLOR;
#if ENABLE_GL_CORE_PROFILE
    if (!OpenGLManager::get_gl_info().is_core_profile())
#endif // ENABLE_GL_CORE_PROFILE
        glsafe(::glLineWidth(0.5f * m_scale_factor));
    m_gridlines_small.set_color(grid_color);
    m_gridlines_small.render();
#if ENABLE_GL_CORE_PROFILE
    if (!OpenGLManager::get_gl_info().is_core_profile())
#endif // ENABLE_GL_CORE_PROFILE
         glsafe(::glLineWidth(1.5f * m_scale_factor));
    m_gridlines.set_color(grid_color);
    m_gridlines.render();
#if ENABLE_GL_CORE_PROFILE
    if (!OpenGLManager::get_gl_info().is_core_profile())
#endif // ENABLE_GL_CORE_PROFILE
        glsafe(::glLineWidth(3.0f * m_scale_factor));
    m_gridlines_big.set_color(grid_color);
    m_gridlines_big.render();
}

void Bed3D::render_system(GLCanvas3D& canvas, const Transform3d& view_matrix, const Transform3d& projection_matrix, bool bottom, bool show_texture)
{
    if (!bottom) {
        render_model(view_matrix, projection_matrix);
        render_exclude_area(false);
    }
   
    if (show_texture)
        render_texture(bottom, canvas, view_matrix, projection_matrix);
    else if (bottom)
        render_contour(view_matrix, projection_matrix);
}

void Bed3D::render_texture(bool bottom, GLCanvas3D& canvas, const Transform3d& view_matrix, const Transform3d& projection_matrix)
{
    if (m_texture_filename.empty()) {
        m_texture.reset();
        render_default(bottom, false, true, view_matrix, projection_matrix);
        return;
    }
    std::string texture_absolute_filename = m_texture_path.generic_string();

    if (m_texture.get_id() == 0 || m_texture.get_source() != texture_absolute_filename) {
        m_texture.reset();

        if (boost::algorithm::iends_with(texture_absolute_filename, ".svg")) {
            // use higher resolution images if graphic card and opengl version allow
            GLint max_tex_size = OpenGLManager::get_gl_info().get_max_tex_size();
            if (m_temp_texture.get_id() == 0 || m_temp_texture.get_source() != texture_absolute_filename) {
                // generate a temporary lower resolution texture to show while no main texture levels have been compressed
                if (!m_temp_texture.load_from_svg_file(texture_absolute_filename, false, false, false, max_tex_size / 8)) {
                    render_default(bottom, false, true, view_matrix, projection_matrix);
                    return;
                }
                canvas.request_extra_frame();
            }

            // starts generating the main texture, compression will run asynchronously
            if (!m_texture.load_from_svg_file(texture_absolute_filename, true, true, true, max_tex_size)) {
                render_default(bottom, false, true, view_matrix, projection_matrix);
                return;
            }
        } 
        else if (boost::algorithm::iends_with(texture_absolute_filename, ".png")) {
            // generate a temporary lower resolution texture to show while no main texture levels have been compressed
            if (wxGetApp().app_config->get("compress_png_texture") == "1" && (m_temp_texture.get_id() == 0 || m_temp_texture.get_source() != texture_absolute_filename)) {
                if (!m_temp_texture.load_from_file(texture_absolute_filename, false, GLTexture::None, false)) {
                    render_default(bottom, false, true, view_matrix, projection_matrix);
                    return;
                }
                canvas.request_extra_frame();
            }

            // starts generating the main texture, compression will run asynchronously
            if (!m_texture.load_from_file(texture_absolute_filename, true,
                wxGetApp().app_config->get("compress_png_texture") == "1" ? GLTexture::MultiThreaded : GLTexture::ECompressionType::None, true)) {
                render_default(bottom, false, true, view_matrix, projection_matrix);
                return;
            }
        }
        else {
            render_default(bottom, false, true, view_matrix, projection_matrix);
            return;
        }
    }
    else if (m_texture.unsent_compressed_data_available()) {
        // sends to gpu the already available compressed levels of the main texture
        m_texture.send_compressed_data_to_gpu();

        // the temporary texture is not needed anymore, reset it
        if (m_temp_texture.get_id() != 0)
            m_temp_texture.reset();

        canvas.request_extra_frame();
    }

    init_triangles();

    GLShaderProgram* shader = wxGetApp().get_shader("printbed");
    if (shader != nullptr) {
        shader->start_using();
        shader->set_uniform("view_model_matrix", view_matrix);
        shader->set_uniform("projection_matrix", projection_matrix);
        shader->set_uniform("transparent_background", bottom);
        shader->set_uniform("svg_source", boost::algorithm::iends_with(m_texture.get_source(), ".svg"));

        glsafe(::glEnable(GL_DEPTH_TEST));
        if (bottom)
            glsafe(::glDepthMask(GL_FALSE));

        glsafe(::glEnable(GL_BLEND));
        glsafe(::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

        if (bottom)
            glsafe(::glFrontFace(GL_CW));

        // if m_texture_with_grid, show a grid on top of texture.                             
        if (this->m_texture_with_grid) {
            glsafe(::glDisable(GL_DEPTH_TEST));
            glsafe(::glDisable(GL_BLEND));
            render_grid(bottom, true);
            glsafe(::glEnable(GL_DEPTH_TEST));
            glsafe(::glEnable(GL_BLEND));
        }

        // show the temporary texture while no compressed data is available
        GLuint tex_id = (GLuint)m_temp_texture.get_id();
        if (tex_id == 0)
            tex_id = (GLuint)m_texture.get_id();

        glsafe(::glBindTexture(GL_TEXTURE_2D, tex_id));
        m_triangles.render();
        glsafe(::glBindTexture(GL_TEXTURE_2D, 0));

        if (bottom)
            glsafe(::glFrontFace(GL_CCW));

        glsafe(::glDisable(GL_BLEND));
        if (bottom)
            glsafe(::glDepthMask(GL_TRUE));

        shader->stop_using();
    }
}

void Bed3D::render_model(const Transform3d& view_matrix, const Transform3d& projection_matrix)
{
    if (m_model_filename.empty())
        return;

    std::string model_absolute_filename = m_model_path.generic_string();
    if (m_model.model.get_filename() != model_absolute_filename && m_model.model.init_from_file(model_absolute_filename)) {
        m_model.model.set_color(DEFAULT_MODEL_COLOR);

        // move the model so that its origin (0.0, 0.0, 0.0) goes into the bed shape center and a bit down to avoid z-fighting with the texture quad
        m_model_offset = to_3d(m_build_volume.bounding_volume2d().center(), -0.03);

        // register for picking
        const std::vector<std::shared_ptr<SceneRaycasterItem>>* const raycaster = wxGetApp().plater()->canvas3D()->get_raycasters_for_picking(SceneRaycaster::EType::Bed);
        if (!raycaster->empty()) {
            // The raycaster may have been set by the call to init_triangles() made from render_texture() if the printbed was
            // changed while the camera was pointing upward.
            // In this case we need to remove it before creating a new using the model geometry
            wxGetApp().plater()->canvas3D()->remove_raycasters_for_picking(SceneRaycaster::EType::Bed);
            m_model.mesh_raycaster.reset();
        }
        register_raycasters_for_picking(m_model.model.get_geometry(), Geometry::translation_transform(m_model_offset));

        // update extended bounding box
        m_extended_bounding_box = this->calc_extended_bounding_box();
    }

    if (!m_model.model.get_filename().empty()) {
        GLShaderProgram* shader = wxGetApp().get_shader("gouraud_light");
        if (shader != nullptr) {
            shader->start_using();
            shader->set_uniform("emission_factor", 0.0f);
            const Transform3d model_matrix = Geometry::translation_transform(m_model_offset);
            shader->set_uniform("view_model_matrix", view_matrix * model_matrix);
            shader->set_uniform("projection_matrix", projection_matrix);
            const Matrix3d view_normal_matrix = view_matrix.matrix().block(0, 0, 3, 3) * model_matrix.matrix().block(0, 0, 3, 3).inverse().transpose();
            shader->set_uniform("view_normal_matrix", view_normal_matrix);
            m_model.model.render();
            shader->stop_using();
        }
    }
}

void Bed3D::render_custom(GLCanvas3D& canvas, const Transform3d& view_matrix, const Transform3d& projection_matrix, bool bottom, bool show_texture, bool picking)
{
    if (m_texture_filename.empty() && m_model_filename.empty()) {
        render_default(bottom, picking, show_texture, view_matrix, projection_matrix);
        return;
    }

    if (!bottom) {
        render_model(view_matrix, projection_matrix);
        render_exclude_area(false);
      }
      
    if (show_texture)
        render_texture(bottom, canvas, view_matrix, projection_matrix);
    else if (bottom)
        render_contour(view_matrix, projection_matrix);
}

void Bed3D::render_default(bool bottom, bool picking, bool show_texture, const Transform3d& view_matrix, const Transform3d& projection_matrix)
{
    m_texture.reset();

    init_gridlines();
    init_triangles();

    GLShaderProgram* shader = wxGetApp().get_shader("flat");
    if (shader != nullptr) {
        shader->start_using();

        shader->set_uniform("view_model_matrix", view_matrix);
        shader->set_uniform("projection_matrix", projection_matrix);

        glsafe(::glEnable(GL_DEPTH_TEST));
        glsafe(::glEnable(GL_BLEND));
        glsafe(::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

        const bool has_model = !m_model.model.get_filename().empty();
        if (!has_model && !bottom) {
            // draw background
            glsafe(::glDepthMask(GL_FALSE));
            m_triangles.render();
            glsafe(::glDepthMask(GL_TRUE));
        }

        if (show_texture) {
            render_grid(bottom, has_model);
        } else
            render_contour(view_matrix, projection_matrix);

        glsafe(::glDisable(GL_BLEND));

        shader->stop_using();
    }
}

void Bed3D::render_contour(const Transform3d& view_matrix, const Transform3d& projection_matrix)
{
    init_contourlines();

    GLShaderProgram* shader = wxGetApp().get_shader("flat");
    if (shader != nullptr) {
        shader->start_using();
        shader->set_uniform("view_model_matrix", view_matrix);
        shader->set_uniform("projection_matrix", projection_matrix);

        glsafe(::glEnable(GL_DEPTH_TEST));
        glsafe(::glEnable(GL_BLEND));
        glsafe(::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

        // draw contour
#if ENABLE_GL_CORE_PROFILE
        if (!OpenGLManager::get_gl_info().is_core_profile())
#endif // ENABLE_GL_CORE_PROFILE
            glsafe(::glLineWidth(1.5f * m_scale_factor));
        m_contourlines.render();

        glsafe(::glDisable(GL_BLEND));

        shader->stop_using();
    }
}

void Bed3D::register_raycasters_for_picking(const GLModel::Geometry& geometry, const Transform3d& trafo)
{
    assert(m_model.mesh_raycaster == nullptr);

    indexed_triangle_set its;
    its.vertices.reserve(geometry.vertices_count());
    for (size_t i = 0; i < geometry.vertices_count(); ++i) {
        its.vertices.emplace_back(geometry.extract_position_3(i));
    }
    its.indices.reserve(geometry.indices_count() / 3);
    for (size_t i = 0; i < geometry.indices_count() / 3; ++i) {
        const size_t tri_id = i * 3;
        its.indices.emplace_back(geometry.extract_index(tri_id), geometry.extract_index(tri_id + 1), geometry.extract_index(tri_id + 2));
    }

    m_model.mesh_raycaster = std::make_unique<MeshRaycaster>(std::make_shared<const TriangleMesh>(std::move(its)));
    wxGetApp().plater()->canvas3D()->add_raycaster_for_picking(SceneRaycaster::EType::Bed, 0, *m_model.mesh_raycaster, trafo);
}

} // GUI
} // Slic3r
