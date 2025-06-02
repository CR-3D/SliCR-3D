
#include <catch_main.hpp>

#include <libslic3r/Point.hpp>
#include <libslic3r/BoundingBox.hpp>
#include <libslic3r/MutablePolygon.hpp>
#include <libslic3r/Polygon.hpp>
#include <libslic3r/Polyline.hpp>
#include <libslic3r/Line.hpp>
#include <libslic3r/Geometry.hpp>
#include <libslic3r/ClipperUtils.hpp>
#include <libslic3r/SVG.hpp>

using namespace Slic3r;

TEST_CASE("Polygon::contains works properly", ""){
   // this test was failing on Windows (GH #1950)
    Slic3r::Polygon polygon( Points{
        Point{207802834,-57084522},
        Point{196528149,-37556190},
        Point{173626821,-25420928},
        Point{171285751,-21366123},
        Point{118673592,-21366123},
        Point{116332562,-25420928},
        Point{93431208,-37556191},
        Point{82156517,-57084523},
        Point{129714478,-84542120},
        Point{160244873,-84542120}
    } );
    Point point{ 95706562, -57294774 };
    REQUIRE(polygon.contains(point));
}

SCENARIO("Intersections of line segments"){
    GIVEN("Integer coordinates"){
        Line line1{ Point::new_scale(5,15),Point::new_scale(30,15) };
        Line line2{ Point::new_scale(10,20), Point::new_scale(10,10) };
        THEN("The intersection is valid"){
            Point point;
            line1.intersection(line2,&point);
            REQUIRE(Point::new_scale(10,15) == point);
        }
    }

    GIVEN("Scaled coordinates"){
        Line line1{ Point::new_scale(73.6310778185108, 371.74239268924), Point::new_scale(73.6310778185108, 501.74239268924) };
        Line line2{ Point::new_scale(75, 437.9853), Point::new_scale(62.7484, 440.4223) };
        THEN("There is still an intersection"){
            Point point;
            REQUIRE(line1.intersection(line2,&point));
        }
    }
}

    inline void my_clip_clipper_polygon_with_subject_bbox_templ(const Points &src, const BoundingBox &bbox, Points &out)
    {

        out.clear();
        const size_t cnt = src.size();
        if (cnt < 3)
            return;

        enum class Side {
            Left   = 1,
            Right  = 2,
            Top    = 4,
            Bottom = 8
        };

        auto sides = [bbox](const Point &p) {
            return  int(p.x() < bbox.min.x()) * int(Side::Left) +
                    int(p.x() > bbox.max.x()) * int(Side::Right) +
                    int(p.y() < bbox.min.y()) * int(Side::Bottom) +
                    int(p.y() > bbox.max.y()) * int(Side::Top);
        };

        int sides_prev = sides(src.back());
        int sides_this = sides(src.front());
        const size_t last = cnt - 1;
        for (size_t i = 0; i < last; ++ i) {
            int sides_next = sides(src[i + 1]);
            if (// This point is inside. Take it.
                sides_this == 0 ||
                // Either this point is outside and previous or next is inside, or
                // the edge possibly cuts corner of the bounding box.
                (sides_prev & sides_this & sides_next) == 0) {
                out.emplace_back(src[i]);
                sides_prev = sides_this;
            } else {
                // All the three points (this, prev, next) are outside at the same side.
                // Ignore this point.
            }
            sides_this = sides_next;
        }

        // Never produce just a single point output polygon.
        if (! out.empty())
            if (int sides_next = sides(out.front());
                // The last point is inside. Take it.
                sides_this == 0 ||
                // Either this point is outside and previous or next is inside, or
                // the edge possibly cuts corner of the bounding box.
                (sides_prev & sides_this & sides_next) == 0)
                out.emplace_back(src.back());
        //assert(out.size() > 2 || out.empty());
    }

SCENARIO("clip_clipper_polygon_with_subject_bbox"){
    BoundingBox bb(Point::new_scale(0,0),Point::new_scale(10,10));
    GIVEN("inside"){
        Slic3r::Polygon poly({Point::new_scale(3,1),Point::new_scale(6,2),Point::new_scale(4,8)});
        THEN("The intersection is valid"){
            Slic3r::Polygon result = ClipperUtils::clip_clipper_polygon_with_subject_bbox(poly, bb);
            REQUIRE(poly == result);
        }
    }
    
    GIVEN("both"){
        Slic3r::Polygon poly({Point::new_scale(-15,1),Point::new_scale(1,1),Point::new_scale(9,1),Point::new_scale(15,1),
            Point::new_scale(9,7),Point::new_scale(7,9),Point::new_scale(5,11),
            Point::new_scale(3,9),Point::new_scale(1,7)});
        THEN("The intersection is valid"){
            Slic3r::Polygon result = ClipperUtils::clip_clipper_polygon_with_subject_bbox(poly, bb);
            //Slic3r::Polygon poly_result({Point::new_scale(1,1),Point::new_scale(9,1),
            //Point::new_scale(9,7),Point::new_scale(7,9),
            //Point::new_scale(3,9),Point::new_scale(1,7)});
            REQUIRE(poly == result);
        }
    }

    GIVEN("outside but crossing"){
        Slic3r::Polygon poly({Point::new_scale(-3,-1),Point::new_scale(6,-2),Point::new_scale(4,11)});
        THEN("The intersection is valid"){
            Slic3r::Polygon result = ClipperUtils::clip_clipper_polygon_with_subject_bbox(poly, bb);
            REQUIRE(poly == result);
        }
    }

    GIVEN("outside, including the bb inside"){
        Slic3r::Polygon poly({Point::new_scale(-3,-3),Point::new_scale(20,-3),Point::new_scale(20,20),Point::new_scale(-3,20)});
        THEN("The intersection is valid"){
            Slic3r::Polygon result = ClipperUtils::clip_clipper_polygon_with_subject_bbox(poly, bb);
            REQUIRE(poly == result);
        }
    }

    GIVEN("outside not crossing"){
        Slic3r::Polygon poly({Point::new_scale(-3,-1),Point::new_scale(-6,-2),Point::new_scale(-4,-11)});
        THEN("The intersection is none"){
            Slic3r::Polygon result = ClipperUtils::clip_clipper_polygon_with_subject_bbox(poly, bb);
            REQUIRE(result.empty());
        }
    }

    GIVEN("weird thing"){
        Slic3r::Polygon polytest({Point{-5253696,6941803},Point{-5390322,7004051},Point{-5529994,7112838},Point{-5642583,7262245},Point{-5708854,7426076},Point{-5731772,7600991},Point{-5710375,7774699},Point{-5643429,7942791},Point{-3001108,11534999},Point{-2782155,11687851},Point{-2602580,11739919},Point{-2335818,11727897},Point{-2161635,11659878},Point{-1955359,11486297},Point{-1830017,11244692},Point{-1806677,10973499},Point{-1888105,10716510},Point{-4515353,7139500},Point{-4638040,7031082},Point{-4773007,6958051},Point{-4924657,6915497},Point{-5069276,6907438}});
        BoundingBox boxtest(Point{-1854941, 7335228}, Point{4734351, 9668157});
        Slic3r::Polygon result = ClipperUtils::clip_clipper_polygon_with_subject_bbox(polytest, boxtest);
        //Slic3r::Polygon result;
        //my_clip_clipper_polygon_with_subject_bbox_templ(polytest.points, boxtest, result.points);
        //::Slic3r::SVG svg("weird.svg");
        //svg.draw(boxtest.polygon(), "grey");
        //svg.draw(polytest.split_at_first_point(), "blue", scale_t(0.05));
        //for(Point pt : result.points)
        //    svg.draw(pt, "green", scale_t(0.03));
        //svg.Close();

        //REQUIRE(result.size() > 2);
        REQUIRE(result.empty());
    }
}

/*
Tests for unused methods still written in perl
{
    my $polygon = Slic3r::Polygon->new(
        [45919000, 515273900], [14726100, 461246400], [14726100, 348753500], [33988700, 315389800], 
        [43749700, 343843000], [45422300, 352251500], [52362100, 362637800], [62748400, 369577600], 
        [75000000, 372014700], [87251500, 369577600], [97637800, 362637800], [104577600, 352251500], 
        [107014700, 340000000], [104577600, 327748400], [97637800, 317362100], [87251500, 310422300], 
        [82789200, 309534700], [69846100, 294726100], [254081000, 294726100], [285273900, 348753500], 
        [285273900, 461246400], [254081000, 515273900],
    );
    
    # this points belongs to $polyline
    # note: it's actually a vertex, while we should better check an intermediate point
    my $point = Slic3r::Point->new(104577600, 327748400);
    
    local $Slic3r::Geometry::epsilon = 1E-5;
    is_deeply Slic3r::Geometry::polygon_segment_having_point($polygon, $point)->pp, 
        [ [107014700, 340000000], [104577600, 327748400] ],
        'polygon_segment_having_point';
}
{
        auto point = Point{736310778.185108, 5017423926.8924};
        auto line = Line(Point{(long int} 627484000, (long int) 3695776000), Point{(long int} 750000000, (long int)3720147000));
        //is Slic3r::Geometry::point_in_segment($point, $line), 0, 'point_in_segment';
}

// Possible to delete
{
        //my $p1 = [10, 10];
        //my $p2 = [10, 20];
        //my $p3 = [10, 30];
        //my $p4 = [20, 20];
        //my $p5 = [0,  20];
        
        THEN("Points in a line give the correct angles"){
            //is Slic3r::Geometry::angle3points($p2, $p3, $p1),  PI(),   'angle3points';
            //is Slic3r::Geometry::angle3points($p2, $p1, $p3),  PI(),   'angle3points';
        }
        THEN("Left turns give the correct angle"){
            //is Slic3r::Geometry::angle3points($p2, $p4, $p3),  PI()/2, 'angle3points';
            //is Slic3r::Geometry::angle3points($p2, $p1, $p4),  PI()/2, 'angle3points';
        }
        THEN("Right turns give the correct angle"){
            //is Slic3r::Geometry::angle3points($p2, $p3, $p4),  PI()/2*3, 'angle3points';
            //is Slic3r::Geometry::angle3points($p2, $p1, $p5),  PI()/2*3, 'angle3points';
        }
        //my $p1 = [30, 30];
        //my $p2 = [20, 20];
        //my $p3 = [10, 10];
        //my $p4 = [30, 10];
        
        //is Slic3r::Geometry::angle3points($p2, $p1, $p3), PI(),       'angle3points';
        //is Slic3r::Geometry::angle3points($p2, $p1, $p4), PI()/2*3,   'angle3points';
        //is Slic3r::Geometry::angle3points($p2, $p1, $p1), 2*PI(),     'angle3points';
}

SCENARIO("polygon_is_convex works"){
    GIVEN("A square of dimension 10"){
        //my $cw_square = [ [0,0], [0,10], [10,10], [10,0] ];
        THEN("It is not convex clockwise"){
            //is polygon_is_convex($cw_square), 0, 'cw square is not convex';
        }
        THEN("It is convex counter-clockwise"){
            //is polygon_is_convex([ reverse @$cw_square ]), 1, 'ccw square is convex';
        } 

    }
    GIVEN("A concave polygon"){
        //my $convex1 = [ [0,0], [10,0], [10,10], [0,10], [0,6], [4,6], [4,4], [0,4] ];
        THEN("It is concave"){
            //is polygon_is_convex($convex1), 0, 'concave polygon';
        }
    }
}*/


TEST_CASE("Creating a polyline generates the obvious lines"){
    auto polyline = Slic3r::Polyline();
    polyline.points = Points({Point::new_scale(0, 0), Point::new_scale(10, 0), Point::new_scale(20, 0)});
    REQUIRE(polyline.lines().at(0).a == Point::new_scale(0,0));
    REQUIRE(polyline.lines().at(0).b == Point::new_scale(10,0));
    REQUIRE(polyline.lines().at(1).a == Point::new_scale(10,0));
    REQUIRE(polyline.lines().at(1).b == Point::new_scale(20,0));
}

TEST_CASE("Splitting a Polygon generates a polyline correctly"){
    Slic3r::Polygon polygon = Slic3r::Polygon({Point::new_scale(0, 0), Point::new_scale(10, 0), Point::new_scale(5, 5)});
    Slic3r::Polyline split = polygon.split_at_index(1);
    REQUIRE(split.points.size() == 4);
    REQUIRE(split.points[0]==Point::new_scale(10,0));
    REQUIRE(split.points[1]==Point::new_scale(5,5));
    REQUIRE(split.points[2]==Point::new_scale(0,0));
    REQUIRE(split.points[3]==Point::new_scale(10,0));
}


TEST_CASE("Bounding boxes are scaled appropriately"){
    Slic3r::BoundingBox bb(Points{Point::new_scale(0, 1), Point::new_scale(10, 2), Point::new_scale(20, 2)});
    bb.scale(2);
    REQUIRE(bb.min == Point::new_scale(0,2));
    REQUIRE(bb.max == Point::new_scale(40,4));
}


TEST_CASE("Offseting a line generates a polygon correctly"){
    Slic3r::Polyline tmp(Points{{10,10},{20,10} });
    Slic3r::Polygon area = offset(tmp, scale_d(5)).at(0);
    REQUIRE(area.area() == Slic3r::Polygon({Point::new_scale(10,5),Point::new_scale(20,5),Point::new_scale(20,15),Point::new_scale(10,15)}).area());
}

SCENARIO("Circle Fit, TaubinFit with Newton's method") {
    GIVEN("A vector of Pointfs arranged in a half-circle with approximately the same distance R from some point") {
        Vec2d expected_center(-6, 0);
        Pointfs sample {Vec2d{6.0, 0}, Vec2d{5.1961524, 3}, Vec2d{3 ,5.1961524}, Vec2d{0, 6.0}, Vec2d{-3, 5.1961524}, Vec2d{-5.1961524, 3}, Vec2d{-6.0, 0}};
        std::transform(sample.begin(), sample.end(), sample.begin(), [expected_center] (const Vec2d& a) { return a + expected_center;});

        WHEN("Circle fit is called on the entire array") {
            Vec2d result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample);
            THEN("A center point of -6,0 is returned.") {
                REQUIRE((result_center - expected_center).norm() < EPSILON);
            }
        }
        WHEN("Circle fit is called on the first four points") {
            Vec2d result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample.cbegin(), sample.cbegin()+4);
            THEN("A center point of -6,0 is returned.") {
                REQUIRE((result_center - expected_center).norm() < EPSILON);
            }
        }
        WHEN("Circle fit is called on the middle four points") {
            Vec2d result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample.cbegin()+2, sample.cbegin()+6);
            THEN("A center point of -6,0 is returned.") {
                REQUIRE((result_center - expected_center).norm() < EPSILON);
            }
        }
    }
    GIVEN("A vector of Pointfs arranged in a half-circle with approximately the same distance R from some point") {
        Vec2d expected_center(-3, 9);
        Vec2ds sample {Vec2d{6.0, 0}, Vec2d{5.1961524, 3}, Vec2d{3 ,5.1961524},
                        Vec2d{0, 6.0}, 
                        Vec2d{3, 5.1961524}, Vec2d{-5.1961524, 3}, Vec2d{-6.0, 0}};

        std::transform(sample.begin(), sample.end(), sample.begin(), [expected_center] (const Vec2d& a) { return a + expected_center;});


        WHEN("Circle fit is called on the entire array") {
            Vec2d result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample);
            THEN("A center point of 3,9 is returned.") {
                REQUIRE((result_center - expected_center).norm() < EPSILON);
            }
        }
        WHEN("Circle fit is called on the first four points") {
            Vec2d result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample.cbegin(), sample.cbegin()+4);
            THEN("A center point of 3,9 is returned.") {
                REQUIRE((result_center - expected_center).norm() < EPSILON);
            }
        }
        WHEN("Circle fit is called on the middle four points") {
            Vec2d result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample.cbegin()+2, sample.cbegin()+6);
            THEN("A center point of 3,9 is returned.") {
                REQUIRE((result_center - expected_center).norm() < EPSILON);
            }
        }
    }
    GIVEN("A vector of Points arranged in a half-circle with approximately the same distance R from some point") {
        Point expected_center { Point::new_scale(-3, 9)};
        Points sample {Point::new_scale(6.0, 0), Point::new_scale(5.1961524, 3), Point::new_scale(3 ,5.1961524), 
                        Point::new_scale(0, 6.0), 
                        Point::new_scale(3, 5.1961524), Point::new_scale(-5.1961524, 3), Point::new_scale(-6.0, 0)};

        std::transform(sample.begin(), sample.end(), sample.begin(), [expected_center] (const Point& a) { return a + expected_center;});


        WHEN("Circle fit is called on the entire array") {
            Point result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample);
            THEN("A center point of scaled 3,9 is returned.") {
                REQUIRE(result_center.coincides_with_epsilon(expected_center));
            }
        }
        WHEN("Circle fit is called on the first four points") {
            Point result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample.cbegin(), sample.cbegin()+4);
            THEN("A center point of scaled 3,9 is returned.") {
                REQUIRE(result_center.coincides_with_epsilon(expected_center));
            }
        }
        WHEN("Circle fit is called on the middle four points") {
            Point result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample.cbegin()+2, sample.cbegin()+6);
            THEN("A center point of scaled 3,9 is returned.") {
                REQUIRE(result_center.coincides_with_epsilon(expected_center));
            }
        }
    }
}

// A PU
//TEST_CASE("Chained path working correctly"){
//    // if chained_path() works correctly, these points should be joined with no diagonal paths
//    // (thus 26 units long)
//    std::vector<Point> points = {Point::new_scale(26,26),Point::new_scale(52,26),Point::new_scale(0,26),Point::new_scale(26,52),Point::new_scale(26,0),Point::new_scale(0,52),Point::new_scale(52,52),Point::new_scale(52,0)};
//    std::vector<Points::size_type> indices;
//    Geometry::chained_path(points,indices);
//    for(Points::size_type i = 0; i < indices.size()-1;i++){
//        double dist = points.at(indices.at(i)).distance_to(points.at(indices.at(i+1)));
//        REQUIRE(abs(dist-26) <= EPSILON);
//    }
//}

SCENARIO("Line distances"){
    GIVEN("A line"){
        Line line{ Point::new_scale(0, 0), Point::new_scale(20, 0) };
        THEN("Points on the line segment have 0 distance"){
            REQUIRE(line.distance_to(Point::new_scale(0, 0))  == 0);
            REQUIRE(line.distance_to(Point::new_scale(20, 0)) == 0);
            REQUIRE(line.distance_to(Point::new_scale(10, 0)) == 0);
        
        }
        THEN("Points off the line have the appropriate distance"){
            REQUIRE(line.distance_to(Point::new_scale(10, 10)) == scale_t(10));
            REQUIRE(line.distance_to(Point::new_scale(50, 0)) == scale_t(30));
        }
    }
}

SCENARIO("Polygon convex/concave detection"){

    //GIVEN("Polygon with many angles"){
    //   // this test was failing on Windows (GH #1950)
    //    Points pts;
    //    double x = 0;
    //    pts.push_back(Point::new_scale(x,0));
    //    for (int angle = 0; angle < 90; angle += 10) {
    //        //std::cout<<"angles:"<<angle<<"\n";
    //        double xadd = std::cos(PI * angle / 180.);
    //        x += xadd;
    //        pts.push_back(Point::new_scale(x, std::sin(PI * angle / 180.)));
    //        if (pts.size() > 2) {
    //            std::cout << "add pt " << ((int(unscaled(pts[pts.size() - 2].x()) * 100)) / 100.) << ":"
    //                      << ((int(unscaled(pts[pts.size() - 2].y()) * 100)) / 100.) << "[" << (pts.size() - 2)
    //                      << "] (convex if ccw) with angle :"
    //                      << (180 *
    //                          angle_ccw(pts[pts.size() - 3] - pts[pts.size() - 2],
    //                                    pts[pts.size() - 1] - pts[pts.size() - 2]) /
    //                          PI)
    //                      << "\n";
    //        }
    //        x += xadd;
    //        pts.push_back(Point::new_scale(x, 0));
    //        std::cout << "add pt " << ((int(unscaled(pts[pts.size() - 2].x()) * 100)) / 100.) << ":"
    //                  << ((int(unscaled(pts[pts.size() - 2].y()) * 100)) / 100.) << "[" << (pts.size() - 2)
    //                  << "] (convex if cw) with angle :"
    //                  << (180 *
    //                      angle_ccw(pts[pts.size() - 3] - pts[pts.size() - 2],
    //                                pts[pts.size() - 1] - pts[pts.size() - 2]) /
    //                      PI)
    //                  << "\n";
    //    }
    //    Slic3r::Polygon polygon_ccw(pts);
    //    polygon_ccw.points.push_back(Point::new_scale(x, 2));
    //            std::cout << "add ccw pt " << (pts.size() - 1) << " with angle :"
    //                      << (180 *
    //                          angle_ccw(pts[pts.size() - 2] - pts[pts.size() - 1],
    //                                    Point::new_scale(x, 2) - pts[pts.size() - 1]) /
    //                          PI)
    //                      << "\n";
    //    polygon_ccw.points.push_back(Point::new_scale(0, 2));
    //    Slic3r::Polygon polygon_cw(pts);
    //    polygon_cw.points.push_back(Point::new_scale(x, -2));
    //            std::cout << "add cw pt " << (pts.size() - 1) << " with angle :"
    //                      << (180 *
    //                          angle_ccw(pts[pts.size() - 2] - pts[pts.size() - 1],
    //                                    Point::new_scale(x, -2) - pts[pts.size() - 1]) /
    //                          PI)
    //                      << "\n";
    //    polygon_cw.points.push_back(Point::new_scale(0, -2));
    //
    //    std::cout<<"TEST MAX\n";
    //    for (int angle = 5; angle < 180; angle += 10) {
    //        std::cout<<"===== angle "<<angle<<" =====\n";
    //        std::vector<size_t> concave_1 = polygon_ccw.concave_points_idx(0, PI * angle / 180.);
    //        std::cout<<"concave ccw  :";
    //        for(size_t idx : concave_1) std::cout<<", "<<idx;
    //        std::cout<<"\n";
    //        std::vector<size_t> concave_2 = polygon_cw.concave_points_idx(0, PI * angle / 180.);
    //        std::cout<<"concave cw  :";
    //        for(size_t idx : concave_2) std::cout<<", "<<idx;
    //        std::cout<<"\n";
    //        std::vector<size_t> convex_1 = polygon_ccw.convex_points_idx(0, PI * angle / 180.);
    //        std::cout<<"convex ccw  :";
    //        for(size_t idx : convex_1) std::cout<<", "<<idx;
    //        std::cout<<"\n";
    //        std::vector<size_t> convex_2 = polygon_cw.convex_points_idx(0, PI * angle / 180.);
    //        std::cout<<"convex cw  :";
    //        for(size_t idx : convex_2) std::cout<<", "<<idx;
    //        std::cout<<"\n";
    //    }
    //
    //    std::cout<<"TEST MIN\n";
    //    for (int angle = 5; angle < 180; angle += 10) {
    //        std::cout<<"===== angle "<<angle<<" =====\n";
    //        std::vector<size_t> concave_1 = polygon_ccw.concave_points_idx(PI * angle / 180., PI);
    //        std::cout<<"concave ccw  :";
    //        for(size_t idx : concave_1) std::cout<<", "<<idx;
    //        std::cout<<"\n";
    //        std::vector<size_t> concave_2 = polygon_cw.concave_points_idx(PI * angle / 180., PI);
    //        std::cout<<"concave cw  :";
    //        for(size_t idx : concave_2) std::cout<<", "<<idx;
    //        std::cout<<"\n";
    //        std::vector<size_t> convex_1 = polygon_ccw.convex_points_idx(PI * angle / 180., PI);
    //        std::cout<<"convex ccw  :";
    //        for(size_t idx : convex_1) std::cout<<", "<<idx;
    //        std::cout<<"\n";
    //        std::vector<size_t> convex_2 = polygon_cw.convex_points_idx(PI * angle / 180., PI);
    //        std::cout<<"convex cw  :";
    //        for(size_t idx : convex_2) std::cout<<", "<<idx;
    //        std::cout<<"\n";
    //    }

    //    REQUIRE(true);
    //}

    GIVEN(("A Square with dimension 100")){
        Slic3r::Polygon square/*new_scale*/( Points{
            Point::new_scale(100,100),
            Point::new_scale(200,100),
            Point::new_scale(200,200),
            Point::new_scale(100,200)});
        THEN("It has 4 convex points counterclockwise"){
            REQUIRE(square.concave_points(0, PI).size() == 0);
            REQUIRE(square.concave_points(0, PI*2/3).size() == 0);
            REQUIRE(square.convex_points(0, PI).size() == 4);
            REQUIRE(square.convex_points(0, PI*2/3).size() == 4);
        }
        THEN("It has 4 concave points clockwise"){
            square.make_clockwise();
            REQUIRE(square.concave_points(0, PI).size() == 4);
            REQUIRE(square.concave_points(0, PI*2/3).size() == 4);
            REQUIRE(square.convex_points(0, PI).size() == 0);
            REQUIRE(square.convex_points(0, PI*2/3).size() == 0);
        }
    }
    GIVEN("A Square with an extra colinearvertex"){
        Slic3r::Polygon square /*new_scale*/( Points{
            Point::new_scale(150,100),
            Point::new_scale(200,100),
            Point::new_scale(200,200),
            Point::new_scale(100,200),
            Point::new_scale(100,100)} );
        THEN("It has 4 convex points counterclockwise"){
            REQUIRE(square.concave_points(0, PI).size() == 1);
            REQUIRE(square.concave_points(0, PI*2/3).size() == 0);
            REQUIRE(square.convex_points(0, PI).size() == 5);
            REQUIRE(square.convex_points(0, PI*2/3).size() == 4);
        }
    }
    GIVEN("A Square with an extra collinear vertex in different order"){
        Slic3r::Polygon square /*new_scale*/( Points{
            Point::new_scale(200,200),
            Point::new_scale(100,200),
            Point::new_scale(100,100),
            Point::new_scale(150,100),
            Point::new_scale(200,100)} );
        THEN("It has 4 convex points counterclockwise"){
            auto concave_points  = square.concave_points(0, PI*2/3);
            auto concave_points23  = square.concave_points(0, PI);
            auto convex_points  = square.convex_points(0, PI);
            auto convex_points23  = square.convex_points(0, PI*2/3);
            REQUIRE(square.concave_points(0, PI).size() == 1);
            REQUIRE(square.concave_points(0, PI*2/3).size() == 0);
            REQUIRE(square.convex_points(0, PI).size() == 5);
            REQUIRE(square.convex_points(0, PI*2/3).size() == 4);
        }
    }

    GIVEN("A triangle"){
        Slic3r::Polygon triangle( Points{
            Point{16000170,26257364},
            Point{714223,461012},
            Point{31286371,461008}
        } );
        THEN("it has three convex vertices"){
            REQUIRE(triangle.concave_points(0, PI).size() == 0);
            REQUIRE(triangle.concave_points(0, PI*2/3).size() == 0);
            REQUIRE(triangle.convex_points(0, PI).size() == 3);
            REQUIRE(triangle.convex_points(0, PI*2/3).size() == 3);
        }
    }

    GIVEN("A triangle with an extra collinear point"){
        Slic3r::Polygon triangle( Points{
            Point{16000170,26257364},
            Point{714223,461012},
            Point{20000000,461012},
            Point{31286371,461012}
        } );
        THEN("it has three convex vertices"){
            REQUIRE(triangle.concave_points(0, PI).size() == 1);
            REQUIRE(triangle.concave_points(0, PI*2/3).size() == 0);
            REQUIRE(triangle.convex_points(0, PI).size() == 4);
            REQUIRE(triangle.convex_points(0, PI*2/3).size() == 3);
        }
    }
    GIVEN("A polygon with concave vertices with angles of specifically 4/3pi"){
        // Two concave vertices of this polygon have angle = PI *4/3, so this test fails
        // if epsilon is not used.
        Slic3r::Polygon polygon( Points{
            Point{60246458,14802768},Point{64477191,12360001},
            Point{63727343,11060995},Point{64086449,10853608},
            Point{66393722,14850069},Point{66034704,15057334},
            Point{65284646,13758387},Point{61053864,16200839},
            Point{69200258,30310849},Point{62172547,42483120},
            Point{61137680,41850279},Point{67799985,30310848},
            Point{51399866,1905506},Point{38092663,1905506},
            Point{38092663,692699},Point{52100125,692699}
        } );
        THEN("the correct number of points are detected"){
            auto concave_points  = polygon.concave_points(0, PI*2/3);
            auto concave_points23  = polygon.concave_points(0, PI);
            auto convex_points  = polygon.convex_points(0, PI);
            auto convex_points23  = polygon.convex_points(0, PI*2/3);
            REQUIRE(polygon.concave_points(0, PI).size() == 6);
            REQUIRE(polygon.concave_points(0, PI*2/3).size() == 6);
            REQUIRE(polygon.convex_points(0, PI).size() == 10);
            REQUIRE(polygon.convex_points(0, PI*2/3).size() == 10);
        }
    }

}

TEST_CASE("Triangle Simplification does not result in less than 3 points"){
    Slic3r::Polygon triangle( Points{
        Point{16000170,26257364}, Point{714223,461012}, Point{31286371,461008}
    } );
    REQUIRE(triangle.simplify(250000).at(0).points.size() == 3);
}

//FIXME mutablePolygon
TEST_CASE("mutablepolygon clip_narrow_corner / smooth_outward") {
    Slic3r::Polygon polygon = {Point{-27085765,-2297259},Point{-27082608,-2297617},Point{-27073487,-2295946},Point{-26854179,-2272504},Point{-26853123,-2272196},Point{-26849973,-2270818},Point{-26783356,-2234478},Point{-26776453,-2232803},Point{-26750681,-2216654},Point{-26643136,-2157988},Point{-26569935,-2111663},Point{-26546446,-2085256},Point{-26509374,-2059233},Point{-26441565,-1964519},Point{-26346712,-1848324},Point{-26329548,-1804341},Point{-26326223,-1795414},Point{-26321733,-1788619},Point{-26317713,-1772565},Point{-26292655,-1705289},Point{-26278880,-1614690},Point{-26241365,-1455661},Point{-26249369,-1370864},Point{-26252087,-1100896},Point{-26264306,-1059921},Point{-26122099,-1162417},Point{-26052097,-1207892},Point{-25984511,-1231581},Point{-25932909,-1256179},Point{-25864667,-1298021},Point{-25798258,-1325399},Point{-25717810,-1347805},Point{-25567898,-1365005},Point{-25530512,-1365436},Point{-25470478,-1371330},Point{-25446729,-1366402},Point{-25440196,-1366477},Point{-25429087,-1362741},Point{-25255410,-1326699},Point{-25162981,-1300586},Point{-24987930,-1182486},Point{-24894329,-1110712},Point{-24815339,-994760},Point{-24814326,-1025860},Point{-24815491,-1036901},Point{-24813026,-1065765},Point{-24807523,-1234691},Point{-24805154,-1245409},Point{-24792463,-1273178},Point{-24787825,-1296407},Point{-24762324,-1357142},Point{-24720505,-1430628},Point{-24683619,-1511337},Point{-24653092,-1554698},Point{-24631481,-1575289},Point{-24575695,-1630390},Point{-24546333,-1672910},Point{-24513596,-1704605},Point{-24287683,-1876248},Point{-24257885,-1890510},Point{-24094570,-1947481},Point{-23977656,-1982361},Point{-23950759,-1987058},Point{-23701790,-1975124},Point{-23568857,-1955329},Point{-23445734,-1907554},Point{-23222346,-1799536},Point{-23208341,-1792329},Point{-23183027,-1767712},Point{-22943303,-1534376},Point{-22875009,-1410144},Point{-22810834,-1290930},Point{-22764219,-1204843},Point{-22740974,-1082764},Point{-22705459,-899064},Point{-22698659,-865062},Point{-22693641,-838309},Point{-22720854,-636501},Point{-22744632,-475178},Point{-22786311,-398421},Point{-22863006,-261004},Point{-22756579,-206454},Point{-22610826,-46704},Point{-22666568,-266249},Point{-22660305,-442615},Point{-22639565,-588149},Point{-22578167,-761273},Point{-22539958,-846897},Point{-22482682,-925136},Point{-22468714,-949605},Point{-22440726,-1010046},Point{-22395447,-1078548},Point{-22394858,-1078980},Point{-22388333,-1090410},Point{-22300652,-1182712},Point{-22171665,-1293408},Point{-21972306,-1388354},Point{-21889163,-1417398},Point{-21659717,-1430683},Point{-21560895,-1425934},Point{-21391356,-1368994},Point{-21369243,-1359253},Point{-21356827,-1358612},Point{-21334671,-1350830},Point{-21285118,-1333510},Point{-21253959,-1308473},Point{-21229035,-1297494},Point{-21167246,-1247456},Point{-21098219,-1183330},Point{-21057384,-1150518},Point{-21050915,-1141877},Point{-20987227,-1093743},Point{-20934489,-1047967},Point{-20845828,-926358},Point{-20748029,-777545},Point{-20730464,-743519},Point{-20713223,-708768},Point{-20689987,-588879},Point{-20620950,-554892},Point{-20507155,-498555},Point{-20397677,-441559},Point{-20365883,-411653},Point{-20285700,-361301},Point{-20283159,-358444},Point{-20250125,-336928},Point{-20174314,-236037},Point{-20094818,-146636},Point{-20076558,-125133},Point{-20065562,-100098},Point{-20059432,-79955},Point{-20058598,-78762},Point{-20056243,-69476},Point{-20024033,36369},Point{-20006467,68792},Point{-19994343,92162},Point{-19980540,179110},Point{-19955619,421610},Point{-20012743,692788},Point{-20023173,740095},Point{-20180373,1014726},Point{-20185569,1023323},Point{-20210005,1045744},Point{-20307070,1122404},Point{-20315233,1132964},Point{-20327355,1147046},Point{-20372924,1174415},Point{-20442485,1229353},Point{-20508804,1256024},Point{-20558565,1285911},Point{-20595821,1304929},Point{-20668609,1315985},Point{-20684600,1321704},Point{-20714228,1338197},Point{-20781734,1353762},Point{-20832960,1361086},Point{-20910406,1357833},Point{-20915745,1358252},Point{-20876054,1390313},Point{-20812623,1485959},Point{-20687395,1692688},Point{-20686729,1695362},Point{-20685697,1699772},Point{-20614743,1892673},Point{-20556354,2077545},Point{-20555423,2090474},Point{-20554945,2102471},Point{-20560417,2274485},Point{-20564695,2309905},Point{-20561396,2337232},Point{-20525680,2646750},Point{-20570329,2919926},Point{-20596331,3002333},Point{-20704678,3166795},Point{-20802636,3311706},Point{-20843684,3346772},Point{-20924123,3410168},Point{-21027290,3503802},Point{-21039704,3513640},Point{-21044380,3517832},Point{-21046804,3519267},Point{-21082958,3547919},Point{-21125573,3565895},Point{-21200956,3610519},Point{-21255331,3632064},Point{-21241554,3642397},Point{-21165017,3635112},Point{-20994400,3619306},Point{-20932202,3634618},Point{-20844802,3656366},Point{-20711180,3688795},Point{-20628479,3709633},Point{-20510353,3781291},Point{-20365232,3871180},Point{-20327928,3894744},Point{-20299088,3913331},Point{-20179561,4049331},Point{-20046014,4204087},Point{-20042647,4209074},Point{-20037829,4223484},Point{-19934043,4542286},Point{-19928042,4720691},Point{-19910166,4885489},Point{-19954615,5077323},Point{-19987545,5211322},Point{-20062917,5324702},Point{-20172604,5477707},Point{-20215954,5508054},Point{-20262534,5555339},Point{-20366461,5607426},Point{-20437280,5652078},Point{-20499851,5667757},Point{-20600464,5709139},Point{-20634180,5716025},Point{-20669141,5719762},Point{-20713046,5717746},Point{-20742415,5723492},Point{-20886068,5709711},Point{-21051137,5709506},Point{-21223994,5665708},Point{-21346369,5628842},Point{-21422206,5584605},Point{-21639399,5423583},Point{-21680641,5369234},Point{-21765238,5403693},Point{-21959520,5422099},Point{-21981938,5427821},Point{-22018869,5427722},Point{-22035039,5429254},Point{-22106654,5434195},Point{-22152369,5432653},Point{-22228084,5415440},Point{-22277983,5411947},Point{-22364004,5384541},Point{-22369415,5383311},Point{-22381451,5378983},Point{-22453883,5355906},Point{-22531391,5325060},Point{-22554189,5316861},Point{-22558821,5314143},Point{-22562983,5312487},Point{-22570579,5307245},Point{-22622614,5276716},Point{-22634803,5267043},Point{-22637625,5265879},Point{-22685081,5235353},Point{-22718125,5205422},Point{-22720952,5203471},Point{-22724810,5199367},Point{-22758288,5169043},Point{-22793394,5141182},Point{-22844658,5094634},Point{-22853987,5083539},Point{-22886887,5054671},Point{-22907109,5030376},Point{-23029608,4786627},Point{-23039478,4755289},Point{-23046010,4666759},Point{-23077118,4519001},Point{-23080354,4462682},Point{-23047748,4162328},Point{-23047458,4160411},Point{-23047313,4159985},Point{-22912067,3905275},Point{-22836208,3805020},Point{-22724724,3698460},Point{-22522443,3553562},Point{-22495292,3534791},Point{-22352774,3488855},Point{-22151610,3424036},Point{-22184344,3395788},Point{-22238041,3339062},Point{-22298010,3234154},Point{-22399256,3101224},Point{-22498423,2883060},Point{-22512887,2846022},Point{-22515903,2825997},Point{-22531199,2652008},Point{-22536618,2457557},Point{-22537256,2444689},Point{-22537437,2437388},Point{-22553503,2378353},Point{-22559494,2355847},Point{-22559255,2346584},Point{-22547632,1996395},Point{-22472902,1753626},Point{-22439967,1681161},Point{-22393105,1614567},Point{-22259647,1456821},Point{-22074225,1311724},Point{-22035916,1275180},Point{-21998436,1254305},Point{-21944598,1229428},Point{-21866199,1181358},Point{-21799790,1153980},Point{-21719341,1131575},Point{-21690863,1128307},Point{-21761865,1083475},Point{-21995984,806323},Point{-21998090,803801},Point{-21999494,802117},Point{-22048285,676921},Point{-22095350,658855},Point{-22268279,511728},Point{-22358511,435206},Point{-22340366,599440},Point{-22319911,779114},Point{-22378023,1094303},Point{-22384924,1125629},Point{-22388789,1142203},Point{-22502996,1346780},Point{-22575120,1465832},Point{-22661959,1547972},Point{-22849465,1706122},Point{-22871618,1716423},Point{-22984515,1758905},Point{-23062656,1784247},Point{-23118774,1806921},Point{-23142588,1810170},Point{-23147025,1811609},Point{-23176191,1818809},Point{-23183647,1818813},Point{-23208951,1828752},Point{-23275315,1836473},Point{-23334906,1836409},Point{-23407966,1846377},Point{-23447238,1850877},Point{-23459226,1849225},Point{-23484565,1836249},Point{-23555878,1836172},Point{-23559809,1835058},Point{-23846534,1713868},Point{-23870191,1695016},Point{-24035054,1529422},Point{-24046656,1519878},Point{-24053724,1510669},Point{-24059183,1505186},Point{-24063321,1498165},Point{-24091286,1461730},Point{-24120587,1428034},Point{-24168424,1346390},Point{-24216640,1268978},Point{-24229419,1233681},Point{-24266809,1142546},Point{-24273243,1112635},Point{-24317094,991515},Point{-24318103,988583},Point{-24318404,979995},Point{-24330954,618907},Point{-24299648,502122},Point{-24263483,363346},Point{-24234269,251820},Point{-24181885,168340},Point{-24109962,54167},Point{-24097547,34361},Point{-24109720,29246},Point{-24218273,-16444},Point{-24289851,-77407},Point{-24377165,-151743},Point{-24449505,-211692},Point{-24506203,-258801},Point{-24580042,-370878},Point{-24607620,-412807},Point{-24603783,-143998},Point{-24606303,-122275},Point{-24608396,-104443},Point{-24656422,8974},Point{-24655483,17416},Point{-24659104,283351},Point{-24746635,570343},Point{-24755023,606297},Point{-24759952,626159},Point{-24771082,644256},Point{-24779146,674610},Point{-24796759,704947},Point{-24911507,882059},Point{-24937668,907101},Point{-24946907,920851},Point{-24975819,943619},Point{-25013267,979465},Point{-25021902,986701},Point{-25029973,996629},Point{-25041410,1003051},Point{-25102428,1054187},Point{-25198626,1106082},Point{-25214678,1117199},Point{-25225014,1120317},Point{-25233958,1125142},Point{-25359773,1171548},Point{-25462536,1181213},Point{-25522497,1194607},Point{-25566839,1191022},Point{-25579279,1192192},Point{-25674171,1197817},Point{-25726245,1187195},Point{-25757769,1170097},Point{-25828738,1158948},Point{-25904392,1124001},Point{-26052701,1277233},Point{-26065934,1290750},Point{-26073686,1295139},Point{-26290508,1385666},Point{-26459779,1441238},Point{-26520323,1448326},Point{-26617971,1449965},Point{-26770823,1466510},Point{-26791052,1466334},Point{-26845556,1472255},Point{-26896520,1465572},Point{-26926788,1449436},Point{-27034237,1424110},Point{-27203017,1358985},Point{-27292486,1297606},Point{-27452247,1149041},Point{-27512686,1058075},Point{-27562220,952931},Point{-27560332,976364},Point{-27552946,1007762},Point{-27516741,1125524},Point{-27495885,1195264},Point{-27493359,1534985},Point{-27493283,1558530},Point{-27496186,1568980},Point{-27572837,1743867},Point{-27575088,1926682},Point{-27661726,2212505},Point{-27670117,2248472},Point{-27675048,2268341},Point{-27686173,2286430},Point{-27694238,2316787},Point{-27711851,2347124},Point{-27826599,2524236},Point{-27852769,2549286},Point{-27862004,2563031},Point{-27890900,2585787},Point{-27928358,2621642},Point{-27936990,2628876},Point{-27945063,2638806},Point{-27956503,2645229},Point{-28017519,2696364},Point{-28113732,2748267},Point{-28129775,2759378},Point{-28140105,2762494},Point{-28149049,2767319},Point{-28274865,2813725},Point{-28377633,2823390},Point{-28437592,2836784},Point{-28481932,2833199},Point{-28494373,2834369},Point{-28540640,2837112},Point{-28531524,2844300},Point{-28400410,2962699},Point{-28346182,3003425},Point{-28314580,3030606},Point{-28208170,3176386},Point{-28086946,3360409},Point{-28086537,3361207},Point{-28086188,3361888},Point{-28048175,3561838},Point{-28024697,3685703},Point{-28016128,3690150},Point{-27868582,3768634},Point{-27821840,3801184},Point{-27754049,3881455},Point{-27606944,4061844},Point{-27555870,4179360},Point{-27540068,4198252},Point{-27526672,4216254},Point{-27486106,4318713},Point{-27462426,4363763},Point{-27458755,4387796},Point{-27452852,4402705},Point{-27434715,4451749},Point{-27427383,4547475},Point{-27421508,4639479},Point{-27414138,4690135},Point{-27417341,4704747},Point{-27416137,4723598},Point{-27462617,4957468},Point{-27483898,5017083},Point{-27529473,5080556},Point{-27244641,5093325},Point{-27240902,5093047},Point{-27237887,5093684},Point{-27077858,5135170},Point{-27030611,5132785},Point{-26991138,5142566},Point{-26942235,5170329},Point{-26935417,5172097},Point{-26884214,5204578},Point{-26709314,5309707},Point{-26677433,5343378},Point{-26671555,5347466},Point{-26665198,5356299},Point{-26611347,5413172},Point{-26522464,5539631},Point{-26511406,5567419},Point{-26507376,5572473},Point{-26503279,5586571},Point{-26487655,5609834},Point{-26477131,5653546},Point{-26476744,5654519},Point{-26427972,5791953},Point{-26422409,5865219},Point{-26421695,5867693},Point{-26421988,5870761},Point{-26419902,5898235},Point{-26412879,5929124},Point{-26415796,5952313},Point{-26411909,6003499},Point{-26409452,6044680},Point{-26416866,6077093},Point{-26446317,6194976},Point{-26454164,6257366},Point{-26454888,6262857},Point{-26457655,6268550},Point{-26579172,6577962},Point{-26722149,6768739},Point{-26726954,6773708},Point{-26749811,6834296},Point{-26825876,6951379},Point{-26917515,7073238},Point{-26952186,7133438},Point{-26975891,7170867},Point{-27127478,7306037},Point{-27253551,7404002},Point{-27350768,7441909},Point{-27573763,7507071},Point{-27637554,7506340},Point{-27887919,7479395},Point{-27988617,7437560},Point{-28058374,7400721},Point{-28090025,7394275},Point{-28147442,7359472},Point{-28221229,7300402},Point{-28316430,7201920},Point{-28361931,7159645},Point{-28376495,7143621},Point{-28387718,7123157},Point{-28536855,6914248},Point{-28649869,6660771},Point{-28651891,6654913},Point{-28652662,6649717},Point{-28661307,6279830},Point{-28625555,6145697},Point{-28596388,6024603},Point{-28558376,5900511},Point{-28502802,5811455},Point{-28472969,5769041},Point{-28434848,5668796},Point{-28352153,5565175},Point{-28449848,5576747},Point{-28543437,5584961},Point{-28591634,5577209},Point{-28762207,5529722},Point{-28904495,5490854},Point{-29047588,5402779},Point{-29224144,5286611},Point{-29396477,5078535},Point{-29479034,5232452},Point{-29507253,5284863},Point{-29522487,5300075},Point{-29779169,5547107},Point{-29941713,5632807},Point{-30053331,5675346},Point{-30151509,5703624},Point{-30289584,5715951},Point{-30484015,5713212},Point{-30535949,5717706},Point{-30567604,5716767},Point{-30795380,5660277},Point{-30926332,5607920},Point{-31044014,5526985},Point{-31186450,5393202},Point{-31197183,5377159},Point{-31219069,5362314},Point{-31237879,5338054},Point{-31257132,5303020},Point{-31313282,5163737},Point{-31332645,5122092},Point{-31335694,5108143},Point{-31340000,5097461},Point{-31376274,4933908},Point{-31381488,4828789},Point{-31366070,4751273},Point{-31347753,4616135},Point{-31346031,4561057},Point{-31337716,4521376},Point{-31300346,4410038},Point{-31300479,4409213},Point{-31296705,4224840},Point{-31286275,4139830},Point{-31255675,4046098},Point{-31252732,3997618},Point{-31228916,3926345},Point{-31216409,3868071},Point{-31173782,3775805},Point{-31173780,3775802},Point{-31162739,3746287},Point{-31062540,3595819},Point{-31003808,3530323},Point{-30937324,3474543},Point{-30890561,3446079},Point{-30842231,3409829},Point{-30807413,3395468},Point{-30804847,3393906},Point{-30668433,3333108},Point{-30646319,3330513},Point{-30530844,3299307},Point{-30377487,3297158},Point{-30170348,3317835},Point{-30118143,3336369},Point{-30028423,3377867},Point{-29973230,3396737},Point{-29905214,3244518},Point{-29791530,3086565},Point{-29747199,3008426},Point{-29704762,2963248},Point{-29683297,2944459},Point{-29646999,2908182},Point{-29545359,2818366},Point{-29413119,2742010},Point{-29319243,2704944},Point{-29256161,2688407},Point{-29247129,2685059},Point{-29241824,2684649},Point{-29224330,2680063},Point{-29087435,2668620},Point{-29045098,2669446},Point{-29043352,2669311},Point{-29031725,2669587},Point{-29049360,2658182},Point{-29260627,2506533},Point{-29366583,2385122},Point{-29464980,2247751},Point{-29567542,1958461},Point{-29589609,1890308},Point{-29593185,1790580},Point{-29597988,1536066},Point{-29545462,1347922},Point{-29545014,1330391},Point{-29543108,1289006},Point{-29484362,1126370},Point{-29427589,979603},Point{-29418522,953036},Point{-29408407,926526},Point{-29314647,794180},Point{-29239548,695290},Point{-29180996,628951},Point{-29040643,530438},Point{-28937092,464020},Point{-28853067,435379},Point{-28810483,415081},Point{-28744855,374903},Point{-28680962,348628},Point{-28603158,327029},Point{-28450717,309571},Point{-28413707,309149},Point{-28354497,303248},Point{-28330417,308199},Point{-28321674,308099},Point{-28307291,312953},Point{-28142359,346862},Point{-28047238,373466},Point{-27871905,491196},Point{-27778832,562176},Point{-27692716,688218},Point{-27635688,779336},Point{-27635478,698896},Point{-27636857,692634},Point{-27644088,602152},Point{-27635032,528718},Point{-27634994,514266},Point{-27629804,486331},Point{-27626780,461806},Point{-27627320,331175},Point{-27603281,199301},Point{-27569022,79674},Point{-27529243,-1451},Point{-27467127,-107221},Point{-27464860,-110746},Point{-27528114,-110594},Point{-27652485,-109057},Point{-27756223,-108459},Point{-27851013,-135115},Point{-27948058,-163008},Point{-28056327,-205854},Point{-28163991,-265927},Point{-28261006,-340219},Point{-28355944,-437622},Point{-28375533,-451164},Point{-28395804,-471389},Point{-28449844,-515075},Point{-28495804,-566445},Point{-28551361,-656139},Point{-28637228,-789405},Point{-28651129,-843729},Point{-28650574,-858983},Point{-28668289,-904223},Point{-28690838,-1022820},Point{-28689748,-1234088},Point{-28668101,-1344076},Point{-28597940,-1516693},Point{-28500392,-1670153},Point{-28434524,-1741101},Point{-28379265,-1784106},Point{-28372738,-1793667},Point{-28308419,-1839242},Point{-28299633,-1846080},Point{-28232983,-1909975},Point{-28180428,-1947374},Point{-28014957,-2030928},Point{-27820970,-2099084},Point{-27785066,-2107450},Point{-27781453,-2108286},Point{-27676570,-2172751},Point{-27415123,-2258318},Point{-27390501,-2265119},Point{-27363912,-2265680},Point{-27271029,-2276225},Point{-27165823,-2297781},Point{-27116837,-2300580}};
    coord_t clip_dist_scaled = 480000;
    Slic3r::MutablePolygon mp;
    mp.assign(polygon, polygon.size() * 2);
    smooth_outward(mp, clip_dist_scaled);
    mp.polygon(polygon);
    assert(!polygon.empty());
}

