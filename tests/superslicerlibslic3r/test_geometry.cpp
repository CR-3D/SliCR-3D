
#include <catch_main.hpp>

#include <libslic3r/Point.hpp>
#include <libslic3r/BoundingBox.hpp>
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

    
