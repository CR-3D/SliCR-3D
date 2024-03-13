#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>

#include <libslic3r/Format/STL.hpp>
#include <libslic3r/Model.hpp>
#include <libslic3r/LocalesUtils.hpp>
#include <libslic3r/SLA/IndexedMesh.hpp>

#include "ClipboardXX/include/clipboardxx.hpp"

int main(int argc, char const *argv[])
{
    if (argc != 2) {
        std::cout<<"usage: stl_to_cpp \"path/to/stl.stl\"\n";
        return 0;
    }
    std::string path_str = argv[1];
    if(path_str.front() == '\"' && path_str.back() == '\"')
        path_str = path_str.substr(1,path_str.size()-2);
    Slic3r::Model model;
    bool result = load_stl(path_str.c_str(), &model, "obj");
    if (!result) {
        std::cout << "error, can't read '" << path_str << "'\n";
        return 0;
    }
    clipboardxx::clipboard clipboard;
    //TriangleMesh tm2 = TriangleMesh(std::vector<Vec3f>{{-5, -5, -0.1}},std::vector<Vec3i32>{{1,4,3}});
    std::stringstream out_cpp;
    int idx_obj = 0;
    for (Slic3r::ModelObject* obj : model.objects) {
        int idx_vol = 0;
        for(Slic3r::ModelVolume *vol : obj->volumes) {
            Slic3r::TriangleMesh mesh = vol->mesh();
            Slic3r::sla::IndexedMesh indexed_mesh(mesh); // more user-friendly
            out_cpp << "TriangleMesh vol_"<< idx_obj << "_" << idx_vol <<" = TriangleMesh(std::vector<Vec3f>{";
            int ptidx= 0;
            for(const Slic3r::Vec3f &pt : indexed_mesh.vertices())
                out_cpp << (0==ptidx++?"{":",{") << Slic3r::to_string_nozero(pt.x(), 7)
                         << ',' << Slic3r::to_string_nozero(pt.y(), 7)
                         << ',' << Slic3r::to_string_nozero(pt.z(), 7) << '}';
            out_cpp << "},std::vector<Vec3i32>{";
            ptidx= 0;
            for(const Slic3r::Vec3i32 &tri : indexed_mesh.indices())
                out_cpp << (0==ptidx++?"{":",{") << tri(0) << ',' << tri(1) << ',' << tri(2) << '}';
            out_cpp << "});\n";

            idx_vol++;
        }
        out_cpp << "\n";
        idx_obj++;
    }

    clipboard << out_cpp.str();
    std::cout << out_cpp.str();

    return 0;
}
