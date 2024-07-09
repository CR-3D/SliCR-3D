#ifndef slic3r_ArcOverhang_hpp_
#define slic3r_ArcOverhang_hpp_

#include <slic3r/GUI/PythonScriptExecutor.hpp>

#include <string>
#include <pybind11/include/pybind11/pybind11.h>
#include <pybind11/include/pybind11/embed.h>

#include <vector>

namespace py = pybind11;

namespace Slic3r {

class ArcOverhang {
public:
    ArcOverhang(); // Default constructor
    ~ArcOverhang();
    
    void set_script_arc_overhang(const std::string& script_path);
    
private:
    GUI::PythonScriptExecutor executor_;
};
}
#endif // slic3r_ArcOverhang_hpp_

