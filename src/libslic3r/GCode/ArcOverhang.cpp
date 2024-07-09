#include "ArcOverhang.hpp"
#include "pybind11/include/pybind11/pybind11.h"
#include "pybind11/include/pybind11/embed.h"
#include <slic3r/GUI/PythonScriptExecutor.hpp>

namespace py = pybind11;

namespace Slic3r {


ArcOverhang::ArcOverhang() : executor_() {
    // Default constructor implementation (initializes executor_ by default)
}

ArcOverhang::~ArcOverhang() {
    // Destructor implementation (if needed)
}

void ArcOverhang::set_script_arc_overhang(const std::string& script_path) {
    //executor_.set_script_path(script_path);
    std::string gcode_path = "";
    executor_.run_python_script_result(gcode_path, "slicr-3d_arc_overhang_post_processing_script.py", "main");

}


}
