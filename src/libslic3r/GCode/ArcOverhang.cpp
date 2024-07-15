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

void ArcOverhang::set_script_arc_overhang(std::string gcode) {
    
    //TODO: Make it dynamic
    executor_.set_script_path("C:/CR3D/SliCR-3D/src/slic3r/Scripts");

   /* py::object PythonScriptExecutor::run_python_script_result(
                                                              const std::string& gcode_path,
                                                              const char* script_name,
                                                              const char* function_name
                                                              ) {*/
    executor_.run_python_script_result(gcode,
                                       "slicr-3d_arc_overhang_post_processing_script",
                                       "main");

}


}
