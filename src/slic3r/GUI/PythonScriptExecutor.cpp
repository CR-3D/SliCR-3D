#include <iostream>
#include "pybind11/pybind11.h"
#include "pybind11/embed.h"
#include "../libslic3r/GCode/ArcOverhang.hpp"
#include "PythonScriptExecutor.hpp"

namespace py = pybind11;



namespace Slic3r {
namespace GUI {

py::scoped_interpreter PythonScriptExecutor::guard_;


PythonScriptExecutor::PythonScriptExecutor() {
    // Python interpreter is initialized in the constructor
}

PythonScriptExecutor::~PythonScriptExecutor() {
    // Destructor will finalize the Python interpreter
}

void PythonScriptExecutor::set_script_path(const std::string& script_path) {
    m_script_path = script_path;
}

py::object PythonScriptExecutor::run_python_script_result(
                                                          const std::string& gcode_path,
                                                          const char* script_name,
                                                          const char* function_name
                                                          ) {
    
    try {
        // Initialize the Python interpreter if not already initialized
        if (!Py_IsInitialized()) {

        }
        else {
            std::cout << "Python interpreter already initialized." << std::endl;
        }
        
        py::object sys = py::module::import("sys");
        py::list path = sys.attr("path");
        
        // Add the directory containing the script to sys.path
        std::string script_dir = m_script_path.substr(0, m_script_path.find_last_of("/\\"));
        path.append(script_dir);
        
        std::cout << "Script directory added to sys.path: " << script_dir << std::endl;
        
        // Import the script as a module
        py::module script_module = py::module::import(script_name);
        
        
        // Open the G-code file in Python and pass the file object to the script
        py::object builtins = py::module::import("builtins");
        py::object open = builtins.attr("open");
        py::object gcode_file = open(gcode_path, "r");
        
        std::cout << "G-code file opened: " << gcode_path << std::endl;
        
        // Call the main function from the script
        script_module.attr(function_name)(gcode_file, gcode_path, true);
        
        //Debug
        py::object result = script_module.attr("print_a_string")();
        
        std::cout << "Python script executed successfully." << std::endl;
        
        return result;
        
    }
    catch (const py::error_already_set& e) {
        std::cerr << "Python error: " << e.what() << std::endl;
        return py::none(); // Return None in case of error
    }
    catch (const std::exception& e) {
        std::cerr << "Error running Python script: " << e.what() << std::endl;
        return py::none(); // Return None in case of error
    }
}

py::module_ PythonScriptExecutor::import(const char *name) {
    PyObject *obj = PyImport_ImportModule(name);
    if (!obj) {
        throw py::error_already_set();
    }
    return py::reinterpret_steal<py::module_>(obj);
}

PYBIND11_MODULE(python_module, m) {
    py::class_<PythonScriptExecutor>(m, "PythonScriptExecutor")
        .def(py::init<>())
        .def("set_script_path", &PythonScriptExecutor::set_script_path)
        .def("run_python_script_result", &PythonScriptExecutor::run_python_script_result,
             py::arg("gcode_path"), py::arg("script_name"), py::arg("function_name"))
        .def_static("import", &PythonScriptExecutor::import, "Import a Python module");
}
}
}
