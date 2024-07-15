#include <iostream>
#include <pybind11/include/pybind11/pybind11.h>
#include <pybind11/include/pybind11/embed.h>
#include "../libslic3r/GCode/ArcOverhang.hpp"
#include "PythonScriptExecutor.hpp"
#include <boost/filesystem.hpp>

namespace py = pybind11;
namespace fs = boost::filesystem;

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
                                                          const std::string gcode_path,
                                                          const char* script_name,
                                                          const char* function_name
                                                          ) {
    try {
        // Initialize the Python interpreter if not already initialized
        if (!Py_IsInitialized()) {
            py::initialize_interpreter();
        }
        else {
            std::cout << "Python interpreter already initialized." << std::endl;
        }

        std::cout << "G-code path: " << gcode_path << std::endl;
        std::cout << "Script name: " << script_name << std::endl;
        std::cout << "Function name: " << function_name << std::endl;

        // Modify sys.path to include the script directory
        py::module sys = py::module::import("sys");
        py::list path = sys.attr("path");

        std::string script_dir = "C:\\CR3D\\SliCR-3D\\src\\slic3r\\Scripts";
        path.append(script_dir);

        std::cout << "Script directory added to sys.path: " << script_dir << std::endl;

        // Import the script as a module by its name
        py::module script_module = py::module::import(script_name);

        std::cout << "Script module imported: " << script_name << std::endl;

        // Open the G-code file in Python and pass the file object to the script
        py::object builtins = py::module::import("builtins");
        py::object open = builtins.attr("open");
        py::object gcode_file = open(gcode_path, "r");
        
        std::cout << "G-code file opened: " << gcode_path << std::endl;
        
        // Call the specified function from the script with the new G-code file path
        py::object result = script_module.attr(function_name)(gcode_file, gcode_path, true);

        std::cout << "Python script executed successfully." << std::endl;

        // Return the result of the script function call
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
