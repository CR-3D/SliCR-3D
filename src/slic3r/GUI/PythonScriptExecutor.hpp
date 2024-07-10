#ifndef slic3r_PythonScriptExecutor_hpp_
#define slic3r_PythonScriptExecutor_hpp_

#include <string>
#include <pybind11/include/pybind11/pybind11.h>
#include <pybind11/include/pybind11/embed.h>

namespace py = pybind11;

namespace Slic3r { namespace GUI {

class PythonScriptExecutor {
public:
    PythonScriptExecutor();
    ~PythonScriptExecutor();
    
    void set_script_path(const std::string& script_path);
    
    py::object run_python_script_result(
                                        const std::string gcode_path,
                                        const char* script_name,
                                        const char* function_name
                                    );
    
    static py::module_ import(const char *name);
    
private:
    std::string m_script_path;
    static py::scoped_interpreter guard_; // Keep the Python interpreter alive
};

}
}
#endif // slic3r_PythonScriptExecutor_hpp_
