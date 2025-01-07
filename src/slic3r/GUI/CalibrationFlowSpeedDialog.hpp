#ifndef slic3r_GUI_CalibrationFlowSpeedDialog_hpp_
#define slic3r_GUI_CalibrationFlowSpeedDialog_hpp_

#include "CalibrationAbstractDialog.hpp"
#include "libslic3r/Flow.hpp"
#include "Widgets/ComboBox.hpp"

#include <tuple>

namespace Slic3r { 
namespace GUI {

class CalibrationFlowSpeedDialog : public CalibrationAbstractDialog
{

public:
    CalibrationFlowSpeedDialog(GUI_App* app, MainFrame* mainframe) : CalibrationAbstractDialog(app, mainframe, "Extruder flow calibration (by weight)") { create(boost::filesystem::path("calibration") / "extruder_flow","extruder_flow.html", wxSize(900, 500));  }
    virtual ~CalibrationFlowSpeedDialog() {}
    
protected:
    void create_buttons(wxStdDialogButtonSizer* sizer) override;
    void create_speed(wxCommandEvent& event_args);
    void create_flow(wxCommandEvent& event_args);
    void create_overlap(wxCommandEvent& event_args);
    void create_geometry(float min_flow, float max_flow, float min_speed, float max_speed, float min_overlap, float max_overlap);
    std::tuple<float,float, Flow> get_cube_size(float overlap);
    
    ComboBox* cmb_gram;
    ComboBox* cmb_nb_steps;
    wxTextCtrl* txt_min_speed;
    wxTextCtrl* txt_max_speed;
    wxTextCtrl* txt_min_flow;
    wxTextCtrl* txt_max_flow;
    ComboBox* cmb_min_overlap;
    ComboBox* cmb_max_overlap;
};

} // namespace GUI
} // namespace Slic3r

#endif
