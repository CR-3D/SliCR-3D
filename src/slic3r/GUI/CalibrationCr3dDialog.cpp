#include "CalibrationCr3dDialog.hpp"
#include "I18N.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Utils.hpp"
#include "GLCanvas3D.hpp"
#include "GUI.hpp"
#include "GUI_ObjectList.hpp"
#include "Plater.hpp"
#include "Tab.hpp"
#include <wx/scrolwin.h>
#include <wx/display.h>
#include <wx/file.h>
#include "wxExtensions.hpp"

#if ENABLE_SCROLLABLE
static wxSize get_screen_size(wxWindow* window)
{
    const auto idx = wxDisplay::GetFromWindow(window);
    wxDisplay display(idx != wxNOT_FOUND ? idx : 0u);
    return display.GetClientArea().GetSize();
}
#endif // ENABLE_SCROLLABLE

namespace Slic3r {
namespace GUI {

void CalibrationCr3dAbstractDialog::create_geometry(std::string calibration_path) {
    Plater* plat = this->main_frame->plater();
    Model& model = plat->model();
    gui_app->app_config->set("autocenter", "1");

    if (!plat->new_project(L("CR-3D Calibration cube")))
        return;

    //GLCanvas3D::set_warning_freeze(true);
    std::vector<size_t> objs_idx = plat->load_files(std::vector<std::string>{
        Slic3r::resources_dir()+"/calibration/cr3d"+ calibration_path
    }, true, false, false, false);

    //update plater
    //GLCanvas3D::set_warning_freeze(false);
    plat->changed_objects(objs_idx);
    plat->is_preview_shown();
    
    //update everything, easier to code.
    ObjectList* obj = this->gui_app->obj_list();
    obj->update_after_undo_redo();

    plat->reslice();
    gui_app->app_config->set("autocenter", "0");

}

void CalibrationCr3dCubeDialog::create_buttons(wxStdDialogButtonSizer* buttons){
    wxButton* bt = new wxButton(this, wxID_FILE1, _(L("Single Extruder Calibration Cube")));
    bt->Bind(wxEVT_BUTTON, &CalibrationCr3dCubeDialog::create_geometry_single, this);
    bt->SetToolTip(_L("CR-3D Single Extruder calibration Cube"));
    buttons->Add(bt);
    bt->SetForegroundColour(*wxBLACK); // Set text color to black
    bt->SetBackgroundColour(*wxBLACK); // Set text color to black

    buttons->AddSpacer(10);

    bt = new wxButton(this, wxID_FILE1, _(L("Dual Extruder Calibration Cube")));
    bt->Bind(wxEVT_BUTTON, &CalibrationCr3dCubeDialog::create_geometry_dual, this);
    bt->SetToolTip(_L("CR-3D Dual Extruder calibration Cube"));
    bt->SetForegroundColour(*wxBLACK); // Set text color to black
    bt->SetBackgroundColour(*wxBLACK); // Set text color to black

    buttons->Add(bt);
}

void CalibrationCr3dSampleCardDialog::create_buttons(wxStdDialogButtonSizer* buttons){
    wxButton* bt = new wxButton(this, wxID_FILE1, _(L("Box")));
    bt->Bind(wxEVT_BUTTON, &CalibrationCr3dSampleCardDialog::create_geometry_box, this);
    bt->SetToolTip(_L("CR-3D Sample Box"));
    bt->SetForegroundColour(*wxBLACK); // Set text color to black
    bt->SetBackgroundColour(*wxBLACK); // Set text color to black

    buttons->Add(bt);

    buttons->AddSpacer(10);

    bt = new wxButton(this, wxID_FILE1, _(L("Card")));
    bt->Bind(wxEVT_BUTTON, &CalibrationCr3dSampleCardDialog::create_geometry_card, this);
    bt->SetToolTip(_L("CR-3D Sample Card"));
    bt->SetForegroundColour(*wxBLACK); // Set text color to black
    bt->SetBackgroundColour(*wxBLACK); // Set text color to black

    buttons->Add(bt);
}

void CalibrationCr3dMasterSpoolDialog::create_buttons(wxStdDialogButtonSizer* buttons){
    
    
}

void CalibrationCr3dIDEXDialog::create_buttons(wxStdDialogButtonSizer *buttons)
{
    wxButton *bt = new wxButton(this, wxID_FILE1, _(L("Generate")));
    bt->Bind(wxEVT_BUTTON, &CalibrationCr3dIDEXDialog::create_geometry_single, this);
    bt->SetToolTip(_L("CR-3D IDEX Calibration"));
    bt->SetForegroundColour(*wxBLACK); // Set text color to black
    bt->SetBackgroundColour(*wxBLACK); // Set text color to black
   
    buttons->Add(bt);
}

} // namespace GUI
} // namespace Slic3r
