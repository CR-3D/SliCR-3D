///|/ Copyright (c) Prusa Research 2020 - 2023 Oleksandra Iushchenko @YuSanka, David Kocík @kocikdav, Enrico Turri @enricoturri1966, Vojtěch Bubník @bubnikv
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "BulkExportDialog.hpp"

#include <cstddef>
#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>

#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include "libslic3r/PresetBundle.hpp"

#include "GUI.hpp"
#include "GUI_App.hpp"
#include "format.hpp"
#include "Tab.hpp"

using Slic3r::GUI::format_wxstr;

namespace Slic3r {
namespace GUI {

constexpr auto BORDER_W = 10;

//-----------------------------------------------
//          BulkExportDialog::Item
//-----------------------------------------------


void BulkExportDialog::Item::init_input_name_ctrl(wxBoxSizer* input_path_sizer, const std::string path)
{
#ifdef _WIN32
    long style = wxBORDER_SIMPLE;
#else
    long style = 0L;
#endif
    m_text_ctrl = new wxTextCtrl(m_parent, wxID_ANY, from_u8(path), wxDefaultPosition, wxSize(45 * wxGetApp().em_unit(), -1), style);
    wxGetApp().UpdateDarkUI(m_text_ctrl);
    m_text_ctrl->Bind(wxEVT_TEXT, [this](wxCommandEvent&) { update(); });

    input_path_sizer->Add(m_text_ctrl, 1, wxEXPAND, BORDER_W);
}

BulkExportDialog::Item::Item(wxWindow* parent, wxBoxSizer* sizer, PrintToExport& pte):
    m_parent(parent),
    m_print_to_export(&pte),
    m_valid_bmp(new wxStaticBitmap(m_parent, wxID_ANY, *get_bmp_bundle("tick_mark"))),
    m_valid_label(new wxStaticText(m_parent, wxID_ANY, ""))
{
    m_valid_label->SetFont(wxGetApp().bold_font());

    wxBoxSizer* input_path_sizer = new wxBoxSizer(wxHORIZONTAL);
    input_path_sizer->Add(m_valid_bmp,    0, wxALIGN_CENTER_VERTICAL | wxRIGHT, BORDER_W);
    init_input_name_ctrl(input_path_sizer, pte.output_path.filename().string());

    sizer->Add(input_path_sizer,0, wxEXPAND | wxTOP, BORDER_W);
    sizer->Add(m_valid_label,   0, wxEXPAND | wxLEFT,   3*BORDER_W);

    update();
}

void BulkExportDialog::Item::update()
{
    m_path = into_u8(m_text_ctrl->GetValue());

    m_valid_type = ValidationType::Valid;
    wxString info_line;

    const char* unusable_symbols = "<>[]:/\\|?*\"";

    for (size_t i = 0; i < std::strlen(unusable_symbols); i++) {
        if (m_path.find_first_of(unusable_symbols[i]) != std::string::npos) {
            info_line = _L("The following characters are not allowed in the name") + ": " + unusable_symbols;
            m_valid_type = ValidationType::NoValid;
            break;
        }
    }

    bool existing = false;// is_existing(m_path);
    if (m_valid_type == ValidationType::Valid && existing) {
        info_line = _L("This name is already used, use another.");
        m_valid_type = ValidationType::Warning;
    }

    if (m_valid_type == ValidationType::Valid && m_path.empty()) {
        info_line = _L("The name cannot be empty.");
        m_valid_type = ValidationType::NoValid;
    }

#ifdef __WXMSW__
    const int max_path_length = MAX_PATH;
#else
    const int max_path_length = 255;
#endif

    if (m_valid_type == ValidationType::Valid && m_path.length() >= max_path_length) {
        info_line = _L("The name is too long.");
        m_valid_type = ValidationType::NoValid;
    }

    if (m_valid_type == ValidationType::Valid && m_path.find_first_of(' ') == 0) {
        info_line = _L("The name cannot start with space character.");
        m_valid_type = ValidationType::NoValid;
    }

    if (m_valid_type == ValidationType::Valid && m_path.find_last_of(' ') == m_path.length()-1) {
        info_line = _L("The name cannot end with space character.");
        m_valid_type = ValidationType::NoValid;
    }

    m_valid_label->SetLabel(info_line);
    m_valid_label->Show(!info_line.IsEmpty());

    update_valid_bmp();

    if (is_valid()) {
        boost::filesystem::path field = m_print_to_export->output_path.parent_path();
        m_print_to_export->output_path = field / m_path;
    }

    m_parent->Layout();
}

void BulkExportDialog::Item::update_valid_bmp()
{
    std::string bmp_name =  m_valid_type == ValidationType::Warning ? "exclamation_manifold" :
                            m_valid_type == ValidationType::NoValid ? "exclamation"          : "tick_mark" ;
    m_valid_bmp->SetBitmap(*get_bmp_bundle(bmp_name));
}

//-----------------------------------------------
//          BulkExportDialog
//-----------------------------------------------

BulkExportDialog::BulkExportDialog(wxWindow* parent, std::vector<PrintToExport>& exports)
    : DPIDialog(parent, wxID_ANY, exports.size() == 1 ? _L("Save bed") : _L("Save beds"),
                wxDefaultPosition, wxSize(45 * wxGetApp().em_unit(), 5 * wxGetApp().em_unit()), wxDEFAULT_DIALOG_STYLE | wxICON_WARNING)
{
    this->SetFont(wxGetApp().normal_font());

#ifndef __WXMSW__
    SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW));
#endif // __WXMSW__

    wxBoxSizer* topSizer = new wxBoxSizer(wxVERTICAL);

    m_sizer = new wxBoxSizer(wxVERTICAL);

    for (PrintToExport& exp : exports)
        AddItem(exp);

    // Add dialog's buttons
    wxStdDialogButtonSizer* btns = this->CreateStdDialogButtonSizer(wxOK | wxCANCEL);
    wxButton* btnOK = static_cast<wxButton*>(this->FindWindowById(wxID_OK, this));
    btnOK->Bind(wxEVT_UPDATE_UI, [this](wxUpdateUIEvent& evt)   { evt.Enable(enable_ok_btn()); });

    topSizer->Add(m_sizer,  0, wxEXPAND | wxALL, BORDER_W);

    topSizer->Add(btns,     0, wxEXPAND | wxALL, BORDER_W);

    SetSizer(topSizer);
    topSizer->SetSizeHints(this);

    this->CenterOnScreen();

#ifdef _WIN32
    wxGetApp().UpdateDlgDarkUI(this);
#endif
}

BulkExportDialog::~BulkExportDialog()
{
    for (auto item : m_items)
        delete item;
}

void BulkExportDialog::AddItem(PrintToExport& pte)
{
    m_items.emplace_back(new Item{ this, m_sizer, pte });
}

bool BulkExportDialog::enable_ok_btn() const
{
    for (const Item* item : m_items)
        if (!item->is_valid())
            return false;

    return true;
}

bool BulkExportDialog::Layout()
{
    const bool ret = DPIDialog::Layout();
    this->Fit();
    return ret;
}

void BulkExportDialog::on_dpi_changed(const wxRect& suggested_rect)
{
    const int& em = em_unit();

    msw_buttons_rescale(this, em, { wxID_OK, wxID_CANCEL });

    for (Item* item : m_items)
        item->update_valid_bmp();

    const wxSize& size = wxSize(65 * em, 35 * em);
    SetMinSize(size);

    Fit();
    Refresh();
}

}}    // namespace Slic3r::GUI
