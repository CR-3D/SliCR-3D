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

namespace fs = boost::filesystem;

namespace Slic3r {
namespace GUI {

constexpr auto BORDER_W = 10;

void BulkExportDialog::Item::init_input_name_ctrl(wxBoxSizer* input_path_sizer, const std::string &path)
{
#ifdef _WIN32
    const long style = wxBORDER_SIMPLE;
#else
    const long style = 0L;
#endif
    m_text_ctrl = new wxTextCtrl(m_parent, wxID_ANY, from_u8(path), wxDefaultPosition, wxSize(45 * wxGetApp().em_unit(), -1), style);
    wxGetApp().UpdateDarkUI(m_text_ctrl);
    m_text_ctrl->Bind(wxEVT_TEXT, [this](wxCommandEvent&) { update(); });

    input_path_sizer->Add(m_text_ctrl, 1, wxEXPAND, BORDER_W);
}
BulkExportDialog::Item::Item(
    wxWindow *parent,
    wxBoxSizer *sizer,
    const boost::filesystem::path &path,
    Validator validator
):
    path(path),
    m_parent(parent),
    m_valid_bmp(new wxStaticBitmap(m_parent, wxID_ANY, *get_bmp_bundle("tick_mark"))),
    m_valid_label(new wxStaticText(m_parent, wxID_ANY, "")),
    m_validator(std::move(validator)),
    m_directory(path.parent_path())
{
    m_valid_label->SetFont(wxGetApp().bold_font());

    wxBoxSizer* input_path_sizer = new wxBoxSizer(wxHORIZONTAL);
    input_path_sizer->Add(m_valid_bmp, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, BORDER_W);
    init_input_name_ctrl(input_path_sizer, path.filename().string());

    sizer->Add(input_path_sizer,0, wxEXPAND | wxTOP, BORDER_W);
    sizer->Add(m_valid_label,   0, wxEXPAND | wxLEFT,   3*BORDER_W);

    update();
}

#ifdef __WXMSW__
constexpr int max_path_length = MAX_PATH;
#else
constexpr int max_path_length = 255;
#endif

struct PathValidator {
    std::reference_wrapper<std::vector<std::unique_ptr<BulkExportDialog::Item>>> items;
    using ItemStatus = BulkExportDialog::ItemStatus;

    bool is_duplicate(const fs::path &path) {
        const int64_t count{std::count_if(
            items.get().begin(),
            items.get().end(),
            [&](const auto &item){
                return item->path == path;
            }
        )};

        return count >= 2;
    }

    std::pair<BulkExportDialog::ItemStatus, wxString> operator()(
        const fs::path &path,
        const std::string &filename
    ) {
        const char* unusable_symbols = "<>[]:/\\|?*\"";
        for (size_t i = 0; i < std::strlen(unusable_symbols); i++) {
            if (filename.find_first_of(unusable_symbols[i]) != std::string::npos) {
                return {
                    ItemStatus::NoValid,
                    _L("The following characters are not allowed in the name") + ": " + unusable_symbols
                };
            }
        }

        if (filename.empty()) {
            return {
                ItemStatus::NoValid,
                _L("The name cannot be empty.")
            };
        }

        if (path.string().length() >= max_path_length) {
            return {
                ItemStatus::NoValid,
                _L("The name is too long.")
            };
        }

        if (filename.find_first_of(' ') == 0) {
            return {
                ItemStatus::NoValid,
                _L("The name cannot start with space character.")
            };
        }

        if (filename.find_last_of(' ') == filename.length()-1) {
            return {
                ItemStatus::NoValid,
                _L("The name cannot end with space character.")
            };
        }

        if (is_duplicate(path)) {
            return {
                ItemStatus::NoValid,
                _L("This name is already used, use another.")
            };
        }

        if (fs::exists(path)) {
            return {
                ItemStatus::Warning,
                _L("The file already exists!")
            };
        }

        return {ItemStatus::Valid, ""};
    }
};

void BulkExportDialog::Item::update()
{
    std::string filename{into_u8(m_text_ctrl->GetValue())};
    path = m_directory / filename;

    // Validator needs to be called after path is set!
    // It has has a reference to all items and searches
    // for duplicates;
    const auto [status, info_line]{m_validator(path, filename)};

    m_valid_label->SetLabel(info_line);
    m_valid_label->Show(!info_line.IsEmpty());
    m_status = status;

    update_valid_bmp();

    m_parent->Layout();
}

std::string get_bmp_name(const BulkExportDialog::ItemStatus status) {
    using ItemStatus = BulkExportDialog::ItemStatus;
    switch(status) {
        case ItemStatus::Warning: return "exclamation_manifold";
        case ItemStatus::NoValid: return "exclamation";
        case ItemStatus::Valid: return "tick_mark";
    }
    return ""; // unreachable
}

void BulkExportDialog::Item::update_valid_bmp()
{
    m_valid_bmp->SetBitmap(*get_bmp_bundle(get_bmp_name(m_status)));
}

BulkExportDialog::BulkExportDialog(const std::vector<fs::path> &paths):
    DPIDialog(
        nullptr,
        wxID_ANY,
        paths.size() == 1 ? _L("Save bed") : _L("Save beds"),
        wxDefaultPosition,
        wxSize(45 * wxGetApp().em_unit(), 5 * wxGetApp().em_unit()),
        wxDEFAULT_DIALOG_STYLE | wxICON_WARNING
    )
{
    this->SetFont(wxGetApp().normal_font());

#ifndef __WXMSW__
    SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW));
#endif // __WXMSW__

    wxBoxSizer* topSizer = new wxBoxSizer(wxVERTICAL);

    m_sizer = new wxBoxSizer(wxVERTICAL);

    for (const fs::path& path : paths) {
        AddItem(path);
    }

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

void BulkExportDialog::AddItem(const fs::path& path)
{
    m_items.push_back(std::make_unique<Item>(this, m_sizer, path, PathValidator{m_items}));
}

bool BulkExportDialog::enable_ok_btn() const
{
    for (const auto &item : m_items)
        if (!item->is_valid()) {
            return false;
        }

    return true;
}

bool BulkExportDialog::Layout()
{
    const bool ret = DPIDialog::Layout();
    this->Fit();
    return ret;
}

std::vector<boost::filesystem::path> BulkExportDialog::get_paths() const {
    std::vector<boost::filesystem::path> result;
    std::transform(
        m_items.begin(),
        m_items.end(),
        std::back_inserter(result),
        [](const auto &item){
            return item->path;
        }
    );
    return result;
}

void BulkExportDialog::on_dpi_changed(const wxRect&)
{
    const int& em = em_unit();

    msw_buttons_rescale(this, em, { wxID_OK, wxID_CANCEL });

    for (auto &item : m_items)
        item->update_valid_bmp();

    const wxSize& size = wxSize(65 * em, 35 * em);
    SetMinSize(size);

    Fit();
    Refresh();
}

}}    // namespace Slic3r::GUI
