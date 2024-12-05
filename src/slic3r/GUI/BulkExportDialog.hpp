///|/ Copyright (c) Prusa Research 2020 - 2023 Oleksandra Iushchenko @YuSanka, David Kocík @kocikdav, Lukáš Matěna @lukasmatena
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#pragma once

//#include <wx/gdicmn.h>

#include <boost/filesystem/path.hpp>
#include "wxExtensions.hpp"
#include "GUI_Utils.hpp"

class wxString;
class wxStaticText;
class wxTextCtrl;
class wxStaticBitmap;

namespace Slic3r {
    
class Print;
struct GCodeProcessorResult;

namespace GUI {

struct PrintToExport {
    std::reference_wrapper<Slic3r::Print> print;
    std::reference_wrapper<Slic3r::GCodeProcessorResult> processor_result;
    boost::filesystem::path output_path;
};

class BulkExportDialog : public DPIDialog
{
public:
    struct Item
    {
        enum class ValidationType
        {
            Valid,
            NoValid,
            Warning
        };

        // Item as a separate control(f.e. as a part of ConfigWizard to check name of the new custom priter)
        Item(wxWindow* parent, wxBoxSizer* sizer, PrintToExport& path);

        void            update_valid_bmp();
        bool            is_valid()      const { return m_valid_type != ValidationType::NoValid; }

    private:
        std::string		    m_path;
        PrintToExport*      m_print_to_export   { nullptr };

        ValidationType      m_valid_type    {ValidationType::NoValid};
        wxWindow*           m_parent        {nullptr};
        wxStaticBitmap*     m_valid_bmp     {nullptr};
        wxTextCtrl*         m_text_ctrl     {nullptr};
        wxStaticText*       m_valid_label   {nullptr};


        void        init_input_name_ctrl(wxBoxSizer *input_name_sizer, std::string path);
        void        update();
    };

private:
    std::vector<Item*>   m_items;

    std::vector<PrintToExport>* m_exports   {nullptr};

    wxBoxSizer*         m_sizer             {nullptr};
    wxStaticText*       m_label             {nullptr};

    std::string         m_ph_printer_name;
    std::string         m_old_preset_name;
    wxString            m_info_line_extention{wxEmptyString};

public:

    BulkExportDialog(wxWindow* parent, std::vector<PrintToExport>& exports);
    ~BulkExportDialog() override;
    bool Layout() override;

    void AddItem(PrintToExport& pte);

protected:
    void on_dpi_changed(const wxRect& suggested_rect) override;
    void on_sys_color_changed() override {}

private:
    bool enable_ok_btn() const;
};

}

}
