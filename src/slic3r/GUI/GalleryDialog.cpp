///|/ Copyright (c) Prusa Research 2021 - 2023 Oleksandra Iushchenko @YuSanka, Enrico Turri @enricoturri1966, Filip Sykala @Jony01, Vojtěch Bubník @bubnikv, Lukáš Matěna @lukasmatena
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "GalleryDialog.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>
#include <string>

#include <boost/algorithm/string.hpp>
#include <boost/log/trivial.hpp>
#include <boost/filesystem.hpp>

#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/button.h>
#include <wx/statbox.h>
#include <wx/wupdlock.h>
#include <wx/notebook.h>
#include <wx/listctrl.h>
#include <wx/dirdlg.h>

#include "GUI.hpp"
#include "GUI_App.hpp"
#include "format.hpp"
#include "wxExtensions.hpp"
#include "I18N.hpp"
#include "Notebook.hpp"
#include "3DScene.hpp"
#include "GLCanvas3D.hpp"
#include "Plater.hpp"
#include "MsgDialog.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/AppConfig.hpp"
#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/GCode/ThumbnailData.hpp"
#include "libslic3r/Format/OBJ.hpp"
#include "libslic3r/MultipleBeds.hpp"
#include "../Utils/MacDarkMode.hpp"

namespace Slic3r {
namespace GUI {

#define BORDER_W    10
#define IMG_PX_CNT  64

namespace fs = boost::filesystem;

// Gallery::DropTarget
class GalleryDropTarget : public wxFileDropTarget
{
public:
    GalleryDropTarget(GalleryDialog* gallery_dlg) : gallery_dlg(gallery_dlg) { this->SetDefaultAction(wxDragCopy); }

    bool OnDropFiles(wxCoord x, wxCoord y, const wxArrayString& filenames) override;

private:
    GalleryDialog* gallery_dlg {nullptr};
};

bool GalleryDropTarget::OnDropFiles(wxCoord x, wxCoord y, const wxArrayString& filenames)
{
#ifdef WIN32
    // hides the system icon
    this->MSWUpdateDragImageOnLeave();
#endif // WIN32
    return gallery_dlg ? gallery_dlg->load_files(filenames) : false;
}


GalleryDialog::GalleryDialog(wxWindow* parent) :
    DPIDialog(parent, wxID_ANY, _L("Shape Gallery"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER, "gallery")
{
#ifndef _WIN32
    SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW));
#endif
    SetFont(wxGetApp().normal_font());

    wxStaticText* label_top = new wxStaticText(this, wxID_ANY, _L("Select shape from the gallery") + ":");

    m_list_ctrl = new wxListCtrl(this, wxID_ANY, wxDefaultPosition, wxSize(50 * wxGetApp().em_unit(), 35 * wxGetApp().em_unit()),
                                wxLC_ICON | wxSIMPLE_BORDER);
    m_list_ctrl->Bind(wxEVT_LIST_ITEM_SELECTED,     &GalleryDialog::select, this);
    m_list_ctrl->Bind(wxEVT_LIST_ITEM_DESELECTED,   &GalleryDialog::deselect, this);
    m_list_ctrl->Bind(wxEVT_LIST_KEY_DOWN,          &GalleryDialog::key_down, this);
    m_list_ctrl->Bind(wxEVT_LIST_ITEM_RIGHT_CLICK,  &GalleryDialog::show_context_menu, this);
    m_list_ctrl->Bind(wxEVT_LIST_ITEM_ACTIVATED, [this](wxListEvent& event) {
        m_selected_items.clear();
        select(event);
        this->EndModal(wxID_OK);
    });
    this->Bind(wxEVT_SIZE, [this](wxSizeEvent& event) {
        event.Skip();
        layout_items_grid();
    });

    wxStdDialogButtonSizer* buttons = this->CreateStdDialogButtonSizer(wxOK | wxCLOSE);
    wxGetApp().SetWindowVariantForButton(buttons->GetCancelButton());
    m_ok_btn = buttons->GetAffirmativeButton();
    wxGetApp().SetWindowVariantForButton(m_ok_btn);
    m_ok_btn->Bind(wxEVT_UPDATE_UI, [this](wxUpdateUIEvent& evt) { evt.Enable(!m_selected_items.empty()); });

    buttons->GetCancelButton()->Bind(wxEVT_BUTTON, [this](wxCommandEvent&){ this->EndModal(wxID_CLOSE); });
    this->SetEscapeId(wxID_CLOSE);
    auto add_btn = [this, buttons]( size_t pos, int& ID, wxString title, wxString tooltip,
                                    void (GalleryDialog::* method)(wxEvent&), 
                                    std::function<bool()> enable_fn = []() {return true; }) {
        ID = NewControlId();
        wxButton* btn = new wxButton(this, ID, title);
        btn->SetToolTip(tooltip);
        wxGetApp().SetWindowVariantForButton(btn);
        btn->Bind(wxEVT_UPDATE_UI, [enable_fn](wxUpdateUIEvent& evt) { evt.Enable(enable_fn()); });
        buttons->Insert(pos, btn, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, BORDER_W);
        this->Bind(wxEVT_BUTTON, method, this, ID);
    };

    size_t btn_pos = 0;
    add_btn(btn_pos++, ID_BTN_ADD_CUSTOM_SHAPE,   _L("Add"),                _L("Add one or more custom shapes"),                                                &GalleryDialog::add_custom_shapes);
    add_btn(btn_pos++, ID_BTN_ADD_CUSTOM_FOLDER,  _L("Add Folder"),         _L("Add all supported files from a folder"),                                        &GalleryDialog::add_custom_folder);
    add_btn(btn_pos++, ID_BTN_DEL_CUSTOM_SHAPE,   _L("Delete"),             _L("Delete one or more custom shape. You can't delete system shapes"),              &GalleryDialog::del_custom_shapes,  [this](){ return can_delete();           });
    //add_btn(btn_pos++, ID_BTN_REPLACE_CUSTOM_PNG, _L("Change thumbnail"),   _L("Replace PNG for custom shape. You can't raplace thimbnail for system shape"),   &GalleryDialog::change_thumbnail, [this](){ return can_change_thumbnail(); });
    buttons->InsertStretchSpacer(btn_pos, 2* BORDER_W);

    load_label_icon_list();

    wxBoxSizer* topSizer = new wxBoxSizer(wxVERTICAL);

    topSizer->Add(label_top , 0, wxEXPAND | wxLEFT | wxTOP | wxRIGHT, BORDER_W);
    topSizer->Add(m_list_ctrl, 1, wxEXPAND | wxLEFT | wxTOP | wxRIGHT, BORDER_W);
    topSizer->Add(buttons   , 0, wxEXPAND | wxALL, BORDER_W);

    SetSizer(topSizer);
    topSizer->SetSizeHints(this);

    wxGetApp().UpdateDlgDarkUI(this);
    this->CenterOnScreen();

    this->SetDropTarget(new GalleryDropTarget(this));
}

GalleryDialog::~GalleryDialog()
{
    // From wxWidgets docs:
    // The method void wxListCtrl::SetImageList(wxImageList* imageList, int which)
    // does not take ownership of the image list, you have to delete it yourself.
    if (m_image_list)
        delete m_image_list;
}

int GalleryDialog::show(bool show_from_menu) 
{
    m_ok_btn->SetLabel(  show_from_menu ? _L("Add to bed")                       : _L("OK"));
    m_ok_btn->SetToolTip(show_from_menu ? _L("Add selected shape(s) to the bed") : "");
    // Ensure final control sizes are known before placing grid items.
    this->Layout();
    layout_items_grid();

    return this->ShowModal();
}

bool GalleryDialog::can_delete() 
{
    if (m_selected_items.empty())
        return false;
    for (const Item& item : m_selected_items)
        if (item.is_system)
            return false;
    return true;
}

bool GalleryDialog::can_change_thumbnail() 
{
    return (m_selected_items.size() == 1 && !m_selected_items[0].is_system);
}

void GalleryDialog::on_dpi_changed(const wxRect& suggested_rect)
{
    update();

    const int& em = em_unit();
    msw_buttons_rescale(this, em, { ID_BTN_ADD_CUSTOM_SHAPE, ID_BTN_ADD_CUSTOM_FOLDER, ID_BTN_DEL_CUSTOM_SHAPE, ID_BTN_REPLACE_CUSTOM_PNG, wxID_OK, wxID_CLOSE });

    wxSize size = wxSize(50 * em, 35 * em);
    m_list_ctrl->SetMinSize(size);
    m_list_ctrl->SetSize(size);
    layout_items_grid();

    Fit();
    Refresh();
}

void GalleryDialog::layout_items_grid()
{
    if (m_list_ctrl == nullptr)
        return;

    const int item_count = m_list_ctrl->GetItemCount();
    if (item_count <= 0)
        return;

    constexpr int columns = 3;
    const int em = std::max(1, em_unit());
    const int client_w = std::max(1, m_list_ctrl->GetClientSize().GetWidth());
    const int cell_w = std::max(1, (client_w - 2 * em) / columns);
    const int icon_h = m_image_list ? m_image_list->GetSize().GetHeight() : IMG_PX_CNT;
    const int row_h = std::max(icon_h + 3 * em, 9 * em);

    m_list_ctrl->Freeze();
    for (int i = 0; i < item_count; ++i) {
        const int col = i % columns;
        const int row = i / columns;
        m_list_ctrl->SetItemPosition(i, wxPoint(em + col * cell_w, em + row * row_h));
    }
    m_list_ctrl->Thaw();
}

static wxString compact_gallery_label(const std::string &label_utf8)
{
    wxString label = from_u8(label_utf8);
    static constexpr int max_chars = 22;
    if (int(label.length()) <= max_chars)
        return label;
    // Keep start and end for recognizability (filenames / suffixes).
    const int left = 11;
    const int right = max_chars - left - 3;
    return label.Left(left) + "..." + label.Right(std::max(1, right));
}

static void add_lock(wxImage& image, wxWindow* parent_win) 
{
    wxBitmapBundle* bmp_bndl = get_bmp_bundle("lock", 22);
#ifdef __APPLE__
    wxBitmap bmp = bmp_bndl->GetBitmap(bmp_bndl->GetDefaultSize() * mac_max_scaling_factor());
#else
    wxBitmap bmp = bmp_bndl->GetBitmapFor(parent_win);
#endif

    wxImage lock_image = bmp.ConvertToImage();
    if (!lock_image.IsOk() || lock_image.GetWidth() == 0 || lock_image.GetHeight() == 0)
        return;

    auto lock_px_data = (uint8_t*)lock_image.GetData();
    auto lock_a_data = (uint8_t*)lock_image.GetAlpha();
    int lock_width  = lock_image.GetWidth();
    int lock_height = lock_image.GetHeight();
    
    auto px_data = (uint8_t*)image.GetData();
    auto a_data = (uint8_t*)image.GetAlpha();

    int width = image.GetWidth();
    int height = image.GetHeight();

    size_t beg_x = width - lock_width;
    size_t beg_y = height - lock_height;
    for (size_t x = 0; x < (size_t)lock_width; ++x) {
        for (size_t y = 0; y < (size_t)lock_height; ++y) {
            const size_t lock_idx = (x + y * lock_width);
            if (lock_a_data && lock_a_data[lock_idx] == 0)
                continue;

            const size_t idx = (beg_x + x + (beg_y + y) * width);
            if (a_data)
                a_data[idx] = lock_a_data[lock_idx];

            const size_t idx_rgb = (beg_x + x + (beg_y + y) * width) * 3;
            const size_t lock_idx_rgb = (x + y * lock_width) * 3;
            px_data[idx_rgb] = lock_px_data[lock_idx_rgb];
            px_data[idx_rgb + 1] = lock_px_data[lock_idx_rgb + 1];
            px_data[idx_rgb + 2] = lock_px_data[lock_idx_rgb + 2];
        }
    }
}

static void add_default_image(wxImageList* img_list, bool is_system, wxWindow* parent_win)
{
    wxBitmapBundle* bmp_bndl = get_bmp_bundle("cog", IMG_PX_CNT);
#ifdef __APPLE__
    wxBitmap bmp = bmp_bndl->GetBitmap(bmp_bndl->GetDefaultSize() * mac_max_scaling_factor());
#else
    wxBitmap bmp = bmp_bndl->GetBitmapFor(parent_win);
#endif

    bmp = bmp.ConvertToDisabled();
    if (is_system) {
        wxImage image = bmp.ConvertToImage();
        if (image.IsOk() && image.GetWidth() != 0 && image.GetHeight() != 0) {
            add_lock(image, parent_win);
#ifdef __APPLE__
            bmp = wxBitmap(std::move(image), -1, mac_max_scaling_factor());
#else
            bmp = wxBitmap(std::move(image));
#endif
        }
    }

    img_list->Add(bmp);
};

static fs::path get_dir(bool sys_dir)
{
    return fs::absolute(fs::path(sys_dir ? sys_shapes_dir() : custom_shapes_dir())).make_preferred();
}

static void generate_thumbnail_from_model(const std::string& filename)
{
    if (!boost::algorithm::iends_with(filename, ".stl") &&
        !boost::algorithm::iends_with(filename, ".obj")) {
        BOOST_LOG_TRIVIAL(error) << "Found invalid file type in generate_thumbnail_from_model() [" << filename << "]";
        return;
    }

    Model model;
    try {
        model = Model::read_from_file(filename);
    }
    catch (std::exception&) {
        BOOST_LOG_TRIVIAL(error) << "Error loading model from " << filename << " in generate_thumbnail_from_model()";
        return;
    }

    assert(model.objects.size() == 1);
    assert(model.objects[0]->volumes.size() == 1);
    assert(model.objects[0]->instances.size() == 1);

    model.objects[0]->center_around_origin(false);
    model.objects[0]->ensure_on_bed(false);

    model.center_instances_around_point(to_2d(wxGetApp().plater()->build_volume().bounding_volume().center()));

    GLVolumeCollection volumes;
    GLVolume* volume = volumes.volumes.emplace_back(new GLVolume()).get();
    volume->model.init_from(model.mesh());
    volume->set_instance_transformation(model.objects[0]->instances[0]->get_transformation());
    volume->set_volume_transformation(model.objects[0]->volumes[0]->get_transformation());

    ThumbnailData thumbnail_data;
    const ThumbnailsParams thumbnail_params = { {}, false, false, false, true };
    s_multiple_beds.set_thumbnail_bed_idx(-2);
    wxGetApp().plater()->canvas3D()->render_thumbnail(thumbnail_data, 256, 256, thumbnail_params, volumes, Camera::EType::Perspective);
    s_multiple_beds.set_thumbnail_bed_idx(-1);

    if (thumbnail_data.width == 0 || thumbnail_data.height == 0)
        return;

    wxImage image(thumbnail_data.width, thumbnail_data.height);
    image.InitAlpha();

    for (unsigned int r = 0; r < thumbnail_data.height; ++r) {
        unsigned int rr = (thumbnail_data.height - 1 - r) * thumbnail_data.width;
        for (unsigned int c = 0; c < thumbnail_data.width; ++c) {
            unsigned char* px = (unsigned char*)thumbnail_data.pixels.data() + 4 * (rr + c);
            image.SetRGB((int)c, (int)r, px[0], px[1], px[2]);
            image.SetAlpha((int)c, (int)r, px[3]);
        }
    }

    fs::path out_path = fs::path(filename);
    out_path.replace_extension("png");
    image.SaveFile(from_u8(out_path.string()), wxBITMAP_TYPE_PNG);
}

void GalleryDialog::load_label_icon_list()
{
    m_items.clear();

    // load names from files
    auto add_files_from_gallery = [](std::vector<Item> &items, bool is_sys_dir, fs::path &dir_path)
    {
        fs::path dir = get_dir(is_sys_dir);
        if (!fs::exists(dir))
            return;

        dir_path = dir;

        std::vector<Item> sorted_items;
        for (auto &dir_entry : fs::recursive_directory_iterator(dir)) {
            if (!fs::is_regular_file(dir_entry.path()))
                continue;
            TriangleMesh mesh;
            if ((is_gallery_file(dir_entry, ".stl") && mesh.ReadSTLFile(dir_entry.path().string().c_str())) || 
                (is_gallery_file(dir_entry, ".obj") && load_obj(dir_entry.path().string().c_str(), &mesh) )    ) {
                const fs::path rel = fs::relative(dir_entry.path(), dir);
                const fs::path parent = rel.parent_path();
                const std::string folder = parent.empty() || parent == "." ? std::string{} : parent.generic_string();
                const std::string name = rel.filename().string();
                const std::string rel_path = rel.generic_string();
                const std::string display_name = folder.empty() ? name : folder + " / " + name;
                sorted_items.push_back(Item{ name, rel_path, folder, display_name, is_sys_dir });
            }
        }

        // sort folder + filename case insensitive
        std::sort(sorted_items.begin(), sorted_items.end(), [](const Item &a, const Item &b) {
            const std::string af = boost::algorithm::to_lower_copy(a.folder);
            const std::string bf = boost::algorithm::to_lower_copy(b.folder);
            if (af != bf)
                return af < bf;
            return boost::algorithm::to_lower_copy(a.name) < boost::algorithm::to_lower_copy(b.name);
        });

        items.insert(items.end(), sorted_items.begin(), sorted_items.end());
    };

    wxBusyCursor busy;

    fs::path m_sys_dir_path, m_cust_dir_path;
    add_files_from_gallery(m_items, true, m_sys_dir_path);
    add_files_from_gallery(m_items, false, m_cust_dir_path);

    // Make an image list containing large icons

#ifdef __APPLE__
    m_image_list = new wxImageList(IMG_PX_CNT, IMG_PX_CNT);
    int px_cnt = IMG_PX_CNT * mac_max_scaling_factor();
#else
    int px_cnt = (int)(em_unit() * IMG_PX_CNT * 0.1f + 0.5f);
    m_image_list = new wxImageList(px_cnt, px_cnt);
#endif

    for (const auto& item : m_items) {
        const fs::path model_path = (item.is_system ? m_sys_dir_path : m_cust_dir_path) / fs::path(item.relative_path);
        std::string model_name = model_path.string();
        fs::path png_path = model_path;
        png_path.replace_extension("png");
        std::string img_name = png_path.string();

#if 0 // use "1" just in DEBUG mode to the generation of the thumbnails for the sistem shapes
        bool can_generate_thumbnail = true;
#else
        bool can_generate_thumbnail = !item.is_system;
#endif //DEBUG
        if (!fs::exists(img_name)) {
            if (can_generate_thumbnail)
                generate_thumbnail_from_model(model_name);
            else {
                add_default_image(m_image_list, item.is_system, this);
                continue;
            }
        }

        wxImage image;
        BOOST_LOG_TRIVIAL(debug) << "Trying to load (load_label_icon_list, png) image: '"<<img_name<<"'";
        if (!image.CanRead(from_u8(img_name)) ||
            !image.LoadFile(from_u8(img_name), wxBITMAP_TYPE_PNG) ||
            image.GetWidth() == 0 || image.GetHeight() == 0) {
            add_default_image(m_image_list, item.is_system, this);
            continue;
        }
        image.Rescale(px_cnt, px_cnt, wxIMAGE_QUALITY_BILINEAR);

        if (item.is_system)
            add_lock(image, this);
#ifdef __APPLE__
        wxBitmap bmp = wxBitmap(std::move(image), -1, mac_max_scaling_factor());
#else
        wxBitmap bmp = wxBitmap(std::move(image));
#endif
        m_image_list->Add(bmp);
    }

    m_list_ctrl->SetImageList(m_image_list, wxIMAGE_LIST_NORMAL);

    int img_cnt = m_image_list->GetImageCount();
    for (int i = 0; i < img_cnt; i++) {
        m_list_ctrl->InsertItem(i, compact_gallery_label(m_items[size_t(i)].display_name), i);
        m_list_ctrl->SetItemData(i, m_items[size_t(i)].is_system ? 1 : 0);
    }
    layout_items_grid();
}

void GalleryDialog::get_input_files(wxArrayString& input_files)
{
    for (const Item& item : m_selected_items)
        input_files.Add(from_u8((get_dir(item.is_system) / fs::path(item.relative_path)).string()));
}

void GalleryDialog::add_custom_shapes(wxEvent& event)
{
    wxArrayString input_files;
    wxFileDialog dialog(this, _L("Choose one or more files (STL, OBJ):"),
        from_u8(wxGetApp().app_config->get_last_dir()), "",
        file_wildcards(FT_GALLERY), wxFD_OPEN | wxFD_MULTIPLE | wxFD_FILE_MUST_EXIST);

    if (dialog.ShowModal() == wxID_OK)
        dialog.GetPaths(input_files);

    if (input_files.IsEmpty())
        return;

    load_files(input_files);
}

void GalleryDialog::add_custom_folder(wxEvent& event)
{
    wxDirDialog dialog(this, _L("Choose a folder with shapes (STL, OBJ):"),
        from_u8(wxGetApp().app_config->get_last_dir()),
        wxDD_DEFAULT_STYLE | wxDD_DIR_MUST_EXIST);

    if (dialog.ShowModal() != wxID_OK)
        return;

    wxArrayString input_paths;
    input_paths.Add(dialog.GetPath());
    load_files(input_paths);
}

void GalleryDialog::del_custom_shapes()
{
    auto custom_dir = get_dir(false);

    auto remove_file = [custom_dir](const std::string& name) {
        if (!fs::exists(custom_dir / name))
            return;
        try {
            fs::remove(custom_dir / name);
        }
        catch (fs::filesystem_error const& e) {
            std::cerr << e.what() << '\n';
        }
    };

    for (const Item& item : m_selected_items) {
        remove_file(item.relative_path);
        fs::path path = fs::path(item.relative_path);
        path.replace_extension("png");
        remove_file(path.generic_string());
    }

    update();
}

static void show_warning(const wxString& title, const std::string& error_file_type)
{
    const wxString msg_text = format_wxstr(_L("It looks like selected %1%-file has an error or is destructed.\n"
        "We can't load this file"), error_file_type);
    MessageDialog dialog(nullptr, msg_text, title, wxICON_WARNING | wxOK);
    dialog.ShowModal();
}

void GalleryDialog::change_thumbnail()
{
    if (m_selected_items.size() != 1 || m_selected_items[0].is_system)
        return;

    wxFileDialog dialog(this, _L("Choose one PNG file:"),
                        from_u8(wxGetApp().app_config->get_last_dir()), "",
                        "PNG files (*.png)|*.png;*.PNG", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (dialog.ShowModal() != wxID_OK)
        return;

    wxArrayString input_files;
    dialog.GetPaths(input_files);

    if (input_files.IsEmpty())
        return;

    if (wxImage image; !image.CanRead(input_files.Item(0))) {
        show_warning(_L("Replacing of the PNG"), "PNG");
        return;
    }

    try {
        fs::path png_path = fs::path(get_dir(false) / m_selected_items[0].relative_path);
        png_path.replace_extension("png");

        fs::path current = fs::path(into_u8(input_files.Item(0)));
		std::string error_msg;
        if (copy_file_inner(current, png_path, error_msg)) // TODO: fs::copy_options::overwrite_existing instead of fs::copy_option::overwrite_if_exists
            throw FileIOError(error_msg);
    }
    catch (fs::filesystem_error const& e) {
        std::cerr << e.what() << '\n';
        return;
    }

    update();
}

void GalleryDialog::select(wxListEvent& event)
{
    int idx = event.GetIndex();
    if (idx < 0 || size_t(idx) >= m_items.size())
        return;
    const Item &item = m_items[size_t(idx)];
    auto it = std::find_if(m_selected_items.begin(), m_selected_items.end(),
                           [&item](const Item &sel) { return sel.is_system == item.is_system && sel.relative_path == item.relative_path; });
    if (it == m_selected_items.end())
        m_selected_items.push_back(item);
}

void GalleryDialog::deselect(wxListEvent& event)
{
    if (m_list_ctrl->GetSelectedItemCount() == 0) {
        m_selected_items.clear();
        return;
    }

    const int idx = event.GetIndex();
    if (idx < 0 || size_t(idx) >= m_items.size())
        return;
    const Item &item = m_items[size_t(idx)];
    m_selected_items.erase(std::remove_if(m_selected_items.begin(), m_selected_items.end(),
                                          [&item](const Item &sel) {
                                              return sel.is_system == item.is_system && sel.relative_path == item.relative_path;
                                          }),
                           m_selected_items.end());
}

void GalleryDialog::show_context_menu(wxListEvent& event)
{
    wxMenu* menu = new wxMenu();
    if (can_delete())
        append_menu_item(menu, wxID_ANY, _L("Delete"), "", [this](wxCommandEvent&) { del_custom_shapes(); });
    if (can_change_thumbnail())
        append_menu_item(menu, wxID_ANY, _L("Change thumbnail"), "", [this](wxCommandEvent&) { change_thumbnail(); });

    this->PopupMenu(menu);
}

void GalleryDialog::key_down(wxListEvent& event)
{
    if (can_delete() && (event.GetKeyCode() == WXK_DELETE || event.GetKeyCode() == WXK_BACK))
        del_custom_shapes();
}

void GalleryDialog::update()
{
    m_selected_items.clear();
    m_image_list->RemoveAll();
    m_list_ctrl->ClearAll();
    load_label_icon_list();
}

bool GalleryDialog::load_files(const wxArrayString& input_files)
{
    struct ImportEntry {
        fs::path source_file;
        fs::path root_folder;
    };

    auto dest_dir = get_dir(false);

    try {
        if (!fs::exists(dest_dir))
            if (!fs::create_directory(dest_dir)) {
                std::cerr << "Unable to create destination directory" << dest_dir.string() << '\n' ;
                return false;
            }
    }
    catch (fs::filesystem_error const& e) {
        std::cerr << e.what() << '\n';
        return false;
    }

    std::vector<ImportEntry> files_to_import;
    files_to_import.reserve(input_files.size());

    for (size_t i = 0; i < input_files.size(); ++i) {
        fs::path input_path = fs::path(into_u8(input_files.Item(i)));
        if (!fs::exists(input_path))
            continue;

        if (fs::is_directory(input_path)) {
            for (auto& dir_entry : fs::recursive_directory_iterator(input_path)) {
                if (!fs::is_regular_file(dir_entry.path()))
                    continue;
                std::string file = dir_entry.path().string();
                if (is_gallery_file(file, ".stl") || is_gallery_file(file, ".obj"))
                    files_to_import.push_back(ImportEntry{ dir_entry.path(), input_path });
            }
        } else {
            std::string file = input_path.string();
            if (is_gallery_file(file, ".stl") || is_gallery_file(file, ".obj"))
                files_to_import.push_back(ImportEntry{ input_path, fs::path{} });
        }
    }

    if (files_to_import.empty())
        return false;

    // Iterate through the input files
    for (const ImportEntry& entry : files_to_import) {
        const std::string input_file = entry.source_file.string();

        TriangleMesh mesh; 
        if (is_gallery_file(input_file, ".stl") && !mesh.ReadSTLFile(input_file.c_str())) {
            show_warning(format_wxstr(_L("Loading of the \"%1%\""), input_file), "STL");
            continue;
        }

        if (is_gallery_file(input_file, ".obj") && !load_obj(input_file.c_str(), &mesh)) {
            show_warning(format_wxstr(_L("Loading of the \"%1%\""), input_file), "OBJ");
            continue;
        }

        try {
            fs::path rel = entry.root_folder.empty() ? entry.source_file.filename() : fs::relative(entry.source_file, entry.root_folder);
            if (rel.empty())
                rel = entry.source_file.filename();
            fs::path target = dest_dir / rel;
            fs::create_directories(target.parent_path());

            if (fs::exists(target)) {
                const fs::path parent = target.parent_path();
                const std::string stem = target.stem().string();
                const std::string ext = target.extension().string();
                int file_idx = 1;
                do {
                    target = parent / fs::path(stem + " (" + std::to_string(file_idx) + ")" + ext);
                    ++file_idx;
                } while (fs::exists(target));
            }

            std::string error_msg;
            if (copy_file_inner(entry.source_file, target, error_msg))
                throw FileIOError(error_msg);
        }
        catch (fs::filesystem_error const& e) {
            std::cerr << e.what() << '\n';
            return false;
        }
    }

    update();
    return true;
}

}}    // namespace Slic3r::GUI
