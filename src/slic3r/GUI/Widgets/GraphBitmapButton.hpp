#ifndef slic3r_GUI_GraphButton_hpp_
#define slic3r_GUI_GraphButton_hpp_

#include "../wxExtensions.hpp"
#include <wx/bmpbuttn.h>

namespace Slic3r {
    struct GraphData;
    struct GraphSettings;
} // namespace Slic3r

class GraphBitmapButton : public wxBitmapButton
{
public:
    GraphBitmapButton(wxWindow* parent = NULL, const wxSize &size=wxDefaultSize/*, const wxString& name = wxEmptyString*/);

public:
    void Update() override;
    bool Enable(bool enable = true) override;

    void Rescale();

    void update_bitmap(const Slic3r::GraphSettings &graph_settings, const Slic3r::GraphData &data_storage);

protected:
#ifdef __WXMSW__
    virtual State GetNormalState() const wxOVERRIDE;
#endif
    
#ifdef __WXOSX__
    virtual wxBitmap DoGetBitmap(State which) const wxOVERRIDE;
    
    void update_state(wxEvent & evt);
    
    bool m_disable = false;
    bool m_hover = false;
    bool m_focus = false;
#endif
    void update_size();
    void draw_bitmap(wxDC &mem_dc,
                     wxColour Graph_color,
                     wxSize image_size,
                     const Slic3r::GraphSettings &graph_settings,
                     const Slic3r::GraphData &data_storage);

private:
    void update();

    wxBitmap  m_image;
    wxBitmap  m_image_disabled;
    wxBitmap  m_image_focused;
};

#endif // !slic3r_GUI_GraphButton_hpp_
