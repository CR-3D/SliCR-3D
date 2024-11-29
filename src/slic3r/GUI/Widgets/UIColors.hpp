#ifndef slic3r_UI_Colors_hpp_
#define slic3r_UI_Colors_hpp_

namespace Slic3r { namespace GUI { namespace Widget {
static const int clr_border_normal = 0x646464;   // 0xDBDBDB;
//static int clr_border_hovered = 0xED6B21; // 0x00AE42;
int get_clr_border_hovered();
void set_clr_border_hovered(int);
static const int clr_border_disabled = 0x646464; // 0xDBDBDB;

static const int clr_background_normal_light = 0xFFFFFF;
static const int clr_background_normal_dark = 0x2B2B2B;    // 0x434343;
//static int clr_background_focused = 0x000021;//0xED6B21;       // 0xEDFAF2;
int get_clr_background_focused();
void set_clr_background_focused(int);
static const int clr_background_disabled_dark = 0x404040;  // 0xF0F0F0;
static const int clr_background_disabled_light = 0xD9D9D9; // 0xF0F0F0;

static const int clr_foreground_normal = 0x262E30;
static const int clr_foreground_focused = 0x00AE42;
static const int clr_foreground_disabled = 0x909090;

}}}    // namespace Slic3r::GUI::Widget
#endif // !slic3r_UI_Colors_hpp_