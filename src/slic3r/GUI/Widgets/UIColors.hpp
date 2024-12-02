#ifndef slic3r_UI_Colors_hpp_
#define slic3r_UI_Colors_hpp_

namespace Slic3r { namespace GUI { namespace Widget {

static const int clr_border_normal      = 0x646464;//0xDBDBDB;
//int get_clr_border_hovered();
//void set_clr_border_hovered(int);
//static const int clr_border_hovered     = 0x1A7476;//0x00AE42;
static const int clr_border_disabled    = 0x646464;//0xDBDBDB;

static const int clr_background_normal_light    = 0x646464;
static const int clr_background_normal_dark     = 0x646464;//0x434343;
//static const int clr_background_focused         = 0x1A7476;//0xEDFAF2;
//int get_clr_background_focused();
//void set_clr_background_focused(int);
static const int clr_background_disabled_dark   = 0x646464;//0xF0F0F0;
static const int clr_background_disabled_light  = 0x646464;//0xF0F0F0;

static const int clr_foreground_normal      = 0x646464;
static const int clr_foreground_focused     = 0x646464;
static const int clr_foreground_disabled    = 0x646464;

}}}    // namespace Slic3r::GUI::Widget

#endif // !slic3r_UI_Colors_hpp_
