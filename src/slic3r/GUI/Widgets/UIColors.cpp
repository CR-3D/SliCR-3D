#include "UIColors.hpp"

namespace Slic3r { namespace GUI { namespace Widget {
static int clr_border_hovered = 0x000000; //0xED6B21; // 0x00AE42;
int clr_background_focused = 0x000000; //0xED6B21;       // 0xEDFAF2;


int get_clr_border_hovered() { 
    assert(clr_border_hovered != 0x000000);
    return clr_border_hovered;
}
void set_clr_border_hovered(int new_color) { clr_border_hovered = new_color; }
int get_clr_background_focused() { 
    assert(clr_background_focused != 0x000000);
    return clr_background_focused;
}
void set_clr_background_focused(int new_color) { clr_background_focused = new_color; }

}}}    // namespace Slic3r::GUI::Widget