///|/ Copyright (c) 2023 Robert Schiele @schiele
///|/ Copyright (c) Prusa Research 2023 Vojtěch Bubník @bubnikv
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_ExtrusionRole_hpp_
#define slic3r_ExtrusionRole_hpp_

#include "enum_bitmask.hpp"

#include <string>
#include <string_view>
#include <cstdint>

namespace Slic3r {

enum class ExtrusionRoleModifier : uint16_t {
    // 1) Extrusion types
    // Perimeter (external, inner, ...)
    Perimeter,
    // Infill (top / bottom / solid inner / sparse inner / bridging inner ...)
    Infill,
    // Support material extrusion
    Support,
    Skirt, //(brim if not external)
    Wipe,
    Mill,
    // 2) Extrusion modifiers
    External,
    Solid,
    Ironing,
    Bridge,
    // Variable width extrusion (also gapfill or thinwall if external)
    Thin,
    // 3) Special types
    // Indicator that the extrusion role was mixed from multiple differing extrusion roles,
    // for example from Support and SupportInterface.
    Mixed,
    //Travel
    Travel,
    // Stopper, there should be maximum 16 modifiers defined for uint16_t bit mask.
    Count
};
// There should be maximum 16 modifiers defined for uint16_t bit mask.
static_assert(int(ExtrusionRoleModifier::Count) <= 16,
              "ExtrusionRoleModifier: there must be maximum 16 modifiers defined to fit a 16 bit bitmask");

using ExtrusionRoleModifiers = enum_bitmask<ExtrusionRoleModifier>;
ENABLE_ENUM_BITMASK_OPERATORS(ExtrusionRoleModifier);

struct ExtrusionRole : public ExtrusionRoleModifiers
{
    constexpr ExtrusionRole(const ExtrusionRoleModifier bit) : ExtrusionRoleModifiers(bit) {}
    constexpr ExtrusionRole(const ExtrusionRoleModifiers bits) : ExtrusionRoleModifiers(bits) {}

    static constexpr const ExtrusionRoleModifiers None{};
    // Internal perimeter, not bridging.
    static constexpr const ExtrusionRoleModifiers Perimeter{ExtrusionRoleModifier::Perimeter};
    // External perimeter, not bridging.
    static constexpr const ExtrusionRoleModifiers ExternalPerimeter{ExtrusionRoleModifier::Perimeter | ExtrusionRoleModifier::External};
    // Perimeter, bridging. To be or'ed with ExtrusionRoleModifier::External for external bridging perimeter.
    static constexpr const ExtrusionRoleModifiers OverhangPerimeter{ExtrusionRoleModifier::Perimeter | ExtrusionRoleModifier::Bridge};
    // Sparse internal infill.
    static constexpr const ExtrusionRoleModifiers InternalInfill{ExtrusionRoleModifier::Infill};
    // Solid internal infill.
    static constexpr const ExtrusionRoleModifiers SolidInfill{ExtrusionRoleModifier::Infill | ExtrusionRoleModifier::Solid};
    // Top solid infill (visible).
    // FIXME why there is no bottom solid infill type?
    static constexpr const ExtrusionRoleModifiers TopSolidInfill{ExtrusionRoleModifier::Infill | ExtrusionRoleModifier::Solid |
                                                                 ExtrusionRoleModifier::External};
    // Ironing infill at the top surfaces.
    static constexpr const ExtrusionRoleModifiers Ironing{ExtrusionRoleModifier::Infill | ExtrusionRoleModifier::Solid |
                                                          ExtrusionRoleModifier::Ironing | ExtrusionRoleModifier::External};
    // Visible bridging infill at the bottom of an object.
    static constexpr const ExtrusionRoleModifiers BridgeInfill{ExtrusionRoleModifier::Infill | ExtrusionRoleModifier::Solid |
                                                               ExtrusionRoleModifier::Bridge | ExtrusionRoleModifier::External};
    static constexpr const ExtrusionRoleModifiers InternalBridgeInfill{ExtrusionRoleModifier::Infill | ExtrusionRoleModifier::Solid |
                                                                       ExtrusionRoleModifier::Bridge};
    // Gap fill extrusion, currently used for any variable width extrusion: Thin walls outside of the outer extrusion,
    // gap fill in between perimeters, gap fill between the inner perimeter and infill.
    static constexpr const ExtrusionRoleModifiers GapFill{ExtrusionRoleModifier::Thin};
    static constexpr const ExtrusionRoleModifiers ThinWall{ExtrusionRoleModifier::Thin | ExtrusionRoleModifier::External};
    static constexpr const ExtrusionRoleModifiers Skirt{ExtrusionRoleModifier::Skirt};
    // Support base material, printed with non-soluble plastic.
    static constexpr const ExtrusionRoleModifiers SupportMaterial{ExtrusionRoleModifier::Support};
    // Support interface material, printed with soluble plastic.
    static constexpr const ExtrusionRoleModifiers SupportMaterialInterface{ExtrusionRoleModifier::Support | ExtrusionRoleModifier::External};
    // Wipe tower material.
    static constexpr const ExtrusionRoleModifiers WipeTower{ExtrusionRoleModifier::Wipe};
    // Milling
    static constexpr const ExtrusionRoleModifiers Milling{ExtrusionRoleModifier::Mill};
    // Extrusion role for a collection with multiple extrusion roles.
    static constexpr const ExtrusionRoleModifiers Mixed{ExtrusionRoleModifier::Mixed};
    // Travel
    static constexpr const ExtrusionRoleModifiers Travel{ExtrusionRoleModifier::Travel};

    bool is_perimeter() const { return this->ExtrusionRoleModifiers::has(ExtrusionRoleModifier::Perimeter); }
    bool is_external_perimeter() const { return this->is_perimeter() && this->is_external(); }
    bool is_infill() const { return this->ExtrusionRoleModifiers::has(ExtrusionRoleModifier::Infill); }
    bool is_solid_infill() const { return this->is_infill() && this->ExtrusionRoleModifiers::has(ExtrusionRoleModifier::Solid); }
    bool is_sparse_infill() const { return this->is_infill() && !this->ExtrusionRoleModifiers::has(ExtrusionRoleModifier::Solid); }
    bool is_external() const { return this->ExtrusionRoleModifiers::has(ExtrusionRoleModifier::External); }
    bool is_bridge() const { return this->ExtrusionRoleModifiers::has(ExtrusionRoleModifier::Bridge); }

    bool is_support() const { return this->ExtrusionRoleModifiers::has(ExtrusionRoleModifier::Support); }
    bool is_support_base() const { return this->is_support() && !this->is_external(); }
    bool is_support_interface() const { return this->is_support() && this->is_external(); }
    bool is_mixed() const { return this->ExtrusionRoleModifiers::has(ExtrusionRoleModifier::Mixed); }

    // Brim is currently marked as skirt.
    bool is_skirt() const { return this->ExtrusionRoleModifiers::has(ExtrusionRoleModifier::Skirt); }
};

// Special flags describing loop
enum ExtrusionLoopRole : uint16_t {
    // useless
    elrDefault = 1 << 0, // 1
    // doesn't contains more contour: it's the most internal one
    elrInternal = 1 << 1, // 2
    elrSkirt    = 1 << 2, // 4
    // it's a modifier that indicate that the loop is around a hole, not around the infill
    elrHole = 1 << 3, // 8
    // it's a modifier that indicate that the loop should be printed as vase
    elrVase = 1 << 4, // 16
    // it's a modifier that indicate that the loop does not contains an inner loop, used for random seam
    elrFirstLoop = 1 << 5, // 32
};

// Be careful when editing this list, you also have to add values to other lists like
// GCodeViewer::Extrusion_Role_Colors ; for that, search occurences of GCodeExtrusionRole::Custom
enum class GCodeExtrusionRole : uint8_t {
    None = 0,
    Perimeter,
    ExternalPerimeter,
    OverhangPerimeter,
    InternalInfill,
    InternalBridgeInfill,
    SolidInfill,
    TopSolidInfill,
    Ironing,
    BridgeInfill,
    ThinWall,
    GapFill,
    Skirt,
    SupportMaterial,
    SupportMaterialInterface,
    WipeTower,
    Milling,
    // Custom (user defined) G-code block, for example start / end G-code.
    Custom,
    // for post-processing
    Travel,
    // Stopper to count number of enums.
    Count
};

// Convert a rich bitmask based ExtrusionRole to a less expressive ordinal GCodeExtrusionRole.
// GCodeExtrusionRole is to be serialized into G-code and deserialized by G-code viewer,
GCodeExtrusionRole extrusion_role_to_gcode_extrusion_role(ExtrusionRole role);

std::string        gcode_extrusion_role_to_string(GCodeExtrusionRole role);
GCodeExtrusionRole string_to_gcode_extrusion_role(const std::string_view role);

//for debug output
std::string role_to_code(ExtrusionRole role);
std::string looprole_to_code(ExtrusionLoopRole role);

inline std::string er_to_string(ExtrusionRole role) { return gcode_extrusion_role_to_string(extrusion_role_to_gcode_extrusion_role(role)); }

} // namespace Slic3r

//for unordered_set/map
namespace std {
template<> struct hash<Slic3r::ExtrusionRole>
{
    typedef Slic3r::ExtrusionRole argument_type;
    typedef uint16_t              result_type;

    result_type operator()(const argument_type &t) const {
        return *t;
    }
};
} // namespace std

#endif // slic3r_ExtrusionRole_hpp_
