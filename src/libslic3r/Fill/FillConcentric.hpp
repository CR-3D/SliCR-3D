///|/ Copyright (c) Prusa Research 2016 - 2023 Vojtěch Bubník @bubnikv, Lukáš Hejl @hejllukas
///|/ Copyright (c) Slic3r 2016 Alessandro Ranellucci @alranel
///|/
///|/ ported from lib/Slic3r/Fill/Concentric.pm:
///|/ Copyright (c) Prusa Research 2016 Vojtěch Bubník @bubnikv
///|/ Copyright (c) Slic3r 2011 - 2015 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2012 Mark Hindess
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_FillConcentric_hpp_
#define slic3r_FillConcentric_hpp_

#include "FillBase.hpp"

namespace Slic3r {

class FillConcentric : public Fill
{
public:
    FillConcentric() : Fill() { can_fill_surface_single = true; }
    ~FillConcentric() override = default;

protected:
    Fill* clone() const override { return new FillConcentric(*this); };
    void init_spacing(coordf_t spacing, const FillParams &params) override;
    void _fill_surface_single(
        const FillParams                &params,
        unsigned int                     thickness_layers,
        const std::pair<float, Point>   &direction,
        ExPolygon                        expolygon,
        Polylines                       &polylines_out) const override;

    void _fill_surface_single(const FillParams              &params,
                              unsigned int                   thickness_layers,
                              const std::pair<float, Point> &direction,
                              ExPolygon                      expolygon,
                              ThickPolylines                &thick_polylines_out) const override;

    bool no_sort() const override { return true; }
};


class FillConcentricWGapFill : public Fill {
public:
    virtual ~FillConcentricWGapFill() {}

protected:
    Fill* clone() const override { return new FillConcentricWGapFill(*this); };
    void fill_surface_extrusion(const Surface *surface, const FillParams &params, ExtrusionEntitiesPtr &out) const override;

    bool no_sort() const override { return true; }
};

} // namespace Slic3r

#endif // slic3r_FillConcentric_hpp_
