///|/ Copyright (c) Prusa Research 2021 - 2022 Vojtěch Bubník @bubnikv, Enrico Turri @enricoturri1966
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "libslic3r.h"
#include "Color.hpp"

#include <random>

static const float INV_255 = 1.0f / 255.0f;

namespace Slic3r {

// Conversion from RGB to HSV color space
// The input RGB values are in the range [0, 1]
// The output HSV values are in the ranges h = [0, 360], and s, v = [0, 1]
static void RGBtoHSV(float r, float g, float b, float& h, float& s, float& v)
{
	assert(0.0f <= r && r <= 1.0f);
	assert(0.0f <= g && g <= 1.0f);
	assert(0.0f <= b && b <= 1.0f);

	const float max_comp = std::max(std::max(r, g), b);
	const float min_comp = std::min(std::min(r, g), b);
	const float delta = max_comp - min_comp;

	if (delta > 0.0f) {
		if (max_comp == r)
			h = 60.0f * (std::fmod(((g - b) / delta), 6.0f));
		else if (max_comp == g)
			h = 60.0f * (((b - r) / delta) + 2.0f);
		else  // max_comp == b
			h = 60.0f * (((r - g) / delta) + 4.0f);

		s = (max_comp > 0.0f) ? delta / max_comp : 0.0f;
	}
	else {
		h = 0.0f;
		s = 0.0f;
	}
	v = max_comp;

	while (h < 0.0f) { h += 360.0f; }
	while (h > 360.0f) { h -= 360.0f; }

	assert(0.0f <= s && s <= 1.0f);
	assert(0.0f <= v && v <= 1.0f);
	assert(0.0f <= h && h <= 360.0f);
}

// Conversion from HSV to RGB color space
// The input HSV values are in the ranges h = [0, 360], and s, v = [0, 1]
// The output RGB values are in the range [0, 1]
static void HSVtoRGB(float h, float s, float v, float& r, float& g, float& b)
{
	assert(0.0f <= s && s <= 1.0f);
	assert(0.0f <= v && v <= 1.0f);
	assert(0.0f <= h && h <= 360.0f);

	const float chroma = v * s;
	const float h_prime = std::fmod(h / 60.0f, 6.0f);
	const float x = chroma * (1.0f - std::abs(std::fmod(h_prime, 2.0f) - 1.0f));
	const float m = v - chroma;

	if (0.0f <= h_prime && h_prime < 1.0f) {
		r = chroma;
		g = x;
		b = 0.0f;
	}
	else if (1.0f <= h_prime && h_prime < 2.0f) {
		r = x;
		g = chroma;
		b = 0.0f;
	}
	else if (2.0f <= h_prime && h_prime < 3.0f) {
		r = 0.0f;
		g = chroma;
		b = x;
	}
	else if (3.0f <= h_prime && h_prime < 4.0f) {
		r = 0.0f;
		g = x;
		b = chroma;
	}
	else if (4.0f <= h_prime && h_prime < 5.0f) {
		r = x;
		g = 0.0f;
		b = chroma;
	}
	else if (5.0f <= h_prime && h_prime < 6.0f) {
		r = chroma;
		g = 0.0f;
		b = x;
	}
	else {
		r = 0.0f;
		g = 0.0f;
		b = 0.0f;
	}

	r += m;
	g += m;
	b += m;

	assert(0.0f <= r && r <= 1.0f);
	assert(0.0f <= g && g <= 1.0f);
	assert(0.0f <= b && b <= 1.0f);
}

class Randomizer
{
	std::random_device m_rd;

public:
	float random_float(float min, float max) {
		std::mt19937 rand_generator(m_rd());
		std::uniform_real_distribution<float> distrib(min, max);
		return distrib(rand_generator);
	}
};

ColorRGB::ColorRGB(float r, float g, float b)
: m_data({ std::clamp(r, 0.0f, 1.0f), std::clamp(g, 0.0f, 1.0f), std::clamp(b, 0.0f, 1.0f) })
{
}

ColorRGB::ColorRGB(unsigned char r, unsigned char g, unsigned char b)
: m_data({ std::clamp(r * INV_255, 0.0f, 1.0f), std::clamp(g * INV_255, 0.0f, 1.0f), std::clamp(b * INV_255, 0.0f, 1.0f) })
{
}

bool ColorRGB::operator < (const ColorRGB& other) const
{
	for (size_t i = 0; i < 3; ++i) {
		if (m_data[i] < other.m_data[i])
			return true;
		else if (m_data[i] > other.m_data[i])
			return false;
	}

	return false;
}

bool ColorRGB::operator > (const ColorRGB& other) const
{
	for (size_t i = 0; i < 3; ++i) {
		if (m_data[i] > other.m_data[i])
			return true;
		else if (m_data[i] < other.m_data[i])
			return false;
	}

	return false;
}

ColorRGB ColorRGB::operator + (const ColorRGB& other) const
{
	ColorRGB ret;
	for (size_t i = 0; i < 3; ++i) {
		ret.m_data[i] = std::clamp(m_data[i] + other.m_data[i], 0.0f, 1.0f);
	}
	return ret;
}

ColorRGB ColorRGB::operator * (float value) const
{
	assert(value >= 0.0f);
	ColorRGB ret;
	for (size_t i = 0; i < 3; ++i) {
		ret.m_data[i] = std::clamp(value * m_data[i], 0.0f, 1.0f);
	}
	return ret;
}

ColorRGBA::ColorRGBA(float r, float g, float b, float a)
: m_data({ std::clamp(r, 0.0f, 1.0f), std::clamp(g, 0.0f, 1.0f), std::clamp(b, 0.0f, 1.0f), std::clamp(a, 0.0f, 1.0f) })
{
}

ColorRGBA::ColorRGBA(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
: m_data({ std::clamp(r * INV_255, 0.0f, 1.0f), std::clamp(g * INV_255, 0.0f, 1.0f), std::clamp(b * INV_255, 0.0f, 1.0f), std::clamp(a * INV_255, 0.0f, 1.0f) })
{
}

bool ColorRGBA::operator < (const ColorRGBA& other) const
{
	for (size_t i = 0; i < 3; ++i) {
		if (m_data[i] < other.m_data[i])
			return true;
		else if (m_data[i] > other.m_data[i])
			return false;
	}

	return false;
}

bool ColorRGBA::operator > (const ColorRGBA& other) const
{
	for (size_t i = 0; i < 3; ++i) {
		if (m_data[i] > other.m_data[i])
			return true;
		else if (m_data[i] < other.m_data[i])
			return false;
	}

	return false;
}

ColorRGBA ColorRGBA::operator + (const ColorRGBA& other) const
{
	ColorRGBA ret;
	for (size_t i = 0; i < 3; ++i) {
		ret.m_data[i] = std::clamp(m_data[i] + other.m_data[i], 0.0f, 1.0f);
	}
	return ret;
}

ColorRGBA ColorRGBA::operator * (float value) const
{
	assert(value >= 0.0f);
	ColorRGBA ret;
	for (size_t i = 0; i < 3; ++i) {
		ret.m_data[i] = std::clamp(value * m_data[i], 0.0f, 1.0f);
	}
	ret.m_data[3] = m_data[3];
	return ret;
}

ColorRGB operator * (float value, const ColorRGB& other) { return other * value; }
ColorRGBA operator * (float value, const ColorRGBA& other) { return other * value; }

ColorRGB lerp(const ColorRGB& a, const ColorRGB& b, float t)
{
	assert(0.0f <= t && t <= 1.0f);
	return (1.0f - t) * a + t * b;
}

ColorRGBA lerp(const ColorRGBA& a, const ColorRGBA& b, float t)
{
	assert(0.0f <= t && t <= 1.0f);
	return (1.0f - t) * a + t * b;
}

ColorRGB complementary(const ColorRGB& color)
{
	return { 1.0f - color.r(), 1.0f - color.g(), 1.0f - color.b() };
}

ColorRGBA complementary(const ColorRGBA& color)
{
	return { 1.0f - color.r(), 1.0f - color.g(), 1.0f - color.b(), color.a() };
}

ColorRGB saturate(const ColorRGB& color, float factor)
{
	float h, s, v;
	RGBtoHSV(color.r(), color.g(), color.b(), h, s, v);
	s = std::clamp(s * factor, 0.0f, 1.0f);
	float r, g, b;
	HSVtoRGB(h, s, v, r, g, b);
	return { r, g, b };
}

ColorRGBA saturate(const ColorRGBA& color, float factor)
{
	return to_rgba(saturate(to_rgb(color), factor), color.a());
}

ColorRGB opposite(const ColorRGB& color)
{
	float h, s, v;
	RGBtoHSV(color.r(), color.g(), color.b(), h, s, v);

	h += 65.0f; // 65 instead 60 to avoid circle values
	if (h > 360.0f)
		h -= 360.0f;

	Randomizer rnd;
	if (s < 0.8) {
        s = rnd.random_float(0.8f, 1.0f);
	} else if (s > 0.85) {
        s = rnd.random_float(0.65f, 0.85f);
	} else {
        s = rnd.random_float(0.65f, 1.0f);
	}
	if (v < 0.8) {
        v = rnd.random_float(0.8f, 1.0f);
	} else if (v > 0.85) {
        v = rnd.random_float(0.65f, 0.85f);
	} else {
        v = rnd.random_float(0.65f, 1.0f);
	}

	float r, g, b;
	HSVtoRGB(h, s, v, r, g, b);
	return { r, g, b };
}

ColorRGB opposite(const ColorRGB& a, const ColorRGB& b)
{
	float ha, sa, va;
	RGBtoHSV(a.r(), a.g(), a.b(), ha, sa, va);
	float hb, sb, vb;
	RGBtoHSV(b.r(), b.g(), b.b(), hb, sb, vb);

	float delta_h = std::abs(ha - hb);
	float start_h = (delta_h > 180.0f) ? std::min(ha, hb) : std::max(ha, hb);

	start_h += 5.0f; // to avoid circle change of colors for 120 deg
	if (delta_h < 180.0f)
		delta_h = 360.0f - delta_h;

	Randomizer rnd;
	float out_h = start_h + 0.5f * delta_h;
	if (out_h > 360.0f)
		out_h -= 360.0f;
	float out_s = rnd.random_float(0.65f, 1.0f);
	float out_v = rnd.random_float(0.65f, 1.0f);

	float out_r, out_g, out_b;
	HSVtoRGB(out_h, out_s, out_v, out_r, out_g, out_b);
	return { out_r, out_g, out_b };
}

bool can_decode_color(const std::string& color) { return color.size() == 7 && color.front() == '#'; }

bool decode_color(const std::string& color_in, ColorRGB& color_out)
{
	auto hex_digit_to_int = [](const char c) {
		return
			(c >= '0' && c <= '9') ? int(c - '0') :
			(c >= 'A' && c <= 'F') ? int(c - 'A') + 10 :
			(c >= 'a' && c <= 'f') ? int(c - 'a') + 10 : -1;
	};

	color_out = ColorRGB::BLACK();
	if (can_decode_color(color_in)) {
		const char* c = color_in.data() + 1;
		for (unsigned int i = 0; i < 3; ++i) {
			const int digit1 = hex_digit_to_int(*c++);
			const int digit2 = hex_digit_to_int(*c++);
			if (digit1 != -1 && digit2 != -1)
				color_out.set(i, float(digit1 * 16 + digit2) * INV_255);
		}
	}
	else
		return false;

	assert(0.0f <= color_out.r() && color_out.r() <= 1.0f);
	assert(0.0f <= color_out.g() && color_out.g() <= 1.0f);
	assert(0.0f <= color_out.b() && color_out.b() <= 1.0f);
	return true;
}

bool decode_color(const std::string& color_in, ColorRGBA& color_out)
{
	ColorRGB rgb;
	if (!decode_color(color_in, rgb))
		return false;

	color_out = to_rgba(rgb, color_out.a());
	return true;
}

bool decode_colors(const std::vector<std::string> &colors_in, std::vector<ColorRGB> &colors_out) {
    bool all_success = true;
    colors_out.resize(colors_in.size(), ColorRGB::BLACK());
    for (size_t i = 0; i < colors_in.size(); ++i) {
        if (!decode_color(colors_in[i], colors_out[i])) {
            // continue, please.
            // return false;
            all_success = false;
        }
    }
    return all_success;
}

bool decode_colors(const std::vector<std::string>& colors_in, std::vector<ColorRGBA>& colors_out)
{
    bool all_success = true;
    colors_out.resize(colors_in.size(), ColorRGBA::BLACK());
    for (size_t i = 0; i < colors_in.size(); ++i) {
        if (!decode_color(colors_in[i], colors_out[i])) {
            // continue, please.
            // return false;
            all_success = false;
        }
    }
    return all_success;
}

static const std::array<ColorRGBA, 12> COLOR_ROTATION = {{
	ColorRGBA::GREEN(),
	ColorRGBA::RED(),
	ColorRGBA::BLUE(),
	ColorRGBA::YELLOW(),
	ColorRGBA::MAGENTA(),
	ColorRGBA::CYAN(),
	ColorRGBA::ORANGE(),
	ColorRGBA::GREENISH(),
	ColorRGBA(1.0f, 0.f, 0.5f, 1.0f),//PINK(),
	ColorRGBA::LIGHT_GRAY(),
	ColorRGBA::BLUEISH(),
	ColorRGBA::REDISH(),
}};
ColorRGBA get_a_color(size_t idx){ return COLOR_ROTATION[idx % 12]; }

std::string encode_color(const ColorRGB& color)
{
	char buffer[64];
	::sprintf(buffer, "#%02X%02X%02X", color.r_uchar(), color.g_uchar(), color.b_uchar());
	return std::string(buffer);
}

std::string encode_color(const ColorRGBA& color) { return encode_color(to_rgb(color)); }
 
ColorRGB to_rgb(const ColorRGBA& other_rgba) { return { other_rgba.r(), other_rgba.g(), other_rgba.b() }; }
ColorRGBA to_rgba(const ColorRGB& other_rgb) { return { other_rgb.r(), other_rgb.g(), other_rgb.b(), 1.0f }; }
ColorRGBA to_rgba(const ColorRGB& other_rgb, float alpha) { return { other_rgb.r(), other_rgb.g(), other_rgb.b(), alpha }; }

hsv rgb2hsv(const ColorRGB& in)
{
    hsv         out;
    double      min, max, delta;

    min = in.r() < in.g() ? in.r() : in.g();
    min = min < in.b() ? min : in.b();

    max = in.r() > in.g() ? in.r() : in.g();
    max = max > in.b() ? max : in.b();

    out.v = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if (max > 0.0) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0              
        // s = 0, h is undefined
        out.s = 0.0;
        out.h = NAN;                            // its now undefined
        return out;
    }
    if (in.r() >= max)                           // > is bogus, just keeps compilor happy
        out.h = (in.g() - in.b()) / delta;        // between yellow & magenta
    else
        if (in.g() >= max)
            out.h = 2.0 + (in.b() - in.r()) / delta;  // between cyan & yellow
        else
            out.h = 4.0 + (in.r() - in.g()) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if (out.h < 0.0)
        out.h += 360.0;

    return out;
}


ColorRGB hsv2rgb(const hsv& in)
{
    double      hh, p, q, t, ff;
    long        i;
    ColorRGB    out;

    if (in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r(in.v);
        out.g(in.v);
        out.b(in.v);
        return out;
    }
    hh = in.h;
    if (hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch (i) {
    case 0:
        out.r(in.v);
        out.g(t);
        out.b(p);
        break;
    case 1:
        out.r(q);
        out.g(in.v);
        out.b(p);
        break;
    case 2:
        out.r(p);
        out.g(in.v);
        out.b(t);
        break;

    case 3:
        out.r(p);
        out.g(q);
        out.b(in.v);
        break;
    case 4:
        out.r(t);
        out.g(p);
        out.b(in.v);
        break;
    case 5:
    default:
        out.r(in.v);
        out.g(p);
        out.b(q);
        break;
    }
    return out;
}

uint32_t hex2int(const std::string& hex)
{
    uint32_t int_color;
    if (hex.empty() || !(hex.size() == 6 || hex.size() == 7)) {
        int_color = 0x2172eb;
    } else {
        std::stringstream ss;
        ss << std::hex << (hex[0] == '#' ? hex.substr(1) : hex);
        ss >> int_color;
    }
    // #RRVVBB so r in in the high bit, but we store it in the low one in an int
    uint32_t good_int_color = 0;
    good_int_color |= ((int_color & 0xFF0000) >> 16);
    good_int_color |= ((int_color & 0xFF00));
    good_int_color |= ((int_color & 0xFF) << 16);
    return good_int_color;
}

std::string int2hex(uint32_t int_color)
{
    std::stringstream ss;
    ss << std::hex << ((int_color & 0xF0) >> 4) << ((int_color & 0xF));
    ss << ((int_color & 0xF000) >> 12) << ((int_color & 0xF00) >> 8);
    ss << ((int_color & 0xF00000) >> 20) << ((int_color & 0xF0000) >> 16);
    return ss.str();
}

ColorRGB int2rgb(uint32_t int_color)
{
    return ColorRGB(
            uint8_t((int_color & 0xFF)),
            uint8_t((int_color & 0xFF00) >> 8),
            uint8_t((int_color & 0xFF0000) >> 16));
}
uint32_t rgb2int(const ColorRGB& rgb_color)
{
    uint32_t int_color = 0;
    int_color |= std::min(255, int(rgb_color.r() * 255));
    int_color |= std::min(255, int(rgb_color.g() * 255)) << 8;
    int_color |= std::min(255, int(rgb_color.b() * 255)) << 16;
    return int_color;
}
uint32_t change_endian_int24(uint32_t int_color)
{
    uint32_t out_int = 0;
    out_int |= (int_color & 0xFF) << 16;
    out_int |= ((int_color & 0xFF00) >> 8) << 8;
    out_int |= ((int_color & 0xFF0000) >> 16);
    return out_int;
}

void ColorReplaces::add(const std::string &sold, const std::string &snew) {
    changes.push_back(ColorReplace{sold, {}, snew, {}});
	changes.back().is_valid = decode_color(sold, changes.back().color_to_replace);
	changes.back().is_valid &= decode_color(snew, changes.back().new_color) ;
}
void ColorReplaces::add(const ColorRGB& cold, const ColorRGB& cnew) {
    changes.push_back(ColorReplace{encode_color(cold), cold, encode_color(cnew), cnew});
}
void ColorReplaces::add(const uint32_t& iold, const uint32_t&inew) {
	ColorRGB cold = int2rgb(iold);
	ColorRGB cnew = int2rgb(inew);
    changes.push_back(ColorReplace{encode_color(cold), cold, encode_color(cnew), cnew});
}
void ColorReplaces::add(const std::string& sold, const uint32_t&inew) {
	ColorRGB cnew = int2rgb(inew);
    changes.push_back(ColorReplace{sold, {}, encode_color(cnew), cnew});
	changes.back().is_valid = decode_color(sold, changes.back().color_to_replace);
}
std::optional<ColorReplace> ColorReplaces::has_key(const ColorRGB &to_replace) const
{
    for (const ColorReplace &change : changes) {
        if (change.color_to_replace == to_replace)
            return {change};
    }
    return {};
}
		
std::optional<ColorReplace> ColorReplaces::has_value(const ColorRGB & new_col) const
{
    for (const ColorReplace &change : changes) {
        if (change.new_color == new_col)
            return {change};
    }
    return {};
}
std::optional<ColorReplace> ColorReplaces::has_key(const std::string &to_replace) const
{
    for (const ColorReplace &change : changes) {
        if (change.color_to_replace_str == to_replace)
            return {change};
    }
    return {};
}
		
std::optional<ColorReplace> ColorReplaces::has_value(const std::string & new_col) const
{
    for (const ColorReplace &change : changes) {
        if (change.new_color_str == new_col)
            return {change};
    }
    return {};
}

ColorRGBA picking_decode(unsigned int id)
{
	return {
			   float((id >> 0) & 0xff) * INV_255,  // red
			   float((id >> 8) & 0xff) * INV_255,  // green
			   float((id >> 16) & 0xff) * INV_255, // blue
			   float(picking_checksum_alpha_channel(id & 0xff, (id >> 8) & 0xff, (id >> 16) & 0xff)) * INV_255 // checksum for validating against unwanted alpha blending and multi sampling
		   };
}

unsigned int picking_encode(unsigned char r, unsigned char g, unsigned char b) { return r + (g << 8) + (b << 16); }

unsigned char picking_checksum_alpha_channel(unsigned char red, unsigned char green, unsigned char blue)
{
	// 8 bit hash for the color
	unsigned char b = ((((37 * red) + green) & 0x0ff) * 37 + blue) & 0x0ff;
	// Increase enthropy by a bit reversal
	b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
	b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
	b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
	// Flip every second bit to increase the enthropy even more.
	b ^= 0x55;
	return b;
}

} // namespace Slic3r

