/*******************************************************************************
* Copyright (c) 2026.
* IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
* All rights reserved.
******************************************************************************/

/**
 * @file  util.hpp
 * @brief Compile-time string and numeric helpers.
 *
 * @details
 * Everything here is constexpr and free of C library calls, because the usual
 * facilities (strtod, from_chars) are not usable in constant evaluation.
 */

#ifndef FINEMOTE_URDF_UTIL_HPP
#define FINEMOTE_URDF_UTIL_HPP

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>

namespace fineurdf {

using sv = std::string_view;

constexpr bool is_space(char c) noexcept {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v';
}

constexpr bool is_digit(char c) noexcept { return c >= '0' && c <= '9'; }

constexpr bool is_alpha(char c) noexcept {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}

constexpr sv trim(sv s) noexcept {
    std::size_t b = 0, e = s.size();
    while (b < e && is_space(s[b])) ++b;
    while (e > b && is_space(s[e - 1])) --e;
    return s.substr(b, e - b);
}

constexpr bool equals(sv a, sv b) noexcept { return a.size() == b.size() && a == b; }

constexpr bool iequals(sv a, sv b) noexcept {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        char x = a[i], y = b[i];
        if (x >= 'A' && x <= 'Z') x = static_cast<char>(x - 'A' + 'a');
        if (y >= 'A' && y <= 'Z') y = static_cast<char>(y - 'A' + 'a');
        if (x != y) return false;
    }
    return true;
}

/**
 * @brief Parse a decimal or 0x-prefixed hexadecimal unsigned integer.
 */
constexpr bool try_parse_uint(sv s, std::uint64_t& value) noexcept {
    s = trim(s);
    if (s.empty()) return false;
    std::size_t i = 0;
    int base = 10;
    if (s.size() > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        base = 16;
        i = 2;
    }
    std::uint64_t v = 0;
    for (; i < s.size(); ++i) {
        const char c = s[i];
        int d;
        if (c >= '0' && c <= '9') d = c - '0';
        else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
        else return false;
        if (d >= base) return false;
        if (v > (std::numeric_limits<std::uint64_t>::max() - d) / base) return false;
        v = v * static_cast<std::uint64_t>(base) + static_cast<std::uint64_t>(d);
    }
    value = v;
    return true;
}

constexpr std::uint64_t parse_uint(sv s) noexcept {
    std::uint64_t value = 0;
    return try_parse_uint(s, value) ? value : 0;
}

/**
 * @brief Parse a floating point literal with optional fraction and exponent.
 * @note Precision is sufficient for URDF geometry and limits.
 */
constexpr bool try_parse_double(sv s, double& value) noexcept {
    s = trim(s);
    std::size_t i = 0;
    bool neg = false;
    if (i < s.size() && (s[i] == '+' || s[i] == '-')) {
        neg = (s[i] == '-');
        ++i;
    }
    double v = 0.0;
    std::size_t digits = 0;
    for (; i < s.size() && is_digit(s[i]); ++i) {
        if (v > (std::numeric_limits<double>::max() - 9.0) / 10.0) return false;
        v = v * 10.0 + static_cast<double>(s[i] - '0');
        ++digits;
    }
    if (i < s.size() && s[i] == '.') {
        ++i;
        double scale = 0.1;
        for (; i < s.size() && is_digit(s[i]); ++i) {
            v += static_cast<double>(s[i] - '0') * scale;
            scale *= 0.1;
            ++digits;
        }
    }
    if (digits == 0) return false;
    if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
        ++i;
        bool eneg = false;
        if (i < s.size() && (s[i] == '+' || s[i] == '-')) {
            eneg = (s[i] == '-');
            ++i;
        }
        int e = 0;
        const std::size_t begin = i;
        for (; i < s.size() && is_digit(s[i]); ++i) {
            if (e > 308) return false;
            e = e * 10 + (s[i] - '0');
        }
        if (i == begin || e > 308) return false;
        double p = 1.0;
        for (int k = 0; k < e; ++k) p *= 10.0;
        if (!eneg && v > std::numeric_limits<double>::max() / p) return false;
        v = eneg ? v / p : v * p;
    }
    if (i != s.size()) return false;
    value = neg ? -v : v;
    return true;
}

constexpr double parse_double(sv s) noexcept {
    double value = 0.0;
    return try_parse_double(s, value) ? value : 0.0;
}

constexpr bool valid_float(sv s) noexcept {
    double value = 0.0;
    return try_parse_double(s, value) &&
        value <= std::numeric_limits<float>::max() && value >= -std::numeric_limits<float>::max();
}

constexpr bool valid_vector(sv s, std::size_t count = 3) noexcept {
    s = trim(s);
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t end = s.find_first_of(" \t\r\n");
        if (!valid_float(s.substr(0, end))) return false;
        s = end == sv::npos ? sv{} : trim(s.substr(end));
    }
    return s.empty();
}

constexpr float parse_float(sv s) noexcept { return static_cast<float>(parse_double(s)); }

constexpr bool parse_bool(sv s) noexcept {
    return iequals(s, "true") || equals(s, "1") || iequals(s, "yes") || iequals(s, "on");
}

}  // namespace fineurdf

#endif // FINEMOTE_URDF_UTIL_HPP
