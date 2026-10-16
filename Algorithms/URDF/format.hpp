/*******************************************************************************
* Copyright (c) 2026.
* IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
* All rights reserved.
******************************************************************************/

/**
 * @file  format.hpp
 * @brief Compile-time number to string conversion.
 *
 * @details
 * Used to render the parsed model as text during constant evaluation, so the
 * whole report can live in .rodata. That doubles as evidence that no runtime
 * formatting code is emitted, and it is the only way to inspect the result when
 * the target binary cannot be executed on the host.
 */

#ifndef FINEMOTE_URDF_FORMAT_HPP
#define FINEMOTE_URDF_FORMAT_HPP

#include "util.hpp"

#include <array>
#include <cstddef>

namespace fineurdf {

/**
 * @brief Heap-free string builder for constant evaluation.
 */
template <std::size_t Cap>
struct FixedString {
    static_assert(Cap > 0, "FixedString needs space for a terminator");
    std::array<char, Cap> data{};
    std::size_t len = 0;
    bool overflow = false;

    constexpr void push(char c) noexcept {
        if (len < Cap - 1) data[len++] = c;
        else overflow = true;
    }

    constexpr void append(sv s) noexcept {
        for (char c : s) push(c);
    }

    constexpr void newline() noexcept { push('\n'); }

    constexpr sv view() const noexcept { return sv{data.data(), len}; }
};

template <std::size_t Cap>
constexpr void append_uint(FixedString<Cap>& out, unsigned long long v) noexcept {
    char buf[24];
    int n = 0;
    if (v == 0) buf[n++] = '0';
    while (v > 0) { buf[n++] = static_cast<char>('0' + (v % 10)); v /= 10; }
    while (n > 0) out.push(buf[--n]);
}

/**
 * @brief Append a fixed-point decimal with the given number of fraction digits.
 */
template <std::size_t Cap>
constexpr void append_fixed(FixedString<Cap>& out, double v, int digits = 5) noexcept {
    if (v < 0.0) { out.push('-'); v = -v; }
    double scale = 1.0;
    for (int k = 0; k < digits; ++k) scale *= 10.0;
    const double rounded = static_cast<double>(static_cast<long long>(v * scale + 0.5));
    const unsigned long long scaled = static_cast<unsigned long long>(rounded);
    const unsigned long long ip = scaled / static_cast<unsigned long long>(scale);
    const unsigned long long fp = scaled % static_cast<unsigned long long>(scale);
    append_uint(out, ip);
    if (digits > 0) {
        out.push('.');
        unsigned long long mod = static_cast<unsigned long long>(scale) / 10;
        while (mod > 0) {
            out.push(static_cast<char>('0' + (fp / mod) % 10));
            mod /= 10;
        }
    }
}

template <std::size_t Cap>
constexpr void append_hex(FixedString<Cap>& out, unsigned long long v, bool prefix = false) noexcept {
    if (prefix) out.append("0x");
    char buf[16];
    int n = 0;
    if (v == 0) buf[n++] = '0';
    while (v > 0) {
        const int d = static_cast<int>(v & 0xF);
        buf[n++] = static_cast<char>(d < 10 ? ('0' + d) : ('a' + d - 10));
        v >>= 4;
    }
    while (n > 0) out.push(buf[--n]);
}

}  // namespace fineurdf

#endif // FINEMOTE_URDF_FORMAT_HPP
