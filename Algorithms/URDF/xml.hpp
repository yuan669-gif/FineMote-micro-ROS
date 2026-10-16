/*******************************************************************************
* Copyright (c) 2026.
* IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
* All rights reserved.
******************************************************************************/

/**
 * @file  xml.hpp
 * @brief Compile-time XML reader covering the subset used by URDF.
 *
 * @details
 * Constraints that shape the design:
 * 1. No heap during constant evaluation, so every container is a fixed-size array
 *    sized by a template parameter. Exceeding a budget is reported as an error
 *    rather than silently truncating.
 * 2. No tree allocation: nodes live in a flat array and the hierarchy is expressed
 *    with index links, which keeps the whole structure a POD.
 * 3. All views point into the URDF string literal, whose lifetime outlives the
 *    document. That is what makes the projected model usable as a template argument.
 *
 * Supported: declarations, comments, processing instructions, self-closing
 * and paired tags, quoted attributes. Character data is skipped apart from the
 * first text run of each element, which URDF needs for <type> and <plugin>.
 * DTDs, CDATA and entity references are rejected rather than misinterpreted.
 */

#ifndef FINEMOTE_URDF_XML_HPP
#define FINEMOTE_URDF_XML_HPP

#include "util.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace fineurdf {

struct XmlAttr {
    sv name{};
    sv value{};
};

struct XmlNode {
    static constexpr std::uint16_t kNone = 0xFFFF;

    sv tag{};
    sv text{};                       // first character-data run, used by <type>/<plugin>
    std::uint16_t parent = kNone;
    std::uint16_t first_child = kNone;
    std::uint16_t next_sibling = kNone;
    std::uint16_t attr_begin = 0;
    std::uint8_t attr_count = 0;
    std::uint8_t depth = 0;
    std::uint8_t flags = 0;          // bit0: self-closing
};

struct XmlLimits {
    std::uint8_t max_attrs_per_node = 24;
    std::uint8_t max_depth = 12;
};

/**
 * @brief Memory budget of the syntax layer.
 * @note Sized for every XML element, not just the semantically relevant ones,
 *       so it is larger than ModelLimits. Measured reference: a 24-joint URDF
 *       uses about 560 nodes and 1450 attributes.
 */
struct SyntaxBudget {
    std::size_t nodes = 2048;
    std::size_t attrs = 8192;
};

enum class XmlErr : std::uint8_t {
    None = 0,
    TooManyNodes,
    TooManyAttrs,
    TooManyAttrsPerNode,
    DepthOverflow,
    UnclosedTag,
    MalformedTag,
    StrayCloseTag,
    NoRoot,
};

constexpr sv xml_err_message(XmlErr e) noexcept {
    switch (e) {
        case XmlErr::None: return "no error";
        case XmlErr::TooManyNodes: return "too many XML nodes: raise SyntaxBudget::nodes";
        case XmlErr::TooManyAttrs: return "too many XML attributes: raise SyntaxBudget::attrs";
        case XmlErr::TooManyAttrsPerNode: return "too many attrs on one element: raise max_attrs_per_node";
        case XmlErr::DepthOverflow: return "XML nesting too deep: raise XmlLimits::max_depth";
        case XmlErr::UnclosedTag: return "unclosed XML tag at end of document";
        case XmlErr::MalformedTag: return "malformed XML tag";
        case XmlErr::StrayCloseTag: return "</tag> without matching open tag";
        case XmlErr::NoRoot: return "no root element found";
    }
    return "unknown";
}

template <std::size_t MaxNodes, std::size_t MaxAttrs>
struct XmlDocument {
    static_assert(MaxNodes < XmlNode::kNone && MaxAttrs <= 65535, "XML budget exceeds index range");
    std::array<XmlNode, MaxNodes> nodes{};
    std::array<XmlAttr, MaxAttrs> attrs{};
    std::uint16_t node_count = 0;
    std::uint16_t attr_count = 0;
    std::uint16_t root = XmlNode::kNone;
    XmlErr err = XmlErr::None;
    std::size_t err_offset = 0;
    sv err_tag{};                    // tag that triggered the error, for diagnostics

    constexpr bool ok() const noexcept { return err == XmlErr::None; }
    constexpr const XmlNode& node(std::uint16_t i) const noexcept { return nodes[i]; }
};

namespace detail {

struct Cursor {
    sv s{};
    std::size_t i = 0;

    constexpr bool eof() const noexcept { return i >= s.size(); }

    constexpr char peek(std::size_t k = 0) const noexcept {
        return (i + k) < s.size() ? s[i + k] : '\0';
    }

    constexpr void skip_ws() noexcept {
        while (!eof() && is_space(s[i])) ++i;
    }
};

constexpr bool skip_comment(Cursor& c) noexcept {
    if (!(c.s.size() - c.i >= 4 && c.s.compare(c.i, 4, "<!--") == 0)) return false;
    c.i += 4;
    while (!c.eof()) {
        if (c.peek() == '-' && c.peek(1) == '-' && c.peek(2) == '>') {
            c.i += 3;
            return true;
        }
        ++c.i;
    }
    return false;
}

constexpr bool is_name_char(char c) noexcept {
    return is_alpha(c) || is_digit(c) || c == '_' || c == '-' || c == '.' || c == ':';
}

constexpr sv read_name(Cursor& c) noexcept {
    const std::size_t b = c.i;
    if (!(is_alpha(c.peek()) || c.peek() == '_' || c.peek() == ':')) return {};
    while (!c.eof() && is_name_char(c.s[c.i])) ++c.i;
    return c.s.substr(b, c.i - b);
}

}  // namespace detail

/**
 * @brief Second pass: build the document with the given capacity.
 */
template <std::size_t MaxNodes, std::size_t MaxAttrs>
constexpr XmlDocument<MaxNodes, MaxAttrs> parse_xml(sv src, XmlLimits lim = {}) noexcept {
    using detail::Cursor;
    using detail::read_name;

    XmlDocument<MaxNodes, MaxAttrs> doc{};
    Cursor c{src, 0};
    if (src.size() >= 3 && src.substr(0, 3) == "\xef\xbb\xbf") c.i = 3;

    std::array<std::uint16_t, 64> stack{};
    std::size_t sp = 0;

    auto fail = [&](XmlErr e, std::size_t off) constexpr {
        doc.err = e;
        doc.err_offset = off;
    };

    while (!c.eof()) {
        if (doc.err != XmlErr::None) break;

        if (c.peek() != '<') {
            const std::size_t tb = c.i;
            while (!c.eof() && c.peek() != '<') ++c.i;
            const sv text = c.s.substr(tb, c.i - tb);
            if ((sp == 0 && !trim(text).empty()) || text.find('&') != sv::npos) {
                fail(XmlErr::MalformedTag, tb); break;
            }
            if (sp != 0 && doc.nodes[stack[sp - 1]].text.empty())
                doc.nodes[stack[sp - 1]].text = c.s.substr(tb, c.i - tb);
            continue;
        }

        if (c.peek(1) == '!') {
            if (c.peek(2) == '-' && c.peek(3) == '-') {
                if (!detail::skip_comment(c)) { fail(XmlErr::MalformedTag, c.i); break; }
                continue;
            }
            fail(XmlErr::MalformedTag, c.i);
            break;
        }

        if (c.peek(1) == '?') {
            bool closed = false;
            while (!c.eof()) {
                if (c.peek() == '?' && c.peek(1) == '>') { c.i += 2; closed = true; break; }
                ++c.i;
            }
            if (!closed) fail(XmlErr::UnclosedTag, c.i);
            continue;
        }

        if (c.peek(1) == '/') {
            c.i += 2;
            const sv name = read_name(c);
            c.skip_ws();
            if (c.peek() != '>') { fail(XmlErr::MalformedTag, c.i); break; }
            ++c.i;
            if (sp == 0 || doc.nodes[stack[sp - 1]].tag != name) {
                fail(XmlErr::StrayCloseTag, c.i);
                break;
            }
            --sp;
            continue;
        }

        ++c.i;
        if (doc.node_count >= MaxNodes) { fail(XmlErr::TooManyNodes, c.i); break; }
        if (sp >= lim.max_depth || sp >= stack.size()) { fail(XmlErr::DepthOverflow, c.i); break; }
        if (sp == 0 && doc.root != XmlNode::kNone) { fail(XmlErr::MalformedTag, c.i); break; }

        XmlNode& n = doc.nodes[doc.node_count];
        n = XmlNode{};
        n.tag = read_name(c);
        if (n.tag.empty()) { fail(XmlErr::MalformedTag, c.i); break; }
        n.depth = static_cast<std::uint8_t>(sp);
        n.attr_begin = doc.attr_count;
        n.parent = sp ? stack[sp - 1] : XmlNode::kNone;

        if (n.parent != XmlNode::kNone) {
            std::uint16_t& cur = doc.nodes[n.parent].first_child;
            if (cur == XmlNode::kNone) {
                cur = doc.node_count;
            } else {
                std::uint16_t w = cur;
                while (doc.nodes[w].next_sibling != XmlNode::kNone) w = doc.nodes[w].next_sibling;
                doc.nodes[w].next_sibling = doc.node_count;
            }
        }

        bool self_close = false;
        for (;;) {
            c.skip_ws();
            if (c.peek() == '/' && c.peek(1) == '>') { c.i += 2; self_close = true; break; }
            if (c.peek() == '>') { ++c.i; break; }
            if (c.eof()) { fail(XmlErr::UnclosedTag, c.i); break; }
            if (doc.attr_count >= MaxAttrs) { fail(XmlErr::TooManyAttrs, c.i); break; }
            if (n.attr_count >= lim.max_attrs_per_node) {
                doc.err_tag = n.tag;
                fail(XmlErr::TooManyAttrsPerNode, c.i);
                break;
            }

            XmlAttr& a = doc.attrs[doc.attr_count];
            a = XmlAttr{};
            a.name = read_name(c);
            if (a.name.empty()) { fail(XmlErr::MalformedTag, c.i); break; }
            c.skip_ws();
            for (std::uint8_t i = 0; i < n.attr_count; ++i)
                if (doc.attrs[n.attr_begin + i].name == a.name) fail(XmlErr::MalformedTag, c.i);
            if (doc.err != XmlErr::None) break;
            if (c.peek() != '=') { fail(XmlErr::MalformedTag, c.i); break; }
            ++c.i;
            c.skip_ws();
            const char q = c.peek();
            if (q != '"' && q != '\'') { fail(XmlErr::MalformedTag, c.i); break; }
            ++c.i;
            const std::size_t vb = c.i;
            while (!c.eof() && c.peek() != q) ++c.i;
            if (c.eof()) { fail(XmlErr::UnclosedTag, c.i); break; }
            a.value = c.s.substr(vb, c.i - vb);
            if (a.value.find_first_of("<&") != sv::npos) { fail(XmlErr::MalformedTag, vb); break; }
            ++c.i;
            if (!is_space(c.peek()) && c.peek() != '/' && c.peek() != '>') {
                fail(XmlErr::MalformedTag, c.i); break;
            }
            ++doc.attr_count;
            ++n.attr_count;
        }
        if (doc.err != XmlErr::None) break;

        if (self_close) {
            n.flags |= 1u;
        } else {
            stack[sp++] = doc.node_count;
        }

        if (doc.root == XmlNode::kNone) doc.root = doc.node_count;
        ++doc.node_count;
    }

    if (doc.err == XmlErr::None && sp != 0) doc.err = XmlErr::UnclosedTag;
    if (doc.err == XmlErr::None && doc.root == XmlNode::kNone) doc.err = XmlErr::NoRoot;
    return doc;
}

// The parser enforces the caller's budget while building the document.
template <SyntaxBudget B>
constexpr XmlDocument<B.nodes, B.attrs> parse_checked(sv src, XmlLimits lim = {}) noexcept {
    return parse_xml<B.nodes, B.attrs>(src, lim);
}
inline constexpr SyntaxBudget kDefaultSyntaxBudget{};

constexpr XmlDocument<kDefaultSyntaxBudget.nodes, kDefaultSyntaxBudget.attrs>
parse_auto(sv src) noexcept {
    return parse_checked<kDefaultSyntaxBudget>(src, XmlLimits{});
}

}  // namespace fineurdf

#endif // FINEMOTE_URDF_XML_HPP
