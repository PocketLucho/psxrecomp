// dc_target.cpp - Dreamcast (SH-4) lean output for generated translation units.
//
// Enabled with `--target dc`. The normal emitter output stays the reference;
// this pass rewrites each emitted TU so it can run on a small 32-bit host:
//
//   1. Guest stores go through DC_SW/DC_SH/DC_SB (runtime/include/dc_target.h)
//      instead of the host write_* function pointers; tooling-only store-PC
//      bookkeeping is dropped.
//   2. Inside every generated function the MIPS GPRs (and HI/LO) it touches
//      live in C locals. They are loaded on entry, written back before any
//      operation that can observe the guest register file (guest calls,
//      dispatch, syscalls, exceptions, interrupt service) and before returning,
//      and reloaded after such operations.
//   3. The common `if (psx_check_interrupts_at(cpu, X)) return;` form shares a
//      single cold write-back stub per function, resumed with a computed goto,
//      so the register cache does not multiply code size.
//
// The pass is textual on purpose: it relies only on shapes the emitter
// produces and is easy to diff against the reference output.

#include "dc_target_pass.h"

#include <cctype>
#include <set>
#include <string>
#include <vector>

namespace PSXRecomp {
namespace {

bool is_ident(char c) {
    return std::isalnum(static_cast<unsigned char>(c)) || c == '_';
}

void replace_all(std::string& s, const std::string& from, const std::string& to) {
    if (from.empty()) return;
    std::string out;
    out.reserve(s.size());
    size_t pos = 0;
    for (;;) {
        size_t hit = s.find(from, pos);
        if (hit == std::string::npos) {
            out.append(s, pos, std::string::npos);
            break;
        }
        out.append(s, pos, hit - pos);
        out += to;
        pos = hit + from.size();
    }
    s.swap(out);
}

// Drop `g_debug_last_store_pc = 0x........u;` (tooling-only attribution).
void drop_store_pc(std::string& s) {
    static const std::string key = "g_debug_last_store_pc = 0x";
    std::string out;
    out.reserve(s.size());
    size_t pos = 0;
    for (;;) {
        size_t hit = s.find(key, pos);
        if (hit == std::string::npos) {
            out.append(s, pos, std::string::npos);
            break;
        }
        size_t p = hit + key.size();
        while (p < s.size() && std::isxdigit(static_cast<unsigned char>(s[p]))) ++p;
        if (p + 1 < s.size() && s[p] == 'u' && s[p + 1] == ';') {
            out.append(s, pos, hit - pos);
            pos = p + 2;
        } else {
            out.append(s, pos, p - pos);
            pos = p;
        }
    }
    s.swap(out);
}

// Index just past the ')' matching the '(' at s[i].
size_t match_paren(const std::string& s, size_t i) {
    int depth = 0;
    for (; i < s.size(); ++i) {
        if (s[i] == '(') {
            ++depth;
        } else if (s[i] == ')') {
            if (--depth == 0) return i + 1;
        }
    }
    return s.size();
}

// Pseudo register numbers: 0..31 GPRs, 32 = HI, 33 = LO.
constexpr int kHi = 32;
constexpr int kLo = 33;

std::string mem_name(int n) {
    if (n == kHi) return "cpu->hi";
    if (n == kLo) return "cpu->lo";
    return "cpu->gpr[" + std::to_string(n) + "]";
}

// Parse `cpu->gpr[N]`, `cpu->hi`, `cpu->lo` at s[i]. Returns length or 0.
size_t parse_reg(const std::string& s, size_t i, int& reg) {
    static const std::string gpr = "cpu->gpr[";
    if (s.compare(i, gpr.size(), gpr) == 0) {
        size_t p = i + gpr.size();
        int n = 0;
        size_t d = p;
        while (d < s.size() && std::isdigit(static_cast<unsigned char>(s[d]))) {
            n = n * 10 + (s[d] - '0');
            ++d;
        }
        if (d == p || d >= s.size() || s[d] != ']') return 0;
        reg = n;
        return d + 1 - i;
    }
    if (s.compare(i, 7, "cpu->hi") == 0 && (i + 7 >= s.size() || !is_ident(s[i + 7]))) {
        reg = kHi;
        return 7;
    }
    if (s.compare(i, 7, "cpu->lo") == 0 && (i + 7 >= s.size() || !is_ident(s[i + 7]))) {
        reg = kLo;
        return 7;
    }
    return 0;
}

bool followed_by_assign(const std::string& s, size_t p) {
    while (p < s.size() && (s[p] == ' ' || s[p] == '\t')) ++p;
    return p + 1 < s.size() && s[p] == '=' && s[p + 1] != '=';
}

bool starts_with_at(const std::string& s, size_t i, const char* lit) {
    for (size_t k = 0; lit[k]; ++k) {
        if (i + k >= s.size() || s[i + k] != lit[k]) return false;
    }
    return true;
}

// Calls that may read or write the guest register file through `cpu`.
enum class Barrier { None, Void, Int };

Barrier barrier_at(const std::string& s, size_t i, size_t& name_len) {
    if (i > 0 && is_ident(s[i - 1])) return Barrier::None;
    size_t e = i;
    while (e < s.size() && is_ident(s[e])) ++e;
    if (e == i || !starts_with_at(s, e, "(cpu")) return Barrier::None;
    if (e + 4 < s.size() && is_ident(s[e + 4])) return Barrier::None;
    const std::string name = s.substr(i, e - i);
    name_len = e - i;
    auto hex8_after = [&](const char* prefix) {
        size_t pl = std::char_traits<char>::length(prefix);
        if (name.compare(0, pl, prefix) != 0 || name.size() < pl + 8) return false;
        for (size_t k = pl; k < pl + 8; ++k) {
            if (!std::isxdigit(static_cast<unsigned char>(name[k]))) return false;
        }
        return true;
    };
    if (hex8_after("func_") || hex8_after("psx_alias_body_")) return Barrier::Void;
    static const std::set<std::string> void_names = {
        "psx_syscall", "psx_break", "psx_unknown_dispatch", "psx_dispatch",
        "psx_unaligned_access", "psx_arith_overflow", "psx_restore_state_escape",
        "psx_rfe_mark_escape", "psx_raise_ri"};
    static const std::set<std::string> int_names = {
        "psx_mod_function_entry", "psx_xg_render_auth_resident_ft4_observe",
        "psx_datashard_enter", "psx_vsync_query_hle_enter"};
    if (void_names.count(name) || name.rfind("psx_raise_", 0) == 0) return Barrier::Void;
    if (int_names.count(name)) return Barrier::Int;
    return Barrier::None;
}

struct FunctionRewrite {
    std::string spill;   // write-back of registers this function writes
    std::string reload;  // reload of every register this function uses
};

// `cpu->MEM[...]` spelling keeps spills/reloads out of the register renaming.
std::string spill_code(const std::set<int>& written) {
    std::string out;
    for (int n : written) {
        out += "cpu->MEM" + std::to_string(n) + "=r" + std::to_string(n) + "; ";
    }
    return out;
}

std::string reload_code(const std::set<int>& used) {
    std::string out;
    for (int n : used) {
        out += "r" + std::to_string(n) + "=cpu->MEM" + std::to_string(n) + "; ";
    }
    return out;
}

void restore_mem_spelling(std::string& s) {
    std::string out;
    out.reserve(s.size());
    size_t pos = 0;
    for (;;) {
        size_t hit = s.find("cpu->MEM", pos);
        if (hit == std::string::npos) {
            out.append(s, pos, std::string::npos);
            break;
        }
        out.append(s, pos, hit - pos);
        size_t d = hit + 8;
        int n = 0;
        while (d < s.size() && std::isdigit(static_cast<unsigned char>(s[d]))) {
            n = n * 10 + (s[d] - '0');
            ++d;
        }
        out += mem_name(n);
        pos = d;
    }
    s.swap(out);
}

std::string rewrite_body(const std::string& body, std::set<int>& used_out) {
    std::set<int> used, written;
    for (size_t i = 0; i < body.size(); ++i) {
        int reg = 0;
        size_t len = parse_reg(body, i, reg);
        if (!len) continue;
        if (reg != 0) used.insert(reg);
        if (reg != 0 && followed_by_assign(body, i + len)) written.insert(reg);
        i += len - 1;
    }
    used_out = used;
    if (used.empty()) return body;

    const std::string sp = spill_code(written);
    const std::string rl = reload_code(used);

    // Pass A: barrier calls, the shared interrupt stub, other interrupt
    // checks and returns. Text is copied verbatim otherwise.
    std::string a;
    a.reserve(body.size() + body.size() / 4);
    int irq_sites = 0;
    static const std::string irq_stmt = "if (psx_check_interrupts_at(cpu, ";
    static const std::string irq_expr = "psx_check_interrupts_at(cpu,";
    for (size_t i = 0; i < body.size();) {
        if (body.compare(i, irq_stmt.size(), irq_stmt) == 0) {
            size_t arg = i + irq_stmt.size();
            size_t close = body.find(')', arg);
            size_t lparen = body.find('(', arg);
            if (close != std::string::npos && (lparen == std::string::npos || lparen > close) &&
                body.compare(close, 10, ")) return;") == 0) {
                ++irq_sites;
                const std::string n = std::to_string(irq_sites);
                a += "if (__builtin_expect(g_irq_pending, 0)) { _dc_irq_pc = (" +
                     body.substr(arg, close - arg) + "); _dc_irq_back = &&dc_irqb_" + n +
                     "; goto dc_irq_stub; } dc_irqb_" + n + ": ;";
                i = close + 10;
                continue;
            }
        }
        if (body.compare(i, irq_expr.size(), irq_expr) == 0 &&
            (i == 0 || !is_ident(body[i - 1]))) {
            size_t lp = i + std::string("psx_check_interrupts_at").size();
            size_t end = match_paren(body, lp);
            const std::string inner = body.substr(lp + 1, end - lp - 2);
            a += "(__builtin_expect(g_irq_pending, 0) ? ({ " + sp + "int _dcr = dc_irq_slow(" +
                 inner + "); " + rl + "_dcr; }) : 0)";
            i = end;
            continue;
        }
        size_t name_len = 0;
        Barrier b = is_ident(body[i]) ? barrier_at(body, i, name_len) : Barrier::None;
        if (b != Barrier::None) {
            size_t end = match_paren(body, i + name_len);
            const std::string call = body.substr(i, end - i);
            if (b == Barrier::Void) {
                a += "({ " + sp + call + "; " + rl + "})";
            } else {
                a += "({ " + sp + "int _dcr = " + call + "; " + rl + "_dcr; })";
            }
            i = end;
            continue;
        }
        if (body.compare(i, 6, "return") == 0 && (i == 0 || !is_ident(body[i - 1])) &&
            (i + 6 >= body.size() || !is_ident(body[i + 6]))) {
            size_t p = i + 6;
            while (p < body.size() && (body[p] == ' ' || body[p] == '\t')) ++p;
            if (p < body.size() && body[p] == ';') {
                a += "goto dc_ret_stub;";
                i = p + 1;
                continue;
            }
        }
        a += body[i++];
    }

    // Pass B: rename register accesses to locals; $zero writes go to a sink.
    std::string b;
    b.reserve(a.size());
    for (size_t i = 0; i < a.size();) {
        int reg = 0;
        size_t len = parse_reg(a, i, reg);
        if (len) {
            if (reg == 0) {
                b += followed_by_assign(a, i + len) ? "_dc_r0sink" : "0u";
            } else {
                b += "r" + std::to_string(reg);
            }
            i += len;
            continue;
        }
        b += a[i++];
    }

    std::string tail = "\n    return;\n";
    if (irq_sites) {
        tail += "dc_irq_stub: " + sp + "if (dc_irq_slow(cpu, _dc_irq_pc)) return; " + rl +
                "goto *_dc_irq_back;\n";
    }
    tail += "dc_ret_stub: " + sp + "return;\n";

    std::string decl = "\n    uint32_t _dc_r0sink; (void)_dc_r0sink; uint32_t ";
    bool first = true;
    for (int n : used) {
        if (!first) decl += ", ";
        first = false;
        decl += "r" + std::to_string(n) + " = " + mem_name(n);
    }
    decl += ";";
    std::string out = decl + "uint32_t _dc_irq_pc; void *_dc_irq_back; (void)_dc_irq_pc; (void)_dc_irq_back;\n" +
                      b + tail;
    restore_mem_spelling(out);
    return out;
}

// Start of a generated function definition: `void NAME(CPUState* cpu...)` then '{'.
bool function_header_at(const std::string& s, size_t i, size_t& brace) {
    if (i != 0 && s[i - 1] != '\n') return false;
    size_t p = i;
    if (starts_with_at(s, p, "static ")) p += 7;
    if (!starts_with_at(s, p, "void ")) return false;
    p += 5;
    size_t name = p;
    while (p < s.size() && is_ident(s[p])) ++p;
    if (p == name || !starts_with_at(s, p, "(CPUState* cpu")) return false;
    size_t close = s.find(')', p);
    if (close == std::string::npos) return false;
    size_t q = close + 1;
    while (q < s.size() && (s[q] == ' ' || s[q] == '\n' || s[q] == '\r')) ++q;
    if (q >= s.size() || s[q] != '{') return false;
    brace = q;
    return true;
}

}  // namespace

std::string dc_target_rewrite_tu(const std::string& tu, const std::string& after_include) {
    std::string s = tu;
    replace_all(s, "cpu->write_word(", "DC_SW(");
    replace_all(s, "cpu->write_half(", "DC_SH(");
    replace_all(s, "cpu->write_byte(", "DC_SB(");
    drop_store_pc(s);

    std::string out;
    out.reserve(s.size() + s.size() / 3);
    size_t pos = 0;
    for (size_t i = 0; i < s.size(); ++i) {
        size_t brace = 0;
        if (s[i] != 'v' && s[i] != 's') continue;
        if (!function_header_at(s, i, brace)) continue;
        size_t end = s.find("\n}", brace);
        if (end == std::string::npos) break;
        out.append(s, pos, brace + 1 - pos);
        std::set<int> used;
        out += rewrite_body(s.substr(brace + 1, end + 1 - (brace + 1)), used);
        pos = end + 1;
        i = end;
    }
    out.append(s, pos, std::string::npos);

    // Make the DC overrides visible right after the TU's main include.
    if (!after_include.empty()) {
        const std::string inc = "#include \"" + after_include + "\"";
        size_t at = out.find(inc);
        if (at != std::string::npos) {
            out.insert(at + inc.size(), "\n#include \"dc_target.h\"");
            return out;
        }
    }
    // Fallback: right after the last top-of-file #include line.
    size_t last = std::string::npos, scan = 0;
    while (true) {
        size_t hit = out.find("\n#include ", scan);
        if (hit == std::string::npos || hit > 4096) break;
        last = hit;
        scan = hit + 1;
    }
    if (last != std::string::npos) {
        size_t eol = out.find('\n', last + 1);
        out.insert(eol == std::string::npos ? out.size() : eol, "\n#include \"dc_target.h\"");
    } else {
        out.insert(0, "#include \"cpu_state.h\"\n#include \"dc_target.h\"\n");
    }
    return out;
}

}  // namespace PSXRecomp
