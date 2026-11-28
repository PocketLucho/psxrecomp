#pragma once
// Dreamcast (SH-4) lean output pass for generated translation units.
// See recompiler/src/dc_target.cpp and runtime/include/dc_target.h.
#include <string>

namespace PSXRecomp {

// Rewrites one generated TU for the `--target dc` build. `after_include` is
// the header the TU includes first (e.g. "<stem>_decls.h"); the DC overrides
// header is included right after it.
std::string dc_target_rewrite_tu(const std::string& tu, const std::string& after_include);

}  // namespace PSXRecomp
