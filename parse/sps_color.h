// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// The color an H.264 or H.265 stream's sequence parameter set says (its VUI
// video_signal_type), found in the access units on their way to a decoder, for
// the engines whose pictures come back without it: the V4L2 stateful and
// OpenMAX engines.
//
// The bitstream is attacker-controlled: every read is bounds-checked, and the
// SPS itself goes through the same parsers the stateless engines use.
#ifndef V4L2WC_PARSE_SPS_COLOR_H_
#define V4L2WC_PARSE_SPS_COLOR_H_

#include <cstddef>
#include <cstdint>

namespace v4l2wc {

enum class SpsCodec { kH264, kH265 };

// Codes from H.264/H.265 Annex E; 2 is "unspecified". With them the SPS's
// bit depths, which an engine whose pictures are 8-bit NV12 checks.
struct SpsColor {
  uint8_t colour_primaries = 2;
  uint8_t transfer_characteristics = 2;
  uint8_t matrix_coeffs = 2;
  bool video_full_range_flag = false;
  uint8_t bit_depth_luma = 8;
  uint8_t bit_depth_chroma = 8;
};

// Looks through an Annex B access unit, as far as its first coded slice, for
// an SPS that parses. True with *out set to what it says (unspecified where it
// has no VUI or the VUI no color description); false with *out untouched when
// there is none.
bool FindSpsColor(SpsCodec codec, const uint8_t* data, size_t size,
                  SpsColor* out);

}  // namespace v4l2wc

#endif  // V4L2WC_PARSE_SPS_COLOR_H_
