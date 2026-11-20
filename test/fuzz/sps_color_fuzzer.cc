// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// libFuzzer harness over FindSpsColor. The first byte picks the codec; the
// rest is fed as an Annex B access unit. Relies on the bounds-check
// invariants (verified under ASan/UBSan).

#include <cstddef>
#include <cstdint>

#include "parse/sps_color.h"

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  if (size == 0) {
    return 0;
  }
  const auto codec =
      (data[0] & 1) != 0 ? v4l2wc::SpsCodec::kH265 : v4l2wc::SpsCodec::kH264;
  v4l2wc::SpsColor color;
  v4l2wc::FindSpsColor(codec, data + 1, size - 1, &color);
  return 0;
}
