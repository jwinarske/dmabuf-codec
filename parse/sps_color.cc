// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "parse/sps_color.h"

#include <vector>

#include "parse/h264/sps.h"
#include "parse/h265/sps.h"

namespace v4l2wc {
namespace {

// The longest SPS taken; a real one is a few hundred bytes at most.
constexpr size_t kMaxSps = 64 * 1024;

// The RBSP of the NAL payload starting at data[begin] (after its header), up
// to the next start code, with the emulation-prevention bytes removed.
std::vector<uint8_t> Rbsp(const uint8_t* data, size_t size, size_t begin) {
  std::vector<uint8_t> rbsp;
  int zeros = 0;
  for (size_t j = begin; j < size && rbsp.size() < kMaxSps; ++j) {
    const uint8_t c = data[j];
    if (zeros >= 2 && c <= 1) {
      break;  // 00 00 00 or 00 00 01: the next start code
    }
    if (zeros >= 2 && c == 3) {
      zeros = 0;
      continue;
    }
    zeros = (c == 0) ? zeros + 1 : 0;
    rbsp.push_back(c);
  }
  return rbsp;
}

bool ParseH264(const std::vector<uint8_t>& rbsp, SpsColor* out) {
  h264::Sps sps;
  if (!h264::ParseSps(rbsp.data(), rbsp.size(), &sps)) {
    return false;
  }
  out->colour_primaries = sps.colour_primaries;
  out->transfer_characteristics = sps.transfer_characteristics;
  out->matrix_coeffs = sps.matrix_coeffs;
  out->video_full_range_flag = sps.video_full_range_flag;
  return true;
}

bool ParseH265(const std::vector<uint8_t>& rbsp, SpsColor* out) {
  h265::Sps sps;
  if (!h265::ParseSps(rbsp.data(), rbsp.size(), &sps)) {
    return false;
  }
  out->colour_primaries = sps.colour_primaries;
  out->transfer_characteristics = sps.transfer_characteristics;
  out->matrix_coeffs = sps.matrix_coeffs;
  out->video_full_range_flag = sps.video_full_range_flag;
  return true;
}

}  // namespace

bool FindSpsColor(SpsCodec codec, const uint8_t* data, size_t size,
                  SpsColor* out) {
  if (data == nullptr || out == nullptr) {
    return false;
  }
  const size_t header = (codec == SpsCodec::kH264) ? 1 : 2;
  size_t i = 0;
  while (i + 2 + header < size) {
    if (data[i] != 0 || data[i + 1] != 0 || data[i + 2] != 1) {
      ++i;
      continue;
    }
    i += 3;  // the NAL header
    bool sps = false;
    if (codec == SpsCodec::kH264) {
      const uint8_t type = data[i] & 0x1f;
      if (type >= 1 && type <= 5) {
        return false;  // a coded slice: nothing ahead of it said
      }
      sps = (type == 7);
    } else {
      const uint8_t type = (data[i] >> 1) & 0x3f;
      if (type <= 31) {
        return false;  // a VCL unit
      }
      sps = (type == 33);
    }
    if (sps) {
      const std::vector<uint8_t> rbsp = Rbsp(data, size, i + header);
      SpsColor c;
      if (codec == SpsCodec::kH264 ? ParseH264(rbsp, &c)
                                   : ParseH265(rbsp, &c)) {
        *out = c;
        return true;
      }
    }
    ++i;
  }
  return false;
}

}  // namespace v4l2wc
