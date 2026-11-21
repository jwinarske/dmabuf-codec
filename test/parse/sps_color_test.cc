// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Unit test for FindSpsColor. Runs on the host (no webrtc / V4L2). The SPSs
// are libx264's and libx265's (64x64) for the color tags named.

#include "parse/sps_color.h"

#include <cstdint>
#include <cstdio>
#include <vector>

using v4l2wc::FindSpsColor;
using v4l2wc::SpsCodec;
using v4l2wc::SpsColor;

static int g_failures = 0;

#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                               \
    }                                                             \
  } while (0)

namespace {

std::vector<uint8_t> AnnexB(std::initializer_list<std::vector<uint8_t>> nals) {
  std::vector<uint8_t> out;
  for (const auto& n : nals) {
    out.insert(out.end(), {0x00, 0x00, 0x00, 0x01});
    out.insert(out.end(), n.begin(), n.end());
  }
  return out;
}

// H.264, colorprim/transfer/colormatrix=bt709, limited range.
const std::vector<uint8_t> kH264Bt709 = {
    0x67, 0x64, 0x00, 0x0a, 0xac, 0xd9, 0x44, 0x26, 0xc0,
    0x5a, 0x80, 0x80, 0x80, 0xa0, 0x00, 0x00, 0x03, 0x00,
    0x20, 0x00, 0x00, 0x06, 0x41, 0xe2, 0x44, 0xb2, 0xc0};
// H.264, bt470bg, full range.
const std::vector<uint8_t> kH264Bt601Full = {
    0x67, 0x64, 0x00, 0x0a, 0xac, 0xd9, 0x44, 0x26, 0xc0,
    0x5b, 0x82, 0x82, 0x82, 0xa0, 0x00, 0x00, 0x03, 0x00,
    0x20, 0x00, 0x00, 0x06, 0x41, 0xe2, 0x44, 0xb2, 0xc0};
// H.264 Baseline, no VUI.
const std::vector<uint8_t> kH264NoVui = {0x67, 0x42, 0x00, 0x0a,
                                         0xf8, 0x58, 0x98, 0x80};
const std::vector<uint8_t> kH264Pps = {0x68, 0xeb, 0xe3, 0xcb, 0x22, 0xc0};
const std::vector<uint8_t> kH264Idr = {0x65, 0x88, 0x84, 0x00, 0x20};

// H.265, bt709, limited range.
const std::vector<uint8_t> kH265Bt709 = {
    0x42, 0x01, 0x01, 0x01, 0x60, 0x00, 0x00, 0x03, 0x00, 0x90, 0x00,
    0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x1e, 0xa0, 0x20, 0x81, 0x05,
    0x96, 0x56, 0x69, 0x24, 0xca, 0xf0, 0x16, 0xa0, 0x20, 0x20, 0x20,
    0x80, 0x00, 0x00, 0x03, 0x00, 0x80, 0x00, 0x00, 0x0c, 0x84};
// H.265, bt470bg, full range.
const std::vector<uint8_t> kH265Bt601Full = {
    0x42, 0x01, 0x01, 0x01, 0x60, 0x00, 0x00, 0x03, 0x00, 0x90, 0x00,
    0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x1e, 0xa0, 0x20, 0x81, 0x05,
    0x96, 0x56, 0x69, 0x24, 0xca, 0xf0, 0x16, 0xe0, 0xa0, 0xa0, 0xa0,
    0x80, 0x00, 0x00, 0x03, 0x00, 0x80, 0x00, 0x00, 0x0c, 0x84};
const std::vector<uint8_t> kH265Vps = {0x40, 0x01, 0x0c, 0x01, 0xff, 0xff};
const std::vector<uint8_t> kH265Idr = {0x26, 0x01, 0xaf, 0x00, 0x20};

}  // namespace

int main() {
  // H.264 behind an access unit delimiter, ahead of its PPS and slice.
  {
    const auto au = AnnexB({{0x09, 0xf0}, kH264Bt601Full, kH264Pps, kH264Idr});
    SpsColor c;
    CHECK(FindSpsColor(SpsCodec::kH264, au.data(), au.size(), &c));
    CHECK(c.colour_primaries == 5);
    CHECK(c.transfer_characteristics == 5);
    CHECK(c.matrix_coeffs == 5);
    CHECK(c.video_full_range_flag);
  }
  {
    const auto au = AnnexB({kH264Bt709, kH264Pps, kH264Idr});
    SpsColor c;
    CHECK(FindSpsColor(SpsCodec::kH264, au.data(), au.size(), &c));
    CHECK(c.matrix_coeffs == 1);
    CHECK(!c.video_full_range_flag);
  }
  // An SPS with no VUI says "unspecified": a stream that drops its color
  // description does not keep the last one.
  {
    const auto au = AnnexB({kH264NoVui});
    SpsColor c;
    c.matrix_coeffs = 1;
    c.video_full_range_flag = true;
    CHECK(FindSpsColor(SpsCodec::kH264, au.data(), au.size(), &c));
    CHECK(c.matrix_coeffs == 2);
    CHECK(!c.video_full_range_flag);
  }
  // H.265 behind its VPS.
  {
    const auto au = AnnexB({kH265Vps, kH265Bt601Full, kH265Idr});
    SpsColor c;
    CHECK(FindSpsColor(SpsCodec::kH265, au.data(), au.size(), &c));
    CHECK(c.colour_primaries == 5);
    CHECK(c.matrix_coeffs == 5);
    CHECK(c.video_full_range_flag);
  }
  {
    const auto au = AnnexB({kH265Bt709, kH265Idr});
    SpsColor c;
    CHECK(FindSpsColor(SpsCodec::kH265, au.data(), au.size(), &c));
    CHECK(c.colour_primaries == 1);
    CHECK(c.transfer_characteristics == 1);
    CHECK(c.matrix_coeffs == 1);
    CHECK(!c.video_full_range_flag);
  }
  // Nothing past the first coded slice is looked at, and an access unit with
  // no SPS leaves the color as it was.
  {
    SpsColor c;
    c.matrix_coeffs = 9;
    const auto late = AnnexB({kH264Idr, kH264Bt709});
    CHECK(!FindSpsColor(SpsCodec::kH264, late.data(), late.size(), &c));
    const auto late265 = AnnexB({kH265Idr, kH265Bt709});
    CHECK(!FindSpsColor(SpsCodec::kH265, late265.data(), late265.size(), &c));
    const auto plain = AnnexB({kH264Pps, kH264Idr});
    CHECK(!FindSpsColor(SpsCodec::kH264, plain.data(), plain.size(), &c));
    CHECK(c.matrix_coeffs == 9);
    CHECK(!FindSpsColor(SpsCodec::kH264, nullptr, 0, &c));
    CHECK(!FindSpsColor(SpsCodec::kH264, plain.data(), plain.size(), nullptr));
  }
  // The bit depths: 8 for the 8-bit SPSs above, 10 for libx264's High 10 and
  // libx265's Main 10 (64x64).
  {
    SpsColor c;
    const auto h264 = AnnexB({kH264Bt709, kH264Idr});
    CHECK(FindSpsColor(SpsCodec::kH264, h264.data(), h264.size(), &c));
    CHECK(c.bit_depth_luma == 8 && c.bit_depth_chroma == 8);
    const auto h265 = AnnexB({kH265Vps, kH265Bt709, kH265Idr});
    CHECK(FindSpsColor(SpsCodec::kH265, h265.data(), h265.size(), &c));
    CHECK(c.bit_depth_luma == 8 && c.bit_depth_chroma == 8);
    const std::vector<uint8_t> high10 = {
        0x67, 0x6e, 0x00, 0x0a, 0xa6, 0xcd, 0x94, 0x42, 0x6c, 0x05,
        0xa8, 0x08, 0x08, 0x0a, 0x00, 0x00, 0x03, 0x00, 0x02, 0x00,
        0x00, 0x03, 0x00, 0x28, 0x1e, 0x24, 0x4b, 0x2c};
    const auto deep264 = AnnexB({high10, kH264Idr});
    CHECK(FindSpsColor(SpsCodec::kH264, deep264.data(), deep264.size(), &c));
    CHECK(c.bit_depth_luma == 10 && c.bit_depth_chroma == 10);
    const std::vector<uint8_t> main10 = {
        0x42, 0x01, 0x01, 0x02, 0x20, 0x00, 0x00, 0x03, 0x00, 0x90, 0x00,
        0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x1e, 0xa0, 0x20, 0x81, 0x04,
        0xd9, 0x65, 0x66, 0x92, 0x4c, 0xaf, 0x01, 0x68, 0x08, 0x00, 0x00,
        0x03, 0x00, 0x08, 0x00, 0x00, 0x03, 0x00, 0x50, 0x40};
    const auto deep265 = AnnexB({kH265Vps, main10, kH265Idr});
    CHECK(FindSpsColor(SpsCodec::kH265, deep265.data(), deep265.size(), &c));
    CHECK(c.bit_depth_luma == 10 && c.bit_depth_chroma == 10);
  }
  // Every cut and every flipped byte stays inside the buffer (the sanitizer
  // build checks that); a cut H.265 SPS says nothing.
  {
    const auto h264 = AnnexB({kH264Bt601Full});
    const auto h265 = AnnexB({kH265Bt601Full});
    for (const auto* au : {&h264, &h265}) {
      const SpsCodec codec = (au == &h264) ? SpsCodec::kH264 : SpsCodec::kH265;
      for (size_t n = 0; n <= au->size(); ++n) {
        SpsColor c;
        FindSpsColor(codec, au->data(), n, &c);
      }
      for (size_t i = 4; i < au->size(); ++i) {
        std::vector<uint8_t> bad = *au;
        bad[i] ^= 0xff;
        SpsColor c;
        FindSpsColor(codec, bad.data(), bad.size(), &c);
      }
    }
    SpsColor c;
    CHECK(!FindSpsColor(SpsCodec::kH265, h265.data(), 20, &c));
  }

  // Where the picture sits: 640x360 at (16,16) in a 672x392 coded picture
  // (libx264 and libx265, then cropped 16 left and top with h264_metadata
  // and hevc_metadata).
  {
    const auto avc =
        AnnexB({{0x67, 0x64, 0x00, 0x1e, 0xac, 0xb2, 0x01, 0x50, 0x67,
                 0x89, 0x12, 0x24, 0x6e, 0x02, 0x20, 0x00, 0x00, 0x03,
                 0x00, 0x20, 0x00, 0x00, 0x07, 0x81, 0xe2, 0xc5, 0xc9}});
    SpsColor c;
    CHECK(FindSpsColor(SpsCodec::kH264, avc.data(), avc.size(), &c));
    CHECK(c.width == 640 && c.height == 360);
    CHECK(c.crop_left == 16 && c.crop_top == 16);
    const auto hevc =
        AnnexB({{0x42, 0x01, 0x01, 0x01, 0x60, 0x00, 0x00, 0x03, 0x00,
                 0x90, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x5a,
                 0xa0, 0x05, 0x42, 0x01, 0x89, 0x89, 0x12, 0x24, 0x4e,
                 0x59, 0x2a, 0x49, 0x32, 0xbc, 0x05, 0xa0, 0x20, 0x00,
                 0x00, 0x03, 0x00, 0x20, 0x00, 0x00, 0x03, 0x03, 0xc1}});
    CHECK(FindSpsColor(SpsCodec::kH265, hevc.data(), hevc.size(), &c));
    CHECK(c.width == 640 && c.height == 360);
    CHECK(c.crop_left == 16 && c.crop_top == 16);
  }

  if (g_failures == 0) {
    std::printf("SPS_COLOR_TEST_OK\n");
    return 0;
  }
  std::printf("SPS_COLOR_TEST_FAIL (%d)\n", g_failures);
  return 1;
}
