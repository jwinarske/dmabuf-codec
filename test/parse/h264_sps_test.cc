// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Unit test for the H.264 SPS parser. Runs on the host (no webrtc / V4L2).

#include <cstdint>
#include <cstdio>
#include <vector>

#include "parse/h264/nal.h"
#include "parse/h264/sps.h"

using v4l2wc::h264::NalUnitType;
using v4l2wc::h264::ParseAnnexB;
using v4l2wc::h264::ParseSps;
using v4l2wc::h264::Sps;

static int g_failures = 0;

#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                               \
    }                                                             \
  } while (0)

int main() {
  // Baseline (profile 66) SPS for 176x144, hand-encoded and verified bit by
  // bit: sps_id=0, poc_type=0, one ref, 11 mbs wide (ue 10), 9 map units tall
  // (ue 8), frame_mbs_only=1, no cropping, no VUI, then rbsp trailing bits.
  const std::vector<uint8_t> rbsp = {0x42, 0x00, 0x0a, 0xf8, 0x58, 0x98, 0x80};
  {
    Sps sps;
    CHECK(ParseSps(rbsp.data(), rbsp.size(), &sps));
    CHECK(sps.profile_idc == 66);
    CHECK(sps.level_idc == 10);
    CHECK(sps.width == 176);
    CHECK(sps.height == 144);
    CHECK(sps.frame_mbs_only_flag);
  }

  // Integration: extract the SPS NAL from an Annex-B stream, then parse it.
  {
    std::vector<uint8_t> stream = {0x00, 0x00, 0x00, 0x01, 0x67};
    stream.insert(stream.end(), rbsp.begin(), rbsp.end());
    auto nals = ParseAnnexB(stream.data(), stream.size());
    CHECK(nals.size() == 1);
    if (nals.size() == 1) {
      CHECK(nals[0].type == NalUnitType::kSps);
      Sps sps;
      CHECK(ParseSps(nals[0].rbsp.data(), nals[0].rbsp.size(), &sps));
      CHECK(sps.width == 176 && sps.height == 144);
    }
  }

  // Real SPSs from libx264 (High, 64x64) with a VUI color description. Fed as
  // Annex B so the emulation-prevention bytes are removed as in a decoder.
  const auto parse_nal = [](std::vector<uint8_t> nal, size_t keep, Sps* sps) {
    std::vector<uint8_t> stream = {0x00, 0x00, 0x00, 0x01};
    stream.insert(stream.end(), nal.begin(), nal.begin() + keep);
    auto nals = ParseAnnexB(stream.data(), stream.size());
    return nals.size() == 1 &&
           ParseSps(nals[0].rbsp.data(), nals[0].rbsp.size(), sps);
  };
  // colorprim/transfer/colormatrix=bt709, limited range.
  const std::vector<uint8_t> bt709 = {0x67, 0x64, 0x00, 0x0a, 0xac, 0xd9, 0x44,
                                      0x26, 0xc0, 0x5a, 0x80, 0x80, 0x80, 0xa0,
                                      0x00, 0x00, 0x03, 0x00, 0x20, 0x00, 0x00,
                                      0x06, 0x41, 0xe2, 0x44, 0xb2, 0xc0};
  {
    Sps sps;
    CHECK(parse_nal(bt709, bt709.size(), &sps));
    CHECK(sps.width == 64 && sps.height == 64);
    CHECK(sps.colour_description_present_flag);
    CHECK(sps.colour_primaries == 1);
    CHECK(sps.transfer_characteristics == 1);
    CHECK(sps.matrix_coeffs == 1);
    CHECK(!sps.video_full_range_flag);
  }
  // colorprim/transfer/colormatrix=bt470bg, full range.
  {
    std::vector<uint8_t> bt601 = bt709;
    bt601[9] = 0x5b;
    bt601[10] = bt601[11] = bt601[12] = 0x82;
    Sps sps;
    CHECK(parse_nal(bt601, bt601.size(), &sps));
    CHECK(sps.width == 64 && sps.height == 64);
    CHECK(sps.colour_primaries == 5);
    CHECK(sps.transfer_characteristics == 5);
    CHECK(sps.matrix_coeffs == 5);
    CHECK(sps.video_full_range_flag);
  }
  // A VUI cut off inside the color description keeps the geometry and leaves
  // the color unspecified, never half written.
  {
    Sps sps;
    CHECK(parse_nal(bt709, 11, &sps));
    CHECK(sps.width == 64 && sps.height == 64);
    CHECK(!sps.colour_description_present_flag);
    CHECK(sps.matrix_coeffs == 2);
    CHECK(!sps.video_full_range_flag);
  }
  // No VUI: unspecified.
  {
    Sps sps;
    CHECK(ParseSps(rbsp.data(), rbsp.size(), &sps));
    CHECK(!sps.colour_description_present_flag);
    CHECK(sps.matrix_coeffs == 2);
  }

  // Malformed / truncated input must fail cleanly, never crash.
  {
    Sps sps;
    CHECK(!ParseSps(nullptr, 0, &sps));
    CHECK(!ParseSps(rbsp.data(), 0, &sps));
    CHECK(!ParseSps(rbsp.data(), 2, &sps));  // truncated before dimensions
    const std::vector<uint8_t> zeros(8, 0x00);
    CHECK(!ParseSps(zeros.data(), zeros.size(), &sps));  // ue never terminates
  }

  if (g_failures == 0) {
    std::printf("H264_SPS_TEST_OK\n");
    return 0;
  }
  std::printf("H264_SPS_TEST_FAIL (%d)\n", g_failures);
  return 1;
}
