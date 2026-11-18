// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// OpenMAX IL video decoder.
//
// For a board whose hardware decoder is reachable only through the vendor's
// OpenMAX core, not through a V4L2 M2M node: a vendor kernel whose video
// device takes the vendor's ioctl dialect (Qualcomm's downstream msm_vidc is
// one). The core is loaded at run time and the decoder is found by its role
// ("video_decoder.avc"), so no vendor is named to select it.
//
// What is vendor-specific is where a decoded buffer's dma-buf fd is: the IL
// has no field for it, each vendor puts it in the buffer header's
// pPlatformPrivate in its own layout. The layouts this engine knows are in
// omx_decoder.cc; a component whose buffers it cannot get an fd for is
// refused at Create, since the seam's frames are dma-bufs and this library
// has no allocator to copy into.

#ifndef V4L2WC_SRC_OMX_DECODER_H_
#define V4L2WC_SRC_OMX_DECODER_H_

#include <cstdint>
#include <memory>

#include "src/dma_decoder.h"  // V4l2DmaFrame, SubmitResult, IDmaDecoder

namespace v4l2wc {

class OmxDecoder : public IDmaDecoder {
 public:
  // Loads `core_library` (null or empty: "libOmxCore.so"), finds a decoder
  // for `codec_fourcc` (a V4L2_PIX_FMT_*) by role, asks it for linear NV12 and
  // brings it to the executing state. `extra_output_buffers` is added to the
  // component's own minimum output count, for the frames a consumer holds.
  // Returns nullptr when there is no core, no decoder for the codec, no linear
  // NV12, or no dma-buf fd on its output buffers.
  static std::unique_ptr<OmxDecoder> Create(const char* core_library,
                                            std::uint32_t codec_fourcc,
                                            std::uint32_t coded_width,
                                            std::uint32_t coded_height,
                                            std::uint32_t extra_output_buffers);

  ~OmxDecoder() override;

  OmxDecoder(const OmxDecoder&) = delete;
  OmxDecoder& operator=(const OmxDecoder&) = delete;

  SubmitResult SubmitBitstream(const std::uint8_t* data, std::size_t size,
                               std::uint64_t timestamp) override;
  // Handles the component's events. The reconfiguration a component asks for
  // before its first frame (it has by then read the stream's real geometry) is
  // done here and is not reported. One after frames have been handed out is
  // kSourceChange: the consumer holds buffers the component wants freed.
  DriveResult Drive() override;
  bool Acquire(V4l2DmaFrame* out) override;
  void Release(std::uint32_t capture_index) override;
  void Flush() override;
  void Drain() override;
  std::uint32_t PoolSize() const override;

 private:
  OmxDecoder();
  struct State;
  std::unique_ptr<State> s_;
};

}  // namespace v4l2wc

#endif  // V4L2WC_SRC_OMX_DECODER_H_
