// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// Broadcom zero-copy display: decode an HEVC clip on the Pi's stateless
// decoder (rpi-hevc-dec), import each resulting SAND128-tiled NV12/P030
// dma-buf straight into a KMS framebuffer, and scan it out on an overlay plane
// over HDMI -- no CPU copy, no detile, the Tier-1 path. Proves the decoder's
// dma-buf and its DRM fourcc+modifier are accepted by the display hardware end
// to end, and puts moving video on the glass for someone to look at.
//
// The clip plays once at the given rate (the bitstream's own timing is not
// read) and the last frame is held for two seconds.

#include <fcntl.h>
#include <time.h>
#include <unistd.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "parse/h265/nal.h"
#include "src/v4l2_stateless_h265_decoder.h"

using namespace v4l2wc;

namespace {

#define LOGE(...) std::fprintf(stderr, __VA_ARGS__)

std::vector<std::uint8_t> ReadFile(const char* path) {
  std::vector<std::uint8_t> out;
  FILE* f = std::fopen(path, "rb");
  if (!f) return out;
  std::fseek(f, 0, SEEK_END);
  long n = std::ftell(f);
  std::fseek(f, 0, SEEK_SET);
  if (n > 0) {
    out.resize(static_cast<size_t>(n));
    if (std::fread(out.data(), 1, out.size(), f) != out.size()) out.clear();
  }
  std::fclose(f);
  return out;
}

// Access units split at the first non-VCL after a VCL, or a first-slice VCL.
std::vector<std::vector<std::uint8_t>> SplitAccessUnits(
    const std::vector<std::uint8_t>& data) {
  auto nals = h265::ParseAnnexB(data.data(), data.size());
  std::vector<std::vector<std::uint8_t>> aus;
  std::vector<std::uint8_t> cur;
  bool have_vcl = false;
  for (const auto& n : nals) {
    const bool vcl = h265::IsVcl(n.type);
    const bool first = vcl && !n.rbsp.empty() && (n.rbsp[0] & 0x80);
    if (have_vcl && (!vcl || first)) {
      aus.push_back(cur);
      cur.clear();
      have_vcl = false;
    }
    cur.insert(cur.end(), {0, 0, 1});
    cur.insert(cur.end(), n.raw.begin(), n.raw.end());
    if (vcl) have_vcl = true;
  }
  if (!cur.empty()) aus.push_back(cur);
  return aus;
}

// Picks a plane usable on `crtc_index` that lists `fourcc`, preferring an
// overlay. Returns the plane id or 0.
std::uint32_t PickPlane(int drm, std::uint32_t crtc_index,
                        std::uint32_t fourcc) {
  drmModePlaneRes* pr = drmModeGetPlaneResources(drm);
  if (!pr) return 0;
  std::uint32_t chosen = 0;
  bool chosen_overlay = false;
  for (std::uint32_t i = 0; i < pr->count_planes; ++i) {
    drmModePlane* pl = drmModeGetPlane(drm, pr->planes[i]);
    if (!pl) continue;
    if (pl->possible_crtcs & (1u << crtc_index)) {
      bool has_fmt = false;
      for (std::uint32_t f = 0; f < pl->count_formats; ++f)
        if (pl->formats[f] == fourcc) has_fmt = true;
      if (has_fmt) {
        // Read the plane type; prefer OVERLAY so the primary keeps the console.
        bool overlay = false;
        drmModeObjectProperties* props = drmModeObjectGetProperties(
            drm, pl->plane_id, DRM_MODE_OBJECT_PLANE);
        if (props) {
          for (std::uint32_t p = 0; p < props->count_props; ++p) {
            drmModePropertyRes* pp = drmModeGetProperty(drm, props->props[p]);
            if (pp && std::strcmp(pp->name, "type") == 0)
              overlay = props->prop_values[p] == DRM_PLANE_TYPE_OVERLAY;
            if (pp) drmModeFreeProperty(pp);
          }
          drmModeFreeObjectProperties(props);
        }
        if (chosen == 0 || (overlay && !chosen_overlay)) {
          chosen = pl->plane_id;
          chosen_overlay = overlay;
        }
      }
    }
    drmModeFreePlane(pl);
  }
  drmModeFreePlaneResources(pr);
  return chosen;
}

// Imports `frame` as a framebuffer with its fourcc and modifier. The GEM
// handles are closed again at once; the framebuffer keeps the buffers. Returns
// the framebuffer id or 0.
std::uint32_t ImportFrame(int drm, const V4l2DmaFrame& frame) {
  std::uint32_t handles[4] = {0, 0, 0, 0};
  std::uint32_t pitches[4] = {0, 0, 0, 0};
  std::uint32_t offsets[4] = {0, 0, 0, 0};
  std::uint64_t modifiers[4] = {0, 0, 0, 0};
  const std::uint32_t planes =
      frame.num_planes < 4 ? frame.num_planes : 4;  // the arrays above
  bool ok = true;
  for (std::uint32_t p = 0; p < planes && ok; ++p) {
    if (drmPrimeFDToHandle(drm, frame.fds[p], &handles[p]) != 0) {
      LOGE("drmPrimeFDToHandle plane %u failed\n", p);
      handles[p] = 0;
      ok = false;
      break;
    }
    pitches[p] = frame.pitches[p];
    offsets[p] = frame.offsets[p];
    modifiers[p] = frame.modifier;
  }
  std::uint32_t fb = 0;
  if (ok && drmModeAddFB2WithModifiers(
                drm, frame.width, frame.height, frame.drm_fourcc, handles,
                pitches, offsets, modifiers, &fb, DRM_MODE_FB_MODIFIERS) != 0) {
    LOGE("drmModeAddFB2WithModifiers failed: %s\n", std::strerror(errno));
    fb = 0;
  }
  // Two planes of one buffer come back as one handle; close it once.
  for (std::uint32_t p = 0; p < planes; ++p) {
    if (handles[p] == 0) continue;
    bool seen = false;
    for (std::uint32_t q = 0; q < p; ++q)
      seen = seen || handles[q] == handles[p];
    if (!seen) drmCloseBufferHandle(drm, handles[p]);
  }
  return fb;
}

// Feeds one access unit, pumping the engine while it has no free input
// buffer. False when the engine cannot go on.
bool Submit(V4l2StatelessH265Decoder* decoder,
            const std::vector<std::uint8_t>& au, std::uint64_t timestamp) {
  for (int tries = 0; tries < 100; ++tries) {
    const SubmitResult r =
        decoder->SubmitBitstream(au.data(), au.size(), timestamp);
    if (r == SubmitResult::kOk) return true;
    if (r != SubmitResult::kTryAgain) return false;
    if (decoder->Drive() == DriveResult::kError) return false;
    ::usleep(1000);
  }
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    LOGE("usage: %s <clip.hevc> [video_node] [card] [fps]\n", argv[0]);
    return 2;
  }
  const char* node = argc > 2 ? argv[2] : "/dev/video19";
  const char* card = argc > 3 ? argv[3] : "/dev/dri/card0";
  const long fps = argc > 4 ? std::strtol(argv[4], nullptr, 10) : 30;
  if (fps < 1 || fps > 240) {
    LOGE("fps must be 1 to 240\n");
    return 2;
  }

  // Decode the first frame before touching KMS: its fourcc picks the plane.
  // A frame on a plane is kept (not Released) so its dma-buf stays valid while
  // it is scanned out.
  std::vector<std::uint8_t> data = ReadFile(argv[1]);
  if (data.empty()) {
    LOGE("cannot read %s\n", argv[1]);
    return 1;
  }
  auto decoder = V4l2StatelessH265Decoder::Create(node, 16);
  if (!decoder) {
    LOGE("decoder create failed on %s\n", node);
    return 1;
  }
  V4l2DmaFrame frame;
  bool got = false;
  std::uint64_t ts = 0;
  const std::vector<std::vector<std::uint8_t>> aus = SplitAccessUnits(data);
  std::size_t next = 0;
  while (next < aus.size() && !got) {
    if (!Submit(decoder.get(), aus[next++], ts++)) break;
    got = decoder->Acquire(&frame);
  }
  if (!got) {
    LOGE("no frame decoded\n");
    return 1;
  }
  LOGE("decoded frame: %ux%u fourcc=%c%c%c%c modifier=0x%llx planes=%u\n",
       frame.width, frame.height, frame.drm_fourcc & 0xff,
       (frame.drm_fourcc >> 8) & 0xff, (frame.drm_fourcc >> 16) & 0xff,
       (frame.drm_fourcc >> 24) & 0xff,
       static_cast<unsigned long long>(frame.modifier), frame.num_planes);

  // --- KMS ---
  int drm = ::open(card, O_RDWR | O_CLOEXEC);
  if (drm < 0) {
    LOGE("open %s failed\n", card);
    return 1;
  }
  if (drmSetMaster(drm) != 0) LOGE("drmSetMaster failed (continuing)\n");

  drmModeRes* res = drmModeGetResources(drm);
  if (!res) {
    LOGE("drmModeGetResources failed\n");
    return 1;
  }
  drmModeConnector* conn = nullptr;
  for (int i = 0; i < res->count_connectors; ++i) {
    drmModeConnector* c = drmModeGetConnector(drm, res->connectors[i]);
    if (c && c->connection == DRM_MODE_CONNECTED && c->count_modes > 0) {
      conn = c;
      break;
    }
    if (c) drmModeFreeConnector(c);
  }
  if (!conn) {
    LOGE("no connected connector\n");
    return 1;
  }
  drmModeModeInfo mode = conn->modes[0];
  LOGE("connector %u: %s %ux%u@%u\n", conn->connector_id, conn->modes[0].name,
       mode.hdisplay, mode.vdisplay, mode.vrefresh);

  // Find a CRTC via the connector's encoder.
  drmModeEncoder* enc = drmModeGetEncoder(drm, conn->encoder_id);
  std::uint32_t crtc_id = enc ? enc->crtc_id : 0;
  std::uint32_t crtc_index = 0;
  if (crtc_id == 0 && enc) {
    for (int i = 0; i < res->count_crtcs; ++i)
      if (enc->possible_crtcs & (1u << i)) {
        crtc_id = res->crtcs[i];
        break;
      }
  }
  for (int i = 0; i < res->count_crtcs; ++i)
    if (res->crtcs[i] == crtc_id) crtc_index = static_cast<std::uint32_t>(i);
  if (enc) drmModeFreeEncoder(enc);
  if (crtc_id == 0) {
    LOGE("no CRTC\n");
    return 1;
  }

  // A dumb background so the CRTC has a mode set and the display is lit.
  drm_mode_create_dumb creq{};
  creq.width = mode.hdisplay;
  creq.height = mode.vdisplay;
  creq.bpp = 32;
  drmIoctl(drm, DRM_IOCTL_MODE_CREATE_DUMB, &creq);
  std::uint32_t bg_fb = 0;
  drmModeAddFB(drm, mode.hdisplay, mode.vdisplay, 24, 32, creq.pitch,
               creq.handle, &bg_fb);
  if (drmModeSetCrtc(drm, crtc_id, bg_fb, 0, 0, &conn->connector_id, 1,
                     &mode) != 0) {
    LOGE("drmModeSetCrtc failed: %s\n", std::strerror(errno));
    return 1;
  }
  LOGE("mode set on crtc %u (bg fb %u)\n", crtc_id, bg_fb);

  // Import the decoded dma-buf as a framebuffer with its fourcc + modifier.
  std::uint32_t video_fb = ImportFrame(drm, frame);
  if (video_fb == 0) return 1;
  LOGE("imported decoded dma-buf as fb %u (fourcc+modifier accepted)\n",
       video_fb);

  std::uint32_t plane = PickPlane(drm, crtc_index, frame.drm_fourcc);
  if (plane == 0) {
    LOGE("no plane supports the decoded fourcc on this crtc\n");
    return 1;
  }
  // Scale the decoded frame to fill the output.
  if (drmModeSetPlane(drm, plane, crtc_id, video_fb, 0, 0, 0, mode.hdisplay,
                      mode.vdisplay, 0, 0, frame.width << 16,
                      frame.height << 16) != 0) {
    LOGE("drmModeSetPlane failed: %s\n", std::strerror(errno));
    return 1;
  }
  LOGE("=== SCANOUT: decoded SAND frame on plane %u, %ux%u -> %ux%u ===\n",
       plane, frame.width, frame.height, mode.hdisplay, mode.vdisplay);

  // The rest of the clip, one frame per period. A frame leaves the plane
  // before its framebuffer goes and its buffer returns to the decoder.
  const long period_ns = 1000000000L / fps;
  timespec start{};
  ::clock_gettime(CLOCK_MONOTONIC, &start);
  timespec due = start;
  unsigned shown = 1;
  bool failed = false;
  auto show = [&](const V4l2DmaFrame& f) {
    due.tv_nsec += period_ns;
    if (due.tv_nsec >= 1000000000L) {
      due.tv_nsec -= 1000000000L;
      ++due.tv_sec;
    }
    timespec now{};
    ::clock_gettime(CLOCK_MONOTONIC, &now);
    if (now.tv_sec > due.tv_sec ||
        (now.tv_sec == due.tv_sec && now.tv_nsec > due.tv_nsec)) {
      due = now;  // late: do not try to catch up
    } else {
      ::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &due, nullptr);
    }
    const std::uint32_t fb = ImportFrame(drm, f);
    if (fb == 0 || drmModeSetPlane(drm, plane, crtc_id, fb, 0, 0, 0,
                                   mode.hdisplay, mode.vdisplay, 0, 0,
                                   f.width << 16, f.height << 16) != 0) {
      LOGE("frame %u: %s\n", shown,
           fb == 0 ? "import failed" : std::strerror(errno));
      if (fb != 0) drmModeRmFB(drm, fb);
      decoder->Release(f.capture_index);
      failed = true;
      return;
    }
    drmModeRmFB(drm, video_fb);
    decoder->Release(frame.capture_index);
    frame = f;
    video_fb = fb;
    ++shown;
  };
  V4l2DmaFrame f;
  while (next < aus.size() && !failed) {
    if (!Submit(decoder.get(), aus[next++], ts++)) {
      LOGE("decoder stopped at access unit %zu\n", next);
      failed = true;
      break;
    }
    while (!failed && decoder->Acquire(&f)) show(f);
  }
  if (!failed) {
    decoder->Drain();
    decoder->Drive();
    while (!failed && decoder->Acquire(&f)) show(f);
  }
  timespec end{};
  ::clock_gettime(CLOCK_MONOTONIC, &end);
  const double secs = static_cast<double>(end.tv_sec - start.tv_sec) +
                      static_cast<double>(end.tv_nsec - start.tv_nsec) / 1e9;
  LOGE("showed %u frames of %zu access units in %.2f s\n", shown, aus.size(),
       secs);
  if (!failed) ::sleep(2);  // hold the last frame

  drmModeRmFB(drm, video_fb);
  drmModeRmFB(drm, bg_fb);
  drmDropMaster(drm);
  drmModeFreeConnector(conn);
  drmModeFreeResources(res);
  ::close(drm);
  if (failed) return 1;
  LOGE("done (zero-copy Tier-1 scanout succeeded)\n");
  return 0;
}
