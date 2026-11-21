// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// On-hardware test of a decoded frame as a display controller reads it. The
// decode test next to this one counts frames; it cannot tell a frame whose
// planes are described wrongly (a pitch, an offset, a modifier) from one
// described right, since nothing there reads the samples. Here KMS does: the
// frame goes on an overlay plane exactly as the engine gave it, a writeback
// connector hands back what the controller composed, and that is compared
// with a software decode of the same clip.
//
// No display and no compositor: it needs a card with a writeback connector
// (a Pi's vc4 has one) that no other process is master of, a plane on it that
// takes the frame's format, and a decoder. Run with weston and the like
// stopped, or on a card they do not hold.
//
// The clip is as for dma_decode_hw_test: Annex B, an access unit delimiter
// before each frame, no reordered frames. The reference is the same clip
// decoded in software:
//   ffmpeg -i clip.h265 -f rawvideo -pix_fmt yuv420p ref.yuv
//
// The controller's conversion to RGB is not bit-exact (its chroma filter and
// rounding are its own), so the result is two mean absolute differences in
// 8-bit levels: luma per sample, and chroma per 2x2 block against the
// reference's samples around it. On a Pi 5 a frame described right is within
// 1.2 and 0.5; one with a pitch, an offset or the modifier wrong is 3.7 to 74
// and 3.2 to 72. Use a clip with detail in it: a flat picture reads the same
// however its planes are laid out.
//
// Usage: kms_writeback_hw_test h264|hevc clip ref.yuv WIDTHxHEIGHT
//        [--frame N] [--card node] [--one-buffer] [--v4l2 node] [--hevc node]
//        [--sand] [--ppm file]
//   --frame N   check only frame N (from 1); default the first, the middle
//               and the last
//   --sand      H.264 only: the V4L2 stateful engine with its SAND capture
//               format (NC12, 128-px columns) rather than the factory's
//               choice, which there is linear NV12
//   --ppm file  write the last capture there, to look at

#include <drm_fourcc.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/mman.h>
#include <unistd.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "src/dma_decoder_factory.h"
#include "src/v4l2_m2m_decoder.h"

// The Raspberry Pi kernel's 128-px column format; not in every uapi header.
#ifndef V4L2_PIX_FMT_NV12_COL128
#define V4L2_PIX_FMT_NV12_COL128 v4l2_fourcc('N', 'C', '1', '2')
#endif

namespace {

int g_failures = 0;
#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                               \
    }                                                             \
  } while (0)

// Mean absolute differences a right frame stays under, in 8-bit levels,
// between what was measured for right frames and for wrong ones (above).
constexpr double kMaxLumaError = 2.5;
constexpr double kMaxChromaError = 2.0;

// The background the frame is composed over, as B, G, R.
constexpr std::uint8_t kBackground[3] = {0x60, 0x40, 0x20};

struct Stream {
  std::vector<std::uint8_t> bytes;
  std::vector<std::size_t> units;  // start of each access unit, then the end
};

bool ReadFile(const char* path, std::vector<std::uint8_t>* out) {
  FILE* f = std::fopen(path, "rb");
  if (f == nullptr) {
    return false;
  }
  std::uint8_t buf[65536];
  // A short read is the end (or an error): no read after it.
  for (;;) {
    const std::size_t n = std::fread(buf, 1, sizeof buf, f);
    out->insert(out->end(), buf, buf + n);
    if (n < sizeof buf) {
      break;
    }
  }
  std::fclose(f);
  return true;
}

bool Load(const char* path, bool hevc, Stream* s) {
  if (!ReadFile(path, &s->bytes)) {
    return false;
  }
  const auto& d = s->bytes;
  s->units.push_back(0);
  bool first = true;
  for (std::size_t i = 0; i + 3 < d.size(); ++i) {
    if (d[i] != 0 || d[i + 1] != 0 || d[i + 2] != 1) {
      continue;
    }
    const std::uint8_t b = d[i + 3];
    if (hevc ? ((b >> 1) & 0x3f) == 35 : (b & 0x1f) == 9) {
      const std::size_t start = (i > 0 && d[i - 1] == 0) ? i - 1 : i;
      if (!first) {
        s->units.push_back(start);
      }
      first = false;
    }
  }
  s->units.push_back(d.size());
  return s->units.size() > 2;
}

// ---- KMS -------------------------------------------------------------------

struct Property {
  std::uint32_t id = 0;
  std::uint64_t value = 0;
};

Property FindProperty(int fd, std::uint32_t object, std::uint32_t type,
                      const char* name) {
  Property found;
  drmModeObjectProperties* props = drmModeObjectGetProperties(fd, object, type);
  for (std::uint32_t i = 0; props != nullptr && i < props->count_props; ++i) {
    drmModePropertyRes* p = drmModeGetProperty(fd, props->props[i]);
    if (p != nullptr && std::strcmp(p->name, name) == 0) {
      found.id = p->prop_id;
      found.value = props->prop_values[i];
    }
    drmModeFreeProperty(p);
  }
  drmModeFreeObjectProperties(props);
  return found;
}

struct DumbBuffer {
  std::uint32_t handle = 0, fb = 0, pitch = 0;
  std::uint64_t size = 0;
  std::uint8_t* map = nullptr;
};

bool MakeDumb(int fd, std::uint32_t width, std::uint32_t height,
              DumbBuffer* d) {
  drm_mode_create_dumb create{};
  create.width = width;
  create.height = height;
  create.bpp = 32;
  if (drmIoctl(fd, DRM_IOCTL_MODE_CREATE_DUMB, &create) != 0) {
    return false;
  }
  d->handle = create.handle;
  d->pitch = create.pitch;
  d->size = create.size;
  const std::uint32_t handles[4] = {create.handle};
  const std::uint32_t pitches[4] = {create.pitch};
  const std::uint32_t offsets[4] = {};
  if (drmModeAddFB2(fd, width, height, DRM_FORMAT_XRGB8888, handles, pitches,
                    offsets, &d->fb, 0) != 0) {
    return false;
  }
  drm_mode_map_dumb map{};
  map.handle = create.handle;
  if (drmIoctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &map) != 0) {
    return false;
  }
  void* p = mmap(nullptr, create.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                 static_cast<off_t>(map.offset));
  if (p == MAP_FAILED) {
    return false;
  }
  d->map = static_cast<std::uint8_t*>(p);
  return true;
}

void FreeDumb(int fd, DumbBuffer* d) {
  if (d->map != nullptr) {
    munmap(d->map, d->size);
  }
  if (d->fb != 0) {
    drmModeRmFB(fd, d->fb);
  }
  if (d->handle != 0) {
    drm_mode_destroy_dumb destroy{};
    destroy.handle = d->handle;
    drmIoctl(fd, DRM_IOCTL_MODE_DESTROY_DUMB, &destroy);
  }
  *d = DumbBuffer{};
}

// A writeback connector, a CRTC it can be driven from, and that CRTC's
// primary plane and one overlay plane.
struct Output {
  int fd = -1;
  std::uint32_t connector = 0, crtc = 0, primary = 0, overlay = 0;
  std::uint32_t width = 0, height = 0;
  drmModeModeInfo mode{};
  std::uint32_t mode_blob = 0;
  DumbBuffer background, capture;
};

bool WritesXrgb(int fd, std::uint32_t connector) {
  const Property formats = FindProperty(
      fd, connector, DRM_MODE_OBJECT_CONNECTOR, "WRITEBACK_PIXEL_FORMATS");
  drmModePropertyBlobRes* blob =
      drmModeGetPropertyBlob(fd, static_cast<std::uint32_t>(formats.value));
  bool found = false;
  if (blob != nullptr) {
    const auto* f = static_cast<const std::uint32_t*>(blob->data);
    for (std::uint32_t i = 0; i < blob->length / sizeof(std::uint32_t); ++i) {
      found = found || f[i] == DRM_FORMAT_XRGB8888;
    }
    drmModeFreePropertyBlob(blob);
  }
  return found;
}

// Opens `node` and finds the objects on it. Leaves out->overlay for
// FindOverlay, which needs the frame's format.
bool OpenOutput(const char* node, Output* out) {
  const int fd = open(node, O_RDWR | O_CLOEXEC);
  if (fd < 0) {
    return false;
  }
  drmModeRes* res = nullptr;
  int crtc_index = -1;
  if (drmSetClientCap(fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1) == 0 &&
      drmSetClientCap(fd, DRM_CLIENT_CAP_ATOMIC, 1) == 0 &&
      drmSetClientCap(fd, DRM_CLIENT_CAP_WRITEBACK_CONNECTORS, 1) == 0) {
    res = drmModeGetResources(fd);
  }
  for (int i = 0; res != nullptr && i < res->count_connectors; ++i) {
    drmModeConnector* c = drmModeGetConnector(fd, res->connectors[i]);
    if (c == nullptr) {
      continue;
    }
    if (out->connector == 0 &&
        c->connector_type == DRM_MODE_CONNECTOR_WRITEBACK &&
        WritesXrgb(fd, c->connector_id)) {
      for (int e = 0; e < c->count_encoders && crtc_index < 0; ++e) {
        drmModeEncoder* enc = drmModeGetEncoder(fd, c->encoders[e]);
        for (int k = 0; enc != nullptr && k < res->count_crtcs; ++k) {
          if ((enc->possible_crtcs & (1U << k)) != 0) {
            crtc_index = k;
            break;
          }
        }
        drmModeFreeEncoder(enc);
      }
      if (crtc_index >= 0) {
        out->connector = c->connector_id;
        out->crtc = res->crtcs[crtc_index];
      }
    }
    drmModeFreeConnector(c);
  }
  drmModeFreeResources(res);
  if (out->connector == 0) {
    close(fd);
    return false;
  }
  out->fd = fd;
  return true;
}

// Whether a plane's IN_FORMATS blob pairs `fourcc` with `modifier`. Read by
// hand: the libdrm iterator for it is newer than some sysroots.
bool ListsFormat(const drmModePropertyBlobRes* blob, std::uint32_t fourcc,
                 std::uint64_t modifier) {
  if (blob->length < sizeof(drm_format_modifier_blob)) {
    return false;
  }
  const auto* base = static_cast<const std::uint8_t*>(blob->data);
  drm_format_modifier_blob head;
  std::memcpy(&head, base, sizeof head);
  if (head.formats_offset > blob->length ||
      head.count_formats >
          (blob->length - head.formats_offset) / sizeof(std::uint32_t) ||
      head.modifiers_offset > blob->length ||
      head.count_modifiers > (blob->length - head.modifiers_offset) /
                                 sizeof(drm_format_modifier)) {
    return false;
  }
  for (std::uint32_t i = 0; i < head.count_modifiers; ++i) {
    drm_format_modifier m;
    std::memcpy(&m, base + head.modifiers_offset + i * sizeof m, sizeof m);
    if (m.modifier != modifier) {
      continue;
    }
    for (std::uint32_t bit = 0; bit < 64; ++bit) {
      const std::uint64_t index = static_cast<std::uint64_t>(m.offset) + bit;
      std::uint32_t format;
      if ((m.formats & (1ULL << bit)) == 0 || index >= head.count_formats) {
        continue;
      }
      std::memcpy(&format, base + head.formats_offset + index * sizeof format,
                  sizeof format);
      if (format == fourcc) {
        return true;
      }
    }
  }
  return false;
}

// The CRTC's primary plane, and an overlay plane of it that lists `fourcc`
// with `modifier`.
bool FindPlanes(Output* out, std::uint32_t fourcc, std::uint64_t modifier) {
  const int fd = out->fd;
  drmModeRes* res = drmModeGetResources(fd);
  int crtc_index = -1;
  for (int i = 0; res != nullptr && i < res->count_crtcs; ++i) {
    if (res->crtcs[i] == out->crtc) {
      crtc_index = i;
    }
  }
  drmModeFreeResources(res);
  drmModePlaneRes* planes = drmModeGetPlaneResources(fd);
  for (std::uint32_t i = 0; planes != nullptr && i < planes->count_planes;
       ++i) {
    drmModePlane* p = drmModeGetPlane(fd, planes->planes[i]);
    if (p == nullptr) {
      continue;
    }
    const std::uint32_t id = p->plane_id;
    const bool mine =
        crtc_index >= 0 && (p->possible_crtcs & (1U << crtc_index)) != 0;
    drmModeFreePlane(p);
    if (!mine) {
      continue;
    }
    const std::uint64_t type =
        FindProperty(fd, id, DRM_MODE_OBJECT_PLANE, "type").value;
    if (type == DRM_PLANE_TYPE_PRIMARY && out->primary == 0) {
      out->primary = id;
    }
    if (type != DRM_PLANE_TYPE_OVERLAY || out->overlay != 0) {
      continue;
    }
    const Property in_formats =
        FindProperty(fd, id, DRM_MODE_OBJECT_PLANE, "IN_FORMATS");
    drmModePropertyBlobRes* blob = drmModeGetPropertyBlob(
        fd, static_cast<std::uint32_t>(in_formats.value));
    if (blob != nullptr) {
      if (ListsFormat(blob, fourcc, modifier)) {
        out->overlay = id;
      }
      drmModeFreePropertyBlob(blob);
    }
  }
  drmModeFreePlaneResources(planes);
  return out->primary != 0 && out->overlay != 0;
}

// CEA-861 720p60 or 1080p60, whichever the frame fits.
bool PickMode(std::uint32_t width, std::uint32_t height, drmModeModeInfo* m) {
  if (width <= 1280 && height <= 720) {
    m->clock = 74250;
    m->hdisplay = 1280;
    m->hsync_start = 1390;
    m->hsync_end = 1430;
    m->htotal = 1650;
    m->vdisplay = 720;
    m->vsync_start = 725;
    m->vsync_end = 730;
    m->vtotal = 750;
  } else if (width <= 1920 && height <= 1080) {
    m->clock = 148500;
    m->hdisplay = 1920;
    m->hsync_start = 2008;
    m->hsync_end = 2052;
    m->htotal = 2200;
    m->vdisplay = 1080;
    m->vsync_start = 1084;
    m->vsync_end = 1089;
    m->vtotal = 1125;
  } else {
    return false;
  }
  m->vrefresh = 60;
  m->flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC;
  std::snprintf(m->name, sizeof m->name, "%ux%u", m->hdisplay, m->vdisplay);
  return true;
}

void Add(drmModeAtomicReq* req, int fd, std::uint32_t object,
         std::uint32_t type, const char* name, std::uint64_t value) {
  const Property p = FindProperty(fd, object, type, name);
  if (p.id == 0 || drmModeAtomicAddProperty(req, object, p.id, value) < 0) {
    std::printf("no property %s on object %u\n", name, object);
    ++g_failures;
  }
}

// Shows the width x height rectangle of `fb` that starts at (src_x, src_y) at
// the top left of the output.
void AddPlane(drmModeAtomicReq* req, const Output& out, std::uint32_t plane,
              std::uint32_t fb, std::uint32_t width, std::uint32_t height,
              std::uint32_t src_x = 0, std::uint32_t src_y = 0) {
  const auto P = DRM_MODE_OBJECT_PLANE;
  Add(req, out.fd, plane, P, "FB_ID", fb);
  Add(req, out.fd, plane, P, "CRTC_ID", out.crtc);
  Add(req, out.fd, plane, P, "SRC_X", static_cast<std::uint64_t>(src_x) << 16);
  Add(req, out.fd, plane, P, "SRC_Y", static_cast<std::uint64_t>(src_y) << 16);
  Add(req, out.fd, plane, P, "SRC_W", static_cast<std::uint64_t>(width) << 16);
  Add(req, out.fd, plane, P, "SRC_H", static_cast<std::uint64_t>(height) << 16);
  Add(req, out.fd, plane, P, "CRTC_X", 0);
  Add(req, out.fd, plane, P, "CRTC_Y", 0);
  Add(req, out.fd, plane, P, "CRTC_W", width);
  Add(req, out.fd, plane, P, "CRTC_H", height);
}

void TurnOff(const Output& out) {
  drmModeAtomicReq* req = drmModeAtomicAlloc();
  if (req == nullptr) {
    return;
  }
  const int failures = g_failures;
  Add(req, out.fd, out.connector, DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID", 0);
  Add(req, out.fd, out.crtc, DRM_MODE_OBJECT_CRTC, "MODE_ID", 0);
  Add(req, out.fd, out.crtc, DRM_MODE_OBJECT_CRTC, "ACTIVE", 0);
  for (std::uint32_t plane : {out.primary, out.overlay}) {
    Add(req, out.fd, plane, DRM_MODE_OBJECT_PLANE, "FB_ID", 0);
    Add(req, out.fd, plane, DRM_MODE_OBJECT_PLANE, "CRTC_ID", 0);
  }
  drmModeAtomicCommit(out.fd, req, DRM_MODE_ATOMIC_ALLOW_MODESET, nullptr);
  drmModeAtomicFree(req);
  g_failures = failures;  // turning off is not what is under test
}

// ---- the comparison --------------------------------------------------------

// The luma weights of red and blue for a plane's COLOR_ENCODING value
// (BT.601, BT.709, BT.2020, in the order the kernel numbers them).
struct Matrix {
  double kr, kb;
  bool full_range;
};

Matrix MatrixFor(std::uint64_t encoding, std::uint64_t range) {
  const double kr[] = {0.299, 0.2126, 0.2627};
  const double kb[] = {0.114, 0.0722, 0.0593};
  const std::size_t e = encoding < 3 ? static_cast<std::size_t>(encoding) : 0;
  return {kr[e], kb[e], range == 1};
}

// R, G, B (0..255) to Y, Cb, Cr in the 8-bit levels of the stream.
void ToYCbCr(const Matrix& m, double r, double g, double b, double* y,
             double* cb, double* cr) {
  const double kg = 1.0 - m.kr - m.kb;
  const double luma = (m.kr * r + kg * g + m.kb * b) / 255.0;
  const double pb = (b / 255.0 - luma) / (2.0 * (1.0 - m.kb));
  const double pr = (r / 255.0 - luma) / (2.0 * (1.0 - m.kr));
  if (m.full_range) {
    *y = 255.0 * luma;
    *cb = 128.0 + 255.0 * pb;
    *cr = 128.0 + 255.0 * pr;
  } else {
    *y = 16.0 + 219.0 * luma;
    *cb = 128.0 + 224.0 * pb;
    *cr = 128.0 + 224.0 * pr;
  }
}

struct Errors {
  double luma = 0, chroma = 0;
  std::size_t background_off = 0;  // pixels outside the frame not as painted
};

// How far `value` is outside the range of the 3x3 samples of `plane` around
// (x, y); 0 when within it.
double OutsideNeighbors(const std::uint8_t* plane, std::uint32_t width,
                        std::uint32_t height, std::uint32_t x, std::uint32_t y,
                        double value) {
  std::uint8_t lo = 255, hi = 0;
  for (std::uint32_t ny = y > 0 ? y - 1 : 0; ny <= y + 1 && ny < height; ++ny) {
    for (std::uint32_t nx = x > 0 ? x - 1 : 0; nx <= x + 1 && nx < width;
         ++nx) {
      const std::uint8_t v = plane[static_cast<std::size_t>(ny) * width + nx];
      lo = v < lo ? v : lo;
      hi = v > hi ? v : hi;
    }
  }
  return value < lo ? lo - value : (value > hi ? value - hi : 0.0);
}

// Compares the capture with one yuv420p reference frame. A reference level
// the controller's RGB cannot hold (one outside the nominal range) is
// clamped to what it can.
Errors Compare(const Output& out, const Matrix& m, const std::uint8_t* ref,
               std::uint32_t width, std::uint32_t height) {
  Errors e;
  const std::uint8_t* ref_cb = ref + static_cast<std::size_t>(width) * height;
  const std::uint32_t cw = (width + 1) / 2, ch = (height + 1) / 2;
  const std::uint8_t* ref_cr = ref_cb + static_cast<std::size_t>(cw) * ch;
  const double y_lo = m.full_range ? 0 : 16, y_hi = m.full_range ? 255 : 235;
  double luma_sum = 0, chroma_sum = 0;
  for (std::uint32_t by = 0; by < ch; ++by) {
    for (std::uint32_t bx = 0; bx < cw; ++bx) {
      double r = 0, g = 0, b = 0;
      int n = 0;
      for (std::uint32_t y = 2 * by; y < 2 * by + 2 && y < height; ++y) {
        for (std::uint32_t x = 2 * bx; x < 2 * bx + 2 && x < width; ++x) {
          const std::uint8_t* px =
              out.capture.map +
              static_cast<std::size_t>(y) * out.capture.pitch +
              static_cast<std::size_t>(x) * 4;
          double py, pcb, pcr;
          ToYCbCr(m, px[2], px[1], px[0], &py, &pcb, &pcr);
          double want = ref[static_cast<std::size_t>(y) * width + x];
          want = want < y_lo ? y_lo : (want > y_hi ? y_hi : want);
          luma_sum += std::fabs(py - want);
          r += px[2];
          g += px[1];
          b += px[0];
          ++n;
        }
      }
      double py, pcb, pcr;
      ToYCbCr(m, r / n, g / n, b / n, &py, &pcb, &pcr);
      // Against the reference's samples around this one, not it alone: the
      // controller filters chroma up to the picture's size, so at an edge a
      // block is somewhere between its neighbors. That is not what is being
      // looked for and would count for more the smaller the picture.
      chroma_sum += OutsideNeighbors(ref_cb, cw, ch, bx, by, pcb) +
                    OutsideNeighbors(ref_cr, cw, ch, bx, by, pcr);
    }
  }
  e.luma = luma_sum / (static_cast<double>(width) * height);
  e.chroma = chroma_sum / (2.0 * cw * ch);
  for (std::uint32_t y = 0; y < out.height; ++y) {
    for (std::uint32_t x = y < height ? width : 0; x < out.width; ++x) {
      const std::uint8_t* px = out.capture.map +
                               static_cast<std::size_t>(y) * out.capture.pitch +
                               static_cast<std::size_t>(x) * 4;
      if (std::memcmp(px, kBackground, 3) != 0) {
        ++e.background_off;
      }
    }
  }
  return e;
}

void WritePpm(const Output& out, const char* path) {
  FILE* f = std::fopen(path, "wb");
  if (f == nullptr) {
    std::printf("cannot write %s\n", path);
    return;
  }
  std::fprintf(f, "P6\n%u %u\n255\n", out.width, out.height);
  std::vector<std::uint8_t> row(static_cast<std::size_t>(out.width) * 3);
  for (std::uint32_t y = 0; y < out.height; ++y) {
    const std::uint8_t* px =
        out.capture.map + static_cast<std::size_t>(y) * out.capture.pitch;
    for (std::uint32_t x = 0; x < out.width; ++x) {
      row[std::size_t{3} * x] = px[std::size_t{4} * x + 2];
      row[std::size_t{3} * x + 1] = px[std::size_t{4} * x + 1];
      row[std::size_t{3} * x + 2] = px[std::size_t{4} * x];
    }
    std::fwrite(row.data(), 1, row.size(), f);
  }
  std::fclose(f);
}

// Puts `f` on the overlay plane over the background, waits for the writeback
// and compares the result with `ref`. False when KMS would not take the
// frame or gave nothing back.
bool ShowAndCompare(Output* out, const v4l2wc::V4l2DmaFrame& f,
                    const std::uint8_t* ref, std::size_t number) {
  const int fd = out->fd;
  std::uint32_t handles[4] = {}, pitches[4] = {}, offsets[4] = {};
  std::uint64_t modifiers[4] = {};
  for (std::uint32_t p = 0; p < f.num_planes; ++p) {
    if (drmPrimeFDToHandle(fd, f.fds[p], &handles[p]) != 0) {
      std::printf("frame %zu: the card will not import plane %u: %s\n", number,
                  p, std::strerror(errno));
      return false;
    }
    pitches[p] = f.pitches[p];
    offsets[p] = f.offsets[p];
    modifiers[p] = f.modifier;
  }
  std::uint32_t fb = 0;
  // The framebuffer reaches the picture's bottom right corner; the plane
  // shows the picture from where it starts.
  if (drmModeAddFB2WithModifiers(fd, f.crop_x + f.width, f.crop_y + f.height,
                                 f.drm_fourcc, handles, pitches, offsets,
                                 modifiers, &fb, DRM_MODE_FB_MODIFIERS) != 0) {
    std::printf("frame %zu: no framebuffer of it: %s\n", number,
                std::strerror(errno));
    return false;
  }

  // The plane converts with the matrix the stream names, where it can be
  // told; a stream that names none is taken as BT.601, limited range.
  const auto P = DRM_MODE_OBJECT_PLANE;
  std::uint64_t encoding = 0, range = f.video_full_range ? 1 : 0;
  if (f.matrix_coefficients == 1) {
    encoding = 1;
  } else if (f.matrix_coefficients == 9) {
    encoding = 2;
  }
  const Property enc_prop = FindProperty(fd, out->overlay, P, "COLOR_ENCODING");
  const Property range_prop = FindProperty(fd, out->overlay, P, "COLOR_RANGE");

  std::memset(out->capture.map, 0, out->capture.size);
  std::int32_t fence = -1;
  drmModeAtomicReq* req = drmModeAtomicAlloc();
  if (req == nullptr) {
    drmModeRmFB(fd, fb);
    return false;
  }
  const auto C = DRM_MODE_OBJECT_CONNECTOR, R = DRM_MODE_OBJECT_CRTC;
  Add(req, fd, out->connector, C, "CRTC_ID", out->crtc);
  Add(req, fd, out->connector, C, "WRITEBACK_FB_ID", out->capture.fb);
  Add(req, fd, out->connector, C, "WRITEBACK_OUT_FENCE_PTR",
      reinterpret_cast<std::uintptr_t>(&fence));
  Add(req, fd, out->crtc, R, "MODE_ID", out->mode_blob);
  Add(req, fd, out->crtc, R, "ACTIVE", 1);
  AddPlane(req, *out, out->primary, out->background.fb, out->width,
           out->height);
  AddPlane(req, *out, out->overlay, fb, f.width, f.height, f.crop_x, f.crop_y);
  if (enc_prop.id != 0) {
    drmModeAtomicAddProperty(req, out->overlay, enc_prop.id, encoding);
  } else {
    encoding = 0;
  }
  if (range_prop.id != 0) {
    drmModeAtomicAddProperty(req, out->overlay, range_prop.id, range);
  } else {
    range = 0;
  }
  bool ok = true;
  if (drmModeAtomicCommit(fd, req, DRM_MODE_ATOMIC_ALLOW_MODESET, nullptr) !=
      0) {
    std::printf(
        "frame %zu: KMS refused it: %s%s\n", number, std::strerror(errno),
        errno == EACCES ? " (another process is the card's master)" : "");
    ok = false;
  }
  drmModeAtomicFree(req);
  if (ok) {
    pollfd pfd{fence, POLLIN, 0};
    if (fence < 0 || poll(&pfd, 1, 3000) <= 0) {
      std::printf("frame %zu: no writeback within 3 s\n", number);
      ok = false;
    }
  }
  if (fence >= 0) {
    close(fence);
  }
  if (ok) {
    const Errors e =
        Compare(*out, MatrixFor(encoding, range), ref, f.width, f.height);
    std::printf(
        "frame %zu: luma off by %.2f, chroma by %.2f (mean), %zu "
        "background pixels changed\n",
        number, e.luma, e.chroma, e.background_off);
    CHECK(e.luma <= kMaxLumaError);
    CHECK(e.chroma <= kMaxChromaError);
    CHECK(e.background_off == 0);
  }
  // The plane lets go of the framebuffer before it is removed, or removing
  // it turns the plane off under a commit still on screen.
  TurnOff(*out);
  drmModeRmFB(fd, fb);
  for (std::uint32_t p = 0; p < f.num_planes; ++p) {
    bool again = false;
    for (std::uint32_t q = 0; q < p; ++q) {
      again = again || handles[q] == handles[p];
    }
    if (!again) {
      drm_gem_close gem{};
      gem.handle = handles[p];
      drmIoctl(fd, DRM_IOCTL_GEM_CLOSE, &gem);
    }
  }
  return ok;
}

// "WIDTHxHEIGHT", each a whole number that fits an unsigned.
bool ParseSize(const char* s, unsigned* width, unsigned* height) {
  char* end = nullptr;
  errno = 0;
  const unsigned long w = std::strtoul(s, &end, 10);
  if (end == s || *end != 'x' || errno != 0 || w > UINT_MAX) {
    return false;
  }
  const char* rest = end + 1;
  const unsigned long h = std::strtoul(rest, &end, 10);
  if (end == rest || *end != '\0' || errno != 0 || h > UINT_MAX) {
    return false;
  }
  *width = static_cast<unsigned>(w);
  *height = static_cast<unsigned>(h);
  return true;
}

}  // namespace

// A test: an exception ends it, which is a failure.
int main(int argc, char** argv) {  // NOLINT(bugprone-exception-escape)
  setvbuf(stdout, nullptr, _IONBF, 0);
  const bool hevc = argc > 1 && std::strcmp(argv[1], "hevc") == 0;
  unsigned width = 0, height = 0;
  if (argc < 5 || (!hevc && std::strcmp(argv[1], "h264") != 0) ||
      !ParseSize(argv[4], &width, &height) || width == 0 || height == 0 ||
      width > 1920 || height > 1080) {
    std::printf(
        "usage: %s h264|hevc clip ref.yuv WIDTHxHEIGHT [--frame N] "
        "[--card node] [--one-buffer] [--v4l2 node] [--hevc node] "
        "[--sand] [--ppm file]\n",
        argv[0]);
    return 2;
  }
  v4l2wc::DmaDecoderConfig config;
  config.codec_fourcc = hevc ? V4L2_PIX_FMT_HEVC : V4L2_PIX_FMT_H264;
  config.coded_width = width;
  config.coded_height = height;
  const char* card = nullptr;
  const char* ppm = nullptr;
  std::size_t only = 0;
  bool sand = false;
  for (int i = 5; i < argc; ++i) {
    const char* value = i + 1 < argc ? argv[i + 1] : nullptr;
    if (std::strcmp(argv[i], "--one-buffer") == 0) {
      config.one_buffer_frames = true;
    } else if (std::strcmp(argv[i], "--sand") == 0) {
      sand = true;
    } else if (value == nullptr) {
      std::printf("%s: unknown, or needs a value\n", argv[i]);
      return 2;
    } else if (std::strcmp(argv[i], "--frame") == 0) {
      only = std::strtoul(value, nullptr, 10);
      ++i;
    } else if (std::strcmp(argv[i], "--card") == 0) {
      card = value;
      ++i;
    } else if (std::strcmp(argv[i], "--v4l2") == 0) {
      config.v4l2_device = value;
      ++i;
    } else if (std::strcmp(argv[i], "--hevc") == 0) {
      config.v4l2_hevc_device = value;
      ++i;
    } else if (std::strcmp(argv[i], "--ppm") == 0) {
      ppm = value;
      ++i;
    } else {
      std::printf("%s: unknown\n", argv[i]);
      return 2;
    }
  }

  Stream s;
  if (!Load(argv[2], hevc, &s)) {
    std::printf("cannot read %s, or it has no access unit delimiters\n",
                argv[2]);
    return 2;
  }
  const std::size_t units = s.units.size() - 1;
  std::vector<std::uint8_t> ref;
  const std::size_t frame_bytes =
      static_cast<std::size_t>(width) * height +
      2 * (static_cast<std::size_t>((width + 1) / 2) * ((height + 1) / 2));
  if (!ReadFile(argv[3], &ref) || ref.size() < frame_bytes * units) {
    std::printf("%s is not %zu yuv420p frames of %ux%u\n", argv[3], units,
                width, height);
    return 2;
  }
  // Which frames to look at, from 1 and ascending.
  std::vector<std::size_t> wanted;
  if (only != 0) {
    if (only > units) {
      std::printf("the clip has %zu frames\n", units);
      return 2;
    }
    wanted = {only};
  } else {
    wanted = {1};
    if (units / 2 > 1) {
      wanted.push_back(units / 2);
    }
    if (units > wanted.back()) {
      wanted.push_back(units);
    }
  }

  Output out;
  if (card != nullptr) {
    if (!OpenOutput(card, &out)) {
      std::printf("%s has no writeback connector that writes XRGB8888\n", card);
      return 1;
    }
  } else {
    for (int i = 0; i < 16 && out.fd < 0; ++i) {
      const std::string node = "/dev/dri/card" + std::to_string(i);
      OpenOutput(node.c_str(), &out);
    }
    if (out.fd < 0) {
      std::printf("no card with a writeback connector that writes XRGB8888\n");
      return 1;
    }
  }
  if (!PickMode(width, height, &out.mode) ||
      drmModeCreatePropertyBlob(out.fd, &out.mode, sizeof out.mode,
                                &out.mode_blob) != 0) {
    std::printf("no mode\n");
    return 1;
  }
  out.width = out.mode.hdisplay;
  out.height = out.mode.vdisplay;
  if (!MakeDumb(out.fd, out.width, out.height, &out.background) ||
      !MakeDumb(out.fd, out.width, out.height, &out.capture)) {
    std::printf("no dumb buffers: %s\n", std::strerror(errno));
    return 1;
  }
  for (std::uint64_t i = 0; i + 3 < out.background.size; i += 4) {
    std::memcpy(out.background.map + i, kBackground, 3);
    out.background.map[i + 3] = 0xff;
  }

  std::unique_ptr<v4l2wc::IDmaDecoder> dec;
  if (sand) {
    if (hevc) {
      std::printf("--sand is for the H.264 stateful engine\n");
      return 2;
    }
    // As the factory makes it, but with the SAND capture format.
    dec = v4l2wc::V4l2M2mDecoder::Create(
        config.v4l2_device != nullptr ? config.v4l2_device : "/dev/video10",
        config.codec_fourcc, V4L2_PIX_FMT_NV12_COL128, width, height,
        /*output_buffer_count=*/4, /*capture_buffer_count=*/28,
        std::max<std::size_t>(static_cast<std::size_t>(width) * height,
                              std::size_t{1} << 20));
  } else {
    dec = v4l2wc::CreateDmaDecoder(config);
  }
  CHECK(dec != nullptr);
  std::size_t fed = 0, frames = 0, checked = 0;
  bool drained = false, told = false;
  int idle = 0;
  while (dec != nullptr && checked < wanted.size() && idle < 300) {
    bool progress = false;
    if (fed < units) {
      const std::size_t a = s.units[fed], b = s.units[fed + 1];
      const auto sr =
          dec->SubmitBitstream(s.bytes.data() + a, b - a, (fed + 1) * 33333ULL);
      if (sr == v4l2wc::SubmitResult::kOk) {
        ++fed;
        progress = true;
      } else if (sr != v4l2wc::SubmitResult::kTryAgain) {
        std::printf("submit result %d at unit %zu\n", static_cast<int>(sr),
                    fed);
        ++g_failures;
        break;
      }
    } else if (!drained) {
      dec->Drain();
      drained = true;
    }
    if (dec->Drive() != v4l2wc::DriveResult::kOk) {
      std::printf("the decoder stopped at unit %zu\n", fed);
      ++g_failures;
      break;
    }
    v4l2wc::V4l2DmaFrame f;
    while (checked < wanted.size() && dec->Acquire(&f)) {
      progress = true;
      ++frames;
      if (frames == wanted[checked]) {
        if (!told) {
          std::printf(
              "%ux%u at %u,%u %.4s modifier 0x%llx, %u plane(s) in %s, "
              "pitches %u/%u, offsets %u/%u\n",
              f.width, f.height, f.crop_x, f.crop_y,
              reinterpret_cast<const char*>(&f.drm_fourcc),
              static_cast<unsigned long long>(f.modifier), f.num_planes,
              f.num_planes > 1 && f.fds[1] != f.fds[0] ? "a buffer each"
                                                       : "one buffer",
              f.pitches[0], f.pitches[1], f.offsets[0], f.offsets[1]);
          told = true;
        }
        CHECK(f.width == width && f.height == height);
        if (out.overlay == 0 && !FindPlanes(&out, f.drm_fourcc, f.modifier)) {
          std::printf("no overlay plane takes this format and modifier\n");
          ++g_failures;
        } else if (f.width == width && f.height == height) {
          CHECK(ShowAndCompare(&out, f, ref.data() + frame_bytes * (frames - 1),
                               frames));
        }
        ++checked;
      }
      dec->Release(f.capture_index);
    }
    if (progress) {
      idle = 0;
    } else {
      ++idle;
      usleep(1000);
    }
  }
  CHECK(checked == wanted.size());
  if (checked != wanted.size()) {
    std::printf("%zu frame(s) of %zu access units came out\n", frames, units);
  }
  if (ppm != nullptr && checked > 0) {
    WritePpm(out, ppm);
  }
  dec.reset();
  FreeDumb(out.fd, &out.background);
  FreeDumb(out.fd, &out.capture);
  drmModeDestroyPropertyBlob(out.fd, out.mode_blob);
  close(out.fd);
  std::printf(g_failures == 0 ? "PASS\n" : "FAILED (%d)\n", g_failures);
  return g_failures == 0 ? 0 : 1;
}
