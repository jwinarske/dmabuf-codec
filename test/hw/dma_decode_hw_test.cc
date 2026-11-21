// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// On-hardware test of whichever engine the factory picks on this device.
// Decodes an Annex B H.264 or HEVC file that has an access unit delimiter
// before each frame and no reordered frames, three ways through the
// IDmaDecoder seam, and checks what comes out:
//   1. start to finish, holding several frames before releasing them;
//   2. the whole file again with no flush: decoding goes on after a drain;
//   3. a flush halfway, then the whole file again (a seek);
//   4. a second decoder after the first is destroyed.
// Usage: dma_decode_hw_test h264|hevc clip [--v4l2 node] [--hevc node]
//        [--omx core library] [--va render node] [--one-buffer]
// A clip for it:
//   ffmpeg -f lavfi -i testsrc=s=640x360:d=2:r=30 -c:v libx265
//     -x265-params bframes=0 -bsf:v hevc_metadata=aud=insert -f hevc clip.h265

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <memory>
#include <vector>

#include "src/dma_decoder_factory.h"

namespace {

int g_failures = 0;
bool g_one_buffer = false;  // --one-buffer: a frame is to be one dma-buf
// DRM_FORMAT_MOD_BROADCOM_SAND128 with no column height in it.
constexpr std::uint64_t kModifierSand128 = (7ULL << 56) | 4;
#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++g_failures;                                               \
    }                                                             \
  } while (0)

struct Stream {
  std::vector<std::uint8_t> bytes;
  std::vector<std::size_t> units;  // start of each access unit, then the end
};

// Whether the NAL unit whose header starts at `b` is an access unit delimiter.
bool IsDelimiter(const std::uint8_t* b, bool hevc) {
  return hevc ? ((b[0] >> 1) & 0x3f) == 35 : (b[0] & 0x1f) == 9;
}

bool Load(const char* path, bool hevc, Stream* s) {
  FILE* f = std::fopen(path, "rb");
  if (f == nullptr) {
    return false;
  }
  std::uint8_t buf[65536];
  std::size_t n;
  while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) {
    s->bytes.insert(s->bytes.end(), buf, buf + n);
  }
  std::fclose(f);
  const auto& d = s->bytes;
  // An access unit runs from one delimiter to the next; what precedes the
  // first delimiter goes with the first unit.
  s->units.push_back(0);
  bool first = true;
  for (std::size_t i = 0; i + 3 < d.size(); ++i) {
    if (d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 1 &&
        IsDelimiter(&d[i + 3], hevc)) {
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

struct Result {
  std::size_t frames = 0;
  bool ordered = true;
  std::uint32_t width = 0, height = 0, pool = 0;
};

// Feeds units [from, to) and collects frames until `want` have arrived or the
// decoder goes quiet. Holds up to `hold` frames before releasing the oldest.
bool Pump(v4l2wc::IDmaDecoder* dec, const Stream& s, std::size_t from,
          std::size_t to, bool drain, std::size_t hold, Result* r,
          bool show_first) {
  std::size_t fed = from;
  bool drained = false;
  std::deque<std::uint32_t> held;
  std::uint64_t last_ts = 0;
  bool have_ts = false;
  int idle = 0;
  while (idle < 300) {
    bool progress = false;
    if (fed < to) {
      const std::size_t a = s.units[fed], b = s.units[fed + 1];
      const auto sr =
          dec->SubmitBitstream(s.bytes.data() + a, b - a, (fed + 1) * 33333ULL);
      if (sr == v4l2wc::SubmitResult::kOk) {
        ++fed;
        progress = true;
      } else if (sr != v4l2wc::SubmitResult::kTryAgain) {
        std::printf("submit result %d at unit %zu\n", static_cast<int>(sr),
                    fed);
        return false;
      }
    } else if (drain && !drained) {
      dec->Drain();
      drained = true;
    }
    const auto dr = dec->Drive();
    if (dr != v4l2wc::DriveResult::kOk) {
      std::printf("drive result %d\n", static_cast<int>(dr));
      return false;
    }
    v4l2wc::V4l2DmaFrame f;
    while (dec->Acquire(&f)) {
      progress = true;
      if (have_ts && f.timestamp <= last_ts) {
        r->ordered = false;
      }
      last_ts = f.timestamp;
      have_ts = true;
      r->width = f.width;
      r->height = f.height;
      if (r->frames == 0 && show_first) {
        std::printf(
            "first frame %ux%u at %u,%u fourcc %.4s modifier 0x%llx fd %d "
            "planes %u offsets %u/%u pitch %u ts %llu\n",
            f.width, f.height, f.crop_x, f.crop_y,
            reinterpret_cast<const char*>(&f.drm_fourcc),
            static_cast<unsigned long long>(f.modifier), f.fds[0], f.num_planes,
            f.offsets[0], f.offsets[1], f.pitches[0],
            static_cast<unsigned long long>(f.timestamp));
        // H.273 codes as the engine reports them; 2 is "unspecified".
        std::printf(
            "first frame color: primaries %u transfer %u matrix %u range %s\n",
            f.colour_primaries, f.transfer_characteristics,
            f.matrix_coefficients, f.video_full_range ? "full" : "limited");
        CHECK(f.fds[0] >= 0);
        if (g_one_buffer) {
          for (std::uint32_t p = 1; p < f.num_planes; ++p) {
            CHECK(f.fds[p] == f.fds[0]);
          }
        }
        CHECK(f.width > 0 && f.height > 0 && f.pitches[0] > 0);
        // The samples are read from an 8-bit NV12 buffer that is linear, or
        // in Broadcom's 128-byte columns (SAND128, each plane's pitch the
        // height of its columns in rows) with both planes in one buffer.
        // Another tiling is left to the display.
        const bool nv12 = f.drm_fourcc == V4L2_PIX_FMT_NV12 &&
                          f.num_planes == 2 && f.fds[1] == f.fds[0];
        const bool sand = f.modifier == kModifierSand128;
        const bool rows = nv12 && (f.modifier == 0 || sand);
        const off_t size = rows ? lseek(f.fds[0], 0, SEEK_END) : 0;
        void* p = size > 0 ? mmap(nullptr, static_cast<std::size_t>(size),
                                  PROT_READ, MAP_SHARED, f.fds[0], 0)
                           : MAP_FAILED;
        CHECK(!rows || p != MAP_FAILED);
        if (p != MAP_FAILED) {
          const auto* base = static_cast<const std::uint8_t*>(p);
          const auto at = [&](std::uint32_t plane, std::uint32_t x,
                              std::uint32_t y) -> std::uint8_t {
            const std::uint64_t o =
                sand ? std::uint64_t{f.offsets[plane]} +
                           std::uint64_t{x / 128} * 128 * f.pitches[plane] +
                           std::uint64_t{y} * 128 + x % 128
                     : std::uint64_t{f.offsets[plane]} +
                           std::uint64_t{y} * f.pitches[plane] + x;
            return o < static_cast<std::uint64_t>(size) ? base[o] : 0;
          };
          // The picture's middle row, from where it starts in the buffer.
          const std::uint32_t row = f.crop_y + f.height / 2;
          for (std::uint32_t x = f.width / 12; x < f.width; x += f.width / 6) {
            const std::uint32_t bx = f.crop_x + x;
            std::printf("  x=%u Y=%u U=%u V=%u\n", x, at(0, bx, row),
                        at(1, bx & ~1u, row / 2),
                        at(1, (bx & ~1u) + 1, row / 2));
          }
          munmap(p, static_cast<std::size_t>(size));
        }
      }
      ++r->frames;
      held.push_back(f.capture_index);
      while (held.size() > hold) {
        dec->Release(held.front());
        held.pop_front();
      }
    }
    if (progress) {
      idle = 0;
    } else {
      ++idle;
      usleep(5000);
    }
  }
  for (std::uint32_t index : held) {
    dec->Release(index);
  }
  r->pool = dec->PoolSize();
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  setvbuf(stdout, nullptr, _IONBF, 0);
  const bool hevc = argc > 1 && std::strcmp(argv[1], "hevc") == 0;
  if (argc < 3 || (!hevc && std::strcmp(argv[1], "h264") != 0)) {
    std::printf(
        "usage: %s h264|hevc clip [--v4l2 node] [--hevc node] "
        "[--omx core library] [--va render node]\n",
        argv[0]);
    return 2;
  }
  v4l2wc::DmaDecoderConfig config;
  config.codec_fourcc = hevc ? V4L2_PIX_FMT_HEVC : V4L2_PIX_FMT_H264;
  for (int i = 3; i + 1 < argc; i += 2) {
    if (std::strcmp(argv[i], "--v4l2") == 0) {
      config.v4l2_device = argv[i + 1];
    } else if (std::strcmp(argv[i], "--hevc") == 0) {
      config.v4l2_hevc_device = argv[i + 1];
    } else if (std::strcmp(argv[i], "--omx") == 0) {
      config.omx_core_library = argv[i + 1];
    } else if (std::strcmp(argv[i], "--va") == 0) {
      config.vaapi_render_node = argv[i + 1];
    }
  }
  for (int i = 3; i < argc; ++i) {
    config.one_buffer_frames =
        config.one_buffer_frames || std::strcmp(argv[i], "--one-buffer") == 0;
  }
  g_one_buffer = config.one_buffer_frames;
  Stream s;
  if (!Load(argv[2], hevc, &s)) {
    std::printf("cannot read %s, or it has no access unit delimiters\n",
                argv[2]);
    return 2;
  }
  const std::size_t units = s.units.size() - 1;
  // Open descriptors, to compare at the end: a decoder must take its buffers'
  // fds with it.
  auto count_fds = [] {
    int n = 0;
    for (int fd = 0; fd < 1024; ++fd) {
      n += fcntl(fd, F_GETFD) != -1 ? 1 : 0;
    }
    return n;
  };
  std::printf("%zu bytes, %zu access units\n", s.bytes.size(), units);
  int fds_before = -1;

  {
    auto dec = v4l2wc::CreateDmaDecoder(config);
    CHECK(dec != nullptr);
    if (dec == nullptr) {
      return 1;
    }
    // 1. Start to finish, six frames held at a time.
    Result all;
    CHECK(Pump(dec.get(), s, 0, units, /*drain=*/true, /*hold=*/6, &all, true));
    std::printf("whole file: %zu frames of %zu, %ux%u, pool %u, %s\n",
                all.frames, units, all.width, all.height, all.pool,
                all.ordered ? "in order" : "OUT OF ORDER");
    CHECK(all.frames == units);
    CHECK(all.ordered);
    // Counted with the decoder's buffers in existence: some engines make
    // them only once they have seen the stream.
    fds_before = count_fds();

    // 2. The whole file again with no flush: a SubmitBitstream() after a
    //    Drain() resumes decoding.
    Result more;
    CHECK(Pump(dec.get(), s, 0, units, /*drain=*/true, 2, &more, false));
    std::printf("after a drain: %zu frames of %zu\n", more.frames, units);
    CHECK(more.frames == units);
    CHECK(more.ordered);

    // 3. Half the file, a flush, then all of it again from its key frame.
    dec->Flush();
    Result half;
    CHECK(Pump(dec.get(), s, 0, units / 2, /*drain=*/false, 2, &half, false));
    dec->Flush();
    Result again;
    CHECK(Pump(dec.get(), s, 0, units, /*drain=*/true, 2, &again, false));
    std::printf("after a flush: %zu frames before it, %zu of %zu after\n",
                half.frames, again.frames, units);
    CHECK(half.frames > 0 && half.frames <= units / 2);
    CHECK(again.frames == units);
    CHECK(again.ordered);
  }

  // 4. The first decoder is gone; a second starts clean.
  {
    auto dec = v4l2wc::CreateDmaDecoder(config);
    CHECK(dec != nullptr);
    if (dec != nullptr) {
      Result r;
      CHECK(Pump(dec.get(), s, 0, units, /*drain=*/true, 1, &r, false));
      std::printf("second decoder: %zu frames of %zu, pool %u\n", r.frames,
                  units, r.pool);
      CHECK(r.frames == units);
    }
  }
  // 5. A consumer that gives nothing back. The decoder runs out of buffers
  //    to decode into and has to say so, not wait for one: the caller may be
  //    holding a lock the consumer needs to return a frame. Without that this
  //    step does not come back.
  {
    auto dec = v4l2wc::CreateDmaDecoder(config);
    CHECK(dec != nullptr);
    if (dec != nullptr) {
      Result r;
      CHECK(Pump(dec.get(), s, 0, units, /*drain=*/false, /*hold=*/units + 1,
                 &r, false));
      std::printf("nothing released: %zu frames of %zu, pool %u\n", r.frames,
                  units, r.pool);
      CHECK(r.frames > 0);
      CHECK(r.frames <= r.pool || r.frames == units);
    }
  }
  // The second decoder had the first one's descriptors to reuse; what is open
  // now is what the process keeps of the device (an OpenMAX core's own, a
  // driver library's), which the first decoder already had.
  const int fds_after = count_fds();
  std::printf("open descriptors: %d with one decoder running, %d with none\n",
              fds_before, fds_after);
  CHECK(fds_after < fds_before);
  std::printf(g_failures == 0 ? "PASS\n" : "FAILED (%d)\n", g_failures);
  return g_failures == 0 ? 0 : 1;
}
