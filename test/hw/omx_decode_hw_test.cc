// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

// On-hardware test of the OpenMAX engine. Decodes an Annex B H.264 file that
// has an access unit delimiter before each frame, three ways through the
// IDmaDecoder seam, and checks what comes out:
//   1. start to finish, holding several frames before releasing them;
//   2. a flush halfway, then the whole file again (a seek);
//   3. a second decoder after the first is destroyed.
// Needs a board with an OpenMAX core. Usage: omx_decode_hw_test clip.h264
// [core]

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <deque>
#include <memory>
#include <vector>

#include "src/omx_decoder.h"

namespace {

int g_failures = 0;
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

bool Load(const char* path, Stream* s) {
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
  // An access unit runs from one delimiter (NAL type 9) to the next; what
  // precedes the first delimiter goes with the first unit.
  s->units.push_back(0);
  bool first = true;
  for (std::size_t i = 0; i + 3 < d.size(); ++i) {
    if (d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 1 && (d[i + 3] & 0x1f) == 9) {
      const std::size_t start = (i > 0 && d[i - 1] == 0) ? i - 1 : i;
      if (!first) {
        s->units.push_back(start);
      }
      first = false;
    }
  }
  s->units.push_back(d.size());
  return s->units.size() >= 2;
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
            "first frame %ux%u fd %d planes %u offsets %u/%u pitch %u "
            "ts %llu\n",
            f.width, f.height, f.fds[0], f.num_planes, f.offsets[0],
            f.offsets[1], f.pitches[0],
            static_cast<unsigned long long>(f.timestamp));
        const off_t size = lseek(f.fds[0], 0, SEEK_END);
        void* p = size > 0 ? mmap(nullptr, static_cast<std::size_t>(size),
                                  PROT_READ, MAP_SHARED, f.fds[0], 0)
                           : MAP_FAILED;
        CHECK(p != MAP_FAILED);
        if (p != MAP_FAILED) {
          const auto* base = static_cast<const std::uint8_t*>(p);
          const std::uint32_t row = f.height / 2;
          for (std::uint32_t x = f.width / 12; x < f.width; x += f.width / 6) {
            std::printf(
                "  x=%u Y=%u U=%u V=%u\n", x,
                base[f.offsets[0] + row * f.pitches[0] + x],
                base[f.offsets[1] + (row / 2) * f.pitches[1] + (x & ~1u)],
                base[f.offsets[1] + (row / 2) * f.pitches[1] + (x & ~1u) + 1]);
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
  if (argc < 2) {
    std::printf("usage: %s clip.h264 [core library]\n", argv[0]);
    return 2;
  }
  Stream s;
  if (!Load(argv[1], &s)) {
    std::printf("cannot read %s\n", argv[1]);
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
  const char* core = argc > 2 ? argv[2] : nullptr;
  std::printf("%zu bytes, %zu access units\n", s.bytes.size(), units);
  int fds_before = -1;

  {
    auto dec = v4l2wc::OmxDecoder::Create(core, V4L2_PIX_FMT_H264, 0, 0, 8);
    CHECK(dec != nullptr);
    if (dec == nullptr) {
      return 1;
    }
    fds_before = count_fds();
    // 1. Start to finish, six frames held at a time.
    Result all;
    CHECK(Pump(dec.get(), s, 0, units, /*drain=*/true, /*hold=*/6, &all, true));
    std::printf("whole file: %zu frames of %zu, %ux%u, pool %u, %s\n",
                all.frames, units, all.width, all.height, all.pool,
                all.ordered ? "in order" : "OUT OF ORDER");
    CHECK(all.frames == units);
    CHECK(all.ordered);

    // 2. Half the file, a flush, then all of it again from its key frame.
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

  // 3. The first decoder is gone; a second starts clean.
  {
    auto dec = v4l2wc::OmxDecoder::Create(core, V4L2_PIX_FMT_H264, 0, 0, 0);
    CHECK(dec != nullptr);
    if (dec != nullptr) {
      Result r;
      CHECK(Pump(dec.get(), s, 0, units, /*drain=*/true, 1, &r, false));
      std::printf("second decoder: %zu frames of %zu, pool %u\n", r.frames,
                  units, r.pool);
      CHECK(r.frames == units);
    }
  }
  // The second decoder had the first one's descriptors to reuse; what is open
  // now is the core's own, which the first Create already had.
  const int fds_after = count_fds();
  std::printf("open descriptors: %d with one decoder running, %d with none\n",
              fds_before, fds_after);
  CHECK(fds_after < fds_before);
  std::printf(g_failures == 0 ? "PASS\n" : "FAILED (%d)\n", g_failures);
  return g_failures == 0 ? 0 : 1;
}
