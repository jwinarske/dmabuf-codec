// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "src/omx_decoder.h"

#include <OMX_Component.h>
#include <OMX_Core.h>
#include <OMX_Video.h>
#include <dlfcn.h>
#include <linux/dma-buf.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "parse/sps_color.h"
#include "src/log.h"

namespace v4l2wc {
namespace {

constexpr OMX_U32 kInPort = 0;
constexpr OMX_U32 kOutPort = 1;
constexpr int kCommandTimeoutMs = 3000;

// Until the output has settled (the component's first reconfiguration, or its
// first frame) one access unit is in flight. A component that reconfigures its
// output on the first key frame keeps that one unit through it, but throws
// away what was queued behind it, and everything up to the next key frame is
// then undecodable. A component that settles without a word gets the rest
// after this long.
constexpr auto kSettleWait = std::chrono::milliseconds(300);

// Linear NV12 as Qualcomm's decoders name it (128-aligned stride, 32-aligned
// lines: the layout its display side takes). The IL's own NV12 is the
// fallback there and the choice everywhere else.
constexpr OMX_U32 kQcomNv12Venus = 0x7FA30C04;

// Qualcomm's pPlatformPrivate, as its public mm-core extension header lays it
// out: a list of typed entries, of which type 1 describes the buffer's
// memory. A build of the component against gbm adds the metadata fd.
struct QcomEntry {
  OMX_U32 type;
  OMX_PTR entry;
};
struct QcomList {
  OMX_U32 entries;
  QcomEntry* list;
};
struct QcomMemGbm {
  unsigned long fd;
  unsigned long meta_fd;
  OMX_U32 offset;
  OMX_U32 size;
  OMX_U32 mapped_size;
  OMX_PTR buffer;
};
struct QcomMem {
  unsigned long fd;
  OMX_U32 offset;
  OMX_U32 size;
  OMX_U32 mapped_size;
  OMX_PTR buffer;
};
constexpr OMX_U32 kQcomEntryMem = 1;

template <typename T>
void InitParam(T* p) {
  std::memset(p, 0, sizeof(T));
  p->nSize = sizeof(T);
  p->nVersion.s.nVersionMajor = 1;
  p->nVersion.s.nVersionMinor = 1;
}

const char* RoleFor(std::uint32_t fourcc) {
  switch (fourcc) {
    case V4L2_PIX_FMT_H264:
      return "video_decoder.avc";
    case V4L2_PIX_FMT_HEVC:
      return "video_decoder.hevc";
    case V4L2_PIX_FMT_VP8:
      return "video_decoder.vp8";
    case V4L2_PIX_FMT_VP9:
      return "video_decoder.vp9";
    case V4L2_PIX_FMT_MPEG2:
      return "video_decoder.mpeg2";
    default:
      return nullptr;
  }
}

bool IsDmabuf(int fd) {
  if (fd < 0) {
    return false;
  }
  struct dma_buf_sync sync{};
  sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ;
  if (ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync) != 0) {
    return false;
  }
  sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ;
  ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
  return true;
}

}  // namespace

struct OmxDecoder::State {
  // Who has an output buffer.
  enum class Owner { kComponent, kReady, kClient, kIdle };
  struct OutBuffer {
    OMX_BUFFERHEADERTYPE* header = nullptr;
    int fd = -1;  // the component's; valid while the buffer is allocated
    std::uint32_t offset = 0;
    Owner owner = Owner::kIdle;
    std::uint64_t timestamp = 0;
  };
  // The color the stream's last SPS said (H.264 and HEVC), taken from the
  // bitstream rather than asked of the component. Read as access units go in;
  // `color` is under mu. A picture still in the component when a new SPS
  // arrives is given the new one's.
  bool watch_color = false;
  SpsCodec color_codec = SpsCodec::kH264;
  SpsColor color;
  // The core, loaded at run time.
  void* lib = nullptr;
  OMX_ERRORTYPE (*init)() = nullptr;
  OMX_ERRORTYPE (*deinit)() = nullptr;
  OMX_ERRORTYPE (*components_of_role)(OMX_STRING, OMX_U32*, OMX_U8**) = nullptr;
  OMX_ERRORTYPE (*get_handle)(OMX_HANDLETYPE*, OMX_STRING, OMX_PTR,
                              OMX_CALLBACKTYPE*) = nullptr;
  OMX_ERRORTYPE (*free_handle)(OMX_HANDLETYPE) = nullptr;
  bool inited = false;
  bool executing = false;  // reached the executing state
  OMX_HANDLETYPE handle = nullptr;
  bool qcom = false;  // the component's private data is Qualcomm's layout
  std::uint32_t extra_outputs = 0;

  // Output geometry, from the port definition and the crop rectangle.
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t stride = 0;
  std::uint32_t slice_height = 0;

  // Touched by the component's threads (the callbacks) and the consumer's.
  std::mutex mu;
  std::condition_variable cv;
  int command_done[8] = {};
  bool error = false;
  bool port_changed = false;
  bool crop_changed = false;
  bool flushing = false;     // every returned output buffer is to be discarded
  bool out_enabled = false;  // output buffers may be given to the component
  std::deque<OMX_BUFFERHEADERTYPE*> in_free;
  std::vector<OMX_BUFFERHEADERTYPE*> in_all;
  std::vector<OutBuffer> out;
  std::deque<std::uint32_t> ready;     // decode order, indices into `out`
  std::deque<std::uint32_t> returned;  // came back empty; to be given again

  // Consumer thread only.
  bool settled = false;
  bool need_recreate = false;
  bool drain_pending = false;
  bool eos_sent = false;
  std::uint64_t frames_out = 0;
  bool first_sent = false;
  std::chrono::steady_clock::time_point first_submit{};

  static OMX_ERRORTYPE OnEvent(OMX_HANDLETYPE, OMX_PTR app, OMX_EVENTTYPE event,
                               OMX_U32 d1, OMX_U32 d2, OMX_PTR) {
    auto* s = static_cast<State*>(app);
    std::lock_guard<std::mutex> lock(s->mu);
    switch (event) {
      case OMX_EventCmdComplete:
        if (d1 < 8) {
          ++s->command_done[d1];
        }
        break;
      case OMX_EventPortSettingsChanged:
        if (d1 == kOutPort) {
          if (d2 == 0 || d2 == OMX_IndexParamPortDefinition) {
            s->port_changed = true;
          } else if (d2 == OMX_IndexConfigCommonOutputCrop) {
            s->crop_changed = true;
          }
        }
        break;
      case OMX_EventError:
        V4L2WC_LOG(V4L2WC_ERROR)
            << "omx: component error 0x" << std::hex << d1 << " 0x" << d2;
        s->error = true;
        break;
      default:
        break;
    }
    s->cv.notify_all();
    return OMX_ErrorNone;
  }

  static OMX_ERRORTYPE OnEmptyDone(OMX_HANDLETYPE, OMX_PTR app,
                                   OMX_BUFFERHEADERTYPE* header) {
    auto* s = static_cast<State*>(app);
    std::lock_guard<std::mutex> lock(s->mu);
    s->in_free.push_back(header);
    s->cv.notify_all();
    return OMX_ErrorNone;
  }

  static OMX_ERRORTYPE OnFillDone(OMX_HANDLETYPE, OMX_PTR app,
                                  OMX_BUFFERHEADERTYPE* header) {
    auto* s = static_cast<State*>(app);
    std::lock_guard<std::mutex> lock(s->mu);
    for (std::uint32_t i = 0; i < s->out.size(); ++i) {
      OutBuffer& b = s->out[i];
      if (b.header != header) {
        continue;
      }
      if (header->nFilledLen > 0 && !s->flushing && s->out_enabled) {
        b.owner = Owner::kReady;
        b.timestamp = static_cast<std::uint64_t>(header->nTimeStamp);
        s->ready.push_back(i);
      } else {
        // Empty: handed back by a flush, a port disable or end of stream.
        b.owner = Owner::kIdle;
        s->returned.push_back(i);
      }
      break;
    }
    s->cv.notify_all();
    return OMX_ErrorNone;
  }

  bool WaitCommand(OMX_COMMANDTYPE command, int count = 1) {
    std::unique_lock<std::mutex> lock(mu);
    const bool ok =
        cv.wait_for(lock, std::chrono::milliseconds(kCommandTimeoutMs),
                    [&] { return command_done[command] >= count || error; });
    if (ok && !error) {
      command_done[command] -= count;
      return true;
    }
    V4L2WC_LOG(V4L2WC_ERROR) << "omx: command " << static_cast<int>(command)
                             << (error ? " failed" : " timed out");
    return false;
  }

  bool Send(OMX_COMMANDTYPE command, OMX_U32 param) {
    const OMX_ERRORTYPE e = OMX_SendCommand(handle, command, param, nullptr);
    if (e != OMX_ErrorNone) {
      V4L2WC_LOG(V4L2WC_ERROR)
          << "omx: SendCommand " << static_cast<int>(command) << " -> 0x"
          << std::hex << static_cast<unsigned>(e);
      return false;
    }
    return true;
  }

  // The dma-buf behind an output buffer, where the component's private data
  // is in a layout this engine knows. -1 otherwise. Nothing is dereferenced
  // for a component of unknown make.
  int FindFd(const OMX_BUFFERHEADERTYPE* header, std::uint32_t* offset) const {
    if (!qcom || header->pPlatformPrivate == nullptr) {
      return -1;
    }
    const auto* list = static_cast<const QcomList*>(header->pPlatformPrivate);
    if (list->list == nullptr) {
      return -1;
    }
    for (OMX_U32 i = 0; i < list->entries && i < 4; ++i) {
      if (list->list[i].type != kQcomEntryMem ||
          list->list[i].entry == nullptr) {
        continue;
      }
      // Which of the two layouts it is shows in where the buffer pointer is.
      // The shorter one is read first, so the longer is only looked at when
      // the entry is not the shorter.
      const auto* plain = static_cast<const QcomMem*>(list->list[i].entry);
      if (plain->buffer == header->pBuffer) {
        if (!IsDmabuf(static_cast<int>(plain->fd))) {
          return -1;
        }
        *offset = plain->offset;
        return static_cast<int>(plain->fd);
      }
      const auto* gbm = static_cast<const QcomMemGbm*>(list->list[i].entry);
      if (gbm->buffer == header->pBuffer &&
          IsDmabuf(static_cast<int>(gbm->fd))) {
        *offset = gbm->offset;
        return static_cast<int>(gbm->fd);
      }
    }
    return -1;
  }

  void ReadCrop() {
    OMX_CONFIG_RECTTYPE rect;
    InitParam(&rect);
    rect.nPortIndex = kOutPort;
    if (OMX_GetConfig(handle, OMX_IndexConfigCommonOutputCrop, &rect) ==
            OMX_ErrorNone &&
        rect.nWidth > 0 && rect.nHeight > 0 && rect.nLeft == 0 &&
        rect.nTop == 0 && rect.nWidth <= stride &&
        rect.nHeight <= slice_height) {
      width = rect.nWidth;
      height = rect.nHeight;
    }
  }

  // Allocates the output port's buffers for its current definition. The port
  // must be enabling, or the component moving to idle.
  bool AllocateOutput() {
    OMX_PARAM_PORTDEFINITIONTYPE def;
    InitParam(&def);
    def.nPortIndex = kOutPort;
    if (OMX_GetParameter(handle, OMX_IndexParamPortDefinition, &def) !=
        OMX_ErrorNone) {
      return false;
    }
    // Set it back even when nothing is raised: a component may only work out
    // the stride and line count for a new picture size when the definition
    // is set, and reports the old ones until then.
    const OMX_U32 want = def.nBufferCountMin + extra_outputs;
    if (def.nBufferCountActual < want) {
      def.nBufferCountActual = want;
    }
    if (OMX_SetParameter(handle, OMX_IndexParamPortDefinition, &def) !=
            OMX_ErrorNone ||
        OMX_GetParameter(handle, OMX_IndexParamPortDefinition, &def) !=
            OMX_ErrorNone) {
      V4L2WC_LOG(V4L2WC_ERROR) << "omx: output port definition refused";
      return false;
    }
    const auto& v = def.format.video;
    if (v.nStride <= 0 || v.nSliceHeight == 0 || v.nFrameWidth == 0 ||
        v.nFrameHeight == 0 ||
        static_cast<std::uint32_t>(v.nStride) < v.nFrameWidth) {
      V4L2WC_LOG(V4L2WC_ERROR)
          << "omx: output port has no usable geometry (" << v.nFrameWidth << "x"
          << v.nFrameHeight << " stride " << v.nStride << " lines "
          << v.nSliceHeight << ")";
      return false;
    }
    stride = static_cast<std::uint32_t>(v.nStride);
    slice_height = v.nSliceHeight;
    width = v.nFrameWidth;
    height = v.nFrameHeight < slice_height ? v.nFrameHeight : slice_height;
    const std::uint64_t need =
        static_cast<std::uint64_t>(stride) * slice_height +
        static_cast<std::uint64_t>(stride) * ((height + 1) / 2);

    std::vector<OutBuffer> fresh(def.nBufferCountActual);
    bool ok = true;
    for (OutBuffer& b : fresh) {
      if (OMX_AllocateBuffer(handle, &b.header, kOutPort, nullptr,
                             def.nBufferSize) != OMX_ErrorNone) {
        V4L2WC_LOG(V4L2WC_ERROR) << "omx: output AllocateBuffer failed";
        ok = false;
        break;
      }
      b.fd = FindFd(b.header, &b.offset);
      const off_t size = b.fd >= 0 ? lseek(b.fd, 0, SEEK_END) : -1;
      if (b.fd < 0 || size < 0 ||
          static_cast<std::uint64_t>(size) < b.offset + need) {
        V4L2WC_LOG(V4L2WC_ERROR)
            << "omx: no dma-buf fd on the component's output buffers";
        ok = false;
        break;
      }
    }
    {
      std::lock_guard<std::mutex> lock(mu);
      out = std::move(fresh);
      ready.clear();
      returned.clear();
    }
    if (ok) {
      V4L2WC_LOG(V4L2WC_INFO)
          << "omx: output " << width << "x" << height << " stride " << stride
          << " lines " << slice_height << ", " << out.size() << " buffers";
    }
    return ok;
  }

  void FreeOutput() {
    std::vector<OutBuffer> old;
    {
      std::lock_guard<std::mutex> lock(mu);
      old.swap(out);
      ready.clear();
      returned.clear();
    }
    for (OutBuffer& b : old) {
      if (b.header != nullptr) {
        OMX_FreeBuffer(handle, kOutPort, b.header);
      }
    }
  }

  // Gives the component every output buffer nobody holds.
  void FillIdle() {
    std::vector<OMX_BUFFERHEADERTYPE*> give;
    {
      std::lock_guard<std::mutex> lock(mu);
      if (!out_enabled) {
        return;
      }
      returned.clear();
      for (OutBuffer& b : out) {
        if (b.owner == Owner::kIdle) {
          b.owner = Owner::kComponent;
          give.push_back(b.header);
        }
      }
    }
    for (OMX_BUFFERHEADERTYPE* h : give) {
      h->nFilledLen = 0;
      h->nOffset = 0;
      h->nFlags = 0;
      OMX_FillThisBuffer(handle, h);
    }
  }

  bool WaitOutputsBack() {
    std::unique_lock<std::mutex> lock(mu);
    return cv.wait_for(lock, std::chrono::milliseconds(kCommandTimeoutMs), [&] {
      for (const OutBuffer& b : out) {
        if (b.owner == Owner::kComponent) {
          return false;
        }
      }
      return true;
    });
  }

  // The disable / free / allocate / enable sequence a component asks for with
  // OMX_EventPortSettingsChanged on its output port.
  bool ReconfigureOutput() {
    {
      std::lock_guard<std::mutex> lock(mu);
      out_enabled = false;
      port_changed = false;
    }
    if (!Send(OMX_CommandPortDisable, kOutPort)) {
      return false;
    }
    if (!WaitOutputsBack()) {
      V4L2WC_LOG(V4L2WC_ERROR) << "omx: output buffers did not come back";
      return false;
    }
    FreeOutput();
    if (!WaitCommand(OMX_CommandPortDisable)) {
      return false;
    }
    if (!Send(OMX_CommandPortEnable, kOutPort) || !AllocateOutput() ||
        !WaitCommand(OMX_CommandPortEnable)) {
      return false;
    }
    ReadCrop();
    {
      std::lock_guard<std::mutex> lock(mu);
      out_enabled = true;
    }
    FillIdle();
    return true;
  }

  // Copies one access unit into a free input buffer and queues it. False when
  // there is no free buffer.
  bool Queue(const std::uint8_t* data, std::size_t size,
             std::uint64_t timestamp, OMX_U32 flags, bool* too_big) {
    OMX_BUFFERHEADERTYPE* h = nullptr;
    {
      std::lock_guard<std::mutex> lock(mu);
      if (in_free.empty()) {
        return false;
      }
      h = in_free.front();
      if (size > h->nAllocLen) {
        if (too_big != nullptr) {
          *too_big = true;
        }
        return false;
      }
      in_free.pop_front();
    }
    if (size > 0) {
      std::memcpy(h->pBuffer, data, size);
    }
    h->nFilledLen = static_cast<OMX_U32>(size);
    h->nOffset = 0;
    h->nTimeStamp = static_cast<OMX_TICKS>(timestamp);
    h->nFlags = flags;
    if (OMX_EmptyThisBuffer(handle, h) != OMX_ErrorNone) {
      std::lock_guard<std::mutex> lock(mu);
      in_free.push_back(h);
      error = true;
      return false;
    }
    return true;
  }
};

OmxDecoder::OmxDecoder() : s_(std::make_unique<State>()) {}

std::unique_ptr<OmxDecoder> OmxDecoder::Create(
    const char* core_library, std::uint32_t codec_fourcc,
    std::uint32_t coded_width, std::uint32_t coded_height,
    std::uint32_t extra_output_buffers) {
  const char* role = RoleFor(codec_fourcc);
  if (role == nullptr) {
    return nullptr;
  }
  const char* path = (core_library != nullptr && core_library[0] != '\0')
                         ? core_library
                         : "libOmxCore.so";
  std::unique_ptr<OmxDecoder> dec(new OmxDecoder());
  State* s = dec->s_.get();
  s->extra_outputs = extra_output_buffers;
  s->watch_color =
      codec_fourcc == V4L2_PIX_FMT_H264 || codec_fourcc == V4L2_PIX_FMT_HEVC;
  s->color_codec =
      codec_fourcc == V4L2_PIX_FMT_HEVC ? SpsCodec::kH265 : SpsCodec::kH264;

  // RTLD_GLOBAL: the core dlopens its components, which resolve symbols
  // against it. Never unloaded: the components start threads of their own.
  s->lib = dlopen(path, RTLD_NOW | RTLD_GLOBAL);
  if (s->lib == nullptr) {
    V4L2WC_LOG(V4L2WC_INFO) << "omx: no OpenMAX core (" << path << ")";
    return nullptr;
  }
#define V4L2WC_OMX_SYM(member, name) \
  s->member = reinterpret_cast<decltype(s->member)>(dlsym(s->lib, name))
  V4L2WC_OMX_SYM(init, "OMX_Init");
  V4L2WC_OMX_SYM(deinit, "OMX_Deinit");
  V4L2WC_OMX_SYM(components_of_role, "OMX_GetComponentsOfRole");
  V4L2WC_OMX_SYM(get_handle, "OMX_GetHandle");
  V4L2WC_OMX_SYM(free_handle, "OMX_FreeHandle");
#undef V4L2WC_OMX_SYM
  if (!s->init || !s->deinit || !s->components_of_role || !s->get_handle ||
      !s->free_handle) {
    V4L2WC_LOG(V4L2WC_ERROR) << "omx: " << path << " is not an OpenMAX core";
    return nullptr;
  }
  if (s->init() != OMX_ErrorNone) {
    V4L2WC_LOG(V4L2WC_ERROR) << "omx: OMX_Init failed";
    return nullptr;
  }
  s->inited = true;

  // The first component for the role that is not a protected-content one.
  char role_name[OMX_MAX_STRINGNAME_SIZE];
  std::strncpy(role_name, role, sizeof(role_name) - 1);
  role_name[sizeof(role_name) - 1] = '\0';
  OMX_U32 count = 0;
  if (s->components_of_role(role_name, &count, nullptr) != OMX_ErrorNone ||
      count == 0) {
    V4L2WC_LOG(V4L2WC_INFO) << "omx: no component for " << role;
    return nullptr;
  }
  if (count > 16) {
    count = 16;
  }
  std::vector<std::vector<char>> names(
      count, std::vector<char>(OMX_MAX_STRINGNAME_SIZE));
  std::vector<OMX_U8*> name_ptrs;
  name_ptrs.reserve(names.size());
  for (auto& n : names) {
    name_ptrs.push_back(reinterpret_cast<OMX_U8*>(n.data()));
  }
  if (s->components_of_role(role_name, &count, name_ptrs.data()) !=
      OMX_ErrorNone) {
    return nullptr;
  }
  std::string component;
  for (OMX_U32 i = 0; i < count && i < names.size(); ++i) {
    names[i].back() = '\0';
    const std::string n = names[i].data();
    if (n.find(".secure") == std::string::npos) {
      component = n;
      break;
    }
  }
  if (component.empty()) {
    return nullptr;
  }
  s->qcom = component.rfind("OMX.qcom.", 0) == 0 ||
            component.rfind("OMX.qti.", 0) == 0;
  if (!s->qcom) {
    // Where a component keeps the dma-buf behind an output buffer is its
    // vendor's business, and this engine knows one vendor's. Any other is
    // refused here, before the component is asked to do anything: refusing
    // it later, with buffers half allocated, means tearing it down in the
    // middle of a state change, and a component may not survive that
    // (StarFive's OMX_FreeHandle does not return).
    V4L2WC_LOG(V4L2WC_INFO)
        << "omx: " << component << ": no known way to its output dma-bufs";
    return nullptr;
  }

  static OMX_CALLBACKTYPE callbacks = {&State::OnEvent, &State::OnEmptyDone,
                                       &State::OnFillDone};
  std::vector<char> component_name(component.begin(), component.end());
  component_name.push_back('\0');
  if (s->get_handle(&s->handle, component_name.data(), s, &callbacks) !=
          OMX_ErrorNone ||
      s->handle == nullptr) {
    s->handle = nullptr;
    V4L2WC_LOG(V4L2WC_ERROR) << "omx: GetHandle " << component << " failed";
    return nullptr;
  }

  // Input: the picture size as a hint. The component knows its own codec.
  OMX_PARAM_PORTDEFINITIONTYPE in_def;
  InitParam(&in_def);
  in_def.nPortIndex = kInPort;
  if (OMX_GetParameter(s->handle, OMX_IndexParamPortDefinition, &in_def) !=
      OMX_ErrorNone) {
    return nullptr;
  }
  if (coded_width > 0 && coded_height > 0) {
    in_def.format.video.nFrameWidth = coded_width;
    in_def.format.video.nFrameHeight = coded_height;
    OMX_SetParameter(s->handle, OMX_IndexParamPortDefinition, &in_def);
    if (OMX_GetParameter(s->handle, OMX_IndexParamPortDefinition, &in_def) !=
        OMX_ErrorNone) {
      return nullptr;
    }
  }

  // Output: linear NV12. A vendor's compressed layout is its default and is
  // not something a dma-buf consumer can be told about.
  OMX_U32 chosen = 0;
  for (OMX_U32 i = 0; i < 32; ++i) {
    OMX_VIDEO_PARAM_PORTFORMATTYPE fmt;
    InitParam(&fmt);
    fmt.nPortIndex = kOutPort;
    fmt.nIndex = i;
    if (OMX_GetParameter(s->handle, OMX_IndexParamVideoPortFormat, &fmt) !=
        OMX_ErrorNone) {
      break;
    }
    const auto color = static_cast<OMX_U32>(fmt.eColorFormat);
    if (s->qcom && color == kQcomNv12Venus) {
      chosen = color;
      break;
    }
    if (color == OMX_COLOR_FormatYUV420SemiPlanar && chosen == 0) {
      chosen = color;
    }
  }
  if (chosen == 0) {
    V4L2WC_LOG(V4L2WC_ERROR) << "omx: " << component << " has no linear NV12";
    return nullptr;
  }
  OMX_VIDEO_PARAM_PORTFORMATTYPE fmt;
  InitParam(&fmt);
  fmt.nPortIndex = kOutPort;
  fmt.eColorFormat = static_cast<OMX_COLOR_FORMATTYPE>(chosen);
  if (OMX_SetParameter(s->handle, OMX_IndexParamVideoPortFormat, &fmt) !=
      OMX_ErrorNone) {
    V4L2WC_LOG(V4L2WC_ERROR) << "omx: output format refused";
    return nullptr;
  }

  // Loaded -> idle (buffers are allocated during the transition) -> executing.
  if (!s->Send(OMX_CommandStateSet, OMX_StateIdle)) {
    return nullptr;
  }
  for (OMX_U32 i = 0; i < in_def.nBufferCountActual; ++i) {
    OMX_BUFFERHEADERTYPE* h = nullptr;
    if (OMX_AllocateBuffer(s->handle, &h, kInPort, nullptr,
                           in_def.nBufferSize) != OMX_ErrorNone) {
      V4L2WC_LOG(V4L2WC_ERROR) << "omx: input AllocateBuffer failed";
      return nullptr;
    }
    s->in_all.push_back(h);
    s->in_free.push_back(h);
  }
  if (!s->AllocateOutput() || !s->WaitCommand(OMX_CommandStateSet) ||
      !s->Send(OMX_CommandStateSet, OMX_StateExecuting) ||
      !s->WaitCommand(OMX_CommandStateSet)) {
    return nullptr;
  }
  s->executing = true;
  s->ReadCrop();
  {
    std::lock_guard<std::mutex> lock(s->mu);
    s->out_enabled = true;
  }
  s->FillIdle();
  V4L2WC_LOG(V4L2WC_INFO) << "omx: " << component << " executing";
  return dec;
}

OmxDecoder::~OmxDecoder() {
  State* s = s_.get();
  if (s->handle != nullptr) {
    {
      std::lock_guard<std::mutex> lock(s->mu);
      s->out_enabled = false;
      s->flushing = true;
      // Results of anything still in flight do not matter now.
      for (int& c : s->command_done) {
        c = 0;
      }
      s->error = false;
    }
    // Executing -> idle returns every buffer; idle -> loaded completes once
    // they are freed. A decoder that never got as far as executing just gives
    // back what it allocated.
    if (s->executing) {
      if (OMX_SendCommand(s->handle, OMX_CommandStateSet, OMX_StateIdle,
                          nullptr) == OMX_ErrorNone) {
        s->WaitCommand(OMX_CommandStateSet);
      }
      OMX_SendCommand(s->handle, OMX_CommandStateSet, OMX_StateLoaded, nullptr);
    }
    for (OMX_BUFFERHEADERTYPE* h : s->in_all) {
      OMX_FreeBuffer(s->handle, kInPort, h);
    }
    s->FreeOutput();
    if (s->executing) {
      s->WaitCommand(OMX_CommandStateSet);
    }
    s->free_handle(s->handle);
    s->handle = nullptr;
  }
  if (s->inited) {
    s->deinit();
  }
}

SubmitResult OmxDecoder::SubmitBitstream(const std::uint8_t* data,
                                         std::size_t size,
                                         std::uint64_t timestamp) {
  State* s = s_.get();
  if (data == nullptr || size == 0) {
    return SubmitResult::kError;
  }
  if (s->need_recreate) {
    return SubmitResult::kSourceChange;
  }
  {
    std::lock_guard<std::mutex> lock(s->mu);
    if (s->error) {
      return SubmitResult::kError;
    }
    if (s->port_changed) {
      return SubmitResult::kTryAgain;  // Drive() reconfigures first
    }
  }
  if (!s->settled && s->first_sent) {
    if (std::chrono::steady_clock::now() - s->first_submit < kSettleWait) {
      return SubmitResult::kTryAgain;  // one unit in flight until it settles
    }
    s->settled = true;  // it is not going to reconfigure
  }
  // The pictures are taken as 8-bit NV12, which a deeper stream's are not:
  // Qualcomm's component gives a 10-bit HEVC stream's in its own layout, and
  // read as NV12 they are wrong colors throughout. Refused at its SPS.
  SpsColor color;
  const bool said =
      s->watch_color && FindSpsColor(s->color_codec, data, size, &color);
  if (said && (color.bit_depth_luma > 8 || color.bit_depth_chroma > 8)) {
    V4L2WC_LOG(V4L2WC_ERROR)
        << "omx: the stream is " << static_cast<int>(color.bit_depth_luma)
        << "-bit; only 8-bit pictures are taken here";
    std::lock_guard<std::mutex> lock(s->mu);
    s->error = true;
    return SubmitResult::kError;
  }
  if (s->eos_sent) {
    // Decoding again after a drain: the buffers parked at end of stream go
    // back to the component.
    s->eos_sent = false;
    s->FillIdle();
  }
  bool too_big = false;
  if (!s->Queue(data, size, timestamp, OMX_BUFFERFLAG_ENDOFFRAME, &too_big)) {
    if (too_big) {
      V4L2WC_LOG(V4L2WC_ERROR) << "omx: access unit of " << size
                               << " bytes exceeds the input buffer";
      return SubmitResult::kError;
    }
    std::lock_guard<std::mutex> lock(s->mu);
    return s->error ? SubmitResult::kError : SubmitResult::kTryAgain;
  }
  if (!s->first_sent) {
    s->first_sent = true;
    s->first_submit = std::chrono::steady_clock::now();
  }
  if (said) {
    std::lock_guard<std::mutex> lock(s->mu);  // Acquire reads it under mu
    s->color = color;
  }
  return SubmitResult::kOk;
}

DriveResult OmxDecoder::Drive() {
  State* s = s_.get();
  if (s->need_recreate) {
    return DriveResult::kSourceChange;
  }
  bool port_changed = false;
  bool crop_changed = false;
  bool have_frame = false;
  bool have_returned = false;
  {
    std::lock_guard<std::mutex> lock(s->mu);
    if (s->error) {
      return DriveResult::kError;
    }
    port_changed = s->port_changed;
    crop_changed = s->crop_changed;
    s->crop_changed = false;
    have_frame = !s->ready.empty();
    have_returned = !s->returned.empty();
  }
  if (port_changed) {
    bool client_holds = false;
    {
      std::lock_guard<std::mutex> lock(s->mu);
      for (const State::OutBuffer& b : s->out) {
        client_holds = client_holds || b.owner == State::Owner::kClient;
      }
    }
    if (s->frames_out > 0 || client_holds) {
      // Frames are out there at the old geometry. The consumer rebuilds.
      s->need_recreate = true;
      return DriveResult::kSourceChange;
    }
    if (!s->ReconfigureOutput()) {
      std::lock_guard<std::mutex> lock(s->mu);
      s->error = true;
      return DriveResult::kError;
    }
    s->settled = true;
    return DriveResult::kOk;
  }
  if (crop_changed) {
    s->ReadCrop();
  }
  if (have_frame) {
    s->settled = true;
  }
  if (s->drain_pending) {
    if (s->Queue(nullptr, 0, 0, OMX_BUFFERFLAG_EOS, nullptr)) {
      s->drain_pending = false;
      s->eos_sent = true;
    }
  }
  // Buffers that came back empty go round again, except at end of stream,
  // where the component would only hand them straight back.
  if (have_returned && !s->eos_sent) {
    s->FillIdle();
  }
  return DriveResult::kOk;
}

bool OmxDecoder::Acquire(V4l2DmaFrame* out) {
  State* s = s_.get();
  if (out == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(s->mu);
  if (s->ready.empty()) {
    return false;
  }
  const std::uint32_t index = s->ready.front();
  s->ready.pop_front();
  State::OutBuffer& b = s->out[index];
  b.owner = State::Owner::kClient;
  ++s->frames_out;

  *out = V4l2DmaFrame{};
  out->capture_index = index;
  out->width = s->width;
  out->height = s->height;
  // A crop that does not start at the top left is taken from the stream's
  // SPS. Qualcomm's component gives the rectangle's right and bottom edges
  // for its size, and for some offsets a left of 0: cropped 32 left and 8
  // top, it said 672x376 at 0,8 for a 640x368 picture at 32,8.
  const SpsColor& c = s->color;
  if ((c.crop_left != 0 || c.crop_top != 0) && c.width > 0 && c.height > 0 &&
      std::uint64_t{c.crop_left} + c.width <= s->stride &&
      std::uint64_t{c.crop_top} + c.height <= s->slice_height) {
    out->width = c.width;
    out->height = c.height;
    out->crop_x = c.crop_left;
    out->crop_y = c.crop_top;
  }
  out->drm_fourcc = V4L2_PIX_FMT_NV12;  // the same fourcc as DRM_FORMAT_NV12
  out->modifier = 0;                    // DRM_FORMAT_MOD_LINEAR
  out->timestamp = b.timestamp;
  out->colour_primaries = s->color.colour_primaries;
  out->transfer_characteristics = s->color.transfer_characteristics;
  out->matrix_coefficients = s->color.matrix_coeffs;
  out->video_full_range = s->color.video_full_range_flag;
  out->num_planes = 2;
  out->fds[0] = b.fd;
  out->offsets[0] = b.offset;
  out->pitches[0] = s->stride;
  out->fds[1] = b.fd;
  out->offsets[1] = b.offset + s->stride * s->slice_height;
  out->pitches[1] = s->stride;
  return true;
}

void OmxDecoder::Release(std::uint32_t capture_index) {
  State* s = s_.get();
  {
    std::lock_guard<std::mutex> lock(s->mu);
    if (capture_index >= s->out.size() ||
        s->out[capture_index].owner != State::Owner::kClient) {
      return;
    }
    s->out[capture_index].owner = State::Owner::kIdle;
  }
  if (!s->eos_sent && !s->need_recreate) {
    s->FillIdle();
  }
}

void OmxDecoder::Flush() {
  State* s = s_.get();
  if (s->need_recreate) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(s->mu);
    if (s->error) {
      return;
    }
    s->flushing = true;
  }
  // One completion per port.
  if (s->Send(OMX_CommandFlush, OMX_ALL)) {
    s->WaitCommand(OMX_CommandFlush, 2);
  }
  {
    std::lock_guard<std::mutex> lock(s->mu);
    // Frames decoded and not yet acquired are dropped with the rest.
    for (std::uint32_t index : s->ready) {
      s->out[index].owner = State::Owner::kIdle;
    }
    s->ready.clear();
    s->flushing = false;
  }
  s->drain_pending = false;
  s->eos_sent = false;
  s->FillIdle();
}

void OmxDecoder::Drain() {
  State* s = s_.get();
  if (s->eos_sent || s->need_recreate) {
    return;
  }
  if (s->Queue(nullptr, 0, 0, OMX_BUFFERFLAG_EOS, nullptr)) {
    s->eos_sent = true;
  } else {
    s->drain_pending = true;  // Drive() sends it when an input buffer is free
  }
}

std::uint32_t OmxDecoder::PoolSize() const {
  std::lock_guard<std::mutex> lock(s_->mu);
  return static_cast<std::uint32_t>(s_->out.size());
}

}  // namespace v4l2wc
