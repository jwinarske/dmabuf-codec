# SPDX-FileCopyrightText: 2026 Joel Winarske
# SPDX-License-Identifier: MIT

# Meson build of the same host-buildable layers CMakeLists.txt builds: the
# bitstream parsers (v4l2wc_parse) and the webrtc-free decode core
# (v4l2wc_core). For a consumer that builds with Meson and takes this repo as a
# subproject:
#
#   dmabuf_codec_dep = dependency('dmabuf-codec',
#     fallback : ['dmabuf-codec', 'dmabuf_codec_dep'])
#
# The webrtc wrapper is not built here; it is absorbed into libwebrtc.so via
# BUILD.gn. Fuzzers and the VAAPI on-hardware tests stay with CMake.
project(
  'dmabuf-codec',
  'cpp',
  version : '0.1.0',
  license : 'MIT',
  meson_version : '>=0.63',
  default_options : ['cpp_std=c++17', 'warning_level=2'],
)

inc = include_directories('.')

parse_lib = static_library(
  'v4l2wc_parse',
  files(
    'parse/h264/nal.cc',
    'parse/h264/pps.cc',
    'parse/h264/slice_header.cc',
    'parse/h264/sps.cc',
    'parse/h265/nal.cc',
    'parse/h265/pps.cc',
    'parse/h265/scaling_list.cc',
    'parse/h265/slice_header.cc',
    'parse/h265/sps.cc',
    'parse/sps_color.cc',
  ),
  include_directories : inc,
)

core_sources = files(
  'src/dma_decoder.cc',
  'src/dma_decoder_factory.cc',
  'src/log.cc',
  'src/omx_decoder.cc',
  'src/v4l2_m2m_decoder.cc',
)
dl_dep = meson.get_compiler('cpp').find_library('dl', required : false)
# The OpenMAX engine loads the device's core at run time, so it always builds:
# it needs the IL headers (vendored), the dynamic loader and threads.
core_deps = [dl_dep, dependency('threads')]
omx_inc = include_directories('third_party/openmax_il', is_system : true)

# VAAPI needs the libva headers at build time only: the engine dlopens the
# library itself at runtime. Without them only the V4L2 engines are built.
va_dep = dependency('libva', required : get_option('vaapi'))
have_vaapi = va_dep.found()
if have_vaapi
  core_sources += files(
    'src/va_loader.cc',
    'src/vaapi_h264_decoder.cc',
    'src/vaapi_h265_decoder.cc',
  )
  core_deps += [va_dep.partial_dependency(compile_args : true, includes : true)]
endif
have_vaapi_define = '-DV4L2WC_HAVE_VAAPI=@0@'.format(have_vaapi ? 1 : 0)

# The stateless HEVC engine is written to the kernel's HEVC request controls,
# which became stable API in Linux 6.0. A sysroot with older kernel headers
# builds without that engine.
have_stateless_hevc = meson.get_compiler('cpp').has_header_symbol(
  'linux/v4l2-controls.h',
  'V4L2_CID_STATELESS_HEVC_SPS',
)
if have_stateless_hevc
  core_sources += files('src/v4l2_stateless_h265_decoder.cc')
endif
have_stateless_hevc_define = '-DV4L2WC_HAVE_STATELESS_HEVC=@0@'.format(
  have_stateless_hevc ? 1 : 0,
)

core_lib = static_library(
  'v4l2wc_core',
  core_sources,
  include_directories : [inc, omx_inc],
  link_with : parse_lib,
  dependencies : core_deps,
  cpp_args : [have_vaapi_define, have_stateless_hevc_define],
)

# The decode seam (src/dma_decoder.h), its engines and the parsers. Headers are
# included from the repository root, e.g. "src/dma_decoder_factory.h".
dmabuf_codec_dep = declare_dependency(
  include_directories : include_directories('.', is_system : true),
  link_with : [core_lib, parse_lib],
  compile_args : have_vaapi_define,
  dependencies : [dl_dep, dependency('threads')],
  version : meson.project_version(),
)
meson.override_dependency('dmabuf-codec', dmabuf_codec_dep)

# A consumer wants only the libraries. The tests build when this is the
# top-level project, or when asked for.
build_tests = get_option('tests').enabled() or (
  get_option('tests').auto() and not meson.is_subproject()
)
if build_tests
  foreach t : [
    'h264_nal',
    'h264_pps',
    'h264_slice_header',
    'h264_sps',
    'h265_nal',
    'h265_pps',
    'h265_scaling_list',
    'h265_slice_header',
    'h265_sps',
    'sps_color',
  ]
    test(
      t + '_test',
      executable(
        t + '_test',
        'test/parse/' + t + '_test.cc',
        include_directories : inc,
        link_with : parse_lib,
      ),
    )
  endforeach
  test(
    'v4l2wc_core_test',
    executable(
      'v4l2wc_core_test',
      'test/core/core_smoke_test.cc',
      dependencies : dmabuf_codec_dep,
    ),
  )
endif

# Not registered as tests: they need a decoder on the machine and a clip.
#   dma_decode_hw_test h264|hevc clip
#   kms_writeback_hw_test h264|hevc clip ref.yuv WIDTHxHEIGHT
# The second also needs a card with a writeback connector, and libdrm to
# build; it is left out where libdrm is not found.
if get_option('hw_tests')
  executable(
    'dma_decode_hw_test',
    'test/hw/dma_decode_hw_test.cc',
    dependencies : dmabuf_codec_dep,
  )
  libdrm_dep = dependency('libdrm', required : false)
  if libdrm_dep.found()
    executable(
      'kms_writeback_hw_test',
      'test/hw/kms_writeback_hw_test.cc',
      dependencies : [dmabuf_codec_dep, libdrm_dep],
    )
  endif
endif

summary(
  {
    'VAAPI engines' : have_vaapi,
    'stateless HEVC engine' : have_stateless_hevc,
    'tests' : build_tests,
  },
  bool_yn : true,
)
