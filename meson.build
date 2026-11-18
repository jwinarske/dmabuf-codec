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
# BUILD.gn. Fuzzers and the on-hardware tests stay with CMake.
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
  ),
  include_directories : inc,
)

core_sources = files(
  'src/dma_decoder.cc',
  'src/dma_decoder_factory.cc',
  'src/log.cc',
  'src/v4l2_m2m_decoder.cc',
  'src/v4l2_stateless_h265_decoder.cc',
)
core_deps = []
dl_dep = meson.get_compiler('cpp').find_library('dl', required : false)

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
  core_deps += [
    va_dep.partial_dependency(compile_args : true, includes : true),
    dl_dep,
  ]
endif
have_vaapi_define = '-DV4L2WC_HAVE_VAAPI=@0@'.format(have_vaapi ? 1 : 0)

core_lib = static_library(
  'v4l2wc_core',
  core_sources,
  include_directories : inc,
  link_with : parse_lib,
  dependencies : core_deps,
  cpp_args : have_vaapi_define,
)

# The decode seam (src/dma_decoder.h), its engines and the parsers. Headers are
# included from the repository root, e.g. "src/dma_decoder_factory.h".
dmabuf_codec_dep = declare_dependency(
  include_directories : include_directories('.', is_system : true),
  link_with : [core_lib, parse_lib],
  compile_args : have_vaapi_define,
  dependencies : have_vaapi ? [dl_dep] : [],
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

summary({'VAAPI engines' : have_vaapi, 'tests' : build_tests}, bool_yn : true)
