<!--
SPDX-FileCopyrightText: 2026 Joel Winarske
SPDX-License-Identifier: MIT
-->

# vendored OpenMAX IL headers

The OpenMAX IL 1.1.2 headers the OpenMAX engine (`src/omx_decoder.cc`) is built
against. Nine files, verbatim from the Android Open Source Project,
`platform/frameworks/native` at `d6b05b5758e8`, directory
`headers/media_plugin/media/openmax/`. Do not edit them here.

Each file carries its own notices: the Khronos Group's permissive license
(2008) and, on top, PacketVideo's Apache-2.0 notice (1998-2009).

Why this copy and not the one in the Khronos registry: it declares the IL's
integer types at fixed width (`OMX_U32` is `uint32_t`), which is the ABI the
vendor cores on 64-bit Linux boards are built with. The registry's headers
declare `OMX_U32` as `unsigned long`, which is 64 bits there and does not
match.

Only headers: nothing is linked. The engine loads the device's OpenMAX core
(`libOmxCore.so`) at run time.
