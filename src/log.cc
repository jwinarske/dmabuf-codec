// SPDX-FileCopyrightText: 2026 Joel Winarske
// SPDX-License-Identifier: MIT

#include "src/log.h"

#include <cstdio>
#include <string>

namespace v4l2wc {
namespace {

const char* Label(LogSeverity severity) {
  switch (severity) {
    case LogSeverity::kWarning:
      return "WARN";
    case LogSeverity::kError:
      return "ERROR";
    case LogSeverity::kInfo:
      break;
  }
  return "INFO";
}

void DefaultSink(LogSeverity severity, const char* message) {
  std::fprintf(stderr, "[v4l2wc %s] %s\n", Label(severity), message);
}

LogSink g_sink = &DefaultSink;

}  // namespace

void SetLogSink(LogSink sink) {
  g_sink = sink != nullptr ? sink : &DefaultSink;
}

LogMessage::~LogMessage() {
  const std::string text = stream_.str();
  g_sink(severity_, text.c_str());
}

}  // namespace v4l2wc
