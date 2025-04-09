#pragma once

#include <chrono>
#include <exception>
#include <iostream>
#include <thread>

#include <glog/logging.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

// Issue #7: Glog version > 0.5.0 requires T=size_t, <= 0.5.0 T=int
template <typename T>
void PyBindLogStack(const char* data, T size) {
  std::chrono::milliseconds timespan(2000);  // or whatever
  py::scoped_estream_redirect stream(
      std::cerr,                                // std::ostream&
      py::module::import("sys").attr("stderr")  // Python output
  );
  std::this_thread::sleep_for(timespan);
  std::this_thread::sleep_for(timespan);

  std::cerr << data << std::endl;
  std::cerr << std::endl;
  std::cerr
      << "ERROR: C++ code terminated. Kernel Died. See log files for details.";
  std::cerr << std::endl << std::endl << std::endl;
}

#ifdef __GNUC__
__attribute__((noreturn))
#endif
void PyBindLogTermination() {
  std::chrono::milliseconds timespan(2000);  // or whatever
  py::scoped_estream_redirect stream(
      std::cerr,                                // std::ostream&
      py::module::import("sys").attr("stderr")  // Python output
  );
  std::this_thread::sleep_for(timespan);
  std::this_thread::sleep_for(timespan);

  std::cerr << std::endl;
  std::cerr
      << "ERROR: C++ code terminated. Kernel Died. See log files for details.";
  std::cerr << std::endl << std::endl << std::endl;
  exit(1);
}

// Alternative checks to throw an exception instead of aborting the program.
// Usage: THROW_CHECK(condition) << message;
//        THROW_CHECK_EQ(val1, val2) << message;
//        LOG(FATAL_THROW) << message;
// These macros are copied from glog/logging.h and extended to a new severity
// level FATAL_THROW.
#define COMPACT_GOOGLE_LOG_FATAL_THROW \
  LogMessageFatalThrowDefault(__FILE__, __LINE__)

#define LOG_TO_STRING_FATAL_THROW(message) \
  LogMessageFatalThrowDefault(__FILE__, __LINE__, message)

#define LOG_FATAL_THROW(exception) \
  LogMessageFatalThrow<exception>(__FILE__, __LINE__).stream()

#define THROW_CHECK(condition)                                       \
  LOG_IF(FATAL_THROW, GOOGLE_PREDICT_BRANCH_NOT_TAKEN(!(condition))) \
      << "Check failed: " #condition " "

#define THROW_CHECK_OP(name, op, val1, val2) \
  CHECK_OP_LOG(name, op, val1, val2, LogMessageFatalThrowDefault)

#define THROW_CHECK_EQ(val1, val2) THROW_CHECK_OP(_EQ, ==, val1, val2)
#define THROW_CHECK_NE(val1, val2) THROW_CHECK_OP(_NE, !=, val1, val2)
#define THROW_CHECK_LE(val1, val2) THROW_CHECK_OP(_LE, <=, val1, val2)
#define THROW_CHECK_LT(val1, val2) THROW_CHECK_OP(_LT, <, val1, val2)
#define THROW_CHECK_GE(val1, val2) THROW_CHECK_OP(_GE, >=, val1, val2)
#define THROW_CHECK_GT(val1, val2) THROW_CHECK_OP(_GT, >, val1, val2)

#define THROW_CHECK_NOTNULL(val) \
  ThrowCheckNotNull(__FILE__, __LINE__, "'" #val "' Must be non NULL", (val))

const char* __GetConstFileBaseName(const char* file) {
  const char* base = strrchr(file, '/');
  if (!base) {
    base = strrchr(file, '\\');
  }
  return base ? (base + 1) : file;
}

inline std::string __MakeExceptionPrefix(const char* file, int line) {
  return "[" + std::string(__GetConstFileBaseName(file)) + ":" +
         std::to_string(line) + "] ";
}

template <typename T>
class LogMessageFatalThrow : public google::LogMessage {
 public:
  LogMessageFatalThrow(const char* file, int line)
      : google::LogMessage(file, line, google::GLOG_ERROR, &message_),
        prefix_(__MakeExceptionPrefix(file, line)) {}
  LogMessageFatalThrow(const char* file, int line, std::string* message)
      : google::LogMessage(file, line, google::GLOG_ERROR, message),
        message_(*message),
        prefix_(__MakeExceptionPrefix(file, line)) {}
  LogMessageFatalThrow(const char* file,
                       int line,
#if defined(GLOG_VERSION_MAJOR) && \
    (GLOG_VERSION_MAJOR > 0 || GLOG_VERSION_MINOR >= 7)
                       const google::logging::internal::CheckOpString& result)
#else
                       const google::CheckOpString& result)
#endif
      : google::LogMessage(file, line, google::GLOG_ERROR, &message_),
        prefix_(__MakeExceptionPrefix(file, line)) {
    stream() << "Check failed: " << (*result.str_) << " ";
    // On LOG(FATAL) glog<0.7.0 does not bother cleaning up CheckOpString
    // so we do it here.
#if !(defined(GLOG_VERSION_MAJOR) && \
      (GLOG_VERSION_MAJOR > 0 || GLOG_VERSION_MINOR >= 7))
    delete result.str_;
#endif
  };
  ~LogMessageFatalThrow() noexcept(false) {
    Flush();
#if defined(__cpp_lib_uncaught_exceptions) && \
    (__cpp_lib_uncaught_exceptions >= 201411L)
    if (std::uncaught_exceptions() == 0)
#else
    if (!std::uncaught_exception())
#endif
    {
      throw T(prefix_ + message_);
    }
  };

 private:
  std::string message_;
  std::string prefix_;
};

using LogMessageFatalThrowDefault = LogMessageFatalThrow<std::invalid_argument>;

template <typename T>
T ThrowCheckNotNull(const char* file, int line, const char* names, T&& t) {
  if (t == nullptr) {
    LogMessageFatalThrowDefault(file, line).stream() << names;
  }
  return std::forward<T>(t);
}

struct Logging {
  enum class LogSeverity {
    GLOG_INFO = google::GLOG_INFO,
    GLOG_WARNING = google::GLOG_WARNING,
    GLOG_ERROR = google::GLOG_ERROR,
    GLOG_FATAL = google::GLOG_FATAL,
  };
};  // dummy class

std::pair<std::string, int> GetPythonCallFrame() {
  const auto frame = py::module_::import("sys").attr("_getframe")(0);
  const std::string file = py::str(frame.attr("f_code").attr("co_filename"));
  const std::string function = py::str(frame.attr("f_code").attr("co_name"));
  const int line = py::int_(frame.attr("f_lineno"));
  return std::make_pair(file + ":" + function, line);
}

void BindLogging(py::module& m) {
  py::class_<Logging> PyLogging(m, "logging", py::module_local());

  py::enum_<Logging::LogSeverity>(PyLogging, "Level", py::module_local())
      .value("INFO", Logging::LogSeverity::GLOG_INFO)
      .value("WARNING", Logging::LogSeverity::GLOG_WARNING)
      .value("ERROR", Logging::LogSeverity::GLOG_ERROR)
      .value("FATAL", Logging::LogSeverity::GLOG_FATAL)
      .export_values();

  PyLogging.def_readwrite_static("minloglevel", &FLAGS_minloglevel)
      .def_readwrite_static("stderrthreshold", &FLAGS_stderrthreshold)
      .def_readwrite_static("log_dir", &FLAGS_log_dir)
      .def_readwrite_static("logtostderr", &FLAGS_logtostderr)
      .def_readwrite_static("alsologtostderr", &FLAGS_alsologtostderr)
      .def_static(
          "set_log_destination",
          [](const Logging::LogSeverity severity, const std::string& path) {
            google::SetLogDestination(
                static_cast<google::LogSeverity>(severity), path.c_str());
          })
      .def_static(
          "info",
          [](const std::string& msg) {
            auto frame = GetPythonCallFrame();
            google::LogMessage(frame.first.c_str(), frame.second).stream()
                << msg;
          })
      .def_static("warning",
                  [](const std::string& msg) {
                    auto frame = GetPythonCallFrame();
                    google::LogMessage(
                        frame.first.c_str(), frame.second, google::GLOG_WARNING)
                            .stream()
                        << msg;
                  })
      .def_static("error",
                  [](const std::string& msg) {
                    auto frame = GetPythonCallFrame();
                    google::LogMessage(
                        frame.first.c_str(), frame.second, google::GLOG_ERROR)
                            .stream()
                        << msg;
                  })
      .def_static(
          "fatal",
          [](const std::string& msg) {
            auto frame = GetPythonCallFrame();
            google::LogMessageFatal(frame.first.c_str(), frame.second).stream()
                << msg;
          })
      .def("install_failure_writer",
           []() { google::InstallFailureWriter(&PyBindLogStack); })
      .def("install_failure_function",
           []() { google::InstallFailureFunction(&PyBindLogTermination); });

#if defined(GLOG_VERSION_MAJOR) && \
    (GLOG_VERSION_MAJOR > 0 || GLOG_VERSION_MINOR >= 6)
  if (!google::IsGoogleLoggingInitialized())
#else
  // Check whether pycolmap has already been imported.
  if (!py::module_::import("sys").attr("modules").contains("pycolmap"))
#endif
  {
    google::InitGoogleLogging("");
    google::InstallFailureSignalHandler();
    google::InstallFailureFunction(&PyBindLogTermination);
  }
  FLAGS_alsologtostderr = true;
}
