#pragma once
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <string>

namespace logging = boost::log;
namespace keywords = boost::log::keywords;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
using namespace logging::trivial;
static src::severity_logger<severity_level> lg;
class Logger {
 public:
  Logger() = default;
  void Init(std::string filename, int severity) {
    this->core = logging::core::get();
    logging::add_common_attributes();
    logging::core::get()->set_filter(logging::trivial::severity >= severity);

    if (filename != "")
      logging::add_file_log(
          keywords::file_name = filename,
          keywords::format = "[%TimeStamp%]: *%Severity%* %Message%");
  }
  void Flush() { this->core->flush(); }

  template<typename T, typename... Args>
  void Debug(T arg, Args... args) {
      BOOST_LOG_SEV(lg, debug) << arg << " ";
      Debug(args...);
  }

  template <typename... Args>
  void Info(Args&&... args) {
      std::ostringstream oss;
      ((oss << args << " "), ...);
      BOOST_LOG_TRIVIAL(info) << oss.str();
  }

  template <typename... Args>
  void Warn(Args&&... args) {
      std::ostringstream oss;
      ((oss << args << " "), ...);
      BOOST_LOG_TRIVIAL(warning) << oss.str();
  }
 
  template <typename... Args>
  void Debug(Args&&... args) {
      std::ostringstream oss;
      ((oss << args << " "), ...);
      BOOST_LOG_TRIVIAL(debug) << oss.str();
  }


  template <typename... Args>
  void Error(Args&&... args) {
      std::ostringstream oss;
      ((oss << args << " "), ...);
      BOOST_LOG_TRIVIAL(error) << oss.str();
  }

  template <typename... Args>
  void Fatal(Args&&... args) {
      std::ostringstream oss;
      ((oss << args << " "), ...);
      BOOST_LOG_TRIVIAL(fatal) << oss.str();
  }

 private:
  logging::core_ptr core;
};

extern Logger g_logger;

// #define DEV_DEBUG(msg, args...) ONLYDEV(g_logger.Debug(msg, ##args);)
#define DEV_INFO(msg, args...) ONLYDEV(g_logger.Info(msg, ##args);)
#define DEV_WARN(msg, args...) ONLYDEV(g_logger.Warn(msg, ##args);)
#define DEV_ERROR(msg, args...) ONLYDEV(g_logger.Error(msg, ##args);)
#define DEV_FLUSH() ONLYDEV(g_logger.Flush();)
