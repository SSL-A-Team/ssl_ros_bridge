#ifndef LOG_READER_HPP_
#define LOG_READER_HPP_

#include <istream>
#include <optional>
#include <variant>
#include <ssl_league_protobufs/ssl_vision_wrapper.pb.h>
#include <ssl_league_protobufs/ssl_gc_referee_message.pb.h>

namespace ssl_ros_bridge
{

enum class EntryType : int32_t {
  Blank = 0,
  Unknown = 1,
  Vision2010 = 2,
  Refbox2013 = 3,
  Vision2014 = 4,
  VisionTracker2020 = 5,
  Index2021 = 6
};

struct LogEntry {
  int64_t received_time_ns;
  std::variant<std::monostate, Referee, SSL_WrapperPacket> message;
};

class LogReader {
public:

  explicit LogReader(std::istream & input);

  std::optional<LogEntry> GetNextMessage();

  const std::unordered_map<EntryType, unsigned long> & GetEntryTypeCounts() const {
    return entry_type_counts;
  }

private:
  std::istream & input_stream;
  std::unordered_map<EntryType, unsigned long> entry_type_counts;

  void CheckHeader();

  LogEntry ExtractNextEntry();

};

}  // namespace ssl_ros_bridge

#endif  // LOG_READER_HPP_
