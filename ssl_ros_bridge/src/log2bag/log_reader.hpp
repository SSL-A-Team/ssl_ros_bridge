// Copyright 2025 A Team
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef LOG2BAG__LOG_READER_HPP_
#define LOG2BAG__LOG_READER_HPP_

#include <ssl_league_protobufs/ssl_vision_wrapper.pb.h>
#include <ssl_league_protobufs/ssl_gc_referee_message.pb.h>
#include <istream>
#include <optional>
#include <unordered_map>
#include <variant>

namespace ssl_ros_bridge
{

enum class EntryType : int32_t
{
  Blank = 0,
  Unknown = 1,
  Vision2010 = 2,
  Refbox2013 = 3,
  Vision2014 = 4,
  VisionTracker2020 = 5,
  Index2021 = 6
};

struct LogEntry
{
  int64_t received_time_ns;
  std::variant<std::monostate, Referee, SSL_WrapperPacket> message;
};

class LogReader {
public:
  explicit LogReader(std::istream & input);

  std::optional<LogEntry> GetNextMessage();

  const std::unordered_map<EntryType, uint64_t> & GetEntryTypeCounts() const
  {
    return entry_type_counts;
  }

private:
  std::istream & input_stream;
  std::unordered_map<EntryType, uint64_t> entry_type_counts;

  void CheckHeader();

  LogEntry ExtractNextEntry();
};

}  // namespace ssl_ros_bridge

#endif  // LOG2BAG__LOG_READER_HPP_
