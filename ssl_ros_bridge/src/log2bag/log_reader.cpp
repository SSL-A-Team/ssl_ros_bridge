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

#include "log_reader.hpp"
#include <vector>

namespace ssl_ros_bridge
{

LogReader::LogReader(std::istream & input)
: input_stream(input)
{
  CheckHeader();
  entry_type_counts = {
    {EntryType::Blank, 0},
    {EntryType::Unknown, 0},
    {EntryType::Vision2010, 0},
    {EntryType::Refbox2013, 0},
    {EntryType::Vision2014, 0},
    {EntryType::VisionTracker2020, 0},
    {EntryType::Index2021, 0}
  };
}

std::optional<LogEntry> LogReader::GetNextMessage()
{
  while(!input_stream.eof()) {
    try {
      auto entry = ExtractNextEntry();
      if(!std::holds_alternative<std::monostate>(entry.message)) {
        return entry;
      }
    } catch (const std::exception & e) {
      std::cerr << "Exception thrown while parsing log: " << e.what() << std::endl;
      return std::nullopt;
    }
  }
  return std::nullopt;
}

void LogReader::CheckHeader()
{
  constexpr auto header_size = 16ul;
  std::array<char, header_size> header_buffer;
  const auto bytes_read = input_stream.read(header_buffer.data(), header_size).gcount();
  if(bytes_read != header_size) {
    std::cout << "Read " << bytes_read << " bytes.\n";
    throw std::runtime_error("Not enough bytes for a valid header");
  }
  const std::array<char, header_size> expected_header = {'S', 'S', 'L', '_', 'L', 'O', 'G', '_',
    'F', 'I', 'L', 'E', 0, 0, 0, 1};
  if(!std::ranges::equal(header_buffer, expected_header)) {
    throw std::runtime_error("Unsupported file format based on header bytes.");
  }
}

LogEntry LogReader::ExtractNextEntry()
{
  constexpr auto header_size = 16ul;
  std::array<char, header_size> header_buffer;
  const auto header_bytes_read = input_stream.read(header_buffer.data(), header_size).gcount();
  if(header_bytes_read == 0) {
    return {};
  }
  if(header_bytes_read != header_size) {
    std::cerr << "Not enough bytes left in stream for a valid entry.\n";
    std::cerr << header_bytes_read << '\n';
    return {};
  }
  int32_t entry_type_raw = -1;
  std::copy_n(header_buffer.rend() - 12, sizeof(entry_type_raw),
      reinterpret_cast<int8_t *>(&entry_type_raw));
  if(entry_type_raw < 0 || entry_type_raw > 6) {
    std::cerr << "Unrecognized entry type: " << entry_type_raw << '\n';
    return {};
  }
  const auto entry_type = static_cast<EntryType>(entry_type_raw);
  entry_type_counts[entry_type]++;

  LogEntry entry;

  std::copy_n(header_buffer.rend() - 8, sizeof(entry.received_time_ns),
      reinterpret_cast<int8_t *>(&entry.received_time_ns));

  int32_t data_size = 0;
  std::copy_n(header_buffer.rend() - 16, sizeof(data_size), reinterpret_cast<int8_t *>(&data_size));

  std::vector<char> data;
  data.reserve(data_size);
  std::fill_n(std::back_inserter(data), data_size, 0);

  const auto data_bytes_read = input_stream.read(data.data(), data_size).gcount();
  if(data_bytes_read != data_size) {
    std::cerr << "Not enough bytes for expected data packet.\n";
    std::cerr << data_bytes_read << '\n';
    return {};
  }

  switch(entry_type) {
    case EntryType::Refbox2013:
      {
        Referee ref_message;
        if(!ref_message.ParseFromArray(data.data(), data.size())) {
          std::cerr << "Failed to parse protobuf message\n";
          break;
        }
        entry.message = ref_message;
        break;
      }
    case EntryType::Vision2014:
      {
        SSL_WrapperPacket vision_message;
        if(!vision_message.ParseFromArray(data.data(), data.size())) {
          std::cerr << "Failed to parse protobuf message\n";
          break;
        }
        entry.message = vision_message;
        break;
      }
    default:
      // Do nothing with other message types.
      break;
  }

  return entry;
}

}  // namespace ssl_ros_bridge
