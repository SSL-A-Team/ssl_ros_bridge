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

#include <filesystem>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <ssl_league_msgs/msg/vision_wrapper.hpp>
#include <ssl_league_msgs/msg/referee.hpp>
#include <rosbag2_cpp/writer.hpp>
#include "core/message_conversion.hpp"
#include "log_reader.hpp"

template<typename ProtoType, typename RosType>
void WriteMessageIfExists(
  const ssl_ros_bridge::LogEntry & entry, const std::string & topic,
  const std::string & type_name, rosbag2_cpp::Writer & writer,
  typename rclcpp::Serialization<RosType> & serialization)
{
  if(!std::holds_alternative<ProtoType>(entry.message)) {
    return;
  }
  const auto msg =
    ssl_ros_bridge::message_conversion::fromProto(std::get<ProtoType>(entry.message));
  auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>();
  serialization.serialize_message(&msg, serialized_msg.get());
  rclcpp::Time time(entry.received_time_ns);
  writer.write(serialized_msg, topic, type_name, time);
}

void PrintUsage()
{
  std::cout <<
    R"(
Usage: log2bag FILE
Convert SSL log file to ROS bag.

FILE - The SSL log file to convert
)";
}

void RenderProgressBar(uintmax_t bytes_read, uintmax_t bytes_total)
{
  constexpr auto bar_width = 20;
  const auto percent = (bytes_read * 100) / bytes_total;
  const auto num_filled_chars = (percent * bar_width) / 100;
  std::cout << "\r[";
  std::fill_n(std::ostream_iterator<char>(std::cout), num_filled_chars, '=');
  std::fill_n(std::ostream_iterator<char>(std::cout), bar_width - num_filled_chars, ' ');
  std::cout << "] " << percent << '%';
  std::cout.flush();
}

int main(int argc, char ** argv)
{
  if(argc != 2) {
    PrintUsage();
    return 1;
  }

  std::filesystem::path log_path = argv[1];

  std::ifstream stream(log_path, std::ios::binary);
  if(!stream.is_open()) {
    std::cerr << "Could not open log file.\n";
    return 1;
  }

  std::error_code errcode;
  const auto log_file_size = std::filesystem::file_size(log_path, errcode);
  if(errcode) {
    std::cerr << "Could not determine log file size: " << errcode.message() << '\n';
    return 1;
  }

  ssl_ros_bridge::LogReader reader(stream);

  auto writer = std::make_unique<rosbag2_cpp::Writer>();
  writer->open(log_path.stem().string());

  rclcpp::Serialization<ssl_league_msgs::msg::Referee> referee_serialization;
  rclcpp::Serialization<ssl_league_msgs::msg::VisionWrapper> vision_serialization;

  while(const auto entry = reader.GetNextMessage()) {
    RenderProgressBar(stream.tellg(), log_file_size);
    WriteMessageIfExists<Referee, ssl_league_msgs::msg::Referee>(*entry, "/referee_messages",
      "ssl_league_msgs/msg/Referee", *writer, referee_serialization);
    WriteMessageIfExists<SSL_WrapperPacket, ssl_league_msgs::msg::VisionWrapper>(*entry,
      "/vision_messages", "ssl_league_msgs/msg/VisionWrapper", *writer, vision_serialization);
  }
  std::cout << "\n\n";

  const auto type_stats = reader.GetEntryTypeCounts();

  std::cout << "Translated entries:\n";
  std::cout << "\tRefbox 2013: " << type_stats.at(ssl_ros_bridge::EntryType::Refbox2013) << '\n';
  std::cout << "\tVision 2014: " << type_stats.at(ssl_ros_bridge::EntryType::Vision2014) << '\n';
  std::cout << '\n';
  std::cout << "Skipped entries:\n";
  std::cout << "\tBlank: " << type_stats.at(ssl_ros_bridge::EntryType::Blank) << '\n';
  std::cout << "\tUnknown: " << type_stats.at(ssl_ros_bridge::EntryType::Unknown) << '\n';
  std::cout << "\tVision 2010: " << type_stats.at(ssl_ros_bridge::EntryType::Vision2010) << '\n';
  std::cout << "\tVision Tracker 2020: " <<
    type_stats.at(ssl_ros_bridge::EntryType::VisionTracker2020) << '\n';
  std::cout << "\tIndex 2021: " << type_stats.at(ssl_ros_bridge::EntryType::Index2021) << '\n';

  return 0;
}
