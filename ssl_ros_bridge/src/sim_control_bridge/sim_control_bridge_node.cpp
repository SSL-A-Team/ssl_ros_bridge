// Copyright 2026 A Team
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

#include <boost/asio.hpp>
#include <boost/system/result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ssl_league_msgs/srv/send_simulator_control_packet.hpp>
#include <ssl_league_protobufs/ssl_simulation_control.pb.h>
#include "core/protobuf_logging.hpp"
#include "core/message_conversion.hpp"

namespace ssl_ros_bridge::sim_control_bridge
{

using udp = boost::asio::ip::udp;

class SimControlBridge : public rclcpp::Node
{
public:
  explicit SimControlBridge(const rclcpp::NodeOptions & options)
  : rclcpp::Node("sim_control_bridge", options),
    recv_timer_(io_context_),
    socket_(io_context_)
  {
    SET_ROS_PROTOBUF_LOG_HANDLER("sim_control_bridge.protobuf");

    declare_parameter("sim_ip_address", "127.0.0.1");
    declare_parameter("sim_control_port", 10300);

    recv_timer_.expires_after(std::chrono::milliseconds(declare_parameter("response_timeout_ms",
        10)));

    socket_.open(udp::v4());

    service_ =
      create_service<ssl_league_msgs::srv::SendSimulatorControlPacket>(
        "~/send_simulator_control_packet",
        std::bind(&SimControlBridge::handle_send_simulator_control, this,
        std::placeholders::_1, std::placeholders::_2));
  }

private:
  boost::asio::io_context io_context_;
  boost::asio::steady_timer recv_timer_;
  udp::socket socket_;
  std::thread io_context_thread_;
  rclcpp::Service<ssl_league_msgs::srv::SendSimulatorControlPacket>::SharedPtr service_;
  std::vector<uint8_t> send_buffer_;
  std::array<uint8_t, 2048> recv_buffer_;

  void handle_send_simulator_control(
    ssl_league_msgs::srv::SendSimulatorControlPacket::Request::SharedPtr request,
    ssl_league_msgs::srv::SendSimulatorControlPacket::Response::SharedPtr response)
  {
    SimulatorCommand command;
    SimulatorControl & control = *command.mutable_control();

    try {
      control = message_conversion::fromMsg(request->simulator_control);
    } catch (const std::invalid_argument & e) {
      response->reason = e.what();
      response->success = false;
      return;
    }

    send_buffer_.resize(command.ByteSizeLong());
    if(!command.SerializeToArray(send_buffer_.data(), send_buffer_.size())) {
      response->success = false;
      response->reason = "Failed to serialize protobuf packet.";
      return;
    }

    const auto dest_addr = get_parameter("sim_ip_address").as_string();
    const auto dest_port = get_parameter("sim_control_port").as_int();
    const auto dest = udp::endpoint(boost::asio::ip::make_address(dest_addr), dest_port);

    socket_.send_to(boost::asio::buffer(send_buffer_), dest);

    const auto result = wait_for_response();

    if (result.has_error()) {
      response->success = false;
      response->reason = result.error();
      return;
    }

    const auto sim_response = *result;

    if(!sim_response.errors().empty()) {
      response->success = false;
      std::string reason = "Simulator replied with errors:";
      for(const auto & error : sim_response.errors()) {
        reason += " " + error.code() + "-" + error.message() + ";";
      }
      response->reason = reason;
      return;
    }

    response->success = true;
  }

  boost::system::result<SimulatorResponse, std::string> wait_for_response()
  {
    io_context_.restart();

    recv_timer_.async_wait([&](const boost::system::error_code & ec) {
        if (!ec) {
          socket_.cancel();
        }
    });

    boost::system::result<SimulatorResponse, std::string> result;

    socket_.async_receive(
        boost::asio::buffer(recv_buffer_),
      [&](const boost::system::error_code & ec, std::size_t bytes_recvd) {
        if (!ec) {
          recv_timer_.cancel();
          SimulatorResponse response;
          if(!response.ParseFromArray(recv_buffer_.data(), bytes_recvd)) {
            result = "Failed to parse protobuf from incoming datagram.";
            return;
          }
          result = response;
        } else if (ec == boost::asio::error::operation_aborted) {
          result = "Timed out waiting for simulator response.";
        } else {
          recv_timer_.cancel();
          result = "Error while waiting for simulator response: " + ec.message();
        }
        });

    io_context_.run();

    return result;
  }
};

}  // namespace ssl_ros_bridge::sim_control_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(ssl_ros_bridge::sim_control_bridge::SimControlBridge)
