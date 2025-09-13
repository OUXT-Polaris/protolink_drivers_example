// Copyright (c) 2025 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <proto_files/conversion_geometry_msgs__Vector3.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>

namespace protolink_drivers_example
{

// Network settings
const std::string ip_address = "192.168.0.100";  // of microcontroller, etc.
const uint16_t to_port = 8000;                   // same as above.
const uint16_t from_port = 8000;                 // of the computer executing this code.
const uint16_t sub_port = 9000;                  // same as above

class SimplePubSub : public rclcpp::Node
{
public:
  explicit SimplePubSub()
  : Node("simple_pub_sub"),
    protolink_publisher_(io_context_, ip_address, to_port, from_port, this->get_logger()),
    publish_timer_(create_wall_timer(
      std::chrono::duration<double>(1.0 / 10),
      [&]() {
        std::lock_guard<std::mutex> lock(mutex_);

        auto msg = std::make_unique<geometry_msgs::msg::Vector3>();
        msg->x = count;
        msg->y = count * 1.5;
        msg->z = count * 2.0;
        protolink_publisher_.send(convert(*std::move(msg)));
        RCLCPP_INFO(
          this->get_logger(), "Publish    --->  x: %f  y: %f  z: %f", msg->x, msg->y, msg->z);
      })),
    protolink_subscriber_(io_context_, sub_port, [this](const auto & _msg) {
      std::lock_guard<std::mutex> lock(mutex_);

      geometry_msgs::msg::Vector3 msg = convert(_msg);
      count = msg.x;
      RCLCPP_INFO(
        this->get_logger(), "Subscribe  --->  x: %f  y: %f  z: %f\n", msg.x, msg.y, msg.z);
    })
  {
  }

private:
  boost::asio::io_context io_context_;
  std::mutex mutex_;

  using protoVector3 = protolink__geometry_msgs__Vector3::geometry_msgs__Vector3;

  protolink::udp_protocol::Publisher<protoVector3> protolink_publisher_;
  const rclcpp::TimerBase::SharedPtr publish_timer_;
  protolink::udp_protocol::Subscriber<protoVector3> protolink_subscriber_;

  int count = 0.0;
};
}  // namespace protolink_drivers_example

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<protolink_drivers_example::SimplePubSub>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
