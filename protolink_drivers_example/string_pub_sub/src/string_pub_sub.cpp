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

#include <proto_files/conversion_std_msgs__String.hpp>
#include <protolink/client.hpp>
#include <rclcpp/rclcpp.hpp>

namespace protolink_drivers_example
{

// Network settings
const std::string ip_address = "192.168.0.100";  // of microcontroller, etc.
const uint16_t to_port = 8000;                   // same as above.
const uint16_t from_port = 8000;                 // of the computer executing this code.
const uint16_t sub_port = 9000;                  // same as above

class StringPubSub : public rclcpp::Node
{
public:
  explicit StringPubSub()
  : Node("string_pub_sub"),
    protolink_publisher_(io_context_, ip_address, to_port, from_port, this->get_logger()),
    publish_timer_(create_wall_timer(
      std::chrono::duration<double>(1.0 / 10),
      [&]() {
        std::lock_guard<std::mutex> lock(mutex_);

        auto msg = std::make_unique<std_msgs::msg::String>();
        msg->data = "hello";
        protolink_publisher_.send(convert(*std::move(msg)));
        RCLCPP_INFO(this->get_logger(), "Publish    --->  data: %s", msg->data.c_str());
      })),
    protolink_subscriber_(io_context_, sub_port, [this](const auto & _msg) {
      std::lock_guard<std::mutex> lock(mutex_);

      std_msgs::msg::String msg = convert(_msg);
      RCLCPP_INFO(this->get_logger(), "Subscribe  --->  data: %s\n", msg.data.c_str());
    })
  {
  }

private:
  boost::asio::io_context io_context_;
  std::mutex mutex_;

  using protoString = protolink__std_msgs__String::std_msgs__String;

  protolink::udp_protocol::Publisher<protoString> protolink_publisher_;
  const rclcpp::TimerBase::SharedPtr publish_timer_;
  protolink::udp_protocol::Subscriber<protoString> protolink_subscriber_;
};
}  // namespace protolink_drivers_example

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<protolink_drivers_example::StringPubSub>();
    rclcpp::spin(node);
  } catch (std::exception & e) {
    std::cout << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
