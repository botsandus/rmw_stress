// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rmw_stress/msg/string_stamped.hpp>

using std::placeholders::_1;

class TransientLocalSubscriber : public rclcpp::Node
{
public:
  TransientLocalSubscriber()
  : Node("transient_local_subscriber")
  {
    subscription_ = this->create_subscription<rmw_stress::msg::StringStamped>(
      "topic", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
          std::bind(&TransientLocalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const rmw_stress::msg::StringStamped & msg)
  {
    auto delay = this->now() - msg.header.stamp;
    // compute and display delay statistics every 10 messages
    delays_.push_back(delay.seconds());
    if (delays_.size() >= 10) {
      double sum = 0.0;
      double min = delays_[0];
      double max = delays_[0];
      for (const auto & d : delays_) {
        sum += d;
        min = std::min(min, d);
        max = std::max(max, d);
      }
      double avg = sum / delays_.size();

      if (max > 0.01) {
      RCLCPP_WARN(this->get_logger(), "Over %zu message of [%zu] bytes : min=%.5f, avg=%.5f, max=%.5f seconds",
        delays_.size(), msg.data.size(), min, avg, max);
      }
      delays_.clear();
    }

  }
  rclcpp::Subscription<rmw_stress::msg::StringStamped>::SharedPtr subscription_;
  std::vector<double> delays_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransientLocalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
