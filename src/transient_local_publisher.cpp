#include <rclcpp/rclcpp.hpp>
#include <rmw_stress/msg/string_stamped.hpp>
#include <random>

class TransientLocalPublisher : public rclcpp::Node
{
public:
  TransientLocalPublisher()
  : Node("transient_local_publisher")
  {
    // Declare parameter for message size in bytes instead of megabytes
    this->declare_parameter<int>("message_size_bytes", 1024);  // default 1024 bytes
    message_size_bytes_ = this->get_parameter("message_size_bytes").as_int();

    rclcpp::QoS qos_profile(10);
    qos_profile.transient_local();

    publisher_ = this->create_publisher<rmw_stress::msg::StringStamped>("topic", qos_profile);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TransientLocalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = rmw_stress::msg::StringStamped();
    // Use message_size_bytes_ directly for string length
    message.data = generate_random_string(static_cast<size_t>(message_size_bytes_));
    message.header.stamp = this->now();
    message.header.frame_id = this->get_name();
    // RCLCPP_INFO(this->get_logger(), "Publishing message of size: %zu bytes", message.data.size());
    publisher_->publish(message);
  }

  std::string generate_random_string(size_t length)
  {
    static const std::string characters =
      "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
    static thread_local std::mt19937 generator(std::random_device{}());
    static thread_local std::uniform_int_distribution<size_t> distribution(0, characters.size() - 1);

    std::string random_str;
    random_str.reserve(length);
    for (size_t i = 0; i < length; ++i) {
      random_str += characters[distribution(generator)];
    }
    return random_str;
  }

  rclcpp::Publisher<rmw_stress::msg::StringStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int message_size_bytes_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransientLocalPublisher>());
  rclcpp::shutdown();
  return 0;
}
