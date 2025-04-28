#ifndef ROBP_PHIDGETS_MOTORS_MOTORS_HPP
#define ROBP_PHIDGETS_MOTORS_MOTORS_HPP

// robp_phidgets_motors
#include <robp_phidgets_motors/motor.hpp>

// robp_interfaces
#include <robp_interfaces/msg/duty_cycles.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

// STL
#include <memory>

namespace robp::phidgets
{
class Motors : public rclcpp::Node
{
 public:
	explicit Motors(rclcpp::NodeOptions const& options);

 private:
	void dutyCyclesCallback(robp_interfaces::msg::DutyCycles const& msg);

	void publish();

	void resetFailsafe(std::shared_ptr<std_srvs::srv::Empty::Request> const request,
	                   std::shared_ptr<std_srvs::srv::Empty::Response>      response);

 private:
	std::unique_ptr<Motor> left_;
	std::unique_ptr<Motor> right_;

	rclcpp::Publisher<robp_interfaces::msg::DutyCycles>::SharedPtr pub_;

	rclcpp::Subscription<robp_interfaces::msg::DutyCycles>::SharedPtr sub_;

	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_failsafe_srv_;

	uint32_t failsafe_time_{};
	bool     failsafe_enabled_{false};
};
}  // namespace robp::phidgets

#endif  // ROBP_PHIDGETS_MOTORS_MOTORS_HPP