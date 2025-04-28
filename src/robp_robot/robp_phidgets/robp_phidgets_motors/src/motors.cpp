// robp_phidgets
#include <robp_phidgets_motors/motors.hpp>

namespace robp::phidgets
{
Motors::Motors(rclcpp::NodeOptions const& options) : Node("motors", options)
{
	// default open any device
	int serial_num_left  = this->declare_parameter("left_serial", -1);
	int serial_num_right = this->declare_parameter("right_serial", -1);
	// only used if the device is on a VINT hub_port
	int    hub_port_left    = this->declare_parameter("left_hub_port", 4);
	int    hub_port_right   = this->declare_parameter("right_hub_port", 2);
	double acceleration     = this->declare_parameter("acceleration", 100.0);
	double braking_strength = this->declare_parameter("braking_strength", 1.0);
	double current_limit    = this->declare_parameter("current_limit", 2.0);
	double data_rate        = this->declare_parameter("data_rate", 10.0);
	failsafe_time_          = this->declare_parameter("failsafe_timeout", 500);

	if (hub_port_left == hub_port_right) {
		RCLCPP_FATAL(this->get_logger(), "Left and right port cannot be the same");
		exit(1);
	}

	pub_ =
	    this->create_publisher<robp_interfaces::msg::DutyCycles>("/motor/current_duty_cycles", 1);

	left_  = std::make_unique<Motor>(serial_num_left, hub_port_left, false, 0,
                                  std::bind(&Motors::publish, this));
	right_ = std::make_unique<Motor>(serial_num_right, hub_port_right, false, 0,
	                                 std::bind(&Motors::publish, this));

	left_->setAcceleration(acceleration);
	right_->setAcceleration(acceleration);

	left_->setTargetBrakingStrength(braking_strength);
	right_->setTargetBrakingStrength(braking_strength);

	left_->setCurrentLimit(current_limit);
	right_->setCurrentLimit(current_limit);

	left_->setDataRate(data_rate);
	right_->setDataRate(data_rate);

	sub_ = this->create_subscription<robp_interfaces::msg::DutyCycles>(
	    "/motor/duty_cycles", 1,
	    std::bind(&Motors::dutyCyclesCallback, this, std::placeholders::_1));

	// reset_failsafe_srv_ = this->create_service<std_srvs::srv::Empty>(
	//     "/motor/reset_failsafe", std::bind(&Motors::resetFailsafe, this, std::placeholders::_1,
	//                                 std::placeholders::_2));
}

void Motors::dutyCyclesCallback(robp_interfaces::msg::DutyCycles const& msg)
{
	if (!left_ || !right_) {
		return;
	}

	if (!failsafe_enabled_) {
		failsafe_enabled_ = true;
		left_->setFailsafe(failsafe_time_);
		right_->setFailsafe(failsafe_time_);
	}

	if (1 >= std::abs(msg.duty_cycle_left) && 1 >= std::abs(msg.duty_cycle_right)) {
		left_->setTargetVelocity(msg.duty_cycle_left);
		right_->setTargetVelocity(-msg.duty_cycle_right);
	} else {
		RCLCPP_WARN(this->get_logger(),
		            "Duty cycles (%f, %f) is out out of range ([-1, 1], [-1, 1]). Stopping "
		            "motors!",
		            msg.duty_cycle_left, msg.duty_cycle_right);
		left_->setTargetVelocity(0);
		right_->setTargetVelocity(0);
	}

	left_->resetFailsafe();
	right_->resetFailsafe();
}

void Motors::publish()
{
	if (!left_ || !right_ || !left_->hasUpdate() || !right_->hasUpdate()) {
		return;
	}

	auto msg              = std::make_unique<robp_interfaces::msg::DutyCycles>();
	msg->header.stamp     = this->now();
	msg->header.frame_id  = "";
	msg->duty_cycle_left  = left_->velocityUpdate();
	msg->duty_cycle_right = -right_->velocityUpdate();

	pub_->publish(std::move(msg));
}

void Motors::resetFailsafe(
    std::shared_ptr<std_srvs::srv::Empty::Request> const /* request */,
    std::shared_ptr<std_srvs::srv::Empty::Response> /* response */)
{
	if (left_) {
		left_->resetFailsafe();
	}
	if (right_) {
		right_->resetFailsafe();
	}
}
}  // namespace robp::phidgets

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(robp::phidgets::Motors)