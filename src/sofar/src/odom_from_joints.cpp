#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;

class OdomFromJointStates : public rclcpp::Node
{
public:
  OdomFromJointStates()
      : Node("odom_from_joint_states"),
        x_(0.0), y_(0.0), theta_(0.0),
        last_left_pos_(0.0), last_right_pos_(0.0),
        wheel_radius_(0.033), wheel_separation_(0.287),
        first_reading_(true)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&OdomFromJointStates::joint_callback, this, _1));

    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom2", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "odom_from_joint_states node started.");
  }

private:
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    double left_pos = 0.0, right_pos = 0.0;

    // Get positions of left and right wheel
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (msg->name[i] == "wheel_left_joint")
        left_pos = msg->position[i];
      else if (msg->name[i] == "wheel_right_joint")
        right_pos = msg->position[i];
    }

    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();

    if (first_reading_)
    {
      last_left_pos_ = left_pos;
      last_right_pos_ = right_pos;
      last_time_ = current_time;
      first_reading_ = false;
      return;
    }

    //calculation
    double delta_left = left_pos - last_left_pos_;
    double delta_right = right_pos - last_right_pos_;

    double v_left = wheel_radius_ * delta_left / dt;
    double v_right = wheel_radius_ * delta_right / dt;

    double v = (v_left + v_right) / 2.0;
    double w = (v_right - v_left) / wheel_separation_;

    // Integrate to get new pose
    x_ += v * cos(theta_) * dt;
    y_ += v * sin(theta_) * dt;
    theta_ += w * dt;

    // Create quaternion from yaw
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();

    // Publish odometry
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom2";
    odom_msg.child_frame_id = "base_footprint";

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = q_msg;

    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.angular.z = w;

    odom_publisher_->publish(odom_msg);

    // Publish TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = "odom2";
    tf_msg.child_frame_id = "base_footprint";
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = q_msg;

    tf_broadcaster_->sendTransform(tf_msg);

    last_left_pos_ = left_pos;
    last_right_pos_ = right_pos;
    last_time_ = current_time;
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double x_, y_, theta_;
  double last_left_pos_, last_right_pos_;
  rclcpp::Time last_time_;
  const double wheel_radius_;
  const double wheel_separation_;
  bool first_reading_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomFromJointStates>());
  rclcpp::shutdown();
  return 0;
}
