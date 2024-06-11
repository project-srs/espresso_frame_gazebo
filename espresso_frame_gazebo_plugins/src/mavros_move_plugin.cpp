#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_plugins/gazebo_ros_diff_drive.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sdf/sdf.hh>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <memory>
#include <string>
#include <vector>

namespace espresso_frame_gazebo_plugins
{
class MavrosMoveRosIF
{
public:
  MavrosMoveRosIF()
  {
  }

  void Initialize(const gazebo_ros::Node::SharedPtr ros_node)
  {
    ros_node_ = ros_node;
    odometry_pub_ =
      ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "/device/mavros/velocity_position/odom", rclcpp::QoS(
        1));
    armed_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>(
      "/device/mavros/setup/armed", rclcpp::QoS(
        1));
    mode_pub_ = ros_node_->create_publisher<std_msgs::msg::String>(
      "/device/mavros/setup/mode", rclcpp::QoS(
        1));
    battery_pub_ = ros_node_->create_publisher<sensor_msgs::msg::BatteryState>(
      "/device/mavros/sys_status/battery", rclcpp::QoS(
        1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(ros_node_);

    cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/device/mavros/setpoint_velocity/cmd_vel", rclcpp::QoS(1),
      std::bind(&MavrosMoveRosIF::OnCmdVel, this, std::placeholders::_1));
    request_arming_srv_ = ros_node_->create_service<std_srvs::srv::SetBool>(
      "/device/mavros/setup/request_arming",
      std::bind(
        &MavrosMoveRosIF::onRequestArming, this, std::placeholders::_1,
        std::placeholders::_2));
    request_mode_srv_ = ros_node_->create_service<std_srvs::srv::SetBool>(
      "/device/mavros/setup/request_guided",
      std::bind(
        &MavrosMoveRosIF::onRequestMode, this, std::placeholders::_1,
        std::placeholders::_2));

    ros_node_->declare_parameter("battery.voltage", 19.0f);
  }

  void publishOdom(const nav_msgs::msg::Odometry & odom)
  {
    odometry_pub_->publish(odom);
  }

  void broadcastTf(const nav_msgs::msg::Odometry & odom)
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header = odom.header;
    transformStamped.child_frame_id = odom.child_frame_id;
    transformStamped.transform.translation.x = odom.pose.pose.position.x;
    transformStamped.transform.translation.y = odom.pose.pose.position.y;
    transformStamped.transform.translation.z = odom.pose.pose.position.z;
    transformStamped.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transformStamped);
  }

  void publishArmed(const bool is_armed)
  {
    std_msgs::msg::Bool msg;
    msg.data = is_armed;
    armed_pub_->publish(msg);
  }

  void publishMode(const std::string mode)
  {
    std_msgs::msg::String msg;
    msg.data = mode;
    mode_pub_->publish(msg);
  }

  void publishBatteryState(const rclcpp::Time ros_now)
  {
    const float voltage = ros_node_->get_parameter("battery.voltage").as_double();
    sensor_msgs::msg::BatteryState batt;
    batt.header.stamp = ros_now;
    batt.voltage = voltage;
    batt.current = -0.9f;
    batt.percentage = 0.97f;
    batt.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    batt.present = true;
    batt.cell_voltage.push_back(voltage);
    battery_pub_->publish(batt);
  }

  rclcpp::Logger getLogger(void)
  {
    return ros_node_->get_logger();
  }

protected:
  virtual void OnCmdVel(const geometry_msgs::msg::TwistStamped::SharedPtr _msg) = 0;
  virtual void onRequestArming(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) = 0;
  virtual void onRequestMode(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) = 0;

private:
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr request_arming_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr request_mode_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

class DiffMoveDriver : public gazebo::ModelPlugin
{
public:
  DiffMoveDriver()
  {
  }

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    model_ = _model;
    body_link_ = model_->GetLink("espresso_frame_move::body_link");
    left_joint_ = _model->GetJoint("espresso_frame_move::left_wheel_joint");
    right_joint_ = _model->GetJoint("espresso_frame_move::right_wheel_joint");

    onLoad(gazebo_ros::Node::Get(_sdf));

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&DiffMoveDriver::OnUpdate, this, std::placeholders::_1));
  }

  // Documentation inherited
  void Reset() override
  {
  }

protected:
  virtual void onLoad(const gazebo_ros::Node::SharedPtr ros_node) = 0;
  virtual void onConrtolUpdate(void) = 0;
  virtual void onOdomUpdate(const nav_msgs::msg::Odometry & msg) = 0;
  virtual void onBatteryStatusUpdate(const rclcpp::Time ros_now) = 0;

  void setJointVelocity(const float left, const float right)
  {
    left_joint_->SetVelocity(0, left);
    right_joint_->SetVelocity(0, right);
  }

private:
  void OnUpdate(const gazebo::common::UpdateInfo & _info)
  {
    onConrtolUpdate();

    double seconds_since_last_update = (_info.simTime - last_update_time_).Double();
    if (0.05f <= seconds_since_last_update) {
      last_update_time_ = _info.simTime;

      if (!initial_pose_.has_value()) {
        initial_pose_ = body_link_->WorldPose();
      }

      nav_msgs::msg::Odometry odom;
      odom.header.frame_id = "odom";
      odom.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_info.simTime);
      odom.child_frame_id = "base_link";
      ignition::math::Pose3d pose = body_link_->WorldPose() - initial_pose_.value();
      odom.pose.pose.position.x = pose.Pos().X() + 0.2f *
        std::min(std::max(_info.simTime.Float() - 10.0f, 0.0f), 10.0f);
      odom.pose.pose.position.y = pose.Pos().Y();
      odom.pose.pose.position.z = pose.Pos().Z();
      odom.pose.pose.orientation.w = pose.Rot().W();
      odom.pose.pose.orientation.x = pose.Rot().X();
      odom.pose.pose.orientation.y = pose.Rot().Y();
      odom.pose.pose.orientation.z = pose.Rot().Z();
      ignition::math::Vector3d linear_vel = body_link_->RelativeLinearVel();
      odom.twist.twist.linear.x = linear_vel.X();
      odom.twist.twist.linear.y = linear_vel.Y();
      odom.twist.twist.linear.z = linear_vel.Z();
      ignition::math::Vector3d angular_vel = body_link_->RelativeAngularVel();
      odom.twist.twist.angular.x = angular_vel.X();
      odom.twist.twist.angular.y = angular_vel.Y();
      odom.twist.twist.angular.z = angular_vel.Z();
      onOdomUpdate(odom);

      onBatteryStatusUpdate(rclcpp::Time(gazebo_ros::Convert<builtin_interfaces::msg::Time>(_info.simTime)));
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr body_link_;
  gazebo::common::Time last_update_time_;
  gazebo::physics::JointPtr left_joint_;
  gazebo::physics::JointPtr right_joint_;
  gazebo::event::ConnectionPtr update_connection_;
  std::optional<ignition::math::Pose3d> initial_pose_;
};

class MavrosMove : public DiffMoveDriver, MavrosMoveRosIF
{
public:
  MavrosMove()
  : DiffMoveDriver{}, MavrosMoveRosIF{}
  {
  }

  ~MavrosMove()
  {
  }

protected:
  void onLoad(const gazebo_ros::Node::SharedPtr ros_node) override
  {
    Initialize(ros_node);
    RCLCPP_WARN(getLogger(), "start MavrosMove\n");
  }

  void onConrtolUpdate(void) override
  {
    float vel_x = 0;
    float rot_z = 0;
    if (is_guided_ && is_armed_) {
      vel_x = last_twist_data_.linear.x;
      rot_z = last_twist_data_.angular.z;
    }
    float left_rate = vel_x / wheel_radius_ - rot_z * wheel_distance_ / wheel_radius_;
    float right_rate = vel_x / wheel_radius_ + rot_z * wheel_distance_ / wheel_radius_;
    setJointVelocity(left_rate, right_rate);
  }

  void onOdomUpdate(const nav_msgs::msg::Odometry & msg) override
  {
    publishOdom(msg);
    broadcastTf(msg);

    publishArmed(is_armed_);
    publishMode(is_guided_ ? "GUIDED" : "HOLD");
  }

  void OnCmdVel(const geometry_msgs::msg::TwistStamped::SharedPtr _msg) override
  {
    // RCLCPP_WARN(getLogger(), "OnCmdVel %f %f", _msg->twist.linear.x, _msg->twist.angular.z);
    last_twist_data_ = _msg->twist;
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    last_twist_time_ = ros_clock.now();
  }

  void onRequestArming(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) override
  {
    RCLCPP_INFO(getLogger(), "onRequestArming");
    is_armed_ = request->data;
    response->success = true;
  }
  void onRequestMode(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) override
  {
    RCLCPP_INFO(getLogger(), "onRequestMode");
    is_guided_ = request->data;
    response->success = true;
  }

  void onBatteryStatusUpdate(const rclcpp::Time ros_now)
  {
    publishBatteryState(ros_now);
  }

private:
  geometry_msgs::msg::Twist last_twist_data_;
  rclcpp::Time last_twist_time_;
  bool is_armed_{true};
  bool is_guided_{true};

  const float wheel_radius_ = 0.072;
  const float wheel_distance_ = 0.18;
};

GZ_REGISTER_MODEL_PLUGIN(MavrosMove)
}  // namespace espresso_frame_gazebo_plugins
