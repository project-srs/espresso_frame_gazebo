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
class FeetechTurretRosIF
{
public:
  FeetechTurretRosIF()
  {
  }

  void Initialize(const gazebo_ros::Node::SharedPtr ros_node)
  {
    ros_node_ = ros_node;
    odometry_pub_ =
        ros_node_->create_publisher<nav_msgs::msg::Odometry>("/device/head_turret/odom", rclcpp::QoS(1));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(ros_node_);

    cmd_rate_sub_ = ros_node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/device/head_turret/cmd_rate", rclcpp::QoS(1),
        std::bind(&FeetechTurretRosIF::OnCmdRate, this, std::placeholders::_1));
  }

  void publishOdom(const nav_msgs::msg::Odometry& odom)
  {
    odometry_pub_->publish(odom);
  }

  void broadcastTf(const nav_msgs::msg::Odometry& odom)
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

  rclcpp::Logger getLogger(void)
  {
    return ros_node_->get_logger();
  }

protected:
  virtual void OnCmdRate(const geometry_msgs::msg::TwistStamped::SharedPtr _msg) = 0;

private:
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_rate_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

class TurretDriver : public gazebo::ModelPlugin
{
public:
  TurretDriver()
  {
  }

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    model_ = _model;
    body_link_ = model_->GetLink("espresso_frame_head::body_link");
    pitch_joint_ = _model->GetJoint("espresso_frame_head::pitch_joint");
    yaw_joint_ = _model->GetJoint("espresso_frame_head::yaw_joint");

    onLoad(gazebo_ros::Node::Get(_sdf));

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&TurretDriver::OnUpdate, this, std::placeholders::_1));
  }

  // Documentation inherited
  void Reset() override
  {
  }

protected:
  virtual void onLoad(const gazebo_ros::Node::SharedPtr ros_node) = 0;
  virtual void onConrtolUpdate(void) = 0;
  virtual void onOdomUpdate(const nav_msgs::msg::Odometry& msg) = 0;

  void setJointVelocity(const float pitch, const float yaw)
  {
    pitch_joint_->SetVelocity(0, pitch);
    yaw_joint_->SetVelocity(0, yaw);
  }

private:
  void OnUpdate(const gazebo::common::UpdateInfo& _info)
  {
    onConrtolUpdate();

    double seconds_since_last_update = (_info.simTime - last_update_time_).Double();
    if (0.05f <= seconds_since_last_update)
    {
      last_update_time_ = _info.simTime;

      // static float counter = 0;
      // counter++;
      // float diff_x = 0.1f * std::min(std::max((float)counter * 0.05f, 20.0f), 30.0f);

      nav_msgs::msg::Odometry odom;
      odom.header.frame_id = "odom";
      odom.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_info.simTime);
      odom.child_frame_id = "base_link";
      ignition::math::Pose3d pose = body_link_->WorldPose();
      odom.pose.pose.position.x = pose.Pos().X();
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
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr body_link_;
  gazebo::common::Time last_update_time_;
  gazebo::physics::JointPtr pitch_joint_;
  gazebo::physics::JointPtr yaw_joint_;
  gazebo::event::ConnectionPtr update_connection_;
};

class FeetechTurret : public TurretDriver, FeetechTurretRosIF
{
public:
  FeetechTurret() : TurretDriver{}, FeetechTurretRosIF{}
  {
  }

  ~FeetechTurret()
  {
  }

protected:
  void onLoad(const gazebo_ros::Node::SharedPtr ros_node) override
  {
    Initialize(ros_node);
    RCLCPP_WARN(getLogger(), "start FeetechTurret\n");
  }

  void onConrtolUpdate(void) override
  {
    setJointVelocity(last_twist_data_.angular.y, last_twist_data_.angular.z);
  }

  void onOdomUpdate(const nav_msgs::msg::Odometry& msg) override
  {
    // publishOdom(msg);
    // broadcastTf(msg);
  }

  void OnCmdRate(const geometry_msgs::msg::TwistStamped::SharedPtr _msg) override
  {
    // RCLCPP_WARN(getLogger(), "OnCmdRate %f %f", _msg->twist.angular.y, _msg->twist.angular.z);
    last_twist_data_ = _msg->twist;
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    last_twist_time_ = ros_clock.now();
  }

private:
  geometry_msgs::msg::Twist last_twist_data_{};
  rclcpp::Time last_twist_time_;
};

GZ_REGISTER_MODEL_PLUGIN(FeetechTurret)
}  // namespace espresso_frame_gazebo_plugins