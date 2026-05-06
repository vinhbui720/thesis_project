#include "robot_planning/motomini_feedback_stream_node.hpp"

namespace robot_planning
{

  void MotoMiniFeedbackStreamNode::jointStateCallback(
      const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_state_ = msg;
    if (!arm_init_sent_ && static_cast<int>(msg->name.size()) >= kNumberOfJoint)
    {
      if (initTrackedPositions())
      {
        publishArmInit();
        arm_init_sent_ = true;
      }
    }
  }

  void MotoMiniFeedbackStreamNode::desiredPoseCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    desired_pose_ = *msg;
    has_desired_pose_ = true;
    t_last_pose_cb_ = this->now();

    if (state_ == STATE_IDLE && initTrackedPositions())
    {
      pending_state_ = STATE_POSE_FOLLOW;
      transitionTo(STATE_ARMING, "target pose received");
    }
  }

  void MotoMiniFeedbackStreamNode::targetVelCallback(
      const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    latest_target_vel_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
        msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
    t_last_target_vel_cb_ = this->now();

    if (state_ == STATE_IDLE && latest_target_vel_.head<3>().norm() > targetVelocityDeadband())
    {
      Eigen::VectorXd q;
      Eigen::Vector3d ee_pos;
      Eigen::Matrix3d ee_rot;
      if (!currentJointVector(q) || !getEEPose(q, ee_pos, ee_rot))
      {
        return;
      }
      Eigen::Quaterniond qee(ee_rot);
      desired_pose_.header.frame_id = base_link_;
      desired_pose_.pose.position.x = ee_pos.x();
      desired_pose_.pose.position.y = ee_pos.y();
      desired_pose_.pose.position.z = ee_pos.z();
      desired_pose_.pose.orientation.w = qee.w();
      desired_pose_.pose.orientation.x = qee.x();
      desired_pose_.pose.orientation.y = qee.y();
      desired_pose_.pose.orientation.z = qee.z();
      has_desired_pose_ = true;
      t_last_pose_cb_ = this->now();

      if (initTrackedPositions())
      {
        pending_state_ = STATE_POSE_FOLLOW;
        transitionTo(STATE_ARMING, "target velocity command received");
      }
    }
  }

  void MotoMiniFeedbackStreamNode::collisionWrenchCallback(
      const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    latest_collision_wrench_ << -msg->wrench.force.x, -msg->wrench.force.y, -msg->wrench.force.z,
        -msg->wrench.torque.x, -msg->wrench.torque.y, -msg->wrench.torque.z;
    t_last_collision_wrench_cb_ = this->now();
  }

  void MotoMiniFeedbackStreamNode::collisionDistanceCallback(
      const std_msgs::msg::Float64::SharedPtr msg)
  {
    latest_collision_distance_ = msg->data;
    t_last_collision_distance_cb_ = this->now();
  }

  void MotoMiniFeedbackStreamNode::collisionNormalCallback(
      const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    latest_collision_normal_ << msg->vector.x, msg->vector.y, msg->vector.z;
    t_last_collision_normal_cb_ = this->now();
  }

  void MotoMiniFeedbackStreamNode::initPoseCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    init_pose_ = *msg;
    has_init_pose_ = true;
  }

  void MotoMiniFeedbackStreamNode::controlPhaseCallback(
      const std_msgs::msg::String::SharedPtr msg)
  {
    const ControlPhase next_phase = parseControlPhase(msg->data);
    if (next_phase == active_phase_)
    {
      return;
    }
    active_phase_ = next_phase;
    refreshActivePhaseConfig();
    RCLCPP_INFO(this->get_logger(), "Controller phase switched to %s", phaseLabel(active_phase_));
  }

  void MotoMiniFeedbackStreamNode::startCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    transitionTo(STATE_IDLE, "start service called");
    is_active_ = false;
    arm_trigger_sent_ = false;
    resetControlWindow();
    tracked_positions_.clear();
    tracked_velocities_.clear();
    is_init_done_ = false;
    active_phase_ = parseControlPhase(this->get_parameter("control_phase").as_string());
    refreshActivePhaseConfig();
    res->success = true;
    res->message = "Reset to STATE_IDLE and re-armed.";
  }

  void MotoMiniFeedbackStreamNode::stopCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    transitionTo(STATE_STOP, "stop service called");
    resetControlWindow();
    res->success = true;
    res->message = "Stopped, holding position.";
  }

  void MotoMiniFeedbackStreamNode::initStartCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!has_init_pose_)
    {
      res->success = false;
      res->message = "No init pose received. Publish to /pose_following/init_pose first.";
      return;
    }
    if (state_ != STATE_IDLE)
    {
      res->success = false;
      res->message = "Must be in STATE_IDLE to start init move.";
      return;
    }
    if (!initTrackedPositions())
    {
      res->success = false;
      res->message = "No joint state available yet.";
      return;
    }
    is_init_done_ = false;
    pending_state_ = STATE_INIT;
    transitionTo(STATE_ARMING, "init_start service called");
    res->success = true;
    res->message = "STATE_IDLE -> STATE_ARMING -> STATE_INIT";
  }

} // namespace robot_planning
