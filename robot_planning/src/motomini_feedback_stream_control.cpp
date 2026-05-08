#include "robot_planning/motomini_feedback_stream_node.hpp"

namespace robot_planning
{

  Eigen::MatrixXd MotoMiniFeedbackStreamNode::calcPseudoInverse(const Eigen::MatrixXd &J)
  {
    return J.transpose() * (J * J.transpose()).inverse();
  }

  const char *MotoMiniFeedbackStreamNode::stateLabel(State state) const
  {
    switch (state)
    {
    case STATE_IDLE:
      return "STATE_IDLE";
    case STATE_POSE_FOLLOW:
      return "STATE_POSE_FOLLOW";
    case STATE_STOP:
      return "STATE_STOP";
    case STATE_INIT:
      return "STATE_INIT";
    case STATE_ARMING:
      return "STATE_ARMING";
    default:
      return "STATE_UNKNOWN";
    }
  }

  void MotoMiniFeedbackStreamNode::transitionTo(State next_state, const char *reason)
  {
    if (state_ == next_state)
    {
      return;
    }
    if (reason != nullptr && reason[0] != '\0')
    {
      RCLCPP_INFO(
          this->get_logger(), "%s --> %s (%s)",
          stateLabel(state_), stateLabel(next_state), reason);
    }
    else
    {
      RCLCPP_INFO(
          this->get_logger(), "%s --> %s",
          stateLabel(state_), stateLabel(next_state));
    }
    state_ = next_state;
  }

  Eigen::MatrixXd MotoMiniFeedbackStreamNode::calcSrInverse(
      const Eigen::MatrixXd &J, double w, double w0, double k0)
  {
    const double w0_safe = std::max(1e-9, w0);
    const double k = (w < w0_safe) ? k0 * std::pow(1.0 - w / w0_safe, 2.0) : 0.0;
    const Eigen::Index m = J.rows();
    const Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m, m);
    return J.transpose() * (J * J.transpose() + k * I).inverse();
  }

  Eigen::Vector3d MotoMiniFeedbackStreamNode::orientationError(
      const Eigen::Matrix3d &des_R, const Eigen::Matrix3d &cur_R)
  {
    Eigen::Matrix3d err_R = des_R * cur_R.transpose();
    double trace = err_R.trace();
    double angle = std::acos(std::clamp(0.5 * (trace - 1.0), -1.0, 1.0));
    if (std::abs(angle) < 1e-8)
    {
      return Eigen::Vector3d::Zero();
    }
    Eigen::Vector3d axis(
        err_R(2, 1) - err_R(1, 2),
        err_R(0, 2) - err_R(2, 0),
        err_R(1, 0) - err_R(0, 1));
    axis /= (2.0 * std::sin(angle));
    return angle * axis;
  }

  bool MotoMiniFeedbackStreamNode::currentJointVector(Eigen::VectorXd &q) const
  {
    if (!last_joint_state_)
    {
      return false;
    }
    q.resize(static_cast<Eigen::Index>(joint_names_.size()));
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      auto it = std::find(
          last_joint_state_->name.begin(),
          last_joint_state_->name.end(),
          joint_names_[i]);
      if (it == last_joint_state_->name.end())
      {
        return false;
      }
      q[static_cast<Eigen::Index>(i)] =
          last_joint_state_->position[std::distance(last_joint_state_->name.begin(), it)];
    }
    return true;
  }

  Eigen::VectorXd MotoMiniFeedbackStreamNode::filterJointVelocity(
      const Eigen::VectorXd &qdot_raw, double dt)
  {
    if (qdot_raw.size() == 0 || !qdot_raw.allFinite())
    {
      return Eigen::VectorXd::Zero(qdot_raw.size());
    }

    const rclcpp::Time now = this->now();
    if (qdot_filtered_.size() != qdot_raw.size())
    {
      qdot_filtered_ = qdot_raw;
      first_velocity_read_ = false;
      t_last_velocity_filter_update_ = now;
      have_velocity_filter_update_ = true;
      return qdot_filtered_;
    }

    if (!first_velocity_read_ && have_velocity_filter_update_)
    {
      const double cache_age = (now - t_last_velocity_filter_update_).seconds();
      const double same_cycle_window = 0.5 / std::max(1.0, rate_hz_);
      if (cache_age >= 0.0 && cache_age < same_cycle_window)
      {
        return qdot_filtered_;
      }
    }

    const double dt_safe =
        (dt > 1e-6 && dt < 1.0) ? dt : (1.0 / std::max(1.0, rate_hz_));
    const double max_cutoff = std::max(1.0, 0.45 * std::max(1.0, rate_hz_));
    const double cutoff = std::clamp(velocity_filter_cutoff_hz_, 1.0, max_cutoff);
    const double rc = 1.0 / (2.0 * M_PI * cutoff);
    const double alpha = dt_safe / (rc + dt_safe);

    if (first_velocity_read_)
    {
      qdot_filtered_ = qdot_raw;
      first_velocity_read_ = false;
    }
    else
    {
      qdot_filtered_ = alpha * qdot_raw + (1.0 - alpha) * qdot_filtered_;
    }

    t_last_velocity_filter_update_ = now;
    have_velocity_filter_update_ = true;
    return qdot_filtered_;
  }

  bool MotoMiniFeedbackStreamNode::getMeasuredJointVelocity(
      const Eigen::VectorXd &q,
      double dt_hint,
      Eigen::VectorXd &qdot_out,
      Eigen::VectorXd &q_prev,
      rclcpp::Time &t_prev_q,
      bool &have_q_prev)
  {
    qdot_out = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));

    bool velocity_valid = false;
    if (real_robot_ && last_joint_state_ && last_joint_state_->velocity.size() >= joint_names_.size())
    {
      Eigen::VectorXd qdot_driver(static_cast<Eigen::Index>(joint_names_.size()));
      velocity_valid = true;
      for (size_t i = 0; i < joint_names_.size(); ++i)
      {
        auto it = std::find(
            last_joint_state_->name.begin(),
            last_joint_state_->name.end(),
            joint_names_[i]);
        if (it == last_joint_state_->name.end())
        {
          velocity_valid = false;
          break;
        }
        const size_t idx = static_cast<size_t>(std::distance(last_joint_state_->name.begin(), it));
        if (idx >= last_joint_state_->velocity.size())
        {
          velocity_valid = false;
          break;
        }
        const double v = last_joint_state_->velocity[idx];
        if (!std::isfinite(v))
        {
          velocity_valid = false;
          break;
        }
        qdot_driver[static_cast<Eigen::Index>(i)] = v;
      }
      if (velocity_valid && qdot_driver.allFinite())
      {
        qdot_out = qdot_driver;
      }
    }

    const rclcpp::Time now = this->now();
    const double dt_prev = have_q_prev ? (now - t_prev_q).seconds() : dt_hint;
    if (!velocity_valid)
    {
      if (real_robot_)
      {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "[VEL] real_robot=true but hardware velocity invalid, falling back to pose diff.");
      }
      if (have_q_prev && q_prev.size() == q.size() && dt_prev > 1e-6)
      {
        qdot_out = (q - q_prev) / dt_prev;
      }
      else
      {
        qdot_out.setZero();
      }
    }

    q_prev = q;
    t_prev_q = now;
    have_q_prev = true;

    const double filter_dt = (dt_prev > 1e-6 && dt_prev < 1.0) ? dt_prev : dt_hint;
    qdot_out = filterJointVelocity(qdot_out, filter_dt);
    return qdot_out.allFinite();
  }

  bool MotoMiniFeedbackStreamNode::initTrackedPositions()
  {
    Eigen::VectorXd q;
    if (!currentJointVector(q))
    {
      return false;
    }
    tracked_positions_.assign(q.data(), q.data() + q.size());
    tracked_velocities_.assign(joint_names_.size(), 0.0);
    return true;
  }

  bool MotoMiniFeedbackStreamNode::getEEPose(
      const Eigen::VectorXd &q, Eigen::Vector3d &pos, Eigen::Matrix3d &rot) const
  {
    auto fk = manip_->calcFwdKin(q);
    auto it = fk.find(ee_link_);
    if (it == fk.end())
    {
      return false;
    }
    pos = it->second.translation();
    rot = it->second.rotation();
    return true;
  }

  void MotoMiniFeedbackStreamNode::publishFeedback()
  {
    Eigen::VectorXd q;
    if (!currentJointVector(q))
    {
      return;
    }
    Eigen::Vector3d pos;
    Eigen::Matrix3d rot;
    if (!getEEPose(q, pos, rot))
    {
      return;
    }

    const Eigen::Vector3d rpy = rot.eulerAngles(0, 1, 2);
    geometry_msgs::msg::Twist msg;
    msg.linear.x = pos.x();
    msg.linear.y = pos.y();
    msg.linear.z = pos.z();
    msg.angular.x = rpy.x();
    msg.angular.y = rpy.y();
    msg.angular.z = rpy.z();
    pub_feedback_->publish(msg);

    const double dt_hint = have_q_prev_ ? (this->now() - t_prev_q_).seconds() : (1.0 / std::max(1.0, rate_hz_));
    Eigen::VectorXd qdot;
    if (!getMeasuredJointVelocity(q, dt_hint, qdot, q_prev_, t_prev_q_, have_q_prev_))
    {
      return;
    }

    const Eigen::MatrixXd J = manip_->calcJacobian(q, base_link_, ee_link_);
    const Eigen::VectorXd v_cart = J * qdot;

    geometry_msgs::msg::TwistStamped vmsg;
    vmsg.header.stamp = this->now();
    vmsg.header.frame_id = base_link_;
    vmsg.twist.linear.x = v_cart(0);
    vmsg.twist.linear.y = v_cart(1);
    vmsg.twist.linear.z = v_cart(2);
    vmsg.twist.angular.x = v_cart(3);
    vmsg.twist.angular.y = v_cart(4);
    vmsg.twist.angular.z = v_cart(5);
    pub_feedback_vel_->publish(vmsg);
  }

  double MotoMiniFeedbackStreamNode::clampJointVelocityLimits(Eigen::VectorXd &theta_d)
  {
    static const double lim[kNumberOfJoint] = {
        kJoint1VelLimit * kSafetyVelocityAlpha,
        kJoint2VelLimit * kSafetyVelocityAlpha,
        kJoint3VelLimit * kSafetyVelocityAlpha,
        kJoint4VelLimit * kSafetyVelocityAlpha,
        kJoint5VelLimit * kSafetyVelocityAlpha,
        kJoint6VelLimit * kSafetyVelocityAlpha,
    };

    double scale = 1.0;
    const int n = std::min<int>(kNumberOfJoint, theta_d.size());
    for (int i = 0; i < n; ++i)
    {
      const double a = std::abs(theta_d[i]);
      if (a > lim[i] && a > 1e-12)
      {
        scale = std::min(scale, lim[i] / a);
      }
    }
    if (scale < 1.0)
    {
      theta_d *= scale;
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Joint velocity clamped with scale %.3f. Continuing tracking.", scale);
    }
    return scale;
  }

  bool MotoMiniFeedbackStreamNode::checkCartesianVelocitySafety(
      const Eigen::Matrix<double, 6, 1> &xdot_actual)
  {
    const double linear = xdot_actual.head<3>().norm();
    const double angular = xdot_actual.tail<3>().norm();
    if (measured_cart_linear_vel_limit_ > 0.0 && linear > measured_cart_linear_vel_limit_)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Measured Cartesian linear velocity %.4f m/s exceeds limit %.4f m/s",
          linear, measured_cart_linear_vel_limit_);
      return false;
    }
    if (measured_cart_angular_vel_limit_ > 0.0 && angular > measured_cart_angular_vel_limit_)
    {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Measured Cartesian angular velocity %.4f rad/s exceeds limit %.4f rad/s",
          angular, measured_cart_angular_vel_limit_);
      return false;
    }
    return true;
  }

  void MotoMiniFeedbackStreamNode::clampCartesianVelocity(Eigen::Matrix<double, 6, 1> &xdot) const
  {
    const double lin_n = xdot.head<3>().norm();
    if (lin_n > active_phase_config_.max_cart_linear_vel && lin_n > 1e-9)
    {
      xdot.head<3>() *= active_phase_config_.max_cart_linear_vel / lin_n;
    }
    const double ang_n = xdot.tail<3>().norm();
    if (ang_n > active_phase_config_.max_cart_angular_vel && ang_n > 1e-9)
    {
      xdot.tail<3>() *= active_phase_config_.max_cart_angular_vel / ang_n;
    }
  }

  void MotoMiniFeedbackStreamNode::limitCartesianAcceleration(
      Eigen::Matrix<double, 6, 1> &xdot_next,
      const Eigen::Matrix<double, 6, 1> &xdot_prev,
      double dt) const
  {
    const double dt_safe = (dt > 1e-6 && dt < 1.0) ? dt : (1.0 / std::max(1.0, rate_hz_));
    if (max_cart_linear_acc_ > 0.0)
    {
      const Eigen::Vector3d dv_lin = xdot_next.head<3>() - xdot_prev.head<3>();
      const double max_dv_lin = max_cart_linear_acc_ * dt_safe;
      if (dv_lin.norm() > max_dv_lin && dv_lin.norm() > 1e-9)
      {
        xdot_next.head<3>() = xdot_prev.head<3>() + dv_lin.normalized() * max_dv_lin;
      }
    }
    if (max_cart_angular_acc_ > 0.0)
    {
      const Eigen::Vector3d dv_ang = xdot_next.tail<3>() - xdot_prev.tail<3>();
      const double max_dv_ang = max_cart_angular_acc_ * dt_safe;
      if (dv_ang.norm() > max_dv_ang && dv_ang.norm() > 1e-9)
      {
        xdot_next.tail<3>() = xdot_prev.tail<3>() + dv_ang.normalized() * max_dv_ang;
      }
    }
  }

  bool MotoMiniFeedbackStreamNode::checkPositionLimits(const std::vector<double> &pos) const
  {
    static const double lower[kNumberOfJoint] = {
        kJoint1LowerRad + kSafetyJointPaddingRad,
        kJoint2LowerRad + kSafetyJointPaddingRad,
        kJoint3LowerRad + kSafetyJointPaddingRad,
        kJoint4LowerRad + kSafetyJointPaddingRad,
        kJoint5LowerRad + kSafetyJointPaddingRad,
        kJoint6LowerRad + kSafetyJointPaddingRad,
    };
    static const double upper[kNumberOfJoint] = {
        kJoint1UpperRad - kSafetyJointPaddingRad,
        kJoint2UpperRad - kSafetyJointPaddingRad,
        kJoint3UpperRad - kSafetyJointPaddingRad,
        kJoint4UpperRad - kSafetyJointPaddingRad,
        kJoint5UpperRad - kSafetyJointPaddingRad,
        kJoint6UpperRad - kSafetyJointPaddingRad,
    };
    for (int i = 0; i < kNumberOfJoint; ++i)
    {
      if (pos[i] <= lower[i] || pos[i] >= upper[i])
      {
        RCLCPP_WARN(
            this->get_logger(),
            "Joint %d position %.4f rad outside safe range [%.4f, %.4f]",
            i, pos[i], lower[i], upper[i]);
        return false;
      }
    }
    return true;
  }

  void MotoMiniFeedbackStreamNode::publishToTopic(
      rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr &pub,
      const std::vector<double> &pos,
      const std::vector<double> &vel,
      double time_sec)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = this->now();
    traj.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = pos;
    pt.velocities = vel;
    pt.time_from_start = rclcpp::Duration::from_seconds(time_sec);
    traj.points.push_back(pt);
    pub->publish(traj);
  }

  void MotoMiniFeedbackStreamNode::publishArmInit()
  {
    if (tracked_positions_.empty())
    {
      return;
    }
    std::vector<double> zero_vel(joint_names_.size(), 0.0);
    publishToTopic(pub_path_cmd_, tracked_positions_, zero_vel, 0.5);
  }

  void MotoMiniFeedbackStreamNode::seed()
  {
    if (tracked_positions_.empty())
    {
      return;
    }
    std::vector<double> zero_vel(joint_names_.size(), 0.0);
    publishToTopic(pub_joint_cmd_, tracked_positions_, zero_vel, 0.0);
    streaming_time_ = 0.0;
    is_active_ = true;
  }

  void MotoMiniFeedbackStreamNode::publishTrajectory(
      const std::vector<double> &pos, const std::vector<double> &vel)
  {
    publishToTopic(pub_joint_cmd_, pos, vel, streaming_time_);
    streaming_time_ += 1.0 / rate_hz_;
  }

  bool MotoMiniFeedbackStreamNode::computeControlStep(
      const Eigen::VectorXd &q,
      const Eigen::Vector3d &des_pos,
      const Eigen::Matrix3d &des_rot,
      double dt,
      Eigen::VectorXd &theta_d)
  {
    Eigen::Vector3d ee_pos;
    Eigen::Matrix3d ee_rot;
    if (!getEEPose(q, ee_pos, ee_rot))
    {
      return false;
    }

    const auto &cfg = active_phase_config_;
    e_p_ = des_pos - ee_pos;
    e_o_ = orientationError(des_rot, ee_rot);

    const double t_active = (this->now() - t_start_).seconds();
    const double s_t = 1.0 - std::exp(-cfg.adaptive_lambda * std::max(0.0, t_active));
    const double s_e_pos = std::tanh(cfg.adaptive_alpha_pos * e_p_.norm());
    const double s_e_ori = std::tanh(cfg.adaptive_alpha_ori * e_o_.norm());

    const Eigen::MatrixXd J = manip_->calcJacobian(q, base_link_, ee_link_);
    const double w = std::sqrt(std::max(0.0, (J * J.transpose()).determinant()));
    const double w0_safe = std::max(1e-9, w0_);
    const double s_w = std::clamp(w / w0_safe, 0.2, 1.0);

    const double k_pos_var = s_w * (cfg.k_pos_min + s_t * s_e_pos * (cfg.k_pos_max - cfg.k_pos_min));
    const double m_pos_var = cfg.m_pos_max - s_t * s_e_pos * (cfg.m_pos_max - cfg.m_pos_min);
    const double d_pos_var = 2.0 * cfg.zeta_pos * std::sqrt(std::max(1e-12, m_pos_var * k_pos_var));
    const double k_ori_var = s_w * (cfg.k_ori_min + s_t * s_e_ori * (cfg.k_ori_max - cfg.k_ori_min));
    const double m_ori_var = cfg.m_ori_max - s_t * s_e_ori * (cfg.m_ori_max - cfg.m_ori_min);
    const double d_ori_var = 2.0 * cfg.zeta_ori * std::sqrt(std::max(1e-12, m_ori_var * k_ori_var));

    const double dt_safe = (dt > 1e-6 && dt < 1.0) ? dt : (1.0 / std::max(1.0, rate_hz_));
    e_p_int_ += e_p_ * dt_safe;
    e_o_int_ += e_o_ * dt_safe;
    e_p_int_ = e_p_int_.cwiseMax(Eigen::Vector3d::Constant(-cfg.i_clamp_pos))
                   .cwiseMin(Eigen::Vector3d::Constant(cfg.i_clamp_pos));
    e_o_int_ = e_o_int_.cwiseMax(Eigen::Vector3d::Constant(-cfg.i_clamp_ori))
                   .cwiseMin(Eigen::Vector3d::Constant(cfg.i_clamp_ori));
    if (!e_p_int_.allFinite())
    {
      e_p_int_.setZero();
    }
    if (!e_o_int_.allFinite())
    {
      e_o_int_.setZero();
    }

    Eigen::VectorXd qdot;
    if (!getMeasuredJointVelocity(q, dt_safe, qdot, q_prev_ctrl_, t_prev_q_ctrl_, have_q_prev_ctrl_))
    {
      return false;
    }

    const Eigen::Matrix<double, 6, 1> xdot_actual = J * qdot;
    // Warn if measured velocity is high but do not stop tracking.
    // The commanded output is already capped by clampCartesianVelocity(), so
    // a brief measured spike does not mean the controller is over-commanding.
    checkCartesianVelocitySafety(xdot_actual);

    Eigen::Matrix<double, 6, 1> xdot_des = Eigen::Matrix<double, 6, 1>::Zero();
    const double dt_vel = (this->now() - t_last_target_vel_cb_).seconds();
    if (dt_vel < kTargetVelTimeoutSec)
    {
      xdot_des.head<3>() = latest_target_vel_.head<3>();
      xdot_des.tail<3>() = des_rot * latest_target_vel_.tail<3>();
    }
    if (xdot_des.head<3>().norm() <= targetVelocityDeadband())
    {
      xdot_des.head<3>().setZero();
    }
    if (xdot_des.tail<3>().norm() <= targetVelocityDeadband())
    {
      xdot_des.tail<3>().setZero();
    }

    Eigen::Matrix<double, 6, 1> F_collision = Eigen::Matrix<double, 6, 1>::Zero();
    const double dt_collision_wrench = (this->now() - t_last_collision_wrench_cb_).seconds();
    if (dt_collision_wrench < collision_wrench_timeout_sec_)
    {
      F_collision = latest_collision_wrench_;
    }
    F_collision.head<3>() *= collision_force_scale_;
    const double f_norm = F_collision.head<3>().norm();
    if (collision_force_max_ > 0.0 && f_norm > collision_force_max_ && f_norm > 1e-9)
    {
      F_collision.head<3>() *= collision_force_max_ / f_norm;
    }
    F_collision = filterCollisionWrench(F_collision, dt_safe);

    const double collision_force_mag = F_collision.head<3>().norm();
    const double force_ratio = collision_force_mag / std::max(1e-6, cfg.i_force_ref);
    const double attenuation = std::exp(-std::pow(std::max(0.0, force_ratio), cfg.i_force_shape));
    const double i_scale = cfg.i_force_min_scale + (1.0 - cfg.i_force_min_scale) * attenuation;
    const double i_gain_pos_eff = cfg.i_gain_pos * i_scale;
    const double i_gain_ori_eff = cfg.i_gain_ori * i_scale;
    if (cfg.i_collision_decay_rate > 0.0 && i_scale < 1.0)
    {
      const double leak = std::exp(-cfg.i_collision_decay_rate * (1.0 - i_scale) * dt_safe);
      e_p_int_ *= leak;
      e_o_int_ *= leak;
    }

    const double dt_dist = (this->now() - t_last_collision_distance_cb_).seconds();
    const double dt_norm = (this->now() - t_last_collision_normal_cb_).seconds();
    const bool collision_constraint_active =
        (dt_dist < collision_constraint_timeout_sec_) &&
        (dt_norm < collision_constraint_timeout_sec_) &&
        latest_collision_normal_.allFinite() &&
        (latest_collision_normal_.norm() > 1e-6) &&
        std::isfinite(latest_collision_distance_);

    double gamma = 0.0;
    Eigen::Vector3d n_away = Eigen::Vector3d::Zero();
    if (collision_constraint_active)
    {
      n_away = latest_collision_normal_.normalized();
      const double span = std::max(1e-6, collision_guard_distance_ - collision_task_distance_);
      gamma = 1.0 - std::clamp(
                        (latest_collision_distance_ - collision_task_distance_) / span, 0.0, 1.0);
      gamma = std::min(gamma, collision_projection_max_gamma_);
    }

    if (collision_constraint_active && gamma > 0.0)
    {
      Eigen::Vector3d F_pub = -F_collision.head<3>();
      if (F_pub.allFinite() && n_away.allFinite() && n_away.norm() > 1e-9)
      {
        const double f_n = F_pub.dot(n_away);
        if (f_n > 0.0)
        {
          const Eigen::Vector3d F_normal = f_n * n_away;
          const Eigen::Vector3d F_tangent = F_pub - F_normal;
          F_pub = F_tangent + (1.0 - gamma) * F_normal;
        }
        else if (f_n < 0.0)
        {
          F_pub -= f_n * n_away;
        }
        F_collision.head<3>() = -F_pub;
      }
    }

    if (collision_constraint_active && gamma > 0.0)
    {
      const double vd_into = xdot_des.head<3>().dot(n_away);
      if (vd_into < 0.0)
      {
        xdot_des.head<3>() -= gamma * vd_into * n_away;
      }
    }

    const Eigen::Matrix<double, 6, 1> v_err = xdot_des - xdot_ref_;
    Eigen::Vector3d F_goal_pos =
        k_pos_var * e_p_ + i_gain_pos_eff * e_p_int_ + d_pos_var * v_err.head<3>();
    Eigen::Vector3d F_goal_ori =
        k_ori_var * e_o_ + i_gain_ori_eff * e_o_int_ + d_ori_var * v_err.tail<3>();

    if (collision_goal_suppression_ && gamma > 0.0)
    {
      const double F_into = F_goal_pos.dot(n_away);
      if (F_into < 0.0)
      {
        F_goal_pos += -gamma * F_into * n_away;
      }
    }

    Eigen::Matrix<double, 6, 1> xddot_ref;
    xddot_ref.head<3>() = (F_goal_pos - F_collision.head<3>()) / std::max(1e-9, m_pos_var);
    xddot_ref.tail<3>() = (F_goal_ori - F_collision.tail<3>()) / std::max(1e-9, m_ori_var);

    const Eigen::Matrix<double, 6, 1> xdot_prev = xdot_ref_;
    Eigen::Matrix<double, 6, 1> xdot_next = xdot_ref_ + xddot_ref * dt_safe;
    limitCartesianAcceleration(xdot_next, xdot_prev, dt_safe);
    xdot_ref_ = xdot_next;

    if (enable_collision_projection_ && gamma > 0.0)
    {
      const double v_into = xdot_ref_.head<3>().dot(n_away);
      if (v_into < 0.0)
      {
        xdot_ref_.head<3>() -= gamma * v_into * n_away;
      }
    }
    clampCartesianVelocity(xdot_ref_);

    theta_d = calcSrInverse(J, w, w0_, k0_) * xdot_ref_;
    const double joint_scale = clampJointVelocityLimits(theta_d);
    if (joint_scale < 1.0)
    {
      const Eigen::VectorXd xdot_limited = J * theta_d;
      if (xdot_limited.size() == 6 && xdot_limited.allFinite())
      {
        xdot_ref_ = xdot_limited;
        clampCartesianVelocity(xdot_ref_);
      }
    }

    if (joint_scale < 1.0 && enable_collision_projection_ && collision_constraint_active && gamma > 0.0)
    {
      const double v_into = xdot_ref_.head<3>().dot(n_away);
      if (v_into < 0.0)
      {
        xdot_ref_.head<3>() -= gamma * v_into * n_away;
      }
    }

    return true;
  }

  bool MotoMiniFeedbackStreamNode::initializeReferenceVelocityFromMeasuredState()
  {
    Eigen::VectorXd q;
    if (!currentJointVector(q))
    {
      xdot_ref_.setZero();
      return false;
    }
    Eigen::VectorXd qdot;
    if (!getMeasuredJointVelocity(
            q, 1.0 / std::max(1.0, rate_hz_), qdot, q_prev_ctrl_, t_prev_q_ctrl_, have_q_prev_ctrl_))
    {
      xdot_ref_.setZero();
      return false;
    }
    const Eigen::MatrixXd J = manip_->calcJacobian(q, base_link_, ee_link_);
    const Eigen::VectorXd xdot_measured = J * qdot;
    if (xdot_measured.size() != 6 || !xdot_measured.allFinite())
    {
      xdot_ref_.setZero();
      return false;
    }
    xdot_ref_ = xdot_measured;
    clampCartesianVelocity(xdot_ref_);
    return true;
  }

  double MotoMiniFeedbackStreamNode::lowPassAlpha(double cutoff_hz, double dt) const
  {
    if (dt <= 0.0)
    {
      return 1.0;
    }
    return std::clamp(1.0 - std::exp(-2.0 * M_PI * cutoff_hz * dt), 0.0, 1.0);
  }

  double MotoMiniFeedbackStreamNode::targetVelocityDeadband() const
  {
    return std::max(0.002, 0.01 * active_phase_config_.max_cart_linear_vel);
  }

  double MotoMiniFeedbackStreamNode::collisionForceAttackHz() const
  {
    const double tau = std::max(0.02, 0.2 * collision_wrench_timeout_sec_);
    return 1.0 / tau;
  }

  double MotoMiniFeedbackStreamNode::collisionForceReleaseHz() const
  {
    return std::max(1.0, 0.35 * collisionForceAttackHz());
  }

  Eigen::Matrix<double, 6, 1> MotoMiniFeedbackStreamNode::filterCollisionWrench(
      const Eigen::Matrix<double, 6, 1> &raw_wrench, double dt)
  {
    if (dt <= 0.0 || dt > 1.0)
    {
      filtered_collision_wrench_ = raw_wrench;
      return filtered_collision_wrench_;
    }
    const double cutoff_hz =
        (raw_wrench.head<3>().norm() >= filtered_collision_wrench_.head<3>().norm()) ? collisionForceAttackHz() : collisionForceReleaseHz();
    const double alpha = lowPassAlpha(cutoff_hz, dt);
    filtered_collision_wrench_ += alpha * (raw_wrench - filtered_collision_wrench_);
    if (filtered_collision_wrench_.norm() < 1e-6 && raw_wrench.norm() < 1e-6)
    {
      filtered_collision_wrench_.setZero();
    }
    return filtered_collision_wrench_;
  }

  void MotoMiniFeedbackStreamNode::resetVirtualState()
  {
    xdot_ref_.setZero();
    e_p_int_.setZero();
    e_o_int_.setZero();
    have_q_prev_ctrl_ = false;
    q_prev_ctrl_.resize(0);
    first_velocity_read_ = true;
    qdot_filtered_.resize(0);
    have_velocity_filter_update_ = false;
    t_last_velocity_filter_update_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    latest_collision_wrench_.setZero();
    filtered_collision_wrench_.setZero();
    t_last_collision_wrench_cb_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    latest_collision_distance_ = std::numeric_limits<double>::infinity();
    latest_collision_normal_.setZero();
    t_last_collision_distance_cb_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    t_last_collision_normal_cb_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void MotoMiniFeedbackStreamNode::resetControlWindow()
  {
    resetVirtualState();
    e_p_.setZero();
    e_o_.setZero();
    t_start_ = this->now();
    t_last_ = this->now();
  }

  void MotoMiniFeedbackStreamNode::handleIdle()
  {
    if (last_state_ != state_)
    {
      last_state_ = state_;
      resetVirtualState();
    }
    if (last_joint_state_)
    {
      initTrackedPositions();
      if (is_active_)
      {
        std::vector<double> zero_vel(joint_names_.size(), 0.0);
        publishTrajectory(tracked_positions_, zero_vel);
      }
    }
  }

  void MotoMiniFeedbackStreamNode::handleStop()
  {
    if (last_state_ != state_)
    {
      last_state_ = state_;
      resetVirtualState();
    }
    if (last_joint_state_)
    {
      initTrackedPositions();
      if (is_active_)
      {
        std::vector<double> zero_vel(joint_names_.size(), 0.0);
        publishTrajectory(tracked_positions_, zero_vel);
      }
    }
  }

  void MotoMiniFeedbackStreamNode::handleArming()
  {
    if (last_state_ != state_)
    {
      last_state_ = state_;
      t_arming_start_ = this->now();
      arm_trigger_sent_ = false;
    }
    double elapsed = (this->now() - t_arming_start_).seconds();
    if (!arm_trigger_sent_)
    {
      if (elapsed < kArmPreDelaySec)
      {
        return;
      }
      if (initTrackedPositions())
      {
        publishArmInit();
        arm_trigger_sent_ = true;
      }
      return;
    }

    if (elapsed < kArmPreDelaySec + kArmPostDelaySec)
    {
      return;
    }

    if (initTrackedPositions())
    {
      seed();
      resetControlWindow();
      initializeReferenceVelocityFromMeasuredState();
      transitionTo(pending_state_, "arming complete");
    }
  }

  void MotoMiniFeedbackStreamNode::handleInit()
  {
    if (last_state_ != state_)
    {
      last_state_ = state_;
    }
    if (!has_init_pose_)
    {
      return;
    }

    Eigen::VectorXd q;
    if (!currentJointVector(q))
    {
      return;
    }

    Eigen::Vector3d ee_pos;
    Eigen::Matrix3d ee_rot;
    if (!getEEPose(q, ee_pos, ee_rot))
    {
      return;
    }

    double dx = std::abs(init_pose_.pose.position.x - ee_pos.x());
    double dy = std::abs(init_pose_.pose.position.y - ee_pos.y());
    double dz = std::abs(init_pose_.pose.position.z - ee_pos.z());
    if (dx < kPositionErrorThreshold && dy < kPositionErrorThreshold && dz < kPositionErrorThreshold)
    {
      is_init_done_ = true;
      resetControlWindow();
      initializeReferenceVelocityFromMeasuredState();
      transitionTo(STATE_POSE_FOLLOW, "init pose reached");
      return;
    }

    Eigen::Quaterniond q_des(
        init_pose_.pose.orientation.w,
        init_pose_.pose.orientation.x,
        init_pose_.pose.orientation.y,
        init_pose_.pose.orientation.z);
    q_des.normalize();
    Eigen::Vector3d des_pos(
        init_pose_.pose.position.x,
        init_pose_.pose.position.y,
        init_pose_.pose.position.z);

    double dt = (this->now() - t_last_).seconds();
    t_last_ = this->now();

    Eigen::VectorXd theta_d;
    if (!computeControlStep(q, des_pos, q_des.toRotationMatrix(), dt, theta_d))
    {
      return;
    }
    if (tracked_positions_.size() != static_cast<size_t>(theta_d.size()) && !initTrackedPositions())
    {
      return;
    }

    const std::vector<double> prev_pos = tracked_positions_;
    const double dt_safe = (dt > 1e-6 && dt < 1.0) ? dt : (1.0 / std::max(1.0, rate_hz_));
    for (size_t i = 0; i < tracked_positions_.size(); ++i)
    {
      tracked_positions_[i] = q[static_cast<Eigen::Index>(i)] + theta_d[static_cast<Eigen::Index>(i)] * dt_safe;
      tracked_velocities_[i] = theta_d[static_cast<Eigen::Index>(i)];
    }
    if (!checkPositionLimits(tracked_positions_))
    {
      tracked_positions_ = prev_pos;
      std::fill(tracked_velocities_.begin(), tracked_velocities_.end(), 0.0);
    }
    publishTrajectory(tracked_positions_, tracked_velocities_);
  }

  void MotoMiniFeedbackStreamNode::handlePoseFollow()
  {
    if (last_state_ != state_)
    {
      last_state_ = state_;
    }
    if (!has_desired_pose_)
    {
      return;
    }

    double dt_cb = (this->now() - t_last_pose_cb_).seconds();
    if (dt_cb > kPoseTimeoutSec)
    {
      if (last_joint_state_)
      {
        initTrackedPositions();
        std::vector<double> zero_vel(joint_names_.size(), 0.0);
        publishTrajectory(tracked_positions_, zero_vel);
      }
      return;
    }

    Eigen::VectorXd q;
    if (!currentJointVector(q))
    {
      return;
    }

    Eigen::Quaterniond q_des(
        desired_pose_.pose.orientation.w,
        desired_pose_.pose.orientation.x,
        desired_pose_.pose.orientation.y,
        desired_pose_.pose.orientation.z);
    q_des.normalize();
    Eigen::Vector3d des_pos(
        desired_pose_.pose.position.x,
        desired_pose_.pose.position.y,
        desired_pose_.pose.position.z);

    double dt = (this->now() - t_last_).seconds();
    t_last_ = this->now();
    const double dt_safe = (dt > 1e-6 && dt < 1.0) ? dt : (1.0 / std::max(1.0, rate_hz_));

    const double dt_vel = (this->now() - t_last_target_vel_cb_).seconds();
    const bool vel_fresh = dt_vel < kTargetVelTimeoutSec;
    const bool vel_active = vel_fresh &&
                            (latest_target_vel_.head<3>().norm() > targetVelocityDeadband() ||
                             latest_target_vel_.tail<3>().norm() > targetVelocityDeadband());
    if (integrate_target_vel_to_pose_ && vel_active)
    {
      des_pos.x() += latest_target_vel_(0) * dt_safe;
      des_pos.y() += latest_target_vel_(1) * dt_safe;
      des_pos.z() += latest_target_vel_(2) * dt_safe;
      const Eigen::Quaterniond delta =
          Eigen::AngleAxisd(latest_target_vel_(3) * dt_safe, Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(latest_target_vel_(4) * dt_safe, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(latest_target_vel_(5) * dt_safe, Eigen::Vector3d::UnitZ());
      q_des = (q_des * delta).normalized();
      desired_pose_.pose.position.x = des_pos.x();
      desired_pose_.pose.position.y = des_pos.y();
      desired_pose_.pose.position.z = des_pos.z();
      desired_pose_.pose.orientation.w = q_des.w();
      desired_pose_.pose.orientation.x = q_des.x();
      desired_pose_.pose.orientation.y = q_des.y();
      desired_pose_.pose.orientation.z = q_des.z();
      t_last_pose_cb_ = this->now();
    }

    Eigen::VectorXd theta_d;
    if (!computeControlStep(q, des_pos, q_des.toRotationMatrix(), dt_safe, theta_d))
    {
      return;
    }
    if (tracked_positions_.size() != static_cast<size_t>(theta_d.size()) && !initTrackedPositions())
    {
      return;
    }

    const std::vector<double> prev_pos = tracked_positions_;
    for (size_t i = 0; i < tracked_positions_.size(); ++i)
    {
      tracked_positions_[i] = q[static_cast<Eigen::Index>(i)] + theta_d[static_cast<Eigen::Index>(i)] * dt_safe;
      tracked_velocities_[i] = theta_d[static_cast<Eigen::Index>(i)];
    }
    if (!checkPositionLimits(tracked_positions_))
    {
      tracked_positions_ = prev_pos;
      std::fill(tracked_velocities_.begin(), tracked_velocities_.end(), 0.0);
    }
    publishTrajectory(tracked_positions_, tracked_velocities_);
  }

  void MotoMiniFeedbackStreamNode::tick()
  {
    publishFeedback();
    switch (state_)
    {
    case STATE_IDLE:
      handleIdle();
      break;
    case STATE_ARMING:
      handleArming();
      break;
    case STATE_INIT:
      handleInit();
      break;
    case STATE_POSE_FOLLOW:
      handlePoseFollow();
      break;
    case STATE_STOP:
      handleStop();
      break;
    }
  }

} // namespace robot_planning
