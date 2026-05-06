#include "robot_planning/motomini_feedback_stream_node.hpp"

namespace robot_planning
{

MotoMiniFeedbackStreamNode::MotoMiniFeedbackStreamNode()
: rclcpp::Node("motomini_feedback_stream")
{
  declareParameters();
  loadParameters();

  if (!initializeKinematics()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to initialize Tesseract kinematics. Check if URDF/SRDF is valid.");
    throw std::runtime_error("Tesseract kinematics initialization failed");
  }

  pub_path_cmd_ =
    this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_path_command", 10);
  pub_joint_cmd_ =
    this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_command", 10);
  pub_feedback_ = this->create_publisher<geometry_msgs::msg::Twist>("/motomini/feedback", 10);
  pub_feedback_vel_ =
    this->create_publisher<geometry_msgs::msg::TwistStamped>("/motomini/feedback_vel", 10);

  sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 20,
    std::bind(&MotoMiniFeedbackStreamNode::jointStateCallback, this, std::placeholders::_1));
  sub_desired_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/motomini/target_pose", 1,
    std::bind(&MotoMiniFeedbackStreamNode::desiredPoseCallback, this, std::placeholders::_1));
  sub_init_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/pose_following/init_pose", 1,
    std::bind(&MotoMiniFeedbackStreamNode::initPoseCallback, this, std::placeholders::_1));
  sub_target_vel_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/motomini/target_vel", 10,
    std::bind(&MotoMiniFeedbackStreamNode::targetVelCallback, this, std::placeholders::_1));
  sub_collision_wrench_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/motomini/collision_wrench", 10,
    std::bind(&MotoMiniFeedbackStreamNode::collisionWrenchCallback, this, std::placeholders::_1));
  sub_collision_distance_ = this->create_subscription<std_msgs::msg::Float64>(
    "/motomini/collision_distance", 10,
    std::bind(&MotoMiniFeedbackStreamNode::collisionDistanceCallback, this, std::placeholders::_1));
  sub_collision_normal_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/motomini/collision_normal", 10,
    std::bind(&MotoMiniFeedbackStreamNode::collisionNormalCallback, this, std::placeholders::_1));
  sub_control_phase_ = this->create_subscription<std_msgs::msg::String>(
    "/motomini/control_phase", 10,
    std::bind(&MotoMiniFeedbackStreamNode::controlPhaseCallback, this, std::placeholders::_1));

  srv_start_ = this->create_service<std_srvs::srv::Trigger>(
    "/pose_following/start",
    std::bind(
      &MotoMiniFeedbackStreamNode::startCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  srv_stop_ = this->create_service<std_srvs::srv::Trigger>(
    "/pose_following/stop",
    std::bind(
      &MotoMiniFeedbackStreamNode::stopCallback, this,
      std::placeholders::_1, std::placeholders::_2));
  srv_init_start_ = this->create_service<std_srvs::srv::Trigger>(
    "/pose_following/init_start",
    std::bind(
      &MotoMiniFeedbackStreamNode::initStartCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  t_start_ = this->now();
  t_last_ = this->now();
  t_last_pose_cb_ = this->now();
  t_last_target_vel_cb_ = this->now();

  auto period_ns = std::chrono::nanoseconds(
    static_cast<int64_t>(1e9 / std::max(1.0, rate_hz_)));
  timer_ = this->create_wall_timer(
    period_ns, std::bind(&MotoMiniFeedbackStreamNode::tick, this));

  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&MotoMiniFeedbackStreamNode::onParametersSet, this, std::placeholders::_1));

  logConfiguration();
}

void MotoMiniFeedbackStreamNode::declareParameters()
{
  this->declare_parameter<std::string>(
    "robot_description", "package://robot_planning/urdf/motoman_motomini.urdf");
  this->declare_parameter<std::string>(
    "robot_description_semantic", "package://robot_planning/urdf/motoman_motomini.srdf");
  this->declare_parameter<std::string>("manipulator_group", "manipulator");
  this->declare_parameter<std::string>("base_link", "base_link");
  this->declare_parameter<std::string>("ee_link", "tool0");
  this->declare_parameter<double>("rate_hz", kNodeRate);
  this->declare_parameter<double>("kp_max", kDefaultLegacyKpMax);
  this->declare_parameter<double>("ko_max", kDefaultLegacyKoMax);
  this->declare_parameter<double>("kdp", kDefaultLegacyKdp);
  this->declare_parameter<double>("kdo", kDefaultLegacyKdo);
  this->declare_parameter<double>("w0", kDefaultW0);
  this->declare_parameter<double>("k0", kDefaultK0);
  this->declare_parameter<double>("theta_d_lim", 3.14);
  this->declare_parameter<bool>("enable_seed", false);

  declarePhaseParameters("", phase_profiles_.base);
  declarePhaseParameters("approach", phase_profiles_.base);
  declarePhaseParameters("near", phase_profiles_.base);
  declarePhaseParameters("tracking", phase_profiles_.base);

  this->declare_parameter<std::string>("control_phase", "default");
  this->declare_parameter<double>("collision_wrench_timeout_sec", 0.2);
  this->declare_parameter<bool>("enable_collision_projection", true);
  this->declare_parameter<bool>("collision_goal_suppression", true);
  this->declare_parameter<double>("collision_guard_distance", 0.03);
  this->declare_parameter<double>("collision_task_distance", 0.005);
  this->declare_parameter<double>("collision_stop_distance", 0.001);
  this->declare_parameter<double>("collision_projection_max_gamma", 1.0);
  this->declare_parameter<double>("collision_constraint_timeout_sec", 0.2);
  this->declare_parameter<double>("collision_force_scale", 1.0);
  this->declare_parameter<double>("collision_force_max", 5.0);
  this->declare_parameter<bool>("real_robot", true);
  this->declare_parameter<double>("velocity_filter_cutoff_hz", 15.0);
  this->declare_parameter<double>("max_cart_linear_acc", 0.8);
  this->declare_parameter<double>("max_cart_angular_acc", 2.5);
  this->declare_parameter<double>("measured_cart_linear_vel_limit", 1.0);
  this->declare_parameter<double>("measured_cart_angular_vel_limit", 3.0);
  this->declare_parameter<bool>("integrate_target_vel_to_pose", true);
}

void MotoMiniFeedbackStreamNode::loadParameters()
{
  this->get_parameter("robot_description", urdf_xml_);
  this->get_parameter("robot_description_semantic", srdf_xml_);
  manipulator_group_ = this->get_parameter("manipulator_group").as_string();
  base_link_ = this->get_parameter("base_link").as_string();
  ee_link_ = this->get_parameter("ee_link").as_string();
  rate_hz_ = this->get_parameter("rate_hz").as_double();
  w0_ = this->get_parameter("w0").as_double();
  k0_ = this->get_parameter("k0").as_double();
  enable_seed_ = this->get_parameter("enable_seed").as_bool();

  phase_profiles_.base = readPhaseParameters("", phase_profiles_.base);

  const auto & overrides =
    this->get_node_parameters_interface()->get_parameter_overrides();
  const bool legacy_kp_override = overrides.find("kp_max") != overrides.end();
  const bool legacy_ko_override = overrides.find("ko_max") != overrides.end();
  const bool new_k_pos_max_override = overrides.find("k_pos_max") != overrides.end();
  const bool new_k_ori_max_override = overrides.find("k_ori_max") != overrides.end();
  if (legacy_kp_override && !new_k_pos_max_override) {
    phase_profiles_.base.k_pos_max =
      kDefaultKPosMax *
      std::max(0.0, this->get_parameter("kp_max").as_double()) / kDefaultLegacyKpMax;
  }
  if (legacy_ko_override && !new_k_ori_max_override) {
    phase_profiles_.base.k_ori_max =
      kDefaultKOriMax *
      std::max(0.0, this->get_parameter("ko_max").as_double()) / kDefaultLegacyKoMax;
  }
  sanitizeAdaptiveConfig(phase_profiles_.base);

  phase_profiles_.approach = readPhaseParameters("approach", phase_profiles_.base);
  phase_profiles_.near = readPhaseParameters("near", phase_profiles_.base);
  phase_profiles_.tracking = readPhaseParameters("tracking", phase_profiles_.base);

  active_phase_ = parseControlPhase(this->get_parameter("control_phase").as_string());

  collision_wrench_timeout_sec_ = this->get_parameter("collision_wrench_timeout_sec").as_double();
  enable_collision_projection_ = this->get_parameter("enable_collision_projection").as_bool();
  collision_goal_suppression_ = this->get_parameter("collision_goal_suppression").as_bool();
  collision_guard_distance_ = this->get_parameter("collision_guard_distance").as_double();
  collision_task_distance_ = this->get_parameter("collision_task_distance").as_double();
  collision_stop_distance_ = this->get_parameter("collision_stop_distance").as_double();
  collision_projection_max_gamma_ =
    this->get_parameter("collision_projection_max_gamma").as_double();
  collision_constraint_timeout_sec_ =
    this->get_parameter("collision_constraint_timeout_sec").as_double();
  collision_force_scale_ = this->get_parameter("collision_force_scale").as_double();
  collision_force_max_ = this->get_parameter("collision_force_max").as_double();
  real_robot_ = this->get_parameter("real_robot").as_bool();
  velocity_filter_cutoff_hz_ = this->get_parameter("velocity_filter_cutoff_hz").as_double();
  max_cart_linear_acc_ = this->get_parameter("max_cart_linear_acc").as_double();
  max_cart_angular_acc_ = this->get_parameter("max_cart_angular_acc").as_double();
  measured_cart_linear_vel_limit_ =
    this->get_parameter("measured_cart_linear_vel_limit").as_double();
  measured_cart_angular_vel_limit_ =
    this->get_parameter("measured_cart_angular_vel_limit").as_double();
  integrate_target_vel_to_pose_ =
    this->get_parameter("integrate_target_vel_to_pose").as_bool();

  collision_stop_distance_ = std::max(0.0, collision_stop_distance_);
  collision_task_distance_ = std::max(collision_stop_distance_, collision_task_distance_);
  collision_guard_distance_ = std::max(collision_task_distance_, collision_guard_distance_);
  collision_projection_max_gamma_ = std::clamp(collision_projection_max_gamma_, 0.0, 1.0);
  collision_force_scale_ = std::max(0.0, collision_force_scale_);
  collision_force_max_ = std::max(0.0, collision_force_max_);
  velocity_filter_cutoff_hz_ = std::max(1.0, velocity_filter_cutoff_hz_);
  max_cart_linear_acc_ = std::max(0.0, max_cart_linear_acc_);
  max_cart_angular_acc_ = std::max(0.0, max_cart_angular_acc_);
  measured_cart_linear_vel_limit_ = std::max(0.0, measured_cart_linear_vel_limit_);
  measured_cart_angular_vel_limit_ = std::max(0.0, measured_cart_angular_vel_limit_);
  w0_ = std::max(1e-9, w0_);
  k0_ = std::max(0.0, k0_);

  refreshActivePhaseConfig();
}

void MotoMiniFeedbackStreamNode::logConfiguration() const
{
  RCLCPP_INFO(
    this->get_logger(),
    "motomini_feedback_stream ready. group=%s ee=%s rate=%.0f Hz active_phase=%s",
    manipulator_group_.c_str(), ee_link_.c_str(), rate_hz_, phaseLabel(active_phase_));
  RCLCPP_INFO(
    this->get_logger(),
    "Base phase gains: Mpos[%.3f, %.3f] Kpos[%.3f, %.3f] Mori[%.3f, %.3f] Kori[%.3f, %.3f]",
    phase_profiles_.base.m_pos_min, phase_profiles_.base.m_pos_max,
    phase_profiles_.base.k_pos_min, phase_profiles_.base.k_pos_max,
    phase_profiles_.base.m_ori_min, phase_profiles_.base.m_ori_max,
    phase_profiles_.base.k_ori_min, phase_profiles_.base.k_ori_max);
  RCLCPP_INFO(
    this->get_logger(),
    "Phase switching topic: /motomini/control_phase with values default|approach|near|tracking");
}

std::string MotoMiniFeedbackStreamNode::phaseParamName(
  const std::string & phase_prefix, const std::string & suffix) const
{
  if (phase_prefix.empty()) {
    return suffix;
  }
  return "phase_" + phase_prefix + "_" + suffix;
}

void MotoMiniFeedbackStreamNode::declarePhaseParameters(
  const std::string & phase_prefix, const AdaptivePhaseConfig & defaults)
{
  this->declare_parameter<double>(phaseParamName(phase_prefix, "m_pos_min"), defaults.m_pos_min);
  this->declare_parameter<double>(phaseParamName(phase_prefix, "m_pos_max"), defaults.m_pos_max);
  this->declare_parameter<double>(phaseParamName(phase_prefix, "k_pos_min"), defaults.k_pos_min);
  this->declare_parameter<double>(phaseParamName(phase_prefix, "k_pos_max"), defaults.k_pos_max);
  this->declare_parameter<double>(phaseParamName(phase_prefix, "zeta_pos"), defaults.zeta_pos);
  this->declare_parameter<double>(phaseParamName(phase_prefix, "m_ori_min"), defaults.m_ori_min);
  this->declare_parameter<double>(phaseParamName(phase_prefix, "m_ori_max"), defaults.m_ori_max);
  this->declare_parameter<double>(phaseParamName(phase_prefix, "k_ori_min"), defaults.k_ori_min);
  this->declare_parameter<double>(phaseParamName(phase_prefix, "k_ori_max"), defaults.k_ori_max);
  this->declare_parameter<double>(phaseParamName(phase_prefix, "zeta_ori"), defaults.zeta_ori);
  this->declare_parameter<double>(
    phaseParamName(phase_prefix, "adaptive_lambda"), defaults.adaptive_lambda);
  this->declare_parameter<double>(
    phaseParamName(phase_prefix, "adaptive_alpha_pos"), defaults.adaptive_alpha_pos);
  this->declare_parameter<double>(
    phaseParamName(phase_prefix, "adaptive_alpha_ori"), defaults.adaptive_alpha_ori);
  this->declare_parameter<double>(
    phaseParamName(phase_prefix, "max_cart_linear_vel"), defaults.max_cart_linear_vel);
  this->declare_parameter<double>(
    phaseParamName(phase_prefix, "max_cart_angular_vel"), defaults.max_cart_angular_vel);
  this->declare_parameter<double>(phaseParamName(phase_prefix, "i_gain_pos"), defaults.i_gain_pos);
  this->declare_parameter<double>(phaseParamName(phase_prefix, "i_gain_ori"), defaults.i_gain_ori);
  this->declare_parameter<double>(phaseParamName(phase_prefix, "i_clamp_pos"), defaults.i_clamp_pos);
  this->declare_parameter<double>(phaseParamName(phase_prefix, "i_clamp_ori"), defaults.i_clamp_ori);
  this->declare_parameter<double>(phaseParamName(phase_prefix, "i_force_ref"), defaults.i_force_ref);
  this->declare_parameter<double>(phaseParamName(phase_prefix, "i_force_shape"), defaults.i_force_shape);
  this->declare_parameter<double>(
    phaseParamName(phase_prefix, "i_force_min_scale"), defaults.i_force_min_scale);
  this->declare_parameter<double>(
    phaseParamName(phase_prefix, "i_collision_decay_rate"), defaults.i_collision_decay_rate);
}

AdaptivePhaseConfig MotoMiniFeedbackStreamNode::readPhaseParameters(
  const std::string & phase_prefix, const AdaptivePhaseConfig & fallback) const
{
  AdaptivePhaseConfig cfg = fallback;
  cfg.m_pos_min = this->get_parameter(phaseParamName(phase_prefix, "m_pos_min")).as_double();
  cfg.m_pos_max = this->get_parameter(phaseParamName(phase_prefix, "m_pos_max")).as_double();
  cfg.k_pos_min = this->get_parameter(phaseParamName(phase_prefix, "k_pos_min")).as_double();
  cfg.k_pos_max = this->get_parameter(phaseParamName(phase_prefix, "k_pos_max")).as_double();
  cfg.zeta_pos = this->get_parameter(phaseParamName(phase_prefix, "zeta_pos")).as_double();
  cfg.m_ori_min = this->get_parameter(phaseParamName(phase_prefix, "m_ori_min")).as_double();
  cfg.m_ori_max = this->get_parameter(phaseParamName(phase_prefix, "m_ori_max")).as_double();
  cfg.k_ori_min = this->get_parameter(phaseParamName(phase_prefix, "k_ori_min")).as_double();
  cfg.k_ori_max = this->get_parameter(phaseParamName(phase_prefix, "k_ori_max")).as_double();
  cfg.zeta_ori = this->get_parameter(phaseParamName(phase_prefix, "zeta_ori")).as_double();
  cfg.adaptive_lambda =
    this->get_parameter(phaseParamName(phase_prefix, "adaptive_lambda")).as_double();
  cfg.adaptive_alpha_pos =
    this->get_parameter(phaseParamName(phase_prefix, "adaptive_alpha_pos")).as_double();
  cfg.adaptive_alpha_ori =
    this->get_parameter(phaseParamName(phase_prefix, "adaptive_alpha_ori")).as_double();
  cfg.max_cart_linear_vel =
    this->get_parameter(phaseParamName(phase_prefix, "max_cart_linear_vel")).as_double();
  cfg.max_cart_angular_vel =
    this->get_parameter(phaseParamName(phase_prefix, "max_cart_angular_vel")).as_double();
  cfg.i_gain_pos = this->get_parameter(phaseParamName(phase_prefix, "i_gain_pos")).as_double();
  cfg.i_gain_ori = this->get_parameter(phaseParamName(phase_prefix, "i_gain_ori")).as_double();
  cfg.i_clamp_pos = this->get_parameter(phaseParamName(phase_prefix, "i_clamp_pos")).as_double();
  cfg.i_clamp_ori = this->get_parameter(phaseParamName(phase_prefix, "i_clamp_ori")).as_double();
  cfg.i_force_ref = this->get_parameter(phaseParamName(phase_prefix, "i_force_ref")).as_double();
  cfg.i_force_shape =
    this->get_parameter(phaseParamName(phase_prefix, "i_force_shape")).as_double();
  cfg.i_force_min_scale =
    this->get_parameter(phaseParamName(phase_prefix, "i_force_min_scale")).as_double();
  cfg.i_collision_decay_rate =
    this->get_parameter(phaseParamName(phase_prefix, "i_collision_decay_rate")).as_double();
  sanitizeAdaptiveConfig(cfg);
  return cfg;
}

void MotoMiniFeedbackStreamNode::sanitizeAdaptiveConfig(AdaptivePhaseConfig & cfg) const
{
  cfg.m_pos_min = std::max(1e-6, cfg.m_pos_min);
  cfg.m_pos_max = std::max(1e-6, cfg.m_pos_max);
  if (cfg.m_pos_min > cfg.m_pos_max) {
    std::swap(cfg.m_pos_min, cfg.m_pos_max);
  }

  cfg.k_pos_min = std::max(1e-6, cfg.k_pos_min);
  cfg.k_pos_max = std::max(1e-6, cfg.k_pos_max);
  if (cfg.k_pos_min > cfg.k_pos_max) {
    std::swap(cfg.k_pos_min, cfg.k_pos_max);
  }

  cfg.m_ori_min = std::max(1e-6, cfg.m_ori_min);
  cfg.m_ori_max = std::max(1e-6, cfg.m_ori_max);
  if (cfg.m_ori_min > cfg.m_ori_max) {
    std::swap(cfg.m_ori_min, cfg.m_ori_max);
  }

  cfg.k_ori_min = std::max(1e-6, cfg.k_ori_min);
  cfg.k_ori_max = std::max(1e-6, cfg.k_ori_max);
  if (cfg.k_ori_min > cfg.k_ori_max) {
    std::swap(cfg.k_ori_min, cfg.k_ori_max);
  }

  cfg.zeta_pos = std::max(0.0, cfg.zeta_pos);
  cfg.zeta_ori = std::max(0.0, cfg.zeta_ori);
  cfg.adaptive_lambda = std::max(0.0, cfg.adaptive_lambda);
  cfg.adaptive_alpha_pos = std::max(0.0, cfg.adaptive_alpha_pos);
  cfg.adaptive_alpha_ori = std::max(0.0, cfg.adaptive_alpha_ori);
  cfg.max_cart_linear_vel = std::max(0.0, cfg.max_cart_linear_vel);
  cfg.max_cart_angular_vel = std::max(0.0, cfg.max_cart_angular_vel);
  cfg.i_gain_pos = std::max(0.0, cfg.i_gain_pos);
  cfg.i_gain_ori = std::max(0.0, cfg.i_gain_ori);
  cfg.i_clamp_pos = std::max(0.0, cfg.i_clamp_pos);
  cfg.i_clamp_ori = std::max(0.0, cfg.i_clamp_ori);
  cfg.i_force_ref = std::max(1e-6, cfg.i_force_ref);
  cfg.i_force_shape = std::max(0.1, cfg.i_force_shape);
  cfg.i_force_min_scale = std::clamp(cfg.i_force_min_scale, 0.0, 1.0);
  cfg.i_collision_decay_rate = std::max(0.0, cfg.i_collision_decay_rate);
}

void MotoMiniFeedbackStreamNode::refreshActivePhaseConfig()
{
  switch (active_phase_) {
    case ControlPhase::Approach:
      active_phase_config_ = phase_profiles_.approach;
      break;
    case ControlPhase::Near:
      active_phase_config_ = phase_profiles_.near;
      break;
    case ControlPhase::Tracking:
      active_phase_config_ = phase_profiles_.tracking;
      break;
    case ControlPhase::Default:
    default:
      active_phase_config_ = phase_profiles_.base;
      break;
  }
  sanitizeAdaptiveConfig(active_phase_config_);
}

MotoMiniFeedbackStreamNode::ControlPhase MotoMiniFeedbackStreamNode::parseControlPhase(
  const std::string & text) const
{
  std::string lowered = text;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), ::tolower);
  if (lowered == "approach" || lowered == "phase1") {
    return ControlPhase::Approach;
  }
  if (lowered == "near" || lowered == "phase2") {
    return ControlPhase::Near;
  }
  if (lowered == "tracking" || lowered == "phase3") {
    return ControlPhase::Tracking;
  }
  return ControlPhase::Default;
}

const char * MotoMiniFeedbackStreamNode::phaseLabel(ControlPhase phase) const
{
  switch (phase) {
    case ControlPhase::Approach:
      return "approach";
    case ControlPhase::Near:
      return "near";
    case ControlPhase::Tracking:
      return "tracking";
    case ControlPhase::Default:
    default:
      return "default";
  }
}

bool MotoMiniFeedbackStreamNode::updateAdaptiveField(
  AdaptivePhaseConfig & cfg, const std::string & key, double value) const
{
  if (key == "m_pos_min") cfg.m_pos_min = value;
  else if (key == "m_pos_max") cfg.m_pos_max = value;
  else if (key == "k_pos_min") cfg.k_pos_min = value;
  else if (key == "k_pos_max") cfg.k_pos_max = value;
  else if (key == "zeta_pos") cfg.zeta_pos = value;
  else if (key == "m_ori_min") cfg.m_ori_min = value;
  else if (key == "m_ori_max") cfg.m_ori_max = value;
  else if (key == "k_ori_min") cfg.k_ori_min = value;
  else if (key == "k_ori_max") cfg.k_ori_max = value;
  else if (key == "zeta_ori") cfg.zeta_ori = value;
  else if (key == "adaptive_lambda") cfg.adaptive_lambda = value;
  else if (key == "adaptive_alpha_pos") cfg.adaptive_alpha_pos = value;
  else if (key == "adaptive_alpha_ori") cfg.adaptive_alpha_ori = value;
  else if (key == "max_cart_linear_vel") cfg.max_cart_linear_vel = value;
  else if (key == "max_cart_angular_vel") cfg.max_cart_angular_vel = value;
  else if (key == "i_gain_pos") cfg.i_gain_pos = value;
  else if (key == "i_gain_ori") cfg.i_gain_ori = value;
  else if (key == "i_clamp_pos") cfg.i_clamp_pos = value;
  else if (key == "i_clamp_ori") cfg.i_clamp_ori = value;
  else if (key == "i_force_ref") cfg.i_force_ref = value;
  else if (key == "i_force_shape") cfg.i_force_shape = value;
  else if (key == "i_force_min_scale") cfg.i_force_min_scale = value;
  else if (key == "i_collision_decay_rate") cfg.i_collision_decay_rate = value;
  else return false;
  return true;
}

rcl_interfaces::msg::SetParametersResult MotoMiniFeedbackStreamNode::onParametersSet(
  const std::vector<rclcpp::Parameter> & params)
{
  auto base = phase_profiles_.base;
  auto approach = phase_profiles_.approach;
  auto near_cfg = phase_profiles_.near;
  auto tracking = phase_profiles_.tracking;
  auto active_phase = active_phase_;

  double w0 = w0_;
  double k0 = k0_;
  bool integrate_target_vel_to_pose = integrate_target_vel_to_pose_;

  for (const auto & param : params) {
    const std::string & name = param.get_name();
    if (name == "control_phase" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      active_phase = parseControlPhase(param.as_string());
      continue;
    }
    if (name == "w0" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      w0 = param.as_double();
      continue;
    }
    if (name == "k0" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      k0 = param.as_double();
      continue;
    }
    if (
      name == "integrate_target_vel_to_pose" &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
    {
      integrate_target_vel_to_pose = param.as_bool();
      continue;
    }
    if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      continue;
    }

    if (updateAdaptiveField(base, name, param.as_double())) {
      continue;
    }

    constexpr const char * prefixes[] = {"phase_approach_", "phase_near_", "phase_tracking_"};
    AdaptivePhaseConfig * targets[] = {&approach, &near_cfg, &tracking};
    bool handled = false;
    for (size_t i = 0; i < 3; ++i) {
      const std::string prefix = prefixes[i];
      if (name.rfind(prefix, 0) == 0) {
        handled = updateAdaptiveField(*targets[i], name.substr(prefix.size()), param.as_double());
        break;
      }
    }
    if (!handled) {
      continue;
    }
  }

  sanitizeAdaptiveConfig(base);
  sanitizeAdaptiveConfig(approach);
  sanitizeAdaptiveConfig(near_cfg);
  sanitizeAdaptiveConfig(tracking);

  phase_profiles_.base = base;
  phase_profiles_.approach = approach;
  phase_profiles_.near = near_cfg;
  phase_profiles_.tracking = tracking;
  active_phase_ = active_phase;
  w0_ = std::max(1e-9, w0);
  k0_ = std::max(0.0, k0);
  integrate_target_vel_to_pose_ = integrate_target_vel_to_pose;
  refreshActivePhaseConfig();

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "updated";
  return result;
}

bool MotoMiniFeedbackStreamNode::initializeKinematics()
{
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  env_ = std::make_shared<tesseract_environment::Environment>();
  if (!env_->init(urdf_xml_, srdf_xml_, locator)) {
    return false;
  }

  manip_ = env_->getKinematicGroup(manipulator_group_);
  if (!manip_) {
    return false;
  }

  joint_names_ = manip_->getJointNames();
  qdot_filtered_ = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joint_names_.size()));
  return !joint_names_.empty();
}

}  // namespace robot_planning
