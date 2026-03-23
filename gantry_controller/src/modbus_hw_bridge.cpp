#include "gantry_controller/modbus_hw_bridge.hpp"

#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

namespace gantry_controller
{

    // ═══════════════════════════════════════════════════════════════════════════
    //  Constructor / Destructor
    // ═══════════════════════════════════════════════════════════════════════════

    ModbusHwBridge::ModbusHwBridge(const rclcpp::NodeOptions &options)
        : Node("modbus_hardware_bridge", options)
    {
        // ── Declare parameters (can be overridden from a YAML file or CLI) ────────
        plc_ip_ = this->declare_parameter<std::string>("plc_ip", "192.168.1.1");
        plc_port_ = this->declare_parameter<int>("plc_port", 502);
        plc_unit_id_ = this->declare_parameter<int>("plc_unit_id", 1);
        scale_factor_ = this->declare_parameter<double>("scale_factor", 10000.0);
        default_velocity_ = this->declare_parameter<int>("default_velocity", 100);
        double feedback_rate_hz = this->declare_parameter<double>("feedback_rate_hz", 20.0);

        // ── ROS interfaces ────────────────────────────────────────────────────────
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 10);

        traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/gantry_controller/joint_trajectory", 10,
            std::bind(&ModbusHwBridge::commandCallback, this, std::placeholders::_1));

        auto period_ms = std::chrono::milliseconds(
            static_cast<int>(1000.0 / feedback_rate_hz));
        feedback_timer_ = this->create_wall_timer(
            period_ms, std::bind(&ModbusHwBridge::timerCallback, this));

        // ── Connect Modbus ────────────────────────────────────────────────────────
        connectModbus();
    }

    ModbusHwBridge::~ModbusHwBridge()
    {
        std::lock_guard<std::mutex> lock(modbus_mutex_);
        if (ctx_)
        {
            modbus_close(ctx_);
            modbus_free(ctx_);
            ctx_ = nullptr;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Modbus helpers
    // ═══════════════════════════════════════════════════════════════════════════

    bool ModbusHwBridge::connectModbus()
    {
        // Always start fresh – free a stale context if present.
        if (ctx_)
        {
            modbus_close(ctx_);
            modbus_free(ctx_);
            ctx_ = nullptr;
        }

        ctx_ = modbus_new_tcp(plc_ip_.c_str(), plc_port_);
        if (!ctx_)
        {
            RCLCPP_ERROR(get_logger(), "modbus_new_tcp() failed: %s", modbus_strerror(errno));
            connected_ = false;
            return false;
        }

        // Set slave / unit ID
        modbus_set_slave(ctx_, plc_unit_id_);

        // Response timeout: 500 ms
        modbus_set_response_timeout(ctx_, 0, 500000);

        if (modbus_connect(ctx_) == -1)
        {
            RCLCPP_ERROR(get_logger(), "Modbus connect to %s:%d failed: %s",
                         plc_ip_.c_str(), plc_port_, modbus_strerror(errno));
            modbus_free(ctx_);
            ctx_ = nullptr;
            connected_ = false;
            return false;
        }

        connected_ = true;
        RCLCPP_INFO(get_logger(), "Connected to Modbus PLC at %s:%d (unit=%d)",
                    plc_ip_.c_str(), plc_port_, plc_unit_id_);
        return true;
    }

    bool ModbusHwBridge::ensureConnected()
    {
        if (connected_)
        {
            return true;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Modbus disconnected – attempting reconnect…");
        return connectModbus();
    }

    /** Two's-complement decode: unsigned 16-bit → signed 16-bit. */
    int16_t ModbusHwBridge::decode16Signed(uint16_t raw) const
    {
        return static_cast<int16_t>(raw);
    }

    /** Two's-complement encode: signed integer → unsigned 16-bit register. */
    uint16_t ModbusHwBridge::encode16Signed(int32_t val) const
    {
        return static_cast<uint16_t>(static_cast<int16_t>(val));
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Command callback  (ROS 2 → PLC)
    // ═══════════════════════════════════════════════════════════════════════════

    void ModbusHwBridge::commandCallback(
        const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        if (msg->points.empty())
        {
            RCLCPP_WARN(get_logger(), "Received empty JointTrajectory, ignoring.");
            return;
        }

        const auto &pt = msg->points[0];
        if (pt.positions.size() < 2)
        {
            RCLCPP_WARN(get_logger(), "JointTrajectory point has fewer than 2 positions, ignoring.");
            return;
        }

        // Convert metres → PLC units
        int32_t pos_x_plc = static_cast<int32_t>(pt.positions[0] * scale_factor_);
        int32_t pos_z_plc = static_cast<int32_t>(pt.positions[1] * scale_factor_);

        // Apply software safety limits
        pos_x_plc = std::clamp(pos_x_plc, kAx1Min, kAx1Max);
        pos_z_plc = std::clamp(pos_z_plc, kAx2Min, kAx2Max);

        uint16_t regs_ax1[2] = {encode16Signed(pos_x_plc),
                                static_cast<uint16_t>(default_velocity_)};
        uint16_t regs_ax2[2] = {encode16Signed(pos_z_plc),
                                static_cast<uint16_t>(default_velocity_)};

        std::lock_guard<std::mutex> lock(modbus_mutex_);

        if (!ensureConnected())
        {
            return;
        }

        // Write Axis 1: MW10000 (pos), MW10001 (vel)
        if (modbus_write_registers(ctx_, kBaseAddr + 0, 2, regs_ax1) == -1)
        {
            RCLCPP_ERROR(get_logger(), "Axis 1 write failed: %s", modbus_strerror(errno));
            connected_ = false;
            return;
        }

        // Write Axis 2: MW10004 (pos), MW10005 (vel)
        if (modbus_write_registers(ctx_, kBaseAddr + 4, 2, regs_ax2) == -1)
        {
            RCLCPP_ERROR(get_logger(), "Axis 2 write failed: %s", modbus_strerror(errno));
            connected_ = false;
            return;
        }

        RCLCPP_INFO(get_logger(), "Command sent → Axis1: %d  Axis2: %d (PLC units)",
                    pos_x_plc, pos_z_plc);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //  Feedback timer  (PLC → ROS 2)
    // ═══════════════════════════════════════════════════════════════════════════

    void ModbusHwBridge::timerCallback()
    {
        std::lock_guard<std::mutex> lock(modbus_mutex_);

        if (!ensureConnected())
        {
            return;
        }

        // Read 8 registers starting at MW10000
        uint16_t regs[kNumRegs] = {};
        if (modbus_read_registers(ctx_, kBaseAddr, kNumRegs, regs) == -1)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                  "Modbus read failed: %s", modbus_strerror(errno));
            connected_ = false;
            return;
        }

        // Feedback registers (indices 2, 3, 6, 7)
        const double pos_x_m = decode16Signed(regs[2]) / scale_factor_;
        const double vel_x_m_s = decode16Signed(regs[3]) / scale_factor_;
        const double pos_z_m = decode16Signed(regs[6]) / scale_factor_;
        const double vel_z_m_s = decode16Signed(regs[7]) / scale_factor_;

        // Publish JointState
        auto js = sensor_msgs::msg::JointState();
        js.header.stamp = this->get_clock()->now();
        js.name = {"joint_x", "joint_z"};
        js.position = {pos_x_m, pos_z_m};
        js.velocity = {vel_x_m_s, vel_z_m_s};

        joint_state_pub_->publish(js);
    }

} // namespace gantry_controller

// ═══════════════════════════════════════════════════════════════════════════
//  main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gantry_controller::ModbusHwBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
