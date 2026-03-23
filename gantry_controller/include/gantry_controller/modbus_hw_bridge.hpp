#ifndef GANTRY_CONTROLLER__MODBUS_HW_BRIDGE_HPP_
#define GANTRY_CONTROLLER__MODBUS_HW_BRIDGE_HPP_

#include <modbus/modbus.h>

#include <atomic>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace gantry_controller
{

    class ModbusHwBridge : public rclcpp::Node
    {
    public:
        explicit ModbusHwBridge(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~ModbusHwBridge() override;

    private:
        // ── Modbus helpers ────────────────────────────────────────────────────────
        bool connectModbus();
        bool ensureConnected();
        int16_t decode16Signed(uint16_t raw) const;
        uint16_t encode16Signed(int32_t val) const;

        // ── ROS callbacks ─────────────────────────────────────────────────────────
        void commandCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
        void timerCallback();

        // ── ROS interfaces ────────────────────────────────────────────────────────
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
        rclcpp::TimerBase::SharedPtr feedback_timer_;

        // ── Modbus context ────────────────────────────────────────────────────────
        modbus_t *ctx_{nullptr};
        std::mutex modbus_mutex_;
        std::atomic<bool> connected_{false};

        // ── Parameters ────────────────────────────────────────────────────────────
        std::string plc_ip_;
        int plc_port_;
        int plc_unit_id_;
        double scale_factor_;
        int default_velocity_;

        // Modbus register base addresses (MW10000)
        static constexpr int kBaseAddr = 10000;
        // Axis 1 control: [kBaseAddr+0]=pos, [kBaseAddr+1]=vel
        // Axis 1 feedback:[kBaseAddr+2]=pos, [kBaseAddr+3]=vel
        // Axis 2 control: [kBaseAddr+4]=pos, [kBaseAddr+5]=vel
        // Axis 2 feedback:[kBaseAddr+6]=pos, [kBaseAddr+7]=vel
        static constexpr int kNumRegs = 8;

        // Software safety limits (in PLC units)
        static constexpr int32_t kAx1Min = -2800;
        static constexpr int32_t kAx1Max = 0;
        static constexpr int32_t kAx2Min = -600;
        static constexpr int32_t kAx2Max = 0;
    };

} // namespace gantry_controller

#endif // GANTRY_CONTROLLER__MODBUS_HW_BRIDGE_HPP_
