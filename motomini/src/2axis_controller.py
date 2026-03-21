#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from pymodbus.client.sync import ModbusTcpClient

class ModbusHardwareBridge(Node):
    def __init__(self):
        super().__init__('modbus_hardware_bridge')
        
        # Scale factor: 10000 units = 1 meter (e.g., 2800 units = 0.28m)
        self.SCALE_FACTOR = 10000.0

        # Publisher for RViz visualization (Feedback)
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscriber to listen to GUI/ROS2 commands (Control)
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/gantry_controller/joint_trajectory',
            self.command_callback,
            10
        )
        
        # Read from the PLC at 20 Hz (every 0.05 seconds)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # Connect to the Modbus PLC
        self.ip_address = '192.168.1.1'
        self.port = 502
        self.client = ModbusTcpClient(self.ip_address, port=self.port)
        
        if self.client.connect():
            self.get_logger().info(f'Successfully connected to Modbus PLC at {self.ip_address}:{self.port}')
        else:
            self.get_logger().error(f'Failed to connect to Modbus PLC at {self.ip_address}:{self.port}')

    def decode_16bit_signed(self, val):
        """Converts an unsigned 16-bit Modbus register into a signed integer."""
        if val > 32767:
            return val - 65536
        return val

    def encode_16bit_signed(self, val):
        """Converts a signed integer into an unsigned 16-bit Modbus register format."""
        val = int(val)
        if val < 0:
            return val + 65536
        return val

    def command_callback(self, msg):
        """Receives commands from ROS 2 and writes them to the PLC control registers."""
        if not self.client.is_socket_open():
            self.get_logger().warn('Cannot send command. Modbus disconnected.')
            return
        
        try:
            # The ROS 2 Controller/GUI sends target positions in METERS
            pos_x_m = msg.points[0].positions[0]
            pos_z_m = msg.points[0].positions[1]
            
            # Convert meters to PLC units (e.g., -0.28m * 10000 = -2800)
            pos_x_plc = int(pos_x_m * self.SCALE_FACTOR)
            pos_z_plc = int(pos_z_m * self.SCALE_FACTOR)
            
            # Software Safety Limits
            pos_x_plc = max(-2800, min(0, pos_x_plc))
            pos_z_plc = max(-600, min(0, pos_z_plc))
            
            # Encode values (handling Two's Complement for negative numbers)
            modbus_x_pos = self.encode_16bit_signed(pos_x_plc)
            modbus_z_pos = self.encode_16bit_signed(pos_z_plc)
            
            # Assuming a default velocity command of 100 for now (can be adjusted)
            default_vel = 100
            
            # WRITE AXIS 1: MW10000 (Pos) and MW10001 (Vel)
            self.client.write_registers(10000, [modbus_x_pos, default_vel], unit=1)
            
            # WRITE AXIS 2: MW10004 (Pos) and MW10005 (Vel)
            self.client.write_registers(10004, [modbus_z_pos, default_vel], unit=1)
            
            self.get_logger().info(f'Command Sent -> Axis 1: {pos_x_plc}, Axis 2: {pos_z_plc}')
            
        except Exception as e:
            self.get_logger().error(f'Exception during Modbus write: {str(e)}')

    def timer_callback(self):
        """Reads feedback registers and publishes them to RViz."""
        if not self.client.is_socket_open():
            self.get_logger().warn('Modbus socket is closed. Attempting reconnect...', throttle_duration_sec=2.0)
            self.client.connect()
            return

        try:
            # Read 8 registers starting at MW10000
            # [0] MW10000: Ax1 Pos Ctrl | [1] MW10001: Ax1 Vel Ctrl
            # [2] MW10002: Ax1 Pos FB   | [3] MW10003: Ax1 Vel FB
            # [4] MW10004: Ax2 Pos Ctrl | [5] MW10005: Ax2 Vel Ctrl
            # [6] MW10006: Ax2 Pos FB   | [7] MW10007: Ax2 Vel FB
            result = self.client.read_holding_registers(10000, 8, unit=1)
            
            if result.isError():
                self.get_logger().error('Modbus read error', throttle_duration_sec=1.0)
                return

            # Apply Two's Complement to the FEEDBACK registers only
            pos_x_raw = self.decode_16bit_signed(result.registers[2])
            vel_x_raw = self.decode_16bit_signed(result.registers[3])
            pos_z_raw = self.decode_16bit_signed(result.registers[6])
            vel_z_raw = self.decode_16bit_signed(result.registers[7])

            # Convert PLC units back to METERS for RViz
            pos_x_m = pos_x_raw / self.SCALE_FACTOR
            vel_x_m_s = vel_x_raw / self.SCALE_FACTOR
            pos_z_m = pos_z_raw / self.SCALE_FACTOR
            vel_z_m_s = vel_z_raw / self.SCALE_FACTOR

            # Create the JointState message to send to RViz
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            
            # The names MUST exactly match the names in your URDF file
            msg.name = ['joint_x', 'joint_z']
            msg.position = [pos_x_m, pos_z_m]
            msg.velocity = [vel_x_m_s, vel_z_m_s]
            
            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Exception during Modbus read: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ModbusHardwareBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.client.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()