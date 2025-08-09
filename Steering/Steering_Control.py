
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import can
import os
import time
import threading

def map_range(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

class SteeringMotorController(Node):
    def __init__(self):
        super().__init__('steering_motor_controller')

        self.controller_id = 1
        self.control_id = 0x6000000 + self.controller_id
        self.feedback_id = 0x5800000 + self.controller_id

        self.control_mode = 3
        self.current_position = 0

        self.setup_can_interface()
        self.enable_motor()

        self.subscription = self.create_subscription(
            Float32,
            '/cmd_angle',
            self.angle_callback,
            10
        )

        self.monitoring = True
        threading.Thread(target=self.monitor_feedback, daemon=True).start()

        self.get_logger().info("Steering Motor Controller initialized.")

    def setup_can_interface(self):
        os.system('sudo ifconfig can0 down')
        os.system('sudo ip link set can0 type can bitrate 250000')
        os.system("sudo ifconfig can0 txqueuelen 2000")
        os.system('sudo ifconfig can0 up')

        global can0
        can0 = can.interface.Bus(channel='can0', interface='socketcan')
        self.get_logger().info("CAN interface ready.")

    def enable_motor(self):
        msg_enable = can.Message(
            arbitration_id=self.control_id,
            data=[0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00]
        )
        self.send_can_message(msg_enable, "Motor enabled")

    def send_can_message(self, message, success_log):
        try:
            can0.send(message)
            self.get_logger().info(success_log)
            time.sleep(0.05)
        except can.CanOperationError as e:
            self.get_logger().error(f"[ERROR] Failed to send message: {e}")

    def monitor_feedback(self):
        while self.monitoring:
            try:
                msg = can0.recv(timeout=1.0)
                if msg and msg.arbitration_id == self.feedback_id:
                    self.get_logger().info(f"[FEEDBACK] ID: {hex(msg.arbitration_id)} Data: {msg.data.hex()}")
            except Exception as e:
                self.get_logger().error(f"[ERROR] Feedback read failed: {e}")
            time.sleep(0.1)

    def smooth_position_transition(self, current_position, target_position):
        step_size = 10
        delay = 0.05

        if abs(target_position - current_position) < step_size:
            self.send_position_command(target_position)
            self.current_position = target_position
            return

        step = step_size if target_position > current_position else -step_size
        position = current_position

        while (step > 0 and position < target_position) or (step < 0 and position > target_position):
            position += step
            if (step > 0 and position > target_position) or (step < 0 and position < target_position):
                position = target_position

            self.send_position_command(position)
            time.sleep(delay)

        self.current_position = target_position

    def move_steering(self, angle_deg):
        angle_deg = max(min(angle_deg, 5), -5)
        target_position = map_range(angle_deg, -5, 5, -50, 50)

        self.get_logger().info(f"Angle: {angle_deg:.1f}° → Target Pos: {target_position}")

        threading.Thread(
            target=self.smooth_position_transition,
            args=(self.current_position, target_position),
            daemon=True
        ).start()

    def angle_callback(self, msg):
        angle = msg.data
        self.move_steering(angle)

    def send_position_command(self, position):
        position_hex = '{:0>8X}'.format(position & 0xFFFFFFFF)
        DATA_Hh, DATA_Hl, DATA_Lh, DATA_Ll = (int(position_hex[i:i + 2], 16) for i in range(0, 8, 2))

        msg_position = can.Message(
            arbitration_id=self.control_id,
            data=[0x23, 0x02, 0x20, 0x01, DATA_Ll, DATA_Lh, DATA_Hl, DATA_Hh]
        )
        self.send_can_message(msg_position, f"Sent position command: {position}")

    def shutdown(self):
        self.monitoring = False
        msg_disable = can.Message(
            arbitration_id=self.control_id,
            data=[0x23, 0x0C, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00]
        )
        self.send_can_message(msg_disable, " Motor disabled")
        os.system('sudo ifconfig can0 down')
        self.get_logger().info("CAN interface shut down.")

def main(args=None):
    rclpy.init(args=args)
    controller = SteeringMotorController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()
        controller.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()

