#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, JoyFeedbackArray, JoyFeedback
from ros2wifibot.msg import Status
import math


class IrDistanceNode(Node):
    def __init__(self):
        super().__init__('ir_distance_node')

        # Paramètres ajustables
        self.declare_parameter('left_channel', 1)    # adc1
        self.declare_parameter('right_channel', 3)    # adc3
        self.declare_parameter('voltage_scale', 1.0)  # correction si necessaire
        self.declare_parameter('coeff_a', 65.0)
        self.declare_parameter('coeff_b', -1.10)
        
        self.declare_parameter('obstacle_threshold', 0.3)
        self.declare_parameter('rumble_duration_sec', 0.3)

        self.left_channel  = self.get_parameter('left_channel').value
        self.right_channel = self.get_parameter('right_channel').value
        self.v_scale = self.get_parameter('voltage_scale').value
        self.coeff_a = self.get_parameter('coeff_a').value
        self.coeff_b = self.get_parameter('coeff_b').value
        
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.rumble_duration_sec = self.get_parameter('rumble_duration_sec').value
        self._rumble_timer = None

        self.sub = self.create_subscription(Status, '/status', self.status_cb, 10)
        self.pub_left  = self.create_publisher(Range, '/range/left', 10)
        self.pub_right = self.create_publisher(Range, '/range/right', 10)
        
        self.pub_feedback = self.create_publisher(JoyFeedbackArray, '/joy/set_feedback', 10)
        #self.pub_rumble_right = self.create_publisher(JoyFeedbackArray, '/joy/set_feedback', 10)

        self.get_logger().info(
            f'IR distance node: left=adc{self.left_channel}, right=adc{self.right_channel}')

    def byte_to_voltage(self, byte_val: int) -> float:
        return (byte_val / 255.0) * 3.3 * self.v_scale

    def voltage_to_distance_m(self, v: float) -> float:
        if v <= 0.01:
            return float('inf')
        d_cm = self.coeff_a * math.pow(v, self.coeff_b)
        return d_cm / 100.0  # → mètres

    def make_range_msg(self, frame_id: str, adc_val: int) -> Range:
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.09   # ~5° pour GP2Y0A02YK
        msg.min_range = 0.20       # 20 cm — sous ce seuil, lecture non fiable
        msg.max_range = 1.50       # 150 cm

        dist = self.voltage_to_distance_m(self.byte_to_voltage(adc_val))
        # clamp dans la plage annoncée du capteur
        msg.range = max(msg.min_range, min(msg.max_range, dist))
        return msg

    def status_cb(self, msg: Status):
        adc_map = {1: msg.adc1, 2: msg.adc2, 3: msg.adc3, 4: msg.adc4}

        left_val  = adc_map[self.left_channel]
        right_val = adc_map[self.right_channel]

        left_range_msg  = self.make_range_msg('ir_left_link', left_val)
        right_range_msg = self.make_range_msg('ir_right_link', right_val)
        self.pub_left.publish(left_range_msg)
        self.pub_right.publish(right_range_msg)
        
        if (left_range_msg.range < self.obstacle_threshold or right_range_msg.range < self.obstacle_threshold):
            self.trigger_short_rumble()
        
    def trigger_short_rumble(self):
        fb = JoyFeedbackArray()
        fb.array = [JoyFeedback(type=1, id=0, intensity=1.0)]
        self.pub_feedback.publish(fb)

        if self._rumble_timer is not None:
            self._rumble_timer.cancel()

        self._rumble_timer = self.create_timer(
            self.rumble_duration_sec, self._rumble_timeout_cb)

    def _rumble_timeout_cb(self):
        fb = JoyFeedbackArray()
        fb.array = [JoyFeedback(type=1, id=0, intensity=0.0)]
        self.pub_feedback.publish(fb)

        self._rumble_timer.cancel()
        self._rumble_timer = None


def main():
    rclpy.init()
    node = IrDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
