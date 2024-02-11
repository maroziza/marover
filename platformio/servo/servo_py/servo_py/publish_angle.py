import rclpy
from rclpy import node, qos
from sensor_msgs.msg import JointState

import board
import busio
from adafruit_ads1x15 import ads1115, ads1x15, analog_in

from servo_py.position_sensor import PositionSensor
from servo_py.kalman_filter import KalmanFilter
import numpy as np


class AnglePublisher(node.Node):
    def __init__(self):
        super().__init__(node_name = "angle_publisher")

        time_step = 0.02

        qos_profile = qos.qos_profile_best_available
        self._joint_state_publisher = self.create_publisher(JointState, "joint_state", qos_profile)
        self._timer = self.create_timer(time_step, self._timer_callback)

        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ads1115.ADS1115(i2c=i2c, mode=ads1x15.Mode.SINGLE)
        self._position_sensor = PositionSensor(analog_in.AnalogIn(ads, ads1115.P0), ads.bits)

        kalman = KalmanFilter(nx=2, ny=1)
        kalman.set_state_estimate(np.array([0., 0.]))
        kalman.set_state_covariance(np.diag([1., 0.1]) ** 2)
        kalman.set_process_noise_covariance(np.diag([0., 0.01]) ** 2)
        kalman.set_measurement_noise_covariance(np.diag([0.001]) ** 2)

        self._kalman = kalman
        self._A = np.array([
            [1., time_step],
            [0., 1.]
        ])
        self._B = np.zeros((2, 0))
        self._C = np.array([[1., 0.]])

    def _timer_callback(self):
        value = self._position_sensor.read()
        now = self.get_clock().now()

        y_expected = np.dot(self._C, self._kalman.get_state_estimate())
        self._kalman.update(np.array([value]) - y_expected, self._C)
        x = self._kalman.get_state_estimate()

        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ["joint_0"]
        joint_state.position = [x[0]]
        joint_state.velocity = [x[1]]

        self._joint_state_publisher.publish(joint_state)

        self._kalman.predict(self._A, self._B, np.array([]))


def main():
    rclpy.init()
    node = AnglePublisher()

    try:
        print("Start spinning")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()