import rclpy
from rclpy import node, qos
from sensor_msgs.msg import JointState

import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_ads1x15 import ads1115, ads1x15, analog_in

from servo_py.kalman_filter import KalmanFilter
from servo_py.position_sensor import PositionSensor
from servo_py.pid_controller import PidController
from servo_py.drive import Drive
from servo_msgs.msg import DutyCycle

import numpy as np


class PositionRegulator(node.Node):
    def __init__(self):
        super().__init__(node_name = "position_regulator")

        time_step = 0.02
        self._time_step = time_step

        qos_profile = qos.qos_profile_best_available
        self._joint_state_publisher = self.create_publisher(JointState, "joint_state", qos_profile)
        self._duty_cycle_publisher = self.create_publisher(DutyCycle, "duty_cycle", qos_profile)
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

        self._controller = PidController(5., 5., 0.2, lbu=-1., ubu=1.)
        self._target_position = 0.2

        pwm_driver = PCA9685(i2c)
        pwm_driver.frequency = 1000
        self._drive = Drive(pwm_driver)

    def stop(self):
        self._drive.set_throttle(0.)

    def _timer_callback(self):
        now = self.get_clock().now()

        # Update state estimate
        value = self._position_sensor.read()
        y_expected = np.dot(self._C, self._kalman.get_state_estimate())
        self._kalman.update(np.array([value]) - y_expected, self._C)
        x = self._kalman.get_state_estimate()

        # Calculate control input
        delta = x[0] - self._target_position
        if delta > 0.01:
            delta -= 0.01
        elif delta < -0.01:
            delta += 0.01
        else:
            delta = 0
        print(delta)
        u = self._controller.feedback(self._time_step, delta, x[1])
        # u += 0.1 * np.sign(u)

        # Send control input
        self._drive.set_throttle(-u)

        # Kalman predict
        self._kalman.predict(self._A, self._B, np.array([]))

        # Publish state
        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ["joint_0"]
        joint_state.position = [x[0]]
        joint_state.velocity = [x[1]]
        joint_state.effort = [u]
        self._joint_state_publisher.publish(joint_state)

        # # Publish duty cycle
        # duty_cycle = DutyCycle()
        # duty_cycle.header.stamp = now.to_msg()
        # duty_cycle.duty_cycle = u
        # self._duty_cycle_publisher.publish(duty_cycle)


def main():
    rclpy.init()
    node = PositionRegulator()

    try:
        print("Start spinning")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
