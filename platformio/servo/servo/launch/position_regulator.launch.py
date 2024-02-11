from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    position_regulator_node = ComposableNode(
        package = "servo",
        plugin = 'servo::PositionRegulatorAs5600',
        parameters = [{
            "input_adc_channel": 0,
            "control_rate": 768.,
            "input_rate": 30.,
            "kalman_process_noise_sigma_q": 0.00,
            "kalman_process_noise_sigma_v": 0.00,
            "kalman_process_noise_sigma_a": 0.5,
            "kalman_measurement_noise_sigma_q": 0.001,
            "velocity.k_p": 0.2,
            "velocity.k_i": 10.,
            "velocity.k_d": 0.005,
            "pwm_modulator_frequency": 1500,
            "enable_control": True
        }],
        extra_arguments = [{
            "use_intra_process_comms": True
        }]
    )

    container = ComposableNodeContainer(
        name = 'ComponentManager',
        namespace = '',
        package = 'rclcpp_components',
        executable = 'component_container',
        composable_node_descriptions = [
            position_regulator_node
        ]
    )

    return LaunchDescription([
        container
    ])
