#include "PositionRegulatorAs5600.hpp"

#include <angles/angles.h>


namespace servo
{
    PositionRegulatorAs5600::PositionRegulatorAs5600(rclcpp::NodeOptions const& options)
    :   rclcpp::Node {"position_regulator", options}
    ,   inputSensor_ {
            bus_io::I2cSlave(
                declare_parameter("input_sensor_bus", "/dev/i2c-1"),
                declare_parameter("input_sensor_address", 0x48)
            )
        }
    ,   positionSensor_ {
            bus_io::I2cSlave(
                declare_parameter("position_sensor_bus", "/dev/i2c-1"),
                declare_parameter("position_sensor_address", 0x36)
            )
        }
    ,   controlInterval_(1. / declare_parameter("control_rate", 200.))
    ,   kalmanFilter_ {NX, NY}
    ,   velocityController_(
            declare_parameter("velocity.k_p", 1.),
            declare_parameter("velocity.k_i", 0.),
            declare_parameter("velocity.k_d", 0.),
            -1.f,
            1.f
        )
    ,   enableControl_ {declare_parameter("enable_control", true)}
    ,   drive_ {createPwmModulator()}
    ,   jointStatePublisher_ {
            create_publisher<sensor_msgs::msg::JointState>("joint_state", rclcpp::SensorDataQoS {})
        }
    ,   sensorStatusPublisher_ {
            create_publisher<servo_msgs::msg::MagneticSensorStatus>("sensor_status", rclcpp::SensorDataQoS {})
        }
    ,   anglePublisher_ {
            create_publisher<servo_msgs::msg::As5600Angle>("angle", rclcpp::SensorDataQoS {})
        }
    ,   timer_ {
            create_timer(
                std::chrono::duration<Real> {controlInterval_},
                std::bind(&PositionRegulatorAs5600::timerCallback, this)
            )
        }
    ,   inputTimer_ {
            create_timer(
                std::chrono::duration<double> {1. / declare_parameter("input_rate", 20.)},
                std::bind(&PositionRegulatorAs5600::inputTimerCallback, this)
            )
        }
    {
        // Configure ADC
        device_kit::Ads1115Settings adc_settings;
        adc_settings.samplingRate = declare_parameter("input_sampling_rate", 860);
        adc_settings.channel = declare_parameter("input_adc_channel", 0);
        inputSensor_.configure(adc_settings);

        // Configure angle sensor
        device_kit::As5600Settings angle_sensor_settings;
        positionSensor_.configure(angle_sensor_settings);

        // Setup Kalman filter
        blaze::StaticVector<Real, NX> x0 {0.f, 0.f};
        blaze::StaticMatrix<Real, NX, NX> P(0.f);
        P(0, 0) = std::pow(1.f, 2);
        P(1, 1) = std::pow(0.1f, 2);
        P(2, 2) = std::pow(0.1f, 2);
        blaze::StaticMatrix<Real, NX, NX> Q(0.f);
        Q(0, 0) = std::pow(declare_parameter("kalman_process_noise_sigma_q", 0.f), 2);
        Q(1, 1) = std::pow(declare_parameter("kalman_process_noise_sigma_v", 0.001f), 2);
        Q(2, 2) = std::pow(declare_parameter("kalman_process_noise_sigma_a", 0.1f), 2);
        blaze::StaticMatrix<Real, NY, NY> R(0.f);
        R(0, 0) = std::pow(declare_parameter("kalman_measurement_noise_sigma_q", 0.001f), 2);
        kalmanFilter_.stateEstimate(x0);
        kalmanFilter_.stateCovariance(P);
        kalmanFilter_.processNoiseCovariance(Q);
        kalmanFilter_.measurementNoiseCovariance(R);

        // Setup system matrices
        A_ = {
            {1.f, controlInterval_, 0.5f * controlInterval_ * controlInterval_},
            {0.f, 1.f, controlInterval_},
            {0.f, 0.f, 1.f}
        };
        C_ = {
            {1.f, 0.f, 0.f}
        };
    }


    PositionRegulatorAs5600::~PositionRegulatorAs5600()
    {
        drive_.throttle(0.f);
    }


    void PositionRegulatorAs5600::timerCallback()
    {
        // Read position sensor
        auto const before_read_position_sensor = now();
        auto const angle_reading = positionSensor_.readAngle();
        auto const stamp = now();
        blaze::StaticVector<Real, NX> const x0 = kalmanFilter_.stateEstimate();
        blaze::StaticVector<Real, NY> const y {
            x0[0] + static_cast<Real>(angles::shortest_angular_distance(x0[0], angle_reading.angleRadians))
        };
        RCLCPP_DEBUG_STREAM(get_logger(), "read_position_sensor took " << (stamp - before_read_position_sensor).seconds());

        blaze::StaticVector<Real, NY> const y_expected = C_ * kalmanFilter_.stateEstimate();
        blaze::StaticVector<Real, NX> x(std::numeric_limits<Real>::quiet_NaN());
        Real throttle = std::numeric_limits<Real>::quiet_NaN();

        try
        {
            // Update Kalman
            kalmanFilter_.update(y - y_expected, C_);
            x = kalmanFilter_.stateEstimate();

            // Calculate control input
            auto const velocity_error = x[1] - velocitySetpoint_;
            Real const velocity_error_rate = x[2];  // not including setpoint rate!
            throttle = enableControl_ ? velocityController_.feedback(controlInterval_, velocity_error, velocity_error_rate) : velocitySetpoint_;

            // Set throttle
            auto const before_set_throttle = now();
            drive_.throttle(throttle);
            auto const after_set_throttle = now();
            RCLCPP_DEBUG_STREAM(get_logger(), "set_throttle took " << (after_set_throttle - before_set_throttle).seconds());

            // Kalman predict
            blaze::StaticVector<Real, NU> u {};
            kalmanFilter_.predict(A_, B_, u);
        }
        catch (std::invalid_argument const& e)
        {
            RCLCPP_ERROR_STREAM(get_logger(), e.what());
        }

        // Publish state
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = stamp;
        joint_state.name = {"joint_0", "raw", "setpoint"};
        joint_state.position = {x[0], angle_reading.angleRadians, 0.};
        joint_state.velocity = {x[1], 0., velocitySetpoint_};
        joint_state.effort = {throttle, x[2], 0.};
        jointStatePublisher_->publish(joint_state);

        auto const sensor_status = positionSensor_.readStatus();
        servo_msgs::msg::MagneticSensorStatus sensor_status_msg;
        sensor_status_msg.header.stamp = stamp;
        sensor_status_msg.magnet_detected = sensor_status.magnetDetected;
        sensor_status_msg.magnet_too_strong = sensor_status.magnetTooStrong;
        sensor_status_msg.magnet_too_weak = sensor_status.magnetTooWeak;
        sensor_status_msg.automatic_gain_control = sensor_status.automaticGainControl;
        sensor_status_msg.magnitude = sensor_status.magnitude;
        sensorStatusPublisher_->publish(sensor_status_msg);

        servo_msgs::msg::As5600Angle angle_msg;
        angle_msg.header.stamp = stamp;
        angle_msg.raw_angle = angle_reading.rawAngle;
        angle_msg.angle = angle_reading.angle;
        angle_msg.angle_radians = angle_reading.angleRadians;
        anglePublisher_->publish(angle_msg);
    }


    device_kit::pca9685::Pca9685 PositionRegulatorAs5600::createPwmModulator()
    {
        device_kit::pca9685::Pca9685 pwm {
            bus_io::I2cSlave(
                declare_parameter("pwm_modulator_bus", "/dev/i2c-1"),
                declare_parameter("pwm_modulator_address", 0x40)
            )
        };

        // Configure PWM
        pwm.frequency(declare_parameter("pwm_modulator_frequency", 1500));
        RCLCPP_INFO_STREAM(get_logger(), "PWM frequency set to " << pwm.frequency() << "Hz");

        return pwm;
    }


    void PositionRegulatorAs5600::inputTimerCallback()
    {
        // Update velocity setpoint
        auto const before_get_input = now();
        velocitySetpoint_ = -2.f * (readNormalizedValue(inputSensor_) - 0.5f);
        auto const after_get_input = now();
        RCLCPP_DEBUG_STREAM(get_logger(), "get_input took " << (after_get_input - before_get_input).seconds());
    }
}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(servo::PositionRegulatorAs5600)
