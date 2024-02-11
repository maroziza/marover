#include "PositionRegulatorAds1115.hpp"


namespace servo
{
    PositionRegulatorAds1115::PositionRegulatorAds1115(rclcpp::NodeOptions const& options)
    :   rclcpp::Node {"position_regulator", options}
    ,   positionSensor_ {
            bus_io::I2cSlave(
                declare_parameter("position_sensor_bus", "/dev/i2c-1"),
                declare_parameter("position_sensor_address", 0x48)
            )
        }
    ,   dataReadyLine_ {
            gpiod::chip {declare_parameter("data_ready_gpio_chip", "/dev/gpiochip0")}
                .get_line(declare_parameter("data_ready_gpio_line", 17))
        }
    ,   dataReadyWaitTimeout_ {declare_parameter("data_ready_wait_timeout_ms", 100)}
    ,   kalmanFilter_ {NX, NY}
    ,   jointStatePublisher_ {create_publisher<sensor_msgs::msg::JointState>("joint_state", rclcpp::SensorDataQoS {})}
    {
        // Configure ADC
        device_kit::Ads1115Settings adc_settings;
        adc_settings.samplingRate = declare_parameter("sampling_rate", 860);
        adc_settings.channel = declare_parameter("adc_channel", 0);
        positionSensor_.configure(adc_settings);

        // Request events for falling edge
        dataReadyLine_.request({
            .consumer = {},
            .request_type = gpiod::line_request::EVENT_FALLING_EDGE,
            .flags = {}
        });

        // Setup Kalman filter
        blaze::StaticVector<Real, NX> x0 {0.f, 0.f};
        blaze::StaticMatrix<Real, NX, NX> P(0.f);
        P(0, 0) = std::pow(1.f, 2);
        P(1, 1) = std::pow(0.1f, 2);
        blaze::StaticMatrix<Real, NX, NX> Q(0.f);
        Q(0, 0) = std::pow(declare_parameter("kalman_process_noise_sigma_q", 0.f), 2);
        Q(1, 1) = std::pow(declare_parameter("kalman_process_noise_sigma_v", 0.001f), 2);
        blaze::StaticMatrix<Real, NY, NY> R(0.f);
        R(0, 0) = std::pow(declare_parameter("kalman_measurement_noise_sigma_q", 0.001f), 2);
        kalmanFilter_.stateEstimate(x0);
        kalmanFilter_.stateCovariance(P);
        kalmanFilter_.processNoiseCovariance(Q);
        kalmanFilter_.measurementNoiseCovariance(R);

        // Setup system matrices
        Real const time_step = 1.f / adc_settings.samplingRate;
        A_ = {
            {1.f, time_step},
            {0.f, 1.f}
        };
        C_ = {
            {1.f, 0.f}
        };

        workThread_ = std::thread {std::bind(&PositionRegulatorAds1115::workThreadFunc, this)};
    }


    PositionRegulatorAds1115::~PositionRegulatorAds1115()
    {
        stop_ = true;
        workThread_.join();
    }


    void PositionRegulatorAds1115::workThreadFunc()
    {
        while (!stop_)
        {
            if (dataReadyLine_.event_wait(dataReadyWaitTimeout_))
            {
                auto const stamp = now();
                auto const event = dataReadyLine_.event_read();
                if (event.event_type == gpiod::line_event::FALLING_EDGE)
                {
                    // Read position sensor
                    blaze::StaticVector<Real, NY> const y {readNormalizedValue(positionSensor_)};

                    // Update Kalman
                    blaze::StaticVector<Real, NY> const y_expected = C_ * kalmanFilter_.stateEstimate();
                    kalmanFilter_.update(y - y_expected, C_);
                    blaze::StaticVector<Real, NX> const x = kalmanFilter_.stateEstimate();

                    // Publish state
                    sensor_msgs::msg::JointState joint_state;
                    joint_state.header.stamp = stamp;
                    joint_state.name = {"joint_0"};
                    joint_state.position = {x[0]};
                    joint_state.velocity = {x[1]};
                    jointStatePublisher_->publish(joint_state);

                    // Kalman predict
                    blaze::StaticVector<Real, NU> u {};
                    kalmanFilter_.predict(A_, B_, u);
                }
            }
        }
    }
}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(servo::PositionRegulatorAds1115)
