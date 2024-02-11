#pragma once

#include "KalmanFilter.hpp"
#include "PidController.hpp"
#include "Drive.hpp"

#include <device_kit/As5600.hpp>
#include <device_kit/Ads1115.hpp>

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <servo_msgs/msg/magnetic_sensor_status.hpp>
#include <servo_msgs/msg/as5600_angle.hpp>

#include <blaze/math/StaticMatrix.h>


namespace servo
{
    class PositionRegulatorAs5600
    :   public rclcpp::Node
    {
    public:
        explicit PositionRegulatorAs5600(rclcpp::NodeOptions const& options);
        ~PositionRegulatorAs5600();

    private:
        void timerCallback();
        void inputTimerCallback();
        device_kit::pca9685::Pca9685 createPwmModulator();

        using Real = float;
        static std::size_t constexpr NX = 3;
        static std::size_t constexpr NU = 0;
        static std::size_t constexpr NY = 1;

        device_kit::Ads1115 inputSensor_;
        Real velocitySetpoint_ {};
        device_kit::As5600 positionSensor_;
        Real controlInterval_;
        tmpc::KalmanFilter<Real> kalmanFilter_;
        blaze::StaticMatrix<Real, NX, NX> A_;
        blaze::StaticMatrix<Real, NX, NU> B_;
        blaze::StaticMatrix<Real, NY, NX> C_;
        PidController<Real> velocityController_;
        bool enableControl_ {false};
        Drive drive_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher_;
        rclcpp::Publisher<servo_msgs::msg::MagneticSensorStatus>::SharedPtr sensorStatusPublisher_;
        rclcpp::Publisher<servo_msgs::msg::As5600Angle>::SharedPtr anglePublisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr inputTimer_;
    };
}