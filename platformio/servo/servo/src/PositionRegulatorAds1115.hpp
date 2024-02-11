#pragma once

#include "KalmanFilter.hpp"

#include <device_kit/Ads1115.hpp>

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <gpiod.hpp>

#include <blaze/math/StaticMatrix.h>

#include <thread>
#include <atomic>


namespace servo
{
    class PositionRegulatorAds1115
    :   public rclcpp::Node
    {
    public:
        explicit PositionRegulatorAds1115(rclcpp::NodeOptions const& options);
        ~PositionRegulatorAds1115();

    private:
        void workThreadFunc();

        using Real = float;
        static std::size_t constexpr NX = 2;
        static std::size_t constexpr NU = 0;
        static std::size_t constexpr NY = 1;

        device_kit::Ads1115 positionSensor_;
        gpiod::line dataReadyLine_;
        std::chrono::milliseconds dataReadyWaitTimeout_;
        tmpc::KalmanFilter<Real> kalmanFilter_;
        blaze::StaticMatrix<Real, NX, NX> A_;
        blaze::StaticMatrix<Real, NX, NU> B_;
        blaze::StaticMatrix<Real, NY, NX> C_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher_;
        std::atomic_bool stop_ {false};
        std::thread workThread_;
    };
}