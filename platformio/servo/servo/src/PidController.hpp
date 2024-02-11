#pragma once

#include <limits>
#include <algorithm>
#include <stdexcept>


namespace servo
{
    /// @brief PID controller
    ///
    /// @tparam Real type for real numbers
    ///
    template <typename Real>
    class PidController
    {
    public:
        /// @brief Create a PID controller with specified proportional, integral,
        /// and derivative terms, and control input bounds.
        ///
        /// @param k_p proportional term
        /// @param k_i integral term
        /// @param k_d derivative term
        /// @param lbu lower bound of control input
        /// @param ubu upper bound of control input
        ///
        /// @throw @a std::invalid_argument if !(lbu < ubu)
        ///
        explicit PidController(Real k_p, Real k_i, Real k_d, Real lbu, Real ubu)
        :   kP_ {k_p}
        ,   kI_ {k_i}
        ,   kD_ {k_d}
        ,   lbu_ {lbu}
        ,   ubu_ {ubu}
        {
            if (!(lbu < ubu))
                throw std::invalid_argument {"The control input lower bound must be less than the upper bound"};
        }

        /// @brief Calculate control feedback
        ///
        /// @param h time passed since previous feedback calculation, 0 if this is the first feedback calculation
        /// @param e error variable, e = process_variable - setpoint
        /// @param e_dot time-derivative of the error variable
        ///
        /// @return saturated control input
        ///
        Real feedback(Real h, Real e, Real e_dot = Real {}) noexcept
        {
            // Update error integrad only if the input is not saturated, to prevent wind-up.
            if (lastInputUnsaturated_ > lbu_ && lastInputUnsaturated_ < ubu_)
                errorIntegral_ += h * (e + lastError_) / 2.;

            // Update unsaturated input
            lastInputUnsaturated_ = -kP_ * e - kI_ * errorIntegral_ - kD_ * e_dot;

            // Return saturated input
            return std::clamp(lastInputUnsaturated_, lbu_, ubu_);
        }

        /// @brief Set control input bounds
        ///
        /// @param lbu lower control input bound
        /// @param ubu upper control input bound
        ///
        /// @throw @a std::invalid_argument if !(lbu < ubu)
        ///
        void controlBounds(Real lbu, Real ubu)
        {
            if (!(lbu < ubu))
                throw std::invalid_argument {"The control input lower bound must be less than the upper bound"};

            lbu_ = lbu;
            ubu_ = ubu;
        }

    private:
        Real kP_;
        Real kI_;
        Real kD_;
        Real lbu_ {-std::numeric_limits<Real>::infinity()};
        Real ubu_ {std::numeric_limits<Real>::infinity()};
        Real lastError_ {};
        Real lastInputUnsaturated_ {};
        Real errorIntegral_ {};
    };
}