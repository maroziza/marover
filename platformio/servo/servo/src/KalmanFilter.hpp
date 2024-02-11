#pragma once

#include <blaze/math/DynamicMatrix.h>
#include <blaze/math/SymmetricMatrix.h>
#include <blaze/math/DynamicVector.h>
#include "Trsm.hpp"

#include <cstddef>


namespace tmpc
{
    /// @brief Kalman filter implementation.
    ///
    template <typename Real>
    class KalmanFilter
    {
    public:
        using Matrix = blaze::DynamicMatrix<Real>;
        using SymmetricMatrix = blaze::SymmetricMatrix<Matrix>;


        KalmanFilter(std::size_t nx, std::size_t ny)
        :   nx_(nx)
        ,   ny_(ny)
        ,   stateEstimate_(nx, Real {})
        ,   stateCovariance_(nx)
        ,   processNoiseCovariance_(nx)
        ,   measurementNoiseCovariance_(ny)
        ,   S_(ny)
        ,   S_chol_(ny)
        ,   trans_K_(ny, nx)
        {
        }


        /// @brief Get state estimate
        auto const& stateEstimate() const
        {
            return stateEstimate_;
        }


        /// @brief Set state estimate
        template <typename VT>
        void stateEstimate(blaze::Vector<VT, blaze::columnVector> const& val)
        {
            fix(stateEstimate_) = val;
        }


        /// @brief Get state covariance
        SymmetricMatrix const& stateCovariance() const
        {
            return stateCovariance_;
        }


        /// @brief Set state covariance
        template <typename MT, bool SO>
        void stateCovariance(blaze::Matrix<MT, SO> const& val)
        {
            fix(stateCovariance_) = val;
        }


        /// @brief Get process noise covariance
        SymmetricMatrix const& processNoiseCovariance() const
        {
            return processNoiseCovariance_;
        }


        /// @brief Set process noise covariance
        template <typename MT, bool SO>
        void processNoiseCovariance(blaze::Matrix<MT, SO> const& val)
        {
            fix(processNoiseCovariance_) = val;
        }


        /// @brief Get measurement noise covariance
        SymmetricMatrix const& measurementNoiseCovariance() const
        {
            return measurementNoiseCovariance_;
        }


        /// @brief Set measurement noise covariance
        template <typename MT, bool SO>
        void measurementNoiseCovariance(blaze::Matrix<MT, SO> const& val)
        {
            fix(measurementNoiseCovariance_) = val;
        }


        /// @brief Update state estimate based on next state value and sensitivities.
        ///
        /// @param x_next next state value
        /// @param A sensitivity matrix, A = d(x_next)/dx
        template <typename VT, typename MT, bool SO>
        void predict(blaze::Vector<VT, blaze::columnVector> const& x_next, blaze::Matrix<MT, SO> const& A)
        {
            stateEstimate_ = x_next;
            stateCovariance_ = *A * stateCovariance_ * trans(*A) + processNoiseCovariance_;
        }


        /// @brief Update state estimate based on a linear model and control input.
        ///
        /// @param A linear model matrix A
        /// @param B linear model matrix B
        /// @param u control input
        template <typename MT1, bool SO1, typename MT2, bool SO2, typename VT>
        void predict(blaze::Matrix<MT1, SO1> const& A, blaze::Matrix<MT2, SO2> const& B, blaze::Vector<VT, blaze::columnVector> const& u)
        {
            stateEstimate_ = *A * stateEstimate_ + *B * *u;
            stateCovariance_ = *A * stateCovariance_ * trans(*A) + processNoiseCovariance_;
        }


        /// @brief Update state estimate based on measurement residual and sensitivities.
        ///
        /// @param y measurement residual, the difference between measured and predicted system output.
        /// @param C output sensitivity matrix, C = d(y_pred)/dx, where y_pred is the predicted system output for state x.
        template <typename VT, typename MT, bool SO>
        void update(blaze::Vector<VT, blaze::columnVector> const& y, blaze::Matrix<MT, SO> const& C)
        {
            // TODO: performance of this code can definitely be improved by avoiding temporary subexpressions
            // and using proper LAPACK routines.
            S_ = measurementNoiseCovariance_ + *C * stateCovariance_ * trans(*C);
            llh(S_, S_chol_);

            // Here we want to compute:
            // K_ = stateCovariance_ * trans(*C) * inv(S_);
            //
            // We use the Cholesky decomposition of S to multiply with inv(S).
            // We rewtite the equation above as
            // K_ = stateCovariance_ * trans(*C) * inv(S_);
            // K_ * S_ = stateCovariance_ * trans(*C);
            // K_ * S_chol_ * trans(S_chol_) = stateCovariance_ * trans(*C);
            // S_chol * trans(S_chol) * trans(K_) = *C * trans(stateCovariance_)
            //
            // The right side we compute explicitly.
            // Then we solve for K by calling trsm() two times.
            fix(trans_K_) = *C * trans(stateCovariance_);
            blast::trsm<blast::UpLo::Lower, false>(S_chol_, trans_K_, trans_K_);
            blast::trsm<blast::UpLo::Upper, false>(trans(S_chol_), trans_K_, trans_K_);

            stateEstimate_ += trans(trans_K_) * *y;

            // Use the Cholesky decomposition of S to enforce symmetry by putting the expression in the form A*A^T.
            stateCovariance_ -= trans(trans_K_) * S_chol_ * trans(S_chol_) * trans_K_;
        }


        /// @brief Number of states
        auto const nx() const
        {
            return nx_;
        }


        /// @brief Number of outputs
        auto const ny() const
        {
            return ny_;
        }


        /// @brief Kalman gain on the last update step
        auto gain() const
        {
            return trans(trans_K_);
        }


        /// @brief Measurement prediction covariance on the last update step
        SymmetricMatrix const& measurementPredictionCovariance() const
        {
            return S_;
        }


    private:
        std::size_t const nx_;
        std::size_t const ny_;

        blaze::DynamicVector<Real, blaze::columnVector> stateEstimate_;
        SymmetricMatrix stateCovariance_;

        SymmetricMatrix processNoiseCovariance_;
        SymmetricMatrix measurementNoiseCovariance_;

        SymmetricMatrix S_;

        // Cholesky decomposition of S_;
        blaze::LowerMatrix<Matrix> S_chol_;

        // Transpose of Kalman gain
        Matrix trans_K_;
    };
}