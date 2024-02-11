import numpy as np


class PidController:
    """PID controller
    """
    def __init__(self, k_p: float, k_i: float, k_d: float, lbu: float, ubu: float):
        """
        @brief Create a PID controller with specified proportional, integral,
        and derivative terms, and control input bounds.

        @param k_p proportional term
        @param k_i integral term
        @param k_d derivative term
        @param lbu lower bound of control input
        @param ubu upper bound of control input

        @throw @a ValueError if not lbu < ubu
        """
        if not lbu < ubu:
            raise ValueError("The control input lower bound must be less than the upper bound");

        self._kP = k_p
        self._kI = k_i
        self._kD = k_d
        self._lbu = lbu
        self._ubu = ubu
        self._last_error = 0.
        self._last_input_unsaturated_ = 0.
        self._error_integral = 0.

    def feedback(self, h: float, e: float, e_dot: float) -> float:
        """
        @brief Calculate control feedback

        @param h time passed since previous feedback calculation, 0 if this is the first feedback calculation
        @param e error variable, e = process_variable - setpoint
        @param e_dot time-derivative of the error variable

        @return saturated control input
        """

        # Update error integral only if the input is not saturated, to prevent wind-up.
        if self._last_input_unsaturated_ > self._lbu and self._last_input_unsaturated_ < self._ubu:
            self._error_integral += h * (e + self._last_error) / 2.

        # Update unsaturated input
        self._last_input_unsaturated_ = -self._kP * e - self._kI * self._error_integral - self._kD * e_dot

        # Return saturated input
        return np.clip(self._last_input_unsaturated_, self._lbu, self._ubu)

    def controlBounds(self, lbu: float, ubu: float):
        """
        @brief Set control input bounds

        @param lbu lower control input bound
        @param ubu upper control input bound

        @throw @a ValueError if not lbu < ubu
        """
        if not lbu < ubu:
            raise ValueError("The control input lower bound must be less than the upper bound")

        self._lbu = lbu
        self._ubu = ubu
