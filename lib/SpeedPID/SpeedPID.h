#pragma once
#include <Arduino.h>

class SpeedPID {
    private:
        double kp, ki, kd;
        double out_min, out_max;
        double prev_error, prev_delta_error;
        double output;

    public:
        SpeedPID(double kp, double ki, double kd, double out_min, double out_max)
            : kp(kp), ki(ki), kd(kd), out_min(out_min), out_max(out_max), prev_error(0.0), prev_delta_error(0.0), output(0.0) {}

        double update(double target, double measurement, double dt) {
            if (dt <= 0.0) return output;
            // 目標値 - 実測値でエラーを出す
            const double error = target - measurement;

            // 増分型 PID
            const double delta_error = error - prev_error;
            const double deriv       = delta_error - prev_delta_error;
            const double du          = kp * delta_error + ki * error * dt + kd * deriv / dt;
            output += du;
            output = constrain(output, out_min, out_max);

            prev_error       = error;
            prev_delta_error = delta_error;

            return output;
        }

        void reset() {
            prev_error       = 0.;
            prev_delta_error = 0.;
            output           = 0.;
        }
};
