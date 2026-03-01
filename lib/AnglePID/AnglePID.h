#pragma once
#include <Arduino.h>

class AnglePID {
    private:
        double kp, ki, kd;
        double out_min, out_max;
        double prev_error;
        double control;
        double range;
        double integral;
        double integral_max;
        double integral_min;

    public:
        AnglePID(double kp, double ki, double kd, double out_min, double out_max, double range, double INTEGRAL_MAX,
              double INTEGRAL_MIN)
            : kp(kp), ki(ki), kd(kd), out_min(out_min), out_max(out_max), range(range), integral_max(INTEGRAL_MAX),
              integral_min(INTEGRAL_MIN), prev_error(0.0), control(0.0), integral(0.0) {}

        double update(long target_pos, long now_pos, double dt) {
            if (dt <= 0.0) return control;
            // 目標値 - 実測値でエラーを出す
            double error = (double)target_pos - (double)now_pos;

            // if (error > range) error -= range;
            // if (error < -range) error += range;

            // 角度差を、+-range/2に正規化する
            error = fmod(error + range / 2.0, range);
            if (error < 0) error += range;
            error -= range / 2.0;
            // 増分型 PID
            double derivative = (error - prev_error) / dt;
            prev_error        = error;

            integral += error * dt;
            // I項によるオーバーフローを防ぐため
            integral = constrain(integral, integral_min, integral_max);

            control = kp * error + ki * integral + kd * derivative;

            control = constrain(control, out_min, out_max);

            return control;
        }

        void reset() {
            prev_error = 0.;
            integral   = 0.;
            control    = 0.;
        }
};
