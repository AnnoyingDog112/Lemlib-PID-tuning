#pragma once

class PIDLogger {
public:
    PIDLogger(double kP, double kI, double kD);

    // Reset internal states (use before new run)
    void reset();

    // Update logger and compute PID output
    // - `target`: desired setpoint
    // - `current`: measured value
    // - `rate`: optional derivative (e.g. gyro); leave at 0 if unused
    // - `time_ms`: current timestamp (from pros::millis())
    double update(double target, double current, double rate, int time_ms);

    // Access terms for logging
    double getP() const;
    double getI() const;
    double getD() const;
    double getError() const;
    double getOutput() const;

private:
    double kP, kI, kD;

    double i_error = 0;
    double last_error = 0;
    int last_time = 0;

    double P = 0, I = 0, D = 0, output = 0;
};
