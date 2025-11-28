#include "Lemlib_pid-logging.hpp"
#include <iostream>
#include "api.h"
#include "main.h"  // ensure pros::millis()/delay are declared
#include <cmath>


PIDLogger::PIDLogger(double kP, double kI, double kD)
    : kP(kP), kI(kI), kD(kD) {}

void PIDLogger::reset() {
    // clear internal state
    i_error   = 0.0;
    last_error = 0.0;
    last_time  = pros::millis();
    P = I = D = output = 0.0;
}

double PIDLogger::update(double target, double current, double rate, int time_ms) {
    double error = target - current;

    // int dt_ms = time_ms - last_time;
    // double dt = (dt_ms > 0) ? static_cast<double>(dt_ms) / 1000.0 : 1e-3;
    double dt = ((double)time_ms - (double)last_time);
    // dt /= 1000.0;
    dt = dt > 0 ? dt : 1e-3; // prevent divide by zero
    // Integral
    i_error += error * dt;

    // Derivative: if rate is supplied, it's -measured rate
    double d_error = (std::isnan(rate)) ? -rate : (error - last_error) / dt;

    // PID terms
    P = kP * error;
    I = kI * i_error;
    D = kD * d_error;
    output = P + I + D;

    // Save state
    last_error = error;
    last_time  = time_ms;

    return output;
}

double PIDLogger::getP() const { return P; }
double PIDLogger::getI() const { return I; }
double PIDLogger::getD() const { return D; }
double PIDLogger::getError() const { return last_error; }
double PIDLogger::getOutput() const { return output; }
