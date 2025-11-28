#include <iostream>
#include "api.h"  // ensure pros::millis()/delay are declared
#include "Lemlib_pid-logging.hpp"
#include "lemlib/api.hpp"
#include "pid_tuning.hpp"
#include <cmath>

int error_counter=0;
int out_of_range_counter=0;
float error_rate_limit = 600.0; // deg/s

void validateInput(double rate, double time, double target, double current,
    double P, double I, double D, double output
    ) {
    if (std::isnan(rate)) {
        throw std::invalid_argument("Gyro Rate is NaN");
    } 
    if (std::isinf(rate)) {
        throw std::invalid_argument("Gyro Rate is Infinity");
    }
    if (std::abs(rate) > error_rate_limit) {
        throw std::out_of_range("Gyro Rate is out of range");
    }
    if (time < 0) {
        throw std::invalid_argument("Time is negative");
    }
    if (std::isnan(time)) {
        throw std::invalid_argument("Target is NaN");
    }
    if (std::isinf(time)) {
        throw std::invalid_argument("Target is Infinity");
    }
    if (std::isnan(target)) {
        throw std::invalid_argument("Target is NaN");
    }
    if (std::isinf(target)) {
        throw std::invalid_argument("Target is Infinity");
    }
    if (std::isnan(current)) {
        throw std::invalid_argument("Current is NaN");
    }
    if (std::isinf(current)) {
        throw std::invalid_argument("Current is Infinity");
    }
    if (std::isnan(P)) {
        throw std::invalid_argument("P is NaN");
    }
    if (std::isinf(P)) {
        throw std::invalid_argument("P is Infinity");
    }
    if (std::isnan(I)) {
        throw std::invalid_argument("I is NaN");
    }
    if (std::isinf(I)) {
        throw std::invalid_argument("I is Infinity");
    }
    if (std::isnan(D)) {
        throw std::invalid_argument("D is NaN");
    }
    if (std::isinf(D)) {
        throw std::invalid_argument("D is Infinity");
    }
    if (std::isnan(output)) {
        throw std::invalid_argument("Output is NaN");
    }
    if (std::isinf(output)) {
        throw std::invalid_argument("Output is Infinity");
    }
}

static void angular_pid_logging_task(void* param) {
    auto* args = static_cast<AngularPidLogArgs*>(param);

    PIDLogger logger(args->controller.kP, args->controller.kI, args->controller.kD);
    logger.reset();

    std::cout << "time_ms, error, P, I, D, output" << std::endl;
    
    double error;

    while (!args->stop.load(std::memory_order_acquire)) {
        int now = pros::millis();
        double current = args->chassis->getPose().theta;
        // PROS IMU returns Z gyro rate in deg/s
        double rate = (args->imu->get_gyro_rate().z)/1000;

        double output = logger.update(args->target, current, rate, now);
        
        error = logger.getError();
        double P = logger.getP();
        double I = logger.getI();
        double D = logger.getD();

        try {
            validateInput(rate, now, args->target, current, P, I, D, output);
        } catch (const std::out_of_range& e) {
            // Out of range errors are less critical; just log and continue
            out_of_range_counter++;
            continue;

        } catch (const std::exception& e) {
            std::cerr << "Input validation error: " << e.what() << std::endl;
            args->stop.store(true, std::memory_order_release);
            if (error_counter < 5) { 
                error_counter++;
            }
            continue;
        } catch (...) {
            std::cerr << "Unknown input validation error" << std::endl;
            args->stop.store(true, std::memory_order_release);
            break;
        }
        
        std::cout << now << ", "
                  << error << ", "
                  << P << ", "
                  << I << ", "
                  << D << ", "
                  << output << "\n";

        int elapsed = pros::millis() - now;
        if (elapsed < args->loop_delay_ms) pros::delay(args->loop_delay_ms - elapsed);
    }
    std::cout << std::flush;
    std::cout << "Steady State Error: " << error << std::endl;
    std::cout << "Out of range errors: " << out_of_range_counter <<
                ", Total errors: " << error_counter << std::endl;
    
    delete args;  // Free heap-allocated struct (task owns it)
    return;
}

static void lateral_pid_logging_task(void* param) {
    auto* args = static_cast<AngularPidLogArgs*>(param);

    PIDLogger logger(args->controller.kP, args->controller.kI, args->controller.kD);
    logger.reset();

    std::cout << "time_ms, error, P, I, D, output" << std::endl;

    double error;
    
    while (!args->stop.load(std::memory_order_acquire)) {
        int now = pros::millis();
        double current = args->chassis->getPose().y;  // lateral position
        double rate = args->imu->get_accel().y; // Use x for now. Change later

        double P = logger.getP();
        double I = logger.getI();
        double D = logger.getD();
        
        double output = logger.update(args->target, current, rate, now);

        try {
            validateInput(rate, now, args->target, current, P, I, D, output);
        } catch (const std::out_of_range& e) {
            // Out of range errors are less critical; just log and continue
            out_of_range_counter++;
            continue;

        } catch (const std::exception& e) {
            std::cerr << "Input validation error: " << e.what() << std::endl;
            args->stop.store(true, std::memory_order_release);
            if (error_counter < 5) { 
                error_counter++;
            }
            continue;
        } catch (...) {
            std::cerr << "Unknown input validation error" << std::endl;
            args->stop.store(true, std::memory_order_release);
            break;
        }

        std::cout << now << ", "
                  << logger.getError() << ", "
                  << logger.getP() << ", "
                  << logger.getI() << ", "
                  << logger.getD() << ", "
                  << output << "\n";

        int elapsed = pros::millis() - now;
        if (elapsed < args->loop_delay_ms) pros::delay(args->loop_delay_ms - elapsed);
    }
    std::cout << std::flush;
    std::cout << "Steady State Error: " << error << std::endl;
    std::cout << "Out of range errors: " << out_of_range_counter <<
            ", Total errors: " << error_counter << std::endl;

    delete args;  // Free heap-allocated struct (task owns it)
    return;
}

void start_angular_pid_logging_task(
    lemlib::Chassis* chassis,
    pros::Imu* imu,
    const lemlib::ControllerSettings& controller,
    float target,
    int timeout_ms,
    int loop_delay_ms  // keep default ONLY in the header
) {
    auto* args = new AngularPidLogArgs{
        .chassis       = chassis,
        .imu           = imu,
        .controller    = controller,
        .target        = target,
        .timeout_ms    = timeout_ms,
        .loop_delay_ms = loop_delay_ms,
    };
    args->stop.store(false, std::memory_order_release);

    // fire-and-forget logging task; it will free args when done
    pros::Task{angular_pid_logging_task, args};

    pros::delay(100);  // Give logging task time to start

    // Perform the motion while logging runs
    // Make this call BLOCKING so logging continues throughout the move
    chassis->turnToHeading(target, timeout_ms, {}, false);

    // Signal the logger to stop once the motion is done
    args->stop.store(true, std::memory_order_release);
}

void start_lateral_pid_logging_task(
    lemlib::Chassis* chassis,
    pros::Imu* imu,
    const lemlib::ControllerSettings& controller,
    float target,
    int timeout_ms,
    int loop_delay_ms  // keep default ONLY in the header
) {
    auto* args = new AngularPidLogArgs{
        .chassis       = chassis,
        .imu           = imu,
        .controller    = controller,
        .target        = target,
        .timeout_ms    = timeout_ms,
        .loop_delay_ms = loop_delay_ms,
    };
    args->stop.store(false, std::memory_order_release);

    // fire-and-forget logging task; it will free args when done
    pros::Task{lateral_pid_logging_task, args};

    pros::delay(100);  // Give logging task time to start

    // Perform the motion while logging runs
    // Make this call BLOCKING so logging continues throughout the move
    chassis->moveToPose(0, target, 0, timeout_ms, {}, false);

    // Signal the logger to stop once the motion is done
    args->stop.store(true, std::memory_order_release);
}