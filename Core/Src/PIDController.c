/*
 * PIDController.c
 *
 *  Created on: Sep 5, 2022
 *      Author: nsant
 */

#include <PIDController.h>

PIDController new_PID(float kp, float ki, float kd, float *setpoint, float *input, float *output) {
    PIDController pid_controller;
    pid_controller.setpoint = setpoint;
    pid_controller.input = input;
    pid_controller.output = output;
    return pid_controller;
}

void set_PID_parameters(PIDController pid_controller, float kp, float ki, float kd) {
    pid_controller.kp = kp;
    pid_controller.ki = ki;
    pid_controller.kd = kd;
}

void set_PID_limits(PIDController pid_controller, float min_i, float max_i) {
    pid_controller.min_i = min_i;
    pid_controller.max_i = max_i;
}

void PID_enable_antiwindup(PIDController pid_controller) {
    pid_controller.anti_windup |= (1 << 0);
}

void PID_disable_antiwindup(PIDController pid_controller) {
    pid_controller.anti_windup &= ~(1 << 0);
}

void compute_PID(PIDController pid_controller) {
    float t1 = HAL_GetTick();
    float elapsed = (t1 - pid_controller.t0);

    float error = pid_controller.setpoint - pid_controller.input;
    float delta_error = error - pid_controller.last_error;


    float p = pid_controller.kp * error;
    float i = pid_controller.ki * error * elapsed;
    if (i > pid_controller.max_i) {
        i = pid_controller.max_i;
    }
    else if (i < pid_controller.min_i) {
        i = pid_controller.min_i;
    }
    float d = pid_controller.kd * delta_error / elapsed;

    pid_controller.output = p + i + d;

    pid_controller.last_error = error;
    pid_controller.t0 = t1;
}


