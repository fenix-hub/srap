/*
 * PIDController.h
 *
 *  Created on: Sep 5, 2022
 *      Author: nsant
 */

#ifndef INC_PIDCONTROLLER_H_
#define INC_PIDCONTROLLER_H_

typedef struct PIDController
{
    float kp;
    float ki;
    float kd;
    float min_i;
    float max_i;
    float *setpoint;
    float *input;
    float *output;
    uint8_t anti_windup;

    float t0;
    float last_error;
} PIDController;

/*--------*/

PIDController new_PID(float *setpoint, float *input, float *output);
void set_PID_parameters(PIDController pid_controller, float kp, float ki, float kd);
void set_PID_limits(PIDController pid_controller, float min_i, float max_i);
void PID_enable_antiwindup(PIDController pid_controller);
void PID_disable_antiwindup(PIDController pid_controller);
void compute_PID(PIDController pid_controller);


#endif /* INC_PIDCONTROLLER_H_ */
