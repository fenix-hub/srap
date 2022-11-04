/*
 * Servo.h
 *
 *  Created on: Oct 27, 2022
 *      Author: nsant
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32f4xx_hal.h"
#include <math.h>

// The Number OF Servo Motors To Be Used In The Project
#define SERVO_NUM  1

typedef struct Servo
{
    TIM_HandleTypeDef*	TIM_HandleInstance;
    uint16_t       		TIM_Channel;
    float 				min_angle;
    float 				max_angle;
    float 				min_dc;
    float 				max_dc;
    float 				dc_offset;
} Servo;

/*-----[ Prototypes For All Functions ]-----*/

Servo new_servo(TIM_HandleTypeDef* TIM_HandleInstance, uint16_t TIM_Channel, float min_angle, float max_angle, float min_dc, float max_dc, float dc_offset);
void start_servo(Servo servo);
void stop_servo(Servo servo);
void servo_write_deg(Servo servo, float deg);
void servo_write_dc(Servo servo, float dc);
void servo_write_ccr(Servo servo, uint32_t CCR);

#endif /* INC_SERVO_H_ */
