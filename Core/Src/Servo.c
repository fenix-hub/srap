/*
 * Servo.c
 *
 *  Created on: Oct 27, 2022
 *      Author: nsant
 */
#include <Servo.h>

Servo new_servo(TIM_HandleTypeDef* TIM_HandleInstance, uint16_t TIM_Channel, float min_angle, float max_angle, float min_dc, float max_dc, float dc_offset) {
	Servo s;
	s.TIM_HandleInstance = TIM_HandleInstance;
	s.TIM_Channel = TIM_Channel;
	s.max_angle = max_angle;
	s.min_angle = min_angle;
	s.max_dc = max_dc;
	s.min_dc = min_dc;
	s.dc_offset = dc_offset;
	return s;
}

void start_servo(Servo *servo) {
	HAL_TIM_PWM_Start(servo->TIM_HandleInstance, servo->TIM_Channel);
}

void stop_servo(Servo *servo) {
	HAL_TIM_PWM_Stop(servo->TIM_HandleInstance, servo->TIM_Channel);
	servo_write_dc(servo, servo->max_dc);
}

void servo_write_deg(Servo *servo, float deg) {
	float dc = (deg - servo->min_angle) * (servo->max_dc - servo->min_dc) / (servo->max_angle - servo->min_angle) + servo->min_dc;
	servo_write_dc(servo, dc);
}

void servo_write_dc(Servo *servo, float dc) {
	uint32_t ccr = (uint32_t) roundf(((float) (servo->TIM_HandleInstance->Instance->ARR + 1.0) / 100.0) * (dc + servo->dc_offset) );
	servo_write_ccr(servo, ccr);
}

void servo_write_ccr(Servo *servo, uint32_t CCR) {
	__HAL_TIM_SET_COMPARE(servo->TIM_HandleInstance, servo->TIM_Channel, CCR);
	servo->TIM_HandleInstance->Instance->EGR = TIM_EGR_UG;
}


