/*
 * operations.h
 *
 *  Created on: Aug 3, 2022
 *      Author: nsant
 */

#ifndef INC_OPERATIONS_H_
#define INC_OPERATIONS_H_

#include "stm32f4xx_hal.h"
#include <string.h>

// Find the MIN value of an array of samples
uint16_t min(uint16_t* buffer, size_t length);

// Find the MAX value of an array of samples
uint16_t max(uint16_t* buffer, size_t length);

// Normalize an array of values by a normalization factor
void normalize(const uint16_t* samples, size_t length, float* output, uint8_t normalization);

// Compute the average value of an array of samples
float average(const uint16_t* samples, size_t length);

// Compute the average value of an array of samples, relative to a moving point
float relative_average(const float* samples, size_t length, uint8_t moving_point);

// Moving Average Filter caluclated by convolution
void moving_average_filter(const uint16_t* samples, float* filtered, size_t length, uint16_t moving_point);

// Moving Average Filter caluclated by recursion
void moving_average_filterR(const uint16_t* samples, float* filtered, size_t length, uint16_t moving_point);

// Map an input value from one range [in_min, in_max] to another [out_min, out_max]
float map(float val, const float in_min, const float in_max, const float out_min, const float out_max);

#endif /* INC_OPERATIONS_H_ */
