/*
 * operations.c
 *
 *  Created on: Aug 3, 2022
 *      Author: nsant
 */
#include <operations.h>

uint16_t min(uint16_t *buff, size_t length) {
	uint16_t min = buff[0];
	for (uint16_t i = 1; i < length; ++i) {
		if (buff[i] < min) {
			min = buff[i];
		}
	}
	return min;
}

uint16_t max(uint16_t *buff, size_t length) {
	uint16_t max = buff[0];
	for (uint16_t i = 1; i < length; ++i) {
		if (buff[i] > max) {
			max = buff[i];
		}
	}
	return max;
}

float map(float val, const float in_min, const float in_max, const float out_min, const float out_max) {
	return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void normalize(const uint16_t* samples, size_t length, float* output, uint8_t normalization) {
	for (uint16_t i = 0; i < length; i++) {
		output[i] = (float) samples[i] / normalization;
	}
}

float average(const uint16_t* samples, size_t length) {
	float sum = 0.0f;
	for (uint16_t i = 0; i < length; ++i) {
		sum += samples[i];
	}
	return ( sum / length );
}

float relative_average(const float* samples, size_t length, uint8_t moving_point) {
	float sum = 0.0f;
	uint16_t half_interval = (moving_point - 1) / 2;

	for (uint16_t i = half_interval; i < length - half_interval; ++i) {
		sum += samples[i];
	}
	return sum / (length - moving_point - 1) ;
}

void moving_average_filter(const uint16_t* samples, float* filtered, size_t length, uint16_t moving_point) {
	uint16_t half_interval = (moving_point - 1) / 2;

	for (uint16_t i = half_interval; i < length - half_interval; i++) {
		filtered[i] = 0.0f;
		for (int16_t j = -half_interval; j < half_interval; j++) {
			filtered[i] = filtered[i] + samples[i + j];
		}
		filtered[i] = (float) filtered[i] / moving_point;
	}

}

void moving_average_filterR(const uint16_t* samples, float* filtered, size_t length, uint16_t moving_point) {

	float t_filtered[length];
	memset(t_filtered, 0, length);

	uint16_t p = (moving_point - 1) / 2;

	// Find accumulation value
	float acc = 0.0f;
	for (int16_t i = 0; i < (moving_point - 1); i++) {
		acc += samples[i];
	}
	t_filtered[p] = acc / moving_point;

	// y[i] = y[i-1] + x[i+p] — x[i—q]
	// p = (M-1)/2
	// q = p+1
	for (int16_t i = (p + 1); i < length - p; i++) {
		acc = acc + samples[i + p] - samples[i - (p+1)];
		t_filtered[i] = acc;
	}

	memcpy(filtered, t_filtered, length);

}

