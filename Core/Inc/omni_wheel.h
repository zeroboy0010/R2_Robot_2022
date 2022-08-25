
#ifndef INC_OMNI_WHEEL_H_
#define INC_OMNI_WHEEL_H_

#include "stdint.h"

#define pi 3.1415

#define Motor1 0
#define Motor2 1
#define Motor3 2
#define Motor4 3
// IRQ encoder
uint8_t nowA[4];
uint8_t nowB[4];
uint16_t cnt[4];
uint8_t lastA[4];
uint8_t lastB[4];
uint16_t Enc_count[4];
uint8_t dir[4];
uint16_t encoder(int i);

#define wheel_r 6.10	// cm
#define wheel_D 26.0

uint16_t count[4];//count pulse from encoder
uint16_t new_count[4];
uint8_t count_state[4];
uint16_t diff[4];//difference between count and new_count in a sample time

float speedM[4];
float rdps[4];

float wheel1(float vx, float vy, float omega);
float wheel2(float vx, float vy, float omega);
float wheel3(float vx, float vy, float omega);
float wheel4(float vx, float vy, float omega);

float Motors_RPS(int j, float sampleTime, float N_round);

#endif /* INC_OMNI_WHEEL_H_ */
