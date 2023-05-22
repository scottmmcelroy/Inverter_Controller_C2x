/*
 * TEMPERATURE.h
 *
 *  Created on: Jun 11, 2019
 *      Author: mfeurtado
 */

#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_
#include "device.h"

#define capCountsize  10

void initECAP1();
void initECAP2();
void initECAP3();
float32_t getCaseTemp();
float32_t getAnalogTempA();
float32_t getAnalogTempB();
float32_t getAnalogTempC();
float32_t getAnalogNTCA();
float32_t getAnalogNTCB();
float32_t getAnalogNTCC();

float32_t getECAPTempA();
float32_t getECAPTempB();
float32_t getECAPTempC();
float32_t getECAPNTCA();
float32_t getECAPNTCB();
float32_t getECAPNTCC();
#endif /* TEMPERATURE_H_ */
