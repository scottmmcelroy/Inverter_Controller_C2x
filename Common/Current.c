/*
 * Current.c
 *
 *  Created on: Jun 12, 2019
 *      Author: mfeurtado
 */
#include "Current.h"
#include "driverlib.h"
#include "device.h"
#include <math.h>

float32_t getCurrentA(void)
{
    float val;
    val = (float32_t)1600*((float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0)/(float32_t)4095)-(float32_t)800;
    return val;
}

float32_t getCurrentB(void)
{
    float val;
    val = (float32_t)1600*((float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER2)/(float32_t)4095)-(float32_t)800;
    return val;
}

float32_t getCurrentC(void)
{
    float val;
    val = (float32_t)1600*((float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0)/(float32_t)4095)-(float32_t)800;
    return val;
}

float32_t getCurrentEXT(void)
{
    float val;
    val = (float32_t)1600*((float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1)/(float32_t)4095)-(float32_t)800;
    return val;
}

