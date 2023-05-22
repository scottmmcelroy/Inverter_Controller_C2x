/*
 * Voltage.c
 *
 *  Created on: Jun 12, 2019
 *      Author: mfeurtado
 */
#include "Voltage.h"
#include "driverlib.h"
#include "device.h"
#include <math.h>

float32_t getVoltageA()
{
    float val;
    val = (float32_t)2400*((float32_t)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0)/(float32_t)4095)-(float32_t)1200;
    return val;
}



float32_t getVoltageB()
{
    float val = 0;
    val = (float32_t)2400*((float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER3)/(float32_t)4095)-(float32_t)1200;
    return val;
}


float32_t getVoltageC()
{
    float val = 0;
    val = (float32_t)2400*((float32_t)ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1)/(float32_t)4095)-(float32_t)1200;
    return val;
}


float32_t getVoltageDC()
{
    float val = 0;
    val = (float32_t)1200*((float32_t)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1)/(float32_t)4095);
    return val;
}
