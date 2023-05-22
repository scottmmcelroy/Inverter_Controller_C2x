/*
 * TEMPERATURE.c
 *
 *  Created on: Jun 11, 2019
 *      Author: mfeurtado
 */

#include "TEMPERATURE.h"
#include "driverlib.h"
#include "device.h"
#include <math.h>

extern uint16_t cap1Count[];
extern uint16_t cap2Count[];
extern uint16_t cap3Count[];

uint16_t beta = 3435;

    /*
    GPIO-94   A-RTD -> eCAP1
    GPIO-95   C-RTD -> eCAP3
    GPIO-97   B-RTD -> eCAP2
    */

void initECAP1()
{
    //
    // Disable ,clear all capture flags and interrupts
    //
    ECAP_disableInterrupt(ECAP1_BASE,
                          (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                           ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                           ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                           ECAP_ISR_SOURCE_COUNTER_COMPARE));
    ECAP_clearInterrupt(ECAP1_BASE,
                        (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                         ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                         ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                         ECAP_ISR_SOURCE_COUNTER_COMPARE));

    //
    // Disable CAP1-CAP4 register loads
    //
    ECAP_disableTimeStampCapture(ECAP1_BASE);

    //
    // Configure eCAP
    //    Enable capture mode.
    //    One shot mode, stop capture at event 4.
    //    Set polarity of the events to rising, falling, rising, falling edge.
    //    Set capture in time difference mode.
    //    Select input from XBAR7.
    //    Enable eCAP module.
    //    Enable interrupt.
    //
    ECAP_stopCounter(ECAP1_BASE);
    ECAP_enableCaptureMode(ECAP1_BASE);

    ECAP_setCaptureMode(ECAP1_BASE, ECAP_ONE_SHOT_CAPTURE_MODE, ECAP_EVENT_4);

    ECAP_setEventPolarity(ECAP1_BASE, ECAP_EVENT_1, ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(ECAP1_BASE, ECAP_EVENT_2, ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(ECAP1_BASE, ECAP_EVENT_3, ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(ECAP1_BASE, ECAP_EVENT_4, ECAP_EVNT_RISING_EDGE);

    ECAP_enableCounterResetOnEvent(ECAP1_BASE, ECAP_EVENT_1);
    ECAP_enableCounterResetOnEvent(ECAP1_BASE, ECAP_EVENT_2);
    ECAP_enableCounterResetOnEvent(ECAP1_BASE, ECAP_EVENT_3);
    ECAP_enableCounterResetOnEvent(ECAP1_BASE, ECAP_EVENT_4);


    //ECAP_enableLoadCounter(ECAP1_BASE);
    ECAP_disableLoadCounter(ECAP1_BASE); //no sync in
    ECAP_setSyncOutMode(ECAP1_BASE, ECAP_SYNC_OUT_DISABLED);
    ECAP_startCounter(ECAP1_BASE);
    ECAP_enableTimeStampCapture(ECAP1_BASE);
    ECAP_reArm(ECAP1_BASE);

    ECAP_enableInterrupt(ECAP1_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_4);
}

void initECAP2()
{
    //
    // Disable ,clear all capture flags and interrupts
    //
    ECAP_disableInterrupt(ECAP2_BASE,
                          (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                           ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                           ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                           ECAP_ISR_SOURCE_COUNTER_COMPARE));
    ECAP_clearInterrupt(ECAP2_BASE,
                        (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                         ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                         ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                         ECAP_ISR_SOURCE_COUNTER_COMPARE));

    //
    // Disable CAP1-CAP4 register loads
    //
    ECAP_disableTimeStampCapture(ECAP2_BASE);

    //
    // Configure eCAP
    //    Enable capture mode.
    //    One shot mode, stop capture at event 4.
    //    Set polarity of the events to rising, falling, rising, falling edge.
    //    Set capture in time difference mode.
    //    Select input from XBAR8.
    //    Enable eCAP module.
    //    Enable interrupt.
    //
    ECAP_stopCounter(ECAP2_BASE);
    ECAP_enableCaptureMode(ECAP2_BASE);

    ECAP_setCaptureMode(ECAP2_BASE, ECAP_ONE_SHOT_CAPTURE_MODE, ECAP_EVENT_4);

    ECAP_setEventPolarity(ECAP2_BASE, ECAP_EVENT_1, ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(ECAP2_BASE, ECAP_EVENT_2, ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(ECAP2_BASE, ECAP_EVENT_3, ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(ECAP2_BASE, ECAP_EVENT_4, ECAP_EVNT_RISING_EDGE);

    ECAP_enableCounterResetOnEvent(ECAP2_BASE, ECAP_EVENT_1);
    ECAP_enableCounterResetOnEvent(ECAP2_BASE, ECAP_EVENT_2);
    ECAP_enableCounterResetOnEvent(ECAP2_BASE, ECAP_EVENT_3);
    ECAP_enableCounterResetOnEvent(ECAP2_BASE, ECAP_EVENT_4);


    //ECAP_enableLoadCounter(ECAP2_BASE);
    ECAP_disableLoadCounter(ECAP2_BASE); //no sync in
    ECAP_setSyncOutMode(ECAP2_BASE, ECAP_SYNC_OUT_DISABLED);
    ECAP_startCounter(ECAP2_BASE);
    ECAP_enableTimeStampCapture(ECAP2_BASE);
    ECAP_reArm(ECAP2_BASE);

    ECAP_enableInterrupt(ECAP2_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_4);
}


void initECAP3()
{
    //
    // Disable ,clear all capture flags and interrupts
    //
    ECAP_disableInterrupt(ECAP3_BASE,
                          (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                           ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                           ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                           ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                           ECAP_ISR_SOURCE_COUNTER_COMPARE));
    ECAP_clearInterrupt(ECAP3_BASE,
                        (ECAP_ISR_SOURCE_CAPTURE_EVENT_1  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_2  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_3  |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_4  |
                         ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                         ECAP_ISR_SOURCE_COUNTER_PERIOD   |
                         ECAP_ISR_SOURCE_COUNTER_COMPARE));

    //
    // Disable CAP1-CAP4 register loads
    //
    ECAP_disableTimeStampCapture(ECAP3_BASE);

    //
    // Configure eCAP
    //    Enable capture mode.
    //    One shot mode, stop capture at event 4.
    //    Set polarity of the events to rising, falling, rising, falling edge.
    //    Set capture in time difference mode.
    //    Select input from XBAR9.
    //    Enable eCAP module.
    //    Enable interrupt.
    //
    ECAP_stopCounter(ECAP3_BASE);
    ECAP_enableCaptureMode(ECAP3_BASE);

    ECAP_setCaptureMode(ECAP3_BASE, ECAP_ONE_SHOT_CAPTURE_MODE, ECAP_EVENT_4);

    ECAP_setEventPolarity(ECAP3_BASE, ECAP_EVENT_1, ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(ECAP3_BASE, ECAP_EVENT_2, ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(ECAP3_BASE, ECAP_EVENT_3, ECAP_EVNT_RISING_EDGE);
    ECAP_setEventPolarity(ECAP3_BASE, ECAP_EVENT_4, ECAP_EVNT_RISING_EDGE);

    ECAP_enableCounterResetOnEvent(ECAP3_BASE, ECAP_EVENT_1);
    ECAP_enableCounterResetOnEvent(ECAP3_BASE, ECAP_EVENT_2);
    ECAP_enableCounterResetOnEvent(ECAP3_BASE, ECAP_EVENT_3);
    ECAP_enableCounterResetOnEvent(ECAP3_BASE, ECAP_EVENT_4);


    //ECAP_enableLoadCounter(ECAP3_BASE);
    ECAP_disableLoadCounter(ECAP3_BASE);
    ECAP_setSyncOutMode(ECAP3_BASE, ECAP_SYNC_OUT_DISABLED);
    ECAP_startCounter(ECAP3_BASE);
    ECAP_enableTimeStampCapture(ECAP3_BASE);
    ECAP_reArm(ECAP3_BASE);

    ECAP_enableInterrupt(ECAP3_BASE, ECAP_ISR_SOURCE_CAPTURE_EVENT_4);
}

// getECAPTempA
// measure NTC frequency modulated signal with ECAP convert to resistance then to temperature
// average over the last capCountsize number of samples
// RETURN: temperature in Kelvin
float32_t getECAPTempA()
{
    uint32_t avgCount = 0;
    uint16_t i;
    for(i=0;i<capCountsize;i++)
    {
        avgCount += cap1Count[i];
    }
    avgCount = avgCount / capCountsize;

    float32_t val, res, freq;
    freq = 1.0 / ((float32_t)avgCount * 5e-9);
    //res = 33931 * expf(-2E-4 * freq); //full range up to 175C less accurate
    res = 24771 * expf(-1.4923E-4 * freq); //more accurate up to 100C
    val = 1.0 / ((1/298.15)+(logf(res/4700)/beta));

    return val;
}

// getECAPTempB
// measure NTC frequency modulated signal with ECAP convert to resistance then to temperature
// average over the last capCountsize number of samples
// RETURN: temperature in Kelvin
float32_t getECAPTempB()
{
    uint32_t avgCount = 0;
    uint16_t i;
    for(i=0;i<capCountsize;i++)
    {
        avgCount += cap2Count[i];
    }
    avgCount = avgCount / capCountsize;

    float32_t val, res, freq;
    freq = 1.0 / ((float32_t)avgCount * 5e-9);
    //res = 33931 * expf(-2E-4 * freq); //full range up to 175C less accurate
    res = 24771 * expf(-1.4923E-4 * freq); //more accurate up to 100C
    val = 1.0 / ((1/298.15)+(logf(res/4700)/beta));
    return val;
}

// getECAPTempC
// measure NTC frequency modulated signal with ECAP convert to resistance then to temperature
// average over the last capCountsize number of samples
// RETURN: temperature in Kelvin
float32_t getECAPTempC()
{
    uint32_t avgCount = 0;
    uint16_t i;
    for(i=0;i<capCountsize;i++)
    {
        avgCount += cap3Count[i];
    }
    avgCount = avgCount / capCountsize;

    float32_t val, res, freq;
    freq = 1.0 / ((float32_t)avgCount * 5e-9);
    //res = 33931 * expf(-1.76E-4 * freq); //full range up to 175C less accurate
    res = 24771 * expf(-1.4923E-4 * freq); //more accurate up to 100C
    val = 1.0 / ((1/298.15)+(logf(res/4700)/beta));
    return val;
}



// getECAPNTCA
// measure NTC frequency modulated signal with ECAP convert to resistance
// RETURN: NTC resistance in ohms
float32_t getECAPNTCA()
{
    float32_t res, freq;
    freq = 1.0 / ((float32_t)cap1Count[0] * 5e-9);
    res = 33931 * expf(-2E-4 * freq);
    return res;
}

// getECAPNTCB
// measure NTC frequency modulated signal with ECAP convert to resistance
// RETURN: NTC resistance in ohms
float32_t getECAPNTCB()
{
    float32_t res, freq;
    freq = 1.0 / ((float32_t)cap2Count[0] * 5e-9);
    res = 33931 * expf(-2E-4 * freq);
    return res;
}

// getECAPNTCC
// measure NTC frequency modulated signal with ECAP convert to resistance
// RETURN: NTC resistance in ohms
float32_t getECAPNTCC()
{
    float32_t res, freq;
    freq = 1.0 / ((float32_t)cap3Count[0] * 5e-9);
    res = 33931 * expf(-1.76E-4 * freq);
    return res;
}



// getCaseTemp
// measure 10k NTC on controller PCB with 10k pull-down
// RETURN: temperature in Kelvin

float32_t getCaseTemp()
{
    float32_t val, t;
    val = ((float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER6)/(float32_t)4095); //read ADC from CASE-TEMP
    val = val * 3.0F;
    //float32_t res;
    //res = ((3.3F*10000)/val) - 10000; //ADC is 3.0V reference but the NTC is tied to 3.3V
    t = 1.0F / ((logf(3.3F/val - 1.0F)/3900)+(1.0F/298.15F));
    return t;
}

// getAnalogTempA
// measure analog ADC voltage and convert to temperature
// RETURN: Module NTC Temperature in Kelvin
float32_t getAnalogTempA()
{
    float32_t val, res, volt;
    volt = 3.0F *(float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER4)/(float32_t)4095;
    res = 31937 * expf(-2.344F *volt);
    val = 1.0 / ((1/298.15)+(logf(res/4700)/beta));
    return val;
}

// getAnalogTempB
// measure analog ADC voltage and convert to temperature
// RETURN: Module NTC Temperature in Kelvin
float32_t getAnalogTempB()
{
    float32_t val, res, volt;
    volt = 3.0F *(float32_t)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER2)/(float32_t)4095;
    res = 31937 * expf(-2.344F *volt);
    val = 1.0 / ((1/298.15)+(logf(res/4700)/beta));
    return val;
}

// getAnalogTempC
// measure analog ADC voltage and convert to temperature
// RETURN: Module NTC Temperature in Kelvin
float32_t getAnalogTempC()
{
    float32_t val, res, volt;
    volt = 3.0F *(float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER5)/(float32_t)4095;
    res = 31937 * expf(-2.344F *volt);
    val = 1.0 / ((1/298.15)+(logf(res/4700)/beta));
    return val;
}

// getAnalogNTCA
// measure analog ADC voltage and convert to NTC resistance
// RETURN: NTC resistance in ohms
float32_t getAnalogNTCA()
{
    float32_t val;
    val = 3.0F *(float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER4)/(float32_t)4095;
    val = 31937 * expf(-2.344F *val);
    return val;
}

// getAnalogNTCB
// measure analog ADC voltage and convert to NTC resistance
// RETURN: NTC resistance in ohms
float32_t getAnalogNTCB()
{
    float32_t val;
    val = 3.0F *(float32_t)ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER2)/(float32_t)4095;
    val = 31937 * expf(-2.344F *val);
    return val;
}

// getAnalogNTCC
// measure analog ADC voltage and convert to NTC resistance
// RETURN: NTC resistance in ohms
float32_t getAnalogNTCC()
{
    float32_t val;
    val = 3.0F *(float32_t)ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER5)/(float32_t)4095;
    val = 31937 * expf(-2.344F *val);
    return val;
}
