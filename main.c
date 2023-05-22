//#############################################################################
//
// FILE:    main.c
//
// TITLE:   XM3 Controller Example Code
//
// AUTHOR: mfeurtado, Matthew Feurtado
//
// PURPOSE:
//  This software is designed for the evaluation of the XM3 3-phase Inverter
//  Reference Design (CRD300DA12E-XM3) and provides only the basic code
//  required to create an open-loop inverter. The basic 3-phase PWM parameters
//  can be controlled over a CAN interface along with rudimentary feedback from
//  the on-board sensors. It is designed as a starting point only and it is
//  left to the user any customization for a particular application.
//
// REVISION HISTORY:
//
//  V1.0 -2/28/2019- Initial test code release. Used to run first 3-phase power
//  tests of XM3 Reference Design Inverter
//  -rewrote piCAN'T communication code to work with new D-CAN module for the
//  latest Delfino.
//  -PWM fade case LEDs
//
//  V1.1 -3/7/2019-
//  -updated sine PWM to use fewer #defines and more structs
//  -added ADC routines
//
//  V1.2 -6/5/2019
//  -updated RTD measurement to frequency modulation detection with eCAP
//  -added CAN feedback channels for temperature, current, and voltage
//
//  V1.3 -1/6/2020
//  -changed PWM mode from up count to up-down count and adjusted PWM period
//  accordingly in updatePWM, for symmetrical PWM
//
//
//#############################################################################

//
// Included Files
//
//#define _LAUNCHXL_F28379D
#include "driverlib.h"
#include "device.h"
#include "GATEDRIVER.h"
#include "CANSetup.h"
#include "Analog.h"
#include "Temperature.h"
#include "Current.h"
#include "Voltage.h"
#include <math.h>

//
// Defines
//
//CASELED1 PWM for effect
#define EPWM_CMP_UP           1U
#define EPWM_CMP_DOWN         0U
#define EPWM6_TIMER_TBPRD  2000U
#define EPWM6_MAX_CMPA     1950U
#define EPWM6_MIN_CMPA       50U

//
// Globals
//
typedef struct
{
    uint32_t epwmModule;
    uint16_t epwmPeriod;
    float epwmPwmPhase;
    uint16_t epwmDeadTime;
    float epwmRadian;
}epwmInformation;

typedef struct
{
    uint32_t epwmModule;
    uint16_t epwmCompADirection;
    uint16_t epwmTimerIntCount;
    uint16_t epwmMaxCompA;
    uint16_t epwmMinCompA;
}LEDepwmInformation;
//
// Globals to hold the ePWM information used in this example
//
epwmInformation epwm1Info;
epwmInformation epwm2Info;
epwmInformation epwm3Info;
LEDepwmInformation epwmLEDInfo;

//
// Function Prototypes
//
void initGPIO(void);

void TestLEDS(void);
void TestShutdown(void);
void enableNeg15V(void);
void enablePos15V(void);
void disableNeg15V(void);
void disablePos15V(void);
void TestLEDPulse(void);

void initEPWM1(void);
void initEPWM2(void);
void initEPWM3(void);
void initCaseLEDPWM(void);
__interrupt void epwm1ISR(void);
//__interrupt void epwm2ISR(void);
//__interrupt void epwm3ISR(void);
__interrupt void epwm6ISR(void);
__interrupt void epwm1TZISR(void);
__interrupt void epwm2TZISR(void);
__interrupt void epwm3TZISR(void);
void updatePWM(epwmInformation *epwmInfo);
void updateLED(LEDepwmInformation *epwmInfo);

void CANPacketEncode(uint16_t *PacketData);
void CANPacketDecode(uint16_t *PacketData);

//eCAP ISR for measuring NTC frequency feedback signal
__interrupt void ecap1ISR(void);
__interrupt void ecap2ISR(void);
__interrupt void ecap3ISR(void);

#define capCountsize  10
uint16_t cap1Count[capCountsize];
uint16_t cap1index;
uint16_t cap2Count[capCountsize];
uint16_t cap2index;
uint16_t cap3Count[capCountsize];
uint16_t cap3index;


uint16_t FS = 0;
uint16_t FF = 0;
uint16_t ID = 0;
uint16_t TD = 0;
uint16_t PSEN1 = 0;
uint16_t PSEN2 = 0;
uint16_t PSEN3 = 0;
uint16_t LEN1 = 0;
uint16_t LEN2 = 0;
uint16_t LEN3 = 0;
uint16_t FAULT1 = 0;
uint16_t FAULT2 = 0;
uint16_t FAULT3 = 0;
uint16_t RESET = 0;
float DUTY = 0;
uint16_t FUND_FREQ;       // Fundamental frequency of sine wave
uint16_t EPwm_TBPRD;
float MF;               // Modulation factor or modulation depth (0 - 1)
uint16_t SWITCHING_FREQ;  // Switching frequency duh
uint16_t DEAD_TIME;       // Dead time in clock cycles (1 = 6.67 ns)

#define PI 3.141592654  // Pi
float Sine;             //
float Ts;               //
float radian;
uint16_t rampFreq = 0;  //frequency ramping value, will finish at FUND_FREQ
uint16_t delay_count = 0;
uint16_t delay_count_max = 1000; //100 cycle per frequency resolution adjustment (start-up)
uint16_t final_freq = 0; //storage variable while FUND_FREQ gets adjusted
//
// Main
//
void main(void)
{
    //
    // Initializes system control, device clock, and peripherals
    //
    Device_init();
    //
    // Disable pin locks and enable internal pull ups.
    //
    Device_initGPIO();
    //
    // Initializes PIE and clear PIE registers. Disables CPU interrupts.
    // and clear all CPU interrupt flags.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();
    //
    // Assign the interrupt service routines to ePWM interrupts
    //
    Interrupt_register(INT_EPWM1, &epwm1ISR);
    //Interrupt_register(INT_EPWM2, &epwm2ISR);
    //Interrupt_register(INT_EPWM3, &epwm3ISR);
    Interrupt_register(INT_EPWM6, &epwm6ISR);
    Interrupt_register(INT_EPWM1_TZ, &epwm1TZISR);
    Interrupt_register(INT_EPWM2_TZ, &epwm2TZISR);
    Interrupt_register(INT_EPWM3_TZ, &epwm3TZISR);

    Interrupt_register(INT_ECAP1, &ecap1ISR);
    Interrupt_register(INT_ECAP2, &ecap2ISR);
    Interrupt_register(INT_ECAP3, &ecap3ISR);

    //
    // This example is a basic pinout
    //
    initGateDriverGPIO();
    GD_ALL_PSDisable(); //disable all gate drivers for startup
    initCANGPIO();
    initGPIO();

    disableNeg15V();
    disablePos15V();

    //initialize switching parameters
    SWITCHING_FREQ = 10e3;   // Default of 20 kHz switching frequency
    DEAD_TIME = 100;         // 1.3us of dead time by default
    FUND_FREQ = 50;         // Default of 300 Hz fundamental frequency
    final_freq = FUND_FREQ;
    FUND_FREQ = 0;
    rampFreq = 0;
    MF = 0.08;                // Default of 0.9 modulation depth
    radian = 0;              // Initialize radian to 0
    Ts = 2*PI/(SWITCHING_FREQ/FUND_FREQ);  // x

    LEN1 = 1;
    LEN2 = 1;
    LEN3 = 1;
    PSEN1 = 1;
    PSEN2 = 1;
    PSEN3 = 1;
    FAULT1 = 0;
    FAULT2 = 0;
    FAULT3 = 0;
    //
    // Disable sync(Freeze clock to PWM as well)
    //
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    initEPWM1();
    //comment out the two lines below to turn off two half-bridges
    initEPWM2();
    initEPWM3();

    initCaseLEDPWM();

    initECAP1();
    //comment out the two lines below to turn off two half-bridges
    initECAP2();
    initECAP3();

    //
    // Enable sync and clock to PWM
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Enable ePWM interrupts
    //
    Interrupt_enable(INT_EPWM1);
    //Interrupt_enable(INT_EPWM2);
    //Interrupt_enable(INT_EPWM3);
    Interrupt_enable(INT_EPWM6);
    Interrupt_enable(INT_ECAP1);
    Interrupt_enable(INT_ECAP2);
    Interrupt_enable(INT_ECAP3);
    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //enable power to all gate drivers
    GD_ALL_PSEnable();
    //allow time for GD PSU to startup
    DEVICE_DELAY_US(200);
    //reset all gate drivers and set OC_EN high (desat enabled)
    GD_ALL_Reset();

    //
    // Set up ADCs, initializing the SOCs to be triggered by software
    //
    initADCs();
    initADCSOCs();

    //enable Current Sensor Power Supply, 10ms starup delay on power supplies
    enableNeg15V();
    enablePos15V();


    initCAN();
    uint16_t txMsgData[8], rxMsgData[8];
    uint16_t TemperatureMsgData[8];
    uint16_t CurrentMsgData[8];
    uint16_t VoltageMsgData[8];
    *(uint16_t *)rxMsgData = 0;
    while(1){
        NOP;
        DEVICE_DELAY_US(1000000);
        //
        // Read CAN message object 2 and check for new data
        //
        if (CAN_readMessage(CANA_BASE, 2, rxMsgData))
        {
            GPIO_togglePin(52);
            CANPacketDecode(rxMsgData);
            CANPacketEncode(txMsgData);
            CAN_sendMessage(CANA_BASE, 1, 8, txMsgData);
        }

        //check fault status
        //note fast response is done in Tripzone this is for UI status

        if(GD_Global_getFault()) //faults are are combined together, active low
        {

            GD_ALL_LogicDisable();
            LEN1 = 0;
            LEN2 = 0;
            LEN3 = 0;
            FAULT1 = GD_A_getFault();
            FAULT2 = GD_B_getFault();
            FAULT3 = GD_C_getFault();
        }

        //send status update
        CANPacketEncode(txMsgData);
        CAN_sendMessage(CANA_BASE, 1, 8, txMsgData);

        //Read analogs
        //
        // Convert, wait for completion, and store results
        //
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER0);
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER1);
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER2);
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER3);
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER4);
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER5);
        ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER6);
        ADC_forceSOC(ADCB_BASE, ADC_SOC_NUMBER0);
        ADC_forceSOC(ADCB_BASE, ADC_SOC_NUMBER1);
        ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER0);
        ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER1);
        ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER2);

        //
        // Wait for ADCA to complete, then acknowledge flag
        //
        while(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1) == false)
        {
        }
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

        //
        // Wait for ADCB to complete, then acknowledge flag
        //
        while(ADC_getInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1) == false)
        {
        }
        ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);
        //
        // Wait for ADCC to complete, then acknowledge flag
        //
        while(ADC_getInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1) == false)
        {
        }
        ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);

        //
        // Store results
        //

        TemperatureMsgData[0] = (uint16_t)getECAPTempA()>>8; //A-Temp
        TemperatureMsgData[1] = (uint16_t)getECAPTempA();

        TemperatureMsgData[2] = (uint16_t)getECAPTempB()>>8; //B-Temp
        TemperatureMsgData[3] = (uint16_t)getECAPTempB();

        TemperatureMsgData[4] = (uint16_t)getECAPTempC()>>8; //C-Temp
        TemperatureMsgData[5] = (uint16_t)getECAPTempC();

        TemperatureMsgData[6] = (uint16_t)getCaseTemp()>>8; //CASE-Temp
        TemperatureMsgData[7] = (uint16_t)getCaseTemp();


        CurrentMsgData[0] = (int16_t)getCurrentA()>>8; //A-Current
        CurrentMsgData[1] = (int16_t)getCurrentA();

        CurrentMsgData[2] = (int16_t)getCurrentB()>>8; //B-Current
        CurrentMsgData[3] = (int16_t)getCurrentB();

        CurrentMsgData[4] = (int16_t)getCurrentC()>>8; //C-Current
        CurrentMsgData[5] = (int16_t)getCurrentC();

        CurrentMsgData[6] = (int16_t)getCurrentEXT()>>8; //EXT-Current
        CurrentMsgData[7] = (int16_t)getCurrentEXT();


        VoltageMsgData[0] = (int16_t)getVoltageA()>>8; //Vsense-A
        VoltageMsgData[1] = (int16_t)getVoltageA();

        VoltageMsgData[2] = (int16_t)getVoltageB()>>8; //Vsense-B
        VoltageMsgData[3] = (int16_t)getVoltageB();

        VoltageMsgData[4] = (int16_t)getVoltageC()>>8; //Vsense-C
        VoltageMsgData[5] = (int16_t)getVoltageC();

        VoltageMsgData[6] = (int16_t)getVoltageDC()>>8; //Vsense-DC
        VoltageMsgData[7] = (int16_t)getVoltageDC();

        CAN_sendMessage(CANA_BASE, 3, 8, TemperatureMsgData); //transmit temperature feedback
        CAN_sendMessage(CANA_BASE, 4, 8, CurrentMsgData); //transmit current feedback
        CAN_sendMessage(CANA_BASE, 5, 8, VoltageMsgData); //transmit voltage feedback


        if(FUND_FREQ >500)
        {
            FUND_FREQ = 500;
        }

        if (PSEN1 == 1)
        {
            GD_A_PSEnable();
        }
        if (PSEN1 == 0)
        {
            GD_A_PSDisable();
        }

        if (PSEN2 == 1)
        {
            GD_B_PSEnable();
        }
        if (PSEN2 == 0)
        {
            GD_B_PSDisable();
        }

        if (PSEN3 == 1)
        {
            GD_C_PSEnable();
        }
        if (PSEN3 == 0)
        {
            GD_C_PSDisable();
        }

        if (LEN1 == 1)
        {
            GD_A_LogicEnable();
        }
        if (LEN1 == 0)
        {
            GD_A_LogicDisable();
        }

        if (LEN2 == 1)
        {
            GD_B_LogicEnable();
        }
        if (LEN2 == 0)
        {
            GD_B_LogicDisable();
        }
        if (LEN3 == 1)
        {
            GD_C_LogicEnable();
        }
        if (LEN3 == 0)
        {
            GD_C_LogicDisable();
        }

        if (RESET == 1)
        {
            // Reset gate drivers
            GD_ALL_Reset();

            // Reset Trip-Zone and Interrupt flag
            //TODO
            //
            // To re-enable the OST Interrupt, uncomment the below code:
            //
             EPWM_clearTripZoneFlag(EPWM1_BASE,
                                    (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));
             EPWM_clearTripZoneFlag(EPWM2_BASE,
                                             (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));
             EPWM_clearTripZoneFlag(EPWM3_BASE,
                                             (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));

            FAULT1 = 0;
            FAULT2 = 0;
            FAULT3 = 0;
            RESET = 0;
        }
    }
}

//
// epwm1ISR - ePWM 1 ISR
//
__interrupt void epwm1ISR(void)
{
    //
    // Update the CMPA and CMPB values
    //
    updatePWM(&epwm1Info);
    updatePWM(&epwm2Info);
    updatePWM(&epwm3Info);

    //
    // Clear INT flag for this timer
    //
    EPWM_clearEventTriggerInterruptFlag(EPWM1_BASE);

    //
    // Acknowledge interrupt group
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

//
// epwm1TZISR - ePWM1 TZ ISR
//
__interrupt void epwm1TZISR(void)
{
    //FAULT1 =1; //TZ is global fault so might be wrong channel

    //
    // To re-enable the OST Interrupt, uncomment the below code:
    //
    // EPWM_clearTripZoneFlag(EPWM1_BASE,
    //                        (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));

    //
    // Acknowledge this interrupt to receive more interrupts from group 2
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}

//
// epwm2TZISR - ePWM2 TZ ISR
//
__interrupt void epwm2TZISR(void)
{
    //FAULT2 =1; //TZ is global fault so might be wrong channel

    //
    // To re-enable the OST Interrupt, uncomment the below code:
    //
    // EPWM_clearTripZoneFlag(EPWM2_BASE,
    //                        (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));

    //
    // Acknowledge this interrupt to receive more interrupts from group 2
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}

//
// epwm3TZISR - ePWM3 TZ ISR
//
__interrupt void epwm3TZISR(void)
{

    //FAULT3 =1; //TZ is global fault so might be wrong channel

    //
    // To re-enable the OST Interrupt, uncomment the below code:
    //
    // EPWM_clearTripZoneFlag(EPWM3_BASE,
    //                        (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));

    //
    // Acknowledge this interrupt to receive more interrupts from group 2
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}

//
// initEPWM1 - Configure ePWM1
//
void initEPWM1()
{
    //
    // Set-up TBCLK
    //
    EPwm_TBPRD = 100e6/SWITCHING_FREQ/2; //FS is in kHz SWITCHING_FREQ is in Hz
    EPWM_setTimeBasePeriod(EPWM1_BASE, EPwm_TBPRD);
    EPWM_setPhaseShift(EPWM1_BASE, 0); //if PWM phase shift is desired A = 0, B=PRD*1/3, C=PRD*2/3
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0U);
    EPWM_setSyncOutPulseMode(EPWM1_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO); //module EPWM1 is master

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM1_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                EPwm_TBPRD*0.5); //start with 50%

    //
    // Set up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM1_BASE); //master has no phase shift
    EPWM_setClockPrescaler(EPWM1_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM1_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);


    //
    // Set Deadband Active High Complimentary
    //
    EPWM_setDeadBandControlShadowLoadMode(EPWM1_BASE, EPWM_DB_LOAD_ON_CNTR_ZERO);
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM1_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setRisingEdgeDelayCount(EPWM1_BASE, DEAD_TIME);
    EPWM_setFallingEdgeDelayCount(EPWM1_BASE, DEAD_TIME);
    EPWM_setDeadBandCounterClock(EPWM1_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

    //
    // Enable TZ1 as one shot trip sources
    //
    EPWM_enableTripZoneSignals(EPWM1_BASE, EPWM_TZ_SIGNAL_OSHT1);
    //
    // Action on TZ1 set both outputs low
    //
    EPWM_setTripZoneAction(EPWM1_BASE,
                           EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM1_BASE,
                           EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_LOW);    //
    // Enable TZ interrupt
    //
    EPWM_enableTripZoneInterrupt(EPWM1_BASE, EPWM_TZ_INTERRUPT_OST);

    //
    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    EPWM_setInterruptSource(EPWM1_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM1_BASE);
    EPWM_setInterruptEventCount(EPWM1_BASE, 1U); //interupt every switching cycle for (required for sine math)

    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //
    epwm1Info.epwmModule = EPWM1_BASE;
    epwm1Info.epwmPeriod = EPwm_TBPRD;
    epwm1Info.epwmPwmPhase = 0;
    epwm1Info.epwmDeadTime = DEAD_TIME;
    epwm1Info.epwmRadian = 0;

}

//
// initEPWM2 - Configure ePWM2
//
void initEPWM2()
{
    //
    // Set-up TBCLK
    //
    EPwm_TBPRD = 100e6/SWITCHING_FREQ/2; //FS is in kHz SWITCHING_FREQ is in Hz
    EPWM_setTimeBasePeriod(EPWM2_BASE, EPwm_TBPRD);
    EPWM_setPhaseShift(EPWM2_BASE, 0);//if PWM phase shift is desired A = 0, B=PRD*1/3, C=PRD*2/3
    EPWM_setTimeBaseCounter(EPWM2_BASE, 0U);
    EPWM_setSyncOutPulseMode(EPWM2_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN); //module EPWM2 sync is pass-thru

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM2_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                EPwm_TBPRD*0.5); //start with 50%

    //
    // Set-up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM2_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enablePhaseShiftLoad(EPWM2_BASE);
    EPWM_setClockPrescaler(EPWM2_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set-up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM2_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set Action qualifier
    //
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //
    // Set Deadband Active High Complimentary
    //
    EPWM_setDeadBandControlShadowLoadMode(EPWM2_BASE, EPWM_DB_LOAD_ON_CNTR_ZERO);
    EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM2_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM2_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setRisingEdgeDelayCount(EPWM2_BASE, DEAD_TIME);
    EPWM_setFallingEdgeDelayCount(EPWM2_BASE, DEAD_TIME);
    EPWM_setDeadBandCounterClock(EPWM2_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

    //
    // Enable TZ1 as one shot trip sources
    //
    EPWM_enableTripZoneSignals(EPWM2_BASE, EPWM_TZ_SIGNAL_OSHT1);
    //
    // Action on TZ1 set both outputs low
    //
    EPWM_setTripZoneAction(EPWM2_BASE,
                           EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM2_BASE,
                           EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_LOW);    //
    // Enable TZ interrupt
    //
    EPWM_enableTripZoneInterrupt(EPWM2_BASE, EPWM_TZ_INTERRUPT_OST);
    //
    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    //EPWM_setInterruptSource(EPWM2_BASE, EPWM_INT_TBCTR_ZERO);
   // EPWM_enableInterrupt(EPWM2_BASE);
   // EPWM_setInterruptEventCount(EPWM2_BASE, 3U);

    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //

    epwm2Info.epwmModule = EPWM2_BASE;
    epwm2Info.epwmPeriod = EPwm_TBPRD;
    epwm2Info.epwmPwmPhase = 0;
    epwm2Info.epwmDeadTime = DEAD_TIME;
    epwm2Info.epwmRadian = 2*PI/3;
}

//
// initEPWM3 - Configure ePWM3
//
void initEPWM3()
{
    //
    // Set-up TBCLK
    //
    EPwm_TBPRD = 100e6/SWITCHING_FREQ/2; //FS is in kHz SWITCHING_FREQ is in Hz
    EPWM_setTimeBasePeriod(EPWM3_BASE, EPwm_TBPRD);
    EPWM_setPhaseShift(EPWM3_BASE, 0);//if PWM phase shift is desired A = 0, B=PRD*1/3, C=PRD*2/3
    EPWM_setTimeBaseCounter(EPWM3_BASE, 0U);
    EPWM_setSyncOutPulseMode(EPWM3_BASE, EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN); //module EPWM2 sync is pass-thru

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM3_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                EPwm_TBPRD*0.5); //start with 50%

    //
    // Set-up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM3_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_enablePhaseShiftLoad(EPWM3_BASE);
    EPWM_setClockPrescaler(EPWM3_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM3_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set Action qualifier
    //
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //
    // Set Deadband Active High Complimentary
    //
    EPWM_setDeadBandControlShadowLoadMode(EPWM3_BASE, EPWM_DB_LOAD_ON_CNTR_ZERO);
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_FED, true);
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
    EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    EPWM_setRisingEdgeDeadBandDelayInput(EPWM3_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setFallingEdgeDeadBandDelayInput(EPWM3_BASE, EPWM_DB_INPUT_EPWMA);
    EPWM_setRisingEdgeDelayCount(EPWM3_BASE, DEAD_TIME);
    EPWM_setFallingEdgeDelayCount(EPWM3_BASE, DEAD_TIME);
    EPWM_setDeadBandCounterClock(EPWM3_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);


    //
    // Enable TZ1 as one shot trip sources
    //
    EPWM_enableTripZoneSignals(EPWM3_BASE, EPWM_TZ_SIGNAL_OSHT1);
    //
    // Action on TZ1 set both outputs low
    //
    EPWM_setTripZoneAction(EPWM3_BASE,
                           EPWM_TZ_ACTION_EVENT_TZA,
                           EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM3_BASE,
                           EPWM_TZ_ACTION_EVENT_TZB,
                           EPWM_TZ_ACTION_LOW);    //
    // Enable TZ interrupt
    //
    EPWM_enableTripZoneInterrupt(EPWM3_BASE, EPWM_TZ_INTERRUPT_OST);
    //
    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    //EPWM_setInterruptSource(EPWM3_BASE, EPWM_INT_TBCTR_ZERO);
   // EPWM_enableInterrupt(EPWM3_BASE);
   // EPWM_setInterruptEventCount(EPWM3_BASE, 3U);

    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //
    epwm3Info.epwmModule = EPWM3_BASE;
    epwm3Info.epwmPeriod = EPwm_TBPRD;
    epwm3Info.epwmPwmPhase = 0;
    epwm3Info.epwmDeadTime = DEAD_TIME;
    epwm3Info.epwmRadian = 4*PI/3;
}

//
// updateCompare - Function to update the frequency
//
void updatePWM(epwmInformation *epwmInfo)
{
    //count to the ramp delay
    FUND_FREQ = rampFreq; //changing FUND_FREQ to slow start motor
    if(rampFreq < final_freq){
        //start counting the delay cycles
        delay_count += 1;
        //if delay has reached max, reset timer and load ramp with new value
        if(delay_count >= delay_count_max){
            delay_count = 0;
            rampFreq += 1; //ramp goes up 1Hz
        }
    }//end of ramp delay if statement


    Ts = 2*PI/(SWITCHING_FREQ/FUND_FREQ);
    EPwm_TBPRD = 100e6/SWITCHING_FREQ/2; //FS is in kHz SWITCHING_FREQ is in Hz
    EPWM_setTimeBasePeriod(epwmInfo->epwmModule, EPwm_TBPRD);
    EPWM_setPhaseShift(epwmInfo->epwmModule, EPwm_TBPRD*epwmInfo->epwmPwmPhase);
    epwmInfo->epwmDeadTime = DEAD_TIME;
    EPWM_setRisingEdgeDelayCount(epwmInfo->epwmModule, epwmInfo->epwmDeadTime);
    EPWM_setFallingEdgeDelayCount(epwmInfo->epwmModule, epwmInfo->epwmDeadTime);

    // Sine wave math
    //For 50% duty cycle comment out below two lines
    Sine = ((((MF*sin(epwmInfo->epwmRadian))+1.0))/2);
    epwmInfo->epwmRadian += Ts;

    if(epwmInfo->epwmRadian > 2*PI)
        epwmInfo->epwmRadian -= (2*PI);

    EPWM_setCounterCompareValue(epwmInfo->epwmModule,
                                               EPWM_COUNTER_COMPARE_A,
                                               Sine*EPwm_TBPRD);

//    EPWM_setCounterCompareValue(epwmInfo->epwmModule,
//                                               EPWM_COUNTER_COMPARE_A,
//                                               DUTY*EPwm_TBPRD);

}

void updateLED(LEDepwmInformation *epwmInfo)
{
    uint16_t compAValue;

    compAValue = EPWM_getCounterCompareValue(epwmInfo->epwmModule,
                                             EPWM_COUNTER_COMPARE_A);

    //
    //  Change the CMPA values every 10th interrupt.
    //
    if(epwmInfo->epwmTimerIntCount == 10U)
    {
        epwmInfo->epwmTimerIntCount = 0U;

        //
        // If we were increasing CMPA, check to see if we reached the max
        // value. If not, increase CMPA else, change directions and decrease
        // CMPA
        //
        if(epwmInfo->epwmCompADirection == EPWM_CMP_UP)
        {
            if(compAValue < (epwmInfo->epwmMaxCompA))
            {
                EPWM_setCounterCompareValue(epwmInfo->epwmModule,
                                            EPWM_COUNTER_COMPARE_A,
                                            ++compAValue);
            }
            else
            {
                epwmInfo->epwmCompADirection = EPWM_CMP_DOWN;
                EPWM_setCounterCompareValue(epwmInfo->epwmModule,
                                            EPWM_COUNTER_COMPARE_A,
                                            --compAValue);
            }
        }
        //
        // If we were decreasing CMPA, check to see if we reached the min
        // value. If not, decrease CMPA else, change directions and increase
        // CMPA
        //
        else
        {
            if( compAValue == (epwmInfo->epwmMinCompA))
            {
                epwmInfo->epwmCompADirection = EPWM_CMP_UP;
                EPWM_setCounterCompareValue(epwmInfo->epwmModule,
                                            EPWM_COUNTER_COMPARE_A,
                                            ++compAValue);
            }
            else
            {
                EPWM_setCounterCompareValue(epwmInfo->epwmModule,
                                            EPWM_COUNTER_COMPARE_A,
                                            --compAValue);
            }
        }

    }
    else
    {
        epwmInfo->epwmTimerIntCount++;
    }
}

void CANPacketEncode( uint16_t *PacketData)
{
    // Concatenate data values into 8 byte message (make sure all are Uint32 data types)
    FS = SWITCHING_FREQ/1000;
    ID = (MF * 1000);
    TD = DEAD_TIME*10;
    FF = FUND_FREQ;

    PacketData[0] = (uint16_t)FS;
    PacketData[1] = (uint16_t)(ID>>2);
    PacketData[2] = (uint16_t)(ID<<6 | TD >>6);
    PacketData[3] = (uint16_t)(TD<<2 | FF>>8);
    PacketData[4] = (uint16_t)(FF);
    PacketData[5] = (uint16_t)(PSEN1 << 6 | PSEN2 << 5 | PSEN3 << 4 | LEN1 << 2 | LEN2 << 1  | LEN3 );
    PacketData[6] = (uint16_t)(RESET << 7 | FAULT1 <<6 | FAULT2 <<5 | FAULT3 << 4);
    PacketData[7] = 0;

}

void CANPacketDecode(uint16_t *PacketData)
{
    FS = (PacketData[0])&0x000000FF;
    ID = ((PacketData[1]<<2) | (PacketData[2]>>6))&0x000003FF;
    TD = (((PacketData[2]&0x3F)<<6) | (PacketData[3]>>2))&0x00000FFF;
    FF = (((PacketData[3]&0x03)<<8) | PacketData[4])&0x000003FF;
    PSEN1 = (PacketData[5] & 0x40)>>6;
    PSEN2 = (PacketData[5] & 0x20)>>5;
    PSEN3 = (PacketData[5] & 0x10)>>4;
    LEN1 = (PacketData[5] & 0x04)>>2;
    LEN2 = (PacketData[5] & 0x02)>>1;
    LEN3 = PacketData[5] & 0x01;
    FAULT1 = (PacketData[6] & 0x40)>>6;
    FAULT2 = (PacketData[6] & 0x20)>>5;
    FAULT3 = (PacketData[6] & 0x10)>>4;
    RESET = (PacketData[6] & 0x80)>>7;

    SWITCHING_FREQ = (FS) * 1000;
    MF = (ID) / 1000.0;
    DEAD_TIME = (TD) / 10;
    FUND_FREQ = FF;
    if(FUND_FREQ >500)
    {
        FUND_FREQ = 500;
    }

    if (PSEN1 == 1)
    {
        GD_A_PSEnable();
    }
    if (PSEN1 == 0)
    {
        GD_A_PSDisable();
    }

    if (PSEN2 == 1)
    {
        GD_B_PSEnable();
    }
    if (PSEN2 == 0)
    {
        GD_B_PSDisable();
    }

    if (PSEN3 == 1)
    {
        GD_C_PSEnable();
    }
    if (PSEN3 == 0)
    {
        GD_C_PSDisable();
    }

    if (LEN1 == 1)
    {
        GD_A_LogicEnable();
    }
    if (LEN1 == 0)
    {
        GD_A_LogicDisable();
    }

    if (LEN2 == 1)
    {
        GD_B_LogicEnable();
    }
    if (LEN2 == 0)
    {
        GD_B_LogicDisable();
    }
    if (LEN3 == 1)
    {
        GD_C_LogicEnable();
    }
    if (LEN3 == 0)
    {
        GD_C_LogicDisable();
    }

    if (RESET == 1)
    {
        // Reset gate drivers
        GD_ALL_Reset();

        // Reset Trip-Zone and Interrupt flag
        //TODO
        //
        // To re-enable the OST Interrupt, uncomment the below code:
        //
         EPWM_clearTripZoneFlag(EPWM1_BASE,
                                (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));
         EPWM_clearTripZoneFlag(EPWM2_BASE,
                                         (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));
         EPWM_clearTripZoneFlag(EPWM3_BASE,
                                         (EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST));

        FAULT1 = 0;
        FAULT2 = 0;
        FAULT3 = 0;
        RESET = 0;
    }
}

//
// epwm6ISR - ePWM 6 ISR
//
__interrupt void epwm6ISR(void)
{
    //
    // Update the CMPA and CMPB values
    //
    updateLED(&epwmLEDInfo);

    //
    // Clear INT flag for this timer
    //
    EPWM_clearEventTriggerInterruptFlag(EPWM6_BASE);

    //
    // Acknowledge interrupt group
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

//
// eCAP 1 ISR
//
__interrupt void ecap1ISR(void)
{
    //
    // Get the capture counts.
    //
    cap1Count[cap1index] = ECAP_getEventTimeStamp(ECAP1_BASE, ECAP_EVENT_2);
    //cap1Count = (cap1Count + ECAP_getEventTimeStamp(ECAP1_BASE, ECAP_EVENT_3)) / 2; //average over 3 periods
    //cap1Count = (cap1Count = ECAP_getEventTimeStamp(ECAP1_BASE, ECAP_EVENT_4)) / 2;

    //
    // Clear interrupt flags for more interrupts.
    //
    ECAP_clearInterrupt(ECAP1_BASE,ECAP_ISR_SOURCE_CAPTURE_EVENT_4);
    ECAP_clearGlobalInterrupt(ECAP1_BASE);

    //
    // Start eCAP
    //
    ECAP_reArm(ECAP1_BASE);

    //
    // Acknowledge the group interrupt for more interrupts.
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP4);

    if(cap1index > capCountsize)
    {
        cap1index = 0;
    }else
    {
        cap1index++;
    }
}

//
// eCAP 2 ISR
//
__interrupt void ecap2ISR(void)
{
    //
    // Get the capture counts.
    //
    cap2Count[cap2index] = ECAP_getEventTimeStamp(ECAP2_BASE, ECAP_EVENT_2);
    //cap2Count = (cap2Count + ECAP_getEventTimeStamp(ECAP2_BASE, ECAP_EVENT_3)) / 2;
    //cap2Count = (cap2Count + ECAP_getEventTimeStamp(ECAP2_BASE, ECAP_EVENT_4)) / 2; //average over 3 periods

    //
    // Clear interrupt flags for more interrupts.
    //
    ECAP_clearInterrupt(ECAP2_BASE,ECAP_ISR_SOURCE_CAPTURE_EVENT_4);
    ECAP_clearGlobalInterrupt(ECAP2_BASE);

    //
    // Start eCAP
    //
    ECAP_reArm(ECAP2_BASE);

    //
    // Acknowledge the group interrupt for more interrupts.
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP4);

    if(cap2index > capCountsize)
    {
        cap2index = 0;
    }else
    {
        cap2index++;
    }
}

//
// eCAP 3 ISR
//
__interrupt void ecap3ISR(void)
{
    //
    // Get the capture counts.
    //
    cap3Count[cap3index] = ECAP_getEventTimeStamp(ECAP3_BASE, ECAP_EVENT_2);
    //cap3Count = (cap3Count + ECAP_getEventTimeStamp(ECAP3_BASE, ECAP_EVENT_3)) / 2;
    //cap3Count = (cap3Count + ECAP_getEventTimeStamp(ECAP3_BASE, ECAP_EVENT_4)) / 2; //average over 3 periods

    //
    // Clear interrupt flags for more interrupts.
    //
    ECAP_clearInterrupt(ECAP3_BASE,ECAP_ISR_SOURCE_CAPTURE_EVENT_4);
    ECAP_clearGlobalInterrupt(ECAP3_BASE);

    //
    // Start eCAP
    //
    ECAP_reArm(ECAP3_BASE);

    //
    // Acknowledge the group interrupt for more interrupts.
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP4);

    if(cap3index > capCountsize)
    {
        cap3index = 0;
    }else
    {
        cap3index++;
    }
}


//
// TestLEDS - cycle led pins to test functionality
//
void TestLEDPulse(){
    Interrupt_register(INT_EPWM6, &epwm6ISR);
    initCaseLEDPWM();
    Interrupt_enable(INT_EPWM6);

}//TestLEDPulse

//
// initCaseLEDPWM
//
void initCaseLEDPWM()
{
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);    // Enable pullup on GPIO10
    GPIO_setPinConfig(GPIO_10_EPWM6A);              // GPIO10 = EPWM6A
    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM6_BASE, EPWM6_TIMER_TBPRD);
    EPWM_setPhaseShift(EPWM6_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM6_BASE, 0U);

    //
    // Set Compare values
    //
    EPWM_setCounterCompareValue(EPWM6_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                1000U);


    //
    // Set up counter mode
    //
    EPWM_setTimeBaseCounterMode(EPWM6_BASE, EPWM_COUNTER_MODE_UP);
    EPWM_disablePhaseShiftLoad(EPWM6_BASE);
    EPWM_setClockPrescaler(EPWM6_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM6_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM6_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM6_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);


    //
    // Interrupt where we will change the Compare Values
    // Select INT on Time base counter zero event,
    // Enable INT, generate INT on 3rd event
    //
    EPWM_setInterruptSource(EPWM6_BASE, EPWM_INT_TBCTR_ZERO);
    EPWM_enableInterrupt(EPWM6_BASE);
    EPWM_setInterruptEventCount(EPWM6_BASE, 6U);

    //
    // Information this example uses to keep track of the direction the
    // CMPA/CMPB values are moving, the min and max allowed values and
    // a pointer to the correct ePWM registers
    //
    epwmLEDInfo.epwmCompADirection = EPWM_CMP_UP;
    epwmLEDInfo.epwmTimerIntCount = 0U;
    epwmLEDInfo.epwmModule = EPWM6_BASE;
    epwmLEDInfo.epwmMaxCompA = EPWM6_MAX_CMPA;
    epwmLEDInfo.epwmMinCompA = EPWM6_MIN_CMPA;
}

//
// TestLEDS - cycle led pins to test functionality
//
void TestLEDS(){

    GPIO_togglePin(10);
    GPIO_togglePin(31);
    GPIO_togglePin(34);
    GPIO_togglePin(41);
    GPIO_togglePin(52);
    GPIO_togglePin(65);

    DEVICE_DELAY_US(200000);
}//TestLEDS


/*
124 SHUTDOWN+15V
125 SHUTDOWN-15V
*/
void enableNeg15V()
{
    GPIO_writePin(125,0);
}
void enablePos15V()
{
    GPIO_writePin(124,0);
}
void disableNeg15V()
{
    GPIO_writePin(125,1);
}
void disablePos15V()
{
    GPIO_writePin(124,1);
}

//
// initGPIO - this initializes the GPIO for controller not in external files.
//
void initGPIO(void)
{
    //
    // LEDS
    //
    // Enable GPIO outputs on GPIO10,31,34,41,52,65, set it LOW
    /*
    10  CASE-LED
    31  LP-LED-BLUE
    34  LP-LED-RED
    41  LED-G
    52  LED-Y
    65  LED-R
    */
    //CASELED1 is PWM to do fade
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO10
    GPIO_setPinConfig(GPIO_10_EPWM6A);              // GPIO10 = EPWM6A


    GPIO_setPadConfig(31, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO31
    GPIO_writePin(31, 0);                           // Output low
    GPIO_setPinConfig(GPIO_31_GPIO31);              // GPIO31 = GPIO31
    GPIO_setDirectionMode(31, GPIO_DIR_MODE_OUT);   // GPIO31 = output

    GPIO_setPadConfig(34, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO34
    GPIO_writePin(34, 0);                           // Output low
    GPIO_setPinConfig(GPIO_34_GPIO34);              // GPIO34 = GPIO34
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);   // GPIO34 = output

    GPIO_setPadConfig(41, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO41
    GPIO_writePin(41, 0);                           // Output low
    GPIO_setPinConfig(GPIO_41_GPIO41);              // GPIO41 = GPIO41
    GPIO_setDirectionMode(41, GPIO_DIR_MODE_OUT);   // GPIO41 = output

    GPIO_setPadConfig(52, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO52
    GPIO_writePin(52, 0);                           // Output Low
    GPIO_setPinConfig(GPIO_52_GPIO52);              // GPIO52 = GPIO52
    GPIO_setDirectionMode(52, GPIO_DIR_MODE_OUT);   // GPIO52 = output

    GPIO_setPadConfig(65, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO65
    GPIO_writePin(65, 0);                           // Output Low
    GPIO_setPinConfig(GPIO_65_GPIO65);              // GPIO65 = GPIO65
    GPIO_setDirectionMode(65, GPIO_DIR_MODE_OUT);   // GPIO65 = output

    // PS-CONTROL +15V and -15V
    //
    // Enable GPIO outputs on GPIO124,125, set it LOW
    /*
    124 SHUTDOWN+15V
    125 SHUTDOWN-15V
    */
    GPIO_setPadConfig(124, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO124
    GPIO_writePin(124, 0);                           // Output Low
    GPIO_setPinConfig(GPIO_124_GPIO124);              // GPIO124 = GPIO124
    GPIO_setDirectionMode(124, GPIO_DIR_MODE_OUT);   // GPIO124 = output

    GPIO_setPadConfig(125, GPIO_PIN_TYPE_STD);    // Enable push-pull on GPIO125
    GPIO_writePin(125, 0);                           // Output Low
    GPIO_setPinConfig(GPIO_125_GPIO125);              // GPIO125 = GPIO125
    GPIO_setDirectionMode(125, GPIO_DIR_MODE_OUT);   // GPIO125 = output

    //
    // Enable EQEP1 on GPIO's 20,21,99
    /*
    20  QEA_A
    21  QEA_B
    99  QEA_I
    */
    GPIO_setPadConfig(20, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO20
    GPIO_setPadConfig(21, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO21
    GPIO_setPadConfig(99, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO23
    GPIO_setQualificationMode(20, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setQualificationMode(21, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setQualificationMode(99, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setPinConfig(GPIO_20_EQEP1A);              // GPIO20 = EQEP1A
    GPIO_setPinConfig(GPIO_21_EQEP1B);              // GPIO21 = EQEP1B
    GPIO_setPinConfig(GPIO_99_EQEP1I);              // GPIO99 = EQEP1I

    //
    // Enable EQEP2 on GPIO's 54,55,57
    /*
    54  QEB_A
    55  QEB_B
    57  QEB_I
    */
    GPIO_setPadConfig(54, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO54
    GPIO_setPadConfig(55, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO55
    GPIO_setPadConfig(57, GPIO_PIN_TYPE_PULLUP);    // Enable pullup on GPIO57
    GPIO_setQualificationMode(54, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setQualificationMode(55, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setQualificationMode(57, GPIO_QUAL_SYNC);  // Synch to SYSCLKOUT
    GPIO_setPinConfig(GPIO_54_EQEP2A);              // GPIO54 = EQEP1A
    GPIO_setPinConfig(GPIO_55_EQEP2B);              // GPIO55 = EQEP1B
    GPIO_setPinConfig(GPIO_57_EQEP2I);              // GPIO57 = EQEP1I


}
