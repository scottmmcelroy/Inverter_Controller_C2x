/*
 * GATEDRIVER.h
 *
 *  Created on: Feb 22, 2019
 *      Author: mfeurtado
 */

#ifndef GATEDRIVER_H_
#define GATEDRIVER_H_
#include "device.h"

#define XM3GDV2
//#define XM3GDV1

void initGateDriverGPIO(void);

//phase A gate driver control
void GD_A_PSEnable(void);
void GD_A_PSDisable(void);
void GD_A_LogicEnable(void);
void GD_A_LogicDisable(void);
void GD_A_OCEnable(void);
void GD_A_OCDisable(void);

//phase B gate driver control
void GD_B_PSEnable(void);
void GD_B_PSDisable(void);
void GD_B_LogicEnable(void);
void GD_B_LogicDisable(void);
void GD_B_OCEnable(void);
void GD_B_OCDisable(void);

//phase C gate driver control
void GD_C_PSEnable(void);
void GD_C_PSDisable(void);
void GD_C_LogicEnable(void);
void GD_C_LogicDisable(void);
void GD_C_OCEnable(void);
void GD_C_OCDisable(void);

//ALL gate driver control
void GD_ALL_PSEnable(void);
void GD_ALL_PSDisable(void);
void GD_ALL_LogicEnable(void);
void GD_ALL_LogicDisable(void);
void GD_ALL_OCEnable(void);
void GD_ALL_OCDisable(void);
void GD_ALL_Reset(void);

//read Fault state from gate driver
//return true if there is a fault
//return false if there is NO fault
bool GD_A_getFault(void);
bool GD_B_getFault(void);
bool GD_C_getFault(void);
bool GD_Global_getFault(void);

#endif /* GATEDRIVER_H_ */
