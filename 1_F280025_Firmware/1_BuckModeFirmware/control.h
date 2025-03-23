#include "f28x_project.h"
#include "math.h"

#ifndef CONTROL_H_
#define CONTROL_H_

void InitLEDGPIOs(void);

void LED_RED_ON(void);

void LED_RED_OFF(void);

void LED_GREEN_ON(void);

void LED_GREEN_OFF(void);

void LED_Green_Toggle();

void LED_Red_Toggle();

/* implementation of digitalPI controller */
#pragma CODE_SECTION(PIcontroller, ".TI.ramfunc");

void PIcontroller(float *OutputArray, float *ErrorArray, float kp, float ki, float Ts);

/* saturate input value at min and maximum values*/
#pragma CODE_SECTION(saturator, ".TI.ramfunc");

float saturator(float min, float max, float value);

/* Return 1 if Temp_x is greaterthan MaxTemp, else return 0 */
uint16_t OverTemp(float Temp1, float Temp2, uint16_t MaxTemp);

#pragma CODE_SECTION(ConverterOFF, ".TI.ramfunc");

void ConverterOFF(void);

#endif
