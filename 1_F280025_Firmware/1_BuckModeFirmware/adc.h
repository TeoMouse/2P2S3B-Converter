
#include "f28x_project.h"
#include "math.h"

#ifndef ADC_H_
#define ADC_H_

void ConfigureADC(void);

void SetupADC(void);

/* Execute the functions to get ADC values from RAM */
#pragma CODE_SECTION(GetV_LS, ".TI.ramfunc");
#pragma CODE_SECTION(GetV_HS, ".TI.ramfunc");
#pragma CODE_SECTION(GetIph1, ".TI.ramfunc");
#pragma CODE_SECTION(GetIph2, ".TI.ramfunc");


float GetV_LS(void);

float GetV_HS(void);

float GetIph1(void);

float GetIph2(void);


#pragma CODE_SECTION(GetTemp1, ".TI.ramfunc");
#pragma CODE_SECTION(GetTemp2, ".TI.ramfunc");

float GetTemp1(void);

float GetTemp2(void);

/* Execute the functions to manipulate arrays from RAM */
#pragma CODE_SECTION(shift, ".TI.ramfunc");
#pragma CODE_SECTION(CLR_Array, ".TI.ramfunc");

void shift(float *array, int length);

void CLR_Array(float *array, int length);

/* 2nd Order Filter Section */
void secfilt(float *in, float *y, float *a,float *b, float g);


#endif
