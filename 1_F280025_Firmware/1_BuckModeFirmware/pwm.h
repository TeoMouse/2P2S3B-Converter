#include "f28x_project.h"
#include "math.h"

#ifndef PWM_H_
#define PWM_H_

void ePWM_ADC_Trig(void);


void initEPWM1(void);
void initEPWM2(void);
void initEPWM5(void);

/* Execute the function to modify duty cycle from RAM */
#pragma CODE_SECTION(DutyPh1, ".TI.ramfunc");
#pragma CODE_SECTION(DutyPh2, ".TI.ramfunc");


void DutyPh1(float duty, uint16_t status);
void DutyPh2(float duty, uint16_t status);

void FanDuty(float duty);

uint16_t TrigPointSel (float duty);

/* Execute the function to calculate ADC trigger point from RAM */
#pragma CODE_SECTION(Ph1_ADC_Trig_Point, ".TI.ramfunc");
#pragma CODE_SECTION(Ph2_ADC_Trig_Point, ".TI.ramfunc");


void Ph1_ADC_Trig_Point(float duty, uint16_t select);
void Ph2_ADC_Trig_Point(float duty, uint16_t select);

/* Execute the function to EN/DIS each phase from RAM */
#pragma CODE_SECTION(Ph1_En_Dis, ".TI.ramfunc");
#pragma CODE_SECTION(Ph2_En_Dis, ".TI.ramfunc");


uint16_t Ph1_En_Dis(uint16_t status, uint16_t Last_status);
uint16_t Ph2_En_Dis(uint16_t status, uint16_t Last_status);


#endif
