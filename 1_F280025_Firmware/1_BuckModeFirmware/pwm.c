#include "pwm.h"

#define PWM_PERIOD      (1000) // fs = 150 kHz
#define FAN_PWM_PER     (5000) //fs = 20 kHz
#define DUTY_C          (2500)
#define DT_ON           (17)
#define DT_OFF          (15)

void ePWM_ADC_Trig(void){

    EPwm1Regs.ETSEL.bit.SOCAEN = 1;    // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 4;   // Enable event time-base counter equal to CMPA when the timer is incrementing or CMPC when the timer is incrementing
    EPwm1Regs.ETSEL.bit.SOCASELCMP = 1; // Enable event time-base counter equal to CMPC when the timer is incrementing
    EPwm1Regs.ETPS.bit.SOCAPRD = 2;       // Generate pulse on 2nd event

    EPwm2Regs.ETSEL.bit.SOCAEN = 1;    // Enable SOC on A group
    EPwm2Regs.ETSEL.bit.SOCASEL = 4;   // Enable event time-base counter equal to CMPA when the timer is incrementing or CMPC when the timer is incrementing
    EPwm2Regs.ETSEL.bit.SOCASELCMP = 1; // Enable event time-base counter equal to CMPC when the timer is incrementing
    EPwm2Regs.ETPS.bit.SOCAPRD = 2;       // Generate pulse on 2nd event

}


void initEPWM1(void){

    EALLOW;
    EPwm1Regs.TZSEL.bit.OSHT1 = 1; //Enable TZ1 as a one-shot trip source for this ePWM module
    EPwm1Regs.TZCTL.bit.TZA = 2; //Force EPWMxA to a low state
    EPwm1Regs.TZCTL.bit.TZB = 2; //Force EPWMxB to a low state
    EDIS;


    EPwm1Regs.TBCTR = 0; // Clear counter
    EPwm1Regs.TBPRD = PWM_PERIOD;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0; // HighSpeedTimeBaseClock Prescaler = /1
    EPwm1Regs.TBCTL.bit.CLKDIV = 0; // TimeBaseClock Prescaler = /1
    EPwm1Regs.TBPHS.bit.TBPHS = 0; // Phase is 0


    EPwm1Regs.CMPA.bit.CMPA = DUTY_C; // Set compare A value
    EPwm1Regs.CMPC= 750; // Set compare A value
    EPwm1Regs.CMPB.bit.CMPB = 0; // Set Compare B value

    EPwm1Regs.TBCTL.bit.CTRMODE = 0; // Up count mode
    EPwm1Regs.TBCTL.bit.PHSEN = 0; // Enable phase loading
    //EPwm1Regs.TBCTL.bit.SYNCOSEL = 1; // Sync output select: CTR=zero
    EPwm1Regs.TBCTL.bit.SWFSYNC = 1; // Force 1 time sync pulse to be generated


    EPwm1Regs.AQCTLA.bit.CAU = 1; // when CMPA == TBCTR on count up force output LOW
    EPwm1Regs.AQCTLA.bit.ZRO = 2; // when PRD == TBCTR force output HIGH

    EPwm1Regs.AQCTLB.bit.CAU = 1; // when CMPA == TBCTR on count up force output LOW
    EPwm1Regs.AQCTLB.bit.ZRO = 2; // when PRD == TBCTR force output HIGH


    EPwm1Regs.DBCTL.bit.OUT_MODE = 3; // Dead band is fully enabled
    EPwm1Regs.DBCTL.bit.POLSEL = 2; // EPWMxB is inverted
    EPwm1Regs.DBRED.bit.DBRED = DT_ON; // 0.3us deadtime
    EPwm1Regs.DBFED.bit.DBFED = DT_OFF; // 0.3us deadtime

    EALLOW;

    EPwm1Regs.HRCNFG.all = 0x0;
    EPwm1Regs.HRCNFG.bit.EDGMODE = 2; //MEP control on falling edge
    //EPwm1Regs.HRCNFG.bit.EDGMODEB = 1; //MEP control on rising edge
    EPwm1Regs.HRCNFG.bit.CTLMODE = 0; //CMPAHR(8) or TBPRDHR(8) Register controls the edge position
    EPwm1Regs.HRCNFG.bit.HRLOAD = 0; //Load on CTR = Zero
    EPwm1Regs.HRCNFG.bit.AUTOCONV = 1; //Automatic HRMSTEP scaling is enabled


    EPwm1Regs.HRCNFG2.bit.EDGMODEDB = 3;
    //EPwm1Regs.HRPCTL.bit.HRPE = 1;

    EDIS;

}



void initEPWM2(void){

    EALLOW;
    EPwm2Regs.TZSEL.bit.OSHT1 = 1; //Enable TZ1 as a one-shot trip source for this ePWM module
    EPwm2Regs.TZCTL.bit.TZA = 2; //Force EPWMxA to a low state
    EPwm2Regs.TZCTL.bit.TZB = 2; //Force EPWMxB to a low state
    EDIS;

    EPwm2Regs.TBCTR = 0; // Clear counter
    EPwm2Regs.TBPRD = PWM_PERIOD;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0; // HighSpeedTimeBaseClock Prescaler = /1
    EPwm2Regs.TBCTL.bit.CLKDIV = 0; // TimeBaseClock Prescaler = /1
    EPwm2Regs.TBPHS.bit.TBPHS = PWM_PERIOD/2; // Phase is 180


    EPwm2Regs.CMPA.bit.CMPA = DUTY_C; // Set compare A value
    EPwm2Regs.CMPB.bit.CMPB = 0; // Set Compare B value

    EPwm2Regs.TBCTL.bit.CTRMODE = 0; // Up count mode
    EPwm2Regs.TBCTL.bit.PHSEN = 1; // Enable phase loading
    //EPwm1Regs.TBCTL.bit.SYNCOSEL = 1; // Sync output select: CTR=zero
    EPwm2Regs.TBCTL.bit.SWFSYNC = 1; // Force 1 time sync pulse to be generated


    EPwm2Regs.AQCTLA.bit.CAU = 1; // when CMPA == TBCTR on count up force output LOW
    EPwm2Regs.AQCTLA.bit.ZRO = 2; // when PRD == TBCTR force output HIGH

    EPwm2Regs.AQCTLB.bit.CAU = 1; // when CMPA == TBCTR on count up force output LOW
    EPwm2Regs.AQCTLB.bit.ZRO = 2; // when PRD == TBCTR force output HIGH


    EPwm2Regs.DBCTL.bit.OUT_MODE = 3; // Dead band is fully enabled
    EPwm2Regs.DBCTL.bit.POLSEL = 2; // EPWMxB is inverted
    EPwm2Regs.DBRED.bit.DBRED = DT_ON; // 0.3us deadtime
    EPwm2Regs.DBFED.bit.DBFED = DT_OFF; // 0.3us deadtime

    EALLOW;

    EPwm2Regs.HRCNFG.all = 0x0;
    EPwm2Regs.HRCNFG.bit.EDGMODE = 2; //MEP control on falling edge
    //EPwm2Regs.HRCNFG.bit.EDGMODEB = 1; //MEP control on rising edge
    EPwm2Regs.HRCNFG.bit.CTLMODE = 0; //CMPAHR(8) or TBPRDHR(8) Register controls the edge position
    EPwm2Regs.HRCNFG.bit.HRLOAD = 0; //Load on CTR = Zero
    EPwm2Regs.HRCNFG.bit.AUTOCONV = 1; //Automatic HRMSTEP scaling is enabled


    EPwm2Regs.HRCNFG2.bit.EDGMODEDB = 3;
    //EPwm2Regs.HRPCTL.bit.HRPE = 1;

    EDIS;

}

void initEPWM5(void){


    EPwm5Regs.TBCTR = 0; // Clear counter
    EPwm5Regs.TBPRD = FAN_PWM_PER;
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = 0; // HighSpeedTimeBaseClock Prescaler = /1
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // TimeBaseClock Prescaler = /1
    EPwm5Regs.TBPHS.bit.TBPHS = 0; // Phase is 0


    EPwm5Regs.CMPA.bit.CMPA = DUTY_C; // Set compare A value
    EPwm5Regs.CMPB.bit.CMPB = 0; // Set Compare B value

    EPwm5Regs.TBCTL.bit.CTRMODE = 0; // Up count mode
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Enable phase loading
    //EPwm1Regs.TBCTL.bit.SYNCOSEL = 1; // Sync output select: CTR=zero
    EPwm5Regs.TBCTL.bit.SWFSYNC = 1; // Force 1 time sync pulse to be generated


    EPwm5Regs.AQCTLA.bit.CAU = 1; // when CMPA == TBCTR on count up force output LOW
    EPwm5Regs.AQCTLA.bit.ZRO = 2; // when PRD == TBCTR force output HIGH


    EPwm5Regs.DBCTL.bit.OUT_MODE = 0; // Dead band is disabled

}

#pragma CODE_SECTION(DutyPh1, ".TI.ramfunc");
#pragma CODE_SECTION(DutyPh2, ".TI.ramfunc");


void DutyPh1(float duty, uint16_t status){
    Uint32 temp;
    Uint16 CMPA_reg_val, CMPAHR_reg_val;
    int duty_Q15;

    switch(status){
        case 1:
            duty_Q15 = duty*32767;
            if (duty_Q15>32767) duty_Q15 = 32767;
            if (duty_Q15<-32768) duty_Q15 = -32768;

            CMPA_reg_val = ((long)duty_Q15 * (PWM_PERIOD + 1)) >> 15;
            temp = ((long)duty_Q15 * (PWM_PERIOD + 1)) ;
            temp = temp - ((long)CMPA_reg_val << 15);
            CMPAHR_reg_val = temp << 1; // convert to Q16

            EPwm1Regs.CMPA.all = ((long)CMPA_reg_val) << 16 | CMPAHR_reg_val;

            break;
        case 0:
            EPwm1Regs.CMPA.all = 0;
            break;
    }
    
}

void DutyPh2(float duty, uint16_t status){
    Uint32 temp;
    Uint16 CMPA_reg_val, CMPAHR_reg_val;
    int duty_Q15;

    switch(status){
        case 1:
            duty_Q15 = duty*32767;
            if (duty_Q15>32767) duty_Q15 = 32767;
            if (duty_Q15<-32768) duty_Q15 = -32768;

            CMPA_reg_val = ((long)duty_Q15 * (PWM_PERIOD + 1)) >> 15;
            temp = ((long)duty_Q15 * (PWM_PERIOD + 1)) ;
            temp = temp - ((long)CMPA_reg_val << 15);
            CMPAHR_reg_val = temp << 1; // convert to Q16

            EPwm2Regs.CMPA.all = ((long)CMPA_reg_val) << 16 | CMPAHR_reg_val;

            break;
        case 0:
            EPwm2Regs.CMPA.all = 0;
            break;
    }

}

void FanDuty(float duty){

    EPwm5Regs.CMPA.bit.CMPA = duty*(FAN_PWM_PER+1);
}

uint16_t TrigPointSel (float duty){

    uint16_t sel = 0; //sel= 0: trig @ d/2, sel = 1: trig @ d+(1-d)/2

    if (duty > 0.52f) sel = 0;
    else if (duty < 0.48f) sel = 1;

    return sel;
}

#pragma CODE_SECTION(Ph1_ADC_Trig_Point, ".TI.ramfunc");
#pragma CODE_SECTION(Ph2_ADC_Trig_Point, ".TI.ramfunc");


void Ph1_ADC_Trig_Point(float duty, uint16_t select){

    //sel= 0: trig @ d/2, sel = 1: trig @ d+(1-d)/2

    EPwm1Regs.CMPC = (select * (duty + (1-duty)*0.5f) * PWM_PERIOD + (1-select) * (duty*0.5f) * PWM_PERIOD);

}

void Ph2_ADC_Trig_Point(float duty, uint16_t select){

    //sel= 0: trig @ d/2, sel = 1: trig @ d+(1-d)/2

    EPwm2Regs.CMPC = (select * (duty + (1-duty)*0.5f) * PWM_PERIOD + (1-select) * (duty*0.5f) * PWM_PERIOD);

}

#pragma CODE_SECTION(Ph1_En_Dis, ".TI.ramfunc");
#pragma CODE_SECTION(Ph2_En_Dis, ".TI.ramfunc");


uint16_t Ph1_En_Dis(uint16_t status, uint16_t Last_status){

    if (status == Last_status) return 0;

    if (status == 0){

        EALLOW;
        EPwm1Regs.TZFRC.bit.OST = 1; //Force TZ generation for Phase 1

        /* Trigger the Iph1_Meas measurement from timer1 */
        AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 2;
        /* Trigger the VLowSide_Meas measurement from timer1 */
        AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 2;
        /* Trigger the VHighSide_Meas measurement from timer1 */
        AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 2;

        EDIS;
        return 1 ;

    }
    else{

        EALLOW;
        EPwm1Regs.TZCLR.bit.OST = 1; //CLR TZ status for Phase 1
        /* Trigger the Iph1_Meas measurement from ePWM1 */
        AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

        EDIS;
        return 1;
    }

}


uint16_t Ph2_En_Dis(uint16_t status, uint16_t Last_status){

    if (status == Last_status) return 0;

    if (status == 0){


        EALLOW;
        EPwm2Regs.TZFRC.bit.OST = 1; //Force TZ generation for Phase 2
        AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5; //SOC3 conversion trigger -> timer 1
        /* Trigger the VLowSide_Meas measurement from timer 1 */
        AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5;
        /* Trigger the VHighSide_Meas measurement from timer 1 */
        AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 5;
        EDIS;
        return 1;

    }
    else{


        EALLOW;
        EPwm2Regs.TZCLR.bit.OST = 1; //CLR TZ status for Phase 2
        AdcaRegs.ADCSOC1CTL.bit.TRIGSEL= 7; //SOC3 conversion trigger -> ePWM2
        /* Trigger the VLowSide_Meas measurement from ePWM2 */
        AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 7;
        /* Trigger the VHighSide_Meas measurement from ePWM2 */
        AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 7;
        EDIS;
        return 1;
    }

}
