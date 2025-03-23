#include "adc.h"
#define SAMPLE_WIN 50
#define SAMPLE_WIN2 100

/* 2nd order appr. for ADC value in the range [50, 19]*/
#define TEMP_C12        (0.0188f)
#define TEMP_C11        (-2.4593f)
#define TEMP_C10        (170.15f)

/* 2nd order appr. for ADC value in the range [380, 51]*/
#define TEMP_C22        (0.0006f)
#define TEMP_C21        (-0.4422f)
#define TEMP_C20        (109.01f)

/* Linear appr. for ADC value in the range [381, 1810]*/
#define TEMP_C31        (-0.0429f)
#define TEMP_C30        (36.766f)


void ConfigureADC(void){

    EALLOW;

    AnalogSubsysRegs.ANAREFCTL.bit.ANAREFASEL = 1; // Reference voltage of the ADCA is set external
    AnalogSubsysRegs.ANAREFCTL.bit.ANAREFCSEL = 1; // Reference voltage of the ADCC is set external


    AdcaRegs.ADCCTL2.bit.PRESCALE = 0; //set ADCCLK divider to /1 (for ADC A)

    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; //interrupt pulse generation occurs at the end of the conversion
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1; //power up ADCA
    DELAY_US(1000); //delay for 1ms to allow ADC time to power up

    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK  divider to /4 (for ADC C)

    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1; //interrupt pulse generation occurs at the end of the conversion
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1; //power up ADCC
    EDIS;

    DELAY_US(1000); //delay for 1ms to allow ADC time to power up


}

void SetupADC(void){

    EALLOW;

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;  //SOC0 will convert pin A0  /*Iph1_Meas*/
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = SAMPLE_WIN; //sample window is SAMPLE_WIN+1 sysclocks
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //SOC0 conversion trigger -> ePWM1

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;  //SOC1 will convert pin A1 /*Iph2_Meas*/
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = SAMPLE_WIN; //sample window is SAMPLE_WIN+1 sysclocks
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 7; //SOC1 conversion trigger -> ePWM2

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2;  //SOC2 will convert pin A2 /*VLowSide_Meas*/
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = SAMPLE_WIN; //sample window is SAMPLE_WIN+1 sysclocks
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 7; //SOC2 conversion trigger -> ePWM2

    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 8;  //SOC3 will convert pin A8 /*VHighSide_Meas*/
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = SAMPLE_WIN; //sample window is SAMPLE_WIN+1 sysclocks
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 7; //SOC3 conversion trigger -> ePWM2

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 3; //EOC3 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADC_C Temperature measurements

    AdccRegs.ADCSOC0CTL.bit.CHSEL = 1;  //SOC0 will convert pin C1 /*Temp1*/
    AdccRegs.ADCSOC0CTL.bit.ACQPS = SAMPLE_WIN2; //sample window is SAMPLE_WIN2+1 sysclocks
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 1; //SOC0 conversion trigger -> Timer0

    AdccRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert pin C3 /*Temp2*/
    AdccRegs.ADCSOC1CTL.bit.ACQPS = SAMPLE_WIN2; //sample window is SAMPLE_WIN2+1 sysclocks
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 1; //SOC1 conversion trigger -> Timer0

    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //EOC3 will set INT1 flag
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared


    EDIS;
}

/* Execute the functions to get ADC values from RAM */
#pragma CODE_SECTION(GetV_LS, ".TI.ramfunc");
#pragma CODE_SECTION(GetV_HS, ".TI.ramfunc");
#pragma CODE_SECTION(GetIph1, ".TI.ramfunc");
#pragma CODE_SECTION(GetIph2, ".TI.ramfunc");


float GetV_LS(void){
    float Val = 0;

    Val = 0.00667f * AdcaResultRegs.ADCRESULT2; /*Range: [0 V, 27 V]*/

    return Val;
}

float GetV_HS(void){
    float Val = 0;

    Val = 0.01638f * AdcaResultRegs.ADCRESULT3; /*Range: [0 V, 67 V]*/

    return Val;
}

float GetIph1(void){
    float Val = 0;

    Val = 0.01465f * AdcaResultRegs.ADCRESULT0 - 30.f; /*Range: [-30 A, 30 A]*/

    return Val;

}

float GetIph2(void){
    float Val = 0;

    Val = 0.01465f * AdcaResultRegs.ADCRESULT1 - 30.f; /*Range: [-30 A, 30 A]*/

    return Val;

}


#pragma CODE_SECTION(GetTemp1, ".TI.ramfunc");
#pragma CODE_SECTION(GetTemp2, ".TI.ramfunc");

float GetTemp1(void){

    float Val = 0;
    uint16_t ADC_Res = AdccResultRegs.ADCRESULT0;

    if (ADC_Res > 380){
        Val =  TEMP_C31*ADC_Res+ TEMP_C30;
    }
    else if (ADC_Res > 50){
        Val = TEMP_C22*ADC_Res*ADC_Res + TEMP_C21*ADC_Res+ TEMP_C20;
    }
    else{
        Val = TEMP_C12*ADC_Res*ADC_Res + TEMP_C11*ADC_Res+ TEMP_C10;
    }

    return Val;
}

float GetTemp2(void){

    float Val = 0;
    uint16_t ADC_Res = AdccResultRegs.ADCRESULT1;

    if (ADC_Res > 380){
            Val =  TEMP_C31*ADC_Res+ TEMP_C30;
        }
        else if (ADC_Res > 50){
            Val = TEMP_C22*ADC_Res*ADC_Res + TEMP_C21*ADC_Res+ TEMP_C20;
        }
        else{
            Val = TEMP_C12*ADC_Res*ADC_Res + TEMP_C11*ADC_Res+ TEMP_C10;
        }

    return Val;
}


/* Execute the functions to manipulate arrays from RAM */
#pragma CODE_SECTION(shift, ".TI.ramfunc");
#pragma CODE_SECTION(CLR_Array, ".TI.ramfunc");

void shift(float *array, int length){

    int i;
    for (i = length;i > 1;i--) array[i-1] = array[i-2];

    return;
}


void CLR_Array(float *array, int length){

    int i;
    for (i = 0;i < length;i++) array[i] = 0;
}

/* 2nd Order Filter Section */
void secfilt(float *in, float *y, float *a,float *b, float g){

    y[0] = b[0] * in[0]*g;
    y[0] += b[1] * in[1]*g;
    y[0] += b[2] * in[2]*g;
    y[0] -= a[1] * y[1];
    y[0] -= a[2] * y[2];

}
