#include "control.h"

void InitLEDGPIOs(void){
    EALLOW;

    // GPIO12 is LED1
    GpioCtrlRegs.GPAGMUX1.bit.GPIO12 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0; // Pin MUXING: Configure GPIO12 as GPIO
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1; // Disable Pull-Up
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1; //GPIO12 is output

    // GPIO13 is LED2
    GpioCtrlRegs.GPAGMUX1.bit.GPIO13 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0; // Pin MUXING: Configure GPIO13 as GPIO
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 1; // Disable Pull-Up
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 1; //GPIO13 is output



    EDIS;

}

void LED_RED_ON(void){
    GpioDataRegs.GPASET.bit.GPIO12 = 1;
}

void LED_RED_OFF(void){
    GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;
}

void LED_GREEN_ON(void){
    GpioDataRegs.GPASET.bit.GPIO13 = 1;
}

void LED_GREEN_OFF(void){
    GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;
}

void LED_Green_Toggle(){


    if (GpioDataRegs.GPADAT.bit.GPIO13 == 1) LED_GREEN_OFF();
    else LED_GREEN_ON();


}

void LED_Red_Toggle(){

    if (GpioDataRegs.GPADAT.bit.GPIO12 == 1) LED_RED_OFF();
    else LED_RED_ON();


}

/* Execute PI controller function from RAM */
#pragma CODE_SECTION(PIcontroller, ".TI.ramfunc");

void PIcontroller(float *OutputArray, float *ErrorArray, float kp, float ki, float Ts){

    OutputArray[0] = (kp+ki*Ts)*ErrorArray[0] -kp*ErrorArray[1] + OutputArray[1];
}

/* Execute saturation function from RAM */
#pragma CODE_SECTION(saturator, ".TI.ramfunc");

float saturator(float min, float max, float value){

    if (value >= max) value = max;
    if (value <= min) value = min;

    return value;
}

uint16_t OverTemp(float Temp1, float Temp2, uint16_t MaxTemp){

    if (Temp1>MaxTemp || Temp2>MaxTemp){

        return 1;
    }
    else {

        return 0;
    }

}

#pragma CODE_SECTION(ConverterOFF, ".TI.ramfunc");

void ConverterOFF(void){

    EALLOW;
    EPwm1Regs.TZFRC.bit.OST = 1; //Force TZ generation for Phase 2
    EPwm2Regs.TZFRC.bit.OST = 1; //Force TZ generation for Phase 1


    /* Trigger Iph1, Iph2, Vin and Vout measurement from timer 1 */
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 2;
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 2;
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 2;
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 2;


    EDIS;
}


