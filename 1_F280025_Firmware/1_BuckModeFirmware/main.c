#include "f28x_project.h"
#include "pwm.h"
#include "adc.h"
#include "control.h"
#include "math.h"
#include "sfo_v8.h"

#define VIN_CONV_ON     (46)    // Allow converter to operate if Vin > VIN_CONV_ON
#define VIN_CONV_OFF    (40)    // Disable converter if Vin < VIN_CONV_OFF
#define VI_OVP          (51)    // Input voltage for operation after the OVP is triggered
#define VI_OVP_OFF      (52)    // Maximum input voltage for normal operation
#define MAX_TEMP_TH_OFF (105)   // Maximum allowable operating Temp
#define MAX_TEMP_TH_ON  (80)    // Temp on which the temperature fault resets
#define DUTY_MAX        (0.4f)  // Maximum allowable duty cycle
#define V_REF           (12.0f) // Output voltage reference
#define VO_OVP          (14.f)  // Maximum Vout for OVP
#define VO_UVL          (10)    // Minimum Vout for UVLO at steady state
#define IPH_OCP_H       (25)    // Maximum current per phase for OCP (High OCP limit)
#define IPH_OCP_L       (22)    // Maximum current per phase for OCP (Low OCP limit)
#define MAX_IREF        (45)    // Maximum reference current value from the outer voltage loop
#define MIN_IREF        (-1)    // Minimum reference current value from the outer voltage loop

#define OVP_MAX_FAULTS  (5)     // Maximum number of retries if OVP fault occurs
#define OVP_RETRY_CNT   (20)    // After OVP fault retry every OVP_Cnt_R ms

#define UVP_MAX_FAULTS  (8)     // Maximum number of retries if UVP fault occurs
#define UVP_RETRY_CNT   (20)    // After OVP fault retry every UVP_Cnt_R ms

#define D_STEP_RAMP     (0.001f)    // Duty cycle step during start up
#define SS_UP_CNT_VALUE (3)     // Duty cycle during start up increases every 3 cycles

#define TIMER0_CNT      (1000)  //timer 0 1000us count
#define TIMER1_CNT      (20)    //timer 1 20us count
#define MEP_CAL_CNT     (1000)  //Calibrate MEP every 1000 cycles x 1 ms
#define LED_CNT_C       (500)  //LED counter resets every 500 x 1 ms

#define PWM_CH            3     // # of PWM channels + 1
volatile struct EPWM_REGS *ePWM[PWM_CH] = {0, &EPwm1Regs, &EPwm2Regs};  //Struct for high res PWM channels

int MEP_ScaleFactor; // Global variable used by the SFO library
                     // Result can be used for all HRPWM channels
                     // This variable is also copied to HRMSTEP
                     // register by SFO() function.


/* Load and execute interrupt function from RAM */
#pragma CODE_SECTION(adcA1ISR, ".TI.ramfunc");
#pragma CODE_SECTION(adcC1ISR, ".TI.ramfunc");

__interrupt void adcA1ISR(void);
__interrupt void adcC1ISR(void);

void error(void);

uint16_t duty_t = 1000, cnt=0, status_HR;
float duty = 0.26;

/* -- PI voltage controller definitions (Fs = 50 kHz) -- */
float kp = 7.f, ki = 3000.f;
float Ts = 0.00002;

float dpi_Vout[2] = {0,0}, epi_Vout[2]={0,0}; //PI output -> dpi, PI input -> epi

/* -- PI current controller definitions (Fs = 50 kHz) -- */
float kp_I = 0.02f, ki_I = 3.f;

float dpi_Iph1[2] = {0,0}, epi_Iph1[2]={0,0}; //PI output -> dpi, PI input -> epi
float dpi_Iph2[2] = {0,0}, epi_Iph2[2]={0,0}; //PI output -> dpi, PI input -> epi

/* -- LOW PASS FILTER with Fc @ ~0.13 kHz definitions (Fs = 1kHz) -- */

float af[3] = {1, -0.4493, 0}, bf[3] = {0, 0.5507, 0};
float Temp1_x[3] = {0, 0, 0}, Temp1_y[3] = {0, 0 ,0};
float Temp2_x[3] = {0, 0, 0}, Temp2_y[3] = {0, 0 ,0};


void main(void){

    //Initialize System Control:
    //PLL, WatchDog,enable Peripheral Clocks


InitSysCtrl();

    
    //Initialize GPIO

    InitGpio();

    //Clear all __interrupts and initialize PIE vector table:
    //Disable CPU __interrupts  
    DINT;

    //Initialize the PIE control registers to their default state.
    InitPieCtrl();

    //Disable CPU interrupts and clear all CPU interrupt flags: 
    IER = 0x0000;
    IFR = 0x0000;

    //Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).
    InitPieVectTable();

    // Enable PIE interrupt
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx3 = 1;


    EALLOW;
    PieVectTable.ADCA1_INT = &adcA1ISR;     // Function for ADCA interrupt
    PieVectTable.ADCC1_INT = &adcC1ISR;     // Function for ADCC interrupt
    EDIS;


    ConfigureADC();
    ePWM_ADC_Trig();
    SetupADC();

    InitLEDGPIOs();
    

    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 100, TIMER0_CNT); // Configure counter to count TIMER0_CNT us
    CpuTimer0Regs.TCR.all = 0x4000; //Clear timer0 flag

    ConfigCpuTimer(&CpuTimer1, 100, TIMER1_CNT); // Configure counter to count TIMER1_CNT us
    CpuTimer1Regs.TCR.all = 0x4000; //Clear timer0 flag

    // Enable global Interrupts and higher priority real-time debug events:
    IER |= M_INT1;  // Enable group 1 interrupts

    EINT; // Enable Global interrupt INTM
    ERTM; // Enable Global realtime interrupt DBGM

    while(status_HR == SFO_INCOMPLETE){

        status_HR = SFO();
         if(status_HR == SFO_ERROR){

             error();   // SFO function returns 2 if an error occurs & # of MEP
         }              // steps/coarse step exceeds maximum of 255.
    }

    DELAY_US(10000); // Delay 10 ms

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    InitEPwm1Gpio();
    initEPWM1();

    InitEPwm2Gpio();
    initEPWM2();

    InitEPwm5Gpio();
    initEPWM5();


    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;


     while(1);
}

float V_LowSide, V_HighSide, Iout_Ph1, Iout_Ph2;
uint16_t select, Ph1_Status=1,Ph2_Status=1;

float Ph1_d = 0.25, Ph2_d = 0.25, D_total, D_Ex_SS, D_Thresh_SS;

float dpi_Vout_Sat, D_Iph1_Sat, D_Iph2_Sat;

uint16_t CL_Latch = 1, Conv_EN = 1, Conv_SS = 0, StartUP_cnt = 0, Ph_Status_L[2] = {1, 1}, UpdateStatus;
uint16_t OVP_Cnt, OCP_Cnt_Ph1_H, OCP_Cnt_Ph2_H, Latch_Fault_OVP = 0, OCP_Cnt_Ph1_L, OCP_Cnt_Ph2_L;
uint16_t I_prot, Vprot_OVP, TempProt, NoRetries, TempLatch_Fault, OVP_Faults;
uint16_t UVP_Cnt, Latch_Fault, UVP_Cnt, Vprot_UVP, UVP_Faults, HardLatch_UVP, UVP_Cnt_R, UVL_Cnt;

__interrupt void adcA1ISR(void){


    V_LowSide = GetV_LS(); //Get the High side voltage measurement

    /* if Vout > threshold for 3 consecutive measurements then converter off */
    if (V_LowSide > VO_OVP) OVP_Cnt++;
    else OVP_Cnt = 0;

    if (OVP_Cnt > 2) {
        ConverterOFF();
        Latch_Fault = 1;
        Vprot_OVP = 1;
        OVP_Faults++;
        Conv_EN = 0;
    }

    /* if Vout < threshold for 10 consecutive measurements during steady state operation then converter off */
    if ((V_LowSide < VO_UVL) && (Conv_SS == 1)) UVL_Cnt++;
    else if ((V_LowSide > VO_UVL) && (Conv_SS == 1)) UVL_Cnt = 0;

    if (UVL_Cnt > 9) {
        ConverterOFF();
        Latch_Fault = 1;
        Vprot_UVP = 1;
        UVP_Faults++;
        Conv_EN = 0;
    }

    V_HighSide = GetV_HS(); //Get the High side voltage measurement

    Iout_Ph1 = GetIph1(); //Get the Ph1 current measurement

    /* if Iout_Ph1 > threshold for 2 consecutive measurements then converter off */
    if(Iout_Ph1 > IPH_OCP_H) OCP_Cnt_Ph1_H++;
    else OCP_Cnt_Ph1_H = 0;

    if(Iout_Ph1 > IPH_OCP_L) OCP_Cnt_Ph1_L++;
    else OCP_Cnt_Ph1_L = 0;


    if (OCP_Cnt_Ph1_H > 1 || OCP_Cnt_Ph1_L > 14 ){
        ConverterOFF();
        Latch_Fault = 1;
        I_prot = 1;
        Conv_EN = 0;
    }

    Iout_Ph2 = GetIph2(); //Get the Ph1 current measurement

    /* if Iout_Ph1 > threshold for 2 consecutive measurements then converter off */
    if(Iout_Ph2 > IPH_OCP_H) OCP_Cnt_Ph2_H++;
    else OCP_Cnt_Ph2_H = 0;

    if(Iout_Ph2 > IPH_OCP_L) OCP_Cnt_Ph2_L++;
    else OCP_Cnt_Ph2_L = 0;

    if (OCP_Cnt_Ph2_H > 1 || OCP_Cnt_Ph2_L > 14){
        ConverterOFF();
        Latch_Fault = 1;
        I_prot = 1;
        Conv_EN = 0;
    }


    if (Conv_EN == 1 && Latch_Fault == 0){
        /* If Converter Enable  == 1 and there is no fault*/

        switch(Conv_SS){
                case 0:
                    StartUP_cnt++;
                    D_Thresh_SS = V_REF/V_HighSide;
                    D_Thresh_SS = saturator(0.2f, 0.3f, D_Thresh_SS);
                    if(StartUP_cnt>SS_UP_CNT_VALUE){

                        StartUP_cnt = 0;

                        D_total = D_total + D_STEP_RAMP;

                        /* Update duty cycle per phase 
                        During start up all 2 phases are active */

                        DutyPh1(D_total,1);
                        DutyPh2(D_total,1);

                        select = TrigPointSel(D_total); //Calculate the trigger point for the ADC

                        /* Assign trigger point per ADC channel */
                        Ph1_ADC_Trig_Point(D_total, select);
                        Ph2_ADC_Trig_Point(D_total, select);

                    }
                    /* if duty cycle ramp during start up reaches D_THRESH_SS, then switch to steady state operation closed loop operation */
                    if (D_total> D_Thresh_SS || (V_LowSide > V_REF) ){
                        StartUP_cnt = 0;
                        Conv_SS = 1;    //Start up is over and Steady State operation is established

                        /* Assign initial values at the PI controllers to eliminate harsh transients during the transition */

                        dpi_Vout[0] = Iout_Ph1;
                        dpi_Vout[1] = Iout_Ph1;
                        D_Ex_SS = 0.0065f * Iout_Ph1;
                        D_total = D_Ex_SS + D_total;
                        dpi_Iph1[0] = D_total;
                        dpi_Iph1[1] = D_total;
                        dpi_Iph2[0] = D_total;
                        dpi_Iph2[1] = D_total;
                        D_total = 0;

                    }
                    break;
                case 1:

                    switch(CL_Latch){
                        case 1:
                            /* Set each phase status of the converter */
                            UpdateStatus = Ph1_En_Dis(Ph1_Status, Ph_Status_L[0]);
                            Ph_Status_L[0] = Ph1_Status*UpdateStatus + (1-UpdateStatus)*Ph_Status_L[0] ;
                            UpdateStatus = Ph2_En_Dis(Ph2_Status, Ph_Status_L[1]);
                            Ph_Status_L[1] = Ph2_Status*UpdateStatus + (1-UpdateStatus)*Ph_Status_L[1] ;

                            epi_Vout[0] = V_REF - V_LowSide;
                            /*PI controller function and output saturation*/
                            PIcontroller(dpi_Vout, epi_Vout, kp, ki, Ts);
                            dpi_Vout_Sat = 0.5*saturator(MIN_IREF, MAX_IREF, dpi_Vout[0]);



                            epi_Iph1[0] = dpi_Vout_Sat - Iout_Ph1;
                            PIcontroller(dpi_Iph1, epi_Iph1, kp_I, ki_I, Ts);
                            D_Iph1_Sat = saturator(0, DUTY_MAX, dpi_Iph1[0]);

                            epi_Iph2[0] = dpi_Vout_Sat - Iout_Ph2;
                            PIcontroller(dpi_Iph2, epi_Iph2, kp_I, ki_I, Ts);
                            D_Iph2_Sat = saturator(0, DUTY_MAX, dpi_Iph2[0]);



                            /* Update duty cycle per phase */
                            DutyPh1(D_Iph1_Sat,Ph1_Status);
                            DutyPh2(D_Iph2_Sat,Ph2_Status);

                            /* Calculate the trigger point for the ADC */
                            select = TrigPointSel(0.5*(D_Iph1_Sat+D_Iph2_Sat));

                            /* Assign trigger point per ADC channel */
                            Ph1_ADC_Trig_Point(D_Iph1_Sat, select);
                            Ph2_ADC_Trig_Point(D_Iph2_Sat, select);

                            /* Shift array values for the PI controllers */
                            shift(dpi_Vout,2);
                            shift(epi_Vout,2);

                            shift(dpi_Iph1,2);
                            shift(epi_Iph1,2);

                            shift(dpi_Iph2,2);
                            shift(epi_Iph2,2);

                            break;

                        case 0:

                            /* Update duty cycle per phase */
                            DutyPh1(Ph1_d,Ph1_Status);
                            DutyPh2(Ph2_d,Ph2_Status);

                            /* Calculate the trigger point for the ADC */
                            select = TrigPointSel(Ph1_d); 

                            /* Assign trigger point per ADC channel */
                            Ph1_ADC_Trig_Point(Ph1_d, select);
                            Ph2_ADC_Trig_Point(Ph2_d, select);

                            break;

                }

                break;

            }
    }
    else{
        /* If Converter Enable  == 0 OR there is a fault*/

        /* Update duty cycle per phase and set it equal to 0*/
        DutyPh1(0,Ph1_Status);
        DutyPh2(0,Ph2_Status);

        Ph_Status_L[0] = 1;
        Ph_Status_L[1] = 1;

        /* Clear arrays and initialize values for start up */
        StartUP_cnt = 0;
        Conv_SS = 0;
        CLR_Array(dpi_Vout,2);
        CLR_Array(epi_Vout,2);
        CLR_Array(dpi_Iph1,2);
        CLR_Array(epi_Iph1,2);
        CLR_Array(dpi_Iph2,2);
        CLR_Array(epi_Iph2,2);
        D_total = 0;
        OVP_Cnt = 0;
        UVL_Cnt = 0;
        OCP_Cnt_Ph1_H = 0;
        OCP_Cnt_Ph2_H = 0;
        OCP_Cnt_Ph1_L = 0;
        OCP_Cnt_Ph2_L = 0;


    }

    // Acknowledge the interrupt
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

float Temp1, Temp2, MaxTemp;
uint16_t MEP_Cnt, CLR_Fault, OVP_Cnt_R, HardLatch_OVP;
uint16_t LED_CNT, TOG1_CNT, TOG2_CNT;
uint16_t Vin_OVP, Vin_UVP;

__interrupt void adcC1ISR(void){

    /* Check for minimum and maximum operating voltage limits */
    if ((V_HighSide > VIN_CONV_ON) && (V_HighSide < VI_OVP) && (Latch_Fault == 0)) {
        
        Conv_EN  = 1;
        Vin_OVP = 0;
        Vin_UVP = 0;

        if (Conv_SS == 0){
            /* Clear tripzone flag for each phase */
            EALLOW;
            EPwm1Regs.TZCLR.bit.OST = 1; //CLR TZ status for Phase 1
            EPwm2Regs.TZCLR.bit.OST = 1; //CLR TZ status for Phase 2

            /* Trigger ADC conversions according to PWM */
            AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //SOC0 conversion trigger -> ePWM1
            AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 7; //SOC1 conversion trigger -> ePWM2
            AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 7; //SOC2 conversion trigger -> ePWM2
            AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 7; //SOC3 conversion trigger -> ePWM2
            EDIS;
        }
    }

    if (V_HighSide < VIN_CONV_OFF){
        Conv_EN =  0;
        ConverterOFF();
        Vin_UVP = 1;
    }

    if (V_HighSide > VI_OVP_OFF){
        Conv_EN =  0;
        ConverterOFF();
        Vin_OVP = 1;
    }

    /* Get temperature values */
    Temp1_x[0] = GetTemp1();
    Temp2_x[0] = GetTemp2();
    secfilt(Temp1_x , Temp1_y, af, bf, 1);
    secfilt(Temp2_x , Temp2_y, af, bf, 1);
    Temp1 = Temp1_y[0];
    Temp2 = Temp2_y[0];

     /* Shift the arrays for Temp filters */
    shift(Temp1_x,3);
    shift(Temp1_y,3);
    shift(Temp2_x,3);
    shift(Temp2_y,3);

    MaxTemp = 0;
    if (Temp1 > MaxTemp) MaxTemp = Temp1;
    if (Temp2 > MaxTemp) MaxTemp = Temp2;

    /* Check for over temperature protection*/
    if (MaxTemp > MAX_TEMP_TH_OFF){
        ConverterOFF();
        Latch_Fault = 1;
        TempProt = 1;
    }
    if (TempProt == 1 && MaxTemp < MAX_TEMP_TH_ON) CLR_Fault = 1;

    /* Control the fan duty cycle*/
    if (MaxTemp < 45) FanDuty(0);
    else if (MaxTemp < 80) FanDuty(0.0214f*MaxTemp-0.7143f);
    else FanDuty(1);

    if (CLR_Fault == 1 || V_HighSide < VIN_CONV_OFF){
        Latch_Fault = 0;
        CLR_Fault = 0;
        OVP_Cnt_R = 0;
        UVP_Cnt_R = 0;
        HardLatch_OVP = 0;
        HardLatch_UVP = 0;

        Vprot_UVP = 0;
        Vprot_OVP = 0;
        I_prot = 0;
        TempProt = 0;

    }

    if (Latch_Fault == 1 && Vprot_OVP == 1){
        OVP_Cnt_R ++;
        if (OVP_Faults < OVP_MAX_FAULTS && OVP_Cnt_R > OVP_RETRY_CNT && HardLatch_OVP == 0){
            CLR_Fault = 1;
        }
        else if (OVP_Faults >= OVP_MAX_FAULTS){
            OVP_Faults = 0;
            OVP_Cnt_R = 0;
            HardLatch_OVP = 1;
        }
    }

    if (Latch_Fault == 1 && Vprot_UVP == 1){
        UVP_Cnt_R ++;
        if (UVP_Faults < UVP_MAX_FAULTS && UVP_Cnt_R > UVP_RETRY_CNT && HardLatch_UVP == 0){
            CLR_Fault = 1;
        }
        else if (UVP_Faults >= UVP_MAX_FAULTS){
            UVP_Faults = 0;
            UVP_Cnt_R = 0;
            HardLatch_UVP = 1;
        }
    }


    //Run Calibration module for MEP every MEP_CAL_CNT cycles x 1ms
    MEP_Cnt++;
    if (MEP_Cnt> MEP_CAL_CNT){

        MEP_Cnt = 0;

        // Call the scale factor optimizer lib function SFO()
        // periodically to track for any change due to temp/voltage.
        // This function generates MEP_ScaleFactor by running the
        // MEP calibration module in the HRPWM logic. This scale
        // factor can be used for all HRPWM channels. The SFO()
        // function also updates the HRMSTEP register with the
        // scale factor value.


        status_HR = SFO(); // in background, MEP calibration module
                           // continuously updates MEP_ScaleFactor

        if(status_HR == SFO_ERROR){

         error();   // SFO function returns 2 if an error occurs & #
                    // of MEP steps/coarse step
        }

    }

    // LED control
    if (Conv_EN == 1){
        LED_GREEN_ON();
        LED_RED_OFF();
    }
    else if (TempProt == 1){
        LED_GREEN_OFF();
        LED_RED_ON();
    }
    else if (I_prot == 1){

        // Increase and reset LED counter
        LED_CNT++;
        if (LED_CNT > LED_CNT_C){
            LED_CNT = 0;
            TOG2_CNT++;
        }
        if (TOG2_CNT > 0) {
            LED_Green_Toggle();
            LED_Red_Toggle();
            TOG2_CNT = 0;
        }

    }
    else if (Vprot_UVP == 1){
        LED_GREEN_OFF();
        // Increase and reset LED counter
        LED_CNT++;
        if (LED_CNT > LED_CNT_C){
            LED_CNT = 0;
            TOG2_CNT++;
        }
        if (TOG2_CNT > 1) {
            LED_Red_Toggle();
            TOG2_CNT = 0;
        }
    }
    else if (Vprot_OVP == 1){
        LED_GREEN_OFF();
        // Increase and reset LED counter
        LED_CNT++;
        if (LED_CNT > LED_CNT_C){
            LED_CNT = 0;
            TOG2_CNT++;
        }
        if (TOG2_CNT > 0) {
            LED_Red_Toggle();
            TOG2_CNT = 0;
        }
    }
    else if (Vin_OVP == 1){
        LED_RED_OFF();
        // Increase and reset LED counter
        LED_CNT++;
        if (LED_CNT > LED_CNT_C){
            LED_CNT = 0;
            TOG2_CNT++;
        }
        if (TOG2_CNT > 0) {
            LED_Green_Toggle();
            TOG2_CNT = 0;
        }
    }
    else if (Vin_UVP == 1){
        LED_RED_OFF();
        // Increase and reset LED counter
        LED_CNT++;
        if (LED_CNT > LED_CNT_C){
            LED_CNT = 0;
            TOG2_CNT++;
        }
        if (TOG2_CNT > 1) {
            LED_Green_Toggle();
            TOG2_CNT = 0;
        }
    }

    // Acknowledge the interrupt
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


void error (void){
	
    ESTOP0;         // Stop here and handle error
}