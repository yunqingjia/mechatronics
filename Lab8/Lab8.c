/*
 * Lab 8 for AE6705
 *
 *  Created on: Oct 22, 2024
 *      Author: yjia67
 *
 */

#include <Lab8.h>
#include "ti_msp_dl_config.h"
#include "stdio.h"

DL_TimerG_ClockConfig gPWM_0ClockConfig = {
                                           .clockSel = DL_TIMER_CLOCK_BUSCLK, // 32 MHz
                                           .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
                                           .prescale = 0 // 32 MHz
};

DL_TimerG_PWMConfig gPWM_0Config = {
                                    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
                                    .period = 512000, // 1/32e6 * 512000 = 16 ms per period
                                    .startTimer = DL_TIMER_STOP
};

DL_ADC12_ClockConfig gADC_0ClockConfig = {
                                          .clockSel = DL_ADC12_CLOCK_ULPCLK,
                                          .freqRange = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
                                          .divideRatio = DL_ADC12_CLOCK_DIVIDE_8
};

// flags for tracking buttons and motor modes
volatile uint8_t S1_pressed = 0;
volatile uint8_t S2_pressed = 0;
volatile uint8_t mode = 0; // 0: off, 1: forward, 2: reverse

// PWM setting
volatile uint32_t gPWM_period = 512000;
volatile int duty_cycle = 0;

int main(void)
{

    // initialization stuff
    SYSCFG_DL_init();
    GPIO_init();
    ADC_init();
    TimerG_PWM_init();

    // main loop
    while (1) {
        check_mode();
        operate_motor();
    }

    return 0;

}

// operate the motor depending on its mode
void operate_motor(void)
{
    switch (mode) {
        case 0: // off
            DL_GPIO_clearPins(LED2_R);
            DL_GPIO_clearPins(LED2_B);
            DL_TimerG_setCaptureCompareValue(TIMG12, 0, DL_TIMER_CC_0_INDEX);
            DL_TimerG_setCaptureCompareValue(TIMG12, 0, DL_TIMER_CC_1_INDEX);
            break;
        case 1: // forward
            DL_GPIO_clearPins(LED2_B);
            DL_GPIO_setPins(LED2_R);
//            duty_cycle = get_ADC2PWM();
            duty_cycle = 256000;
            DL_TimerG_setCaptureCompareValue(TIMG12, 0, DL_TIMER_CC_1_INDEX);
            DL_TimerG_setCaptureCompareValue(TIMG12, duty_cycle, DL_TIMER_CC_0_INDEX);
            break;
        case 2: // reverse
            DL_GPIO_clearPins(LED2_R);
            DL_GPIO_setPins(LED2_B);
            duty_cycle = get_ADC2PWM();
            DL_TimerG_setCaptureCompareValue(TIMG12, 0, DL_TIMER_CC_0_INDEX);
            DL_TimerG_setCaptureCompareValue(TIMG12, duty_cycle, DL_TIMER_CC_1_INDEX);
            break;
    }
}

// get ADC input and convert to PWM duty cycle
int get_ADC2PWM(void)
{
    while (DL_ADC12_getStatus(ADC0) == DL_ADC12_STATUS_CONVERSION_ACTIVE) {}
    DL_ADC12_enableConversions(ADC0);
    DL_ADC12_startConversion(ADC0);
    uint16_t adc_in = DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);
    // varying between 0% - 100% duty cycle so multiply by the PWM period
    // powering the potentiometer from 3.3V pin so full range = 0-1.65 instead of 0-3.3
    float perc = (float) adc_in / 1.65 * 3.3 / 4096; // 12-bit ADC max = 4096
    return (int) (perc * gPWM_period);
}


// check what mode the motor should be in
void check_mode(void){

    // polling S1 and toggling between modes 0 and 1 on a falling edge
    if (DL_GPIO_readPins(S1) > 0) {
        S1_pressed = 1;
    } else if (S1_pressed == 1) {
        S1_pressed = 0;
        mode = (mode == 1) ? 0 : 1;
    }

    // polling S2 and toggling between modes 0 and 2 on a falling edge
    if (DL_GPIO_readPins(S2) == 0) {
        S2_pressed = 1;
    } else if (S2_pressed == 1) {
        S2_pressed = 0;
        mode = (mode == 2) ? 0 : 2;
    }

}

// initialize / configure timer
void TimerG_PWM_init(void)
{
    // initialize PWM: PB13 (TIMG12_C0) for PWM1, PA25 (TIMG12_C1) for PWM2
    DL_GPIO_initPeripheralOutputFunction(pwm1_iomux, 4);
    DL_GPIO_enableOutput(pwm1_port, pwm1_pin);
    DL_GPIO_initPeripheralOutputFunction(pwm2_iomux, 4);
    DL_GPIO_enableOutput(pwm2_port, pwm2_pin);

    // configure TIMG
    DL_TimerG_enablePower(TIMG12);
    DL_TimerG_setClockConfig(TIMG12, &gPWM_0ClockConfig);
    DL_TimerG_initPWMMode(TIMG12, &gPWM_0Config);

    DL_TimerG_setCaptureCompareOutCtl(TIMG12,
                                      DL_TIMER_CC_OCTL_INIT_VAL_LOW,
                                      DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
                                      DL_TIMER_CC_OCTL_SRC_FUNCVAL,
                                      DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(TIMG12,
                                          DL_TIMER_CC_OCTL_INIT_VAL_LOW,
                                          DL_TIMER_CC_OCTL_INV_OUT_DISABLED,
                                          DL_TIMER_CC_OCTL_SRC_FUNCVAL,
                                          DL_TIMER_CC_1_INDEX);

    // set initial duty cycle to 0
    DL_TimerG_setCaptureCompareValue(TIMG12, duty_cycle, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(TIMG12, duty_cycle, DL_TIMER_CC_1_INDEX);

    // enable timer clock
    DL_TimerG_enableClock(TIMG12);

    // set CCP direction
    DL_TimerG_setCCPDirection(TIMG12, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);

    // start counter
    DL_TimerG_startCounter(TIMG12);

}

// configure ADC
void ADC_init(void)
{
    DL_ADC12_enablePower(ADC0);
    DL_ADC12_setClockConfig(ADC0, &gADC_0ClockConfig);
    DL_ADC12_configConversionMem(ADC0,
                                 DL_ADC12_MEM_IDX_0,
                                 DL_ADC12_INPUT_CHAN_0,
                                 DL_ADC12_REFERENCE_VOLTAGE_VDDA,
                                 DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
                                 DL_ADC12_AVERAGING_MODE_DISABLED,
                                 DL_ADC12_BURN_OUT_SOURCE_DISABLED,
                                 DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
                                 DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    // set the sample time in # of ADC clock cycles
    DL_ADC12_setSampleTime0(ADC0, 10000);
}

// configure LEDs
void GPIO_init(void)
{

    // LEDs
    DL_GPIO_initDigitalOutput(IOMUX_PINCM1);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM57);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM58);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM50);

    DL_GPIO_setPins(LED1);
    DL_GPIO_clearPins(LED2_R);
    DL_GPIO_clearPins(LED2_G);
    DL_GPIO_clearPins(LED2_B);

    DL_GPIO_enableOutput(LED1);
    DL_GPIO_enableOutput(LED2_R);
    DL_GPIO_enableOutput(LED2_G);
    DL_GPIO_enableOutput(LED2_B);

    // switches: S1 - PD resistor -> logic high = ON; S2 - PU resistor -> logic low = ON
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM40, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
                                     DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM49, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
                                     DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}
