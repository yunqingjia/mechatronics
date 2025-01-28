/*
 * Lab 5 code
 *
 *  Created on: Oct 6, 2024
 *      Author: yjia67
 *
 */

#include <Lab5.h>
#include "ti_msp_dl_config.h"
#include "stdio.h"

DL_TimerG_ClockConfig gTIMER_0ClockConfig = {
                                            .clockSel = DL_TIMER_CLOCK_LFCLK, // 32 KHz
                                            .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
                                            .prescale = 255
};

DL_TimerG_TimerConfig gTIMER_0TimerConfig = {
                                             .period = 125,
                                             .timerMode = DL_TIMER_TIMER_MODE_PERIODIC,
                                             .startTimer = DL_TIMER_STOP
};

DL_ADC12_ClockConfig gADC_0ClockConfig = {
                                          .clockSel = DL_ADC12_CLOCK_ULPCLK,
                                          .freqRange = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
                                          .divideRatio = DL_ADC12_CLOCK_DIVIDE_8
};

int main(void)
{

    // initialization stuff
    SYSCFG_DL_init();
    TimerG_init();
    ADC_init();
    GPIO_init();

    // configure interrupts
    NVIC_ClearPendingIRQ(TIMG0_INT_IRQn);
    NVIC_EnableIRQ(TIMG0_INT_IRQn);

    // start timer
    DL_TimerG_startCounter(TIMG0);

    // main loop
    while (1) {
        __WFI();
    }

}

void TIMG0_IRQHandler(void)
{

    // test to see if timer is working as intended
    DL_GPIO_togglePins(LED1);

    // poll to check if sampling is completed
    while (DL_ADC12_getStatus(ADC0) == DL_ADC12_STATUS_CONVERSION_ACTIVE){};

    // enable conversions
    DL_ADC12_enableConversions(ADC0);

    // trigger conversions
    DL_ADC12_startConversion(ADC0);

    // return the conversion results for a specific memory channel
    uint16_t adc_out =  DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);
//    int adc_out_int = (int) adc_out;
//    printf("Current ADC value: %d\n", adc_out);

    double res = 3.3 / 4096; // resolution in volt/number
    double volt = (double) adc_out * res;
//    printf("Current Voltage: %0.2f\n", volt);

    double temp = volt / 0.022;
    printf("Current Temperature: %0.2f\n", temp);
//    printf("%.2f\n", temp);

    return;
}

// initialize / configure timer
void TimerG_init(void)
{

    // enable power
    DL_TimerG_enablePower(TIMG0);

    // configure timer
    DL_TimerG_setClockConfig(TIMG0, &gTIMER_0ClockConfig);
    DL_TimerG_initTimerMode(TIMG0, &gTIMER_0TimerConfig);
    DL_TimerG_enableInterrupt(TIMG0, DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(TIMG0);

}

// initialize / configure ADC
void ADC_init(void)
{

    // 1. enable power
    DL_ADC12_enablePower(ADC0);

    // 2. set clock config
    DL_ADC12_setClockConfig(ADC0, &gADC_0ClockConfig);

    // 3. configure conversion memory
    DL_ADC12_configConversionMem(ADC0,
                                 DL_ADC12_MEM_IDX_0,
                                 DL_ADC12_INPUT_CHAN_0,
                                 DL_ADC12_REFERENCE_VOLTAGE_VDDA,
                                 DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
                                 DL_ADC12_AVERAGING_MODE_DISABLED,
                                 DL_ADC12_BURN_OUT_SOURCE_DISABLED,
                                 DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
                                 DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

    // 4. set the sample time in # of ADC clock cycles
    // from tech ref manual: conversion duration = 6 clock cycles for our setting
    DL_ADC12_setSampleTime0(ADC0, 10000);

}

// configure LED1 for debug purposes
void GPIO_init(void)
{

    DL_GPIO_initDigitalOutput(IOMUX_PINCM1);
    DL_GPIO_setPins(LED1);
    DL_GPIO_enableOutput(LED1);

}


