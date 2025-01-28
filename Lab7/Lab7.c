/*
 * Final Project for AE6705
 *
 *  Created on: Oct 22, 2024
 *      Author: yjia67
 *
 */

#include <Lab7.h>
#include "ti_msp_dl_config.h"
#include "stdio.h"

DL_TimerG_ClockConfig gTIMER_0ClockConfig = {
                                            .clockSel = DL_TIMER_CLOCK_LFCLK, // 32 kHz
                                            .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
                                            .prescale = 0 // 32 kHz
};

DL_TimerG_TimerConfig gTIMER_0TimerConfig = {
                                             .period = 1600,
                                             .timerMode = DL_TIMER_TIMER_MODE_PERIODIC,
                                             .startTimer = DL_TIMER_STOP
};

volatile uint8_t mode = 0; // 0: OFF, 1: CCW, 2: CW
volatile uint8_t cur_pin = 0; // ranging from 0-3 for indexing pin sequence array
volatile uint32_t ccw[] = {A1_PIN, B1_PIN, A2_PIN, B2_PIN}; // pin / coil activation sequence for CCW rotation
volatile uint32_t cw[] = {B2_PIN, A2_PIN, B1_PIN, A1_PIN}; // pin / coil activation sequence for CW rotation
volatile uint32_t counter = 0;

int main(void)
{

    // initialization stuff
    SYSCFG_DL_init();
    GPIO_init();
    TimerG_init();

    // configure interrupts
    NVIC_ClearPendingIRQ(TIMG0_INT_IRQn);
    NVIC_EnableIRQ(TIMG0_INT_IRQn);

    // start timer
    DL_TimerG_startCounter(TIMG0);

    // main loop
    while (1) {

        if (DL_GPIO_readPins(S1) > 0) {
            mode = 1;
            DL_GPIO_setPins(LED2_B);

        } else if (DL_GPIO_readPins(S2) == 0) {
            mode = 2;
            DL_GPIO_setPins(LED2_R);

        } else {
            mode = 0;
            DL_GPIO_clearPins(LED2_B);
            DL_GPIO_clearPins(LED2_R);
        }

    }

    return 0;

}

void TIMG0_IRQHandler(void)
{
    DL_GPIO_clearPins(M_PORT, A1_PIN | B1_PIN | A2_PIN | B2_PIN);
    cur_pin = counter%4;
    switch (mode) {
        case 1: // CCW rotation
            DL_GPIO_setPins(M_PORT, ccw[cur_pin]);
            counter++;
        case 2: // CW rotation
            DL_GPIO_setPins(M_PORT, cw[cur_pin]);
            counter++;
    }
}

// configure timer
void TimerG_init(void)
{
    DL_TimerG_enablePower(TIMG0);
    DL_TimerG_setClockConfig(TIMG0, &gTIMER_0ClockConfig);
    DL_TimerG_initTimerMode(TIMG0, &gTIMER_0TimerConfig);
    DL_TimerG_enableInterrupt(TIMG0, DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(TIMG0);
}

// configure LED for debug purposes
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

    // Output pins for driving motor
    DL_GPIO_initDigitalOutput(IOMUX_PINCM25);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM24);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM23);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM12);

    DL_GPIO_clearPins(M_PORT, A1_PIN | B1_PIN | A2_PIN | B2_PIN);
    DL_GPIO_enableOutput(M_PORT, A1_PIN | B1_PIN | A2_PIN | B2_PIN);

    // switches: S1 - PD resistor -> logic high = ON; S2 - PU resistor -> logic low = ON
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM40, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
                                     DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM49, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
                                     DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}
