/*
 * Lab 3 code
 *
 *  Created on: Sep 19, 2024
 *      Author: yjia67
 *
 */

#include "ti_msp_dl_config.h"
#include "Lab3.h"

// declare global variables to keep track of S2
volatile int s2_counter = 0;
volatile uint8_t s2_state = 0; // 0 = OFF, 1 = ON

int main(void)
{

    SYSCFG_DL_init();
    init_GPIO();

    // main loop
    while (1) {

        // check S1
        if (DL_GPIO_readPins(S1) > 0) {
            DL_GPIO_clearPins(LED1); // turn on LED1
        } else {
            DL_GPIO_setPins(LED1); // turn off LED1
        }

        // check S2
        if (DL_GPIO_readPins(S2) == 0) {
            DL_Common_delayCycles(delay_num_cycles);
            if (DL_GPIO_readPins(S2) == 0) {
                s2_state = 1; // update S2 state
                select_LED(s2_counter);
            }
        } else {
            DL_GPIO_clearPins(LED2_R);
            DL_GPIO_clearPins(LED2_G);
            DL_GPIO_clearPins(LED2_B);
            DL_Common_delayCycles(delay_num_cycles); // delay incrementing the counter to account for debouncing
            if (s2_state == 1) {
                s2_counter++; // only increment the counter if s2 transitions from being ON to OFF
                s2_state = 0;
            }
        }
    }
}

// choose which LED to blink based on the count
void select_LED(int count) {
    switch (count%3) {
        case 0:
            DL_GPIO_setPins(LED2_R); break;
        case 1:
            DL_GPIO_setPins(LED2_G); break;
        case 2:
            DL_GPIO_setPins(LED2_B); break;
    }
}

// initialize pins
void init_GPIO(void) {
    // Set LEDs at output pins
    DL_GPIO_initDigitalOutput(IOMUX_PINCM1);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM57);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM58);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM50);

    // Set their initial state to off
    // LED1 is connected to pull up resistor so logic high = OFF
    DL_GPIO_setPins(LED1);
    DL_GPIO_clearPins(LED2_R);
    DL_GPIO_clearPins(LED2_G);
    DL_GPIO_clearPins(LED2_B);

    // Enable output for all the pins
    DL_GPIO_enableOutput(LED1);
    DL_GPIO_enableOutput(LED2_R);
    DL_GPIO_enableOutput(LED2_G);
    DL_GPIO_enableOutput(LED2_B);

    // Set switch 1 (S1) and switch 2 (S2) as input buttons
    // S1 is connected to a PD resistor - logic high = ON
    // S2 is connected to a PU resistor - logic low = ON
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM40, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
                                     DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM49, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
                                     DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}
