/*
 * Lab_3_example_code.c
 *
 */

#include "ti_msp_dl_config.h"

int main(void)
{

    SYSCFG_DL_init();

    // Set LEDs at output pins and set their initial state to off
    DL_GPIO_initDigitalOutput(IOMUX_PINCM1);
    //... others as well

    DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_0);      // LED1 is wired so that logic high is off
    //...others as well

    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_0) ;
    //...others as well

    // Set switch 1 (S1) as input button
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM40, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN, DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE) ;


    // Infinite loop
    while (1) {

        if (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_18) > 0) {
            // Do stuff...

        }
    }

}
