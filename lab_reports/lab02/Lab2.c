#include "ti_msp_dl_config.h"

volatile short counter = 0 ;

int main(void)
{

    SYSCFG_DL_init();

    DL_GPIO_initDigitalOutput(IOMUX_PINCM50);
    DL_GPIO_initDigitalOutput(IOMUX_PINCM57);

    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_22);
    DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_26 );

    DL_GPIO_enableOutput(GPIOB, DL_GPIO_PIN_22 | DL_GPIO_PIN_26) ;  // Red and blue LEDs

    DL_SYSCTL_enableSleepOnExit();

    // Initializes the SysTick period to 500.00 ms, enables the interrupt, and starts the SysTick Timer
    DL_SYSTICK_config(16000000);

    while (1) {
        __WFI();
    }
}



void SysTick_Handler(void)
{
    int cm ;
    DL_GPIO_togglePins(GPIOB, DL_GPIO_PIN_26 | DL_GPIO_PIN_22) ;
    counter++ ;
    cm = counter*1000 ;
}
