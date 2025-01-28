/*
 * Lab 4 code
 *
 *  Created on: Sep 26, 2024
 *      Author: yjia67
 *
 */

#include <Lab4.h>
#include "ti_msp_dl_config.h"

char RX_buffer[200];
uint8_t RX_max_bytes = sizeof(RX_buffer);
uint8_t RX_bytes = 0;
uint8_t TX_bytes = 0;

// global variables for configuration parameters
DL_UART_ClockConfig gUART_0ClockConfig = {
                                               .clockSel = DL_UART_CLOCK_BUSCLK,
                                               .divideRatio = DL_UART_CLOCK_DIVIDE_RATIO_1
};

DL_UART_Config gUART_0Config = {
                                .mode = DL_UART_MODE_NORMAL,
                                .direction = DL_UART_DIRECTION_TX_RX,
                                .flowControl = DL_UART_FLOW_CONTROL_NONE,
                                .parity = DL_UART_PARITY_NONE,
                                .wordLength = DL_UART_WORD_LENGTH_8_BITS,
                                .stopBits = DL_UART_STOP_BITS_ONE
};


int main(void)
{

    SYSCFG_DL_init();

    UART_init();

    NVIC_DisableIRQ( UART0_INT_IRQn );
    NVIC_ClearPendingIRQ( UART0_INT_IRQn );
    NVIC_EnableIRQ( UART0_INT_IRQn );

    // main loop
    while (1) {
        __WFI();
    }

}


// UART initialization
void UART_init(void)
{
    // enable peripheral functions
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM21, IOMUX_PINCM21_PF_UART0_TX);
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM22, IOMUX_PINCM22_PF_UART0_RX);

    // turn power on
    DL_UART_reset(UART0);
    DL_UART_enablePower(UART0);

    // set clock configuration
    DL_UART_setClockConfig(UART0, &gUART_0ClockConfig);

    // initial configuration
    DL_UART_init(UART0, &gUART_0Config);

    // set oversampling rate
    DL_UART_setOversampling(UART0, DL_UART_OVERSAMPLING_RATE_16X);

    // set baud rate to 9600
    int clock_rate = 32e6;
    int baud_rate = 9600;
    int oversample_freq = 16;
    double temp = (double) clock_rate/(oversample_freq*baud_rate);
    int IBRD = (int)temp;
    int FBRD = (int)( 64*(temp-IBRD) + 0.5 );
    DL_UART_setBaudRateDivisor(UART0, IBRD, FBRD);

    // enable interrupts
    DL_UART_enableInterrupt(UART0, DL_UART_INTERRUPT_RX | DL_UART_INTERRUPT_TX);

    // enable module
    DL_UART_enable(UART0);

}

// interrupt request handler
void UART0_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART0)) {
        case DL_UART_IIDX_RX:

            // check to see if receive buffer is full
            if (RX_bytes < RX_max_bytes) {
                RX_buffer[RX_bytes] = DL_UART_receiveDataBlocking(UART0);

                // when a carriage return is received, enable transmit
                if (RX_buffer[RX_bytes] == '\r') {
                    RX_buffer[RX_bytes++] = '\n';
                    RX_buffer[RX_bytes++] = '\r';

                    DL_UART_enableInterrupt(UART0, DL_UART_INTERRUPT_TX);

                } else {
                    RX_bytes++;
                }
            }
            break;

        case DL_UART_IIDX_TX:

            DL_UART_transmitData(UART0, RX_buffer[TX_bytes++]);

            if (TX_bytes == RX_bytes) {
                DL_UART_disableInterrupt(UART0, DL_UART_INTERRUPT_TX);
                TX_bytes = 0;
                RX_bytes = 0;

            }
            break;

        default:
            break;
    }
}

