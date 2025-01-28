/*
 * Lab 6 code
 *
 *  Created on: Oct 17, 2024
 *      Author: yjia67
 *
 */

#include <Lab6.h>
#include "ti_msp_dl_config.h"
#include "stdio.h"

DL_TimerG_ClockConfig gTIMER_0ClockConfig = {
                                            .clockSel = DL_TIMER_CLOCK_LFCLK, // 32 KHz
                                            .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
                                            .prescale = 255 // 125 Hz
};

DL_TimerG_TimerConfig gTIMER_0TimerConfig = {
                                             .period = 125, // once tick per second
                                             .timerMode = DL_TIMER_TIMER_MODE_PERIODIC,
                                             .startTimer = DL_TIMER_STOP
};

DL_ADC12_ClockConfig gADC_0ClockConfig = {
                                          .clockSel = DL_ADC12_CLOCK_ULPCLK,
                                          .freqRange = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
                                          .divideRatio = DL_ADC12_CLOCK_DIVIDE_8
};
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

float temp_data[30] = {0};
volatile uint8_t num_samples = 0;
volatile uint8_t target_num = 30;
volatile uint8_t S1_pressed = 0;
volatile uint8_t data_ready = 0;
volatile DL_FLASHCTL_COMMAND_STATUS error_flag;

int main(void)
{

    // initialization stuff
    SYSCFG_DL_init();
    GPIO_init();
    UART_init();
    TimerG_init();
    ADC_init();

    // configure interrupts
    NVIC_ClearPendingIRQ(TIMG0_INT_IRQn);
    NVIC_EnableIRQ(TIMG0_INT_IRQn);

    // start timer
    DL_TimerG_startCounter(TIMG0);

    // main loop
    while (1) {

        // if S1 is pressed, set the flag so data collection occurs in timer ISR
        if (DL_GPIO_readPins(S1) > 0) {

            S1_pressed = 1;
            DL_GPIO_setPins(LED1);

        }

        // if data is ready, write to flash memory and set LED2 to red
        if (data_ready == 1) {

            // disable interrupt
            DL_TimerG_disableInterrupt(TIMG0, DL_TIMER_INTERRUPT_ZERO_EVENT);
            NVIC_ClearPendingIRQ(TIMG0_INT_IRQn);
            NVIC_DisableIRQ(TIMG0_INT_IRQn);

            // write to flash memory
            write2Flash();

            // set LED2 to red when done
            DL_GPIO_clearPins(LED2_B);
            DL_GPIO_setPins(LED2_R);
            while (1) {}

        }

        // if S2 is pressed, read from flash memory and print to putty
        if (DL_GPIO_readPins(S2) == 0) {

            // disable interrupt
            DL_TimerG_disableInterrupt(TIMG0, DL_TIMER_INTERRUPT_ZERO_EVENT);
            NVIC_ClearPendingIRQ(TIMG0_INT_IRQn);
            NVIC_DisableIRQ(TIMG0_INT_IRQn);

            // read from flash memory
            float *data_from_flash = (float*)(MAIN_BASE_ADDRESS + 0x00400000);

            // convert the data and print to putty terminal
            print2Putty(data_from_flash);

            // set LED2 to green when done
            DL_GPIO_setPins(LED2_G);
            while (1) {}

        }

    }

    return 0;

}

void TIMG0_IRQHandler(void)
{

    // data acquisition mode if S1 is pressed
    if (S1_pressed == 1) {
        DL_GPIO_togglePins(LED2_B);
        if (num_samples < target_num) {
            collectData();

        // stop data acquisition and raise flag that it's ready
        } else {
            data_ready = 1;
            DL_GPIO_setPins(LED2_B);
            DL_TimerG_disableInterrupt(TIMG0, DL_TIMER_INTERRUPT_ZERO_EVENT);
        }
    } else {
        DL_GPIO_togglePins(LED1);
    }
}

// function for printing the passed in data to putty
void print2Putty(float *data)
{
    char buffer[25];
    for (int i = 0; i < target_num; i++) {
        sprintf(buffer, "time: %ds, temp: %.1f\r\n", i, data[i]);
        for (int j = 0; j < 25; j++) {
            DL_UART_transmitDataBlocking(UART0, buffer[j]);
        }
    }
}

// function for writing to flash memory
void write2Flash(void)
{
    // unprotect sector
    DL_FlashCTL_unprotectSector(FLASHCTL, MAIN_BASE_ADDRESS, DL_FLASHCTL_REGION_SELECT_MAIN);

    // erase sector in main memory
    error_flag = DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL, MAIN_BASE_ADDRESS, DL_FLASHCTL_COMMAND_SIZE_SECTOR);

    // write data to memory
    error_flag = DL_FlashCTL_programMemoryFromRAM(FLASHCTL, MAIN_BASE_ADDRESS, (uint32_t*) &temp_data[0], (uint32_t) target_num, DL_FLASHCTL_REGION_SELECT_MAIN);
}

// collect data through the ADC module
void collectData(void)
{
    while (DL_ADC12_getStatus(ADC0) == DL_ADC12_STATUS_CONVERSION_ACTIVE) {}
    DL_ADC12_enableConversions(ADC0);
    DL_ADC12_startConversion(ADC0);
    uint16_t adc_out =  DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);
    float temp = adc2temp((float)adc_out);
    temp_data[num_samples] = temp;
    num_samples++;
}

// convert ADC level reading to a floating point rep of temperature in F
float adc2temp(float adc_out)
{
    float res = 3.3/4096; // resolution in volt/number
    float volt = adc_out * res;
    float temp = volt / 0.022;
    return temp;
}

// initialize / configure timer
void TimerG_init(void)
{
    DL_TimerG_enablePower(TIMG0);
    DL_TimerG_setClockConfig(TIMG0, &gTIMER_0ClockConfig);
    DL_TimerG_initTimerMode(TIMG0, &gTIMER_0TimerConfig);
    DL_TimerG_enableInterrupt(TIMG0, DL_TIMER_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(TIMG0);
}

// initialize / configure ADC
void ADC_init(void)
{
    DL_ADC12_enablePower(ADC0);
    DL_ADC12_setClockConfig(ADC0, &gADC_0ClockConfig);

    // configure conversion memory
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

// UART initialization
void UART_init(void)
{
    // enable peripheral functions
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM21, IOMUX_PINCM21_PF_UART0_TX);

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

    // enable module
    DL_UART_enable(UART0);
}

// configure LED for debug purposes
void GPIO_init(void)
{
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

    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM40, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
                                     DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(IOMUX_PINCM49, DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
                                     DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
}
