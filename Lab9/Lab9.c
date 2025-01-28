/*
 * Lab 9 for AE6705
 *
 *  Created on: Nov 12, 2024
 *      Author: yjia67
 *
 */

#include "ti_msp_dl_config.h"
#include "stdio.h"
#include <Lab9.h>

DL_TimerG_ClockConfig gCAPTURE_0ClockConfig = {
                                          .clockSel = DL_TIMER_CLOCK_BUSCLK, // 32 MHz
                                          .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
                                          .prescale = 0 // 4 MHz
};


DL_TimerG_CaptureConfig gCAPTURE_0CaptureConfig = {
                                                 .captureMode = DL_TIMER_CAPTURE_MODE_EDGE_TIME,
                                                 .period = 65535, // ~ 2 ms per period
                                                 .startTimer = DL_TIMER_STOP,
                                                 .edgeCaptMode = DL_TIMER_CAPTURE_EDGE_DETECTION_MODE_FALLING,
                                                 .inputChan = DL_TIMER_INPUT_CHAN_0,
                                                 .inputInvMode = DL_TIMER_CC_INPUT_INV_NOINVERT
};

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

// flags for tracking button presses
volatile uint8_t S1_pressed = 0;
volatile uint8_t S2_pressed = 0;
volatile uint8_t motor_mode = 0; // 0: off, 1: on

// PWM setting
volatile uint32_t gPWM_period = 512000;
volatile int duty_cycle = 0;

// data storage
volatile int edge_count = 0; // timer count at falling edge
volatile uint16_t data[5000] = {0}; // storing the timer counts
volatile uint16_t data_size = 5000;
volatile uint16_t count = 0; // counter variable for data
volatile uint8_t data_ready = 0;

// conversion factors
volatile int gCC_period = 65536;
const float timer_conversion = 2.5e-7; // period of each clock cycle in s for 4 MHz clock

int main(void)
{

    // initialization stuff
    SYSCFG_DL_init();
    GPIO_init();
    TimerG_PWM_init();
    TimerG6_CC_init();
    UART_init();

    // main loop
    while (1) {
        check_switches();
        operate_motor();

        if (data_ready == 1) {
            process_data();
            data_ready = 0;
            __WFE();
        }
    }

    return 0;

}

// print out data
void print_data(float time, uint8_t speed)
{
    char buffer[20];
    int length = sprintf(buffer, "%0.6f, %u\n\r", time, speed);
    for (int j = 0; j < length; j++) {
        DL_UART_transmitDataBlocking(UART0, buffer[j]);
    }

//    // print to human-readable format in console
//    printf("time: %0.6f, speed: %d \n", time, speed);
}

// convert time tick data to timestamp and RPM
void process_data(void)
{
    // disable interrupts
    DL_TimerG_disableInterrupt(TIMG6, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);
    NVIC_ClearPendingIRQ(TIMG6_INT_IRQn);
    NVIC_DisableIRQ(TIMG6_INT_IRQn);

    DL_GPIO_clearPins(LED2_B);
    DL_GPIO_setPins(LED2_R);

    uint16_t prev_edge = data[0];
    float timestamp_i = 0.0;
    uint8_t speed_i = 0;
    float period_i = 0.0;

    for (int i = 1; i < data_size; i++) {
        uint16_t cur_edge = data[i];

        // check for rollovers
        if (prev_edge < cur_edge) {
            prev_edge += gCC_period;
        }

        // calculate period
        uint16_t diff = prev_edge - cur_edge;
        period_i = (double) diff / 4e6;

        // calculate timestamp seconds and speed in rpm
        timestamp_i += period_i;
        speed_i = (uint8_t) 60/(960*period_i);

        // only print out data from 0-2.5 seconds
        if (timestamp_i < 2.5) {
            print_data(timestamp_i, speed_i);
        } else {
            // indicate printing is finished and wait in an infinite loop
            DL_GPIO_clearPins(LED2_R);
            DL_GPIO_setPins(LED2_G);
            break;
        }

        prev_edge = cur_edge;
    }
}

// operate motor based on what mode it's in
void operate_motor(void)
{
    DL_TimerG_setCaptureCompareValue(TIMG12, (motor_mode == 1) ? gPWM_period - duty_cycle : gPWM_period, DL_TIMER_CC_0_INDEX);
}

// poll switches to take a action based on button presses
void check_switches(void)
{
    // poll S1 and increment duty cycle by 20% with each press
    if (DL_GPIO_readPins(S1) > 0) {
        S1_pressed = 1;
    } else if (S1_pressed == 1) {
        DL_Common_delayCycles(1000);
        S1_pressed = 0;
        // cap it at the maximum duty cycle
        if (duty_cycle < gPWM_period) {
            duty_cycle += (int)(gPWM_period * 0.2);
        }
    }

    // poll S2 and toggle the motor on and off
    if (DL_GPIO_readPins(S2) == 0) {
        S2_pressed = 1;
    } else if (S2_pressed == 1) {
        S2_pressed = 0;
        DL_GPIO_togglePins(LED2_B);
        motor_mode = (motor_mode == 1) ? 0 : 1;
    }
}

// IRS for capture intterrupt
void TIMG6_IRQHandler(void)
{
    if (count < data_size) {
        data[count] = DL_Timer_getCaptureCompareValue(TIMG6, DL_TIMER_CC_0_INDEX);
        count++;
    } else {
        data_ready = 1;
    }
}

// initialize / configure timer for Capture mode
void TimerG6_CC_init(void)
{
    DL_GPIO_initPeripheralInputFunction(timg6_iomux, 7);
    DL_TimerG_enablePower(TIMG6);
    DL_TimerG_setClockConfig(TIMG6, &gCAPTURE_0ClockConfig);
    DL_TimerG_initCaptureMode(TIMG6, &gCAPTURE_0CaptureConfig);
    DL_TimerG_enableInterrupt(TIMG6, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);
    DL_TimerG_enableClock(TIMG6);
    NVIC_ClearPendingIRQ(TIMG6_INT_IRQn);
    NVIC_EnableIRQ(TIMG6_INT_IRQn);
    DL_TimerG_startCounter(TIMG6);
}

// initialize / configure timer for PWM
void TimerG_PWM_init(void)
{
    DL_GPIO_initPeripheralOutputFunction(pwm1_iomux, 4);
    DL_GPIO_enableOutput(pwm1_port, pwm1_pin);
    DL_GPIO_initPeripheralOutputFunction(pwm2_iomux, 4);
    DL_GPIO_enableOutput(pwm2_port, pwm2_pin);
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
    DL_TimerG_setCaptureCompareValue(TIMG12, gPWM_period, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(TIMG12, gPWM_period, DL_TIMER_CC_1_INDEX);
    DL_TimerG_enableClock(TIMG12);
    DL_TimerG_setCCPDirection(TIMG12, DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT);
    DL_TimerG_startCounter(TIMG12);

}

// UART initialization
void UART_init(void)
{
    // enable peripheral functions
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM21, IOMUX_PINCM21_PF_UART0_TX);
    DL_UART_reset(UART0);
    DL_UART_enablePower(UART0);
    DL_UART_setClockConfig(UART0, &gUART_0ClockConfig);
    DL_UART_init(UART0, &gUART_0Config);
    DL_UART_setOversampling(UART0, DL_UART_OVERSAMPLING_RATE_16X);

    // set baud rate to 9600
    int clock_rate = 32e6;
    int baud_rate = 9600;
    int oversample_freq = 16;
    double temp = (double) clock_rate / (oversample_freq * baud_rate);
    int IBRD = (int)temp;
    int FBRD = (int)((temp-IBRD) * 64 + 0.5 );
    DL_UART_setBaudRateDivisor(UART0, IBRD, FBRD);

    DL_UART_enable(UART0);
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
