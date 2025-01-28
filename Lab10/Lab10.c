/*
 * Lab 10 for AE6705
 *
 *  Created on: Dec 7, 2024
 *      Author: yjia67
 *
 */


#include "ti_msp_dl_config.h"
#include "stdio.h"
#include <Lab10.h>

// timer counts at falling edge
volatile uint32_t cur_edge = 0;
volatile uint32_t prev_edge = 0;

// controller gains
volatile float kp = 2.5;
volatile float ki = 40.0;
volatile float kd = 0.0;

// controller inputs
volatile float duty_cycle = 0.0;
volatile float max_duty_cycle = 100.0;
volatile float set_rpm = 50.0;
volatile float cur_rpm = 0.0;
volatile float prev_rpm = 0.0;
volatile float dt = 1.0 / 100.0;
volatile uint32_t cur_pwm = 512000;;

volatile float rpm_error_p = 0.0;
volatile float rpm_error_d = 0.0;
volatile float rpm_error_i = 0.0;

// conversion factors
volatile int gCC_period = 65536;
const float timer_conversion = 2.5e-7; // period of each clock cycle in s for 4 MHz clock

// house keeping stuff
volatile uint8_t S1_pressed = 0;
volatile uint8_t motor_mode = 0; // 0: off, 1: on
volatile uint8_t print_counter = 0;


int main(void)
{

    // initialization stuff
    SYSCFG_DL_init();
    GPIO_init();
    TimerG_PWM_init();
    TimerG6_CC_init();
    TimerG_init();
    UART_init();


    // main loop
    while (1) {
        check_switches();
        // if motor is supposed to be on, start the controller ISR
        if (motor_mode) {
            DL_GPIO_setPins(LED2_B);
            NVIC_ClearPendingIRQ(TIMG0_INT_IRQn);
            NVIC_EnableIRQ(TIMG0_INT_IRQn);
            DL_TimerG_enableInterrupt(TIMG0, DL_TIMER_INTERRUPT_ZERO_EVENT);
            DL_TimerG_startCounter(TIMG0);
            __WFI();
            motor_mode = 0;
        }
    }

    return 0;
}

// ISR for PID and printing data
void TIMG0_IRQHandler(void)
{
    DL_GPIO_setPins(LED2_G);
    // estimate state
    prev_rpm = cur_rpm;
    cur_rpm = calc_rpm();

    // pid control
    pid();

    // drive motor with updated duty cycle
    drive_motor(duty_cycle);

    // print data at 100 Hz
    print_counter++;
    if (print_counter == 1) {
        print_data();
        print_counter = 0;
    }
}

// pid control
void pid(void)
{
    // calculate error
    rpm_error_p = set_rpm - cur_rpm;

    // calculate derivative
    rpm_error_d = (cur_rpm - prev_rpm) / dt;

    // calculate integral error
    rpm_error_i += rpm_error_p * dt;

    // pid
    duty_cycle = kp * rpm_error_p + ki * rpm_error_i + kd * rpm_error_d;

    // anti-windup
    if ((fabs(duty_cycle) >= max_duty_cycle) && ((rpm_error_i >= 0 && rpm_error_p >= 0) || (rpm_error_i < 0 && rpm_error_p < 0))) {
        rpm_error_i -= rpm_error_p * dt;
    }

    // enforce saturation
    if (duty_cycle < 0.0) {
        duty_cycle = 0.0;
    } else if (duty_cycle > 100.0) {
        duty_cycle = 100.0;
    }

}


// calculate motor rpm based on encoder pulse counts
float calc_rpm(void)
{
    if (prev_edge < cur_edge) {
        prev_edge += gCC_period;
    }
    uint16_t diff = prev_edge - cur_edge;
    float period = (float) diff * timer_conversion;
    float rpm = 60.0 / (960.0 * period);
    return rpm;
}

// drive motor with specified duty_cycle
void drive_motor(float duty_cycle) {
    cur_pwm = (uint32_t) (duty_cycle * (float) pwm_period / 100);
    DL_TimerG_setCaptureCompareValue(TIMG12, pwm_period - cur_pwm, DL_TIMER_CC_0_INDEX);
}

// print out motor rpm
void print_data(void)
{
    char buffer[30];
    int length = sprintf(buffer, "%.3f, ", cur_rpm);
//    int length = sprintf(buffer, "%.3f \n\r", cur_rpm);
    for (int j = 0; j < length; j++) {
        DL_UART_transmitDataBlocking(UART0, buffer[j]);
    }
}

// ISR for capture intterrupt
void TIMG6_IRQHandler(void)
{
    prev_edge = cur_edge;
    cur_edge = DL_Timer_getCaptureCompareValue(TIMG6, DL_TIMER_CC_0_INDEX);
}

// poll switches to take a action based on button presses
void check_switches(void)
{
    // poll S1 and turn motor on/off
    if (DL_GPIO_readPins(S1) > 0) {
        S1_pressed = 1;
    } else if (S1_pressed == 1) {
        DL_Common_delayCycles(1000);
        S1_pressed = 0;
        motor_mode = (motor_mode == 1) ? 0 : 1;
    }
}

