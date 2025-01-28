/*
 * Final Project - Self-Balancing Cube
 * ** Single edge balancing version **
 *
 *  Created on: Oct 22, 2024
 *      Author: yjia67
 *
 */

#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include <project_rev3.h>

int main(void)
{
    // initialization stuff
    SYSCFG_DL_init();
    GPIO_init();
    UART_init();
    TimerG_init();
    TimerA_PWM_init();
    I2C_init();
    mpu6050_init();
    motor_init();
    calibrate_gyro();

    // main loop
    while (1) {

        check_switches();

        if (calibrating) {
            calibrate_accel();
//            test_drive();
        } else if (balancing) {
            NVIC_ClearPendingIRQ(TIMG0_INT_IRQn);
            NVIC_EnableIRQ(TIMG0_INT_IRQn);
            DL_TimerG_startCounter(TIMG0);
            balancing = 0;
            __WFE();
        }

//        if (print_data_flag) {
//            DL_TimerG_disableInterrupt(TIMG0, DL_TIMER_INTERRUPT_ZERO_EVENT);
//            NVIC_ClearPendingIRQ(TIMG0_INT_IRQn);
//            NVIC_DisableIRQ(TIMG0_INT_IRQn);
//            drive_motor(0);
//            DL_GPIO_clearPins(BRAKE_PORT, BRAKE_PIN);
////            print_raw_data();
////            print_data();
////            print_attitude_data();
////            print_control_data();
////            print_converted_data();
//            print_data_flag = 0;
//            DL_GPIO_setPins(LED2_G);
//            __WFE();
//        }
    }
    return 0;
}



void test_drive(void)
{
//    uint16_t test_pwm = pwm_period * 0.9;
//    DL_TimerA_setCaptureCompareValue(TIMA0, test_pwm, DL_TIMER_CC_0_INDEX);
//    DL_GPIO_setPins(BRAKE_PORT, BRAKE_PIN);
//    DL_GPIO_setPins(DIR1_PORT, DIR1_PIN);
    drive_motor(min_pwm);
    delay_cycles(16000);
    drive_motor(0);
}


// read data every 10ms
void TIMG0_IRQHandler(void)
{
    DL_GPIO_clearPins(LED1);
    read_sensor_data();
    estimate_attitude();
//    monitor_attitude_data();
    pid();
    t_count++;
}

void pid(void)
{
//    // p control
//    int u = kp * theta_X;

//    // pd control
//    int u = kp * theta_X + kd * gXc;

//    // pid control
//    theta_X_error += theta_X * dt;
//    int u = kp * theta_X + kd * gXc + ki * theta_X_error;
//

//    float angle_fixrate = 0.001;
//    if (theta_X < theta_X_target) {
//        theta_X_target += angle_fixrate * dt;
//    } else {
//        theta_X_target -= angle_fixrate * dt;
//    }
//
//    theta_X -= theta_X_target;



    integral_error += theta_X * dt;
    motor_speed_x += duty_cycle;

    // pid + speed as a control input
    float omega_x = (float) gXc / gyroSensitivity;
    float p_term = kp * theta_X;
    float d_term = kd * omega_x;
    float i_term = ki * integral_error;
    float s_term = ks * duty_cycle;


    // lqr control
    duty_cycle = kp * theta_X + kd * omega_x + ki * integral_error + ks * duty_cycle;



    // anti windup guard
    if ((fabs(duty_cycle) >= max_duty_cycle) && ((integral_error >= 0 && theta_X >= 0) || (integral_error < 0 && theta_X < 0))) {
        integral_error -= theta_X * dt;
    }

    // clamp the control input - wheel saturates at this point
    if (duty_cycle > max_duty_cycle) {
        duty_cycle = max_duty_cycle;
    } else if (duty_cycle < -max_duty_cycle) {
        duty_cycle = -max_duty_cycle;
    }

    pwm_x = (int) (duty_cycle / 100.0 * pwm_period);
    drive_motor(pwm_x);

    char buffer[60];
    int length = sprintf(buffer, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n\r", theta_X, p_term, omega_x, d_term, duty_cycle, s_term, integral_error, i_term);
    for (int j = 0; j < length; j++) { DL_UART_transmitDataBlocking(UART0, buffer[j]); }

    DL_GPIO_setPins(LED2_G);
}

void drive_motor(int32_t pwm)
{
    // assign direction based on sign
    DL_GPIO_setPins(BRAKE_PORT, BRAKE_PIN);
    if (pwm < 0) {
        DL_GPIO_setPins(DIR1_PORT, DIR1_PIN);
        pwm = abs(pwm);
    } else {
        DL_GPIO_clearPins(DIR1_PORT, DIR1_PIN);
    }

    // cap at 95% duty cycle
    if (pwm > max_pwm) {
        pwm = max_pwm;
    }

    DL_TimerA_setCaptureCompareValue(TIMA0, pwm_period - (uint16_t) pwm, DL_TIMER_CC_0_INDEX);
}

// calculate robot angles and quaternions by fusing together accel and gyro
void estimate_attitude(void)
{
    // convert gyro reading to floats in terms of deg/s
    float gX_val = (float) gXc / (float) gyroSensitivity;
    float gY_val = (float) gYc / (float) gyroSensitivity;
    float gZ_val = (float) gZc / (float) gyroSensitivity;

    // integrate angular velocity to get angle
    theta_gX = theta_X + gX_val * dt;
    theta_gY = theta_X + gY_val * dt;
    theta_gX_wrong += gX_val * dt;

    // calculate angle from acceleration * 180 / pi
    theta_aX = -atan2(aYc, sqrt(aXc*aXc + aZc*aZc)) * 57.2958;

    // complementary filter for accel + gyro
    theta_X = (1.0 - accel_gain) * theta_gX + accel_gain * theta_aX;
}

void monitor_attitude_data(void)
{
    char buffer[25];
    int length = sprintf(buffer, "%.3f, %.3f, %.3f\n\r", theta_aX, theta_gX_wrong, theta_X);
    for (int j = 0; j < length; j++) { DL_UART_transmitDataBlocking(UART0, buffer[j]); }
}

void monitor_raw_data(void)
{
    read_sensor_data();
    float aXp = (float) aX / (float) accelSensitivity;
    float aYp = (float) aY / (float) accelSensitivity;
    float aZp = (float) aZ / (float) accelSensitivity;

    float gXp = (float) gXc / (float) gyroSensitivity;
    float gYp = (float) gYc / (float) gyroSensitivity;
    float gZp = (float) gZc / (float) gyroSensitivity;

    char buffer[100];
    int length = sprintf(buffer, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n\r", aXp, aYp, aZp, gXp, gYp, gZp);
    for (int j = 0; j < length; j++) { DL_UART_transmitDataBlocking(UART0, buffer[j]); }
}

void store_raw_data(void)
{
    if (count < num_data) {
        DL_GPIO_clearPins(LED1);
        read_accel_data();
        read_gyro_data();
        accel_data[count*3] = aX;
        accel_data[count*3+1] = aY;
        accel_data[count*3+2] = aZ;
        gyro_data[count*3] = gX;
        gyro_data[count*3+1] = gY;
        gyro_data[count*3+2] = gZ;
        count++;
    } else {
        print_data_flag = 1;
        DL_GPIO_setPins(LED1);
        DL_GPIO_setPins(LED2_G);
    }
}

void print_raw_data(void)
{
    DL_GPIO_setPins(LED1);
    for (uint16_t i = 0; i < num_data; i++) {
        int16_t aXp = accel_data[3*i];
        int16_t aYp = accel_data[3*i+1];
        int16_t aZp = accel_data[3*i+2];
        int16_t gXp = gyro_data[3*i];
        int16_t gYp = gyro_data[3*i+1];
        int16_t gZp = gyro_data[3*i+2];
        char buffer[100];
        int length = sprintf(buffer, "%d, %d, %d, %d, %d, %d\n\r", aXp, aYp, aZp, gXp, gYp, gZp);
        for (int j = 0; j < length; j++) { DL_UART_transmitDataBlocking(UART0, buffer[j]); }
    }
    print_data_flag = 0;
    DL_GPIO_setPins(LED2_G);
}

void print_converted_data(void)
{
    DL_GPIO_setPins(LED1);
    for (uint16_t i = 0; i < num_data; i++) {
        float aXp = (float) accel_data[3*i] / (float) accelSensitivity;
        float aYp = (float) accel_data[3*i+1] / (float) accelSensitivity;
        float aZp = (float) accel_data[3*i+2] / (float) accelSensitivity;
        float gXp = (float) gyro_data[3*i] / (float) gyroSensitivity;
        float gYp = (float) gyro_data[3*i+1] / (float) gyroSensitivity;
        float gZp = (float) gyro_data[3*i+2] / (float) gyroSensitivity;

        char buffer[100];
        int length = sprintf(buffer, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n\r", aXp, aYp, aZp, gXp, gYp, gZp);
        for (int j = 0; j < length; j++) { DL_UART_transmitDataBlocking(UART0, buffer[j]); }
    }
    print_data_flag = 0;
    DL_GPIO_setPins(LED2_G);
}

// poll switches to take a action based on button presses
void check_switches(void)
{
    if (DL_GPIO_readPins(S1) > 0) {
        S1_pressed = 1;
    } else if (S1_pressed == 1) {
        delay_cycles(1000);
        S1_pressed = 0;
        calibrating = 1;
    }

    if (DL_GPIO_readPins(S2) == 0) {
        S2_pressed = 1;
    } else if (S2_pressed == 1) {
        delay_cycles(1000);
        S2_pressed = 0;
        balancing = 1;
        if (pwm_x < 250) {
            pwm_x += 50;
        }
    }
}

// calibrate accel at balancing point to account for offsets
void calibrate_accel(void)
{
    DL_GPIO_setPins(LED2_B);
    int32_t sum_aX = 0, sum_aY = 0, sum_aZ = 0;

    for (uint16_t i = 0; i < num_cal_data; i++) {
        read_sensor_data();
        sum_aX += aXc;
        sum_aY += aYc;
        sum_aZ += aZc; // +1g for gravity
    }

    // calculate the average offset
    aX_offset = sum_aX / num_cal_data;
    aY_offset = sum_aY / num_cal_data;
    aZ_offset = sum_aZ / num_cal_data + 16384;

    // take a few samples so the moving average isn't messed up
    for (uint8_t i = 0; i < 20; i++) {
        read_sensor_data();
    }

    estimate_attitude();
    theta_X_target = theta_X;

    calibrating = 0;
    DL_GPIO_clearPins(LED2_B);

    /*
        accel calibration complete:
         X: -7803, Y: -659, Z: 1603
     */
    aX_offset = -7803;
    aY_offset = -659;
    aZ_offset = 1603;

    char buffer[100];
    int length = sprintf(buffer, "accel calibration complete:\n\r X: %d, Y: %d, Z: %d\n\r target angle: %.3f\n\r",
                         aX_offset, aY_offset, aZ_offset, theta_X_target);
    for (int j = 0; j < length; j++) {
        DL_UART_transmitDataBlocking(UART0, buffer[j]);
    }



}

// calibrate gyro at program start up
void calibrate_gyro(void) {
    DL_GPIO_setPins(LED2_B);
    int32_t sum_gX = 0, sum_gY = 0, sum_gZ = 0;

    for (int i = 0; i < num_cal_data; i++) {
        read_gyro_data();
        sum_gX += gX;
        sum_gY += gY;
        sum_gZ += gZ;
        delay_cycles(5);
    }
    gX_offset = sum_gX / num_cal_data;
    gY_offset = sum_gY / num_cal_data;
    gZ_offset = sum_gZ / num_cal_data;

    DL_GPIO_clearPins(LED2_B);
    char buffer[100];
    int length = sprintf(buffer, "gyro calibration complete:\n\r X: %d, Y: %d, Z: %d\n\r",
                         gX_offset, gY_offset, gZ_offset);
    for (int j = 0; j < length; j++) {
        DL_UART_transmitDataBlocking(UART0, buffer[j]);
    }

    length = sprintf(buffer, "p: %.3f, d: %.3f, i: %.3f, s: %.3f\n\r",
                         kp, kd, ki, ks);
    for (int j = 0; j < length; j++) {
        DL_UART_transmitDataBlocking(UART0, buffer[j]);
    }
}

// read sensor data
void read_sensor_data(void)
{
    read_accel_data();
    read_gyro_data();

    // take moving average and apply calibration offsets
    aXc = (1 - alpha) * aXc + alpha * (aX - aX_offset);
    aYc = (1 - alpha) * aYc + alpha * (aY - aY_offset);
    aZc = (1 - alpha) * aZc + alpha * (aZ - aZ_offset);

    gXc = (1 - alpha) * gXc + alpha * (gX - gX_offset);
    gYc = (1 - alpha) * gYc + alpha * (gY - gY_offset);
    gZc = (1 - alpha) * gZc + alpha * (gZ - gZ_offset);
}

// read gyroscope data
void read_gyro_data(void)
{
    uint8_t rx_buffer[6];
    i2c_read(MPU6050, GYRO_OUT, rx_buffer, 6);
    gX = (rx_buffer[0] << 8) | rx_buffer[1];
    gY = (rx_buffer[2] << 8) | rx_buffer[3];
    gZ = (rx_buffer[4] << 8) | rx_buffer[5];
}

// read accelerometer data
void read_accel_data(void)
{
    uint8_t rx_buffer[6];
    i2c_read(MPU6050, ACCEL_OUT, rx_buffer, 6);
    aX = (rx_buffer[0] << 8) | rx_buffer[1];
    aY = (rx_buffer[2] << 8) | rx_buffer[3];
    aZ = (rx_buffer[4] << 8) | rx_buffer[5];
}


