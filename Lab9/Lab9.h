/*
 * Lab 9 for AE6705 Header File
 *
 *  Created on: Nov 12, 2024
 *      Author: yjia67
 *
 */


void GPIO_init(void);
void TimerG_PWM_init(void);
void TimerG6_CC_init(void);
void UART_init(void);

void print_data(float time, uint8_t speed);
void process_data(void);
void operate_motor(void);
void check_switches(void);

#define timg6_iomux IOMUX_PINCM15
#define timg6_port  GPIOB
#define timg6_pin   2

#define pwm1_iomux  IOMUX_PINCM30
#define pwm1_port   GPIOB
#define pwm1_pin    13
#define pwm2_iomux  IOMUX_PINCM55
#define pwm2_port   GPIOA
#define pwm2_pin    25

#define LED1    GPIOA, DL_GPIO_PIN_0
#define LED2_R  GPIOB, DL_GPIO_PIN_26
#define LED2_G  GPIOB, DL_GPIO_PIN_27
#define LED2_B  GPIOB, DL_GPIO_PIN_22
#define S1      GPIOA, DL_GPIO_PIN_18
#define S2      GPIOB, DL_GPIO_PIN_21
