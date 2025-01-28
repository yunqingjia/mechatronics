/*
 * Final Project for AE6705 Header File
 *
 *  Created on: Oct 22, 2024
 *      Author: yjia67
 *
 */


void GPIO_init(void);
void ADC_init(void);
void TimerG_PWM_init(void);
void check_mode(void);
int get_ADC2PWM(void);
void operate_motor(void);

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
