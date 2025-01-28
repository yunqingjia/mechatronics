/*
 * Final Project for AE6705 Header File
 *
 *  Created on: Oct 22, 2024
 *      Author: yjia67
 *
 */


void GPIO_init(void);
void TimerG_init(void);

#define LED1    GPIOA, DL_GPIO_PIN_0
#define LED2_R  GPIOB, DL_GPIO_PIN_26
#define LED2_G  GPIOB, DL_GPIO_PIN_27
#define LED2_B  GPIOB, DL_GPIO_PIN_22
#define S1      GPIOA, DL_GPIO_PIN_18
#define S2      GPIOB, DL_GPIO_PIN_21

// A1: blue    B1: pink    A2: yellow    B2: orange
#define M_PORT  GPIOB
#define A1_PIN  DL_GPIO_PIN_8
#define B1_PIN  DL_GPIO_PIN_7
#define A2_PIN  DL_GPIO_PIN_6
#define B2_PIN  DL_GPIO_PIN_0
