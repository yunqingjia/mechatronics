/*
 * Lab3.h
 *
 *  Created on: Sep 19, 2024
 *      Author: yjia67
 *
 *          PINCM   PinName
 * LED1:    1       PA0
 * LED2(R): 57      PB26
 * LED2(G): 58      PB27
 * LED2(B): 50      PB22
 * S1:      40      PA18
 * S2:      49      PB21
 */

#define LED1                GPIOA, DL_GPIO_PIN_0
#define LED2_R              GPIOB, DL_GPIO_PIN_26
#define LED2_G              GPIOB, DL_GPIO_PIN_27
#define LED2_B              GPIOB, DL_GPIO_PIN_22
#define S1                  GPIOA, DL_GPIO_PIN_18
#define S2                  GPIOB, DL_GPIO_PIN_21
#define delay_num_cycles    8e5 // clock cycle = 80MHz (8e7), delay by 10ms

// function prototypes for the ones in the main code
void init_GPIO(void);
void select_LED(int count);
