/*
 * Lab6.h
 *
 *  Created on: Oct 17, 2024
 *      Author: yjia67
 *
 */


void GPIO_init(void);
void UART_init(void);
void ADC_init(void);
void TimerG_init(void);
float adc2temp(float adc_out);
void collectData(void);
void write2Flash(void);
void print2Putty(float *data);

#define LED1   				GPIOA, DL_GPIO_PIN_0
#define LED2_R              GPIOB, DL_GPIO_PIN_26
#define LED2_G              GPIOB, DL_GPIO_PIN_27
#define LED2_B              GPIOB, DL_GPIO_PIN_22
#define S1          		GPIOA, DL_GPIO_PIN_18
#define S2          		GPIOB, DL_GPIO_PIN_21
#define MAIN_BASE_ADDRESS     (0x00008000)


#define gPeriod             125000 // # of clock cycles in 1 second
#define delay_num_cycles    8e5
#define MAIN_BASE_ADDRESS_W   (0x00008000)
#define MAIN_BASE_ADDRESS_R   (0x00008000 + 0x00400000)

