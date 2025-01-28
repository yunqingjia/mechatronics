/*
 * Lab5.h
 *
 *  Created on: Oct 6, 2024
 *      Author: yjia67
 *
 */

void TimerG_init(void);
void ADC_init(void);
void GPIO_init(void);
double adc2temp(int data);

#define LED1   		GPIOA, DL_GPIO_PIN_0
#define gPeriod		125000 // # of clock cycles in 1 second
