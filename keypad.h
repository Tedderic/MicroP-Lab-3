#ifndef KEYPAD_H
#define KEYPAD_H

#include "stm32f4xx_hal.h"
#include "supporting_functions.h"

enum IOKey
{
	input,
	output
};

int read_keypad(void);
void read_keypad_col(void);
void read_keypad_row(void);
void initGPIOT(GPIO_TypeDef* GPIOx, uint16_t pins, enum IOKey input);
int interpret_keypad(void);

#endif
