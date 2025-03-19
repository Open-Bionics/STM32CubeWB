/**
 * @file led.h
 * @brief LED control functions
 */


#include <stdbool.h>
#include "main.h"

void LED_Init(void);
void LED_SetCharging(bool charging);
void LED_Process(void);