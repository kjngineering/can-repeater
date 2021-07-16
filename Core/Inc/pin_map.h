#ifndef PIN_MAP_H
#define PIN_MAP_H 1
#include "stdint.h"
#include "stm32f1xx_hal_gpio.h"
// --- defgroup for PORTA ---
#define BUTTON_0_PIN GPIO_PIN_0
#define BUTTON_0_GPIO_PORT GPIOA
// --- defgroup for PORTB ---
#define LED_0_PIN GPIO_PIN_2
#define LED_0_GPIO_PORT GPIOB

#define CAN_RX_PIN GPIO_PIN_8
#define CAN_RX_GPIO_PORT GPIOB
#define CAN_TX_PIN GPIO_PIN_9
#define CAN_TX_GPIO_PORT GPIOB
// --- defgroup for PORTC ---
#define LED_1_PIN GPIO_PIN_13
#define LED_1_GPIO_PORT GPIOC

#endif // PIN_MAP_H
