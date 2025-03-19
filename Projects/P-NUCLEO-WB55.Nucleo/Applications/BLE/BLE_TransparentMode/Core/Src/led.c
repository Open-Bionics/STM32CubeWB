


#include "main.h"
#include "led.h"


typedef enum LED_COLOUR
{
    LED_COLOUR_OFF,
    LED_COLOUR_GREEN,
    LED_COLOUR_RED,
    LED_COLOUR_ORANGE,
}led_colour_t;



// GPIO_PIN_RESET to turn an LED on
#define LED_ON  GPIO_PIN_RESET
#define LED_OFF GPIO_PIN_SET


#define SOLID_LED       0
#define FLASHING_LED    1

extern TIM_HandleTypeDef htim17;
static bool is_charging = false;
static led_colour_t current_led_colour[2] = {LED_COLOUR_OFF, LED_COLOUR_OFF};

static void set_led_colour(uint8_t led_num, led_colour_t colour);

/**
 * @brief Initialise the LEDs
 */
void LED_Init(void)
{
    // start the timer, set 1 LED on and toggle the other
    set_led_colour(SOLID_LED, LED_COLOUR_GREEN);
    HAL_TIM_Base_Start_IT(&htim17);
}


/**
 * @brief Set the charging state
 * 
 * @param charging 
 */
void LED_SetCharging(bool charging)
{
    is_charging = charging;

    // if we're charging, set the LEDs to orange
    if (is_charging)
    {
        set_led_colour(SOLID_LED, LED_COLOUR_ORANGE);
        set_led_colour(FLASHING_LED, LED_COLOUR_ORANGE);
    }
    // else if we're not charging, set the LEDs to green
    else
    {
        set_led_colour(SOLID_LED, LED_COLOUR_GREEN);
        set_led_colour(FLASHING_LED, LED_COLOUR_GREEN);
    }
}

/**
 * @brief Process the LEDs, called the the interrupt handler
 */
void LED_Process(void)
{
    static led_colour_t next_led_colour = LED_COLOUR_GREEN;

    set_led_colour(FLASHING_LED, next_led_colour);

    // if the LED is currently on, turn it off next time
    if (LED_COLOUR_OFF != next_led_colour)
    {
        next_led_colour = LED_COLOUR_OFF;
    }
    // else if the LED is off, set the next colour
    else
    {
        next_led_colour = is_charging ? LED_COLOUR_ORANGE : LED_COLOUR_GREEN;
    }
}

/**
 * @brief Set the colour of an LED
 * 
 * @param led_num LED_FLASHING or LED_SOLID
 * @param colour LED_COLOUR_GREEN, LED_COLOUR_RED, LED_COLOUR_ORANGE or LED_COLOUR_OFF
 */
static void set_led_colour(uint8_t led_num, led_colour_t colour)
{
    GPIO_TypeDef * green_led_port = NULL;
    GPIO_TypeDef * red_led_port = NULL;
    uint16_t green_led_pin = 0;
    uint16_t red_led_pin = 0;

    switch (led_num)
    {
        case FLASHING_LED:
            green_led_port = LED_GREEN_1_GPIO_Port;
            green_led_pin = LED_GREEN_1_Pin;
            red_led_port = LED_RED_1_GPIO_Port;
            red_led_pin = LED_RED_1_Pin;
            break;
        case SOLID_LED:
            green_led_port = LED_GREEN_2_GPIO_Port;
            green_led_pin = LED_GREEN_2_Pin;
            red_led_port = LED_RED_2_GPIO_Port;
            red_led_pin = LED_RED_2_Pin;
            break;
        default:
            return;
    }


    switch (colour)
    {
        case LED_COLOUR_GREEN:
            HAL_GPIO_WritePin(green_led_port, green_led_pin, LED_ON);
            HAL_GPIO_WritePin(red_led_port,   red_led_pin,   LED_OFF);
            break;
        case LED_COLOUR_RED:
            HAL_GPIO_WritePin(green_led_port, green_led_pin, LED_OFF);
            HAL_GPIO_WritePin(red_led_port,    red_led_pin,  LED_ON);
            break;
        case LED_COLOUR_ORANGE:
            HAL_GPIO_WritePin(green_led_port, green_led_pin, LED_ON);
            HAL_GPIO_WritePin(red_led_port,   red_led_pin,   LED_ON);
            break;
        case LED_COLOUR_OFF:
            HAL_GPIO_WritePin(green_led_port, green_led_pin, LED_OFF);
            HAL_GPIO_WritePin(red_led_port,   red_led_pin,   LED_OFF);
            break;
        default:
            return;
    }

    current_led_colour[led_num] = colour;
}



