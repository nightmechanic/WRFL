/* this is the platform definition header file for the WRFL prototype
 *
 * Created on: Dec , 2013
 * Author: Ran Katz (Nightmechanic)
 *
 * This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
 * To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/deed.en_US.
 */

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "inc/hw_ints.h"

//General
#define WRFL_XTAL			SYSCTL_XTAL_16MHZ

//Switches
#define SW_BASE				GPIO_PORTF_BASE
#define SW1					GPIO_PIN_4
#define SW2					GPIO_PIN_0
#define SW_DEBOUNCE_CNT		3

//LCD
#define LCD_DC_PORT			GPIO_PORTB_BASE
#define LCD_DC_PIN			GPIO_PIN_1
#define LCD_RESET_PORT		GPIO_PORTB_BASE
#define LCD_RESET_PIN		GPIO_PIN_0
#define LCD_SSI_BASE		SSI2_BASE
#define BMP180_I2C_BASE		I2C1_BASE
#define BMP180_I2C_INT		INT_I2C1

// local sea level air pressure (for a more accurate absolute altitude measurement)
#define LOC_ALT_P0				102605.0f //101652.0f

//noise values for kalman filter(s)

#define ALT_KALMAN_R				0.5f
#define ALT_KALMAN_Q				0.1f
