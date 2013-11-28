// this is the platform definition header file

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "inc/hw_ints.h"

//Genral
#define WRFL_XTAL			SYSCTL_XTAL_16MHZ

//LCD
#define LCD_DC_PORT			GPIO_PORTB_BASE
#define LCD_DC_PIN			GPIO_PIN_1
#define LCD_RESET_PORT		GPIO_PORTB_BASE
#define LCD_RESET_PIN		GPIO_PIN_0
#define LCD_SSI_BASE		SSI2_BASE
#define BMP180_I2C_BASE		I2C1_BASE
#define BMP180_I2C_INT		INT_I2C1
