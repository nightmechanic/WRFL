/*
 * "Main" file for WRFL project
 * Ran Katz (nightmechanic) 2013
 *
 * This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
 * To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/deed.en_US.
 */



#include "Nokia5110.h"
#include "platform.h"
#include "wrfl_proto_mux.h"
#include "gears_logo.h"
//#include "Kalman_AltiUno.h"
#include "WRFL_Kalman.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "sensorlib/hw_bmp180.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/bmp180.h"
#include "driverlib/rom_map.h"




#define VAR_COUNT	128 //debug , to be removed

//global Kalman filter  structure for altitude
tKalman_State Alt_KState;

//*****************************************************************************
//
// Define BMP180 I2C Address.
//
//*****************************************************************************
#define BMP180_I2C_ADDRESS  0x77



//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Global instance structure for the BMP180 sensor driver.
//
//*****************************************************************************
tBMP180 g_sBMP180Inst;

//*****************************************************************************
//
// Global new data flag to alert main that BMP180 data is ready.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8DataFlag;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// BMP180 Sensor callback function.  Called at the end of BMP180 sensor driver
// transactions. This is called from I2C interrupt context. Therefore, we just
// set a flag and let main do the bulk of the computations and display.
//
//*****************************************************************************
void BMP180AppCallback(void* pvCallbackData, uint_fast8_t ui8Status)
{
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8DataFlag = 1;
    }
}

//*****************************************************************************
//
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the BMP180.
//
//*****************************************************************************
void
BMP180I2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sI2CInst);
}

//*****************************************************************************
//
// Called by the NVIC as a SysTick interrupt, which is used to generate the
// sample interval
//
//*****************************************************************************
void
SysTickIntHandler()
{
  //  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    BMP180DataRead(&g_sBMP180Inst, BMP180AppCallback, &g_sBMP180Inst);
  //  GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
}

int main(void) {
	
	int  delay;
	float fTemperature, fPressure, fAltitude, fK_Altitude;
	int count,i;
	static float f_meas[VAR_COUNT], fAlt_Mean, fAlt_Var;


	// Set the clocking to run directly from the external crystal/oscillator.
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
			WRFL_XTAL);


	//init ports and peripherals
	PortFunctionInit();
	Nokia5110_Init();
	Nokia5110_Clear();


    Nokia5110_DrawFullImage(gears_logo);
    for(delay=0; delay<1000000; delay=delay+1);

    Nokia5110_Clear();
    Nokia5110_SetCursor(3, 1);
    Nokia5110_OutString("WRFL");
    Nokia5110_SetCursor(1, 2);
    Nokia5110_OutString("Prototype");
    Nokia5110_SetCursor(2, 4);
    Nokia5110_OutString("VER 0.1");
    Nokia5110_SetCursor(0, 5);
    Nokia5110_OutString("NightMecanic");
    for(delay=0; delay<1000000; delay=delay+1);
    Nokia5110_Clear();
    //
   // Enable interrupts to the processor.
   //
   MAP_IntMasterEnable();

   //
   // Initialize I2C1 peripheral.
   //
   I2CMInit(&g_sI2CInst, BMP180_I2C_BASE, BMP180_I2C_INT, 0xff, 0xff,
			MAP_SysCtlClockGet());

   //
   // Initialize the BMP180
   //
   BMP180Init(&g_sBMP180Inst, &g_sI2CInst, BMP180_I2C_ADDRESS,
			  BMP180AppCallback, &g_sBMP180Inst);


   //
   // Wait for initialization callback to indicate reset request is complete.
   //
   while(g_vui8DataFlag == 0)
   {
	   //
	   // Wait for I2C Transactions to complete.
	   //
   }

   //
   // Reset the data ready flag
   //
   g_vui8DataFlag = 0;

   //set oversampling to 8x
   g_sBMP180Inst.ui8Mode = 0xC0;

   //Initialize the Kalman filter
   Kalman_Init(&Alt_KState, 0.0f, 1.0f, ALT_KALMAN_R, ALT_KALMAN_Q);

   //
   // Enable the system ticks at 10 hz.
   //
   MAP_SysTickPeriodSet(MAP_SysCtlClockGet() / (10 * 3));
   MAP_SysTickIntEnable();
   MAP_SysTickEnable();

  //config done
   count = 0;
   //
   // Begin the data collection and printing.  Loop Forever.
   //
   while(1)
   {
	   //
	   // Read the data from the BMP180 over I2C.  This command starts a
	   // temperature measurement.  Then polls until temperature is ready.
	   // Then automatically starts a pressure measurement and polls for that
	   // to complete. When both measurement are complete and in the local
	   // buffer then the application callback is called from the I2C
	   // interrupt context.  Polling is done on I2C interrupts allowing
	   // processor to continue doing other tasks as needed.
	   //
	   BMP180DataRead(&g_sBMP180Inst, BMP180AppCallback, &g_sBMP180Inst);
	   while(g_vui8DataFlag == 0)
	   {
		   //
		   // Wait for the new data set to be available.
		   //
	   }

	   //
	   // Reset the data ready flag.
	   //
	   g_vui8DataFlag = 0;

	   //
	   // Get a local copy of the latest temperature data in float format.
	   //
	   BMP180DataTemperatureGetFloat(&g_sBMP180Inst, &fTemperature);

	   //
	   // Convert the floats to an integer part and fraction part for easy
	   // print.
	   //


	   //
	   // Print temperature with three digits of decimal precision.
	   //

	   Nokia5110_SetCursor(0, 0);
	       Nokia5110_OutString("Temp:");

	       Nokia5110_OutFloatp3(fTemperature);

	   //
	   // Get a local copy of the latest air pressure data in float format.
	   //
	   BMP180DataPressureGetFloat(&g_sBMP180Inst, &fPressure);




	   //
	   // Print Pressure with three digits of decimal precision.
	   //

	   Nokia5110_SetCursor(0, 1);
	   Nokia5110_OutString("Pres:");
	   Nokia5110_SetCursor(0, 2);

	   //display in hPa
	   Nokia5110_OutFloatp3((fPressure / 100.0f));

	   //
	   // Calculate the altitude.
	   //
	   //fAltitude = 44330.0f * (1.0f - powf(fPressure / 101325.0f,
	//									   1.0f / 5.255f));
	   //corrected:
	   fAltitude = 44330.0f * (1.0f - powf(fPressure / LOC_ALT_P0,
	   										   1.0f / 5.255f));


	   //
	   // Print altitude with three digits of decimal precision.
	   //

	   Nokia5110_SetCursor(0, 3);
	   Nokia5110_OutString("Alt:");

	   Nokia5110_OutFloatp3(fAltitude);

	   // Print Kalman filtered altitude

	   Kalman_Update (&Alt_KState, fAltitude);
	   fK_Altitude = Alt_KState.X;

	  	   //
	  	   // Print altitude with three digits of decimal precision.
	  	   //

	  	   Nokia5110_SetCursor(0, 5);
	  	   Nokia5110_OutString("KAlt:");

	  	 Nokia5110_OutFloatp3(fK_Altitude);

	  	 //calculate variance
	  	 f_meas[count]=fK_Altitude;
	  	 count++;

	  	 if (count>=VAR_COUNT){
	  		 fAlt_Mean = 0.0f;
	  		 fAlt_Var = 0.0f;
	  		 // calculate mean
	  		 for(i=0; i<VAR_COUNT; i++){
	  			 fAlt_Mean = fAlt_Mean +f_meas[i];
	  		 }
	  		 fAlt_Mean = fAlt_Mean/((float) VAR_COUNT);

	  		 // Calculate Var
	  		 for(i=0; i<VAR_COUNT; i++){
	  			 fAlt_Var = fAlt_Var + powf((f_meas[i] - fAlt_Mean),2.0f);
	  		 }
	  		 fAlt_Var = fAlt_Var/((float) VAR_COUNT);



	  		 //
	  		 // Print altitude with three digits of decimal precision.
	  		 //

	  		 Nokia5110_SetCursor(0, 4);
	  		 Nokia5110_OutString("Var:");

	  		 Nokia5110_OutFloatp3(fAlt_Var);

	  		 count = 0;

	  	 }

	   //
	   // Delay to keep printing speed reasonable. About 100msec.
	   //
	   MAP_SysCtlDelay(MAP_SysCtlClockGet() / (10 * 3));
//	   for(delay=0; delay<1000000; delay=delay+1);

   }//while end
}

