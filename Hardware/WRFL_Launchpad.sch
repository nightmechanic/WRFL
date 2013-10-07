EESchema Schematic File Version 2  date Monday, October 07, 2013 09:14:17 AM
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:WRFL
LIBS:WRFL-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 5
Title ""
Date "7 oct 2013"
Rev ""
Comp "Nightmechanic"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_2 P8
U 1 1 524B47CF
P 5900 1550
F 0 "P8" V 5850 1550 40  0000 C CNN
F 1 "1x2_2.54_Fem" V 5950 1550 40  0000 C CNN
F 2 "" H 5900 1550 60  0000 C CNN
F 3 "" H 5900 1550 60  0000 C CNN
	1    5900 1550
	1    0    0    -1  
$EndComp
NoConn ~ 5550 1650
$Comp
L VDD_MCU #U11
U 1 1 524B47E3
P 5200 1100
F 0 "#U11" H 5200 1300 60  0001 C CNN
F 1 "VDD_MCU" H 5200 1200 60  0000 C CNN
F 2 "~" H 5200 1100 60  0000 C CNN
F 3 "~" H 5200 1100 60  0000 C CNN
	1    5200 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 1450 5200 1450
Wire Wire Line
	5200 1450 5200 1300
$Comp
L LAUNCHPAD_J1_J3 P13
U 1 1 525219A4
P 3200 4350
F 0 "P13" V 3150 3850 60  0000 C CNN
F 1 "LAUNCHPAD_J1_J3" V 3250 3850 60  0000 C CNN
F 2 "" H 3200 4350 60  0000 C CNN
F 3 "" H 3200 4350 60  0000 C CNN
	1    3200 4350
	1    0    0    -1  
$EndComp
$Comp
L LAUNCHPAD_J2_J4 P14
U 1 1 525219B3
P 8650 4350
F 0 "P14" V 8600 3850 60  0000 C CNN
F 1 "LAUNCHPAD_J2_J4" V 8700 3850 60  0000 C CNN
F 2 "" H 8650 4350 60  0000 C CNN
F 3 "" H 8650 4350 60  0000 C CNN
	1    8650 4350
	1    0    0    -1  
$EndComp
Text GLabel 3700 2200 0    60   Input ~ 0
ON_SW_MCU
Text GLabel 3700 2450 0    60   Input ~ 0
PM_ENABLE
Text GLabel 3700 2700 0    60   Input ~ 0
VBAT_SENS
Text GLabel 3700 2900 0    60   Input ~ 0
VBUS_SENS
Text GLabel 3700 3150 0    60   Input ~ 0
VBAT_EN
Text GLabel 4900 2200 0    60   Input ~ 0
CHARGR_ISET
Text GLabel 4900 2500 0    60   Input ~ 0
CHARGR_IND
Text GLabel 4900 2750 0    60   Input ~ 0
SENS_SSI_CLK
Text GLabel 4900 3050 0    60   Input ~ 0
SENS_SSI_TX
Text GLabel 4900 3300 0    60   Input ~ 0
SENS_SSI_RX
Text GLabel 6250 2250 0    60   Input ~ 0
BAROMETER_CS
Text GLabel 6250 2450 0    60   Input ~ 0
ACCEL_CS
Text GLabel 6300 2650 0    60   Input ~ 0
WAKE_SW
Text GLabel 6300 2850 0    60   Input ~ 0
QUAD_ENC_A
Text GLabel 6300 3050 0    60   Input ~ 0
QUAD_ENC_B
Text GLabel 10150 2250 0    60   Input ~ 0
USER_SW
Text GLabel 8200 2250 0    60   Input ~ 0
LCD_LED_CTRL
Text GLabel 8200 2500 0    60   Input ~ 0
GEN_SSI_CLK
Text GLabel 8200 2700 0    60   Input ~ 0
GEN_SSI_TX
Text GLabel 8200 2900 0    60   Input ~ 0
GEN_SSI_RX
Text GLabel 9050 2250 0    60   Input ~ 0
LCD_RST
Text GLabel 9050 2400 0    60   Input ~ 0
LCD_CS
Text GLabel 9050 2600 0    60   Input ~ 0
LCD_C_D
Text GLabel 9050 2850 0    60   Input ~ 0
FLASH_CS
NoConn ~ 2550 3950
Text GLabel 8300 3150 0    60   Input ~ 0
SERVO1_CTRL
Text GLabel 9250 3200 0    60   Input ~ 0
SERVO2_CTRL
Text GLabel 10200 2600 0    60   Input ~ 0
VREF_SENS
$Comp
L REF3312 U11
U 1 1 525244B8
P 1800 1300
F 0 "U11" H 1600 1450 60  0000 C CNN
F 1 "REF3312" H 2000 1100 60  0000 C CNN
F 2 "~" H 1800 1300 60  0000 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/ref3312.pdf" H 1800 1650 60  0001 C CNN
	1    1800 1300
	1    0    0    -1  
$EndComp
$Comp
L C C19
U 1 1 525246BE
P 1150 1650
F 0 "C19" H 1150 1750 40  0000 L CNN
F 1 "0.47uF 16V" H 1156 1565 40  0000 L CNN
F 2 "0805" H 1188 1500 30  0001 C CNN
F 3 "~" H 1150 1650 60  0000 C CNN
	1    1150 1650
	1    0    0    -1  
$EndComp
$Comp
L VDD_MCU #U10
U 1 1 525246DC
P 1150 900
F 0 "#U10" H 1150 1100 60  0001 C CNN
F 1 "VDD_MCU" H 1150 1000 60  0000 C CNN
F 2 "~" H 1150 900 60  0000 C CNN
F 3 "~" H 1150 900 60  0000 C CNN
	1    1150 900 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR28
U 1 1 525246EB
P 1150 1950
F 0 "#PWR28" H 1150 1950 30  0001 C CNN
F 1 "GND" H 1150 1880 30  0001 C CNN
F 2 "" H 1150 1950 60  0000 C CNN
F 3 "" H 1150 1950 60  0000 C CNN
	1    1150 1950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR29
U 1 1 525246FA
P 1800 1750
F 0 "#PWR29" H 1800 1750 30  0001 C CNN
F 1 "GND" H 1800 1680 30  0001 C CNN
F 2 "" H 1800 1750 60  0000 C CNN
F 3 "" H 1800 1750 60  0000 C CNN
	1    1800 1750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR30
U 1 1 52524709
P 2650 1850
F 0 "#PWR30" H 2650 1850 30  0001 C CNN
F 1 "GND" H 2650 1780 30  0001 C CNN
F 2 "" H 2650 1850 60  0000 C CNN
F 3 "" H 2650 1850 60  0000 C CNN
	1    2650 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 1100 1150 1450
Wire Wire Line
	1150 1850 1150 1950
Wire Wire Line
	1400 1250 1150 1250
Connection ~ 1150 1250
Wire Wire Line
	1800 1600 1800 1750
Wire Wire Line
	2200 1250 2650 1250
Wire Wire Line
	2650 1250 2650 1350
Wire Wire Line
	2650 1750 2650 1850
$Comp
L C C20
U 1 1 52524762
P 2650 1550
F 0 "C20" H 2650 1650 40  0000 L CNN
F 1 "0.47uF 16V" H 2656 1465 40  0000 L CNN
F 2 "0805" H 2688 1400 30  0001 C CNN
F 3 "~" H 2650 1550 60  0000 C CNN
	1    2650 1550
	1    0    0    -1  
$EndComp
Text GLabel 10150 2850 0    60   Input ~ 0
MEAS_EN
$EndSCHEMATC
