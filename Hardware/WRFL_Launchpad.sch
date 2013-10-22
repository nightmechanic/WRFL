EESchema Schematic File Version 2  date Tuesday, October 22, 2013 02:02:38 PM
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
Sheet 3 6
Title ""
Date "22 oct 2013"
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
L VDD_MCU #U040
U 1 1 524B47E3
P 5200 1100
F 0 "#U040" H 5200 1300 60  0001 C CNN
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
Text GLabel 4450 5750 2    60   Input ~ 0
ON_SW_MCU
Text GLabel 4450 5350 2    60   Input ~ 0
PM_ENABLE
Text GLabel 1950 4750 0    60   Input ~ 0
VBAT_SENS
Text GLabel 1950 4150 0    60   Input ~ 0
VBUS_SENS
Text GLabel 4450 5150 2    60   Input ~ 0
VBAT_EN
Text GLabel 1900 5350 0    60   Input ~ 0
CHARGR_ISET
Text GLabel 1900 5550 0    60   Input ~ 0
CHARGR_IND
Text GLabel 1950 5150 0    60   Input ~ 0
SENS_SSI_CLK
Text GLabel 9900 4950 2    60   Input ~ 0
SENS_SSI_TX
Text GLabel 9900 5150 2    60   Input ~ 0
SENS_SSI_RX
Text GLabel 9900 4150 2    60   Input ~ 0
BAROMETER_CS
Text GLabel 9900 4350 2    60   Input ~ 0
ACCEL_CS
Text GLabel 9900 4550 2    60   Input ~ 0
WAKE_SW
Text GLabel 7450 5350 0    60   Input ~ 0
QUAD_ENC_A
Text GLabel 7450 5550 0    60   Input ~ 0
QUAD_ENC_B
Text GLabel 9900 5750 2    60   Input ~ 0
USER_SW
Text GLabel 4450 4550 2    60   Input ~ 0
LCD_LED_CTRL
Text GLabel 4450 4350 2    60   Input ~ 0
GEN_SSI_CLK
Text GLabel 4450 4950 2    60   Input ~ 0
GEN_SSI_TX
Text GLabel 4450 4750 2    60   Input ~ 0
GEN_SSI_RX
Text GLabel 7400 5150 0    60   Input ~ 0
LCD_RST
Text GLabel 7400 4750 0    60   Input ~ 0
LCD_CS
Text GLabel 7400 4950 0    60   Input ~ 0
LCD_C_D
Text GLabel 7400 5750 0    60   Input ~ 0
FLASH_CS
NoConn ~ 2550 3950
Text GLabel 7450 3950 0    60   Input ~ 0
SERVO1_CTRL
Text GLabel 7450 4150 0    60   Input ~ 0
SERVO2_CTRL
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
L VDD_MCU #U041
U 1 1 525246DC
P 1150 900
F 0 "#U041" H 1150 1100 60  0001 C CNN
F 1 "VDD_MCU" H 1150 1000 60  0000 C CNN
F 2 "~" H 1150 900 60  0000 C CNN
F 3 "~" H 1150 900 60  0000 C CNN
	1    1150 900 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR042
U 1 1 525246FA
P 1800 1750
F 0 "#PWR042" H 1800 1750 30  0001 C CNN
F 1 "GND" H 1800 1680 30  0001 C CNN
F 2 "" H 1800 1750 60  0000 C CNN
F 3 "" H 1800 1750 60  0000 C CNN
	1    1800 1750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR043
U 1 1 52524709
P 2650 1850
F 0 "#PWR043" H 2650 1850 30  0001 C CNN
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
	2200 1250 3150 1250
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
Text GLabel 4450 5550 2    60   Input ~ 0
MEAS_EN
Text GLabel 1950 4550 0    60   Input ~ 0
BT_UART_TX
Text GLabel 1950 4350 0    60   Input ~ 0
BT_UART_RX
Text GLabel 7450 4350 0    60   Input ~ 0
BT_RESET
Text GLabel 3150 1250 2    60   Input ~ 0
VREF_SENS
Connection ~ 2650 1250
Text GLabel 1950 4950 0    60   Input ~ 0
VREF_SENS
$Comp
L VBUS #U044
U 1 1 526426B1
P 4300 3600
F 0 "#U044" H 4300 3800 60  0001 C CNN
F 1 "VBUS" H 4300 3700 60  0000 C CNN
F 2 "~" H 4300 3600 60  0000 C CNN
F 3 "~" H 4300 3600 60  0000 C CNN
	1    4300 3600
	1    0    0    -1  
$EndComp
NoConn ~ 9300 4750
NoConn ~ 12850 4200
NoConn ~ 13100 4300
Wire Wire Line
	2550 4150 1950 4150
Wire Wire Line
	1950 4350 2550 4350
Wire Wire Line
	2550 4550 1950 4550
Wire Wire Line
	1950 4750 2550 4750
Wire Wire Line
	2550 4950 1950 4950
Wire Wire Line
	1950 5150 2550 5150
Wire Wire Line
	4450 4350 3850 4350
Wire Wire Line
	4450 4550 3850 4550
Wire Wire Line
	4450 4750 3850 4750
Wire Wire Line
	4450 4950 3850 4950
Wire Wire Line
	3850 3950 4300 3950
Wire Wire Line
	4300 3950 4300 3800
$Comp
L GND #PWR045
U 1 1 52642A59
P 4300 4200
F 0 "#PWR045" H 4300 4200 30  0001 C CNN
F 1 "GND" H 4300 4130 30  0001 C CNN
F 2 "" H 4300 4200 60  0000 C CNN
F 3 "" H 4300 4200 60  0000 C CNN
	1    4300 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 4150 4600 4150
Wire Wire Line
	4300 4150 4300 4200
Wire Wire Line
	8000 3950 7450 3950
Wire Wire Line
	7450 4150 8000 4150
Wire Wire Line
	8000 5350 7450 5350
Wire Wire Line
	8000 5550 7450 5550
Wire Wire Line
	9300 4550 9900 4550
Wire Wire Line
	9300 4950 9900 4950
Wire Wire Line
	9900 5150 9300 5150
$Comp
L GND #PWR046
U 1 1 52642B7B
P 9600 4000
F 0 "#PWR046" H 9600 4000 30  0001 C CNN
F 1 "GND" H 9600 3930 30  0001 C CNN
F 2 "" H 9600 4000 60  0000 C CNN
F 3 "" H 9600 4000 60  0000 C CNN
	1    9600 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 3950 9600 3950
Wire Wire Line
	9600 3950 9600 4000
Wire Wire Line
	2550 5350 1900 5350
Wire Wire Line
	2550 5550 1900 5550
Wire Wire Line
	3850 5150 4450 5150
Wire Wire Line
	4450 5350 3850 5350
Wire Wire Line
	3850 5550 4450 5550
Wire Wire Line
	4450 5750 3850 5750
Wire Wire Line
	9900 4150 9300 4150
Wire Wire Line
	9300 4350 9900 4350
Wire Wire Line
	8000 5150 7400 5150
Wire Wire Line
	8000 4950 7400 4950
Wire Wire Line
	7400 4750 8000 4750
Wire Wire Line
	8000 5750 7400 5750
Wire Wire Line
	8000 4350 7450 4350
NoConn ~ 9300 5350
NoConn ~ 9300 5550
NoConn ~ 8000 4550
NoConn ~ 2550 5750
Wire Wire Line
	9900 5750 9300 5750
$Comp
L GND #PWR047
U 1 1 52664487
P 1150 1950
F 0 "#PWR047" H 1150 1950 30  0001 C CNN
F 1 "GND" H 1150 1880 30  0001 C CNN
F 2 "" H 1150 1950 60  0000 C CNN
F 3 "" H 1150 1950 60  0000 C CNN
	1    1150 1950
	1    0    0    -1  
$EndComp
$Comp
L PWR_FLAG #FLG048
U 1 1 52665EE1
P 4600 4000
F 0 "#FLG048" H 4600 4095 30  0001 C CNN
F 1 "PWR_FLAG" H 4600 4180 30  0000 C CNN
F 2 "" H 4600 4000 60  0000 C CNN
F 3 "" H 4600 4000 60  0000 C CNN
	1    4600 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 4150 4600 4000
Connection ~ 4300 4150
$EndSCHEMATC
