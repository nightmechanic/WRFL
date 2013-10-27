EESchema Schematic File Version 2  date Sunday, October 27, 2013 'AMt' 10:59:49 AM
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
Sheet 4 6
Title ""
Date "27 oct 2013"
Rev ""
Comp "Nightmechanic"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L TSA12110 SW1
U 1 1 5251E149
P 2000 5800
F 0 "SW1" H 2000 6200 60  0000 C CNN
F 1 "TSA12110" H 2000 6100 60  0000 C CNN
F 2 "" H 2000 5780 60  0000 C CNN
F 3 "" H 2000 5780 60  0000 C CNN
	1    2000 5800
	1    0    0    -1  
$EndComp
$Comp
L TSA12110 SW3
U 1 1 5251E158
P 6950 5900
F 0 "SW3" H 6950 6300 60  0000 C CNN
F 1 "TSA12110" H 6950 6200 60  0000 C CNN
F 2 "" H 6950 5880 60  0000 C CNN
F 3 "" H 6950 5880 60  0000 C CNN
	1    6950 5900
	1    0    0    -1  
$EndComp
$Comp
L EN11-HSB SW2
U 1 1 5251E167
P 4450 5050
F 0 "SW2" H 4450 5450 60  0000 C CNN
F 1 "EN11-HSB" H 4450 5350 60  0000 C CNN
F 2 "EN11-HSB" H 4450 5030 60  0001 C CNN
F 3 "" H 4450 5030 60  0000 C CNN
	1    4450 5050
	1    0    0    -1  
$EndComp
$Comp
L SI1012CR Q3
U 1 1 5251E185
P 2850 4000
F 0 "Q3" H 3050 4000 60  0000 C CNN
F 1 "SI1012CR" H 3200 3900 60  0000 C CNN
F 2 "SC-75A" H 2850 4000 60  0001 C CNN
F 3 "http://www.vishay.com/docs/67519/si1012cr.pdf" H 2800 3750 60  0001 C CNN
	1    2850 4000
	1    0    0    -1  
$EndComp
$Comp
L S25FL216K0PMFI040 U9
U 1 1 5251E2A0
P 8250 2250
F 0 "U9" H 8000 2600 60  0000 C CNN
F 1 "S25FL216K0PMFI040" H 8250 1850 60  0000 C CNN
F 2 "" H 8400 2250 60  0000 C CNN
F 3 "" H 8400 2250 60  0000 C CNN
	1    8250 2250
	1    0    0    -1  
$EndComp
Text GLabel 1750 2800 3    60   Input ~ 0
LCD_RST
Text GLabel 1950 2800 3    60   Input ~ 0
LCD_CS
Text GLabel 2150 2800 3    60   Input ~ 0
LCD_C_D
Text GLabel 2350 2800 3    60   Input ~ 0
GEN_SSI_TX
Text GLabel 2550 2800 3    60   Input ~ 0
GEN_SSI_CLK
$Comp
L VDD_PERIPH #U14
U 1 1 5251E2C5
P 950 3350
F 0 "#U14" H 950 3550 60  0001 C CNN
F 1 "VDD_PERIPH" H 950 3450 60  0000 C CNN
F 2 "~" H 950 3350 60  0000 C CNN
F 3 "~" H 950 3350 60  0000 C CNN
	1    950  3350
	1    0    0    -1  
$EndComp
$Comp
L VDD_PERIPH #U18
U 1 1 5251E301
P 9300 1550
F 0 "#U18" H 9300 1750 60  0001 C CNN
F 1 "VDD_PERIPH" H 9300 1650 60  0000 C CNN
F 2 "~" H 9300 1550 60  0000 C CNN
F 3 "~" H 9300 1550 60  0000 C CNN
	1    9300 1550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR33
U 1 1 5251E310
P 950 4350
F 0 "#PWR33" H 950 4350 30  0001 C CNN
F 1 "GND" H 950 4280 30  0001 C CNN
F 2 "" H 950 4350 60  0000 C CNN
F 3 "" H 950 4350 60  0000 C CNN
	1    950  4350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR34
U 1 1 5251E31F
P 2600 6250
F 0 "#PWR34" H 2600 6250 30  0001 C CNN
F 1 "GND" H 2600 6180 30  0001 C CNN
F 2 "" H 2600 6250 60  0000 C CNN
F 3 "" H 2600 6250 60  0000 C CNN
	1    2600 6250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR37
U 1 1 5251E32E
P 4450 5950
F 0 "#PWR37" H 4450 5950 30  0001 C CNN
F 1 "GND" H 4450 5880 30  0001 C CNN
F 2 "" H 4450 5950 60  0000 C CNN
F 3 "" H 4450 5950 60  0000 C CNN
	1    4450 5950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR38
U 1 1 5251E33D
P 6500 6400
F 0 "#PWR38" H 6500 6400 30  0001 C CNN
F 1 "GND" H 6500 6330 30  0001 C CNN
F 2 "" H 6500 6400 60  0000 C CNN
F 3 "" H 6500 6400 60  0000 C CNN
	1    6500 6400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR36
U 1 1 5251E34C
P 3150 2500
F 0 "#PWR36" H 3150 2500 30  0001 C CNN
F 1 "GND" H 3150 2430 30  0001 C CNN
F 2 "" H 3150 2500 60  0000 C CNN
F 3 "" H 3150 2500 60  0000 C CNN
	1    3150 2500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR39
U 1 1 5251E35B
P 7500 2700
F 0 "#PWR39" H 7500 2700 30  0001 C CNN
F 1 "GND" H 7500 2630 30  0001 C CNN
F 2 "" H 7500 2700 60  0000 C CNN
F 3 "" H 7500 2700 60  0000 C CNN
	1    7500 2700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR35
U 1 1 5251E3CC
P 2950 4350
F 0 "#PWR35" H 2950 4350 30  0001 C CNN
F 1 "GND" H 2950 4280 30  0001 C CNN
F 2 "" H 2950 4350 60  0000 C CNN
F 3 "" H 2950 4350 60  0000 C CNN
	1    2950 4350
	1    0    0    -1  
$EndComp
$Comp
L C C17
U 1 1 5251E3F3
P 950 4000
F 0 "C17" H 950 4100 40  0000 L CNN
F 1 "1uF 10V" H 956 3915 40  0000 L CNN
F 2 "0805" H 988 3850 30  0001 C CNN
F 3 "~" H 950 4000 60  0000 C CNN
	1    950  4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  4200 950  4350
Text GLabel 2300 4000 0    60   Input ~ 0
LCD_LED_CTRL
Text Label 2550 2700 1    60   ~ 0
SPI_CLK
Wire Wire Line
	8850 2350 9750 2350
Text Label 9150 2350 0    60   ~ 0
SPI_CLK
Wire Wire Line
	8850 2500 9750 2500
Text Label 9200 2500 0    60   ~ 0
SPI_TX
Wire Wire Line
	9300 2050 8850 2050
Wire Wire Line
	9300 1750 9300 2200
Wire Wire Line
	9300 2200 8850 2200
Connection ~ 9300 2050
Wire Wire Line
	7650 2500 7500 2500
Wire Wire Line
	7500 2500 7500 2700
$Comp
L C C18
U 1 1 5251EA41
P 10150 2100
F 0 "C18" H 10150 2200 40  0000 L CNN
F 1 "0.47uF 16V" H 10156 2015 40  0000 L CNN
F 2 "0805" H 10188 1950 30  0001 C CNN
F 3 "~" H 10150 2100 60  0000 C CNN
	1    10150 2100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR43
U 1 1 5251EA53
P 10150 2500
F 0 "#PWR43" H 10150 2500 30  0001 C CNN
F 1 "GND" H 10150 2430 30  0001 C CNN
F 2 "" H 10150 2500 60  0000 C CNN
F 3 "" H 10150 2500 60  0000 C CNN
	1    10150 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 2300 10150 2500
Wire Wire Line
	10150 1850 10150 1900
$Comp
L VDD_PERIPH #U16
U 1 1 5251EAB9
P 5950 1150
F 0 "#U16" H 5950 1350 60  0001 C CNN
F 1 "VDD_PERIPH" H 5950 1250 60  0000 C CNN
F 2 "~" H 5950 1150 60  0000 C CNN
F 3 "~" H 5950 1150 60  0000 C CNN
	1    5950 1150
	1    0    0    -1  
$EndComp
$Comp
L R R18
U 1 1 5251EAC1
P 7300 1700
F 0 "R18" V 7380 1700 40  0000 C CNN
F 1 "100k" V 7307 1701 40  0000 C CNN
F 2 "0805" V 7230 1700 30  0001 C CNN
F 3 "~" H 7300 1700 30  0000 C CNN
	1    7300 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 2050 7650 2050
Wire Wire Line
	7300 2050 7300 1950
Wire Wire Line
	7300 1400 7300 1450
Wire Wire Line
	5950 1400 7300 1400
Wire Wire Line
	5950 2350 7650 2350
Text GLabel 6700 2200 0    60   Input ~ 0
GEN_SSI_RX
Text GLabel 6700 2050 0    60   Input ~ 0
FLASH_CS
Wire Wire Line
	7650 2200 6700 2200
Connection ~ 7300 2050
Wire Wire Line
	6550 6100 6500 6100
Wire Wire Line
	6500 6100 6500 6400
Wire Wire Line
	7350 6100 7450 6100
Wire Wire Line
	7450 6100 7450 6300
Wire Wire Line
	7450 6300 6500 6300
Connection ~ 6500 6300
Wire Wire Line
	2400 6000 2600 6000
Wire Wire Line
	2600 6000 2600 6250
Wire Wire Line
	2600 6150 1450 6150
Wire Wire Line
	1450 6150 1450 6000
Wire Wire Line
	1450 6000 1600 6000
Connection ~ 2600 6150
Wire Wire Line
	2550 5600 2400 5600
Text GLabel 1200 5600 0    60   Input ~ 0
WAKE_SW
Text GLabel 3650 4950 0    60   Input ~ 0
ON_SW_FET
Text GLabel 4000 6000 0    60   Input ~ 0
QUAD_ENC_A
Text GLabel 4950 6000 2    60   Input ~ 0
QUAD_ENC_B
Text GLabel 6200 5700 0    60   Input ~ 0
USER_SW
Wire Wire Line
	7500 5300 6350 5300
Wire Wire Line
	6350 5300 6350 5700
Connection ~ 6350 5700
Wire Wire Line
	1200 5600 1600 5600
Wire Wire Line
	4450 5750 4450 5950
Wire Wire Line
	4700 5750 4700 6200
Wire Wire Line
	4700 6000 4950 6000
Wire Wire Line
	4200 5750 4200 6200
Wire Wire Line
	4200 6000 4000 6000
$Comp
L R R16
U 1 1 5251EEC5
P 4200 6450
F 0 "R16" V 4280 6450 40  0000 C CNN
F 1 "100k" V 4207 6451 40  0000 C CNN
F 2 "0805" V 4130 6450 30  0001 C CNN
F 3 "~" H 4200 6450 30  0000 C CNN
	1    4200 6450
	1    0    0    -1  
$EndComp
$Comp
L R R17
U 1 1 5251EECB
P 4700 6450
F 0 "R17" V 4780 6450 40  0000 C CNN
F 1 "100k" V 4707 6451 40  0000 C CNN
F 2 "0805" V 4630 6450 30  0001 C CNN
F 3 "~" H 4700 6450 30  0000 C CNN
	1    4700 6450
	1    0    0    -1  
$EndComp
Connection ~ 4200 6000
Connection ~ 4700 6000
Wire Wire Line
	3150 6800 4700 6800
Wire Wire Line
	4700 6800 4700 6700
Wire Wire Line
	4200 6700 4200 6800
Connection ~ 4200 6800
Wire Wire Line
	4050 4950 3650 4950
Wire Wire Line
	2550 5600 2550 5250
Wire Wire Line
	2550 5250 1500 5250
Wire Wire Line
	1500 5250 1500 5600
Connection ~ 1500 5600
Text GLabel 5450 4950 2    60   Input ~ 0
ON_SW_MCU
Wire Wire Line
	5450 4950 4850 4950
$Comp
L CONN_3X2 P16
U 1 1 52523A58
P 9250 4450
F 0 "P16" H 9250 4700 50  0000 C CNN
F 1 "2x3_2_54_RA_MALE" H 9250 4300 40  0000 C CNN
F 2 "" H 9250 4450 60  0000 C CNN
F 3 "" H 9250 4450 60  0000 C CNN
	1    9250 4450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR42
U 1 1 52523A67
P 9800 4700
F 0 "#PWR42" H 9800 4700 30  0001 C CNN
F 1 "GND" H 9800 4630 30  0001 C CNN
F 2 "" H 9800 4700 60  0000 C CNN
F 3 "" H 9800 4700 60  0000 C CNN
	1    9800 4700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR41
U 1 1 52523A76
P 8750 4700
F 0 "#PWR41" H 8750 4700 30  0001 C CNN
F 1 "GND" H 8750 4630 30  0001 C CNN
F 2 "" H 8750 4700 60  0000 C CNN
F 3 "" H 8750 4700 60  0000 C CNN
	1    8750 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 4500 8750 4500
Wire Wire Line
	8750 4500 8750 4700
Wire Wire Line
	9650 4500 9800 4500
Wire Wire Line
	9800 4500 9800 4700
$Comp
L V_BOOST #U17
U 1 1 52523B29
P 8300 4200
F 0 "#U17" H 8300 4400 60  0001 C CNN
F 1 "V_BOOST" H 8300 4300 60  0000 C CNN
F 2 "~" H 8300 4200 60  0000 C CNN
F 3 "~" H 8300 4200 60  0000 C CNN
	1    8300 4200
	1    0    0    -1  
$EndComp
$Comp
L V_BOOST #U19
U 1 1 52523B38
P 10200 4200
F 0 "#U19" H 10200 4400 60  0001 C CNN
F 1 "V_BOOST" H 10200 4300 60  0000 C CNN
F 2 "~" H 10200 4200 60  0000 C CNN
F 3 "~" H 10200 4200 60  0000 C CNN
	1    10200 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 4400 10200 4400
Wire Wire Line
	8300 4400 8850 4400
Text GLabel 8500 3850 0    60   Input ~ 0
SERVO1_CTRL
Text GLabel 10000 3850 2    60   Input ~ 0
SERVO2_CTRL
Wire Wire Line
	10000 3850 9800 3850
Wire Wire Line
	9800 3850 9800 4300
Wire Wire Line
	9800 4300 9650 4300
Wire Wire Line
	8500 3850 8650 3850
Wire Wire Line
	8650 3850 8650 4300
Wire Wire Line
	8650 4300 8850 4300
$Comp
L C C21
U 1 1 52526585
P 8300 4900
F 0 "C21" H 8300 5000 40  0000 L CNN
F 1 "10uF 16V" H 8306 4815 40  0000 L CNN
F 2 "0805" H 8338 4750 30  0001 C CNN
F 3 "~" H 8300 4900 60  0000 C CNN
	1    8300 4900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR40
U 1 1 52526592
P 8300 5350
F 0 "#PWR40" H 8300 5350 30  0001 C CNN
F 1 "GND" H 8300 5280 30  0001 C CNN
F 2 "" H 8300 5350 60  0000 C CNN
F 3 "" H 8300 5350 60  0000 C CNN
	1    8300 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 4700 8300 4400
Wire Wire Line
	8300 5100 8300 5350
$Comp
L NOKIA_5110_LCD DS1
U 1 1 52572B07
P 2450 1550
F 0 "DS1" H 1750 2350 60  0000 C CNN
F 1 "NOKIA_5110_LCD" H 2900 2350 60  0000 C CNN
F 2 "" H 2300 1400 60  0000 C CNN
F 3 "" H 2300 1400 60  0000 C CNN
	1    2450 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 2300 2550 2800
Wire Wire Line
	2350 2800 2350 2300
Wire Wire Line
	2150 2300 2150 2800
Wire Wire Line
	1950 2800 1950 2300
Wire Wire Line
	1750 2300 1750 2800
Text Label 2350 2700 1    60   ~ 0
SPI_TX
Wire Wire Line
	2750 2300 2750 3650
Wire Wire Line
	2750 3650 950  3650
Wire Wire Line
	950  3550 950  3800
Connection ~ 950  3650
Wire Wire Line
	2300 4000 2650 4000
Wire Wire Line
	3150 2300 3150 2500
Wire Wire Line
	2950 2300 2950 3800
Wire Wire Line
	2950 4200 2950 4350
Wire Wire Line
	5950 1350 5950 2350
Connection ~ 5950 1400
Wire Wire Line
	10150 1850 9300 1850
Connection ~ 9300 1850
$Comp
L VDD_PERIPH #U15
U 1 1 525D7AAA
P 3150 6600
F 0 "#U15" H 3150 6800 60  0001 C CNN
F 1 "VDD_PERIPH" H 3150 6700 60  0000 C CNN
F 2 "~" H 3150 6600 60  0000 C CNN
F 3 "~" H 3150 6600 60  0000 C CNN
	1    3150 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 5700 7500 5700
Wire Wire Line
	7500 5700 7500 5300
Wire Wire Line
	6200 5700 6550 5700
$EndSCHEMATC
