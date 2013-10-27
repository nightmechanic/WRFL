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
Sheet 6 6
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
L HM-10 U8
U 1 1 52571F80
P 5500 3800
F 0 "U8" H 4400 5550 60  0000 C CNN
F 1 "HM-10" H 6150 5550 60  0000 C CNN
F 2 "" H 5400 6100 60  0000 C CNN
F 3 "ftp://imall.iteadstudio.com/IM120417010_BT_Shield_v2.2/DS_BluetoothHC05.pdf" H 5500 5750 60  0001 C CNN
	1    5500 3800
	1    0    0    -1  
$EndComp
$Comp
L VDD_PERIPH #U21
U 1 1 52571F8F
P 2100 4600
F 0 "#U21" H 2100 4800 60  0001 C CNN
F 1 "VDD_PERIPH" H 2100 4700 60  0000 C CNN
F 2 "~" H 2100 4600 60  0000 C CNN
F 3 "~" H 2100 4600 60  0000 C CNN
	1    2100 4600
	1    0    0    -1  
$EndComp
$Comp
L C C22
U 1 1 52571F9E
P 2500 5050
F 0 "C22" H 2500 5150 40  0000 L CNN
F 1 "1uF 16V" H 2506 4965 40  0000 L CNN
F 2 "0805" H 2538 4900 30  0001 C CNN
F 3 "~" H 2500 5050 60  0000 C CNN
	1    2500 5050
	1    0    0    -1  
$EndComp
$Comp
L R R23
U 1 1 52571FAD
P 7850 4800
F 0 "R23" V 7930 4800 40  0000 C CNN
F 1 "2k" V 7857 4801 40  0000 C CNN
F 2 "0805" V 7780 4800 30  0001 C CNN
F 3 "~" H 7850 4800 30  0000 C CNN
	1    7850 4800
	0    1    1    0   
$EndComp
$Comp
L GND #PWR58
U 1 1 52571FBC
P 3850 5350
F 0 "#PWR58" H 3850 5350 30  0001 C CNN
F 1 "GND" H 3850 5280 30  0001 C CNN
F 2 "" H 3850 5350 60  0000 C CNN
F 3 "" H 3850 5350 60  0000 C CNN
	1    3850 5350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR59
U 1 1 52571FCB
P 4800 5850
F 0 "#PWR59" H 4800 5850 30  0001 C CNN
F 1 "GND" H 4800 5780 30  0001 C CNN
F 2 "" H 4800 5850 60  0000 C CNN
F 3 "" H 4800 5850 60  0000 C CNN
	1    4800 5850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR60
U 1 1 52571FDA
P 6200 5800
F 0 "#PWR60" H 6200 5800 30  0001 C CNN
F 1 "GND" H 6200 5730 30  0001 C CNN
F 2 "" H 6200 5800 60  0000 C CNN
F 3 "" H 6200 5800 60  0000 C CNN
	1    6200 5800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR61
U 1 1 52571FE9
P 7200 5250
F 0 "#PWR61" H 7200 5250 30  0001 C CNN
F 1 "GND" H 7200 5180 30  0001 C CNN
F 2 "" H 7200 5250 60  0000 C CNN
F 3 "" H 7200 5250 60  0000 C CNN
	1    7200 5250
	1    0    0    -1  
$EndComp
$Comp
L VDD_PERIPH #U23
U 1 1 52571FF6
P 8700 4500
F 0 "#U23" H 8700 4700 60  0001 C CNN
F 1 "VDD_PERIPH" H 8700 4600 60  0000 C CNN
F 2 "~" H 8700 4500 60  0000 C CNN
F 3 "~" H 8700 4500 60  0000 C CNN
	1    8700 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 5000 7200 5000
Wire Wire Line
	7200 5000 7200 5250
Wire Wire Line
	6200 5550 6200 5800
Wire Wire Line
	4800 5550 4800 5850
Wire Wire Line
	4050 5000 3850 5000
Wire Wire Line
	3850 5000 3850 5350
Wire Wire Line
	4050 4800 2100 4800
Wire Wire Line
	2500 4850 2500 4800
Connection ~ 2500 4800
$Comp
L GND #PWR57
U 1 1 5257201B
P 2500 5350
F 0 "#PWR57" H 2500 5350 30  0001 C CNN
F 1 "GND" H 2500 5280 30  0001 C CNN
F 2 "" H 2500 5350 60  0000 C CNN
F 3 "" H 2500 5350 60  0000 C CNN
	1    2500 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 5250 2500 5350
Wire Wire Line
	6900 4800 7600 4800
Wire Wire Line
	8100 4800 8700 4800
Wire Wire Line
	8700 4800 8700 4700
NoConn ~ 5000 5550
NoConn ~ 5200 5550
NoConn ~ 5400 5550
NoConn ~ 5600 5550
NoConn ~ 5800 5550
NoConn ~ 6000 5550
NoConn ~ 6900 4600
NoConn ~ 6900 4400
NoConn ~ 6900 4200
NoConn ~ 6900 4000
NoConn ~ 6900 3800
NoConn ~ 6900 3600
NoConn ~ 6900 3400
NoConn ~ 6900 3200
NoConn ~ 6900 3000
NoConn ~ 6900 2800
NoConn ~ 6900 2600
NoConn ~ 4050 3400
NoConn ~ 4050 3600
NoConn ~ 4050 3800
NoConn ~ 4050 4000
NoConn ~ 4050 4200
NoConn ~ 4050 4400
Text GLabel 2750 2600 0    60   Input ~ 0
BT_UART_RX
Text GLabel 2750 2800 0    60   Input ~ 0
BT_UART_TX
Text GLabel 1500 4150 0    60   Input ~ 0
BT_RESET
$Comp
L R R22
U 1 1 5257207A
P 2200 4150
F 0 "R22" V 2280 4150 40  0000 C CNN
F 1 "ZEROHM" V 2207 4151 40  0000 C CNN
F 2 "0805" V 2130 4150 30  0001 C CNN
F 3 "~" H 2200 4150 30  0000 C CNN
	1    2200 4150
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 2600 4050 2600
Wire Wire Line
	4050 2800 2750 2800
Wire Wire Line
	2350 3000 4050 3000
Wire Wire Line
	1950 4150 1500 4150
$Comp
L R R21
U 1 1 526452CC
P 3200 4050
F 0 "R21" V 3280 4050 40  0000 C CNN
F 1 "100k" V 3207 4051 40  0000 C CNN
F 2 "0805" V 3130 4050 30  0001 C CNN
F 3 "~" H 3200 4050 30  0000 C CNN
	1    3200 4050
	1    0    0    -1  
$EndComp
$Comp
L VDD_PERIPH #U22
U 1 1 526452D2
P 3200 3500
F 0 "#U22" H 3200 3700 60  0001 C CNN
F 1 "VDD_PERIPH" H 3200 3600 60  0000 C CNN
F 2 "~" H 3200 3500 60  0000 C CNN
F 3 "~" H 3200 3500 60  0000 C CNN
	1    3200 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 4600 4050 4600
Wire Wire Line
	3200 4600 3200 4300
Wire Wire Line
	3200 3700 3200 3800
Wire Wire Line
	2450 4150 2800 4150
Wire Wire Line
	2800 4150 2800 4600
Connection ~ 3200 4600
$Comp
L CONN_1 P15
U 1 1 5264532F
P 2200 3000
F 0 "P15" H 2280 3000 40  0000 L CNN
F 1 "TEST_POINT" H 2200 3055 30  0001 C CNN
F 2 "" H 2200 3000 60  0000 C CNN
F 3 "" H 2200 3000 60  0000 C CNN
	1    2200 3000
	-1   0    0    1   
$EndComp
$Comp
L CONN_1 P18
U 1 1 52645349
P 2200 3200
F 0 "P18" H 2280 3200 40  0000 L CNN
F 1 "TEST_POINT" H 2200 3255 30  0001 C CNN
F 2 "" H 2200 3200 60  0000 C CNN
F 3 "" H 2200 3200 60  0000 C CNN
	1    2200 3200
	-1   0    0    1   
$EndComp
Wire Wire Line
	2350 3200 4050 3200
$EndSCHEMATC
