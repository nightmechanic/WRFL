EESchema Schematic File Version 2  date 10/11/2013 1:57:23 AM
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
Date "10 oct 2013"
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
L VDD_PERIPH #U23
U 1 1 52571F8F
P 2100 4600
F 0 "#U23" H 2100 4800 60  0001 C CNN
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
L GND #PWR52
U 1 1 52571FBC
P 3850 5350
F 0 "#PWR52" H 3850 5350 30  0001 C CNN
F 1 "GND" H 3850 5280 30  0001 C CNN
F 2 "" H 3850 5350 60  0000 C CNN
F 3 "" H 3850 5350 60  0000 C CNN
	1    3850 5350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR53
U 1 1 52571FCB
P 4800 5850
F 0 "#PWR53" H 4800 5850 30  0001 C CNN
F 1 "GND" H 4800 5780 30  0001 C CNN
F 2 "" H 4800 5850 60  0000 C CNN
F 3 "" H 4800 5850 60  0000 C CNN
	1    4800 5850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR54
U 1 1 52571FDA
P 6200 5800
F 0 "#PWR54" H 6200 5800 30  0001 C CNN
F 1 "GND" H 6200 5730 30  0001 C CNN
F 2 "" H 6200 5800 60  0000 C CNN
F 3 "" H 6200 5800 60  0000 C CNN
	1    6200 5800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR55
U 1 1 52571FE9
P 7200 5250
F 0 "#PWR55" H 7200 5250 30  0001 C CNN
F 1 "GND" H 7200 5180 30  0001 C CNN
F 2 "" H 7200 5250 60  0000 C CNN
F 3 "" H 7200 5250 60  0000 C CNN
	1    7200 5250
	1    0    0    -1  
$EndComp
$Comp
L VDD_PERIPH #U24
U 1 1 52571FF6
P 8700 4500
F 0 "#U24" H 8700 4700 60  0001 C CNN
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
L GND #PWR51
U 1 1 5257201B
P 2500 5350
F 0 "#PWR51" H 2500 5350 30  0001 C CNN
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
NoConn ~ 4050 4600
Text GLabel 2750 2600 0    60   Input ~ 0
BT_UART_RX
Text GLabel 2750 2800 0    60   Input ~ 0
BT_UART_TX
Text GLabel 2750 3000 0    60   Input ~ 0
BT_UART_RTS
Text GLabel 2750 3200 0    60   Input ~ 0
BT_UART_CTS
$Comp
L R R22
U 1 1 5257207A
P 3450 3200
F 0 "R22" V 3530 3200 40  0000 C CNN
F 1 "ZEROHM" V 3457 3201 40  0000 C CNN
F 2 "0805" V 3380 3200 30  0001 C CNN
F 3 "~" H 3450 3200 30  0000 C CNN
	1    3450 3200
	0    1    1    0   
$EndComp
$Comp
L R R21
U 1 1 5257208C
P 3450 3000
F 0 "R21" V 3530 3000 40  0000 C CNN
F 1 "ZEROHM" V 3457 3001 40  0000 C CNN
F 2 "0805" V 3380 3000 30  0001 C CNN
F 3 "~" H 3450 3000 30  0000 C CNN
	1    3450 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	2750 2600 4050 2600
Wire Wire Line
	4050 2800 2750 2800
Wire Wire Line
	2750 3000 3200 3000
Wire Wire Line
	3700 3000 4050 3000
Wire Wire Line
	4050 3200 3700 3200
Wire Wire Line
	3200 3200 2750 3200
$EndSCHEMATC