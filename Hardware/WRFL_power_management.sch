EESchema Schematic File Version 2  date Thursday, October 03, 2013 02:41:38 PM
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
Sheet 2 5
Title "WRFL power management"
Date "3 oct 2013"
Rev "1.0"
Comp "Nightmechanic"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L BQ24092DGQ U?
U 1 1 524B2D7D
P 2450 1500
F 0 "U?" H 2200 1950 60  0000 C CNN
F 1 "BQ24092DGQ" H 2450 1850 60  0000 C CNN
F 2 "" H 2450 1500 60  0000 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/bq24092.pdf" H 2450 2100 60  0001 C CNN
	1    2450 1500
	1    0    0    -1  
$EndComp
$Comp
L SI2333DDS Q?
U 1 1 524B2D9B
P 5800 1400
F 0 "Q?" V 6100 1200 60  0000 C CNN
F 1 "SI2333DDS" V 6000 1400 60  0000 C CNN
F 2 "" H 5800 1400 60  0000 C CNN
F 3 "http://www.vishay.com/docs/63861/si2333dds.pdf" H 5750 1100 60  0001 C BNN
	1    5800 1400
	0    1    -1   0   
$EndComp
$Comp
L SI1012CR Q?
U 1 1 524B2DAA
P 5700 2450
F 0 "Q?" H 5900 2450 60  0000 C CNN
F 1 "SI1012CR" H 6050 2350 60  0000 C CNN
F 2 "" H 5700 2450 60  0000 C CNN
F 3 "http://www.vishay.com/docs/67519/si1012cr.pdf" H 5650 2200 60  0001 C CNN
	1    5700 2450
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 524B2DCE
P 5250 1650
F 0 "R?" V 5330 1650 40  0000 C CNN
F 1 "1M" V 5257 1651 40  0000 C CNN
F 2 "~" V 5180 1650 30  0000 C CNN
F 3 "~" H 5250 1650 30  0000 C CNN
	1    5250 1650
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 524B2DDD
P 5100 2850
F 0 "R?" V 5180 2850 40  0000 C CNN
F 1 "100k" V 5107 2851 40  0000 C CNN
F 2 "~" V 5030 2850 30  0000 C CNN
F 3 "~" H 5100 2850 30  0000 C CNN
	1    5100 2850
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P?
U 1 1 524B2E40
P 4150 800
F 0 "P?" V 4100 800 40  0000 C CNN
F 1 "JST_XH_2" V 4200 800 40  0000 C CNN
F 2 "" H 4150 800 60  0000 C CNN
F 3 "" H 4150 800 60  0000 C CNN
	1    4150 800 
	0    -1   -1   0   
$EndComp
$Comp
L CONN_2 P?
U 1 1 524B2E50
P 3450 800
F 0 "P?" V 3400 800 40  0000 C CNN
F 1 "JST_XH_2" V 3500 800 40  0000 C CNN
F 2 "" H 3450 800 60  0000 C CNN
F 3 "" H 3450 800 60  0000 C CNN
	1    3450 800 
	0    -1   -1   0   
$EndComp
$Comp
L TPS78233DDC U?
U 1 1 524B2E7C
P 7500 1400
F 0 "U?" H 7350 1600 60  0000 C CNN
F 1 "TPS78233DDC" H 7500 1200 60  0000 C CNN
F 2 "" H 7500 1400 60  0000 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tps78233.pdf" H 7550 1100 60  0001 C CNN
	1    7500 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 1300 5600 1300
Wire Wire Line
	5250 1400 5250 1300
Connection ~ 5250 1300
Wire Wire Line
	3350 1150 3350 1300
Connection ~ 3350 1300
Wire Wire Line
	2900 1400 4050 1400
Wire Wire Line
	4050 1400 4050 1150
Wire Wire Line
	5800 1600 5800 2250
Wire Wire Line
	5250 1900 5250 2050
Wire Wire Line
	5250 2050 5800 2050
Connection ~ 5800 2050
$Comp
L GND #PWR?
U 1 1 524B2F1B
P 5800 3200
F 0 "#PWR?" H 5800 3200 30  0001 C CNN
F 1 "GND" H 5800 3130 30  0001 C CNN
F 2 "" H 5800 3200 60  0000 C CNN
F 3 "" H 5800 3200 60  0000 C CNN
	1    5800 3200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 524B2F2A
P 5100 3200
F 0 "#PWR?" H 5100 3200 30  0001 C CNN
F 1 "GND" H 5100 3130 30  0001 C CNN
F 2 "" H 5100 3200 60  0000 C CNN
F 3 "" H 5100 3200 60  0000 C CNN
	1    5100 3200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 524B2F39
P 3550 1150
F 0 "#PWR?" H 3550 1150 30  0001 C CNN
F 1 "GND" H 3550 1080 30  0001 C CNN
F 2 "" H 3550 1150 60  0000 C CNN
F 3 "" H 3550 1150 60  0000 C CNN
	1    3550 1150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 524B2F48
P 4250 1150
F 0 "#PWR?" H 4250 1150 30  0001 C CNN
F 1 "GND" H 4250 1080 30  0001 C CNN
F 2 "" H 4250 1150 60  0000 C CNN
F 3 "" H 4250 1150 60  0000 C CNN
	1    4250 1150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 524B2F57
P 2450 2150
F 0 "#PWR?" H 2450 2150 30  0001 C CNN
F 1 "GND" H 2450 2080 30  0001 C CNN
F 2 "" H 2450 2150 60  0000 C CNN
F 3 "" H 2450 2150 60  0000 C CNN
	1    2450 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 2000 2450 2150
Wire Wire Line
	4600 2450 5500 2450
Wire Wire Line
	5100 2450 5100 2600
Wire Wire Line
	5800 2650 5800 3200
Wire Wire Line
	6000 1300 7150 1300
Text Label 6550 1300 0    60   ~ 0
VBAT_SW
Text Label 4500 1300 0    60   ~ 0
VBAT
Wire Wire Line
	7150 1500 6800 1500
Wire Wire Line
	6800 1500 6800 1300
Connection ~ 6800 1300
$Comp
L GND #PWR?
U 1 1 524B2FB0
P 7000 1700
F 0 "#PWR?" H 7000 1700 30  0001 C CNN
F 1 "GND" H 7000 1630 30  0001 C CNN
F 2 "" H 7000 1700 60  0000 C CNN
F 3 "" H 7000 1700 60  0000 C CNN
	1    7000 1700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 524B2FBF
P 8000 1700
F 0 "#PWR?" H 8000 1700 30  0001 C CNN
F 1 "GND" H 8000 1630 30  0001 C CNN
F 2 "" H 8000 1700 60  0000 C CNN
F 3 "" H 8000 1700 60  0000 C CNN
	1    8000 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 1400 7150 1400
Wire Wire Line
	7850 1500 8000 1500
Wire Wire Line
	8000 1500 8000 1700
Wire Wire Line
	7000 1400 7000 1700
Wire Wire Line
	7850 1300 9200 1300
$Comp
L CONN_1 P?
U 1 1 524B3040
P 9350 1600
F 0 "P?" H 9430 1600 40  0000 L CNN
F 1 "TEST_POINT" H 9350 1655 30  0001 C CNN
F 2 "" H 9350 1600 60  0000 C CNN
F 3 "" H 9350 1600 60  0000 C CNN
	1    9350 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9200 1600 8950 1600
Connection ~ 8950 1300
Wire Wire Line
	5100 3100 5100 3200
Connection ~ 8950 1600
$Comp
L R R?
U 1 1 524B3188
P 1300 1850
F 0 "R?" V 1380 1850 40  0000 C CNN
F 1 "9.09k_0.1%" V 1200 1850 40  0000 C CNN
F 2 "~" V 1230 1850 30  0000 C CNN
F 3 "~" H 1300 1850 30  0000 C CNN
	1    1300 1850
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 524B31E4
P 1300 2850
F 0 "R?" V 1380 2850 40  0000 C CNN
F 1 "9.09k_0.1%" V 1200 2850 40  0000 C CNN
F 2 "~" V 1230 2850 30  0000 C CNN
F 3 "~" H 1300 2850 30  0000 C CNN
	1    1300 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 2100 1300 2600
$Comp
L BAS40-05-V D?
U 1 1 524B3606
P 4150 2450
F 0 "D?" H 4100 2700 60  0000 C CNN
F 1 "BAS40-05-V" H 4150 2250 60  0000 C CNN
F 2 "" H 4150 2450 60  0000 C CNN
F 3 "http://www.vishay.com/docs/85701/bas40v.pdf" H 4100 2150 60  0001 C CNN
	1    4150 2450
	1    0    0    -1  
$EndComp
Connection ~ 5100 2450
Wire Wire Line
	3750 2350 1300 2350
Connection ~ 1300 2350
$Comp
L GND #PWR?
U 1 1 524B3660
P 1300 3300
F 0 "#PWR?" H 1300 3300 30  0001 C CNN
F 1 "GND" H 1300 3230 30  0001 C CNN
F 2 "" H 1300 3300 60  0000 C CNN
F 3 "" H 1300 3300 60  0000 C CNN
	1    1300 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 3100 1300 3300
Wire Wire Line
	1050 1300 2000 1300
Wire Wire Line
	1300 1300 1300 1600
Connection ~ 1300 1300
$Comp
L C C?
U 1 1 524B3799
P 6500 1600
F 0 "C?" H 6500 1700 40  0000 L CNN
F 1 "1uF 10V" H 6506 1515 40  0000 L CNN
F 2 "~" H 6538 1450 30  0000 C CNN
F 3 "~" H 6500 1600 60  0000 C CNN
	1    6500 1600
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 524B37A8
P 8400 1600
F 0 "C?" H 8400 1700 40  0000 L CNN
F 1 "1uF 10V" H 8406 1515 40  0000 L CNN
F 2 "~" H 8438 1450 30  0000 C CNN
F 3 "~" H 8400 1600 60  0000 C CNN
	1    8400 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 524B37B7
P 8400 2000
F 0 "#PWR?" H 8400 2000 30  0001 C CNN
F 1 "GND" H 8400 1930 30  0001 C CNN
F 2 "" H 8400 2000 60  0000 C CNN
F 3 "" H 8400 2000 60  0000 C CNN
	1    8400 2000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 524B37C6
P 6500 2000
F 0 "#PWR?" H 6500 2000 30  0001 C CNN
F 1 "GND" H 6500 1930 30  0001 C CNN
F 2 "" H 6500 2000 60  0000 C CNN
F 3 "" H 6500 2000 60  0000 C CNN
	1    6500 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 1400 8400 1300
Connection ~ 8400 1300
Wire Wire Line
	6500 1400 6500 1300
Connection ~ 6500 1300
Wire Wire Line
	6500 1800 6500 2000
Wire Wire Line
	8400 1800 8400 2000
$Comp
L TPS73633DCQ U?
U 1 1 524B3D49
P 2700 6100
F 0 "U?" H 2450 6350 60  0000 C CNN
F 1 "TPS73633DCQ" H 2700 5850 60  0000 C CNN
F 2 "" H 2700 6100 60  0000 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tps73633.pdf" H 2650 5750 60  0001 C CNN
	1    2700 6100
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 524B3D6B
P 3600 6500
F 0 "C?" H 3600 6600 40  0000 L CNN
F 1 "0.01uF 25V" H 3606 6415 40  0000 L CNN
F 2 "~" H 3638 6350 30  0000 C CNN
F 3 "~" H 3600 6500 60  0000 C CNN
	1    3600 6500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 524B3D91
P 1300 6200
F 0 "C?" H 1300 6300 40  0000 L CNN
F 1 "1uF 10V" H 1306 6115 40  0000 L CNN
F 2 "~" H 1338 6050 30  0000 C CNN
F 3 "~" H 1300 6200 60  0000 C CNN
	1    1300 6200
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 524B3DAA
P 4150 6500
F 0 "C?" H 4150 6600 40  0000 L CNN
F 1 "1uF 10V" H 4156 6415 40  0000 L CNN
F 2 "~" H 4188 6350 30  0000 C CNN
F 3 "~" H 4150 6500 60  0000 C CNN
	1    4150 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 6100 3600 6100
Wire Wire Line
	3600 6100 3600 6300
Wire Wire Line
	3200 5950 4150 5950
Wire Wire Line
	4150 5900 4150 6300
$Comp
L GND #PWR?
U 1 1 524B3E52
P 3600 6750
F 0 "#PWR?" H 3600 6750 30  0001 C CNN
F 1 "GND" H 3600 6680 30  0001 C CNN
F 2 "" H 3600 6750 60  0000 C CNN
F 3 "" H 3600 6750 60  0000 C CNN
	1    3600 6750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 524B3E61
P 4150 6750
F 0 "#PWR?" H 4150 6750 30  0001 C CNN
F 1 "GND" H 4150 6680 30  0001 C CNN
F 2 "" H 4150 6750 60  0000 C CNN
F 3 "" H 4150 6750 60  0000 C CNN
	1    4150 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 6700 4150 6750
Wire Wire Line
	3600 6700 3600 6750
$Comp
L GND #PWR?
U 1 1 524B3F00
P 1300 6450
F 0 "#PWR?" H 1300 6450 30  0001 C CNN
F 1 "GND" H 1300 6380 30  0001 C CNN
F 2 "" H 1300 6450 60  0000 C CNN
F 3 "" H 1300 6450 60  0000 C CNN
	1    1300 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 6400 1300 6450
Wire Wire Line
	2250 5950 1300 5950
Wire Wire Line
	1300 5950 1300 6000
Text Label 1650 5950 0    60   ~ 0
VBAT_SW
Wire Wire Line
	2250 6100 1550 6100
Text Label 1650 6100 0    60   ~ 0
PM_ENABLE
$Comp
L GND #PWR?
U 1 1 524B4067
P 2100 6400
F 0 "#PWR?" H 2100 6400 30  0001 C CNN
F 1 "GND" H 2100 6330 30  0001 C CNN
F 2 "" H 2100 6400 60  0000 C CNN
F 3 "" H 2100 6400 60  0000 C CNN
	1    2100 6400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 524B4076
P 3400 6400
F 0 "#PWR?" H 3400 6400 30  0001 C CNN
F 1 "GND" H 3400 6330 30  0001 C CNN
F 2 "" H 3400 6400 60  0000 C CNN
F 3 "" H 3400 6400 60  0000 C CNN
	1    3400 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 6400 3400 6250
Wire Wire Line
	3400 6250 3200 6250
Wire Wire Line
	2250 6250 2100 6250
Wire Wire Line
	2100 6250 2100 6400
Wire Wire Line
	4700 5900 4150 5900
Connection ~ 4150 5950
$Comp
L VDD_MCU U?
U 1 1 524B4490
P 9200 1100
F 0 "U?" H 9200 1300 60  0001 C CNN
F 1 "VDD_MCU" H 9200 1200 60  0000 C CNN
F 2 "" H 9200 1100 60  0000 C CNN
F 3 "" H 9200 1100 60  0000 C CNN
	1    9200 1100
	1    0    0    -1  
$EndComp
$Comp
L VDD_PERIPH U?
U 1 1 524B44AE
P 4700 5700
F 0 "U?" H 4700 5900 60  0001 C CNN
F 1 "VDD_PERIPH" H 4700 5800 60  0000 C CNN
F 2 "" H 4700 5700 60  0000 C CNN
F 3 "" H 4700 5700 60  0000 C CNN
	1    4700 5700
	1    0    0    -1  
$EndComp
$Comp
L VBUS U?
U 1 1 524B476C
P 1050 1100
F 0 "U?" H 1050 1300 60  0001 C CNN
F 1 "VBUS" H 1050 1200 60  0000 C CNN
F 2 "~" H 1050 1100 60  0000 C CNN
F 3 "~" H 1050 1100 60  0000 C CNN
	1    1050 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 1600 8950 1300
$EndSCHEMATC
