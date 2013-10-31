EESchema Schematic File Version 2  date 10/31/2013 11:19:55 PM
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
Sheet 2 6
Title "WRFL power management"
Date "31 oct 2013"
Rev "1.0"
Comp "Nightmechanic"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L BQ24092DGQ U3
U 1 1 524B2D7D
P 3050 1500
F 0 "U3" H 2800 1950 60  0000 C CNN
F 1 "BQ24092DGQ" H 3050 1850 60  0000 C CNN
F 2 "" H 3050 1500 60  0000 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/bq24092.pdf" H 3050 2100 60  0001 C CNN
	1    3050 1500
	1    0    0    -1  
$EndComp
$Comp
L SI2333DDS Q2
U 1 1 524B2D9B
P 6750 1400
F 0 "Q2" V 7050 1200 60  0000 C CNN
F 1 "SI2333DDS" V 6950 1400 60  0000 C CNN
F 2 "SOT-23" H 6750 1400 60  0001 C CNN
F 3 "http://www.vishay.com/docs/63861/si2333dds.pdf" H 6700 1100 60  0001 C BNN
	1    6750 1400
	0    1    -1   0   
$EndComp
$Comp
L SI1012CR Q1
U 1 1 524B2DAA
P 6650 2900
F 0 "Q1" H 6850 2900 60  0000 C CNN
F 1 "SI1012CR" H 7000 2800 60  0000 C CNN
F 2 "SC-75A" H 6650 2900 60  0001 C CNN
F 3 "http://www.vishay.com/docs/67519/si1012cr.pdf" H 6600 2650 60  0001 C CNN
	1    6650 2900
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 524B2DCE
P 6200 1650
F 0 "R7" V 6280 1650 40  0000 C CNN
F 1 "1M" V 6207 1651 40  0000 C CNN
F 2 "0805" V 6130 1650 30  0001 C CNN
F 3 "~" H 6200 1650 30  0000 C CNN
	1    6200 1650
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 524B2DDD
P 6050 3300
F 0 "R6" V 6130 3300 40  0000 C CNN
F 1 "1M" V 6057 3301 40  0000 C CNN
F 2 "0805" V 5980 3300 30  0001 C CNN
F 3 "~" H 6050 3300 30  0000 C CNN
	1    6050 3300
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P4
U 1 1 524B2E40
P 5100 800
F 0 "P4" V 5050 800 40  0000 C CNN
F 1 "JST_XH_2" V 5150 800 40  0000 C CNN
F 2 "" H 5100 800 60  0000 C CNN
F 3 "" H 5100 800 60  0000 C CNN
	1    5100 800 
	0    -1   -1   0   
$EndComp
$Comp
L CONN_2 P1
U 1 1 524B2E50
P 4400 800
F 0 "P1" V 4350 800 40  0000 C CNN
F 1 "JST_XH_2" V 4450 800 40  0000 C CNN
F 2 "" H 4400 800 60  0000 C CNN
F 3 "" H 4400 800 60  0000 C CNN
	1    4400 800 
	0    -1   -1   0   
$EndComp
$Comp
L TPS78233DDC U2
U 1 1 524B2E7C
P 8650 6150
F 0 "U2" H 8500 6350 60  0000 C CNN
F 1 "TPS78233DDC" H 8650 5950 60  0000 C CNN
F 2 "" H 8650 6150 60  0000 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tps78233.pdf" H 8700 5850 60  0001 C CNN
	1    8650 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 1300 4300 1300
Wire Wire Line
	4300 1300 5700 1300
Wire Wire Line
	5700 1300 5900 1300
Wire Wire Line
	5900 1300 6200 1300
Wire Wire Line
	6200 1300 6550 1300
Wire Wire Line
	6200 1400 6200 1300
Connection ~ 6200 1300
Wire Wire Line
	4300 1150 4300 1300
Connection ~ 4300 1300
Wire Wire Line
	5000 1450 5000 1150
Wire Wire Line
	6750 1600 6750 2050
Wire Wire Line
	6750 2050 6750 2700
Wire Wire Line
	6200 1900 6200 2050
$Comp
L GND #PWR21
U 1 1 524B2F1B
P 6750 3650
F 0 "#PWR21" H 6750 3650 30  0001 C CNN
F 1 "GND" H 6750 3580 30  0001 C CNN
F 2 "" H 6750 3650 60  0000 C CNN
F 3 "" H 6750 3650 60  0000 C CNN
	1    6750 3650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR20
U 1 1 524B2F2A
P 6050 3650
F 0 "#PWR20" H 6050 3650 30  0001 C CNN
F 1 "GND" H 6050 3580 30  0001 C CNN
F 2 "" H 6050 3650 60  0000 C CNN
F 3 "" H 6050 3650 60  0000 C CNN
	1    6050 3650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR15
U 1 1 524B2F39
P 4500 1150
F 0 "#PWR15" H 4500 1150 30  0001 C CNN
F 1 "GND" H 4500 1080 30  0001 C CNN
F 2 "" H 4500 1150 60  0000 C CNN
F 3 "" H 4500 1150 60  0000 C CNN
	1    4500 1150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR18
U 1 1 524B2F48
P 5200 1150
F 0 "#PWR18" H 5200 1150 30  0001 C CNN
F 1 "GND" H 5200 1080 30  0001 C CNN
F 2 "" H 5200 1150 60  0000 C CNN
F 3 "" H 5200 1150 60  0000 C CNN
	1    5200 1150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR10
U 1 1 524B2F57
P 3050 2250
F 0 "#PWR10" H 3050 2250 30  0001 C CNN
F 1 "GND" H 3050 2180 30  0001 C CNN
F 2 "" H 3050 2250 60  0000 C CNN
F 3 "" H 3050 2250 60  0000 C CNN
	1    3050 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 2900 6050 2900
Wire Wire Line
	6050 2900 6450 2900
Wire Wire Line
	6050 2900 6050 3050
Wire Wire Line
	6750 3100 6750 3650
Wire Wire Line
	7150 6050 7950 6050
Wire Wire Line
	7950 6050 8300 6050
Text Label 5450 1300 0    60   ~ 0
VBAT
Wire Wire Line
	7950 6050 7950 6300
Connection ~ 7950 6050
$Comp
L GND #PWR23
U 1 1 524B2FB0
P 8150 6200
F 0 "#PWR23" H 8150 6200 30  0001 C CNN
F 1 "GND" H 8150 6130 30  0001 C CNN
F 2 "" H 8150 6200 60  0000 C CNN
F 3 "" H 8150 6200 60  0000 C CNN
	1    8150 6200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR25
U 1 1 524B2FBF
P 9150 6450
F 0 "#PWR25" H 9150 6450 30  0001 C CNN
F 1 "GND" H 9150 6380 30  0001 C CNN
F 2 "" H 9150 6450 60  0000 C CNN
F 3 "" H 9150 6450 60  0000 C CNN
	1    9150 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 6150 8300 6150
Wire Wire Line
	9000 6250 9150 6250
Wire Wire Line
	9150 6250 9150 6450
Wire Wire Line
	8150 6150 8150 6200
Wire Wire Line
	9000 6050 9550 6050
Wire Wire Line
	9550 6050 10350 6050
Wire Wire Line
	10350 6050 10550 6050
$Comp
L CONN_1 P2
U 1 1 524B3040
P 10700 6050
F 0 "P2" H 10780 6050 40  0000 L CNN
F 1 "TEST_POINT" H 10700 6105 30  0001 C CNN
F 2 "" H 10700 6050 60  0000 C CNN
F 3 "" H 10700 6050 60  0000 C CNN
	1    10700 6050
	1    0    0    -1  
$EndComp
Connection ~ 10350 6050
Wire Wire Line
	6050 3550 6050 3650
$Comp
L R R1
U 1 1 524B3188
P 1300 2050
F 0 "R1" V 1380 2050 40  0000 C CNN
F 1 "9.09k_0.1%" V 1200 2050 40  0000 C CNN
F 2 "0805" V 1230 2050 30  0001 C CNN
F 3 "~" H 1300 2050 30  0000 C CNN
	1    1300 2050
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 524B31E4
P 1300 3150
F 0 "R2" V 1380 3150 40  0000 C CNN
F 1 "9.09k_0.1%" V 1200 3150 40  0000 C CNN
F 2 "0805" V 1230 3150 30  0001 C CNN
F 3 "~" H 1300 3150 30  0000 C CNN
	1    1300 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 2300 1300 2800
Wire Wire Line
	1300 2800 1300 2900
Connection ~ 6050 2900
Wire Wire Line
	1300 2800 2350 2800
Wire Wire Line
	2350 2800 4700 2800
$Comp
L GND #PWR3
U 1 1 524B3660
P 1300 3600
F 0 "#PWR3" H 1300 3600 30  0001 C CNN
F 1 "GND" H 1300 3530 30  0001 C CNN
F 2 "" H 1300 3600 60  0000 C CNN
F 3 "" H 1300 3600 60  0000 C CNN
	1    1300 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 3400 1300 3600
Wire Wire Line
	950  1300 1300 1300
Wire Wire Line
	1300 1300 1550 1300
Wire Wire Line
	1550 1300 2600 1300
Wire Wire Line
	1300 1300 1300 1800
Connection ~ 1300 1300
$Comp
L C C1
U 1 1 524B3799
P 7150 6350
F 0 "C1" H 7150 6450 40  0000 L CNN
F 1 "1uF 10V" H 7156 6265 40  0000 L CNN
F 2 "0805" H 7188 6200 30  0001 C CNN
F 3 "~" H 7150 6350 60  0000 C CNN
	1    7150 6350
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 524B37A8
P 9550 6350
F 0 "C4" H 9550 6450 40  0000 L CNN
F 1 "1uF 10V" H 9556 6265 40  0000 L CNN
F 2 "0805" H 9588 6200 30  0001 C CNN
F 3 "~" H 9550 6350 60  0000 C CNN
	1    9550 6350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR26
U 1 1 524B37B7
P 9550 6750
F 0 "#PWR26" H 9550 6750 30  0001 C CNN
F 1 "GND" H 9550 6680 30  0001 C CNN
F 2 "" H 9550 6750 60  0000 C CNN
F 3 "" H 9550 6750 60  0000 C CNN
	1    9550 6750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR22
U 1 1 524B37C6
P 7150 6750
F 0 "#PWR22" H 7150 6750 30  0001 C CNN
F 1 "GND" H 7150 6680 30  0001 C CNN
F 2 "" H 7150 6750 60  0000 C CNN
F 3 "" H 7150 6750 60  0000 C CNN
	1    7150 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	9550 6150 9550 6050
Connection ~ 9550 6050
Wire Wire Line
	7150 6150 7150 6050
Connection ~ 7150 6050
Wire Wire Line
	7150 6550 7150 6750
Wire Wire Line
	9550 6550 9550 6750
$Comp
L TPS73633DCQ U1
U 1 1 524B3D49
P 2300 4550
F 0 "U1" H 2050 4800 60  0000 C CNN
F 1 "TPS73633DCQ" H 2300 4300 60  0000 C CNN
F 2 "" H 2300 4550 60  0000 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tps73633.pdf" H 2250 4200 60  0001 C CNN
	1    2300 4550
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 524B3D6B
P 3200 4950
F 0 "C3" H 3200 5050 40  0000 L CNN
F 1 "0.01uF 25V" H 3206 4865 40  0000 L CNN
F 2 "0805" H 3238 4800 30  0001 C CNN
F 3 "~" H 3200 4950 60  0000 C CNN
	1    3200 4950
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 524B3D91
P 900 4650
F 0 "C2" H 900 4750 40  0000 L CNN
F 1 "1uF 10V" H 906 4565 40  0000 L CNN
F 2 "0805" H 938 4500 30  0001 C CNN
F 3 "~" H 900 4650 60  0000 C CNN
	1    900  4650
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 524B3DAA
P 3750 4950
F 0 "C5" H 3750 5050 40  0000 L CNN
F 1 "1uF 10V" H 3756 4865 40  0000 L CNN
F 2 "0805" H 3788 4800 30  0001 C CNN
F 3 "~" H 3750 4950 60  0000 C CNN
	1    3750 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 4550 3200 4550
Wire Wire Line
	3200 4550 3200 4750
Wire Wire Line
	2800 4400 3750 4400
Wire Wire Line
	3750 4400 4300 4400
Wire Wire Line
	4300 4400 5000 4400
Wire Wire Line
	3750 4400 3750 4750
$Comp
L GND #PWR12
U 1 1 524B3E52
P 3200 5200
F 0 "#PWR12" H 3200 5200 30  0001 C CNN
F 1 "GND" H 3200 5130 30  0001 C CNN
F 2 "" H 3200 5200 60  0000 C CNN
F 3 "" H 3200 5200 60  0000 C CNN
	1    3200 5200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR13
U 1 1 524B3E61
P 3750 5200
F 0 "#PWR13" H 3750 5200 30  0001 C CNN
F 1 "GND" H 3750 5130 30  0001 C CNN
F 2 "" H 3750 5200 60  0000 C CNN
F 3 "" H 3750 5200 60  0000 C CNN
	1    3750 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 5150 3750 5200
Wire Wire Line
	3200 5150 3200 5200
$Comp
L GND #PWR1
U 1 1 524B3F00
P 900 4900
F 0 "#PWR1" H 900 4900 30  0001 C CNN
F 1 "GND" H 900 4830 30  0001 C CNN
F 2 "" H 900 4900 60  0000 C CNN
F 3 "" H 900 4900 60  0000 C CNN
	1    900  4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  4850 900  4900
Wire Wire Line
	1850 4400 900  4400
Wire Wire Line
	900  4400 900  4450
Wire Wire Line
	1850 4550 1150 4550
$Comp
L GND #PWR5
U 1 1 524B4067
P 1700 4850
F 0 "#PWR5" H 1700 4850 30  0001 C CNN
F 1 "GND" H 1700 4780 30  0001 C CNN
F 2 "" H 1700 4850 60  0000 C CNN
F 3 "" H 1700 4850 60  0000 C CNN
	1    1700 4850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR9
U 1 1 524B4076
P 3000 4850
F 0 "#PWR9" H 3000 4850 30  0001 C CNN
F 1 "GND" H 3000 4780 30  0001 C CNN
F 2 "" H 3000 4850 60  0000 C CNN
F 3 "" H 3000 4850 60  0000 C CNN
	1    3000 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 4850 3000 4700
Wire Wire Line
	3000 4700 2800 4700
Wire Wire Line
	1850 4700 1700 4700
Wire Wire Line
	1700 4700 1700 4850
Connection ~ 3750 4400
$Comp
L VDD_MCU #U10
U 1 1 524B4490
P 10350 5850
F 0 "#U10" H 10350 6050 60  0001 C CNN
F 1 "VDD_MCU" H 10350 5950 60  0000 C CNN
F 2 "" H 10350 5850 60  0000 C CNN
F 3 "" H 10350 5850 60  0000 C CNN
	1    10350 5850
	1    0    0    -1  
$EndComp
$Comp
L VDD_PERIPH #U4
U 1 1 524B44AE
P 4300 4050
F 0 "#U4" H 4300 4250 60  0001 C CNN
F 1 "VDD_PERIPH" H 4300 4150 60  0000 C CNN
F 2 "" H 4300 4050 60  0000 C CNN
F 3 "" H 4300 4050 60  0000 C CNN
	1    4300 4050
	1    0    0    -1  
$EndComp
$Comp
L VBUS #U3
U 1 1 524B476C
P 950 1100
F 0 "#U3" H 950 1300 60  0001 C CNN
F 1 "VBUS" H 950 1200 60  0000 C CNN
F 2 "~" H 950 1100 60  0000 C CNN
F 3 "~" H 950 1100 60  0000 C CNN
	1    950  1100
	1    0    0    -1  
$EndComp
$Comp
L VBAT_SW #U7
U 1 1 5251AD38
P 7150 5850
F 0 "#U7" H 7150 6050 60  0001 C CNN
F 1 "VBAT_SW" H 7150 5950 60  0000 C CNN
F 2 "~" H 7150 5850 60  0000 C CNN
F 3 "~" H 7150 5850 60  0000 C CNN
	1    7150 5850
	1    0    0    -1  
$EndComp
$Comp
L VBAT_SW #U1
U 1 1 5251AD47
P 900 4200
F 0 "#U1" H 900 4400 60  0001 C CNN
F 1 "VBAT_SW" H 900 4300 60  0000 C CNN
F 2 "~" H 900 4200 60  0000 C CNN
F 3 "~" H 900 4200 60  0000 C CNN
	1    900  4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 4250 4300 4400
Wire Wire Line
	4300 4400 4300 4650
Text GLabel 4250 3000 0    60   Input ~ 0
VBAT_EN
Wire Wire Line
	4700 3000 4250 3000
$Comp
L CONN_1 P3
U 1 1 5251ADE1
P 4450 4650
F 0 "P3" H 4530 4650 40  0000 L CNN
F 1 "TEST_POINT" H 4450 4705 30  0001 C CNN
F 2 "" H 4450 4650 60  0000 C CNN
F 3 "" H 4450 4650 60  0000 C CNN
	1    4450 4650
	1    0    0    -1  
$EndComp
Connection ~ 4300 4400
$Comp
L VBAT_SW #U8
U 1 1 5251AE31
P 7700 850
F 0 "#U8" H 7700 1050 60  0001 C CNN
F 1 "VBAT_SW" H 7700 950 60  0000 C CNN
F 2 "~" H 7700 850 60  0000 C CNN
F 3 "~" H 7700 850 60  0000 C CNN
	1    7700 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 1300 7300 1300
Wire Wire Line
	7300 1300 7700 1300
Wire Wire Line
	7700 1300 8050 1300
Text GLabel 1200 6850 0    60   Input ~ 0
PM_ENABLE
$Comp
L LM2735Y U4
U 1 1 5251AF5C
P 2350 6650
F 0 "U4" H 2150 6950 60  0000 C CNN
F 1 "LM2735Y" H 2300 6350 60  0000 C CNN
F 2 "~" H 2150 6650 60  0000 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2735.pdf" H 2150 6650 60  0001 C CNN
	1    2350 6650
	-1   0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 5251AF7F
P 4600 6500
F 0 "C9" H 4600 6600 40  0000 L CNN
F 1 "10uF 16V" H 4606 6415 40  0000 L CNN
F 2 "0805" H 4638 6350 30  0001 C CNN
F 3 "~" H 4600 6500 60  0000 C CNN
	1    4600 6500
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 5251AF8E
P 4100 6500
F 0 "C8" H 4100 6600 40  0000 L CNN
F 1 "680pF" H 4106 6415 40  0000 L CNN
F 2 "0805" H 4138 6350 30  0001 C CNN
F 3 "~" H 4100 6500 60  0000 C CNN
	1    4100 6500
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 5251AF9D
P 3800 7250
F 0 "R9" V 3880 7250 40  0000 C CNN
F 1 "10k" V 3807 7251 40  0000 C CNN
F 2 "0805" V 3730 7250 30  0001 C CNN
F 3 "~" H 3800 7250 30  0000 C CNN
	1    3800 7250
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 5251AFAC
P 3800 6550
F 0 "R8" V 3880 6550 40  0000 C CNN
F 1 "31.6k" V 3807 6551 40  0000 C CNN
F 2 "0805" V 3730 6550 30  0001 C CNN
F 3 "~" H 3800 6550 30  0000 C CNN
	1    3800 6550
	1    0    0    -1  
$EndComp
$Comp
L INDUCTOR L1
U 1 1 5251AFBB
P 2400 6000
F 0 "L1" V 2350 6000 40  0000 C CNN
F 1 "10uH 60mOhm 3A" V 2500 6000 40  0000 C CNN
F 2 "~" H 2400 6000 60  0000 C CNN
F 3 "~" H 2400 6000 60  0000 C CNN
	1    2400 6000
	0    -1   -1   0   
$EndComp
$Comp
L DIODESCH D2
U 1 1 5251AFE3
P 3650 6000
F 0 "D2" H 3650 6100 40  0000 C CNN
F 1 "CMS06" H 3650 5900 40  0000 C CNN
F 2 "~" H 3650 6000 60  0000 C CNN
F 3 "~" H 3650 6000 60  0000 C CNN
	1    3650 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  6000 1200 6000
Wire Wire Line
	1200 6000 1700 6000
Wire Wire Line
	1700 6000 2100 6000
Wire Wire Line
	1200 6000 1200 6100
Connection ~ 1700 6000
Wire Wire Line
	2700 6000 3100 6000
Wire Wire Line
	3100 6000 3450 6000
Connection ~ 3100 6000
Wire Wire Line
	3850 6000 4100 6000
Wire Wire Line
	4100 6000 4400 6000
Wire Wire Line
	4400 6000 4600 6000
Wire Wire Line
	4600 6000 5100 6000
Wire Wire Line
	5100 6000 5250 6000
Wire Wire Line
	4600 6000 4600 6300
Wire Wire Line
	4100 6000 4100 6200
Wire Wire Line
	4100 6200 4100 6300
Connection ~ 4100 6000
Wire Wire Line
	4100 6850 4100 6700
Wire Wire Line
	2900 6850 3800 6850
Wire Wire Line
	3800 6850 4100 6850
Wire Wire Line
	2900 6450 3100 6450
Wire Wire Line
	3100 6450 3100 6000
Wire Wire Line
	1850 6450 1700 6450
Wire Wire Line
	1700 6450 1700 6000
Wire Wire Line
	3800 6800 3800 6850
Wire Wire Line
	3800 6850 3800 7000
Connection ~ 3800 6850
Wire Wire Line
	3800 7500 3800 7600
Wire Wire Line
	4600 6700 4600 6850
Wire Wire Line
	2900 6650 3100 6650
Wire Wire Line
	3100 6650 3100 6700
$Comp
L GND #PWR11
U 1 1 5251B5EF
P 3100 6700
F 0 "#PWR11" H 3100 6700 30  0001 C CNN
F 1 "GND" H 3100 6630 30  0001 C CNN
F 2 "" H 3100 6700 60  0000 C CNN
F 3 "" H 3100 6700 60  0000 C CNN
	1    3100 6700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR2
U 1 1 5251B5FE
P 1050 6650
F 0 "#PWR2" H 1050 6650 30  0001 C CNN
F 1 "GND" H 1050 6580 30  0001 C CNN
F 2 "" H 1050 6650 60  0000 C CNN
F 3 "" H 1050 6650 60  0000 C CNN
	1    1050 6650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR16
U 1 1 5251B60D
P 4600 6850
F 0 "#PWR16" H 4600 6850 30  0001 C CNN
F 1 "GND" H 4600 6780 30  0001 C CNN
F 2 "" H 4600 6850 60  0000 C CNN
F 3 "" H 4600 6850 60  0000 C CNN
	1    4600 6850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR14
U 1 1 5251B61C
P 3800 7600
F 0 "#PWR14" H 3800 7600 30  0001 C CNN
F 1 "GND" H 3800 7530 30  0001 C CNN
F 2 "" H 3800 7600 60  0000 C CNN
F 3 "" H 3800 7600 60  0000 C CNN
	1    3800 7600
	1    0    0    -1  
$EndComp
$Comp
L VBAT_SW #U2
U 1 1 5251B635
P 900 5700
F 0 "#U2" H 900 5900 60  0001 C CNN
F 1 "VBAT_SW" H 900 5800 60  0000 C CNN
F 2 "~" H 900 5700 60  0000 C CNN
F 3 "~" H 900 5700 60  0000 C CNN
	1    900  5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	900  5900 900  6000
Wire Wire Line
	900  6000 900  6100
Connection ~ 1200 6000
Wire Wire Line
	1850 6850 1200 6850
$Comp
L V_BOOST #U5
U 1 1 5251BF9B
P 5100 5700
F 0 "#U5" H 5100 5900 60  0001 C CNN
F 1 "V_BOOST" H 5100 5800 60  0000 C CNN
F 2 "~" H 5100 5700 60  0000 C CNN
F 3 "~" H 5100 5700 60  0000 C CNN
	1    5100 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 6000 5100 5900
Connection ~ 4600 6000
$Comp
L CONN_1 P7
U 1 1 5251C004
P 5400 6000
F 0 "P7" H 5480 6000 40  0000 L CNN
F 1 "TEST_POINT" H 5400 6055 30  0001 C CNN
F 2 "" H 5400 6000 60  0000 C CNN
F 3 "" H 5400 6000 60  0000 C CNN
	1    5400 6000
	1    0    0    -1  
$EndComp
Connection ~ 5100 6000
Wire Wire Line
	3050 2150 3050 2250
Connection ~ 1300 2800
Wire Wire Line
	3500 1450 3700 1450
Wire Wire Line
	3700 1450 5000 1450
$Comp
L R R4
U 1 1 5251C362
P 2200 2050
F 0 "R4" V 2280 2050 40  0000 C CNN
F 1 "2k" V 2207 2051 40  0000 C CNN
F 2 "0805" V 2130 2050 30  0001 C CNN
F 3 "~" H 2200 2050 30  0000 C CNN
	1    2200 2050
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 5251C368
P 1850 2050
F 0 "R3" V 1930 2050 40  0000 C CNN
F 1 "825" V 1857 2051 40  0000 C CNN
F 2 "0805" V 1780 2050 30  0001 C CNN
F 3 "~" H 1850 2050 30  0000 C CNN
	1    1850 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 1450 1850 1450
Wire Wire Line
	1850 1450 1850 1800
Wire Wire Line
	2600 1750 2200 1750
Wire Wire Line
	2200 1750 2200 1800
$Comp
L GND #PWR7
U 1 1 5251C49F
P 2050 1700
F 0 "#PWR7" H 2050 1700 30  0001 C CNN
F 1 "GND" H 2050 1630 30  0001 C CNN
F 2 "" H 2050 1700 60  0000 C CNN
F 3 "" H 2050 1700 60  0000 C CNN
	1    2050 1700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR6
U 1 1 5251C4AE
P 1850 2400
F 0 "#PWR6" H 1850 2400 30  0001 C CNN
F 1 "GND" H 1850 2330 30  0001 C CNN
F 2 "" H 1850 2400 60  0000 C CNN
F 3 "" H 1850 2400 60  0000 C CNN
	1    1850 2400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR8
U 1 1 5251C4BD
P 2200 2400
F 0 "#PWR8" H 2200 2400 30  0001 C CNN
F 1 "GND" H 2200 2330 30  0001 C CNN
F 2 "" H 2200 2400 60  0000 C CNN
F 3 "" H 2200 2400 60  0000 C CNN
	1    2200 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 1600 2050 1600
Wire Wire Line
	2050 1600 2050 1700
Wire Wire Line
	2200 2300 2200 2400
Wire Wire Line
	1850 2300 1850 2400
NoConn ~ 2600 1900
NoConn ~ 3500 1900
Text GLabel 3900 1750 2    60   Input ~ 0
CHARGR_ISET
Wire Wire Line
	3500 1750 3900 1750
Text Label 1300 6850 0    60   ~ 0
PM_EN
Text Label 1350 4550 0    60   ~ 0
PM_EN
$Comp
L R R5
U 1 1 5251C65E
P 5250 2000
F 0 "R5" V 5330 2000 40  0000 C CNN
F 1 "100k" V 5257 2001 40  0000 C CNN
F 2 "0805" V 5180 2000 30  0001 C CNN
F 3 "~" H 5250 2000 30  0000 C CNN
	1    5250 2000
	1    0    0    -1  
$EndComp
$Comp
L VDD_MCU #U6
U 1 1 5251C670
P 5250 1550
F 0 "#U6" H 5250 1750 60  0001 C CNN
F 1 "VDD_MCU" H 5250 1650 60  0000 C CNN
F 2 "~" H 5250 1550 60  0000 C CNN
F 3 "~" H 5250 1550 60  0000 C CNN
	1    5250 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 2250 5250 2250
Wire Wire Line
	5250 2250 5550 2250
Wire Wire Line
	4850 2250 4850 1600
Wire Wire Line
	4850 1600 3500 1600
Text GLabel 5550 2250 2    60   Input ~ 0
CHARGR_IND
Connection ~ 5250 2250
Text GLabel 2750 3000 2    60   Input ~ 0
VBUS_SENS
Wire Wire Line
	2750 3000 2350 3000
Wire Wire Line
	2350 3000 2350 2800
Connection ~ 2350 2800
$Comp
L C C7
U 1 1 5251CC1A
P 1200 6300
F 0 "C7" H 1200 6400 40  0000 L CNN
F 1 "10uF 16V" H 1206 6215 40  0000 L CNN
F 2 "0805" H 1238 6150 30  0001 C CNN
F 3 "~" H 1200 6300 60  0000 C CNN
	1    1200 6300
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 5251CC20
P 900 6300
F 0 "C6" H 900 6400 40  0000 L CNN
F 1 "10uF 16V" H 906 6215 40  0000 L CNN
F 2 "0805" H 938 6150 30  0001 C CNN
F 3 "~" H 900 6300 60  0000 C CNN
	1    900  6300
	1    0    0    -1  
$EndComp
Connection ~ 900  6000
$Comp
L CONN_1 P6
U 1 1 5251DD62
P 8200 1300
F 0 "P6" H 8280 1300 40  0000 L CNN
F 1 "TEST_POINT" H 8200 1355 30  0001 C CNN
F 2 "" H 8200 1300 60  0000 C CNN
F 3 "" H 8200 1300 60  0000 C CNN
	1    8200 1300
	1    0    0    -1  
$EndComp
$Comp
L CONN_1 P5
U 1 1 5251DDD7
P 6050 950
F 0 "P5" H 6130 950 40  0000 L CNN
F 1 "TEST_POINT" H 6050 1005 30  0001 C CNN
F 2 "" H 6050 950 60  0000 C CNN
F 3 "" H 6050 950 60  0000 C CNN
	1    6050 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 950  5900 1300
Connection ~ 5900 1300
$Comp
L C C15
U 1 1 5251E5D2
P 1550 2050
F 0 "C15" H 1550 2150 40  0000 L CNN
F 1 "10uF 16V" H 1556 1965 40  0000 L CNN
F 2 "0805" H 1588 1900 30  0001 C CNN
F 3 "~" H 1550 2050 60  0000 C CNN
	1    1550 2050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR4
U 1 1 5251E5DA
P 1550 2400
F 0 "#PWR4" H 1550 2400 30  0001 C CNN
F 1 "GND" H 1550 2330 30  0001 C CNN
F 2 "" H 1550 2400 60  0000 C CNN
F 3 "" H 1550 2400 60  0000 C CNN
	1    1550 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 1850 1550 1300
Connection ~ 1550 1300
Wire Wire Line
	1550 2250 1550 2400
$Comp
L C C16
U 1 1 5251E6CD
P 5700 1600
F 0 "C16" H 5700 1700 40  0000 L CNN
F 1 "10uF 16V" H 5706 1515 40  0000 L CNN
F 2 "0805" H 5738 1450 30  0001 C CNN
F 3 "~" H 5700 1600 60  0000 C CNN
	1    5700 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR19
U 1 1 5251E6D3
P 5700 2000
F 0 "#PWR19" H 5700 2000 30  0001 C CNN
F 1 "GND" H 5700 1930 30  0001 C CNN
F 2 "" H 5700 2000 60  0000 C CNN
F 3 "" H 5700 2000 60  0000 C CNN
	1    5700 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 1400 5700 1300
Connection ~ 5700 1300
Wire Wire Line
	5700 1800 5700 2000
Text GLabel 7400 2050 2    60   Input ~ 0
ON_SW_FET
$Comp
L R R15
U 1 1 52521245
P 9850 2450
F 0 "R15" V 9930 2450 40  0000 C CNN
F 1 "100k" V 9857 2451 40  0000 C CNN
F 2 "0805" V 9780 2450 30  0001 C CNN
F 3 "~" H 9850 2450 30  0000 C CNN
	1    9850 2450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR27
U 1 1 5252124B
P 9850 2900
F 0 "#PWR27" H 9850 2900 30  0001 C CNN
F 1 "GND" H 9850 2830 30  0001 C CNN
F 2 "" H 9850 2900 60  0000 C CNN
F 3 "" H 9850 2900 60  0000 C CNN
	1    9850 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 1900 9850 2050
Wire Wire Line
	9850 2050 9850 2200
Wire Wire Line
	9850 2700 9850 2900
$Comp
L R R14
U 1 1 5252135F
P 9400 2050
F 0 "R14" V 9480 2050 40  0000 C CNN
F 1 "10k" V 9407 2051 40  0000 C CNN
F 2 "0805" V 9330 2050 30  0001 C CNN
F 3 "~" H 9400 2050 30  0000 C CNN
	1    9400 2050
	0    -1   -1   0   
$EndComp
Text GLabel 8950 2050 0    60   Input ~ 0
ON_SW_MCU
Wire Wire Line
	9150 2050 8950 2050
Wire Notes Line
	7850 1900 8000 1900
Wire Notes Line
	8250 1900 8450 1900
Wire Notes Line
	8000 1900 8250 1800
$Comp
L R R19
U 1 1 5252599E
P 8950 3600
F 0 "R19" V 9030 3600 40  0000 C CNN
F 1 "9.09k_0.1%" V 8850 3600 40  0000 C CNN
F 2 "0805" V 8880 3600 30  0001 C CNN
F 3 "~" H 8950 3600 30  0000 C CNN
	1    8950 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 2050 6750 2050
Wire Wire Line
	6750 2050 7400 2050
Connection ~ 6750 2050
Text Notes 7350 1700 0    60   ~ 0
Power on switch + User switch\nUsing the rotary encoder's push button
$Comp
L VBAT_SW #U9
U 1 1 52525B52
P 8950 2950
F 0 "#U9" H 8950 3150 60  0001 C CNN
F 1 "VBAT_SW" H 8950 3050 60  0000 C CNN
F 2 "~" H 8950 2950 60  0000 C CNN
F 3 "~" H 8950 2950 60  0000 C CNN
	1    8950 2950
	1    0    0    -1  
$EndComp
$Comp
L R R20
U 1 1 52525B62
P 8950 4250
F 0 "R20" V 9030 4250 40  0000 C CNN
F 1 "9.09k_0.1%" V 8850 4250 40  0000 C CNN
F 2 "0805" V 8880 4250 30  0001 C CNN
F 3 "~" H 8950 4250 30  0000 C CNN
	1    8950 4250
	1    0    0    -1  
$EndComp
$Comp
L SI1012CR Q4
U 1 1 52525B68
P 8850 4850
F 0 "Q4" H 9050 4850 60  0000 C CNN
F 1 "SI1012CR" H 9200 4750 60  0000 C CNN
F 2 "SC-75A" H 8850 4850 60  0001 C CNN
F 3 "http://www.vishay.com/docs/67519/si1012cr.pdf" H 8800 4600 60  0001 C CNN
	1    8850 4850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR24
U 1 1 52525B6E
P 8950 5200
F 0 "#PWR24" H 8950 5200 30  0001 C CNN
F 1 "GND" H 8950 5130 30  0001 C CNN
F 2 "" H 8950 5200 60  0000 C CNN
F 3 "" H 8950 5200 60  0000 C CNN
	1    8950 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 3150 8950 3350
Wire Wire Line
	8950 3850 8950 3950
Wire Wire Line
	8950 3950 8950 4000
Wire Wire Line
	8950 4500 8950 4650
Wire Wire Line
	8950 5050 8950 5200
Text GLabel 8100 4850 0    60   Input ~ 0
MEAS_EN
Wire Wire Line
	8100 4850 8650 4850
Text GLabel 8100 3950 0    60   Input ~ 0
VBAT_SENS
Wire Wire Line
	8100 3950 8950 3950
Connection ~ 8950 3950
$Comp
L CONN_2 P17
U 1 1 52526A5A
P 5350 4500
F 0 "P17" H 5300 4700 40  0000 C CNN
F 1 "1x2_2_54_MALE" H 5400 4300 40  0000 C CNN
F 2 "" H 5350 4500 60  0000 C CNN
F 3 "" H 5350 4500 60  0000 C CNN
	1    5350 4500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR17
U 1 1 52526A67
P 5000 5200
F 0 "#PWR17" H 5000 5200 30  0001 C CNN
F 1 "GND" H 5000 5130 30  0001 C CNN
F 2 "" H 5000 5200 60  0000 C CNN
F 3 "" H 5000 5200 60  0000 C CNN
	1    5000 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 4600 5000 5200
Text Notes 4250 650  0    60   ~ 0
Battery 
Text Notes 4950 650  0    60   ~ 0
BAT NTC
Wire Wire Line
	8300 6250 8300 6300
Wire Wire Line
	8300 6300 7950 6300
Wire Wire Line
	3800 6300 3800 6200
Wire Wire Line
	3800 6200 4100 6200
Connection ~ 4100 6200
Wire Wire Line
	900  6500 900  6550
Wire Wire Line
	900  6550 1050 6550
Wire Wire Line
	1050 6550 1200 6550
Wire Wire Line
	1200 6550 1200 6500
Wire Wire Line
	1050 6650 1050 6550
Connection ~ 1050 6550
$Comp
L BAT54C-V D1
U 1 1 525D7621
P 5100 2900
F 0 "D1" H 5050 3150 60  0000 C CNN
F 1 "BAT54C-V" H 5100 2700 60  0000 C CNN
F 2 "SOT-23" H 5100 2900 60  0001 C CNN
F 3 "http://www.vishay.com/docs/85508/bat54v.pdf" H 5050 2600 60  0001 C CNN
	1    5100 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7700 1050 7700 1300
Connection ~ 7700 1300
$Comp
L PWR_FLAG #FLG2
U 1 1 52665E45
P 7300 1200
F 0 "#FLG2" H 7300 1295 30  0001 C CNN
F 1 "PWR_FLAG" H 7300 1380 30  0000 C CNN
F 2 "" H 7300 1200 60  0000 C CNN
F 3 "" H 7300 1200 60  0000 C CNN
	1    7300 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 1200 7300 1300
Connection ~ 7300 1300
$Comp
L PWR_FLAG #FLG1
U 1 1 52666171
P 4400 5850
F 0 "#FLG1" H 4400 5945 30  0001 C CNN
F 1 "PWR_FLAG" H 4400 6030 30  0000 C CNN
F 2 "" H 4400 5850 60  0000 C CNN
F 3 "" H 4400 5850 60  0000 C CNN
	1    4400 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 5850 4400 6000
Connection ~ 4400 6000
$Comp
L R R24
U 1 1 526CDD62
P 3700 2150
F 0 "R24" V 3780 2150 40  0000 C CNN
F 1 "ZEROHM" V 3707 2151 40  0000 C CNN
F 2 "0805" V 3630 2150 30  0001 C CNN
F 3 "~" H 3700 2150 30  0000 C CNN
	1    3700 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 1900 3700 1450
Connection ~ 3700 1450
Wire Wire Line
	3700 2400 4000 2400
Text GLabel 4000 2400 2    60   Input ~ 0
BAT_TEMP_SENS
$Comp
L VDD_PERIPH #U?
U 1 1 5272BAE9
P 9850 1700
F 0 "#U?" H 9850 1900 60  0001 C CNN
F 1 "VDD_PERIPH" H 9850 1800 60  0000 C CNN
F 2 "" H 9850 1700 60  0000 C CNN
F 3 "" H 9850 1700 60  0000 C CNN
	1    9850 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 2050 9850 2050
Connection ~ 9850 2050
$EndSCHEMATC
