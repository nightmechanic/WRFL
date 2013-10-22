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
Sheet 5 6
Title ""
Date "22 oct 2013"
Rev ""
Comp "Nightmechanic"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	1000 2450 1000 2500
Wire Wire Line
	1000 1850 1000 2050
$Comp
L C C10
U 1 1 524D592A
P 1000 2250
F 0 "C10" H 1000 2350 40  0000 L CNN
F 1 "1uF 10V" H 1006 2165 40  0000 L CNN
F 2 "0805" H 1038 2100 30  0001 C CNN
F 3 "~" H 1000 2250 60  0000 C CNN
	1    1000 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 2500 1450 2700
Wire Wire Line
	1600 2500 1450 2500
Connection ~ 2200 2500
Wire Wire Line
	2100 2500 2200 2500
$Comp
L R R11
U 1 1 524D5934
P 1850 2500
F 0 "R11" V 1930 2500 40  0000 C CNN
F 1 "ZEROHM" V 1857 2501 40  0000 C CNN
F 2 "0805" V 1780 2500 30  0001 C CNN
F 3 "~" H 1850 2500 30  0000 C CNN
	1    1850 2500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2200 2100 2200 2750
Wire Wire Line
	2350 2100 2200 2100
Wire Wire Line
	3400 2700 3400 2800
Wire Wire Line
	4100 2700 4100 2800
$Comp
L GND #PWR066
U 1 1 524D593E
P 1450 2700
F 0 "#PWR066" H 1450 2700 30  0001 C CNN
F 1 "GND" H 1450 2630 30  0001 C CNN
F 2 "" H 1450 2700 60  0000 C CNN
F 3 "" H 1450 2700 60  0000 C CNN
	1    1450 2700
	1    0    0    -1  
$EndComp
$Comp
L AGND #PWR067
U 1 1 524D5944
P 2200 2750
F 0 "#PWR067" H 2200 2750 40  0001 C CNN
F 1 "AGND" H 2200 2680 50  0000 C CNN
F 2 "" H 2200 2750 60  0000 C CNN
F 3 "" H 2200 2750 60  0000 C CNN
	1    2200 2750
	1    0    0    -1  
$EndComp
$Comp
L AGND #PWR068
U 1 1 524D594A
P 4100 2800
F 0 "#PWR068" H 4100 2800 40  0001 C CNN
F 1 "AGND" H 4100 2730 50  0000 C CNN
F 2 "" H 4100 2800 60  0000 C CNN
F 3 "" H 4100 2800 60  0000 C CNN
	1    4100 2800
	1    0    0    -1  
$EndComp
$Comp
L AGND #PWR069
U 1 1 524D5950
P 3400 2800
F 0 "#PWR069" H 3400 2800 40  0001 C CNN
F 1 "AGND" H 3400 2730 50  0000 C CNN
F 2 "" H 3400 2800 60  0000 C CNN
F 3 "" H 3400 2800 60  0000 C CNN
	1    3400 2800
	1    0    0    -1  
$EndComp
Connection ~ 4100 2000
Wire Wire Line
	4100 2000 4100 2300
$Comp
L C C13
U 1 1 524D5958
P 4100 2500
F 0 "C13" H 4100 2600 40  0000 L CNN
F 1 "10uF 16V" H 4106 2415 40  0000 L CNN
F 2 "0805" H 4138 2350 30  0001 C CNN
F 3 "~" H 4100 2500 60  0000 C CNN
	1    4100 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 2200 3400 2300
Wire Wire Line
	3050 2200 3400 2200
$Comp
L C C12
U 1 1 524D5960
P 3400 2500
F 0 "C12" H 3400 2600 40  0000 L CNN
F 1 "0.01uF 25V" H 3406 2415 40  0000 L CNN
F 2 "0805" H 3438 2350 30  0001 C CNN
F 3 "~" H 3400 2500 60  0000 C CNN
	1    3400 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 2000 4450 2000
Wire Wire Line
	2350 2200 1400 2200
Wire Wire Line
	1000 2000 2350 2000
$Comp
L TPS73633DBV U5
U 1 1 524D596A
P 2700 2100
F 0 "U5" H 2550 2300 60  0000 C CNN
F 1 "TPS73633DBV" H 2700 1900 60  0000 C CNN
F 2 "" H 2700 2100 60  0000 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tps73633.pdf" H 2750 1800 60  0001 C CNN
	1    2700 2100
	1    0    0    -1  
$EndComp
Text GLabel 1500 1000 0    60   Input ~ 0
PM_ENABLE
Wire Wire Line
	1500 1000 2500 1000
Text Label 1800 1000 0    60   ~ 0
PM_EN
Text Label 1600 2200 0    60   ~ 0
PM_EN
$Comp
L VBAT_SW #U070
U 1 1 5251CA9A
P 1000 1650
F 0 "#U070" H 1000 1850 60  0001 C CNN
F 1 "VBAT_SW" H 1000 1750 60  0000 C CNN
F 2 "~" H 1000 1650 60  0000 C CNN
F 3 "~" H 1000 1650 60  0000 C CNN
	1    1000 1650
	1    0    0    -1  
$EndComp
Connection ~ 1000 2000
$Comp
L CONN_8 P12
U 1 1 5251CACB
P 10450 2400
F 0 "P12" V 10400 2400 60  0000 C CNN
F 1 "JST_XH_8" V 10500 2400 60  0000 C CNN
F 2 "" H 10450 2400 60  0000 C CNN
F 3 "" H 10450 2400 60  0000 C CNN
	1    10450 2400
	1    0    0    -1  
$EndComp
$Comp
L MS5611-01BA03 U6
U 1 1 5251D2E8
P 3150 4700
F 0 "U6" H 2850 5050 60  0000 C CNN
F 1 "MS5611-01BA03" H 3150 4300 60  0000 C CNN
F 2 "" H 3150 4700 60  0000 C CNN
F 3 "" H 3150 4700 60  0000 C CNN
	1    3150 4700
	1    0    0    -1  
$EndComp
$Comp
L AGND #PWR071
U 1 1 5251D2F7
P 1750 5500
F 0 "#PWR071" H 1750 5500 40  0001 C CNN
F 1 "AGND" H 1750 5430 50  0000 C CNN
F 2 "" H 1750 5500 60  0000 C CNN
F 3 "" H 1750 5500 60  0000 C CNN
	1    1750 5500
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 5251D304
P 1750 5050
F 0 "R10" V 1830 5050 40  0000 C CNN
F 1 "100k" V 1757 5051 40  0000 C CNN
F 2 "0805" V 1680 5050 30  0001 C CNN
F 3 "~" H 1750 5050 30  0000 C CNN
	1    1750 5050
	-1   0    0    1   
$EndComp
$Comp
L AGND #PWR072
U 1 1 5251D323
P 2200 5500
F 0 "#PWR072" H 2200 5500 40  0001 C CNN
F 1 "AGND" H 2200 5430 50  0000 C CNN
F 2 "" H 2200 5500 60  0000 C CNN
F 3 "" H 2200 5500 60  0000 C CNN
	1    2200 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 4500 1250 4500
Wire Wire Line
	1750 4650 1750 4800
Wire Wire Line
	1750 5300 1750 5500
Wire Wire Line
	2200 4800 2200 5500
$Comp
L C C11
U 1 1 5251D3BD
P 1250 5000
F 0 "C11" H 1250 5100 40  0000 L CNN
F 1 "0.47uF 16V" H 1256 4915 40  0000 L CNN
F 2 "0805" H 1288 4850 30  0001 C CNN
F 3 "~" H 1250 5000 60  0000 C CNN
	1    1250 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 4650 1750 4650
Wire Wire Line
	2500 4800 2200 4800
Connection ~ 1250 4500
$Comp
L AGND #PWR073
U 1 1 5251D411
P 1250 5500
F 0 "#PWR073" H 1250 5500 40  0001 C CNN
F 1 "AGND" H 1250 5430 50  0000 C CNN
F 2 "" H 1250 5500 60  0000 C CNN
F 3 "" H 1250 5500 60  0000 C CNN
	1    1250 5500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 5200 1250 5500
Text GLabel 4800 4500 2    60   Input ~ 0
SENS_SSI_CLK
Text GLabel 4800 4650 2    60   Input ~ 0
SENS_SSI_TX
Text GLabel 4800 4800 2    60   Input ~ 0
SENS_SSI_RX
Text GLabel 6050 4950 2    60   Input ~ 0
BAROMETER_CS
Wire Wire Line
	3800 4500 4800 4500
Wire Wire Line
	4800 4650 3800 4650
Wire Wire Line
	3800 4800 4800 4800
Wire Wire Line
	3800 4950 6050 4950
Wire Wire Line
	2500 4950 2400 4950
Wire Wire Line
	2400 4950 2400 5150
Wire Wire Line
	2400 5150 4000 5150
Wire Wire Line
	4000 5150 4000 4950
Connection ~ 4000 4950
Text Label 4000 4500 0    60   ~ 0
SENS_SPI_CLK
Text Label 4000 4650 0    60   ~ 0
SENS_SPI_TX
Text Label 4000 4800 0    60   ~ 0
SENS_SPI_RX
$Comp
L AGND #PWR074
U 1 1 5251D4FC
P 9900 1900
F 0 "#PWR074" H 9900 1900 40  0001 C CNN
F 1 "AGND" H 9900 1830 50  0000 C CNN
F 2 "" H 9900 1900 60  0000 C CNN
F 3 "" H 9900 1900 60  0000 C CNN
	1    9900 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	10100 1800 10100 2050
Wire Wire Line
	9900 1650 9900 1900
Wire Wire Line
	9350 2150 10100 2150
Wire Wire Line
	9350 1100 9350 2150
$Comp
L C C14
U 1 1 5251D55A
P 9900 1450
F 0 "C14" H 9900 1550 40  0000 L CNN
F 1 "1uF 10V" H 9906 1365 40  0000 L CNN
F 2 "0805" H 9938 1300 30  0001 C CNN
F 3 "~" H 9900 1450 60  0000 C CNN
	1    9900 1450
	-1   0    0    1   
$EndComp
$Comp
L CONN_1 P9
U 1 1 5251D5E4
P 4600 2200
F 0 "P9" H 4680 2200 40  0000 L CNN
F 1 "TEST_POINT" H 4600 2255 30  0001 C CNN
F 2 "" H 4600 2200 60  0000 C CNN
F 3 "" H 4600 2200 60  0000 C CNN
	1    4600 2200
	1    0    0    -1  
$EndComp
$Comp
L CONN_1 P10
U 1 1 5251D5F3
P 8600 2350
F 0 "P10" H 8680 2350 40  0000 L CNN
F 1 "TEST_POINT" H 8600 2405 30  0001 C CNN
F 2 "" H 8600 2350 60  0000 C CNN
F 3 "" H 8600 2350 60  0000 C CNN
	1    8600 2350
	-1   0    0    1   
$EndComp
$Comp
L CONN_1 P11
U 1 1 5251D602
P 8600 2450
F 0 "P11" H 8680 2450 40  0000 L CNN
F 1 "TEST_POINT" H 8600 2505 30  0001 C CNN
F 2 "" H 8600 2450 60  0000 C CNN
F 3 "" H 8600 2450 60  0000 C CNN
	1    8600 2450
	-1   0    0    1   
$EndComp
Text Label 8850 2350 0    60   ~ 0
ACC_INT1
Text Label 8850 2450 0    60   ~ 0
ACC_INT2
Wire Wire Line
	10100 2450 8750 2450
Wire Wire Line
	10100 2350 8750 2350
Wire Wire Line
	8400 2250 10100 2250
Text GLabel 8400 2250 0    60   Input ~ 0
ACCEL_CS
Wire Wire Line
	10100 2550 8750 2550
Wire Wire Line
	10100 2650 8750 2650
Wire Wire Line
	10100 2750 8750 2750
Text Label 8900 2750 0    60   ~ 0
SENS_SPI_CLK
Text Label 8900 2550 0    60   ~ 0
SENS_SPI_RX
Text Label 8900 2650 0    60   ~ 0
SENS_SPI_TX
Wire Wire Line
	4450 2000 4450 2200
$Comp
L R R13
U 1 1 5251D93B
P 8700 1500
F 0 "R13" V 8780 1500 40  0000 C CNN
F 1 "100k" V 8707 1501 40  0000 C CNN
F 2 "0805" V 8630 1500 30  0001 C CNN
F 3 "~" H 8700 1500 30  0000 C CNN
	1    8700 1500
	-1   0    0    1   
$EndComp
Wire Wire Line
	8700 1250 8700 1200
Wire Wire Line
	8700 1200 9900 1200
Connection ~ 9350 1200
Wire Wire Line
	8700 1750 8700 2250
Connection ~ 8700 2250
$Comp
L R R12
U 1 1 5251D9F7
P 5900 4500
F 0 "R12" V 5980 4500 40  0000 C CNN
F 1 "100k" V 5907 4501 40  0000 C CNN
F 2 "0805" V 5830 4500 30  0001 C CNN
F 3 "~" H 5900 4500 30  0000 C CNN
	1    5900 4500
	-1   0    0    1   
$EndComp
Wire Wire Line
	5900 4000 5900 4250
Wire Wire Line
	5900 4750 5900 4950
Connection ~ 5900 4950
$Comp
L AGND #PWR075
U 1 1 5251DA81
P 1000 2500
F 0 "#PWR075" H 1000 2500 40  0001 C CNN
F 1 "AGND" H 1000 2430 50  0000 C CNN
F 2 "" H 1000 2500 60  0000 C CNN
F 3 "" H 1000 2500 60  0000 C CNN
	1    1000 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 1200 9900 1250
Wire Wire Line
	10100 1800 9900 1800
Connection ~ 9900 1800
$Comp
L VDD_SENSORS #PWR076
U 1 1 526652F5
P 9350 900
F 0 "#PWR076" H 9350 1100 60  0001 C CNN
F 1 "VDD_SENSORS" H 9350 1000 60  0000 C CNN
F 2 "~" H 9350 900 60  0000 C CNN
F 3 "~" H 9350 900 60  0000 C CNN
	1    9350 900 
	1    0    0    -1  
$EndComp
$Comp
L VDD_SENSORS #PWR077
U 1 1 526653D2
P 5900 3800
F 0 "#PWR077" H 5900 4000 60  0001 C CNN
F 1 "VDD_SENSORS" H 5900 3900 60  0000 C CNN
F 2 "~" H 5900 3800 60  0000 C CNN
F 3 "~" H 5900 3800 60  0000 C CNN
	1    5900 3800
	1    0    0    -1  
$EndComp
$Comp
L VDD_SENSORS #PWR078
U 1 1 526653E1
P 4450 1800
F 0 "#PWR078" H 4450 2000 60  0001 C CNN
F 1 "VDD_SENSORS" H 4450 1900 60  0000 C CNN
F 2 "~" H 4450 1800 60  0000 C CNN
F 3 "~" H 4450 1800 60  0000 C CNN
	1    4450 1800
	1    0    0    -1  
$EndComp
$Comp
L VDD_SENSORS #PWR079
U 1 1 526653F0
P 1250 4200
F 0 "#PWR079" H 1250 4400 60  0001 C CNN
F 1 "VDD_SENSORS" H 1250 4300 60  0000 C CNN
F 2 "~" H 1250 4200 60  0000 C CNN
F 3 "~" H 1250 4200 60  0000 C CNN
	1    1250 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 4400 1250 4800
$Comp
L PWR_FLAG #FLG080
U 1 1 52665DF6
P 2500 2650
F 0 "#FLG080" H 2500 2745 30  0001 C CNN
F 1 "PWR_FLAG" H 2500 2830 30  0000 C CNN
F 2 "" H 2500 2650 60  0000 C CNN
F 3 "" H 2500 2650 60  0000 C CNN
	1    2500 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 2650 2500 2700
Wire Wire Line
	2500 2700 2200 2700
Connection ~ 2200 2700
$EndSCHEMATC
