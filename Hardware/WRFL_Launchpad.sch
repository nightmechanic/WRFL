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
Sheet 3 5
Title ""
Date "3 oct 2013"
Rev ""
Comp "Nightmechanic"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_2 P?
U 1 1 524B47CF
P 5900 1550
F 0 "P?" V 5850 1550 40  0000 C CNN
F 1 "1x2_2.54_Fem" V 5950 1550 40  0000 C CNN
F 2 "" H 5900 1550 60  0000 C CNN
F 3 "" H 5900 1550 60  0000 C CNN
	1    5900 1550
	1    0    0    -1  
$EndComp
NoConn ~ 5550 1650
$Comp
L VDD_MCU U?
U 1 1 524B47E3
P 5200 1100
F 0 "U?" H 5200 1300 60  0001 C CNN
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
$EndSCHEMATC
