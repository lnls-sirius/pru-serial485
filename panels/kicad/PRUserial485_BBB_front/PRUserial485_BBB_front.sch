EESchema Schematic File Version 2
LIBS:Eurocard Template-rescue
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
LIBS:Eurocard Template-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L C64AC-RESCUE-Eurocard_Template P1
U 1 1 57517A5F
P 1375 4450
F 0 "P1" V 1625 1550 50  0000 C CNN
F 1 "C64AC" V 1625 1325 50  0000 C CNN
F 2 "Controle:Eurocard_template" H 1375 4450 50  0001 C CNN
F 3 "" H 1375 4450 50  0000 C CNN
	1    1375 4450
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR01
U 1 1 5751A3DC
P 950 1175
F 0 "#PWR01" H 950 925 50  0001 C CNN
F 1 "GND" H 955 1002 50  0000 C CNN
F 2 "" H 950 1175 50  0000 C CNN
F 3 "" H 950 1175 50  0000 C CNN
	1    950  1175
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 800  950  800 
Wire Wire Line
	950  800  950  1175
Wire Wire Line
	1200 1100 950  1100
Connection ~ 950  1100
Wire Wire Line
	1200 1000 950  1000
Connection ~ 950  1000
Wire Wire Line
	1200 900  950  900 
Connection ~ 950  900 
$EndSCHEMATC
