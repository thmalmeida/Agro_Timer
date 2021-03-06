EESchema Schematic File Version 2  date 01/05/2013 19:26:37
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
LIBS:thmalmeida
LIBS:opendous
LIBS:Timer-cache
EELAYER 25  0
EELAYER END
$Descr A4 11700 8267
encoding utf-8
Sheet 1 1
Title ""
Date "1 may 2013"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L +5V #PWR?
U 1 1 51817701
P 1600 4750
F 0 "#PWR?" H 1600 4840 20  0001 C CNN
F 1 "+5V" H 1600 4840 30  0000 C CNN
	1    1600 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 7050 1650 7050
Connection ~ 5550 4750
Wire Wire Line
	5550 4850 5550 4750
Wire Wire Line
	4850 5050 4850 5100
Connection ~ 5200 5100
Wire Wire Line
	5200 5000 5200 5150
Wire Wire Line
	9150 1350 9150 1500
Wire Wire Line
	9150 1500 9250 1500
Connection ~ 1550 6450
Wire Wire Line
	1900 6450 1550 6450
Wire Wire Line
	1550 6200 1550 6150
Wire Wire Line
	1550 6750 1550 6700
Wire Wire Line
	5550 1550 5600 1550
Wire Wire Line
	5750 1200 5800 1200
Wire Wire Line
	3700 2150 3900 2150
Wire Wire Line
	3650 4700 3650 4750
Wire Wire Line
	3650 4750 3800 4750
Wire Wire Line
	10250 1300 10250 1400
Wire Wire Line
	10250 1700 10250 1600
Wire Wire Line
	10250 1100 10250 1000
Wire Wire Line
	9950 1100 9950 1000
Wire Wire Line
	9950 1700 9950 1600
Wire Wire Line
	9150 1150 9150 1050
Wire Wire Line
	1600 4750 1600 4800
Wire Wire Line
	1600 4800 1550 4800
Wire Wire Line
	900  1450 850  1450
Wire Wire Line
	2300 3500 2300 3650
Connection ~ 2300 1000
Wire Wire Line
	2300 1050 2300 1000
Connection ~ 2300 3550
Wire Wire Line
	5000 1350 5050 1350
Wire Wire Line
	5550 1350 5550 1200
Wire Wire Line
	5800 1600 5800 1700
Wire Wire Line
	5050 1550 5000 1550
Wire Wire Line
	2200 3500 2200 3550
Wire Wire Line
	2200 3550 2400 3550
Wire Wire Line
	2400 3550 2400 3500
Wire Wire Line
	2200 1050 2200 1000
Wire Wire Line
	2200 1000 2400 1000
Wire Wire Line
	2400 1050 2400 950 
Connection ~ 2400 1000
Wire Wire Line
	850  1450 850  1400
Wire Wire Line
	1550 5000 1600 5000
Wire Wire Line
	1600 5000 1600 5050
Wire Wire Line
	9000 1400 9150 1400
Connection ~ 9150 1400
Wire Wire Line
	9950 1300 9950 1400
Wire Wire Line
	3650 5000 3650 4900
Wire Wire Line
	3650 4900 3800 4900
Wire Wire Line
	3700 2050 3900 2050
Wire Wire Line
	5800 1150 5800 1300
Connection ~ 5800 1200
Wire Wire Line
	5000 1550 5000 1650
Wire Wire Line
	1550 6400 1550 6500
Wire Wire Line
	5000 2650 5000 2550
Connection ~ 5800 2200
Wire Wire Line
	5800 2150 5800 2300
Wire Wire Line
	5000 2550 5050 2550
Wire Wire Line
	5800 2600 5800 2700
Wire Wire Line
	5550 2350 5550 2200
Wire Wire Line
	5000 2350 5050 2350
Wire Wire Line
	5750 2200 5800 2200
Wire Wire Line
	5550 2550 5600 2550
Wire Wire Line
	9150 1850 9150 1650
Wire Wire Line
	9150 1650 9250 1650
Wire Wire Line
	4950 4750 4800 4750
Wire Wire Line
	4800 4750 4800 4700
Wire Wire Line
	5450 4750 5600 4750
Wire Wire Line
	5600 4750 5600 4700
Wire Wire Line
	4850 4850 4850 4750
Connection ~ 4850 4750
Wire Wire Line
	5550 5050 5550 5100
Wire Wire Line
	5550 5100 4850 5100
Wire Wire Line
	1650 7050 1650 7000
$Comp
L +3V3 #PWR?
U 1 1 517F1F06
P 1650 7000
F 0 "#PWR?" H 1650 7140 20  0001 C CNN
F 1 "+3V3" H 1650 7110 30  0000 C CNN
	1    1650 7000
	1    0    0    -1  
$EndComp
Text Label 1650 7000 2    45   ~ 9
Vdd
Text Label 1900 6350 2    45   ~ 9
Vdd
Text Label 1550 6150 0    45   ~ 9
Vdd
$Comp
L +3V3 #PWR01
U 1 1 517F1C41
P 1550 6150
F 0 "#PWR01" H 1550 6290 20  0001 C CNN
F 1 "+3V3" H 1550 6260 30  0000 C CNN
	1    1550 6150
	1    0    0    -1  
$EndComp
Text Notes 5950 4500 0    60   ~ 12
Jumper Select
Text Label 6400 4900 3    45   ~ 9
Vdd
Text Label 6200 4900 3    45   ~ 9
Vcc
Text Label 6300 4900 3    45   ~ 9
Vp
$Comp
L CONN3_MINI Conn3_mini1
U 1 1 517F0B5C
P 6300 4750
F 0 "Conn3_mini1" H 6300 4925 50  0000 C CNN
F 1 "CONN3_MINI" H 6300 4825 50  0000 C CNN
	1    6300 4750
	1    0    0    -1  
$EndComp
Text Label 3650 4700 2    45   ~ 9
Vp
Text Label 1900 6850 2    45   ~ 9
PC4
Text Label 3100 7050 0    45   ~ 9
PC0
$Comp
L +3V3 #PWR03
U 1 1 517EE6B0
P 2400 950
F 0 "#PWR03" H 2400 1090 20  0001 C CNN
F 1 "+3V3" H 2400 1060 30  0000 C CNN
	1    2400 950 
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR04
U 1 1 517EE6A1
P 850 1200
F 0 "#PWR04" H 850 1340 20  0001 C CNN
F 1 "+3V3" H 850 1310 30  0000 C CNN
	1    850  1200
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR05
U 1 1 517EE68D
P 9150 1050
F 0 "#PWR05" H 9150 1190 20  0001 C CNN
F 1 "+3V3" H 9150 1160 30  0000 C CNN
	1    9150 1050
	1    0    0    -1  
$EndComp
$Comp
L C_MINI 330n1
U 1 1 517EDFEC
P 5550 4950
F 0 "330n1" V 5500 4990 30  0000 C CNN
F 1 "C_MINI" V 5600 5020 25  0000 C CNN
	1    5550 4950
	0    1    1    0   
$EndComp
$Comp
L CP_MINI 10u1
U 1 1 517EDFCD
P 4850 4950
F 0 "10u1" V 4800 4990 30  0000 C CNN
F 1 "CP_MINI" V 4900 5020 25  0000 C CNN
	1    4850 4950
	0    -1   1    0   
$EndComp
Text Label 5600 4750 0    45   ~ 9
Vdd
$Comp
L GND1 #PWR06
U 1 1 517EDDCD
P 5200 5150
F 0 "#PWR06" H 5200 5150 30  0001 C CNN
F 1 "GND1" H 5200 5080 30  0000 C CNN
	1    5200 5150
	1    0    0    -1  
$EndComp
Text Label 5200 5050 2    45   ~ 9
GND
$Comp
L +5V #PWR07
U 1 1 517EDDBB
P 4800 4700
F 0 "#PWR07" H 4800 4790 20  0001 C CNN
F 1 "+5V" H 4800 4790 30  0000 C CNN
	1    4800 4700
	1    0    0    -1  
$EndComp
Text Label 4800 4750 2    45   ~ 9
Vcc
Text Notes 4800 4500 0    60   ~ 12
Regulator (3v3)
$Comp
L AMS1117_REGULATOR 3v3_Reg1
U 1 1 517EDD87
P 5200 4800
F 0 "3v3_Reg1" H 5050 4675 40  0000 C CNN
F 1 "AMS1117_REGULATOR" H 5200 4945 40  0000 C CNN
	1    5200 4800
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR08
U 1 1 517EDD49
P 5600 4700
F 0 "#PWR08" H 5600 4840 20  0001 C CNN
F 1 "+3V3" H 5600 4810 30  0000 C CNN
	1    5600 4700
	1    0    0    -1  
$EndComp
Text Label 2200 1000 2    45   ~ 9
Vdd
Text Label 2200 3550 2    45   ~ 9
GND
Text Label 2900 5050 3    45   ~ 9
Rx
Text Label 3000 5050 3    45   ~ 9
GND
Text Label 2700 5050 3    45   ~ 9
Vp
Text Label 2200 5050 3    45   ~ 9
Vp
Text Label 2300 5050 3    45   ~ 9
GND
$Comp
L TX_CONN Connector1
U 1 1 5179C438
P 2200 4850
F 0 "Connector1" H 2200 5000 50  0000 C CNN
F 1 "TX_CONN" H 2200 4900 50  0000 C CNN
	1    2200 4850
	1    0    0    -1  
$EndComp
$Comp
L RX_CONN Connector2
U 1 1 5179C431
P 2850 4850
F 0 "Connector2" H 2850 5000 50  0000 C CNN
F 1 "RX_CONN" H 2850 4900 50  0000 C CNN
	1    2850 4850
	1    0    0    -1  
$EndComp
Text Notes 5200 1000 0    60   ~ 12
Start
Text Notes 5200 2050 0    60   ~ 12
Stop
Text Notes 8050 950  0    60   ~ 12
Stop
Text Label 5800 2700 2    45   ~ 9
95_NC
Text Label 5800 2200 0    45   ~ 9
5_L3
Text Label 5800 1700 2    45   ~ 9
13_NO
Text Label 5800 1200 0    45   ~ 9
A1
Text Notes 7450 950  0    60   ~ 12
Start
Text Label 8000 1400 2    45   ~ 9
95_NC
Text Label 8000 1250 2    45   ~ 9
5_L3
Text Label 7250 1400 2    45   ~ 9
13_NO
Text Label 7250 1250 2    45   ~ 9
A1
$Comp
L CONECTOR_2 Conector4
U 1 1 5179BC56
P 9400 1600
F 0 "Conector4" H 9400 1450 50  0000 C CNN
F 1 "PushButton" H 9400 1800 50  0000 C CNN
	1    9400 1600
	1    0    0    -1  
$EndComp
$Comp
L CONECTOR_2 Conector2
U 1 1 5179BC33
P 7400 1350
F 0 "Conector2" H 7400 1200 50  0000 C CNN
F 1 "Start" H 7400 1550 50  0000 C CNN
	1    7400 1350
	1    0    0    -1  
$EndComp
$Comp
L CONECTOR_2 Conector3
U 1 1 5179BC29
P 8150 1350
F 0 "Conector3" H 8150 1200 50  0000 C CNN
F 1 "Stop" H 8150 1550 50  0000 C CNN
	1    8150 1350
	1    0    0    -1  
$EndComp
$Comp
L GND1 #PWR09
U 1 1 5179B63F
P 5000 2650
F 0 "#PWR09" H 5000 2650 30  0001 C CNN
F 1 "GND1" H 5000 2580 30  0000 C CNN
	1    5000 2650
	1    0    0    -1  
$EndComp
$Comp
L OPTOCOUPLER Diac2
U 1 1 5179B63E
P 5300 2450
F 0 "Diac2" H 5300 2600 30  0000 C CNN
F 1 "OPTOCOUPLER" H 5300 2300 25  0000 C CNN
	1    5300 2450
	1    0    0    -1  
$EndComp
$Comp
L R_MINI R7
U 1 1 5179B63D
P 5650 2200
F 0 "R7" H 5580 2250 25  0000 C CNN
F 1 "1k" H 5720 2250 20  0000 C CNN
	1    5650 2200
	1    0    0    -1  
$EndComp
$Comp
L R_MINI R3
U 1 1 5179B63C
P 4900 2350
F 0 "R3" H 4830 2400 25  0000 C CNN
F 1 "1k" H 4970 2400 20  0000 C CNN
	1    4900 2350
	1    0    0    -1  
$EndComp
$Comp
L PHASE_R #U010
U 1 1 5179B63B
P 5800 2100
F 0 "#U010" H 5850 2100 40  0001 C CNN
F 1 "PHASE_R" H 5800 2200 30  0000 C CNN
	1    5800 2100
	1    0    0    -1  
$EndComp
$Comp
L TRIAC_P T2
U 1 1 5179B63A
P 5800 2450
F 0 "T2" H 5850 2600 35  0000 C CNN
F 1 "TRIAC_BT136" H 5900 2350 30  0000 C CNN
	1    5800 2450
	1    0    0    -1  
$EndComp
Text Label 4800 2350 2    45   ~ 9
PD4
$Comp
L GND1 #PWR011
U 1 1 51210D45
P 1550 6750
F 0 "#PWR011" H 1550 6750 30  0001 C CNN
F 1 "GND1" H 1550 6680 30  0000 C CNN
	1    1550 6750
	1    0    0    -1  
$EndComp
$Comp
L R_MINI R6
U 1 1 51210D3D
P 1550 6600
F 0 "R6" H 1480 6650 25  0000 C CNN
F 1 "1k5" H 1620 6650 20  0000 C CNN
	1    1550 6600
	0    -1   -1   0   
$EndComp
$Comp
L R_MINI R4
U 1 1 51210D3B
P 1550 6300
F 0 "R4" H 1480 6350 25  0000 C CNN
F 1 "8k2" H 1620 6350 20  0000 C CNN
	1    1550 6300
	0    -1   -1   0   
$EndComp
$Comp
L R_MINI R2
U 1 1 51210D2E
P 1800 7050
F 0 "R2" H 1730 7100 25  0000 C CNN
F 1 "680" H 1870 7100 20  0000 C CNN
	1    1800 7050
	1    0    0    -1  
$EndComp
Text Label 1900 6650 2    45   ~ 9
PC5
Text Label 4800 1350 2    45   ~ 9
PD3
Text Label 9950 1000 2    45   ~ 9
PB2
Text Label 9000 1400 2    45   ~ 9
PD2
Text Label 10250 1000 2    45   ~ 9
PB1
Text Notes 2200 700  0    60   ~ 12
uC Unit
Text Label 3100 6750 0    45   ~ 9
PC3
Text Label 3100 6850 0    45   ~ 9
PC2
Text Label 3100 6950 0    45   ~ 9
PC1
Text Label 1900 6750 2    45   ~ 9
GND
Text Label 1900 6250 2    45   ~ 9
GND
Text Label 1900 7150 2    45   ~ 9
GND
Text Label 3650 4950 2    45   ~ 9
GND
Text Notes 9750 850  0    60   ~ 12
LED Indicator
Text Notes 8900 850  0    60   ~ 12
Push Button
Text Notes 2050 4550 0    60   ~ 12
Serial Communication
Text Notes 3450 4500 0    60   ~ 12
Power Supply (5V)
Text Notes 2050 5900 0    60   ~ 12
Liquid Crystal Display
Text Notes 1100 4550 0    60   ~ 12
Programmer
$Comp
L LCD U3
U 1 1 5120D10C
P 2500 6750
F 0 "U3" H 2500 6800 60  0000 C CNN
F 1 "LCD" H 2750 7350 60  0000 C CNN
	1    2500 6750
	1    0    0    -1  
$EndComp
Text Label 2800 5050 3    45   ~ 9
Rx
Text Label 2100 5050 3    45   ~ 9
Tx
Text Label 3900 2150 0    45   ~ 9
Rx
Text Label 3900 2050 0    45   ~ 9
Tx
$Comp
L GND1 #PWR013
U 1 1 50C79DCA
P 3650 5000
F 0 "#PWR013" H 3650 5000 30  0001 C CNN
F 1 "GND1" H 3650 4930 30  0000 C CNN
	1    3650 5000
	1    0    0    -1  
$EndComp
$Comp
L CONECTOR_2 Conector1
U 1 1 50C79D98
P 3950 4850
F 0 "Conector1" H 3950 4700 50  0000 C CNN
F 1 "Supply" H 3950 5050 50  0000 C CNN
	1    3950 4850
	1    0    0    -1  
$EndComp
$Comp
L R_MINI R14
U 1 1 50C79C87
P 10250 1200
F 0 "R14" H 10180 1250 25  0000 C CNN
F 1 "680" H 10320 1250 20  0000 C CNN
	1    10250 1200
	0    -1   -1   0   
$EndComp
$Comp
L D_LED D2
U 1 1 50C79C86
P 10250 1500
F 0 "D2" H 10250 1550 30  0000 C CNN
F 1 "D_LED" H 10250 1450 25  0000 C CNN
	1    10250 1500
	0    1    1    0   
$EndComp
$Comp
L GND1 #PWR014
U 1 1 50C79C85
P 10250 1700
F 0 "#PWR014" H 10250 1700 30  0001 C CNN
F 1 "GND1" H 10250 1630 30  0000 C CNN
	1    10250 1700
	1    0    0    -1  
$EndComp
$Comp
L GND1 #PWR015
U 1 1 50C79C60
P 9950 1700
F 0 "#PWR015" H 9950 1700 30  0001 C CNN
F 1 "GND1" H 9950 1630 30  0000 C CNN
	1    9950 1700
	1    0    0    -1  
$EndComp
$Comp
L GND1 #PWR016
U 1 1 50C79C21
P 9150 1850
F 0 "#PWR016" H 9150 1850 30  0001 C CNN
F 1 "GND1" H 9150 1780 30  0000 C CNN
	1    9150 1850
	1    0    0    -1  
$EndComp
$Comp
L R_MINI R11
U 1 1 50C79B16
P 9150 1250
F 0 "R11" H 9080 1300 25  0000 C CNN
F 1 "R_MINI" H 9220 1300 20  0000 C CNN
	1    9150 1250
	0    -1   -1   0   
$EndComp
$Comp
L D_LED D1
U 1 1 50C79B11
P 9950 1500
F 0 "D1" H 9950 1550 30  0000 C CNN
F 1 "D_LED" H 9950 1450 25  0000 C CNN
	1    9950 1500
	0    1    1    0   
$EndComp
$Comp
L R_MINI R13
U 1 1 50C79ACA
P 9950 1200
F 0 "R13" H 9880 1250 25  0000 C CNN
F 1 "680" H 10020 1250 20  0000 C CNN
	1    9950 1200
	0    -1   -1   0   
$EndComp
$Comp
L GND1 #PWR017
U 1 1 50C79AA2
P 1600 5050
F 0 "#PWR017" H 1600 5050 30  0001 C CNN
F 1 "GND1" H 1600 4980 30  0000 C CNN
	1    1600 5050
	1    0    0    -1  
$EndComp
Text Label 1050 5000 2    45   ~ 9
RST
Text Label 850  1450 2    45   ~ 9
RST
Text Label 1550 4900 0    45   ~ 9
MOSI
Text Label 1050 4800 2    45   ~ 9
MISO
Text Label 1050 4900 2    45   ~ 9
SCK
$Comp
L GND1 #PWR018
U 1 1 50C79A32
P 2300 3650
F 0 "#PWR018" H 2300 3650 30  0001 C CNN
F 1 "GND1" H 2300 3580 30  0000 C CNN
	1    2300 3650
	1    0    0    -1  
$EndComp
Text Label 900  1650 2    45   ~ 9
PB7
Text Label 900  1850 2    45   ~ 9
PB6
Text Label 900  2850 2    45   ~ 9
PB0
Text Label 900  2750 2    45   ~ 9
PB1
Text Label 900  2650 2    45   ~ 9
PB2
Text Label 900  2550 2    45   ~ 9
MOSI
Text Label 900  2450 2    45   ~ 9
MISO
Text Label 900  2350 2    45   ~ 9
SCK
$Comp
L R_MINI R1
U 1 1 50C79949
P 850 1300
F 0 "R1" H 780 1350 25  0000 C CNN
F 1 "R_MINI" H 920 1350 20  0000 C CNN
	1    850  1300
	0    1    -1   0   
$EndComp
$Comp
L ATMEGA8-TQFP32 IC1
U 1 1 50C79927
P 2300 2250
F 0 "IC1" H 1200 3250 50  0000 C CNN
F 1 "ATMEGA8-TQFP32" H 3000 3250 50  0000 C CNN
	1    2300 2250
	1    0    0    -1  
$EndComp
Text Label 3700 3150 0    45   ~ 9
PC0
Text Label 3700 3050 0    45   ~ 9
PC1
Text Label 3700 2950 0    45   ~ 9
PC2
Text Label 3700 2850 0    45   ~ 9
PC3
Text Label 3700 2750 0    45   ~ 9
PC4
Text Label 3700 2650 0    45   ~ 9
PC5
Text Label 3700 2550 0    45   ~ 9
ADC6
Text Label 3700 2450 0    45   ~ 9
ADC7
Text Label 3700 2150 0    45   ~ 9
PD0
Text Label 3700 2050 0    45   ~ 9
PD1
Text Label 3700 1950 0    45   ~ 9
PD2
Text Label 3700 1850 0    45   ~ 9
PD3
Text Label 3700 1750 0    45   ~ 9
PD4
Text Label 3700 1650 0    45   ~ 9
PD5
Text Label 3700 1550 0    45   ~ 9
PD6
$Comp
L TRIAC_P T1
U 1 1 50C79625
P 5800 1450
F 0 "T1" H 5850 1600 35  0000 C CNN
F 1 "TRIAC_BT136" H 5900 1350 30  0000 C CNN
	1    5800 1450
	1    0    0    -1  
$EndComp
Text Label 3700 1450 0    45   ~ 9
PD7
$Comp
L PHASE_R #U019
U 1 1 50C79389
P 5800 1100
F 0 "#U019" H 5850 1100 40  0001 C CNN
F 1 "PHASE_R" H 5800 1200 30  0000 C CNN
	1    5800 1100
	1    0    0    -1  
$EndComp
$Comp
L R_MINI R5
U 1 1 50C792BE
P 4900 1350
F 0 "R5" H 4830 1400 25  0000 C CNN
F 1 "1k" H 4970 1400 20  0000 C CNN
	1    4900 1350
	1    0    0    -1  
$EndComp
$Comp
L R_MINI R8
U 1 1 50C792BD
P 5650 1200
F 0 "R8" H 5580 1250 25  0000 C CNN
F 1 "1k" H 5720 1250 20  0000 C CNN
	1    5650 1200
	1    0    0    -1  
$EndComp
$Comp
L OPTOCOUPLER Diac1
U 1 1 50C792BB
P 5300 1450
F 0 "Diac1" H 5300 1600 30  0000 C CNN
F 1 "OPTOCOUPLER" H 5300 1300 25  0000 C CNN
	1    5300 1450
	1    0    0    -1  
$EndComp
$Comp
L GND1 #PWR020
U 1 1 50C792B8
P 5000 1650
F 0 "#PWR020" H 5000 1650 30  0001 C CNN
F 1 "GND1" H 5000 1580 30  0000 C CNN
	1    5000 1650
	1    0    0    -1  
$EndComp
$Comp
L ICSP_SPI_6PIN U1
U 1 1 50C78889
P 1300 4900
F 0 "U1" H 1200 4675 40  0000 C CNN
F 1 "ICSP_SPI_6PIN" H 1300 5095 40  0000 C CNN
	1    1300 4900
	1    0    0    -1  
$EndComp
$EndSCHEMATC
