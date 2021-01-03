EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "jeu. 02 avril 2015"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Transistor_Array:ULN2003 U?
U 1 1 5FF2DB20
P 3800 3450
F 0 "U?" V 3754 3980 50  0000 L CNN
F 1 "ULN2003" V 3845 3980 50  0000 L CNN
F 2 "" H 3850 2900 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/uln2003a.pdf" H 3900 3250 50  0001 C CNN
	1    3800 3450
	0    -1   1    0   
$EndComp
$Comp
L power:-15V #PWR?
U 1 1 5FF4977F
P 1350 5150
F 0 "#PWR?" H 1350 5250 50  0001 C CNN
F 1 "-15V" H 1365 5323 50  0000 C CNN
F 2 "" H 1350 5150 50  0001 C CNN
F 3 "" H 1350 5150 50  0001 C CNN
	1    1350 5150
	1    0    0    -1  
$EndComp
$Comp
L power:VAC #PWR?
U 1 1 5FF4CB7C
P 1350 4100
F 0 "#PWR?" H 1350 4000 50  0001 C CNN
F 1 "VAC" H 1350 4375 50  0000 C CNN
F 2 "" H 1350 4100 50  0001 C CNN
F 3 "" H 1350 4100 50  0001 C CNN
	1    1350 4100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FF4D7BF
P 1350 4650
F 0 "#PWR?" H 1350 4400 50  0001 C CNN
F 1 "GND" H 1355 4477 50  0000 C CNN
F 2 "" H 1350 4650 50  0001 C CNN
F 3 "" H 1350 4650 50  0001 C CNN
	1    1350 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 4750 1750 5150
Wire Wire Line
	1750 5150 1350 5150
Wire Wire Line
	2800 4950 2800 4750
Wire Wire Line
	1350 4100 1350 4300
Wire Wire Line
	3000 4950 3000 4300
Wire Wire Line
	2600 4950 2600 4300
Wire Wire Line
	1350 4300 2600 4300
Connection ~ 2600 4300
$Comp
L power:GND #PWR?
U 1 1 5FF9D1EC
P 3000 3450
F 0 "#PWR?" H 3000 3200 50  0001 C CNN
F 1 "GND" H 3005 3277 50  0000 C CNN
F 2 "" H 3000 3450 50  0001 C CNN
F 3 "" H 3000 3450 50  0001 C CNN
	1    3000 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 3450 3200 3450
$Comp
L Device:R R?
U 1 1 5FF9FCBC
P 2100 5950
F 0 "R?" H 2170 5996 50  0000 L CNN
F 1 "R" H 2170 5905 50  0000 L CNN
F 2 "" V 2030 5950 50  0001 C CNN
F 3 "~" H 2100 5950 50  0001 C CNN
	1    2100 5950
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR?
U 1 1 5FFA0B89
P 1450 6750
F 0 "#PWR?" H 1450 6600 50  0001 C CNN
F 1 "+12V" H 1465 6923 50  0000 C CNN
F 2 "" H 1450 6750 50  0001 C CNN
F 3 "" H 1450 6750 50  0001 C CNN
	1    1450 6750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 6750 2100 6750
Wire Wire Line
	2100 5800 2100 5550
$Comp
L Device:R R?
U 1 1 5FFCB17B
P 3900 5900
F 0 "R?" H 3970 5946 50  0000 L CNN
F 1 "R" H 3970 5855 50  0000 L CNN
F 2 "" V 3830 5900 50  0001 C CNN
F 3 "~" H 3900 5900 50  0001 C CNN
	1    3900 5900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5FFCDAAB
P 5750 5900
F 0 "R?" H 5820 5946 50  0000 L CNN
F 1 "R" H 5820 5855 50  0000 L CNN
F 2 "" V 5680 5900 50  0001 C CNN
F 3 "~" H 5750 5900 50  0001 C CNN
	1    5750 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 6350 3900 6050
Wire Wire Line
	5750 6350 5750 6050
$Comp
L Relay:FINDER-30.22 K1
U 1 1 5FF39B73
P 2500 5250
F 0 "K1" H 3130 5296 50  0000 L CNN
F 1 "Block 1" H 3130 5205 50  0000 L CNN
F 2 "Relay_THT:Relay_DPDT_Finder_30.22" H 3850 5220 50  0001 C CNN
F 3 "http://gfinder.findernet.com/assets/Series/354/S30EN.pdf" H 2500 5250 50  0001 C CNN
	1    2500 5250
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J?
U 1 1 5FF4948A
P 2750 6750
F 0 "J?" V 2622 6830 50  0000 L CNN
F 1 "Block 1" V 2713 6830 50  0000 L CNN
F 2 "" H 2750 6750 50  0001 C CNN
F 3 "~" H 2750 6750 50  0001 C CNN
	1    2750 6750
	0    1    1    0   
$EndComp
Text Notes 2800 7200 1    50   ~ 0
Block 1\nNothalt\nAbschnitt
Wire Wire Line
	2100 6350 2100 6750
Wire Wire Line
	2100 6100 2100 6350
Connection ~ 2100 6350
Wire Wire Line
	2500 5550 2500 6000
Wire Wire Line
	2500 6000 2650 6000
Wire Wire Line
	2650 6000 2650 6550
Wire Wire Line
	2750 6550 2750 6000
Wire Wire Line
	2750 6000 2900 6000
Wire Wire Line
	2900 6000 2900 5550
$Comp
L Connector:Screw_Terminal_01x02 J?
U 1 1 5FF9555F
P 4550 6750
F 0 "J?" V 4422 6830 50  0000 L CNN
F 1 "Block 2" V 4513 6830 50  0000 L CNN
F 2 "" H 4550 6750 50  0001 C CNN
F 3 "~" H 4550 6750 50  0001 C CNN
	1    4550 6750
	0    1    1    0   
$EndComp
Wire Wire Line
	4300 6150 4450 6150
Wire Wire Line
	4450 6150 4450 6550
Wire Wire Line
	4550 6550 4550 6150
Wire Wire Line
	4550 6150 4700 6150
Text Notes 4600 7200 1    50   ~ 0
Block 2\nNothalt\nAbschnitt
$Comp
L power:+12V #PWR?
U 1 1 5FFB7287
P 4450 3850
F 0 "#PWR?" H 4450 3700 50  0001 C CNN
F 1 "+12V" H 4465 4023 50  0000 C CNN
F 2 "" H 4450 3850 50  0001 C CNN
F 3 "" H 4450 3850 50  0001 C CNN
	1    4450 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 3850 4450 3850
Wire Wire Line
	2400 4550 2400 4950
Wire Wire Line
	5750 5750 5750 5550
Wire Wire Line
	6050 4950 6050 5050
$Comp
L Relay:FINDER-30.22 K3
U 1 1 5FF33D60
P 6150 5250
F 0 "K3" H 6780 5296 50  0000 L CNN
F 1 "Block 3" H 6780 5205 50  0000 L CNN
F 2 "Relay_THT:Relay_DPDT_Finder_30.22" H 7500 5220 50  0001 C CNN
F 3 "http://gfinder.findernet.com/assets/Series/354/S30EN.pdf" H 6150 5250 50  0001 C CNN
	1    6150 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 5750 3900 5550
Wire Wire Line
	4800 4950 4800 4300
Wire Wire Line
	4600 4750 4600 4950
Wire Wire Line
	4700 6150 4700 5550
Wire Wire Line
	4400 4950 4400 4300
Wire Wire Line
	4300 5550 4300 6150
$Comp
L Relay:FINDER-30.22 K2
U 1 1 5FF33A10
P 4300 5250
F 0 "K2" H 4930 5296 50  0000 L CNN
F 1 "Block 2" H 4930 5205 50  0000 L CNN
F 2 "Relay_THT:Relay_DPDT_Finder_30.22" H 5650 5220 50  0001 C CNN
F 3 "http://gfinder.findernet.com/assets/Series/354/S30EN.pdf" H 4300 5250 50  0001 C CNN
	1    4300 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 4300 3000 4300
Wire Wire Line
	1750 4750 2800 4750
Wire Wire Line
	3000 4300 4400 4300
Connection ~ 3000 4300
Connection ~ 4400 4300
Wire Wire Line
	6250 4300 6250 4950
Connection ~ 4800 4300
Wire Wire Line
	6250 4300 6650 4300
Wire Wire Line
	6650 4300 6650 4950
Connection ~ 6250 4300
Connection ~ 2800 4750
Wire Wire Line
	2400 4550 1350 4550
Wire Wire Line
	5750 4950 5750 4100
Wire Wire Line
	2100 6350 3900 6350
Connection ~ 3900 6350
Wire Wire Line
	5750 6350 3900 6350
Wire Wire Line
	4800 4300 6250 4300
Wire Wire Line
	4500 4300 4800 4300
Wire Wire Line
	4400 4300 4800 4300
Wire Wire Line
	2800 4750 4600 4750
Wire Wire Line
	2400 4550 4200 4550
Wire Wire Line
	4200 4550 4200 4950
Connection ~ 2400 4550
Wire Wire Line
	4200 4550 6050 4550
Wire Wire Line
	6050 4550 6050 4950
Connection ~ 4200 4550
Connection ~ 6050 4950
Wire Wire Line
	6450 4950 6450 4750
Wire Wire Line
	6450 4750 4600 4750
Connection ~ 4600 4750
Text Notes 6500 7200 1    50   ~ 0
Block 3\nNothalt\nAbschnitt
$Comp
L Connector:Screw_Terminal_01x02 J?
U 1 1 5FFA40A9
P 6450 6750
F 0 "J?" V 6322 6830 50  0000 L CNN
F 1 "Block 3" V 6413 6830 50  0000 L CNN
F 2 "" H 6450 6750 50  0001 C CNN
F 3 "~" H 6450 6750 50  0001 C CNN
	1    6450 6750
	0    1    1    0   
$EndComp
Wire Wire Line
	6150 5550 6150 6200
Wire Wire Line
	6150 6200 6350 6200
Wire Wire Line
	6350 6200 6350 6550
Wire Wire Line
	6450 6550 6450 6200
Wire Wire Line
	6450 6200 6550 6200
Wire Wire Line
	6550 6200 6550 5550
$Comp
L power:GND #PWR?
U 1 1 6008ECA9
P 4900 3200
F 0 "#PWR?" H 4900 2950 50  0001 C CNN
F 1 "GND" H 4905 3027 50  0000 C CNN
F 2 "" H 4900 3200 50  0001 C CNN
F 3 "" H 4900 3200 50  0001 C CNN
	1    4900 3200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 6009341E
P 1550 1650
F 0 "R?" H 1620 1696 50  0000 L CNN
F 1 "5k1" H 1620 1605 50  0000 L CNN
F 2 "" V 1480 1650 50  0001 C CNN
F 3 "~" H 1550 1650 50  0001 C CNN
	1    1550 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:D D?
U 1 1 600946A3
P 1800 1400
F 0 "D?" H 1800 1184 50  0000 C CNN
F 1 "1N4002" H 1800 1275 50  0000 C CNN
F 2 "" H 1800 1400 50  0001 C CNN
F 3 "~" H 1800 1400 50  0001 C CNN
	1    1800 1400
	-1   0    0    1   
$EndComp
$Comp
L Device:R R?
U 1 1 6009687C
P 1200 1650
F 0 "R?" H 1270 1696 50  0000 L CNN
F 1 "22k" H 1270 1605 50  0000 L CNN
F 2 "" V 1130 1650 50  0001 C CNN
F 3 "~" H 1200 1650 50  0001 C CNN
	1    1200 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 600987E6
P 2000 1650
F 0 "R?" H 2070 1696 50  0000 L CNN
F 1 "51k" H 2070 1605 50  0000 L CNN
F 2 "" V 1930 1650 50  0001 C CNN
F 3 "~" H 2000 1650 50  0001 C CNN
	1    2000 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 1800 1550 1800
Wire Wire Line
	2000 1800 1550 1800
Connection ~ 1550 1800
$Comp
L power:GND #PWR?
U 1 1 6009F233
P 1550 1950
F 0 "#PWR?" H 1550 1700 50  0001 C CNN
F 1 "GND" H 1555 1777 50  0000 C CNN
F 2 "" H 1550 1950 50  0001 C CNN
F 3 "" H 1550 1950 50  0001 C CNN
	1    1550 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 1800 1550 1950
Wire Wire Line
	1200 1500 1200 1400
Wire Wire Line
	1200 1400 1550 1400
Wire Wire Line
	1550 1500 1550 1400
Connection ~ 1550 1400
Wire Wire Line
	1550 1400 1650 1400
Wire Wire Line
	1950 1400 2000 1400
Wire Wire Line
	2000 1400 2000 1500
Connection ~ 2000 1400
Wire Wire Line
	700  4300 1050 4300
Connection ~ 1200 1400
Connection ~ 1350 4300
$Comp
L Connector:Screw_Terminal_01x03 J?
U 1 1 600B991E
P 850 4600
F 0 "J?" H 768 4917 50  0000 C CNN
F 1 "Screw_Terminal_01x03" H 768 4826 50  0000 C CNN
F 2 "" H 850 4600 50  0001 C CNN
F 3 "~" H 850 4600 50  0001 C CNN
	1    850  4600
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1050 4500 1050 4300
Connection ~ 1050 4300
Wire Wire Line
	1050 4300 1350 4300
Wire Wire Line
	1050 4600 1350 4600
Wire Wire Line
	1350 4550 1350 4600
Connection ~ 1350 4600
Wire Wire Line
	1350 4600 1350 4650
Wire Wire Line
	1050 4700 1050 5150
Wire Wire Line
	1050 5150 1350 5150
Connection ~ 1350 5150
$Comp
L Connector:Screw_Terminal_01x03 J?
U 1 1 600E93A9
P 6200 950
F 0 "J?" V 6164 762 50  0000 R CNN
F 1 "GBM In" V 6073 762 50  0000 R CNN
F 2 "" H 6200 950 50  0001 C CNN
F 3 "~" H 6200 950 50  0001 C CNN
	1    6200 950 
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 600ED017
P 5700 1800
F 0 "R?" V 5907 1800 50  0000 C CNN
F 1 "R" V 5816 1800 50  0000 C CNN
F 2 "" V 5630 1800 50  0001 C CNN
F 3 "~" H 5700 1800 50  0001 C CNN
	1    5700 1800
	0    -1   -1   0   
$EndComp
$Comp
L MCU_Module:Arduino_Nano_v3.x A?
U 1 1 5FFCAE2E
P 4850 1800
F 0 "A?" H 4850 711 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 4850 620 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 4850 1800 50  0001 C CIN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 4850 1800 50  0001 C CNN
	1    4850 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 1400 4350 1400
Wire Wire Line
	700  1400 1200 1400
Wire Wire Line
	700  1400 700  4300
Wire Wire Line
	4350 1500 3600 1500
Wire Wire Line
	3600 1500 3600 3050
Wire Wire Line
	3700 3050 3700 1600
Wire Wire Line
	3700 1600 4350 1600
Wire Wire Line
	3800 1700 3800 3050
Wire Wire Line
	3800 1700 4350 1700
Wire Wire Line
	3600 3850 3600 4100
Wire Wire Line
	3700 3850 3700 4950
Wire Wire Line
	3700 4950 3900 4950
Wire Wire Line
	3600 4100 2100 4100
Wire Wire Line
	2100 4100 2100 4950
Wire Wire Line
	3800 3850 3800 4100
Wire Wire Line
	3800 4100 5750 4100
Wire Wire Line
	4900 2800 4950 2800
Wire Wire Line
	4900 2800 4900 3200
Wire Wire Line
	4850 2800 4900 2800
Connection ~ 4900 2800
$Comp
L Device:R R?
U 1 1 60154E51
P 5700 1900
F 0 "R?" V 5907 1900 50  0000 C CNN
F 1 "R" V 5816 1900 50  0000 C CNN
F 2 "" V 5630 1900 50  0001 C CNN
F 3 "~" H 5700 1900 50  0001 C CNN
	1    5700 1900
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R?
U 1 1 60155280
P 5700 2000
F 0 "R?" V 5907 2000 50  0000 C CNN
F 1 "R" V 5816 2000 50  0000 C CNN
F 2 "" V 5630 2000 50  0001 C CNN
F 3 "~" H 5700 2000 50  0001 C CNN
	1    5700 2000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5350 1800 5550 1800
Wire Wire Line
	5350 1900 5550 1900
Wire Wire Line
	5350 2000 5550 2000
Wire Wire Line
	6100 1800 6100 1150
Wire Wire Line
	5850 1800 6100 1800
Wire Wire Line
	6200 1150 6200 1900
Wire Wire Line
	6200 1900 5850 1900
Wire Wire Line
	5850 2000 6300 2000
Wire Wire Line
	6300 2000 6300 1150
Text Notes 5800 1750 0    50   ~ 0
100k
Text Notes 5600 3500 0    50   ~ 0
A3-A8 / D6-D10 = Block 4-6\n(needs ULN2803 for 8 Blocks)
$EndSCHEMATC
