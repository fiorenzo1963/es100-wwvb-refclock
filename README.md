# es100-wwvb-refclock
Shared memory NTP reference clock for Everset es100 WWVB receiver

## Description
This repository contains software to interface with Everset es100 WWVB receiver. This receiver is dual-antenna software-defined radioclock which gets the timestamp code transmitted by NIST's WWVB radio station in Fort Collins, Colorado.
The test code is written in Python for ease of use, developed and tested on Raspberry PI 3.

Currently only the test code is implemented. Once it becomes feature complete I'll write the NTP SHM refclock driver based on the test code. Using python for test code allows for quick prototyping and can be easily copied and reused by anyne who is not familiar with C.

The test code runs forever and keeps receiving data from WWVB. It starts with user supplied antenna configuration (1, 2, 2-1, 1-2) then it keeps using the same antenna for as long as RX is successful. Upon RX timeout or RX error it switches to the other antenna.

Two major functionalities need to be added to the best code before I start writing the NTP reference clock:
* Allow receiver to continue retry operation after being notified that current RX was unsucceful.
* Add tracking mode functionality for improved timestamp reception. Tracking mode only uses the top-of-the-minute mark as a PPS indicator, and works so long as the local clock doesn't have excess drift. Adding tracking mode will most likely render the first features somewhat moot.

![alt text](https://raw.githubusercontent.com/fiorenzo1963/es100-wwvb-refclock/master/images/es100_with_dual_antennas.jpg)


## Links

* Datasheet http://everset-tech.com/wp-content/uploads/2014/11/ES100DataSheetver0p97.pdf
* North America distributor https://www.universal-solder.ca/product/everset-es100-cob-wwvb-60khz-bpsk-receiver-kit-with-2-antennas/
* On Amazon https://www.amazon.com/gp/product/B07PDW1QPX/ref=ppx_yo_dt_b_asin_title_o09_s00?ie=UTF8&psc=1

## Hardware Description and Setup

The hardware kit comes with a small PCB with the software-defined radio and two small ferrite antennas. Everset claims significantly improved RX ability over legacy antennas. It is certainly very small and compact.
* White Paper on the WWVB receiver: https://s2.smu.edu/~yliang/publications/A%20Multi-Mode%20Software-Defined%20CMOS%20BPSK%20Receiver%20SoC%20for%20WWVB.PDF
* Any suitable I2C bus and 3.3v GPIO pins can be used of course. In my case the setup is as follows:
```
PI3 PHYS PIN       PI3 PIN FUNCTION                 WIRE COLOR
============       ================                 ==========
1                  VCC (3.3V)                       BLUE
3                  SDA (I2C DATA)                   GREEN  
5                  SCL (I2C CLOCK)                  YELLOW
7                  GPIO.7 (3.3V DEV_ENABLE)         ORANGE
9                  GND                              RED
11                 GPIO.9 (3.3V DEV_IRQ)            BROWN
```
* The corresponding pinout on the ES100 WWVB hardware is as follows:
```
ES100 WWVB PIN          WIRE COLOR
==============          ==========
1 = GND                 RED
2 = VDD                 BLUE
3 = IRQ                 BROWN
4 = SCK                 YELLOW
5 = SDA                 GREEN
6 = EN                  ORANGE
```
![alt text](https://raw.githubusercontent.com/fiorenzo1963/es100-wwvb-refclock/master/images/es100_connection_to_pi3.jpg)
* The wire colors are of course completely arbitrary. I didn't want to break apart the ribbon wire, so I ended up with GND being red. Oh well.
* Before starting the test program, make sure that the I2C bus is enabled and running. I have set a 9600 baud rate for the I2C clock, although quite likely higher baud rates can be used.
* Everset Antenna Considerations: http://everset-tech.com/wp-content/uploads/2014/11/AN-005_Everset_Antenna_Considerations_rev_1p1.pdf
In my current test installation I placed the two antennas at 45 degrees of each other, with the median orientation being loosely pointed to Fort Collins, Colorado. The receiver is located neart Seattle, Washington, at a Great Circle distance of 1577 Km, (980 Mi, 851 Nmi).
![alt text](https://raw.githubusercontent.com/fiorenzo1963/es100-wwvb-refclock/master/images/es100_with_antennas_and_pi3.jpg)

## First results

* These logs have been taken from 2019-11-15T11:09:18Z to 2019-11-15T22:35:49Z
* RX statistics indicate a successful RX rate of 44%
```
        RX_OK_ANT1          1      /* RX count on ANT_1 */
        RX_OK_ANT2        113      /* RX count on ANT_2 */
        IRQ_NO_DATA       145      /* RX unsuccessful, WWVB retrying (currently simply timed-out) */
```
* Phase (aka offset) error statistics compared against a local GPS-disciplined stratum-1 clock
	* Average ~ -10 milliseconds
	* Most samples are within +40/-60 milliseconds
	* All samples are within +60/-120 milliseconds
* Phase (aka offset) error plot, with moving average (note that RX errors are ignored here)
![alt text](https://raw.githubusercontent.com/fiorenzo1963/es100-wwvb-refclock/master/images/wwvb_offset_error_20191115.jpg)
* Raw data
```
35.8819961548 msecs
-2.85601615906 msecs
-13.0240917206 msecs
21.1451053619 msecs
-9.01007652283 msecs
-17.767906189 msecs
-0.637054443359 msecs
-22.0699310303 msecs
-42.7401065826 msecs
11.6429328918 msecs
-9.01198387146 msecs
19.082069397 msecs
-53.3170700073 msecs
26.1480808258 msecs
4.98604774475 msecs
-16.5359973907 msecs
9.21010971069 msecs
-14.3430233002 msecs
-14.9350166321 msecs
10.0729465485 msecs
-10.1869106293 msecs
-30.6680202484 msecs
-2.18296051025 msecs
21.2829113007 msecs
-49.6110916138 msecs
-21.0900306702 msecs
33.3271026611 msecs
-41.2769317627 msecs
32.5000286102 msecs
11.5818977356 msecs
-11.0800266266 msecs
17.6119804382 msecs
-5.25188446045 msecs
-26.1199474335 msecs
-26.0870456696 msecs
3.1430721283 msecs
-17.5850391388 msecs
-37.7249717712 msecs
-9.14692878723 msecs
18.7959671021 msecs
-51.5909194946 msecs
-22.411108017 msecs
-43.2329177856 msecs
5.31792640686 msecs
-15.6650543213 msecs
-12.8180980682 msecs
16.037940979 msecs
-32.1359634399 msecs
-5.85794448853 msecs
22.2079753876 msecs
-48.7809181213 msecs
30.0300121307 msecs
8.15391540527 msecs
-13.8370990753 msecs
15.1979923248 msecs
12.3600959778 msecs
-8.80694389343 msecs
-31.0459136963 msecs
-2.90298461914 msecs
-27.5859832764 msecs
-2.5520324707 msecs
26.9839763641 msecs
-46.2210178375 msecs
-36.6849899292 msecs
41.6009426117 msecs
19.0300941467 msecs
-3.22198867798 msecs
-23.6840248108 msecs
-0.0319480895996 msecs
-24.1389274597 msecs
-43.5659885406 msecs
-16.0810947418 msecs
-16.0429477692 msecs
14.4009590149 msecs
-7.04407691956 msecs
-78.2179832458 msecs
-0.511884689331 msecs
28.9149284363 msecs
-42.2170162201 msecs
-13.5610103607 msecs
-8.29696655273 msecs
-34.9659919739 msecs
-6.4070224762 msecs
-27.8630256653 msecs
-0.458955764771 msecs
-22.3500728607 msecs
6.61110877991 msecs
-114.166021347 msecs
-20.1070308685 msecs
57.629108429 msecs
45.6500053406 msecs
31.9490432739 msecs
53.3010959625 msecs
-46.6289520264 msecs
-9.99689102173 msecs
-3.29303741455 msecs
-8.31890106201 msecs
19.0908908844 msecs
-53.7889003754 msecs
-25.8769989014 msecs
-97.0349311829 msecs
7.91907310486 msecs
-44.4889068604 msecs
-7.21788406372 msecs
-11.9609832764 msecs
-33.595085144 msecs
-4.01997566223 msecs
-86.0741138458 msecs
-7.73811340332 msecs
17.6229476929 msecs
-5.126953125 msecs
-27.853012085 msecs
0.614881515503 msecs
-24.9390602112 msecs
```
