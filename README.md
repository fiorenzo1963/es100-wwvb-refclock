# es100-wwvb-refclock
Python test code and Shared memory NTP reference clock for Everset es100 WWVB receiver

## Description
This repository contains software to interface with Everset es100 WWVB receiver. This receiver is a dual-antenna software-defined radioclock which gets the timestamp code transmitted by NIST's WWVB 60 kHz radio station in Fort Collins, Colorado.
The test code is written in Python for ease of use, developed and tested on Raspberry PI 3.

Currently only the test code is implemented. Once it becomes feature complete I'll write the NTP SHM refclock driver based on the test code. Using python for test code allows for quick prototyping and can be easily copied and reused for any purposes by anyne who is not familiar with C.

The test code runs forever and keeps receiving data from WWVB. It starts with user supplied antenna configuration (1, 2, 2-1, 1-2) then it keeps using the same antenna for as long as RX is successful. Upon RX timeout or RX error it switches to the other antenna. The receive timestamp is taken when GPIO_IRQ goes low, thus its accuracy does not depend on the I2C bus's baud rate.

Two major functionalities need to be added to the best code before I start writing the NTP reference clock:
* Allow receiver to continue retry operation after being notified that current RX was unsucceful.
* Add tracking mode functionality for improved timestamp reception. Tracking mode only uses the top-of-the-minute mark as a PPS indicator, and works so long as the local clock doesn't have excess drift. Adding tracking mode will most likely render the above feature somewhat moot.

One major limitation of the test code is that it is quite dumb in that it simply polls the WWVB receiver every 2 milliseconds. The NTP reference clock implementation will make use of PPS timestamping for accuracy and efficiency. However actual improvements are likely to be fairly low due to the jitter of the signal itself, which is fairly high.

![alt text](https://raw.githubusercontent.com/fiorenzo1963/es100-wwvb-refclock/master/images/es100_with_dual_antennas.jpg)

## TODO

* Need to support continous RX mode.
* The receiver can trigger an IRQ which simply indicates that RX was unsuccessful, and retry is pending. The current code simply treats this as a timeout and restarts reception.
* Tracking mode (essentially equivalent to a "PPS" mode) needs to be supported, see datasheet for details.
* Actually implement shared memory NTP refclock, so the implementation will match the project Description.

## CHANGELOG

* See CHANGELOG file

## Acknowledgments

* Many thanks to members of time-nuts mailing list.
	* Particular thanks to Hal Murray, who is giving me many precious hints and suggestions.
* Many thanks to my wife and daugthers, who somehow think my timekeeping hobby is weird, but still put up with me.

## Links

* EVERSET
	* Datasheet http://everset-tech.com/wp-content/uploads/2014/11/ES100DataSheetver0p97.pdf
	* North America Distributor https://www.universal-solder.ca/product/everset-es100-cob-wwvb-60khz-bpsk-receiver-kit-with-2-antennas/
	* Amazon https://www.amazon.com/gp/product/B07PDW1QPX/ref=ppx_yo_dt_b_asin_title_o09_s00?ie=UTF8&psc=1
	* https://time-nuts.febo.narkive.com/gETMuwcR/new-wwvb-bpsk-dev-board
	* https://www.kb6nu.com/building-a-wwvb-receiver-episode-2/
* WWVB general information
	* WWVB Wikipedia https://en.wikipedia.org/wiki/WWVB
	* NIST's WWVB Page https://www.nist.gov/pml/time-and-frequency-division/radio-stations/wwvb
	* https://www.febo.com/time-freq/wwvb/index.html
* Links of interest and related projects
	* https://www.ion.org/publications/abstract.cfm?articleID=15622
	* http://www.leapsecond.com/pages/sony-wwvb/
	* https://www.raspberrypi.org/forums/viewtopic.php?t=20968
	* https://www.satsignal.eu/ntp/Raspberry-Pi-quickstart.html
	* http://www.buzzard.me.uk/jonathan/radioclock.html
	* http://www.ko4bb.com/Timing/
	* http://leapsecond.com/time-nuts.htm
	* http://www.leapsecond.com/hsn2006/ch2.pdf

## Hardware Description and Setup

The hardware kit comes with a small PCB with the software-defined radio and two small ferrite antennas. Everset claims significantly improved RX ability over a single antenna system -- there is quite a bit of debate about this on time-nuts mailing list. Future statistics should be able to tell if this claim is true.
* White Paper on the WWVB receiver: https://s2.smu.edu/~yliang/publications/A%20Multi-Mode%20Software-Defined%20CMOS%20BPSK%20Receiver%20SoC%20for%20WWVB.PDF
* Any suitable I2C bus and 3.3v GPIO pins can be used of course. In my case the setup for Raspberry PI3 is as follows:
```
PI3 PHYS PIN    BCM PIN     PI3 PIN FUNCTION                 WIRE COLOR
============    =======     ================                 ==========
1                           VCC (3.3V)                       BLUE
3               2           SDA (I2C DATA)                   GREEN
5               3           SCL (I2C CLOCK)                  YELLOW
7               4           GPIO.7 (3.3V DEV_ENABLE)         ORANGE
9                           GND                              RED
11              17          GPIO.0 (3.3V DEV_IRQ)            BROWN
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
* Phase (aka offset) error plot, with moving average (note that RX errors are ignored here). The X axis is dimensionless and corresponds to one received WWVB timestamp. The next plot will show RX errors and actual timestamp.
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

## System Configuration
* Enable I2C
* Enable SMBUS
* Enable PPS - PPS is configured by default, so all you should have to do is make sure that gpio-pps module is loaded at boot time
* Setup PPS in /boot/config.txt - note that the pin numbering in /boot/config.txt follows the BCM numbering
```
dtoverlay=pps-gpio,gpiopin=17
dtoverlay=pps-gpio,assert_falling_edge
```

## Metrics

The sample code now also emits statistical information in a machine parseable format, useful to make Allan Plots and several other kinds of statistical inference. Two types of metrics are emitted:
* RX clockstats information **RX_WWVB_CLOCKSTATS**
```
pi@wwvb-raspberrypi:~/GITHUB/es100-wwvb-refclock $ grep RX_WWVB_CLOCKSTATS es100-wwvb-test.log 
```
* RX stats counter information **RX_WWVB_STAT_COUNTERS**
```
pi@wwvb-raspberrypi:~/GITHUB/es100-wwvb-refclock $ grep RX_WWVB_STAT_COUNTERS es100-wwvb-test.log 
```
Sample results for **RX_WWVB_CLOCKSTATS**:
```
RX_WWVB_CLOCKSTATS,v1,1,RX_OK_ANT1,1,1573874523.02,0,2019-11-16T03-22-03Z,1573874523.0,-19.9429988861
RX_WWVB_CLOCKSTATS,v1,1,RX_OK_ANT1,1,1573874677.99,0,2019-11-16T03-24-38Z,1573874678.0,11.0199451447
RX_WWVB_CLOCKSTATS,v1,1,RX_OK_ANT1,1,1573874833.06,0,2019-11-16T03-27-13Z,1573874833.0,-63.1880760193
RX_WWVB_CLOCKSTATS,v1,1,RX_OK_ANT1,1,1573874988.98,0,2019-11-16T03-29-49Z,1573874989.0,17.4679756165
RX_WWVB_CLOCKSTATS,v1,1,RX_OK_ANT1,1,1573875144.0,0,2019-11-16T03-32-24Z,1573875144.0,-1.39904022217
RX_WWVB_CLOCKSTATS,v1,1,RX_OK_ANT1,1,1573875299.02,0,2019-11-16T03-34-59Z,1573875299.0,-21.5220451355
RX_WWVB_CLOCKSTATS,v1,1,RX_OK_ANT1,1,1573875454.04,0,2019-11-16T03-37-34Z,1573875454.0,-42.2201156616
RX_WWVB_CLOCKSTATS,v1,1,RX_OK_ANT1,1,1573875609.01,0,2019-11-16T03-40-09Z,1573875609.0,-12.2420787811
RX_WWVB_CLOCKSTATS,v1,4,IRQ_NO_DATA,0,1573875770.35,0,,,
RX_WWVB_CLOCKSTATS,v1,4,IRQ_NO_DATA,0,1573875938.09,0,,,
RX_WWVB_CLOCKSTATS,v1,2,RX_OK_ANT2,2,1573876100.02,0,2019-11-16T03-48-20Z,1573876100.0,-18.424987793
```

## Related WWVB Hardware

CANADUINO makes another WWVB receiver, based on the MAS6180C chip. This product is single antenna does not handle phase modulation, but provides for access to the digitalized raw bit stream. Something quite interesting. I have not played with this unit yet. Presumably the ability of reading the timing pulses of the AM modulation should allow software to both decode all the 60 data bits as well as get timing information from all timemark frames FRM, P1-P5 and P0.
* https://www.universal-solder.ca/product/canaduino-60khz-atomic-clock-receiver-module-wwvb-msf-jjy60/
* http://canaduino.ca/downloads/CANADUINO_Atomic_Clock_Receiver_Kit_SMD.pdf
* https://github.com/ahooper/WWVBClock
