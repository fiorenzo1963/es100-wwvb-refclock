# es100-wwvb-refclock
Python test code and Shared memory NTP reference clock for Everset es100 WWVB receiver

## Description
This repository contains software to interface with Everset es100 WWVB receiver. This receiver is a dual-antenna software-defined radioclock which gets the timestamp code transmitted by NIST's WWVB 60 kHz radio station in Fort Collins, Colorado.

The test code is written in Python for ease of use, developed and tested on Raspberry PI 3.

The code which updates NTP SHM refclock is external. This is ugly and should be cleaned up.

The test code runs forever and keeps receiving data from WWVB. It starts with user supplied antenna (1, 2) then it keeps using the same antenna for as long as RX is successful. Upon RX timeout or RX error it switches to the other antenna. The receive timestamp is a PPS timestamp when GPIO_IRQ goes low, thus its accuracy does not depend on the I2C bus's baud rate.

![alt text](https://raw.githubusercontent.com/fiorenzo1963/es100-wwvb-refclock/master/images/es100_with_dual_antennas.jpg)

## Using NTP SHM reflock

* Compile included update_shm_one_shot tool
* Add this configuration to your /etc/ntp.conf
```
#
# WWVB SHM unit 13 refclock  - do you feel lucky, punk?
#
server 127.127.28.13 mode 0 prefer
fudge 127.127.28.13 refid WWVB
```
* Restart ntp
* Run test code in the background. Make sure you start the test code in the same directory where the tool resides (ugly, will fix)
* After a few successful RX updates, the refclock should show as reachable. Example:
```
     remote           refid      st t when poll reach   delay   offset  jitter
==============================================================================
*SHM(13)         .WWVB.           0 l  329   64  240    0.000   -4.031  25.183
+ticktock.pengui .GPS.            1 u   39   64  377    0.361  -18.748   2.061
+pendulum.pengui .GPS.            1 u   25   64  377    0.411  -18.490   2.184
+clepsydra.pengu .GSYM.           1 u   10   64  377    0.374  -16.582   3.385
```

## TODO

* See code comments for full FIXME list.
* Allow receiver to continue retry operation after being notified that current RX was unsucceful.
* Add tracking mode functionality for improved timestamp reception. Tracking mode only uses the top-of-the-minute mark as a PPS indicator, and works so long as the local clock doesn't have excess drift. Adding tracking mode will most likely render the above feature somewhat moot.
* Calling external code to update NTP SHM segment is ugly. It should at least be done with a C library called directly from python.
* Using /sys/devices to read PPS timestamps is non-portable and needs to be fixed. Python ctypes is the magic word.
* Cleanup code, split logic into WWVB ES100 library, test code and NTP SHM refclock driver. Python ctypes is the magic word.

## CHANGELOG

* See CHANGELOG file

## Acknowledgments

* Many thanks to members of time-nuts mailing list. Particular thanks to Hal Murray.
* Many thanks to my wife and daugthers, who somehow think my timekeeping hobby is weird, but still put up with me.

## Links

* EVERSET
	* Datasheet http://everset-tech.com/wp-content/uploads/2014/11/ES100DataSheetver0p97.pdf
	* Related Arduino project which contains information not specified in the datasheet https://sites.google.com/site/wwvbreceiverwitharduino/
	* North America Distributor https://www.universal-solder.ca/product/everset-es100-cob-wwvb-60khz-bpsk-receiver-kit-with-2-antennas/
	* Amazon https://www.amazon.com/gp/product/B07PDW1QPX/ref=ppx_yo_dt_b_asin_title_o09_s00?ie=UTF8&psc=1
	* https://time-nuts.febo.narkive.com/gETMuwcR/new-wwvb-bpsk-dev-board
	* https://www.kb6nu.com/building-a-wwvb-receiver-episode-2/
	* https://www.tinaja.com/third/wwvbuser01.pdf
* WWVB general information
	* WWVB Wikipedia https://en.wikipedia.org/wiki/WWVB
	* NIST's WWVB Page https://www.nist.gov/pml/time-and-frequency-division/radio-stations/wwvb
	* https://tf.nist.gov/general/pdf/2429.pdf
	* https://www.febo.com/time-freq/wwvb/index.html
	* https://tf.nist.gov/general/pdf/1969.pdf
	* https://www.nist.gov/system/files/documents/2017/05/09/NIST-Enhanced-WWVB-Broadcast-Format-1_01-2013-11-06.pdf
* Links of interest and related projects
	* https://sites.google.com/site/wwvbreceiverwitharduino/
	* https://ieeexplore.ieee.org/document/1701081
	* https://www.ion.org/publications/abstract.cfm?articleID=15622
	* http://www.leapsecond.com/pages/sony-wwvb/
	* https://www.raspberrypi.org/forums/viewtopic.php?t=20968
	* https://www.satsignal.eu/ntp/Raspberry-Pi-quickstart.html
	* http://www.buzzard.me.uk/jonathan/radioclock.html
	* http://www.ko4bb.com/Timing/
	* http://leapsecond.com/time-nuts.htm
	* http://www.leapsecond.com/hsn2006/ch2.pdf
	* https://github.com/hans-mayer/ntpgraph
	* http://support.ntp.org/bin/view/Support/MonitoringAndControllingNTP#scripts_stats_tools
	* http://www.ntp.org/ntpfaq/NTP-s-trouble.htm

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

## System Configuration
* Enable I2C
* Enable SMBUS
* Enable PPS (PPS should already configured by default, so all you should have to do is make sure that gpio-pps module is loaded at boot time)
* Setup PPS in /boot/config.txt - note that the pin numbering in /boot/config.txt follows the BCM numbering
```
dtoverlay=pps-gpio,gpiopin=17,capture_clear
```

## ES100 kinks

* None seen so far. ES100 appears to behave per Everset datasheet specs.

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
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574088910.037290096,58805,53710.037290096,0,2019-11-18T14-55-10Z,1574088910.005239725,-0.032050371
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574089046.004205942,58805,53846.004205942,0,2019-11-18T14-57-26Z,1574089046.005239725,0.001033783
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574089182.027056932,58805,53982.027056932,0,2019-11-18T14-59-42Z,1574089182.005239725,-0.021817207
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574089317.994098663,58805,54117.994098663,0,2019-11-18T15-01-58Z,1574089318.005239725,0.011141062
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574089453.971482515,58805,54253.971482515,0,2019-11-18T15-04-14Z,1574089454.005239725,0.033757210
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574089590.009804487,58805,54390.009804487,0,2019-11-18T15-06-30Z,1574089590.005239725,-0.004564762
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574089726.036934853,58805,54526.036934853,0,2019-11-18T15-08-46Z,1574089726.005239725,-0.031695127
RX_WWVB_CLOCKSTATS,v2,4,IRQ_CYCLE_COMPL,1,1574089868.322228432,58805,54668.322228432,0,,,
RX_WWVB_CLOCKSTATS,v2,4,IRQ_CYCLE_COMPL,2,1574090010.431529522,58805,54810.431529522,0,,,
RX_WWVB_CLOCKSTATS,v2,4,IRQ_CYCLE_COMPL,1,1574090152.551741838,58805,54952.551741838,0,,,
RX_WWVB_CLOCKSTATS,v2,4,IRQ_CYCLE_COMPL,2,1574090294.674129963,58805,55094.674129963,0,,,
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574090430.977062702,58805,55230.977062702,0,2019-11-18T15-20-31Z,1574090431.005239725,0.028177023
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574090567.001712799,58805,55367.001712799,0,2019-11-18T15-22-47Z,1574090567.005239725,0.003526926
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574090703.022387028,58805,55503.022387028,0,2019-11-18T15-25-03Z,1574090703.005239725,-0.017147303
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574090838.985434532,58805,55638.985434532,0,2019-11-18T15-27-19Z,1574090839.005239725,0.019805193
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574090975.007650614,58805,55775.007650614,0,2019-11-18T15-29-35Z,1574090975.005239725,-0.002410889
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574091110.978703022,58805,55910.978703022,0,2019-11-18T15-31-51Z,1574091111.005239725,0.026536703
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574091247.013198376,58805,56047.013198376,0,2019-11-18T15-34-07Z,1574091247.005239725,-0.007958651
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574091383.027441025,58805,56183.027441025,0,2019-11-18T15-36-23Z,1574091383.005239725,-0.022201300
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574091519.040785789,58805,56319.040785789,0,2019-11-18T15-38-39Z,1574091519.005239725,-0.035546064
RX_WWVB_CLOCKSTATS,v2,4,IRQ_CYCLE_COMPL,1,1574091661.316339493,58805,56461.316339493,0,,,
RX_WWVB_CLOCKSTATS,v2,4,IRQ_CYCLE_COMPL,2,1574091803.429315090,58805,56603.429315090,0,,,
RX_WWVB_CLOCKSTATS,v2,4,IRQ_CYCLE_COMPL,1,1574091945.548914671,58805,56745.548914671,0,,,
RX_WWVB_CLOCKSTATS,v2,4,IRQ_CYCLE_COMPL,2,1574092087.664652348,58805,56887.664652348,0,,,
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574092223.974880695,58805,57023.974880695,0,2019-11-18T15-50-24Z,1574092224.005239725,0.030359030
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574092360.003244638,58805,57160.003244638,0,2019-11-18T15-52-40Z,1574092360.005239725,0.001995087
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574092495.982347250,58805,57295.982347250,0,2019-11-18T15-54-56Z,1574092496.005239725,0.022892475
RX_WWVB_CLOCKSTATS,v2,1,RX_OK_ANT1,1,1574092632.011371613,58805,57432.011371613,0,2019-11-18T15-57-12Z,1574092632.005239725,-0.006131887
```
* Using gnuplot to plot RX_WWVB_CLOCKSTATS. Gnuplot can easily handle commas as separators instead of white spaces, note however a little data massaging for error entries is needed if null values are not accepted by gnuplot or intermediate scripts.
```
set datafile separator comma
```

## Related WWVB Hardware

CANADUINO makes another WWVB receiver, based on the MAS6180C chip. This product is single antenna does not handle phase modulation, but provides for access to the digitalized raw bit stream. Something quite interesting. I have not played with this unit yet. Presumably the ability of reading the timing pulses of the AM modulation should allow software to both decode all the 60 data bits as well as get timing information from all timemark frames FRM, P1-P5 and P0.
* https://www.universal-solder.ca/product/canaduino-60khz-atomic-clock-receiver-module-wwvb-msf-jjy60/
* http://canaduino.ca/downloads/CANADUINO_Atomic_Clock_Receiver_Kit_SMD.pdf
* https://github.com/ahooper/WWVBClock
* https://sites.google.com/site/wwvbreceiverwitharduino/
