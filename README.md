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

## Links

* Datasheet http://everset-tech.com/wp-content/uploads/2014/11/ES100DataSheetver0p97.pdf
* North America distributor https://www.universal-solder.ca/product/everset-es100-cob-wwvb-60khz-bpsk-receiver-kit-with-2-antennas/
* On Amazon https://www.amazon.com/gp/product/B07PDW1QPX/ref=ppx_yo_dt_b_asin_title_o09_s00?ie=UTF8&psc=1

## Hardware Description and Setup

The hardware kit comes with a small PCB with the software-defined radio and two small ferrite antennas. Everset claims significantly improved RX ability over legacy antennas. It is certainly very small and compact.
* White Paper on the WWVB receiver: https://s2.smu.edu/~yliang/publications/A%20Multi-Mode%20Software-Defined%20CMOS%20BPSK%20Receiver%20SoC%20for%20WWVB.PDF
* Everset Antenna Considerations: http://everset-tech.com/wp-content/uploads/2014/11/AN-005_Everset_Antenna_Considerations_rev_1p1.pdf
In my current test installation I placed the two antennas at 45 degrees of each other, with the median orientation being loosely pointed to Fort Collins, Colorado. The receiver is located neart Seattle, Washington, at a Great Circle distance of 1577 Km, (980 Mi, 851 Nmi).
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
* The wire colors are of course completely arbitrary. I didn't want to break apart the ribbon wire, so I ended up with GND being red. Oh well.
* Before starting the test program, make sure that the I2C bus is enabled and running. I have set a 9600 baud rate for the I2C clock, although quite likely higher baud rates can be used.
