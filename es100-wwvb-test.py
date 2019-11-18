#!/usr/bin/python -u

#
# Simple python code to test ES100 WWVB receiver.
# Ths code tries to update NTP SHM segment, if any.
#
# This code has been developed and tested using Raspberry PI 3. Your mileage may vary.
#
# Copyright (C) 2019 Fio Cattaneo <fio@cattaneo.us>, All Rights Reserved.
#
# This code is released under dual GPL Version 2 / FreeBSD license, at your option.
#
# This test code is fairly dumb and uses busy-waiting (with 2 millisecond sleep intervals).
# The NTP refclock will be written in C and will use PPS timestamping for accuracy and efficiency.
# However any improvement is not likely to be very high, given the jitter of WWVB reception.
#
# TODO: major issues to be fixed:
#
# (A) Handle IO errors when reading/writing I2C device
# (B) Need to support continous RX mode.
# (C) The receiver can trigger an IRQ which simply indicates that RX was unsuccessful,
#     and retry is pending. The current code simply treats this as a timeout and restarts reception.
# (D) Tracking mode (essentially equivalent to a "PPS" mode) needs to be supported,
#     see datasheet for details.
# (E) Figure out best antenna strategy.
# (F) Forking external code to update NTP SHM segment is ugly.
#
# TODO: minor issues to be fixed:
#
# (C) general code cleanup
# (D) convert to a class object
#

import RPi.GPIO as GPIO
import smbus
import time
import sys
import os

#
# The next five constants depend on which Raspberry PI's IC2BUS and GPIO pins the device is actually wired to.
# FIXME: I2C CHANNEL 1 is pretty much hardcoded with current implementation.
# probably not worth trying to do better.
#
I2C_DEV_CHANNEL             = 1         # use I2C channel 1
GPIO_DEV_I2C_SDA_PIN        = 3         # I2C SDA pin
GPIO_DEV_I2C_SCL_PIN        = 5         # I2C SCL pin
GPIO_DEV_ENABLE             = 7         # device ENABLE pin connected to physical pin 7
GPIO_DEV_IRQ                = 11        # device IRQ connected to physical pin 9
KILOMETERS_FROM_FTCOLLINS_CO= 1568      # great circle distance from Fort Collins, Colorado, kilometers

#
# from https://ieeexplore.ieee.org/document/1701081
#
SPEED_OF_LIGHT_OVERLAND     = 299250

#
# Constants for EVERSET WWVB receiver copied from es100_host_arduino.c,
# Xtendwave ES100 Example Host MCU Code for Arduino - version 0002.
# See included original ES100_Arduino.ino file for more details.
#
# The algorithm was loosely inspired to the code in ES100_Arduino.ino code, and
# modified to comply with the ES100 specs EVERSET's datasheet version 0.97
#

ES100_SLAVE_ADDR            = 0x32

ES100_CONTROL0_REG          = 0x00
ES100_CONTROL1_REG          = 0x01
ES100_IRQ_STATUS_REG        = 0x02
ES100_STATUS0_REG           = 0x03
ES100_YEAR_REG              = 0x04
ES100_MONTH_REG             = 0x05
ES100_DAY_REG               = 0x06
ES100_HOUR_REG              = 0x07
ES100_MINUTE_REG            = 0x08
ES100_SECOND_REG            = 0x09
ES100_NEXT_DST_MONTH_REG    = 0x0A
ES100_NEXT_DST_DAY_REG      = 0x0B
ES100_NEXT_DST_HOUR_REG     = 0x0C
ES100_DEVICE_ID_REG         = 0x0D

#
# value you get when reading from slave without specifying a register
#
ES100_SLAVE_ADDR_VAL        = 0x00
#
# ES100 device ID
#
ES100_DEVICE_ID             = 0x10

#
#
ES100_CONTROL_START_RX_ANT1                 = 0x05
ES100_CONTROL_START_RX_ANT2                 = 0x03
#
# XXX: NOTE: there seems to be no advantage in asking for ANT1_ANT2 over ANT1
# or asking for ANT2_ANT1 over ANT2
#
ES100_CONTROL_START_RX_ANT1_ANT2            = 0x01
ES100_CONTROL_START_RX_ANT2_ANT1            = 0x09
#
# FIXME: add support for tracking
#
# ES100_CONTROL_START_RX_ANT2_TRACKING        = 0x13
# ES100_CONTROL_START_RX_ANT1_TRACKING        = 0x15
#

def make_timespec_s(timestamp):
        return "{0:09.09f}".format(timestamp)

def decode_bcd_byte(raw_bcd, offset = 0):
        val = raw_bcd & 0xf
        val = val + ((raw_bcd >> 4) & 0xf) * 10
        return val + offset

def str2(val):
        return "{0:02d}".format(val)

#
# FIXME: need to handle IO errors
#
def write_wwvb_device(bus, reg, value):
        bus.write_byte_data(ES100_SLAVE_ADDR, reg, value)
        time.sleep(0.005)

#
# FIXME: need to handle IO errors
#
def read_wwvb_device(bus, reg):
        bus.write_byte(ES100_SLAVE_ADDR, reg)
        time.sleep(0.005)
        val = bus.read_byte(ES100_SLAVE_ADDR)
        time.sleep(0.005)
        return val

#
# enable/disable state changes take a significant amount of time.
# everset data sheet doesn't say how long these transitions should last,
# but it does seem excessive for this to even taken more than 100 milliseconds
#
def gpio_wait_state_change(pin, pin_name, curr_state, curr_state_s, new_state, new_state_s):
        c = 0
        print "gpio_wait_state_change: wait for " + pin_name + " state change to " + new_state_s
        # FIXME: need to timeout this loop
        while GPIO.input(pin) == curr_state:
                time.sleep(0.005)
                c = c + 0.005
        if c >= 0.050:
                print "gpio_wait_state_change: WARNING: state change for " + pin_name + " took " + str(c) + " secs"
        time.sleep(0.050)

#
# FIXME: need to do substantial cleanup of this code
#
def enable_wwvb_device():
        if GPIO.input(GPIO_DEV_ENABLE) != 0:
                # FIXME: this is an error and shouldn't happen
                print "enable_wwvb_device: ERROR: WWVB device already enabled"
        else:
                print "enable_wwvb_device: enabling WWVB device"
                if GPIO.input(GPIO_DEV_IRQ) != 0:
                        #
                        # README FIXME: need to handle this error -- it's not clear at why this happens.
                        # when the device is disabled with GPIO_DEV_ENABLED set to 0, the hardware is
                        # not supposed to set IRQ high. the disable path always waits for IRQ to go low
                        # before completing.
                        # will probably need to debug this with the oscillscope.
                        #
                        print "enable_wwvb_device: ERROR: GPIO_IRQ pin is high, but should be low"
                        print "enable_wwvb_device: ERROR: GPIO_IRQ pin is high, but should be low"
                        print "enable_wwvb_device: ERROR: GPIO_IRQ pin is high, but should be low"
                        time.sleep(1.000)
                GPIO.output(GPIO_DEV_ENABLE, GPIO.HIGH)
        gpio_wait_state_change(GPIO_DEV_IRQ, "DEV_IRQ", 0, "low", 1, "high")

#
# see README comment in enable_wwvb_device -- need to handle QPIO_IRQ possibly bouncing a bit
#
def disable_wwvb_device(deep_disable = False):
        if GPIO.input(GPIO_DEV_ENABLE) == 0:
                print "disable_wwvb_device: NOTE: WWVB device already disabled"
        else:
                print "disable_wwvb_device: disabling WWVB device"
                GPIO.output(GPIO_DEV_ENABLE, GPIO.LOW)
        if deep_disable is True:
                print "disable_wwvb_device: deep disable"
                time.sleep(10.000)
                print "disable_wwvb_device: deep disable done"
        gpio_wait_state_change(GPIO_DEV_IRQ, "DEV_IRQ", 1, "high", 0, "low")

def set_gpio_pins_wwvb_device():
        #
        # FIXME: check pin functions to make sure I2C/SMBUS is enabled
        #
        print "set_gpio_pins_wwvb_device: setting GPIO pins for WWVB receiver"
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(GPIO_DEV_ENABLE, GPIO.OUT)
        GPIO.output(GPIO_DEV_ENABLE, GPIO.LOW)
        GPIO.setup(GPIO_DEV_IRQ, GPIO.IN)
        func = GPIO.gpio_function(GPIO_DEV_I2C_SCL_PIN)
        print "set_gpio_pins_wwvb_device: func I2C_SCL_PIN = " + str(func) + "/" + str(GPIO.I2C)
        if func != GPIO.I2C:
                #
                # non recoverable
                #
                print "set_gpio_pins_wwvb_device: FATAL ERROR: function I2C_SCL_PIN is not GPIO.I2C"
                exit(1)
        func = GPIO.gpio_function(GPIO_DEV_I2C_SDA_PIN)
        print "set_gpio_pins_wwvb_device: func I2C_SDA_PIN = " + str(func) + "/" + str(GPIO.I2C)
        if func != GPIO.I2C:
                #
                # non recoverable
                #
                print "set_gpio_pins_wwvb_device: FATAL ERROR: function I2C_SDA_PIN is not GPIO.I2C"
                exit(1)
        #
        # make sure IRQ is low
        #
        time.sleep(0.500)
        gpio_wait_state_change(GPIO_DEV_IRQ, "DEV_IRQ", 1, "high", 0, "low")

#
# main entry point - init
#
def one_time_init_wwvb_device():
        print "one_time_init_wwvb_device:"
        GPIO.setwarnings(False)
        #
        # set gpio pins
        #
        set_gpio_pins_wwvb_device()
        #
        # make sure WWVB receiver is powered down
        #
        disable_wwvb_device(deep_disable = True)
        print "one_time_init_wwvb_device: done"

def init_wwvb_device():
        print "init_wwvb_device: initializing ES100 WWVB receiver"
        #
        # set ENABLE pin to LOW to power it down
        #
        disable_wwvb_device()
        #
        # set ENABLE pin to HIGH to power it up
        #
        enable_wwvb_device()
        #
        # open i2c bus, read from device to make sure it's there
        #
        print "init_wwvb_device: opening i2c bus channel = " + str(I2C_DEV_CHANNEL)
        bus = smbus.SMBus(I2C_DEV_CHANNEL)
        print "init_wwvb_device: i2c_bus_object = " + str(bus)
        time.sleep(0.100)
        #
        # not sure how to call an I2C read which does not specify a source address
        #
        es100_slave_addr_val = bus.read_byte(ES100_SLAVE_ADDR)
        print "init_wwvb_device: es100_slave_addr_val = " + str(es100_slave_addr_val)
        if es100_slave_addr_val != ES100_SLAVE_ADDR_VAL:
                print "init_wwvb_device: ERROR: invalid ES100 es100_slave_addr_val"
                return None
        #
        time.sleep(0.100)
        val = read_wwvb_device(bus, ES100_DEVICE_ID_REG)
        print "init_wwvb_device: es100_device_id = " + str(val)
        if val != ES100_DEVICE_ID:
                print "init_wwvb_device: ERROR: invalid ES100 device_id"
                return None
        #
        # don't check irq_status register, as it has side effects
        #
        val = read_wwvb_device(bus, ES100_CONTROL0_REG)
        print "init_wwvb_device: control0 reg = " + str(val)
        if val != 0:
                print "init_wwvb_device: ERROR: invalid control0 reg"
                return None
        val = read_wwvb_device(bus, ES100_STATUS0_REG)
        print "init_wwvb_device: status0 reg = " + str(val)
        if val != 0:
                print "init_wwvb_device: ERROR: invalid status0 reg"
                return None
        val = GPIO.input(GPIO_DEV_IRQ)
        print "init_wwvb_device: gpio_dev_irq pin = " + str(val)
        if val != 1:
                print "init_wwvb_device: ERROR: invalid gpio_dev_irq_pin"
                return None
        print "init_wwvb_device: done initializing ES100 WWVB receiver"
        return bus

#
# RX status codes
#
RX_STATUS_WWVB_RX_OK_ANT1     = 1 # RX OK, ANTENNA 1
RX_STATUS_WWVB_RX_OK_ANT2     = 2 # RX OK, ANTENNA 1
RX_STATUS_WWVB_TIMEOUT        = 3 # RX timed out
RX_STATUS_WWVB_IRQ_NO_DATA    = 4 # RX done, but data was not received, device retrying
                                  # FIXME: current code does not allow for device retry
RX_STATUS_WWVB_BAD_IRQ_STATUS = 5 # RX done, bad IRQ_STATUS register value
RX_STATUS_WWVB_BAD_STATUS0    = 6 # RX done, bad STATUS0 register value
RX_STATUS_WWVB_STATUS0_NO_RX  = 7 # RX done, STATUS0 indicates no data received
RX_STATUS_WWVB_DEV_INIT_FAILED= 8 # failed to initialize ES100 device (usually failed read from device_id register)
RX_STATUS_MAX_STATUS          = 8

RX_STATUS_WWVB_STR = (
                "",
                "RX_OK_ANT1",
                "RX_OK_ANT2",
                "TIMEOUT",
                "IRQ_NO_DATA",
                "BAD_IRQ_STATUS",
                "WWVB_BAD_STATUS0",
                "WWVB_STATUS0_NO_RX",
                "WWVB_DEV_INIT_FAILED"
)

#
# machine readable line for automated parsing and analysis
# no other text printed by this tool begins with RX_WWVB_STAT
#
# format:
#       rx_ret numerical form (one of the RX status codes above)
#       rx_ret string form (one of the RX status codes above)
#       rx_ant (1, 2 or 0 if unknown - latter only true in case of RX timeout)
#       rx_timestamp (unix timestamp with fractional portion)
#       wwvb_timestamp in ISO format - None if RX error
#       wwvb_time (unix timestamp) - None if RX error
#       rx_delta (wwvb_timestamp - rx_timestamp) - None if RX error
#

def wwvb_emit_clockstats(rx_ret, rx_ant, rx_timestamp, wwvb_time_text = None, wwvb_time = None, wwvb_delta_rx = None):
        # yikes, use better formatting technique
        rx_s = str(rx_ret) + "," + RX_STATUS_WWVB_STR[rx_ret] + ","
        rx_s = rx_s + str(rx_ant) + "," + make_timespec_s(rx_timestamp) + ","
        #
        # FIXME: add last_rx_timestamp code when this is converted into a class.
        # for now just emit a zero
        #
        # if last_rx_timestamp == 0.0:
        #        rx_s = rx_s + "0" + ","
        # else:
        #        rx_s = rx_s + str(rx_timestamp - last_rx_timestamp) + ","
        rx_s = rx_s + "0" + ","
        if wwvb_time_text is None:
                wwvb_time_text = ""
                wwvb_time_s = ""
                wwvb_delta_rx_s = ""
        else:
                wwvb_time_s = make_timespec_s(wwvb_time)
                wwvb_delta_rx_s = make_timespec_s(wwvb_delta_rx)
        rx_s = rx_s + wwvb_time_text + ","
        rx_s = rx_s + wwvb_time_s + ","
        rx_s = rx_s + wwvb_delta_rx_s
        # version 1
        print "RX_WWVB_CLOCKSTATS,v1," + rx_s
        #last_rx_timestamp = rx_timestamp

#
# initiate RX operation on WWVB device and return data
#
def start_rx_wwvb_device(bus, rx_params = ES100_CONTROL_START_RX_ANT1_ANT2):
        # FIXME: check input rx_params
        if rx_params == ES100_CONTROL_START_RX_ANT1:
                print "start_rx_wwvb_device: WWVB receive: starting RX on antenna 1 only"
        if rx_params == ES100_CONTROL_START_RX_ANT2:
                print "start_rx_wwvb_device: WWVB receive: starting RX on antenna 2 only"
        if rx_params == ES100_CONTROL_START_RX_ANT1_ANT2:
                print "start_rx_wwvb_device: WWVB receive: starting RX on antenna 1-2"
        if rx_params == ES100_CONTROL_START_RX_ANT2_ANT1:
                print "start_rx_wwvb_device: WWVB receive: starting RX on antenna 2-1"
        write_wwvb_device(bus, ES100_CONTROL0_REG, rx_params)
        rx_start_time = time.time()
        rx_start_offset = int(rx_start_time) % 60
        print "start_rx_wwvb_device: time/offset = " + str(rx_start_time) + "/" + str(rx_start_offset)

#
# wait for RX operation to complete, return RX timestamp on RX, -1 on timeout
#
def wait_rx_wwvb_device(bus):
        rx_start_time = time.time()
        rx_start_offset = int(rx_start_time) % 60
        print "wait_rx_wwvb_device: time/offset = " + str(rx_start_time) + "/" + str(rx_start_offset)
        c = 0
        rx_time_timeout = rx_start_time + 140
        rx_time_print_debug = 0
        print "wait_rx_wwvb_device:",
        while GPIO.input(GPIO_DEV_IRQ) != 0:
                #
                # per EVERSET specs, irq_status cannot be read until GPIO irq pin is low
                #
                #status0 = read_wwvb_device(bus, ES100_STATUS0_REG)
                rx_time_now = time.time()
                rx_time_waiting = int(rx_time_now - rx_start_time)
                rx_time_now_offset = int(rx_time_now) % 60
                if rx_time_now > rx_time_print_debug:
                        print "   " + str(rx_time_waiting) + "/" + str(rx_time_now_offset),
                        rx_time_print_debug = rx_time_now + 15
                if rx_time_waiting > 150:
                        print ""
                        irq_status = read_wwvb_device(bus, ES100_IRQ_STATUS_REG)
                        control0 = read_wwvb_device(bus, ES100_CONTROL0_REG)
                        status0 = read_wwvb_device(bus, ES100_STATUS0_REG)
                        gpio_irq_pin = GPIO.input(GPIO_DEV_IRQ)
                        #print "read_rx_wwvb_device: control0 reg = " + str(control0)
                        #print "read_rx_wwvb_device: status0 reg = " + str(status0)
                        #print "read_rx_wwvb_device: irq_status reg = " + str(irq_status)
                        print "wait_rx_wwvb_device: TIMEOUT: status0=" + str(status0) + ", " + "irq_status=" + str(irq_status) + ": " + str(time.time() - rx_start_time)
                        return -1
                time.sleep(0.001)
                #
                # FIXME: how does this method know which pin to look for? right?
                # GPIO.wait_for_edge(bus, GPIO.FALLING, timeout = 5700)
                #
        #
        # RX TIMESTAMP is when GPIO_IRQ goes low, thus its accuracy does not depend on the I2C baud rate
        #
        rx_timestamp = time.time()
        print ""
        return rx_timestamp

#
# read RX data from WWVB receiver
#
def read_rx_wwvb_device(bus, rx_timestamp):
        #
        # read irq_status register first, per EVERSET timing diagrams
        #
        irq_status = read_wwvb_device(bus, ES100_IRQ_STATUS_REG)
        control0 = read_wwvb_device(bus, ES100_CONTROL0_REG)
        status0 = read_wwvb_device(bus, ES100_STATUS0_REG)
        gpio_irq_pin = GPIO.input(GPIO_DEV_IRQ)
        print "read_rx_wwvb_device: control0 reg = " + str(control0)
        print "read_rx_wwvb_device: status0 reg = " + str(status0)
        print "read_rx_wwvb_device: irq_status reg = " + str(irq_status)
        print "read_rx_wwvb_device: GPIO_IRQ pin = " + str(gpio_irq_pin)
        #
        # get rx_ant first, then process irq_status before status0 register
        #
        rx_ant = 0
        if (status0 & 0x2) != 0x0:
                print "read_rx_wwvb_device: status0 reg: RX ANTENNA 2"
                rx_ant = 2
        else:
                print "read_rx_wwvb_device: status0 reg: RX ANTENNA 1"
                rx_ant = 1
        #
        # check IRQ_STATUS register first
        #
        if (irq_status & 0x5) == 0x1:
                print "read_rx_wwvb_device: irq_status reg = RX complete - OK"
        else:
                if (irq_status & 0x5) == 0x4:
                        # FIXME: handle retry state. when retrying, irq_status is set to 0x4
                        # control0 reg = 5
                        # status0 reg = 0
                        # irq_status reg = 4
                        print "read_rx_wwvb_device: irq_status reg = RX cycle complete, but no data, receiver retrying - FAILED"
                        print "read_rx_wwvb_device: FIXME: need to handle this case by waiting again"
                        disable_wwvb_device()
                        wwvb_emit_clockstats(RX_STATUS_WWVB_IRQ_NO_DATA, rx_ant, rx_timestamp)
                        return RX_STATUS_WWVB_IRQ_NO_DATA
                else:
                        print "read_rx_wwvb_device: irq_status reg = RX unsuccessful - BAD IRQ STATUS"
                        disable_wwvb_device()
                        wwvb_emit_clockstats(RX_STATUS_WWVB_IRQ_STATUS, rx_ant, rx_timestamp)
                        return RX_STATUS_WWVB_BAD_IRQ_STATUS
        if (status0 & 0x4) != 0x0:
                print "read_rx_wwvb_device: status0 reg: RESERVED BIT IS SET --- ERROR"
                disable_wwvb_device()
                wwvb_emit_clockstats(RX_STATUS_WWVB_BAD_STATUS0, rx_ant, rx_timestamp)
                return RX_STATUS_WWVB_BAD_STATUS0
        if (status0 & 0x5) == 0x1:
                print "read_rx_wwvb_device: status0 reg: RX_OK - OK"
        else:
                print "read_rx_wwvb_device: status0 reg: !RX_OK - FAILED"
                disable_wwvb_device()
                wwvb_emit_clockstats(RX_STATUS_WWVB_BAD_STATUS0, rx_ant, rx_timestamp)
                return RX_STATUS_WWVB_STATUS0_NO_RX
        rx_ret = 0
        if (status0 & 0x2) != 0x0:
                print "read_rx_wwvb_device: status0 reg: RX ANTENNA 2"
                rx_ret = RX_STATUS_WWVB_RX_OK_ANT2
        else:
                print "read_rx_wwvb_device: status0 reg: RX ANTENNA 1"
                rx_ret = RX_STATUS_WWVB_RX_OK_ANT1
        if (status0 & 0x10) != 0:
                print "read_rx_wwvb_device: status0 reg: LEAP second flag indicator"
        if (status0 & 0x60) != 0:
                print "read_rx_wwvb_device: status0 reg: DST flags set"
        if (status0 & 0x80) != 0:
                # FIXME: we do not handle tracking mode yet
                print "read_rx_wwvb_device: status0 reg: **** INTERNAL ERROR: TRACKING FLAG SET UNEXPECTEDLY ****"
                disable_wwvb_device()
                wwvb_emit_clockstats(RX_STATUS_WWVB_BAD_STATUS0, rx_ant, rx_timestamp)
                return RX_STATUS_WWVB_BAD_STATUS0
        year_reg = decode_bcd_byte(read_wwvb_device(bus, ES100_YEAR_REG), offset = 2000)
        month_reg = decode_bcd_byte(read_wwvb_device(bus, ES100_MONTH_REG))
        day_reg = decode_bcd_byte(read_wwvb_device(bus, ES100_DAY_REG))
        hour_reg = decode_bcd_byte(read_wwvb_device(bus, ES100_HOUR_REG))
        minute_reg = decode_bcd_byte(read_wwvb_device(bus, ES100_MINUTE_REG))
        second_reg = decode_bcd_byte(read_wwvb_device(bus, ES100_SECOND_REG))
        next_dst_month_reg = decode_bcd_byte(read_wwvb_device(bus, ES100_NEXT_DST_MONTH_REG))
        next_dst_day_reg = decode_bcd_byte(read_wwvb_device(bus, ES100_NEXT_DST_DAY_REG))
        next_dst_hour_reg = decode_bcd_byte(read_wwvb_device(bus, ES100_NEXT_DST_HOUR_REG))
        #print "read_rx_wwvb_device: YEAR_REG   = " + str(year_reg)
        #print "read_rx_wwvb_device: MONTH_REG  = " + str2(month_reg)
        #print "read_rx_wwvb_device: DAY_REG    = " + str2(day_reg)
        #print "read_rx_wwvb_device: HOUR_REG   = " + str2(hour_reg)
        #print "read_rx_wwvb_device: MINUTE_REG = " + str2(minute_reg)
        #print "read_rx_wwvb_device: SECOND_REG = " + str2(second_reg)
        #print "read_rx_wwvb_device: NEXT_DST_MONTH_REG = " + str2(next_dst_month_reg)
        #print "read_rx_wwvb_device: NEXT_DST_DAY_REG   = " + str2(next_dst_day_reg)
        #print "read_rx_wwvb_device: NEXT_DST_HOUR_REG  = " + str2(next_dst_hour_reg)
        # FIXME: use a time format method instead of this
        wwvb_time = (
                        year_reg,
                        month_reg,
                        day_reg,
                        hour_reg,
                        minute_reg,
                        second_reg,
                        0, 0, 0
                    )
        # FIXME: use a time format method instead of this
        wwvb_time_txt = str(year_reg) + "-" + str2(month_reg) + "-" + str2(day_reg)
        wwvb_time_txt = wwvb_time_txt + "T"
        wwvb_time_txt = wwvb_time_txt + str2(hour_reg) + "-" + str2(minute_reg) + "-" + str2(second_reg)
        wwvb_time_txt = wwvb_time_txt + "Z"
        wwvb_time_secs = time.mktime(wwvb_time)
        #
        # adjust for great circle distance from Ft Collins
        # radio wave speed in earth's atmosphere is roughly the same as the distance in vacuum
        #
        distance_delay = (KILOMETERS_FROM_FTCOLLINS_CO * 1.0) / (SPEED_OF_LIGHT_OVERLAND * 1.0)
        print "read_rx_wwvb_device: adjusting wwvb timestamp for distance_delay = " + str(distance_delay)
        wwvb_time_secs = wwvb_time_secs + distance_delay
        #
        #
        #
        wwvb_delta_rx = wwvb_time_secs - rx_timestamp
        print "read_rx_wwvb_device: WWVB_TIME = " + make_timespec_s(wwvb_time_secs)
        print "read_rx_wwvb_device: WWVB_TIME = " + wwvb_time_txt
        print "read_rx_wwvb_device: wwwb_delta_rx = " + make_timespec_s(wwvb_delta_rx)
        # machine readable line for automated parsing and analysis
        # no other text printed by this tool begins with RX_WWVB
        # emit machine readable stat
        wwvb_emit_clockstats(rx_ret, rx_ant, rx_timestamp, wwvb_time_txt, wwvb_time_secs, wwvb_delta_rx)
        #
        # update NTP SHM segment.
        # FIXME: forking external code is butt-ugly
        # FIXME: should use subprocess.popen
        #
        update_shm_cmd = "./update_shm_one_shot " + make_timespec_s(wwvb_time_secs) + " " + make_timespec_s(rx_timestamp)
        print "read_rx_wwvb_device: update_shm_cmd = " + update_shm_cmd
        ret = os.system(update_shm_cmd)
        print "read_rx_wwvb_device: update_shm_cmd ret code = " + str(ret)
        #
        #
        # FIXME FIXME FIXME
        # from the datasheet it's not quite clear how the ES100 knows when to restart the RX and raise IRQ line again.
        # datasheet says ES100 will start rx after IRS status and timestamp and next dst registers are read, but
        # it's scant in details on exactly what trigers the RX restart.
        # it also doesn't say whether code should do single register reads or multi-byte reads
        #
        gpio_wait_state_change(GPIO_DEV_IRQ, "DEV_IRQ", 0, "low", 1, "high")
        #
        # that's all folks!
        #
        return rx_ret

#
# main entry point - read rx timestamp from wwvb
#
def rx_wwvb_device(rx_params):
        #
        # INITIALIZE EVERSET WWVB RECEIVER
        #
        bus = init_wwvb_device()
        if bus == None:
                print "rx_wwvb_device: ERROR: failed to initialize ES100 device"
                disable_wwvb_device(deep_disable = True)
                wwvb_emit_clockstats(RX_STATUS_WWVB_DEV_INIT_FAILED, 0, time.time())
                return RX_STATUS_WWVB_DEV_INIT_FAILED
        #
        # START EVERSET WWVB RECEIVER RX OPERATION
        #
        start_rx_wwvb_device(bus, rx_params)
        #
        # WAIT FOR RX COMPLETE ON EVERSET WWVB RECEIVER
        #
        rx_timestamp = wait_rx_wwvb_device(bus)
        if rx_timestamp < 0:
                print "rx_wwvb_device: rx operation timeout at " + time.time()
                disable_wwvb_device()
                wwvb_emit_clockstats(RX_STATUS_WWVB_TIMEOUT, 0, time.time())
                return RX_STATUS_WWVB_TIMEOUT
        #
        # RX COMPLETE, READ DATETIME TIMESTAMP FROM EVERSET WWVB RECEIVER
        #
        print "rx_wwvb_device: rx operation complete: " + str(rx_timestamp % 60) + " (minute offset)"
        rx_ret = read_rx_wwvb_device(bus, rx_timestamp)
        print "rx_wwvb_device: LOONEY TUNES -- THAT'S ALL FOLKS!"
        #
        # that's all folks!
        #
        disable_wwvb_device()
        return rx_ret

def rx_wwvb_select_antenna(rx_ret, rx_params):
        #
        # set rx_params for next rx based on current ok_rx
        # XXX: there seems to be no advantage in asking for ANT1_ANT2 over ANT1
        # or asking for ANT2_ANT1 over ANT2. maybe once we allow WWVB device to continue retry, it will.
        # FIXME: add a small function to select this.
        #
        if rx_ret == RX_STATUS_WWVB_RX_OK_ANT1:
                print "rx_wwvb_select_antenna: RX OK: using same antenna ANT1 for next RX"
                return ES100_CONTROL_START_RX_ANT1
        if rx_ret == RX_STATUS_WWVB_RX_OK_ANT2:
                print "rx_wwvb_select_antenna: RX OK: using same antenna ANT2 for next RX"
                return ES100_CONTROL_START_RX_ANT2
        if rx_params == ES100_CONTROL_START_RX_ANT1:
                print "rx_wwvb_select_antenna: RX FAILED on antenna ANT1: using antenna ANT2 for next RX"
                return ES100_CONTROL_START_RX_ANT2
        else:
                print "rx_wwvb_select_antenna: RX FAILED on antenna ANT2: using antenna ANT1 for next RX"
                return ES100_CONTROL_START_RX_ANT1

#
# emit machine readable rx stats line
#
def wwvb_emit_rx_stats(rx_loop, rx_stats):
        rx_total = rx_stats[RX_STATUS_WWVB_RX_OK_ANT1] + rx_stats[RX_STATUS_WWVB_RX_OK_ANT2]
        rx_total = rx_total + rx_stats[RX_STATUS_WWVB_TIMEOUT]
        rx_total = rx_total + rx_stats[RX_STATUS_WWVB_IRQ_NO_DATA] + rx_stats[RX_STATUS_WWVB_BAD_IRQ_STATUS]
        rx_total = rx_total + rx_stats[RX_STATUS_WWVB_BAD_STATUS0] + rx_stats[RX_STATUS_WWVB_STATUS0_NO_RX]
        #
        rx_s = str(rx_stats[RX_STATUS_WWVB_RX_OK_ANT1]) + ","
        rx_s = rx_s + str(rx_stats[RX_STATUS_WWVB_RX_OK_ANT2]) + ","
        rx_s = rx_s + str(rx_stats[RX_STATUS_WWVB_TIMEOUT]) + ","
        rx_s = rx_s + str(rx_stats[RX_STATUS_WWVB_IRQ_NO_DATA]) + ","
        rx_s = rx_s + str(rx_stats[RX_STATUS_WWVB_BAD_IRQ_STATUS]) + ","
        rx_s = rx_s + str(rx_stats[RX_STATUS_WWVB_BAD_STATUS0]) + ","
        rx_s = rx_s + str(rx_stats[RX_STATUS_WWVB_STATUS0_NO_RX]) + ","
        rx_s = rx_s + str(rx_stats[RX_STATUS_WWVB_DEV_INIT_FAILED])
        #
        # version 1
        print "RX_WWVB_STAT_COUNTERS,v1," + str(rx_loop) + "," + str(rx_total) + "," + rx_s

def main():
        rx_params = 0
        if len(sys.argv) > 1 and sys.argv[1] == '1': 
                rx_params = ES100_CONTROL_START_RX_ANT1
        if len(sys.argv) > 1 and sys.argv[1] == '2': 
                rx_params = ES100_CONTROL_START_RX_ANT2
        #
        # XXX: there seems to be no advantage in asking for ANT1_ANT2 over ANT1
        # or asking for ANT2_ANT1 over ANT2
        #
        # if len(sys.argv) > 1 and sys.argv[1] == '1-2': 
        #        rx_params = ES100_CONTROL_START_RX_ANT1_ANT2
        # if len(sys.argv) > 1 and sys.argv[1] == '2-1': 
        #        rx_params = ES100_CONTROL_START_RX_ANT2_ANT1
        # if rx_params == 0:
        #         rx_params = ES100_CONTROL_START_RX_ANT1_ANT2
        #
        if rx_params == 0:
                rx_params = ES100_CONTROL_START_RX_ANT1
        # stats
        rx_stats = [ 0 ] * (RX_STATUS_MAX_STATUS + 1)
        rx_loop = 0
        #
        # do one time init before entering RX loop
        #
        one_time_init_wwvb_device()
        #
        # RX loop
        #
        while True:
                #
                # get wwvb timestamp
                #
                t0 = time.time()
                rx_ret = rx_wwvb_device(rx_params)
                t1 = time.time()
                #
                # emit stats
                #
                print "main: rx_loop = " + str(rx_loop) + " complete, rx_ret = " + str(rx_ret) + ", elapsed = " + str(t1-t0)
                rx_loop = rx_loop + 1
                rx_stats[rx_ret] = rx_stats[rx_ret] + 1
                wwvb_emit_rx_stats(rx_loop, rx_stats)
                #
                # select rx antenna for next receive
                #
                rx_params = rx_wwvb_select_antenna(rx_ret, rx_params)

main()
