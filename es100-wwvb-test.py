#!/usr/bin/python -u

#
# Simple python code to test ES100 WWVB receiver.
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
#
# TODO: issues to be fixed:
# (A) The receiver can trigger an IRQ which simply indicates that RX was unsuccessful,
#     and retry is pending. The current code simply treats this as a timeout and restarts reception.
# (B) Tracking mode (essentially equivalent to a "PPS" mode) needs to be supported,
#     see datasheet for details.
#

import RPi.GPIO as GPIO
import smbus
import time
import sys

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

def decode_bcd_byte(raw_bcd, offset = 0):
        val = raw_bcd & 0xf
        val = val + ((raw_bcd >> 4) & 0xf) * 10
        return val + offset

def get_gpio_irq():
        return GPIO.input(GPIO_DEV_IRQ)

def write_wwvb_device(bus, reg, value):
        bus.write_byte_data(ES100_SLAVE_ADDR, reg, value)
        time.sleep(0.005)

def read_wwvb_device(bus, reg):
        bus.write_byte(ES100_SLAVE_ADDR, reg)
        time.sleep(0.005)
        val = bus.read_byte(ES100_SLAVE_ADDR)
        time.sleep(0.005)
        return val

def enable_wwvb_device():
        #
        # FIXME: wait util IRQ GPIO pin goes low and add timeout
        #
        print 'enable_wwvb_device: enabling WWVB device, setting enable=1'
        GPIO.output(GPIO_DEV_ENABLE, GPIO.HIGH)
        print "enable_wwvb_device: GPIO IRQ pin = " + str(get_gpio_irq())
        time.sleep(0.5)
        print "enable_wwvb_device: GPIO IRQ pin = " + str(get_gpio_irq())
        time.sleep(2)
        print "enable_wwvb_device: GPIO IRQ pin = " + str(get_gpio_irq())
        time.sleep(2)

def disable_wwvb_device():
        #
        # FIXME: wait util IRQ GPIO pin goes high and add timeout
        #
        print 'disable_wwvb_device: disabling WWVB device, setting enable=0'
        GPIO.output(GPIO_DEV_ENABLE, GPIO.LOW)
        print "disable_wwvb_device: GPIO IRQ pin = " + str(get_gpio_irq())
        time.sleep(0.5)
        print "disable_wwvb_device: GPIO IRQ pin = " + str(get_gpio_irq())
        time.sleep(2)
        print "disable_wwvb_device: GPIO IRQ pin = " + str(get_gpio_irq())
        time.sleep(2)
        print "disable_wwvb_device: GPIO IRQ pin = " + str(get_gpio_irq())
        time.sleep(2)

def set_gpio_pins_wwvb_device():
        #
        # FIXME: check pin functions to make sure I2C/SMBUS is enabled
        #
        print "set_gpio_pins_wwvb_device: setting GPIO pins for WWVB receiver"
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(GPIO_DEV_ENABLE, GPIO.OUT)
        GPIO.setup(GPIO_DEV_IRQ, GPIO.IN)
        func = GPIO.gpio_function(GPIO_DEV_I2C_SCL_PIN)
        print "set_gpio_pins_wwvb_device: func I2C_SCL_PIN = " + str(func) + "/" + str(GPIO.I2C)
        if func != GPIO.I2C:
                print "set_gpio_pins_wwvb_device: ERROR: function I2C_SCL_PIN is not GPIO.I2C"
        func = GPIO.gpio_function(GPIO_DEV_I2C_SDA_PIN)
        if func != GPIO.I2C:
                print "set_gpio_pins_wwvb_device: ERROR: function I2C_SDA_PIN is not GPIO.I2C"
        print "set_gpio_pins_wwvb_device: func I2C_SDA_PIN = " + str(func) + "/" + str(GPIO.I2C)
        time.sleep(0.5)
        print "set_gpio_pins_wwvb_device: GPIO IRQ pin = " + str(get_gpio_irq())
        time.sleep(0.5)
        print "set_gpio_pins_wwvb_device: GPIO IRQ pin = " + str(get_gpio_irq())

def init_wwvb_device():
        print "init_wwvb_device: initializing ES100 WWVB receiver"
        #
        # set gpio pins
        #
        set_gpio_pins_wwvb_device()
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
        time.sleep(0.5)
        print "init_wwvb_device: reading from WWVB device"
        val = bus.read_byte(ES100_SLAVE_ADDR)
        print "init_wwvb_device: device value = " + str(val)
        time.sleep(0.5)
        #
        # per EVERSET specs, only read irq_status after IRQ goes low
        #
        print "init_wwvb_device: control0 reg = " + str(read_wwvb_device(bus, ES100_CONTROL0_REG))
        # control 1 register can be read and written, but it's a NOOP
        print "init_wwvb_device: control1 reg = " + str(read_wwvb_device(bus, ES100_CONTROL1_REG))
        print "init_wwvb_device: status0 reg = " + str(read_wwvb_device(bus, ES100_STATUS0_REG))
        # print "irq_status reg = " + str(read_wwvb_device(ES100_IRQ_STATUS_REG))
        print "init_wwvb_device: GPIO IRQ pin = " + str(get_gpio_irq())
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
        print "start_rx_wwvb_device: RX start time = " + str(rx_start_time)
        print "start_rx_wwvb_device: RX start offset = " + str(rx_start_offset)

#
# wait for RX operation to complete, return RX timestamp on RX, -1 on timeout
#
def wait_rx_wwvb_device(bus):
        rx_start_time = time.time()
        rx_start_offset = int(rx_start_time) % 60
        print "wait_rx_wwvb_device: RX start time = " + str(rx_start_time)
        print "wait_rx_wwvb_device: RX start offset = " + str(rx_start_offset)
        c = 0
        rx_time_timeout = rx_start_time + 140
        rx_time_print_debug = 0
        while get_gpio_irq() != 0:
                #
                # per EVERSET specs, irq_status cannot be read until GPIO irq pin is low
                #
                #status0 = read_wwvb_device(bus, ES100_STATUS0_REG)
                rx_time_now = time.time()
                rx_time_waiting = int(rx_time_now - rx_start_time)
                rx_time_now_offset = int(rx_time_now) % 60
                if rx_time_now > rx_time_print_debug:
                        #print "wait_rx_wwvb_device: status0 = " + str(status0) + ": wait time " + str(rx_time_waiting) + ", wait offset " + str(rx_time_now_offset)
                        print "wait_rx_wwvb_device: wait time " + str(rx_time_waiting) + ", wait offset " + str(rx_time_now_offset)
                        rx_time_print_debug = rx_time_now + 15
                if rx_time_waiting > 150:
                        print "wait_rx_wwvb_device: TIMEOUT: status0 = " + str(status0) + ": wait time: " + str(time.time() - rx_start_time) + " seconds"
                        return -1
                time.sleep(0.002)
                #
                # FIXME: how does this method know which pin to look for? right?
                # GPIO.wait_for_edge(bus, GPIO.FALLING, timeout = 5700)
                #

        return time.time()

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
        gpio_irq_pin = get_gpio_irq()
        print "read_rx_wwvb_device: control0 reg = " + str(control0)
        print "read_rx_wwvb_device: status0 reg = " + str(status0)
        print "read_rx_wwvb_device: irq_status reg = " + str(irq_status)
        print "read_rx_wwvb_device: GPIO IRQ pin = " + str(gpio_irq_pin)
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
                        return RX_STATUS_WWVB_IRQ_NO_DATA
                else:
                        print "read_rx_wwvb_device: irq_status reg = RX unsuccessful - FAILED"
                        disable_wwvb_device()
                        return RX_STATUS_WWVB_BAD_IRQ_STATUS
        if (status0 & 0x4) != 0x0:
                print "read_rx_wwvb_device: status0 reg: RESERVED BIT IS SET --- ERROR"
                disable_wwvb_device()
                return RX_STATUS_WWVB_BAD_STATUS0
        if (status0 & 0x5) == 0x1:
                print "read_rx_wwvb_device: status0 reg: RX_OK - OK"
        else:
                print "read_rx_wwvb_device: status0 reg: !RX_OK - FAILED"
                disable_wwvb_device()
                return RX_STATUS_WWVB_STATUS0_NO_RX
        rx_ok_ret = 0
        if (status0 & 0x2) != 0x0:
                print "read_rx_wwvb_device: status0 reg: RX ANTENNA 2"
                rx_ok_ret = RX_STATUS_WWVB_RX_OK_ANT2
        else:
                print "read_rx_wwvb_device: status0 reg: RX ANTENNA 1"
                rx_ok_ret = RX_STATUS_WWVB_RX_OK_ANT1
        if (status0 & 0x10) != 0:
                print "read_rx_wwvb_device: status0 reg: LEAP second flag indicator"
        if (status0 & 0x60) != 0:
                print "read_rx_wwvb_device: status0 reg: DST flags set"
        if (status0 & 0x80) != 0:
                # FIXME: we do not handle tracking mode yet
                print "read_rx_wwvb_device: status0 reg: **** TRACKING FLAG SET UNEXPECTEDLY ****"
                disable_wwvb_device()
                return RX_STATUS_WWVB_BAD_STATUS0
        year_reg = read_wwvb_device(bus, ES100_YEAR_REG)
        month_reg = read_wwvb_device(bus, ES100_MONTH_REG)
        day_reg = read_wwvb_device(bus, ES100_DAY_REG)
        hour_reg = read_wwvb_device(bus, ES100_HOUR_REG)
        minute_reg = read_wwvb_device(bus, ES100_MINUTE_REG)
        second_reg = read_wwvb_device(bus, ES100_SECOND_REG)
        print "read_rx_wwvb_device: YEAR_REG   = " + str(decode_bcd_byte(year_reg, offset = 2000))
        print "read_rx_wwvb_device: MONTH_REG  = " + str(decode_bcd_byte(month_reg))
        print "read_rx_wwvb_device: DAY_REG    = " + str(decode_bcd_byte(day_reg))
        print "read_rx_wwvb_device: HOUR_REG   = " + str(decode_bcd_byte(hour_reg))
        print "read_rx_wwvb_device: MINUTE_REG = " + str(decode_bcd_byte(minute_reg))
        print "read_rx_wwvb_device: SECOND_REG = " + str(decode_bcd_byte(second_reg))
        print "read_rx_wwvb_device:"
        print "read_rx_wwvb_device:"
        wwvb_time = (
                        decode_bcd_byte(year_reg, offset = 2000),
                        decode_bcd_byte(month_reg),
                        decode_bcd_byte(day_reg),
                        decode_bcd_byte(hour_reg),
                        decode_bcd_byte(minute_reg),
                        decode_bcd_byte(second_reg),
                        0, 0, 0
                    )
        wwvb_time_secs = time.mktime(wwvb_time)
        wwvb_delta_rx_timestamp_ms = (wwvb_time_secs - rx_timestamp) * 1000.0
        print "read_rx_wwvb_device: WWVB_TIME = " + str(wwvb_time)
        print "read_rx_wwvb_device: WWVB_TIME = " + str(wwvb_time_secs)
        print "read_rx_wwvb_device: delta(WWVB_TIME, rx_timestamp) = " + str(wwvb_delta_rx_timestamp_ms) + " msecs"
        print "read_rx_wwvb_device:"
        print "read_rx_wwvb_device:"

        #
        # that's all folks!
        #
        return rx_ok_ret

#
# RX time -- currently the only exposed API
#
def rx_wwvb_device(rx_params):
        #
        # INITIALIZE EVERSET WWVB RECEIVER
        #
        print "rx_wwvb_device: initializing, rx_params=" + str(rx_params)
        bus = init_wwvb_device()
        #
        # START EVERSET WWVB RECEIVER RX OPERATION
        #
        print "rx_wwvb_device: starting rx operation"
        start_rx_wwvb_device(bus, rx_params)
        #
        # WAIT FOR RX COMPLETE ON EVERSET WWVB RECEIVER
        #
        print "rx_wwvb_device: wait for rx operation to complete"
        rx_timestamp = wait_rx_wwvb_device(bus)
        print "rx_wwvb_device: rx operation complete: " + str(rx_timestamp)
        if rx_timestamp < 0:
                print "rx_wwvb_device: ERROR: rx operation timeout"
                disable_wwvb_device()
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
        if len(sys.argv) > 1 and sys.argv[1] == '1-2': 
               rx_params = ES100_CONTROL_START_RX_ANT1_ANT2
        if len(sys.argv) > 1 and sys.argv[1] == '2-1': 
               rx_params = ES100_CONTROL_START_RX_ANT2_ANT1
        if rx_params == 0:
                rx_params = ES100_CONTROL_START_RX_ANT1_ANT2
        rx_ret_count = [ 0, 0, 0, 0, 0, 0, 0, 0 ]
        rx_loop = 0
        while True:
                print "main: rx_loop = " + str(rx_loop)
                t0 = time.time()
                rx_ret = rx_wwvb_device(rx_params)
                t1 = time.time()
                print "main: rx_loop = " + str(rx_loop) + " complete, rx_ret = " + str(rx_ret)
                print "main: rx_loop = " + str(rx_loop) + " complete, elapsed = " + str(t1-t0)
                rx_ret_count[rx_ret] = rx_ret_count[rx_ret] + 1
                print "main: rx_ret_count[RX_OK_ANT1]     = " + str(rx_ret_count[RX_STATUS_WWVB_RX_OK_ANT1])
                print "main: rx_ret_count[RX_OK_ANT2]     = " + str(rx_ret_count[RX_STATUS_WWVB_RX_OK_ANT2])
                print "main: rx_ret_count[TIMEOUT]        = " + str(rx_ret_count[RX_STATUS_WWVB_TIMEOUT])
                print "main: rx_ret_count[IRQ_NO_DATA]    = " + str(rx_ret_count[RX_STATUS_WWVB_IRQ_NO_DATA])
                print "main: rx_ret_count[BAD_IRQ_STATUS] = " + str(rx_ret_count[RX_STATUS_WWVB_BAD_IRQ_STATUS])
                print "main: rx_ret_count[BAD_STATUS0]    = " + str(rx_ret_count[RX_STATUS_WWVB_BAD_STATUS0])
                print "main: rx_ret_count[STATUS0_NO_RX]  = " + str(rx_ret_count[RX_STATUS_WWVB_STATUS0_NO_RX])
                print ""
                print ""
                #
                # set rx_params for next rx based on current ok_rx
                # XXX: there seems to be no advantage in asking for ANT1_ANT2 over ANT1
                # or asking for ANT2_ANT1 over ANT2
                # FIXME: add a small function to select this
                #
                if rx_ret == RX_STATUS_WWVB_RX_OK_ANT1:
                        print "main: rx_loop = " + str(rx_loop) + " RX ok, using same antenna ANT1 for next RX"
                        rx_params = ES100_CONTROL_START_RX_ANT1
                if rx_ret == RX_STATUS_WWVB_RX_OK_ANT2:
                        print "main: rx_loop = " + str(rx_loop) + " RX ok, using same antenna ANT2 for next RX"
                        rx_params = ES100_CONTROL_START_RX_ANT2
                if rx_ret != RX_STATUS_WWVB_RX_OK_ANT1 and rx_ret != RX_STATUS_WWVB_RX_OK_ANT2:
                        if rx_params == ES100_CONTROL_START_RX_ANT1:
                                print "main: rx_loop = " + str(rx_loop) + " RX failed, using other antenna ANT2 for next RX"
                                rx_params = ES100_CONTROL_START_RX_ANT2
                        else:
                                print "main: rx_loop = " + str(rx_loop) + " RX failed, using other antenna ANT1 for next RX"
                                rx_params = ES100_CONTROL_START_RX_ANT2
                rx_loop = rx_loop + 1

main()
