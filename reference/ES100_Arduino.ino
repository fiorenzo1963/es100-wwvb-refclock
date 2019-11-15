
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// es100_host_arduino.c
//
// Xtendwave ES100 Example Host MCU Code for Arduino - version 0002
//
// Copyright 2013 Xtendwave
//
// THIS SOFTWARE IS PROVIDED TO YOU "AS IS," AND WE MAKE NO EXPRESS OR IMPLIED
// WARRANTIES WHATSOEVER WITH RESPECT TO ITS FUNCTIONALITY, OPERABILITY, OR USE,
// INCLUDING, WITHOUT LIMITATION, ANY IMPLIED WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE, OR INFRINGEMENT. WE EXPRESSLY DISCLAIM ANY
// LIABILITY WHATSOEVER FOR ANY DIRECT, INDIRECT, CONSEQUENTIAL, INCIDENTAL OR
// SPECIAL DAMAGES, REGARDLESS OF THE FORM OF ACTION OR LEGAL THEORY UNDER
// WHICH THE LIABILITY MAY BE ASSERTED, EVEN IF ADVISED OF THE POSSIBILITY OR
// LIKELIHOOD OF SUCH DAMAGES.
//
// This software contains example C code that shows how an Arduino host
// microcontroller can control an ES100 device. The software uses the
// Serial.print() function to display the time.
//
// 4 pins are used. 1 digital output pin controls EN and 1 digital input pin
// monitors IRQ. Two pins, SCL and SDA, connect to the ES100 serial interface.
//
// This software was testing using Arduino Software version 1.5.2 and an Arduino Due.
//
// User-Supplied Functions
//
//   mcu_init
//   mcu_gpio_set_high
//   mcu_gpio_set_low
//   mcu_gpio_read
//   mcu_timer_read
//   mcu_timer_wait_us
//
// General I2C Functions
//
//   i2c_write
//   i2c_read
//
// ES100 Functions
//
//   es100_write_register
//   es100_read_register
//   es100_enable
//   es100_disable
//   es100_start_rx
//   es100_wait_for_irq
//   es100_get_irq_status
//   es100_read_time
//
// Other
//
//   setup
//   loop
//
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

#include <Wire.h>

//------------------------------------------------------------------------------
// MCU constants - USER TO MODIFY
//------------------------------------------------------------------------------

// GPIO pins
#define GPIO_EN      4
#define GPIO_IRQ     2

//------------------------------------------------------------------------------
// constants
//------------------------------------------------------------------------------

// I2C slave address
#define ES100_SLAVE_ADDR 0x32

// enable delay time
//#define ENABLE_DELAY_US 1500
#define ENABLE_DELAY_US 100000       // 100ms: module version 1 only

// I2C clock high/low time
#define SCL_TIME_US 2

// ES100 API register addresses
#define ES100_CONTROL0_REG       0x00
#define ES100_CONTROL1_REG       0x01
#define ES100_IRQ_STATUS_REG     0x02
#define ES100_STATUS0_REG        0x03
#define ES100_YEAR_REG           0x04
#define ES100_MONTH_REG          0x05
#define ES100_DAY_REG            0x06
#define ES100_HOUR_REG           0x07
#define ES100_MINUTE_REG         0x08
#define ES100_SECOND_REG         0x09
#define ES100_NEXT_DST_MONTH_REG 0x0A
#define ES100_NEXT_DST_DAY_REG   0x0B
#define ES100_NEXT_DST_HOUR_REG  0x0C

//------------------------------------------------------------------------------
// array to store date and time
//------------------------------------------------------------------------------

#define DT_STATUS         0
#define DT_YEAR           1
#define DT_MONTH          2
#define DT_DAY            3
#define DT_HOUR           4
#define DT_MINUTE         5
#define DT_SECOND         6
#define DT_NEXT_DST_MONTH 7
#define DT_NEXT_DST_DAY   8
#define DT_NEXT_DST_HOUR  9

#define DT_LENGTH        10

//------------------------------------------------------------------------------
// mcu functions - USER TO MODIFY
//------------------------------------------------------------------------------

void mcu_init(void)
{
  // optional code to initialize GPIO, timer and other MCU functions
  pinMode(GPIO_EN, OUTPUT);
  pinMode(GPIO_IRQ, INPUT);
  Wire.begin();
  Serial.begin(9600);
}

void mcu_gpio_set_high(int pin)
{
  // set pin to a logic one
  digitalWrite(pin, HIGH);
}

void mcu_gpio_set_low(int pin)
{
  // set pin to a logic zero
  digitalWrite(pin, LOW);
}

int mcu_gpio_read(int pin)
{
  // read current state of input pin
  return(digitalRead(pin));
}

int mcu_timer_read(void)
{
  // read timer value and return as integer
  return(millis());
}

void mcu_timer_wait_us(int count)
{
  // wait 1 microsecond or greater for each count
  delayMicroseconds(count);
}

//------------------------------------------------------------------------------
// write I2C data - USER TO MODIFY
//------------------------------------------------------------------------------

void i2c_write(uint8_t slave_addr, uint8_t num_bytes, uint8_t *ptr)
{

  int i;

  Wire.beginTransmission(slave_addr);

  for (i=0; i<num_bytes; i++)
  {
    Wire.write(ptr[i]);
  }

  Wire.endTransmission();

}

//------------------------------------------------------------------------------
// read I2C data - USER TO MODIFY
//------------------------------------------------------------------------------

void i2c_read(uint8_t slave_addr, uint8_t num_bytes, uint8_t *ptr)
{

  int i;
  const uint8_t stop_flag = 1;

  Wire.requestFrom(slave_addr, num_bytes, stop_flag);
  for(i=0; (i<num_bytes && Wire.available()); i++)
  {
    ptr[i] = Wire.read();
  }
}

//------------------------------------------------------------------------------
// write data to an ES100 API register
//------------------------------------------------------------------------------

void es100_write_register(uint8_t addr, uint8_t data)
{
  uint8_t es100_write_array[2];

  es100_write_array[0]= addr;
  es100_write_array[1]= data;
  i2c_write(ES100_SLAVE_ADDR, 0x2, es100_write_array);
}

//------------------------------------------------------------------------------
// read data from an ES100 API register
//------------------------------------------------------------------------------

uint8_t es100_read_register(uint8_t addr)
{
  uint8_t   data;

  i2c_write(ES100_SLAVE_ADDR, 0x1, &addr);
  i2c_read(ES100_SLAVE_ADDR, 0x1, &data);

  return(data);
}

//------------------------------------------------------------------------------
// enable ES100
//------------------------------------------------------------------------------

void es100_enable(void)
{
  mcu_gpio_set_high(GPIO_EN);
}

//------------------------------------------------------------------------------
// disable ES100
//------------------------------------------------------------------------------

void es100_disable(void)
{
  mcu_gpio_set_low(GPIO_EN);
}

//------------------------------------------------------------------------------
// start reception
//------------------------------------------------------------------------------

void es100_start_rx()
{
  es100_write_register(ES100_CONTROL0_REG, 0x01);

  // perform read of control register for i2c debug only
  es100_read_register(ES100_CONTROL0_REG);
}

//------------------------------------------------------------------------------
// wait for falling edge of IRQ
//------------------------------------------------------------------------------

void es100_wait_for_irq()
{
  while(mcu_gpio_read(GPIO_IRQ));   // wait until IRQ is low
}

//------------------------------------------------------------------------------
// read IRQ status register
//------------------------------------------------------------------------------

uint8_t es100_get_irq_status()
{
  //Read IRQ status register
  return(es100_read_register(ES100_IRQ_STATUS_REG));
}

//------------------------------------------------------------------------------
// read date and time from API registers
//------------------------------------------------------------------------------

void es100_read_time(int dt_array[])
{
  dt_array[DT_STATUS]         = es100_read_register(ES100_STATUS0_REG);
  dt_array[DT_YEAR]           = es100_read_register(ES100_YEAR_REG);
  dt_array[DT_MONTH]          = es100_read_register(ES100_MONTH_REG);
  dt_array[DT_DAY]            = es100_read_register(ES100_DAY_REG);
  dt_array[DT_HOUR]           = es100_read_register(ES100_HOUR_REG);
  dt_array[DT_MINUTE]         = es100_read_register(ES100_MINUTE_REG);
  dt_array[DT_SECOND]         = es100_read_register(ES100_SECOND_REG);
  dt_array[DT_NEXT_DST_MONTH] = es100_read_register(ES100_NEXT_DST_MONTH_REG);
  dt_array[DT_NEXT_DST_DAY]   = es100_read_register(ES100_NEXT_DST_DAY_REG);
  dt_array[DT_NEXT_DST_HOUR]  = es100_read_register(ES100_NEXT_DST_HOUR_REG);
}

//------------------------------------------------------------------------------
// top level function to receive time from WWVB
//------------------------------------------------------------------------------

int es100_receive(int dt_array[])
{
  // local variables
  int         irq_status = 0;
  int         current_timer_value;
  int         count = 0;

  // enable and delay
  Serial.println("enabling es100");
  es100_enable();
  mcu_timer_wait_us(ENABLE_DELAY_US);

  // start reception
  es100_start_rx();

  // loop until time received
  while (irq_status != 0x01)
  {
    // wait for interrupt
    Serial.print("count = ");
    Serial.print(count++);
    Serial.print(", waiting for interrupt ... ");
    es100_wait_for_irq();
    Serial.println("received");

    // interrupt defines second boundary, so save current timer value
    current_timer_value = mcu_timer_read();

    // read interrupt status
    irq_status = es100_get_irq_status();
    Serial.print("IRQ status = 0x");
    Serial.println(irq_status, HEX);
  }

  // read date and time
  es100_read_time(dt_array);

  // disable ES100
  es100_disable();

  // return timer value when interrupt occurred
  return(current_timer_value);
}

//------------------------------------------------------------------------------
// setup function - runs once
//------------------------------------------------------------------------------

void setup()
{
  // local variables
  int         current_timer_value;
  int         dt_array[DT_LENGTH];

  // initialize MCU
  mcu_init();

  // initialize gpios
  mcu_gpio_set_low(GPIO_EN);

  // receive time from WWVB
  current_timer_value = es100_receive(dt_array);

  // display date and time
  Serial.print("received UTC time = 20");
  Serial.print(dt_array[DT_YEAR], HEX);
  Serial.print(":");
  Serial.print(dt_array[DT_MONTH], HEX);
  Serial.print(":");
  Serial.print(dt_array[DT_DAY], HEX);
  Serial.print(" ");
  Serial.print(dt_array[DT_HOUR], HEX);
  Serial.print(":");
  Serial.print(dt_array[DT_MINUTE], HEX);
  Serial.print(":");
  Serial.println(dt_array[DT_SECOND], HEX);

  // display timer value when second occurred
  Serial.print("second boundary occurred at timer count = ");
  Serial.println(current_timer_value);

  // display status register
  Serial.print("status register = 0x");
  Serial.println(dt_array[DT_STATUS], HEX);

  // display next DST transition date
  Serial.print("next DST transition = month ");
  Serial.print(dt_array[DT_NEXT_DST_MONTH], HEX);
  Serial.print(", day ");
  Serial.print(dt_array[DT_NEXT_DST_DAY], HEX);
  Serial.print(", hour ");
  Serial.println(dt_array[DT_NEXT_DST_HOUR], HEX);

}


//------------------------------------------------------------------------------
// loop function - not used
//------------------------------------------------------------------------------

void loop() {
  // put your main code here, to run repeatedly:

}



