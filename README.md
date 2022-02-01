# BMP388-Python-Module
Python 3.x to use with the BMP388 pressure and temperature sensor. Testing done with a BMP388 breakout
 board from Adafuit and a Raspberry Pi3.

This supports both SPI and i2C. SPI requires py-spidev and python-dev modules. i2C requires smbus.
Examples that use the interrupt pin require the RPi.GPIO module (examples 3,4 & 7).

SPI connections to the BMP388 board from the Pi are as follows:
- Pi 3.3V to BMP388 Vin
- Pi Gnd to BMP388 Gnd
- Pi SCLK to BMP388 SCL
- Pi MOSI to BMP388 SDI
- Pi MISO to BMP388 SDO
- Pi CE0 or CE1 to BMP388 CS
- Pi GPIO # to BMP388 INT - optional

i2c connections to the BMP388 board from the Pi are as follows:
- Pi 3.3V to BMP388 Vin
- Pi Gnd to BMP388 Gnd
- Pi SCL to BMP388 SCL
- Pi SDA to BMP388 SDI
- Pi GND to BMP388 SDO for i2C address 0x76
- Pi 3.3V to BMP388 SDO for i2C address 0x77
- BMP388 CS - not connected
- Pi GPIO # to BMP388 INT - optional

Included are 7 example files
- BMP388example1 - Simple example to read the pressure and temperature output registers
- BMP388example2 - Example using the sensor data ready interrupt to read the pressure and temperature output registers
- BMP388example3 - Example using the sensor data ready interrupt and the sensor interrupt pin to read the pressure and
temperature output registers
- BMP388example4 - 2nd example using the sensor data ready interrupt and the sensor interrupt pin to read the pressure and
temperature output registers
- BMP388example5 - Example using the FIFO watermark interrupt to read the pressure and temperature from FIFO
- BMP388example6 - Example using the FIFO watermark interrupt to read the pressure, temperature and time from FIFO
- BMP388example7 - Example using the sensor FIFO watermark interrupt and the sensor interrupt pin to read the pressure and temperature
from FIFO

The BMP388 datasheet is useful for figuring out how to correctly configure the BMP338 for your application.

Current functions include:
- get_output() - read the sensor output from the temperature and pressure registers (0x04 to 0x09)
- set_power_mode(mode = 'normal') - function to set the power mode of the sensor. Valid values are; sleep, force, normal
- set_sensor_enables(t = 1, p = 1) - function to enable/disable the temperature and pressure sensors
- set_odr(odrSc = 7) - function to set the odr for the sensor. Values of 0-17 are valid
- set_osr(t_osr = 1, p_osr = 1) - function to set the oversampling rates for the temperature and pressue sensor. Vaild values for each are 1,2,4,8,16 & 32
- set_iir_filter(iirFilter = 0) - function to set the iir filter coefficient for the sensor. Valid values are 0,1,3,7,15,31,63,127
- sensor_reset() - sets configuration settings back to default values and clears the contents of the FIFO data buffer

- config_int_pin(outputMode = 'pushpull', level = 'high', latch = False) - function to configure the sensor interrupt pin. OutputMode sets the pin
 to either pushpull or opendrain.  level sets the active level to either high or low. latch sets the pin to be either latching (True) 
or non-latching (False)
- set_interrupts(drdy = 0, fifoFull = 0, fifoWtm = 0) - function to set which conditions trigger the sensor interrupt pin. The 3 conditions are;
 Data Ready, FIFO Full and FIFO Watermark. These conditions are ORed together.
- read_interrupt_status() - function to read the INT_STATUS register (0x11) and return the 3 components 
(drdy, fifoFull, fifoWtm) in a tuple

- set_fifo_mode(mode = 'on', fifoStopOnFull = 0, fifo_p = 1, fifo_t = 1, fifo_time = 1) - function to turn the fifo mode on or off and 
enable/disable storing of pressure, temperature and/or time data in the fifo data buffer. It also enables/disables the fifo stop on full.
- set_fifo_watermark(fifo_wtm = 1) - function to set the fifo watermark value. Valid values are between 1 and 511.
- set_fifo_data_options(subsampling = 0, iirFilter = 0) - function to set the FIFO downsampling value, valid values between 0 and 7 
corresponding to downsampling of 2^subsamping for range of 1-128. This also sets whether unfiltered or iiR filter data is stored in FIFO.
- read_bytes_in_fifo() - returns the number of bytes currently in the fifo data buffer
- clear_fifo() - clears the contents of the FIFO data buffer.
- read_fifo(numBytes, timeEnabled = False) - read numBytes of data out of the fifo data buffer. Returns temperature, pressure and, if enabled, time data
back in individual lists
-
