#!/usr/bin/env python3
"""BMP388example2, example for using the BMP388 python
module. Example using the sensor data ready interrupt to
read the pressure and temperature output registers

created Jan 30, 2022 OM
modified Jan 30, 2022 OM"""

"""
Copyright 2022 Owain Martin

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import time, smbus, spidev
import BMP388

# sensor data ready interrupt set up and output reading example    

# uncomment line below if using SPI
#bmp388 = BMP388.BMP388('spi', spiPort = 0, spiCS = 1)

# uncomment the 2 lines below if using i2c
i2cAddress = 0x76
bmp388 = BMP388.BMP388('i2c', i2cAddress)  

# read chip id register - should return 0x50
print(f'Reading Chip ID register (should output 0x50): {hex(bmp388.single_access_read(0x00))}')     
bmp388.set_sensor_enables(t = 1, p = 1)     # enable T & P sensors
bmp388.set_power_mode('normal')             # set power mode to normal - set after enabling sensors
bmp388.set_odr(0x07)                        # set ODR to 25/18 Hz (640ms)
bmp388.set_osr(t_osr = 1, p_osr = 4)        # set t_OSR x1, p_OSR x4
bmp388.set_iir_filter(0)                    # set iiR filter coeff to 0
bmp388.config_int_pin(outputMode = 'pushpull', level = 'high', latch = False)
bmp388.set_interrupts(drdy = 1, fifoFull = 0, fifoWtm = 0)    

print('Ctrl-C to stop readings')

while True:
    # check status register to see if data is ready to be read
    intStatus = bmp388.read_interrupt_status()    
    if intStatus[0] == 1:
        temperature, pressure = bmp388.get_output()
        print(f'Temperature: {temperature:.2f}C Pressure: {pressure:.2f}hPa')
    
    time.sleep(1)
