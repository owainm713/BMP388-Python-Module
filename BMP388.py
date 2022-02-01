#!/usr/bin/env python3
"""BMP388, module for use with a BMP388 pressure and
temperature sensor

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

class BMP388:
    
    def __init__(self, comMode, i2cAddress = 0x0, spiPort = 0, spiCS = 0):
        
        self.comMode = comMode
        
        if self.comMode == 'i2c':
            # set up i2c bus
            self.i2cBus  = smbus.SMBus(1)
            self.i2cAddress = i2cAddress
        else:
            self.spi=spidev.SpiDev()
            self.spi.open(spiPort,spiCS)
            self.spi.max_speed_hz = 4000000
            
        self.get_calibration_parameters()
        
    def multi_access_write(self, reg=0x00, regValues = [0x00]):
        """multi_access_write function to write to multiple registers at
        once"""
        
        if self.comMode == 'i2c':
            self.i2cBus.write_i2c_block_data(self.i2cAddress, reg, regValues)
        else:
            rwBit = 0b0  # read/write bit set to write
            data = [(rwBit<<7)+reg]
            data.extend(regValues)
            self.spi.xfer2(data)
            

        return

    def multi_access_read(self, reg=0x00, numRead = 1):
        """multi_access_read, function to read multiple registers at
        once"""
        
        if self.comMode == 'i2c':
            dataTransfer = self.i2cBus.read_i2c_block_data(self.i2cAddress, reg, numRead)
        else:
            rwBit = 0b1  # read/write bit set to read
            data = [(rwBit<<7)+reg]
            data.extend([0x00 for i in range(numRead+1)])
            dataTransfer = self.spi.xfer2(data)
            # actual read data doesn't start until the 3rd byte
            dataTransfer = dataTransfer[2:] 
            
        

        return dataTransfer   

    def single_access_write(self, reg=0x00, regValue=0x0):
        """single_access_write, function to write to a single data register
        """
        
        if self.comMode == 'i2c':
            self.i2cBus.write_byte_data(self.i2cAddress, reg, regValue)
        else:
            rwBit = 0b0  # read/write bit set to write
            self.spi.xfer2([(rwBit<<7)+reg,regValue]) 
            

        return

    def single_access_read(self, reg=0x00):
        """single_access_read, function to read a single data register
        """
        
        if self.comMode == 'i2c':
            dataTransfer = self.i2cBus.read_byte_data(self.i2cAddress,reg)
        else:
            rwBit = 0b1  # read/write bit set to read
            dataTransfer = self.spi.xfer2([(rwBit<<7)+reg,0,0])
            # actual read data doesn't start until the 3rd byte
            dataTransfer = dataTransfer[2]
        
        return dataTransfer
    
    def twos_complement_conversion(self, num, bitsInNumber, leftJustification = 0, bitsRHS = 0):
        
        # remove any left justification
        num = num >> leftJustification

        signMask = (1<<(bitsInNumber-1)) | 0x0000
        signBit = (num & signMask)>>(bitsInNumber-1)

        #print("sign bit", signBit)

        if signBit == 1:  # negative number

            convMask = 0x0000
            for i in range(bitsInNumber-1):
                convMask = convMask + 2**i

            #print("conversion mask", hex(convMask))
                        
            x = num & convMask # strip off sign bit            
            x = x^convMask
            x = -(x + 1)
            #print("x", x)
        else: # positive number        
            x = num
            
        # shift the decimal place    
        decimalShift = 0x0001

        for i in range(bitsRHS):
            decimalShift = decimalShift + 2**i

        #print("decimal shift", decimalShift)

        x = x/decimalShift

        return x
    
    def get_calibration_parameters(self):
        """get_calibration_parameters, function to get the trimming coefficients
        from the NVM registers and convert to the form used in the output calculations"""
        
        nvmValues = self.multi_access_read(0x31, 21)
        
        # define sensor temperature calibration coefficients
        NVM_PAR_T1 = (nvmValues[1]<<8) + nvmValues[0]
        NVM_PAR_T2 = (nvmValues[3]<<8) + nvmValues[2]
        NVM_PAR_T3 = self.twos_complement_conversion(nvmValues[4], 8)
        
        # convert sensor temperature calibration coefficients
        # to floating point    
        self.PAR_T1 = NVM_PAR_T1/pow(2,-8)  #NVM_PAR_T1/2^-8
        self.PAR_T2 = NVM_PAR_T2/pow(2, 30)
        self.PAR_T3 = NVM_PAR_T3/pow(2, 48)
        
        # define sensor pressure calibration coefficients
        NVM_PAR_P1 = self.twos_complement_conversion((nvmValues[6]<<8)+nvmValues[5], 16)
        NVM_PAR_P2 = self.twos_complement_conversion((nvmValues[8]<<8)+nvmValues[7], 16)
        NVM_PAR_P3 = self.twos_complement_conversion(nvmValues[9], 8)
        NVM_PAR_P4 = self.twos_complement_conversion(nvmValues[10], 8)
        NVM_PAR_P5 = (nvmValues[12]<<8) + nvmValues[11]
        NVM_PAR_P6 = (nvmValues[14]<<8) + nvmValues[13]
        NVM_PAR_P7 = self.twos_complement_conversion(nvmValues[15], 8)
        NVM_PAR_P8 = self.twos_complement_conversion(nvmValues[16], 8)
        NVM_PAR_P9 = self.twos_complement_conversion((nvmValues[18]<<8)+nvmValues[17], 16)
        NVM_PAR_P10 = self.twos_complement_conversion(nvmValues[19], 8)
        NVM_PAR_P11 = self.twos_complement_conversion(nvmValues[20], 8)
        
        # convert sensor pressure calibration coefficients
        # to floating point    
        self.PAR_P1 = (NVM_PAR_P1 - pow(2,14))/pow(2,20)  #(NVM_PAR_P1-2^14)/2^20
        self.PAR_P2 = (NVM_PAR_P2 - pow(2,14))/pow(2,29)
        self.PAR_P3 = NVM_PAR_P3/pow(2,32)
        self.PAR_P4 = NVM_PAR_P4/pow(2,37)
        self.PAR_P5 = NVM_PAR_P5/pow(2,-3)
        self.PAR_P6 = NVM_PAR_P6/pow(2,6)
        self.PAR_P7 = NVM_PAR_P7/pow(2,8)
        self.PAR_P8 = NVM_PAR_P8/pow(2,15)
        self.PAR_P9 = NVM_PAR_P9/pow(2,48)
        self.PAR_P10 = NVM_PAR_P10/pow(2,48)
        self.PAR_P11 = NVM_PAR_P11/pow(2,65)
        
        return    
    def temperature_calc(self, sensorOutput):
        """temperature_calc, function to calculate the
        temperature (C) from the sensor output"""        
        
        # apply vendor formulas to obtain output temperature in C
        partialData1 = sensorOutput-self.PAR_T1
        partialData2 = partialData1*self.PAR_T2        
        
        temp = partialData2 + (partialData1 * partialData1)*self.PAR_T3
        
        return temp
    
    def pressure_calc(self, sensorOutput, tempOutput):
        """pressure_calc, function to calculate the
        pressure(hPa) from the sensor output"""   
        
        # apply vendor formulas to obtain output pressure in hPa
        partialData1 = self.PAR_P6 * tempOutput
        partialData2 = self.PAR_P7 * pow(tempOutput,2)
        partialData3 = self.PAR_P8 * pow(tempOutput,3)
        partialOut1 = self.PAR_P5 + partialData1 + partialData2 + partialData3
        
        partialData4 = self.PAR_P2 * tempOutput
        partialData5 = self.PAR_P3 * pow(tempOutput, 2)
        partialData6 = self.PAR_P4 * pow(tempOutput, 3)
        partialOut2 = sensorOutput * (self.PAR_P1 + partialData4 + partialData5 + partialData6)
        
        partialData8 = pow(sensorOutput,2)
        partialData9 = self.PAR_P9 + (self.PAR_P10 * tempOutput)
        partialData10 = partialData8 * partialData9
        partialOut3 = partialData10 + (pow(sensorOutput,3)*self.PAR_P11)
        
        press = (partialOut1 + partialOut2 + partialOut3)/100    
        
        return press
    
    def get_output(self):
        """get_output, function to read the sensor output from the
        temperature and pressure registers (0x04 to 0x09)"""
        
        rawOutput = self.multi_access_read(0x04,6) # read Temp & Press output registers           
        rawPressOut = (rawOutput[2]<<16) + (rawOutput[1]<<8) + (rawOutput[0])
        rawTempOut = (rawOutput[5]<<16) + (rawOutput[4]<<8) + (rawOutput[3])        

        temperature = self.temperature_calc(rawTempOut)
        pressure = self.pressure_calc(rawPressOut, temperature)
        
        return temperature, pressure
    
    def set_power_mode(self, mode = 'normal'):
        """set_power_mode, function to set the power mode of the sensor. Valid
        values are; sleep, force, normal. This sets bits 4 & 5 of PWR_CTRL
        register (0x1B)"""
        
        PWR_CTRL = self.single_access_read(0x1B)
        
        modeBits = 0b11 # default value: normal
        
        if mode == 'sleep':
            modeBits = 0b00
        elif mode == 'force':
            modeBits = 0b01        
        
        PWR_CTRL = PWR_CTRL & 0b11001111
        PWR_CTRL = PWR_CTRL | (modeBits<<4)        
        self.single_access_write(0x1B, PWR_CTRL)
        
        return
    
    def set_sensor_enables(self, t = 1, p = 1):
        """set_sensor_enables, function to enable/disable the temperature
        and pressure sensors. This sets bits 0 (temperature) & 1 (pressure) of
        PWR_CTRL register (0x1B)"""
        
        # 1 = sensor enabled
        # 0 = sensor disabled
        
        # Note: setting both sensors to disabled will put the entire
        # sensor into sleep mode        
        
        PWR_CTRL = self.single_access_read(0x1B)
        
        PWR_CTRL = PWR_CTRL & 0b11111100
        PWR_CTRL = PWR_CTRL | ((p<<1) + t)        
        self.single_access_write(0x1B, PWR_CTRL)
        
        return
    
    def set_odr(self, odrSc = 7):
        """set_odr, function to set the odr for the sensor. Values of 0-17
        are valid. This sets bits 0-4 of ODR register (0x1D)"""
        
        # ODR = 200/(odrSc+1) Hz
        
        if odrSc < 0:
            odrSc = 0
        elif odrSc > 17:
            odrSc = 17
        
        ODR = self.single_access_read(0x1D)
        
        ODR = ODR & 0b11100000
        ODR = ODR | odrSc
        self.single_access_write(0x1D, ODR)
        
        return
    
    def set_osr(self, t_osr = 1, p_osr = 1):
        """set_osr, function to set the oversampling rates for
        the temperature and pressue sensor. Vaild values for
        each are 1,2,4,8,16 & 32. This sets bits 0-2 (p_osr)
        and bits 3-5 (t_osr) of OSR register (0x1C)"""
        
        osrRates = {1:0b0, 2:0b1, 4:0b10, 8:0b11, 16:0b100, 32:0b101}
        
        t_osrBits = 0b0
        p_osrBits = 0b0
        
        if t_osr in osrRates:
            t_osrBits = osrRates[t_osr]
        
        if p_osr in osrRates:
            p_osrBits = osrRates[p_osr]
            
        OSR = self.single_access_read(0x1C)
        
        OSR = OSR & 0b11000000
        OSR = OSR | ((t_osrBits<<3) + p_osrBits)
        self.single_access_write(0x1C, OSR)        
        
        return
    
    def set_iir_filter(self, iirFilter = 0):
        """set_iir_filter, function to set the iir filter
        coefficient for the sensor. Valid values are 0,1,3,
        7,15,31,63,127. This sets bits 1-3 of CONFIG register
        (0x1F)"""
        
        # filter value of 0 the iiR filter is in bypass mode
        
        iirCoef = {0:0b0, 1:0b1, 3:0b10, 7:0b11, 15:0b100, 31:0b101,
                   63:0b110, 127:0b111}
        
        iirBits = 0b0
        
        if iirFilter in iirCoef:
            iirBits = iirCoef[iirFilter]
            
        CONFIG = self.single_access_read(0x1F)
        
        CONFIG = CONFIG & 0b11110001
        CONFIG = CONFIG | (iirBits<<1)
        self.single_access_write(0x1F, CONFIG)
        
        return
    
    def config_int_pin(self, outputMode = 'pushpull', level = 'high', latch = False):
        """config_int_pin, function to configure the sensor interrupt pin. OutputMode
        sets the pin to either pushpull or opendrain.  level sets the active level to
        either high or low. latch sets the pin to be either latching (True) or non-latching
        (False). This sets bits 0, 1 & 2 of INT_CTRL register (0x19)"""
        
        if outputMode == 'opendrain':
            odBit = 0b1
        else:
            odBit = 0b0
            
        if level == 'low':
            levelBit = 0b0
        else:
            levelBit = 0b1
            
        if latch == True:
            latchBit = 0b1
        else:
            latchBit = 0b0
            
        INT_CTRL = self.single_access_read(0x19)
        
        INT_CTRL = INT_CTRL & 0b11111000
        INT_CTRL = INT_CTRL | ((latchBit<<2) + (levelBit<<1) + odBit)
        self.single_access_write(0x19, INT_CTRL)
        
        return
    
    def set_interrupts(self, drdy = 0, fifoFull = 0, fifoWtm = 0):
        """set_interrupts, function to set which conditions trigger
        the sensor interrupt pin. the 3 conditions are; Data Ready, FIFO
        Full and FIFO Watermark. These conditions are ORed together.
        This sets bits 6, 4 & 3 of INT_CTRL register (0x19)"""
        
        # 1 = interrupt enabled
        # 0 = interrupt disabled                
        
        INT_CTRL = self.single_access_read(0x19)
        
        INT_CTRL = INT_CTRL & 0b10100111
        INT_CTRL = INT_CTRL | ((drdy<<6) + (fifoFull<<4) + (fifoWtm<<3))
        self.single_access_write(0x19, INT_CTRL)
        
        return
    
    def read_interrupt_status(self):
        """read_interrput_status, function to read the INT_STATUS
        register (0x11) and return the 3 components (drdy, fifoFull,
        fifoWtm) in a tuple"""
        
        # 1 = interrupt triggered
        # 0 = interrupt not triggered
        
        INT_STATUS = self.single_access_read(0x11)
        
        drdy = (INT_STATUS & 0b00001000)>>3
        fifoFull = (INT_STATUS & 0b00000010)>>1
        fifoWtm = INT_STATUS & 0b00000001
        
        return (drdy, fifoFull, fifoWtm)
    
    def set_fifo_mode(self, mode = 'on', fifoStopOnFull = 0, fifo_p = 1, fifo_t = 1, fifo_time = 1):
        """set_fifo_mode, function to turn the fifo mode on or off and enable/
        disable storing of pressure, temperature and/or time data in the fifo
        data buffer. It also enables/disables the fifo stop on full. This sets
        bits 0-4 of FIFO_CONFIG_1 register (0x17)"""
        
        # 1 = sensor enabled
        # 0 = sensor disabled
        
        if mode == 'off':
            modeBit = 0b0
        else:
            modeBit = 0b1
            
        FIFO_CONFIG_1 = self.single_access_read(0x17)
        FIFO_CONFIG_1 = FIFO_CONFIG_1 & 0b11100000
        FIFO_CONFIG_1 = FIFO_CONFIG_1 | ((fifo_t<<4) + (fifo_p<<3) + (fifo_time<<2) +
                                         (fifoStopOnFull<<1) + modeBit)
        self.single_access_write(0x17, FIFO_CONFIG_1)
        
        return
    
    def set_fifo_watermark(self, fifo_wtm = 1):
        """set_fifo_watermark. function to set the fifo watermark
        value. Valid values are between 1 and 511. This sets bits 0-7 of
        FIFO_WTM_0 (0x15) and bit 1 of FIFO_WTM_1 (0x16)"""
        
        if fifo_wtm < 1:
            fifo_wtm = 1
        elif fifo_wtm > 512:
            fifo_wtm = 512
            
        fifo0 = fifo_wtm & 0XFF
        fifo1 = (fifo_wtm & 0x100)>>8        
        
        #self.multi_access_write(0x15, [fifo0, fifo1]) - isn't working
        self.single_access_write(0x15, fifo0)
        self.single_access_write(0x16, fifo1)
        
        return
    
    def set_fifo_data_options(self, subsampling = 0, iirFilter = 0):
        """set_fifo_data_options, function to set the FIFO downsampling
        value, valid values between 0 and 7 corresponding to downsampling
        of 2^subsamping for range of 1-128. This also sets whether unfiltered
        or iiR filter data is stored in FIFO. This sets bits 0-2 and 3&4 of
        FIFO_CONFIG_2 register (0x18)"""
        
        # iiRFilter = 0 unfiltered data stored in FIFO
        # iiRFilter = 1 filtered data stored in FIFO
        
        if subsampling < 0:            
            subsampling = 0
        elif subsampling > 7:
            subsampling = 7
            
        FIFO_CONFIG_2 = self.single_access_read(0x18)
        FIFO_CONFIG_2 = FIFO_CONFIG_2 & 0b11100000
        FIFO_CONFIG_2 = FIFO_CONFIG_2 | ((iirFilter<<3) + subsampling)
        self.single_access_write(0x18, FIFO_CONFIG_2)
        
        return
    
    def read_bytes_in_fifo(self):
        """read_bytes_in_fifo, function to read the number of
        bytes currently in the fifo data buffer. This reads
        FIFO_LENGTH0 (0x12) and FIFO_LENGTH1 (0x13)"""
        
        fifoLength = bmp388.multi_access_read(0x12,2)
        fifoLength = ((fifoLength[1] & 0b1)<<8) + fifoLength[0]
        
        return fifoLength
    
    def clear_fifo(self):
        """clear_fifo, function to clear/flush the contents of
        the FIFO data buffer. This writes 0xB0 to the CMD register
        (0x7E)"""
        
        self.single_access_write(0x7E, 0xB0)  # flush fifo
        
        return
        
    def read_fifo(self, numBytes, timeEnabled = False):
        """read_fifo, function to read numBytes of data out of
        the fifo data buffer"""   
        
        results = []        
        while len(results) < numBytes:
            fifoReading = self.multi_access_read(0x14,28)
            # check header frame and if not FIFO empty frame
            # add to results list
            if fifoReading[0] != 0x80:
                results.extend(fifoReading)
                
        # process results
        temperatureResults = []
        pressureResults = []
        timeResults = []
        
        i = 0
        while i < len(results):            
            # evaluate header
            if results[i] == 0x44:
                # error frame
                print('FIFO Config Error', end='')
                if (i+1) < len(results):
                    print(bin(results[i]+1))
                i += 2                
            elif results[i] == 0x94:
                # temperature and pressure frame
                if (i+6) < len(results):
                    rawTempOut = (results[i+3]<<16) + (results[i+2]<<8) + (results[i+1])
                    rawPressOut = (results[i+6]<<16) + (results[i+5]<<8) + (results[i+4])                
                    temperature = self.temperature_calc(rawTempOut)
                    pressure = self.pressure_calc(rawPressOut, temperature)                
                    temperatureResults.append(temperature)
                    pressureResults.append(pressure)
                i += 7                
            elif results[i] == 0x84:
                # pressure only frame, only return raw
                # pressure out since no temperature available
                if (i+3) < len(results):
                    rawPressOut = (results[i+3]<<16) + (results[i+2]<<8) + (results[i+1])                
                    pressureResults.append(rawPressOut)
                i += 4            
            elif results[i] == 0x90:
                # temperature only frame                               
                if (i+3) < len(results):                    
                    rawTempOut = (results[i+3]<<16) + (results[i+2]<<8) + (results[i+1])                    
                    temperature = self.temperature_calc(rawTempOut)
                    temperatureResults.append(temperature)
                i += 4
            elif results[i] == 0xA0:
                # time frame                
                if (i+3) < len(results):
                    timeData = (results[i+3]<<16) + (results[i+2]<<8) + (results[i+1])                
                    timeResults.append(timeData)
                i += 4
            elif results[i] == 0x80:
                # empty frame
                i += 2
            else:
                # unknown frame
                #print('unknown frame', end = ' ') # for testing
                #print(str(bin(results[i]))) # for testing               
                i += 1
        
        if timeEnabled == True:
            results = self.multi_access_read(0x14,4)
            if results[0] == 0xA0:
                # time frame            
                timeData = (results[3]<<16) + (results[2]<<8) + (results[1])                
                timeResults.append(timeData)        
        
        if timeEnabled == True:
            return temperatureResults, pressureResults, timeResults
        else:
            return temperatureResults, pressureResults
    
    def sensor_reset(self):
        """sensor_reset, function to do a soft reset of the sensor
        which sets configuration settings back to default values and
        clears/flushes the contents of the FIFO data buffer. This
        writes 0xB6 to the CMD register (0x7E)"""
        
        self.single_access_write(0x7E, 0xB6)  # soft reset        
        
        return
    
if __name__ == "__main__":
    
    # simple sensor set up and output reading example    
    
    # uncomment line below if using SPI
    #bmp388 = BMP388('spi', spiPort = 0, spiCS = 1)
    
    # uncomment the 2 lines below if using i2c
    i2cAddress = 0x76
    bmp388 = BMP388('i2c', i2cAddress)  
    
    # read chip id register - should return 0x50
    print(f'Reading Chip ID register (should output 0x50): {hex(bmp388.single_access_read(0x00))}')     
    bmp388.set_sensor_enables(t = 1, p = 1)     # enable T & P sensors
    bmp388.set_power_mode('normal')             # set power mode to normal - set after enabling sensors
    bmp388.set_odr(0x07)                        # set ODR to 25/18 Hz (640ms)
    bmp388.set_osr(t_osr = 1, p_osr = 4)        # set t_OSR x1, p_OSR x4
    bmp388.set_iir_filter(0)                    # set iiR filter coeff to 0
    bmp388.config_int_pin(outputMode = 'pushpull', level = 'high', latch = False)
    bmp388.set_interrupts(drdy = 0, fifoFull = 0, fifoWtm = 0)    
    
    for i in range(5):
        temperature, pressure = bmp388.get_output()
        print(f'Temperature: {temperature:.2f}C Pressure: {pressure:.2f}hPa')
        time.sleep(1)