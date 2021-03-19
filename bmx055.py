#!/usr/bin/python3
import smbus2
import time
import numpy as np

class bmx055:
    def __init__(self, accl_addr=0x19, gyro_addr=0x69, mag_addr=0x13, 
                accl_reg=0x02, gyro_reg=0x02, mag_reg=0x42):
        self.i2c = smbus2.SMBus(1)
        self.accl_addr = accl_addr
        self.gyro_addr = gyro_addr
        self.mag_addr = mag_addr
        self.accl_reg = accl_reg
        self.gyro_reg = gyro_reg
        self.mag_reg = mag_reg

        self.accl_tmp = np.zeros(6, dtype=int)
        self.gyro_tmp = np.zeros(6, dtype=int)
        self.mag_tmp = np.zeros(8, dtype=int)
    
        self.g = 0.00098 # For Accl range +/- 2g
        self.s = 0.0038 # For Gyro range +/- 125 degree/s


        # Init Accl
        self.i2c.write_byte_data(self.accl_addr, 0x0F, 0x03) # Range = +/- 2g
        time.sleep(0.1)
        self.i2c.write_byte_data(self.accl_addr, 0x10, 0x08) # Bandwidth = 7.81 Hz
        time.sleep(0.1)
        self.i2c.write_byte_data(self.accl_addr, 0x11, 0x00) # Normal mode, Sleep duration = 0.5ms
        time.sleep(0.1)
        
        # Init Gyro
        self.i2c.write_byte_data(self.gyro_addr, 0x0F, 0x04) # Full scale = +/- 125 degree/s
        time.sleep(0.1)
        self.i2c.write_byte_data(self.gyro_addr, 0x10, 0x07) # ODR = 100 Hz
        time.sleep(0.1)
        self.i2c.write_byte_data(self.gyro_addr, 0x11, 0x00) # Normal mode, Sleep duration = 2ms
        time.sleep(0.3)

        # Init Mag
        self.i2c.write_byte_data(self.mag_addr, 0x4B, 0x83) # Soft reset
        time.sleep(0.1)
        self.i2c.write_byte_data(self.mag_addr, 0x4B, 0x01) # Soft reset
        time.sleep(0.1)
        self.i2c.write_byte_data(self.mag_addr, 0x4C, 0x00) # Normal mode, ODR = 10Hz
        time.sleep(0.1)
        self.i2c.write_byte_data(self.mag_addr, 0x4E, 0x84) # X, Y, Z-Axis enabled
        time.sleep(0.1)
        self.i2c.write_byte_data(self.mag_addr, 0x51, 0x04) # No. of Repetitions for X-Y Axis = 9
        time.sleep(0.1)
        self.i2c.write_byte_data(self.mag_addr, 0x52, 0x16) # No. of Repetitions for Z-Axis = 15
        time.sleep(0.1)
        

    def __del__(self):
        self.i2c.write_byte_data(self.mag_addr, 0x4B, 0x00) # Suspend mode
        time.sleep(0.1)
        self.i2c.close()
    
    def read_accl(self):
        for i in range(6):
            self.accl_tmp[i] = self.i2c.read_byte_data(self.accl_addr, self.accl_reg+i)
        x = self.conv_accl_12bit(self.accl_tmp[0], self.accl_tmp[1])
        y = self.conv_accl_12bit(self.accl_tmp[2], self.accl_tmp[3])
        z = self.conv_accl_12bit(self.accl_tmp[4], self.accl_tmp[5])

        x = x * self.g
        y = y * self.g
        z = z * self.g

        return x, y, z
    
    def conv_accl_12bit(self, lsb, msb):
        val = (msb * 256) + ((lsb & 0xF0) / 16)
        if val > 2047:
            val = val - 4096
        return val

    def read_gyro(self):
        for i in range(6):
            self.gyro_tmp[i] = self.i2c.read_byte_data(self.gyro_addr, self.gyro_reg+i)
        x = self.conv_gyro_12bit(self.gyro_tmp[0], self.gyro_tmp[1])
        y = self.conv_gyro_12bit(self.gyro_tmp[2], self.gyro_tmp[3])
        z = self.conv_gyro_12bit(self.gyro_tmp[4], self.gyro_tmp[5])

        x = x * self.s
        y = y * self.s
        z = z * self.s
        return x, y, z

    def conv_gyro_12bit(self, lsb, msb):
        val = (msb * 256) + lsb
        if val > 32767:
            val = val - 65546
        return val

    def read_mag(self):
        for i in range(8):
            self.mag_tmp[i] = self.i2c.read_byte_data(self.mag_addr, self.mag_reg+i)
        
        x = (self.mag_tmp[1] << 8) | (self.mag_tmp[0] >> 3)
        if x > 4095:
            x = x - 8192

        y = (self.mag_tmp[3] << 8) | (self.mag_tmp[2] >> 3)
        if y > 4095:
            y = y - 8192
        
        z = (self.mag_tmp[5] << 8) | (self.mag_tmp[4] >> 3)
        if z > 16383:
            z = z -  32768

        return x, y, z

    def read_sensor(self):
        accl = self.read_accl()
        gyro = self.read_gyro()
        mag = self.read_mag()
        return accl, gyro, mag

if __name__ == "__main__":
    bmx_opener = bmx055()
    data_list = []
    csv_output_flag = True

    for i in range(2000):
        read_data = bmx_opener.read_sensor()
        data_list.append(np.array(read_data))

    data_array = np.array(data_list)
    accl_array = np.round(data_array[:,0,:], decimals=3)
    gyro_array = np.round(data_array[:,1,:], decimals=3)
    mag_array = np.round(data_array[:,2,:], decimals=3)

    if(csv_output_flag):
        # accl
        np.savetxt('accl.csv', accl_array, delimiter=',')
        # gyro
        np.savetxt('gyro.csv', gyro_array, delimiter=',')
        # mag
        np.savetxt('mag.csv', mag_array, delimiter=',')
    else:
        print(accl_array)
        print(gyro_array)
        print(mag_array)

    print(data_array.shape)


