/*

MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

Copyright (c) 2011-2016 Pololu Corporation.
http://www.pololu.com/

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.

*/

#ifdef IMU_V5
#include <LSM6.h>
#include <LIS3MDL.h>
LSM6 lsm6_sensor;
LIS3MDL lis3mdl_sensor;
#else // older IMUs through v4
#include <L3G.h>
#include <LSM303.h>
L3G l3g_sensor;
LSM303 lsm303_sensor;
#endif

void I2CInit()
{
    Wire.begin();
}

void GyroInit()
{
#ifdef IMU_V5
    // Sensors_Init() should have already called lsm6_sensor.init() datad enableDefault()
    lsm6_sensor.writeReg(LSM6::CTRL2_G, 0x4C); // 104 Hz, 2000 dps full scale
#else
    l3g_sensor.init();
    l3g_sensor.enableDefault();
    l3g_sensor.writeReg(L3G::CTRL_REG4, 0x20); // 2000 dps full scale
    l3g_sensor.writeReg(L3G::CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
#endif
}

void SensorsInit() {
#ifdef IMU_V5
    lsm6_sensor.init();
    lsm6_sensor.enableDefault();
    lsm6_sensor.writeReg(LSM6::CTRL1_XL, 0x3C); // 52 Hz, 8 g full scale
#else
    lsm303_sensor.init();
    lsm303_sensor.enableDefault();
    switch (lsm303_sensor.getDeviceType()) {
    case LSM303::device_D:
        lsm303_sensor.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
        break;
    case LSM303::device_DLHC:
        lsm303_sensor.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
        break;
    default: // DLM, DLH
        lsm303_sensor.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
  }
#endif
}

void MagnetoInit() {
#ifdef IMU_V5
    lis3mdl_sensor.init();
    lis3mdl_sensor.enableDefault();
#else
    // LSM303: doesn't need to do dataything because Sensors_Init() should have already called lsm303_sensor.enableDefault()
#endif
}

float *ReadGyro() {
    float data[3];
    float output[3];

#ifdef IMU_V5
    lsm6_sensor.readGyro();

    data[0] = lsm6_sensor.g.x;
    data[1] = lsm6_sensor.g.y;
    data[2] = lsm6_sensor.g.z;
#else
    l3g_sensor.read();

    data[0] = l3g_sensor.g.x;
    data[1] = l3g_sensor.g.y;
    data[2] = l3g_sensor.g.z;
#endif

    output[0] = SENSOR_SIGN[0] * data[0];
    output[1] = SENSOR_SIGN[1] * data[1];
    output[2] = SENSOR_SIGN[2] * data[2];

    return output;
}

float *ReadAccelero() {
    float data[3];
    float output[3];

#ifdef IMU_V5
    lsm6_sensor.readAcc();

    data[3] = lsm6_sensor.a.x >> 4; // shift right 4 bits to use 12-bit representation (1 g = 256)
    data[4] = lsm6_sensor.a.y >> 4;
    data[5] = lsm6_sensor.a.z >> 4;
#else
    lsm303_sensor.readAcc();

    data[3] = lsm303_sensor.a.x >> 4; // shift right 4 bits to use 12-bit representation (1 g = 256)
    data[4] = lsm303_sensor.a.y >> 4;
    data[5] = lsm303_sensor.a.z >> 4;
#endif

    output[0] = SENSOR_SIGN[3] * data[3];
    output[1] = SENSOR_SIGN[4] * data[4];
    output[2] = SENSOR_SIGN[5] * data[5];

    return output;
}

float *ReadMagneto() {
    float data[3];
    float output[3];

#ifdef IMU_V5
    lis3mdl_sensor.read();

    data[0] = SENSOR_SIGN[6] * lis3mdl_sensor.m.x;
    data[1] = SENSOR_SIGN[7] * lis3mdl_sensor.m.y;
    data[2] = SENSOR_SIGN[8] * lis3mdl_sensor.m.z;
#else
    lsm303_sensor.readMag();

    data[0] = SENSOR_SIGN[6] * lsm303_sensor.m.x;
    data[1] = SENSOR_SIGN[7] * lsm303_sensor.m.y;
    data[2] = SENSOR_SIGN[8] * lsm303_sensor.m.z;
#endif
    output[0] = SENSOR_SIGN[6] * data[6];
    output[1] = SENSOR_SIGN[7] * data[7];
    output[2] = SENSOR_SIGN[8] * data[8];

    return output;
}

