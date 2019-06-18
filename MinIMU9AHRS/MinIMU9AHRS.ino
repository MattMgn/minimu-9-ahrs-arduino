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

// Uncomment the following line to use a MinIMU-9 v5 or AltIMU-10 v5. Leave commented for older IMUs (up through v4).
//#define IMU_V5

// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the right
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
float SENSOR_SIGN[9] = { 1.0f,  1.0f,  1.0f,
                        -1.0f, -1.0f, -1.0f,
                         1.0f,  1.0f,  1.0f};

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

#include <Timer.h>
#include <Wire.h>
#include <MadgwickAHRS.h>


#define GRAVITY             9.81                   //  [m/s^Z]

#define DEG_TO_RAD          0.01745329252
#define RAD_TO_DEG          57.2957795131
#define G_TO_MS2            1.0 / GRAVITY

#define FREQUENCY_ESTIMATOR 20  // [ms]
#define FREQUENCY_PRINT     250 // [ms]

#define BIAS_MEASURE_LENGTH 1000

/* Accelero Scale */
// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
// this equivalent to 1G in the raw data coming from the accelerometer
#define ACC_SCALE           1.0 / 256.0

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define GYRO_SCALE          0.07 * DEG_TO_RAD

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board

#define M_X_MIN             -1000
#define M_Y_MIN             -1000
#define M_Z_MIN             -1000
#define M_X_MAX             +1000
#define M_Y_MAX             +1000
#define M_Z_MAX             +1000

#define PRINT_BIAS          1
#define PRINT_RAW_DATA      0
#define PRINT_DATA          1

#define FLOATING_PRECISION  3

#define STATUS_LED          13

float dt;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

unsigned long timer = 0;    // ms
unsigned long prev_timer;   // ms

/* Raw values from sensors */
float gyro_raw[3];          // rad/s
float acc_raw[3];           // m/s^2
float mag_raw[3];

/* Filtered and unbiased values */
float gyro[3];          // rad/s
float acc[3];           // m/s^2
float mag[3];

float angle_est[3];

float gyro_bias[3];
float acc_bias[3];

Madgwick estimator;

Timer _frequency_estimator(FREQUENCY_ESTIMATOR);
Timer _frequency_print(FREQUENCY_PRINT);


void setup()
{
    Serial.begin(115200);

    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    I2CInit();
    
    Serial.println("Pololu MinIMU-9 + Arduino AHRS");

    /* define tasks frequency */
    _frequency_estimator.start((unsigned long) millis());
    _frequency_print.start((unsigned long) millis());
    
    delay(1500);
    
    /* Sensors initialization */
    SensorsInit();
    MagnetoInit();
    GyroInit();
    
    delay(20);

    /* Compute bias */
    unsigned ts_start_bias = millis();
    for (int i = 0; i < BIAS_MEASURE_LENGTH; i++) {
        /* Gyro bias */
        ReadGyro();
        for (int i = 0; i < 3; i++) {
            gyro_bias[i] += gyro_raw[i];
        }
        /* Accelero bias */
        ReadAccelero();
        for (int i = 0; i < 3; i++) {
            acc_bias[i] += acc_raw[i];
        }
    }
    for (int i = 0; i < 3; i++) {
        gyro_bias[i] = gyro_bias[i] / (BIAS_MEASURE_LENGTH);
        acc_bias[i] = acc_bias[i] / (BIAS_MEASURE_LENGTH);
    }
    acc_bias[2] += GRAVITY;

    if (PRINT_BIAS) {
        Serial.println("Gyro bias:");
        for (int i = 0; i < 3; i++) {
            Serial.print(gyro_bias[i] * RAD_TO_DEG, FLOATING_PRECISION);
            Serial.println(" deg/s");
        }
        Serial.println("Accelero bias:");
        for (int i = 0; i < 3; i++) {
            Serial.print(acc_bias[i] * G_TO_MS2, FLOATING_PRECISION);
            Serial.println(" m/s");
        }
    }

    
    delay(500);
    digitalWrite(STATUS_LED, HIGH);

    prev_timer = millis();
}

void loop()
{
    if(_frequency_estimator.delay(millis())) {
        /* Read sensors */
        ReadGyro();
        ReadAccelero();
        ReadMagneto();

        /* Remove bias */
        for (int i = 0; i < 3; i++) {
            gyro[i] = gyro_raw[i] - gyro_bias[i];
            acc[i] = acc_raw[i] - acc_bias[i];
        }

        timer = millis();
        dt = (float)(timer - prev_timer) / 1000.0f;
        prev_timer = timer;
    }

    if(_frequency_print.delay(millis())) {
        printdata();
    }

}
