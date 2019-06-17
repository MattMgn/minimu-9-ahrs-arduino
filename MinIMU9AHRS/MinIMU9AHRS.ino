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
int SENSOR_SIGN[9] = { 1,  1,  1,
                      -1, -1, -1,
                       1,  1,  1};

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

#include <Timer.h>
#include <Wire.h>

// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY             256  //this equivalent to 1G in the raw data coming from the accelerometer
#define NULL_VECTOR         {0, 0, 0}

#define ToRad(x)            ((x) * 0.01745329252)  // *pi/180
#define ToDeg(x)            ((x) * 57.2957795131)  // *180/pi

#define FREQUENCY_UPDATE    20  // [ms]
#define FREQUENCY_COMPASS   100 // [ms] -> 10Hz

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X         0.07 //X axis Gyro gain
#define Gyro_Gain_Y         0.07 //Y axis Gyro gain
#define Gyro_Gain_Z         0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x)    ((x)*ToRad(Gyro_Gain_X))
#define Gyro_Scaled_Y(x)    ((x)*ToRad(Gyro_Gain_Y))
#define Gyro_Scaled_Z(x)    ((x)*ToRad(Gyro_Gain_Z))

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board

#define M_X_MIN             -1000
#define M_Y_MIN             -1000
#define M_Z_MIN             -1000
#define M_X_MAX             +1000
#define M_Y_MAX             +1000
#define M_Z_MAX             +1000

#define Kp_ROLLPITCH        0.02
#define Ki_ROLLPITCH        0.00002
#define Kp_YAW              1.2
#define Ki_YAW              0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE          1

#define PRINT_DCM           0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS       0 //Will print the analog raw data
#define PRINT_EULER         1   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED          13

float dt = 1.0f / FREQUENCY_UPDATE;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer = 0;   //general purpuse timer
long timer_old;
long timer24 = 0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6] = {0, 0, 0, 0, 0, 0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3] = NULL_VECTOR; //Store the acceleration in a vector
float Gyro_Vector[3] = NULL_VECTOR;//Store the gyros turn rate in a vector
float Omega_Vector[3] = NULL_VECTOR; //Corrected Gyro_Vector data
float Omega_P[3] = NULL_VECTOR;//Omega Proportional correction
float Omega_I[3] = NULL_VECTOR;//Omega Integrator
float Omega[3] = NULL_VECTOR;

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3] = NULL_VECTOR;
float errorYaw[3] = NULL_VECTOR;

unsigned int counter = 0;
byte gyro_sat = 0;

float DCM_Matrix[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};

float Update_Matrix[3][3] = {
    {0, 1, 2},
    {3, 4, 5},
    {6, 7, 8}
};

float Temporary_Matrix[3][3] = {
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0}
};


Timer _frequency_rate(FREQUENCY_UPDATE);
Timer _frequency_compass(FREQUENCY_COMPASS);


void setup()
{
    Serial.begin(115200);
    pinMode (STATUS_LED, OUTPUT);
    
    I2C_Init();
    
    Serial.println("Pololu MinIMU-9 + Arduino AHRS");

    /* define tasks frequency */
    _frequency_rate.start((unsigned long) millis());
    _frequency_compass.start((unsigned long) millis());

    
    digitalWrite(STATUS_LED, LOW);
    delay(1500);
    
    Accel_Init();
    Compass_Init();
    Gyro_Init();
    
    delay(20);

    /* Compute bias */
    for(int i = 0; i < 32; i++){
        Read_Gyro();
        Read_Accel();

        for(int y = 0; y < 6; y++)
            AN_OFFSET[y] += AN[y];
        delay(20);
    }
    
    for(int y = 0; y < 6; y++)
        AN_OFFSET[y] = AN_OFFSET[y]/32;
    
    AN_OFFSET[5] -= GRAVITY*SENSOR_SIGN[5];

    Serial.println("Offset:");
    for(int y = 0; y < 6; y++)
        Serial.println(AN_OFFSET[y]);
    
    delay(2000);
    digitalWrite(STATUS_LED,HIGH);

    timer = millis();
    delay(20);
}

void loop()
{

    if(_frequency_compass.delay(millis())) {
            Read_Compass();    // Read I2C magnetometer
            Compass_Heading(); // Calculate magnetic heading
    }

    if(_frequency_rate.delay(millis())) {
        timer_old = timer;
        timer = millis();
        dt = (timer - timer_old) / 1000.0;
    
        // *** DCM algorithm
        // Data adquisition
        Read_Gyro();   // This read gyro data
        Read_Accel();     // Read I2C accelerometer

        // Calculations...
        Matrix_update();
        Normalize();
        Drift_correction();
        Euler_angles();
    
        printdata();
    }



}
