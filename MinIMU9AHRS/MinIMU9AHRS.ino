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

/* Uncomment this line for Arduino Due (must be placed before ros.h */
#define USE_USBCON

/*
 * Enable ROS imu topic publishing
 * run the following command to get acces to topic:
 *      rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200
 * WARNING: not enough dynamic memory to be activated on Arduino Nano, but
 * succesfully tested on Arduino Mega and Due
 */

//#define MINIMU_ROS      // disable ROS imu topic by commenting this line
//#define RAW_IMU_ROS     // disable raw imu topic by commenting this line

/*
 * Uncomment the following line to use a MinIMU-9 v5 or AltIMU-10 v5.
 * Leave commented for older IMUs (up through v4).
 */
//#define IMU_V5

// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
float SENSOR_SIGN[9] = { 1.0f,  1.0f,  1.0f,
                        -1.0f, -1.0f, -1.0f,
                         1.0f,  1.0f,  1.0f};

#include <Timer.h>
#include <Wire.h>
#include <MadgwickAHRS.h>

#ifdef MINIMU_ROS
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#endif


#define GRAVITY             9.81                   //  [m/s^2]

#define G_TO_MS2            GRAVITY

#define FREQUENCY_ESTIMATOR 100       // [ms]
#define FREQUENCY_PRINT     100     // [ms]
#define FREQUENCY_TOPIC     200     // [ms]

#define BIAS_MEASURE_LENGTH 1000

/* Accelero Scale */
// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
// this equivalent to 1G in the raw data coming from the accelerometer
#define ACC_SCALE           1.0 / 256.0

/* Magneto Scale */
// 1.3 gauss on 2047 LSB
#define MAGNETO_SCALE       1.3 / 2047

// gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define GYRO_SCALE          0.07 * DEG_TO_RAD

/* Filter parameters */
#define betaDef         1.0f
#define zetaDef         0.00005f

#define PRINT_BIAS          0
#define PRINT_DATA_RAW      0
#define PRINT_DATA          0
#define PRINT_EULER_ANGLES  0

#define FLOATING_PRECISION  3

#define STATUS_LED          13

float dt;

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

float gyro_bias[3];
float acc_bias[3];

float angle_est[3];
float quat_est[4];

Madgwick estimator;

Timer _frequency_estimator(FREQUENCY_ESTIMATOR);
Timer _frequency_print(FREQUENCY_PRINT);

#ifdef MINIMU_ROS
Timer _frequency_topic(FREQUENCY_TOPIC);
ros::NodeHandle  nh;
unsigned long seq = 0;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_topic("/imu", &imu_msg);
#ifdef RAW_IMU_ROS
sensor_msgs::Imu imu_raw_msg;
ros::Publisher imu_raw_topic("/imu_raw", &imu_raw_msg);
#endif

#endif

void setup()
{
#ifdef MINIMU_ROS
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(imu_topic);
    nh.loginfo("Starting /imu topic");
#ifdef RAW_IMU_ROS
    nh.advertise(imu_raw_topic);
    nh.loginfo("Starting /imu_raw topic");
#endif
#else
    Serial.begin(115200);
#endif

    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    /* define tasks frequency */
    _frequency_estimator.start((unsigned long) millis());
    _frequency_print.start((unsigned long) millis());
#ifdef MINIMU_ROS
    _frequency_topic.start((unsigned long) millis());
#endif

    /* Sensors initialization */
    I2CInit();
    delay(1500);
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
    acc_bias[2] -= 1.0; // g

#ifdef MINIMU_ROS
#else
    if (PRINT_BIAS) {
        Serial.println("Initial gyro bias:");
        for (int i = 0; i < 3; i++) {
            Serial.print(gyro_bias[i] * RAD_TO_DEG, FLOATING_PRECISION);
            Serial.println(" deg/s");
        }
        Serial.println("Initial accelero bias:");
        for (int i = 0; i < 3; i++) {
            Serial.print(acc_bias[i] * G_TO_MS2, FLOATING_PRECISION);
            Serial.println(" m/s");
        }
    }
#endif

    /* Initialize estimator */
    estimator.begin(betaDef, zetaDef);
    
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

        /* Compute estimated attitude */
        timer = millis();
        dt = (float)(timer - prev_timer) / 1000.0f;
        prev_timer = timer;
        estimator.update(dt, gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], mag_raw[0], mag_raw[1], mag_raw[2], &gyro_bias[0], &gyro_bias[1], &gyro_bias[2]);
    }

    if(_frequency_print.delay(millis())) {
        PrintData();
    }

#ifdef MINIMU_ROS
    if(_frequency_topic.delay(millis())) {
        estimator.getQuaternion(quat_est);
        
        seq++;
        imu_msg.header.seq = seq;
        imu_msg.header.stamp = nh.now();
        imu_msg.header.frame_id = "imu";

        imu_msg.orientation.x = quat_est[0];
        imu_msg.orientation.y = quat_est[1];
        imu_msg.orientation.z = quat_est[2];
        imu_msg.orientation.w = quat_est[3];
 
        imu_msg.angular_velocity.x = gyro[0];
        imu_msg.angular_velocity.y = gyro[1];
        imu_msg.angular_velocity.z = gyro[2];

        imu_msg.linear_acceleration.x = acc[0];
        imu_msg.linear_acceleration.y = acc[1];
        imu_msg.linear_acceleration.z = acc[2];

#ifdef RAW_IMU_ROS
        imu_raw_msg.header.seq = seq;
        imu_raw_msg.header.stamp = nh.now();
        imu_raw_msg.header.frame_id = "imu_raw";

        /* orientation section is used for magnetometer values */
        imu_raw_msg.orientation.x = mag_raw[0];
        imu_raw_msg.orientation.y = mag_raw[1];
        imu_raw_msg.orientation.z = mag_raw[2];
        imu_raw_msg.orientation.w = 0.0;

        imu_raw_msg.angular_velocity.x = gyro_raw[0];
        imu_raw_msg.angular_velocity.y = gyro_raw[1];
        imu_raw_msg.angular_velocity.z = gyro_raw[2];

        imu_raw_msg.linear_acceleration.x = acc_raw[0];
        imu_raw_msg.linear_acceleration.y = acc_raw[1];
        imu_raw_msg.linear_acceleration.z = acc_raw[2];

        /* publish message */
        imu_raw_topic.publish(&imu_raw_msg);
#endif
        imu_topic.publish(&imu_msg);
        nh.spinOnce();
    }
#endif

}
