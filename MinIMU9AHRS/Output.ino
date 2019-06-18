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

void printdata()
{   
      Serial.print("!");

      if (PRINT_RAW_DATA) {
            Serial.print("gyro_raw:");
            Serial.print(gyro_raw[0] * RAD_TO_DEG, FLOATING_PRECISION);
            Serial.print(",");
            Serial.print(gyro_raw[1] * RAD_TO_DEG, FLOATING_PRECISION);
            Serial.print(",");
            Serial.print(gyro_raw[2] * RAD_TO_DEG, FLOATING_PRECISION);
            Serial.print(",");
            Serial.print("accelero_raw:");
            Serial.print (",");
            Serial.print(acc_raw[0], FLOATING_PRECISION);
            Serial.print (",");
            Serial.print(acc_raw[1], FLOATING_PRECISION);
            Serial.print (",");
            Serial.print(acc_raw[2], FLOATING_PRECISION);
            Serial.print (",");
            Serial.print("mag_raw:");
            Serial.print(mag_raw[0], FLOATING_PRECISION);
            Serial.print (",");
            Serial.print(mag_raw[1], FLOATING_PRECISION);
            Serial.print (",");
            Serial.print(mag_raw[2], FLOATING_PRECISION);
            Serial.print (",");
      }

      if (PRINT_DATA) {
            Serial.print("gyro:");
            Serial.print(gyro[0] * RAD_TO_DEG, FLOATING_PRECISION);
            Serial.print(",");
            Serial.print(gyro[1] * RAD_TO_DEG, FLOATING_PRECISION);
            Serial.print(",");
            Serial.print(gyro[2] * RAD_TO_DEG, FLOATING_PRECISION);
            Serial.print(",");
            Serial.print("accelero:");
            Serial.print(acc[0] * G_TO_MS2, FLOATING_PRECISION);
            Serial.print (",");
            Serial.print(acc[1] * G_TO_MS2, FLOATING_PRECISION);
            Serial.print (",");
            Serial.print(acc[2] * G_TO_MS2, FLOATING_PRECISION);
            Serial.print (",");
            Serial.print("mag:");
            Serial.print(mag[0], FLOATING_PRECISION);
            Serial.print (",");
            Serial.print(mag[1], FLOATING_PRECISION);
            Serial.print (",");
            Serial.print(mag[2], FLOATING_PRECISION);
            Serial.print (",");
      }

      Serial.println();
      
}
