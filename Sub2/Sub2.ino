/*
 *  Sub1.ino - MP Example to communicate message data
 *  Copyright 2019 Sony Semiconductor Solutions Corporation
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#if (SUBCORE != 2)
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include <BMI160Gen.h>


#define MSGLEN      64
#define MY_MSGID    20

/* Const values */

const int i2c_addr                = 0x68;
const int baudrate                = 115200;

const int walking_stride          = 60; /* 60cm */
const int running_stride          = 80; /* 80cm */

const int gyro_range             = 2; /* +/- 2000 degrees/second */
const int gyro_rate              = 200; /* 50 Hz */
const int accel_range             = 2; /* 2G */
const int accel_rate              = 200; /* 50 Hz */
const int accel_sample_num        = 100; /* 50 sample */
const int accel_sample_size       = sizeof(float) * 3;

struct MyPacket {
  volatile int status; /* 0:ready, 1:busy */
  char message[MSGLEN];
};

struct BMI160Packet {
  volatile int status; /* 0:ready, 1:busy */
  char ax[MSGLEN];
  char ay[MSGLEN];
  char az[MSGLEN];
  char gx[MSGLEN];
  char gy[MSGLEN];
  char gz[MSGLEN];
};


MyPacket packet;
BMI160Packet sensor_packet;

uint32_t  currentTime;
uint32_t  loopTimer=0;

uint8_t oversampling = 7;

void setup()
{
  int ret = 0;
  memset(&packet, 0, sizeof(packet));
  ret = MP.begin();
  if (ret < 0) {
    errorLoop(2);
  }

  Serial.begin(baudrate);
  while (!Serial);

  /* Initialize device. */
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);

   /* Set device setting */
  BMI160.setGyroRange(gyro_range);
  BMI160.setGyroRate(gyro_rate);
  BMI160.setAccelerometerRange(accel_range);
  BMI160.setAccelerometerRate(200);

  BMI160.autoCalibrateGyroOffset();
  BMI160.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

  
  currentTime = micros();
  loopTimer = micros();
}

void loop()
{
  while (currentTime - loopTimer <= 10000)currentTime = micros();
  loopTimer = currentTime;
//  MPLog("loop\n");
  float gx, gy, gz;
  float ax, ay, az;
  
  /* Read raw accelerometer measurements from BMI160 */
  BMI160.readGyroScaled(gx, gy, gz);
  BMI160.readAccelerometerScaled(ax, ay, az);
//  printf("%i, %lf, %lf, %lf, %lf, %lf, %lf\n", currentTime, gx, gy, gz, ax, ay, az);
  
  if (sensor_packet.status == 0) {
  
    /* status -> busy */
    sensor_packet.status = 1;
  
    /* Create a message */
    snprintf(sensor_packet.ax, MSGLEN, "%lf", ax);
    snprintf(sensor_packet.ay, MSGLEN, "%lf", ay);
    snprintf(sensor_packet.az, MSGLEN, "%lf", az);
    snprintf(sensor_packet.gx, MSGLEN, "%lf", gx);
    snprintf(sensor_packet.gy, MSGLEN, "%lf", gy);
    snprintf(sensor_packet.gz, MSGLEN, "%lf", gz);
  
    /* Send to MainCore */
    int mp_ret = 0;
    mp_ret = MP.Send(MY_MSGID, &sensor_packet);
    if (mp_ret < 0) {
      printf("MP.Send error = %d\n", mp_ret);
    }
  }


}

void errorLoop(int num)
{
  int i;

  while (1) {
    for (i = 0; i < num; i++) {
      ledOn(LED0);
      delay(300);
      ledOff(LED0);
      delay(300);
    }
    delay(1000);
  }
}
