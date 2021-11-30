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

#if (SUBCORE != 1)
#error "Core selection is wrong!!"
#endif

#include <MP.h>
#include <SlowSoftWire.h>
#include "dps310_softwire/Dps310Soft.h"
#define MSGLEN      64
#define MY_MSGID    10

#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 29

struct MyPacket {
  volatile int status; /* 0:ready, 1:busy */
  char message[MSGLEN];
};


MyPacket packet;
SlowSoftWire WireSoft = SlowSoftWire(I2C_SDA_PIN, I2C_SCL_PIN, true);
Dps310Soft Dps310PressureSensor = Dps310Soft();

uint32_t  currentTime;
uint32_t  loopTimer=0;

const int baudrate                = 115200;
uint8_t oversampling = 7;

/* Const values */
//BMI160
const int i2c_addr                = 0x68;
const int gyro_range             = 2; /* +/- 2000 degrees/second */
const int gyro_rate              = 200; /* 50 Hz */
const int accel_range             = 2; /* 2G */
const int accel_rate              = 200; /* 50 Hz */
const int accel_sample_num        = 100; /* 50 sample */
const int accel_sample_size       = sizeof(float) * 3;

void setup()
{
  int ret = 0;
  memset(&packet, 0, sizeof(packet));
  ret = MP.begin();
  Serial.println("subcore1 start");
  Serial.println(ret);
  if (ret < 0) {
    errorLoop(2);
  }

  Serial.begin(baudrate);
  while (!Serial);

  /* Initialize device. */
  Dps310PressureSensor.begin(WireSoft);

//  puts("pressure[Pascal]");
  
  currentTime = micros();
  loopTimer = micros();
}

void loop()
{
  while (currentTime - loopTimer <= 10000)currentTime = micros();
  loopTimer = currentTime;
  float pressure;
  int16_t pressure_ret;
  //Pressure measurement behaves like temperature measurement
  //ret = Dps310PressureSensor.measurePressureOnce(pressure);
  pressure_ret = Dps310PressureSensor.measurePressureOnce(pressure, oversampling);
  if (pressure_ret != 0)
  {
    //Something went wrong.
    //Look at the library code for more information about return codes
    Serial.print("FAIL! ret = ");
    Serial.println(pressure_ret);
  }
  else
  {
//    Serial.println(pressure);
    if (packet.status == 0) {
    
        /* status -> busy */
        packet.status = 1;
    
        /* Create a message */
        snprintf(packet.message, MSGLEN, "%lf", pressure);
    
        /* Send to MainCore */
        int mp_ret = 0;
        mp_ret = MP.Send(MY_MSGID, &packet);
        if (mp_ret < 0) {
          printf("MP.Send error = %d\n", mp_ret);
        }
      }
  }




  
//  int      ret;
//  int8_t   msgid;
//  uint32_t msgdata;
//
//  /* Echo back */
//
//  ret = MP.Recv(&msgid, &msgdata);
//  if (ret < 0) {
//    errorLoop(3);
//  }
//
//  ret = MP.Send(msgid, msgdata);
//  if (ret < 0) {
//    errorLoop(4);
//  }
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
