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
#include <BMI160Gen.h>
#define MSGLEN      64
#define MY_MSGID    10
#define MY_MSGID2    11

#define I2C_SDA_PIN 2 //20  //2
#define I2C_SCL_PIN 29  //21    //29

const int i2c_addr                = 0x68;
const int baudrate                = 115200;
const int gyro_range             = 2; /* +/- 2000 degrees/second */
const int gyro_rate              = 200; /* 50 Hz */
const int accel_range             = 2; /* 2G */
const int accel_rate              = 200; /* 50 Hz */


struct DPS310Packet {
  volatile int status; /* 0:ready, 1:busy */
  char message[MSGLEN];
};

struct FallDownPacket {
  volatile int status; /* 0:ready, 1:busy */
  char pressure[MSGLEN];
  char ax[MSGLEN];
  char ay[MSGLEN];
  char az[MSGLEN];
  char gx[MSGLEN];
  char gy[MSGLEN];
  char gz[MSGLEN];
  bool falldown;
};

FallDownPacket packet;
//DPS310Packet packet;
//DPS310Packet pressure_packet;

SlowSoftWire WireSoft = SlowSoftWire(I2C_SDA_PIN, I2C_SCL_PIN, true);
Dps310Soft Dps310PressureSensor = Dps310Soft();

uint32_t  currentTime;
uint32_t  loopTimer=0;

uint8_t oversampling = 4;//0~7



class GyroPattern
{
  public:
    GyroPattern() {};

    String get_state()
    {
      return state;
    }

    void update(float gz)
    {
      if (abs(gz) < 5)stoppicg_cnt++;
      else stoppicg_cnt = 0;

      if (stoppicg_cnt > 50)
      {
        state = "stop";
        //        printf("STOP\n");
      }
      else
      {
        state = "move";
        //        printf("UNKNOWN\n");
      }
    }

  private:
    uint stoppicg_cnt = 0;
    String state;
};

GyroPattern gyro_pattern;

/* Const values */

void setup()
{
  int ret = 0;
  memset(&packet, 0, sizeof(packet));//メインコアに通信
  ret = MP.begin();
  if (ret < 0) {
    errorLoop(2);
  }

  Serial.begin(baudrate);
  while (!Serial);

  /* Initialize device. */
  Dps310PressureSensor.begin(WireSoft);
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

//  puts("pressure[Pascal]");
  
  currentTime = micros();
  loopTimer = micros();
}

void loop()
{
  
//  while (currentTime - loopTimer <= 10000)currentTime = micros();
//  loopTimer = currentTime;
//  printf("loop\n");
  static float B_pressure; //1つ前の気圧
  static float p_diff;
  static float pressure;
  int16_t pressure_ret;
  float gx, gy, gz;
  float ax, ay, az;
  bool fall_down = false;
  bool pressure_update;
  currentTime = micros();
  BMI160.readGyroScaled(gx, gy, gz);
  BMI160.readAccelerometerScaled(ax, ay, az);
//  printf("%i, %lf, %lf, %lf, %lf, %lf, %lf\n", currentTime, gx, gy, gz, ax, ay, az);
  gyro_pattern.update(gz);

  


  //Pressure measurement behaves like temperature measurement
  //ret = Dps310PressureSensor.measurePressureOnce(pressure);
  pressure_ret = Dps310PressureSensor.measurePressureOnce(pressure, oversampling);
  
  if (pressure_ret != 0)
  {
    //Something went wrong.
    //Look at the library code for more information about return codes
    Serial.print("FAIL! ret = ");
    Serial.println(pressure_ret);
    pressure_update = false;
  }
  else
  {
    pressure_update = true;
    if(pressure != B_pressure)
    {
      pressure_update = true;
      p_diff = pressure-B_pressure;
    }
    else
    {
      pressure_update = false;
    }
    
    
//    Serial.println(pressure);
  }

  //転倒判定
  if (gyro_pattern.get_state() == "move")
  {
    //    printf("Gyro_Moving\n");


    //    printf("%lf\n",diff);
    if (p_diff > 1 && abs(gz) > 70) //気圧が高くなった場合
    {
      fall_down = true;
//            printf("------fall down?---------\n");
//            printf("%lf\n",p_diff);
    }
    else fall_down = false;
  }

  printf("%i, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf, %d\n", currentTime, gx, gy, gz, ax, ay, az, pressure, p_diff, fall_down);

  if (packet.status == 0) {
    /* status -> busy */
    packet.status = 1;

    /* Create a message */
    snprintf(packet.pressure, MSGLEN, "%lf", pressure);
    snprintf(packet.ax, MSGLEN, "%lf", ax);
    snprintf(packet.ay, MSGLEN, "%lf", ay);
    snprintf(packet.az, MSGLEN, "%lf", az);
    snprintf(packet.gx, MSGLEN, "%lf", gx);
    snprintf(packet.gy, MSGLEN, "%lf", gy);
    snprintf(packet.gz, MSGLEN, "%lf", gz);
    packet.falldown = fall_down;

    /* Send to MainCore */
    int mp_ret = 0;
    mp_ret = MP.Send(MY_MSGID, &packet);
    if (mp_ret < 0) {
      printf("MP.Send error = %d\n", mp_ret);
    }
  }

  if(pressure_update)
  {
    B_pressure = pressure;
    pressure_update = false;
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
