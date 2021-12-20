/*
 *  LteHttpSecureClient.ino - Example for secure HTTP client using LTE
 *  Copyright 2019, 2021 Sony Semiconductor Solutions Corporation
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
 *
 *  This sketch connects to a website via LTE.
 *  In this example, an HTTP GET request is sent to https://httpin.org/get, 
 *  and an HTTP POST request is sent to https://httpin.org/post. 
 */

#ifdef SUBCORE
#error "Core selection is wrong!!"
#endif

// libraries
#include <MP.h>
#include <Camera.h>
#include <stdlib.h>
#include <stdio.h>


#include <MemoryUtil.h>
#include <SensorManager.h>
#include <AccelSensor.h>
#include <Aesm.h>
#include <ApplicationSensor.h> 

#include <ArduinoHttpClient.h>
#include <RTC.h>
#include <SDHCI.h>
#include <LTE.h>
//#include <Arduino_JSON.h>
#include <GNSS.h>
#include "gnss_nmea.h"

bool step_counter_result(sensor_command_data_mh_t &data);

int intPin=4;
int gCounter=0;
int Mpin=6;

bool stopping = true;

int subcore=1;


const int walking_stride          = 60; /* 60cm */
const int running_stride          = 80; /* 80cm */

const int gyro_range             = 2; /* +/- 2000 degrees/second */
const int gyro_rate              = 200; /* 50 Hz */
const int accel_range             = 2; /* 2G */
const int accel_rate              = 100; /* 50 Hz */
const int accel_sample_num        = 50; /* 50 sample */
const int accel_sample_size       = sizeof(float) * 3;



#define MSGLEN      64


/*コア間で送受信する構造体*/
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


//struct MyPacket {
//  volatile int status; /* 0:ready, 1:busy */
//  char message[MSGLEN];
//};
//
//struct BMI160Packet {
//  volatile int status; /* 0:ready, 1:busy */
//  char ax[MSGLEN];
//  char ay[MSGLEN];
//  char az[MSGLEN];
//  char gx[MSGLEN];
//  char gy[MSGLEN];
//  char gz[MSGLEN];
//};



#define TOTALNUM      2

// APN name
#define APP_LTE_APN "iijmio.jp" // replace your APN

/* APN authentication settings
 * Ignore these parameters when setting LTE_NET_AUTHTYPE_NONE.
 */
#define APP_LTE_USER_NAME "mio@iij"     // replace with your username
#define APP_LTE_PASSWORD  "iij" // replace with your password

// APN IP type
#define APP_LTE_IP_TYPE (LTE_NET_IPTYPE_V4V6) // IP : IPv4v6


#define APP_LTE_AUTH_TYPE (LTE_NET_AUTHTYPE_CHAP) // Authentication : CHAP

#define APP_LTE_RAT (LTE_NET_RAT_CATM) // RAT : LTE-M (LTE Cat-M1)


// URL, path & port (for example: httpbin.org)
char server[] = "54.199.101.204";
char getPath[] = "/get";
char postPath[] = "/";
int port = 5000; // port 443 is the default for HTTPS

// initialize the library instance
LTE lteAccess;
LTEClient tlsClient;
HttpClient client = HttpClient(tlsClient, server, port);
SDClass theSD;
SpGnss Gnss;


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



void printClock(RtcTime &rtc)
{ 
  printf("%04d/%02d/%02d %02d:%02d:%02d\n",
         rtc.year(), rtc.month(), rtc.day(),
         rtc.hour(), rtc.minute(), rtc.second());
}

void doAttach()
{
  while (true) {

    /* Power on the modem and Enable the radio function. */

    if (lteAccess.begin() != LTE_SEARCHING) {
      Serial.println("Could not transition to LTE_SEARCHING.");
      Serial.println("Please check the status of the LTE board.");
      for (;;) {
        sleep(1);
      }
    }

    /* The connection process to the APN will start.
     * If the synchronous parameter is false,
     * the return value will be returned when the connection process is started.
     */
    if (lteAccess.attach(APP_LTE_RAT,
                         APP_LTE_APN,
                         APP_LTE_USER_NAME,
                         APP_LTE_PASSWORD,
                         APP_LTE_AUTH_TYPE,
                         APP_LTE_IP_TYPE,
                         false) == LTE_CONNECTING) {
      Serial.println("Attempting to connect to network.");
      break;
    }

    /* If the following logs occur frequently, one of the following might be a cause:
     * - APN settings are incorrect
     * - SIM is not inserted correctly
     * - If you have specified LTE_NET_RAT_NBIOT for APP_LTE_RAT,
     *   your LTE board may not support it.
     */
    Serial.println("An error has occurred. Shutdown and retry the network attach preparation process after 1 second.");
    lteAccess.shutdown();
    sleep(1);
  }
}


void setup()
{
  pinMode(Mpin, OUTPUT);
  digitalWrite(Mpin,LOW);
  // initialize serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.println("Starting secure HTTP client.");
  
  
  
  

  /* Initialize SD */
  while (!theSD.begin()) {
    ; /* wait until SD card is mounted. */
  }
  
  
//  theCamera.begin();
//  
//  theCamera.setStillPictureImageFormat( //CAM_IMGSIZE_QUADVGA_Vの方が画質がいいがデータ容量が大きくなる
//    CAM_IMGSIZE_VGA_H, CAM_IMGSIZE_VGA_V, CAM_IMAGE_PIX_FMT_JPG
//  );
  
  
  

//  doAttach();

  int result;
  MPLog("aaaaaaaaa");
  /* Activate GNSS device */
  result = Gnss.begin();
  MPLog("bbbbbbbbb");
  assert(result == 0);
  MPLog("cccccccccc");
  /* Start positioning */
  result = Gnss.start();
  MPLog("dddddddd");
  assert(result == 0);
  Serial.println("Gnss setup OK");

  

  for(int subid=1; subid<=1; subid++){
    int ret=0;
    ret=MP.begin(subid);
    Serial.println(ret);
    if(ret<0){
      printf("MP.begin(%d) error=%d\n",subid,ret);
    }
    else
    {
      printf("subcore%d start",subid);
    }
  }

//  lte_setup();
  
  /* Polling */
  MP.RecvTimeout(MP_RECV_POLLING);
}

void lte_setup()
{
  // Wait for the modem to connect to the LTE network.
  Serial.println("Waiting for successful attach.");
  LTEModemStatus modemStatus = lteAccess.getStatus();

  while(LTE_READY != modemStatus) {
    if (LTE_ERROR == modemStatus) {

      /* If the following logs occur frequently, one of the following might be a cause:
       * - Reject from LTE network
       */
      Serial.println("An error has occurred. Shutdown and retry the network attach process after 1 second.");
      lteAccess.shutdown();
      sleep(1);
      doAttach();
    }
    sleep(1);
    Serial.println("Error");
    modemStatus = lteAccess.getStatus();
  }

  Serial.println("attach succeeded.");


  // Set local time (not UTC) obtained from the network to RTC.
  RTC.begin();
  unsigned long currentTime;
  while(0 == (currentTime = lteAccess.getTime())) {
    sleep(1);
  }
  RtcTime rtc(currentTime);
  printClock(rtc);
  RTC.setTime(rtc);
}

String GNSS(String result){
  String contentType ="application/json ; charset=utf-8";
  
  
  if (Gnss.waitUpdate(-1)){
    String answer = "0";
    SpNavData navData;
    Gnss.getNavData(&navData);
    bool posFix = ((navData.posDataExist) && (navData.posFixMode != FixInvalid));
    if (posFix){
      Serial.println("Position is fixed");
      String nmeaString = getNmeaGga(&navData);
      String contentType ="application/json ; charset=utf-8";
      float lati = navData.latitude;
      //float lati = 35.0000000000000000;
      float longi = navData.longitude;
      //float longi = 135.000000000000000;
      
      String data1 = String("\"latitude\":") + String(lati,9) + String(",");
      String data2 = String("\"longitude\":") + String(longi,9) + String(",");
      String data3; 
      if (result.indexOf("True") != -1 ){
        data3 = String("\"is_fell\":") + String("true");
      }else{
        data3 = String("\"is_fell\":") + String("false");
      }
       

      String moji = String("{")+data1+data2+data3+String("}");
      Serial.print(moji);
      String postData = moji;
      //String postData = "{\"latitude\":navData.latitude,\"longitude\":navData.longitude,\"is_fell\":\"true\"}";
      Serial.print("Connection to Server");
      client.post(postPath, contentType, postData);
      int statusCode = client.responseStatusCode();
      String response = client.responseBody();
      Serial.print("Status code: ");
      Serial.println(statusCode);
      Serial.print("Response: ");
      Serial.println(response);
      for (int i=0; i < 10; i++) {
        client.flush();
      }
      Serial.println(response.indexOf("false"));
      if (response.indexOf("false") != -1){
        answer = "1";
        Serial.println("座標登録");
      }
      else if(response.indexOf("true")!= -1){
        answer = "2";
        Serial.println("領域展開");
      }
      else{
        answer = "3";
        Serial.println("Response Error");
      }
      client.stop();
    }else{
      Serial.println("position is not fixed");
    }
    return answer;
  }
}

void Moter(){
  int counter=0;
  digitalWrite(Mpin, HIGH);
  sleep(1);
  digitalWrite(Mpin, LOW);
}

void loop()
{
  int      ret1, ret2;
  int      subid;
  int8_t   msgid;
  int8_t   msgid2;
  FallDownPacket *packet;
//  BMI160Packet *bmi160_packet;
  static float gx=0;
  static float gy=0;
  static float gz=0;
  static float ax=0;
  static float ay=0;
  static float az=0;
  static float B_pressure=0;//1つ前の気圧
  String result = "False";
  String answer = "0";
 

  static float pressure=0;
  bool standing = false;
  

  //サブコア1から転倒検知結果を受信
    ret1 = MP.Recv(&msgid, &packet, 1);
    if (ret1 > 0) {
      pressure=atof(packet->pressure);
//      MPLog("pressure = %lf\n", pressure);
//      if(packet->falldown)MPLog("falldown!!");
  
      /* status -> ready */
      packet->status = 0;
    }
//  MPLog("ax = %lf, ay = %lf, az = %lf, gz = %lf\n",ax, ay, az, gz);

  
  
//  answer=GNSS(result);
//  Serial.println("result");
//  Serial.println(answer);
//  Serial.println("Fall Down signal");
//  if(answer.indexOf("2") == -1){
//    Serial.println("Not fall down"); 
////    sleep(5);   
//  }
//  else{
//    Moter();
//  }
}

/**
void Camera(){
  
  CamImage img = theCamera.takePicture();

  char filename[16] = {0};
  if(img.isAvailable())
  {
    
    digitalWrite(LED0,HIGH);
    printf("gCounter = %d\n", gCounter);
    printf("camera\n");
    sprintf(filename,"PICT%02d.JPG", gCounter);
    
    if(theSD.exists(filename))
    {
      theSD.remove(filename);
      printf("delete\n");
    }
    File myFile = theSD.open(filename, FILE_WRITE);
    
    myFile.write(img.getImgBuff(), img.getImgSize());
    myFile.close();

    gCounter++;
    if(gCounter == 10)gCounter=0;
  }
  digitalWrite(LED0,LOW);

  delay(1000);
}
**/
