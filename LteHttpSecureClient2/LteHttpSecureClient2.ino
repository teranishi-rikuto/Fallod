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

// libraries
#include <ArduinoHttpClient.h>
#include <RTC.h>
#include <SDHCI.h>
#include <LTE.h>
//#include <Arduino_JSON.h>
#include <GNSS.h>
#include "gnss_nmea.h"

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

  doAttach();

  int result;

  /* Activate GNSS device */
  result = Gnss.begin();
  assert(result == 0);

  /* Start positioning */
  result = Gnss.start();
  assert(result == 0);
  Serial.println("Gnss setup OK");

  lte_setup();
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

void GNSS(){
  String contentType ="application/json ; charset=utf-8";
  
  if (Gnss.waitUpdate(-1)){
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
      String data3 = String("\"is_fell\":") + String("true"); 

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
      int i = 0;
      while(i < 10){
        client.flush();
        i++;
      }
      client.stop();
      sleep(15);
    }else{
      Serial.println("position is not fixed");
    }

    lte_setup();
  }
}

void loop()
{
  GNSS();
}
