/*
  Web client
 
 This sketch connects to a website to fetch the sunrise and sunset times for 'today'
 using an Arduino Wiznet Ethernet shield. 
 
 Circuit:
 * Ethernet shield attached to pins 53, 11, 12, 13
 
 created 14 October 2015
 author: Michael Hogue
 
 */
/*
#include <SPI.h>
#include <SD.h>
// ethernet library from https://github.com/Seeed-Studio/Ethernet_Shield_W5200
#include <EthernetV2_0.h>
// httpclient library from https://github.com/amcewen/HttpClient
#include <HttpClient.h>
// ArduinoJson library from https://github.com/bblanchon/ArduinoJson
#include <ArduinoJson.h>

// Number of milliseconds to wait without receiving any data before we give up
const int kNetworkTimeout = 30*1000;
// Number of milliseconds to wait if no data is available before trying again
const int kNetworkDelay = 1000;
// Hour, Minute, Second of Sunrise CST
int sunriseHour, sunriseMinute, sunriseSecond;
// Hour, Minute, Second of Sunset CST
int sunsetHour, sunsetMinute, sunsetSecond;

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
//byte mac[] = {  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
//IPAddress ip(10,10,10,198); // local IP HogHouse
//IPAddress ip(10,110,7,199); // local IP TDI

// Initialize the Ethernet client library
// with the IP address and port of the server 
// that you want to connect to (port 80 is default for HTTP):
//EthernetClient client;
//HttpClient http(client);
#define W5200_CS 53
#define SDCARD_CS 4

// parameters needed to fetch sunrise and sunset time RESTfully.
const char sunriseSunsetServer[] = "api.sunrise-sunset.org";
const char sunriseSunsetPath[] = "/json?lat=40.547554&lng=-89.614399&date=today";
IPAddress sunriseSunsetIP(104,131,2,15);

#define JSON_START_CHAR '{'
*/

/**
 * Method to fetch sunrise and sunset time
 * 
 * The parameter results will be populated with the sunrise and sunset returned by the web service
 */
 /*
void getSunriseSunsetHTTP(String results[]) {
  StaticJsonBuffer<1024> jsonBuffer;
  int err = 0;
  err = http.get(sunriseSunsetIP, sunriseSunsetServer, sunriseSunsetPath);
  if(err == 0) {
    Serial.println("connection ok");
    err = http.responseStatusCode();

    if(err >= 0) {
      Serial.print("response status code: ");
      Serial.println(err);

      // TODO: should check here that response code is 200, which means "OK"
      err = http.skipResponseHeaders();
      if(err >= 0) {
        // this particular web service doesn't return a Content-Length header.
        // That means that we'll need to skip to the start of the json response to start parsing, handling the length dynamically.
        // I've left the below commented out since it's really useless here, but may be in other web requests.
        //int bodyLen = http.contentLength();
        //Serial.print("content length: ");
        //Serial.println(bodyLen);
        //Serial.println("Body returned:");

        unsigned int timeoutStart = millis();
        char c;
        bool readJson = false;
        char buff[1024];
        int count = 0;
        
        // while we're connected to the host and we've not yet hit the timeout, process response.
        while( (http.connected() || http.available()) && ((millis() - timeoutStart) < kNetworkTimeout)) {
          if(http.available()) {
            c = http.read();
            // We read something, reset the timeout counter
            timeoutStart = millis();
            
            if(!readJson && JSON_START_CHAR == c) {
              Serial.println("found start of json");
              readJson = true;
              
            } else if(!readJson) {
              // we've not yet reached the json content.. Continue reading and build the body length.
              if(isdigit(c)) {
                // leftshift digit and add c (i.e. if we had read 1 and are now reading c: 1*10 = 10 + c. If c was 3, then we'd have 13.)
              // bodyLen = bodyLen*10 + (c - '0');
              }
              continue;
            }
            Serial.print(c);
            buff[count] = c;
            count++;
          } else {
              // We haven't got any data, so let's pause to allow some to
              // arrive
              delay(kNetworkDelay);
          }
        }
        buff[count] = '\0';
        JsonObject& root = jsonBuffer.parseObject(buff);
        if(!root.success()) {
          Serial.println("failed to parse json response. parseObject() failed");
        }
        
        String respSunrise = root["results"]["sunrise"];
        String respSunset = root["results"]["sunset"];
        // construct new strings so as to copy the values.
        results[0] = String(respSunrise);
        results[1] = String(respSunset);
      } else {
        Serial.print("Failed to skip response headers: ");
        Serial.println(err);
      }
    } else {
      Serial.print("Getting response failed: ");
      Serial.println(err);
    }
  } else {
    Serial.print("Connection failed: ");
    Serial.println(err);
  }
  
  Serial.println("disconnecting.");
  http.stop();
}
*/
/**
 * Setup method
 */
/*void sunrise_sunset_setup() {
 // Open serial communications and wait for port to open:
  Serial.begin(115200);
  // disable w5100 SPI while starting SD
  pinMode(W5200_CS,OUTPUT);
  digitalWrite(W5200_CS,HIGH);
  
  Serial.println("Starting SD...");
  if(!SD.begin(SDCARD_CS)) {
    Serial.println("failed.");
  } else {
    Serial.println("ok.");
  }
  
  // start the Ethernet connection:
  Serial.println("Starting Ethernet...");
  Ethernet.begin(mac, ip);

  // give time for ethernet shield to initialize.
  delay(2000);
  Serial.println("ok.");
  Serial.println("connecting...");

  // Ethernet begin returns with its SPI enabled, so disable it.
  digitalWrite(W5200_CS,HIGH);

  Serial.print("My address is ");
  Serial.println(Ethernet.localIP());
}*/
/*
void sunrise_sunset_loop()
{
  // this array will hold the requested sunrise/sunset times.
  String results[2];
  getSunriseSunsetHTTP(results);
 
  //sunrise
  String risetimeString = results[0];
  int firstriseColon = risetimeString.indexOf(":");
  int secondriseColon = risetimeString.lastIndexOf(":");
  String risehourString = risetimeString.substring(0,firstriseColon);
  String riseminString = risetimeString.substring(firstriseColon+1, secondriseColon);
  String risesecString = risetimeString.substring(secondriseColon+1);
  sunriseHour = risehourString.toInt()- 6;
  sunriseMinute = riseminString.toInt();
  sunriseSecond = risesecString.toInt();
   
  //sunset
  String settimeString = results[1];
  int firstsetColon = settimeString.indexOf(":");
  int secondsetColon = settimeString.lastIndexOf(":");
  String sethourString = settimeString.substring(0,firstsetColon);
  String setminString = settimeString.substring(firstsetColon+1, secondsetColon);
  String setsecString = settimeString.substring(secondsetColon+1);
  sunsetHour = sethourString.toInt()- 6;
  sunsetMinute = setminString.toInt();
  sunsetSecond = setsecString.toInt();
   
  Serial.print("result sunrise: ");
  Serial.println(results[0]);
  Serial.print("result sunset: ");
  Serial.println(results[1]);
  Serial.print("sunrise hour: "), Serial.println(sunriseHour);
  Serial.print("sunrise min: "), Serial.println(sunriseMinute);
  Serial.print("sunset hour: "), Serial.println(sunsetHour);
  Serial.print("sunset min: "), Serial.println(sunsetMinute);

  int haveSStime = 1;
}
*/
