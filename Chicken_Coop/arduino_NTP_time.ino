/*

 Udp NTP Client

 Get the time from a Network Time Protocol (NTP) time server
 Demonstrates use of UDP sendPacket and ReceivePacket
 For more on NTP time servers and the messages needed to communicate with them,
 see http://en.wikipedia.org/wiki/Network_Time_Protocol

 created 4 Sep 2010
 by Michael Margolis
 modified 9 Apr 2012
 by Tom Igoe
 modified 02 Sept 2015
 by Arturo Guadalupi

 This code is in the public domain.

 */
/*
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
//byte mac[] = {  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
//IPAddress ip(10,110,7,199); // local IP TDI

unsigned int localPort = 8888;       // local port to listen for UDP packets

char timeServer[] = "time.nist.gov"; // time.nist.gov NTP server

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
EthernetUDP EthUdp;
*/
/*
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // start the Ethernet connection:
  Serial.println("Starting Ethernet...");
  Ethernet.begin(mac, ip);
  // give time for ethernet shield to initialize.
  delay(1000);
  Serial.println("ok.");
  Serial.println("connecting...");
  Serial.print("My address is ");
  Serial.println(Ethernet.localIP());

  EthUdp.begin(localPort);
  }
  */

/*
void getNTPtime() {
  sendNTPpacket(timeServer); // send an NTP packet to a time server

  // wait to see if a reply is available
  delay(1000);
  if (EthUdp.parsePacket()) {
    // We've received a packet, read the data from it
    EthUdp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    // the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, extract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = ");
    Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);


    // print the hour, minute and second:
    /*Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if (((epoch % 3600) / 60) < 10) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    if ((epoch % 60) < 10) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second*/
        
   /*     //Time of Day Conversion to CST
    String NTPtimeString = String(epoch);
    int firsttimeColon = NTPtimeString.indexOf(":");
    int secondtimeColon = NTPtimeString.lastIndexOf(":");
    String timehourString = NTPtimeString.substring(0,firsttimeColon);
    String timeminString = NTPtimeString.substring(firsttimeColon+1, secondtimeColon);
    String timesecString = NTPtimeString.substring(secondtimeColon+1);*/
    /*
    //int timeHour = timehourString.toInt()- 6;
    //int timeMinute = timeminString.toInt();
    //int timeSecond = timesecString.toInt();
    int timeHour = ((epoch  % 86400L) / 3600)-6;
    int timeMinute = ((epoch  % 3600) / 60);
    int timeSecond = (epoch % 60);
    
    Serial.print("NTP hour: "), Serial.println(timeHour);
    Serial.print("NTP min: "), Serial.println(timeMinute);
    Serial.print("NTP sec: "), Serial.println(timeSecond);
    Update_RTC(0,0,0,timeHour,timeMinute,timeSecond); // Update Time only, 0's in for date only for placeholders 
  }
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(char* address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  EthUdp.beginPacket(address, 123); //NTP requests are to port 123
  EthUdp.write(packetBuffer, NTP_PACKET_SIZE);
  EthUdp.endPacket();
}
*/
/****************************************************************
*Update Time on Real Time Clock
*****************************************************************/
/*
  int Update_RTC(int d, int mo, int y, int h, int mi, int s){
    int TimeDate [7] = {s, mi, h, 0, d, mo, y};
    for (int i = 0; i <= 2; i++) {
      int b = TimeDate[i] / 10;
      int a = TimeDate[i] - b * 10;
      if (i == 2) {
        if (b == 2)
          b = B00000010;
        else if (b == 1)
          b = B00000001;
      }
      TimeDate[i] = a + (b << 4);
  
      digitalWrite(rtcs, LOW);
      SPI.transfer(i + 0x80);
      SPI.transfer(TimeDate[i]);
      digitalWrite(rtcs, HIGH);
    }
  }
  */
