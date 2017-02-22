//**** Automated Chicken Coop ****//
//**** Date: 14 July 2015 ****//
//**** Author: J. Hogue ****//

#include <SPI.h>
#include <OneWire.h>
#include "DualVNH5019MotorShield.h"
#include "Bridge.h"
#include <EthernetV2_0.h>
#include <HttpClient.h>
#include <BridgeServer.h>
#include <BridgeClient.h>
#include <SD.h>


#define ServerDEBUG
#define DOOR_MOVING 0
#define DOOR_OPEN 1
#define DOOR_CLOSED 2

/****************************************************************
* Designated Digital Pinout
*****************************************************************/
// Pin 0 = Serial Comms RX0
// Pin 1 = Serial Comms TX0
//const int Mtr_1_Dir_In_A_Pin = 2; // Motor 1 Direction Input A Pin
//const int Mtr_1_Dir_In_B_Pin = 3; // Motor 1 Direction Input B Pin
const int SDchipSelect = 4; // SD Card Pin
OneWire  ds(5);  // on pin 5 (a 4.7K resistor is necessary)-Coop Inside Temperature Sensor Input Pin
//const int Mtr_1_Enable_Diag_Pin = 6; // Motor 1 Enable & Diagnostic Pin
//const int Mtr_2_Dir_In_A_Pin = 7; // Motor 2 Direction Input A Pin
//const int Mtr_2_Dir_In_B_Pin = 8; // Motor 2 Direction Input B Pin
//const int Mtr_1_Speed_Input_Pin = 9; // Motor 1 PWM Speed Input Pin
//const int Mtr_2_Speed_Input_Pin = 10; // Motor 2 PWM Speed Input Pin
// Pin 11 = SPARE
//const int Mtr_2_Enable_Diag_Pin = 12; // Motor 2 Enable & Diagnostic Pin
// Pin 13 = SPARE
// Pins 14-19 = Serial Comms 1, 2, & 3
// Pin 20 = SDA
// Pin 21 = SCL
const int Door_Up_Ovrd_Pin = 22; // Door Up Override Pushbutton Input Pin
const int Door_Up_Switch_Pin = 23; // Door in Up Position Switch Input Pin
const int Door_Down_Ovrd_Pin = 24; // Door Down Override Pushbutton Input Pin
const int Door_Down_Switch_Pin = 25; // Door in Down Position Switch Input Pin
const int Water_Heater_Pin = 26; // Water Heater Output Pin
const int Ceiling_Heater_Pin = 27; // Ceiling Heater Output Pin
const int Inside_Light_Pin = 28; // Inside Lights Output Pin
const int Exhaust_Fan_Pin = 29; // Exhaust Fan Output Pin
const int Outside_Light_Pin = 30; // Outside Lights Output Pin
const int Fault_LED_Pin = 33; // Fault LED Output Pin
// Pins 31-48 = SPARE
const int rtcs = 49; // Real Time Chip Pin
// Pin 50 = MISO connected to RTC MISO Pin
// Pin 51 = MOSI connected to RTC MOSI Pin
// Pin 52 = SCK connected to RTC CLK Pin
//const int EthchipSelect = 53; // Ethernet Chip Pin


//*****************************************************************************************************//
//  FOR DEBUG ONLY  MAKE SURE TO COMMENT OUT FOR ACTUAL APPLICATION   //
const int temperaturePin = 3; // Analog Pin

//*****************************************************************************************************//

/****************************************************************
* Designated Analog Pinout
*****************************************************************/

const int Mtr_1_Current_Sense_Pin = A0; // Motor 1 Current Sense Output Pin
const int Mtr_2_Current_Sense_Pin = A1; // Motor 2 Current Sense Output Pin
const int Ammonia_Pin = A2; // Ammonia Sensor Input Pin

/****************************************************************
* Variables
*****************************************************************/
byte timerState[20];
unsigned long timer[20];
int TimeDate [7]; //second,minute,hour,null,day,month,year
int Door_State; // Door State Open/Closed
boolean Door_Open; // Door Switch 0 = Closed, 1 = Open
boolean Door_Closed; // Door Switch 0 = Open, 1 = Closed
int Door_2nd_chance; // Door opens and closes 5 minutes after initial closing for the evening
int DoorUpSwitch = 0; // Limit switch that monitors when the coop door is in the up position
int DoorDownSwitch = 0; // Limit switch that monitors when the coop door is in the down position
unsigned long IOTimer = 5000; // Timer value to update IO
boolean DoorUpOvrd; // Door Open Overide Button
boolean DoorDownOvrd; // Door Close Overide Button
float celsius, fahrenheit, CoopTemp, WaterTemp;
int nr = 0;  // Number of Fault LED blinks in relation to fault code
int FaultCode = 0; // Fault Code Register
float AmmoniaLevel;
bool KeepFanOn = 0; // Used to keep fan from cycling On/Off too quickly
bool KeepFanOff = 0; // Used to keep fan from cycling On/Off too quickly
float CoopTemp_FanHighLimit = 80;  // Limit to control the exhaust fan
int CloseDoorHour = 20, CloseDoorMin = 0;
int OpenDoorHour = 7, OpenDoorMin = 0;
//int CloseDoorHour = 14, CloseDoorMin = 32;  //debug
//int OpenDoorHour = 14, OpenDoorMin = 36;  //debug
boolean isInitialized = false;

DualVNH5019MotorShield md;
//EthernetServer server(80);
BridgeServer server;

// Initialize the Ethernet client library
// with the IP address and port of the server 
// that you want to connect to (port 80 is default for HTTP):
//EthernetClient client;
//HttpClient http(client);

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while (1) {
      FaultCode = 2, CheckFaults();
    }
  }
}

void setup() {

  // Define digital pins as inputs/outputs:
  //pinMode(Mtr_1_Dir_In_A_Pin, INPUT);
  //pinMode(Mtr_1_Dir_In_B_Pin, INPUT);
  pinMode(SDchipSelect, OUTPUT);
  //pinMode(Mtr_1_Enable_Diag_Pin, INPUT);
  //pinMode(Mtr_2_Dir_In_A_Pin, INPUT);
  //pinMode(Mtr_2_Dir_In_B_Pin, INPUT);
  //pinMode(Mtr_1_Speed_Input_Pin, INPUT);
  //pinMode(Mtr_2_Speed_Input_Pin, INPUT);
  //pinMode(Mtr_2_Enable_Diag_Pin, INPUT);
  pinMode(Door_Up_Ovrd_Pin, INPUT);
  pinMode(Door_Up_Switch_Pin, INPUT);
  pinMode(Door_Down_Ovrd_Pin, INPUT);
  pinMode(Door_Down_Switch_Pin, INPUT);
  pinMode(Water_Heater_Pin, OUTPUT);
  pinMode(Ceiling_Heater_Pin, OUTPUT);
  pinMode(Inside_Light_Pin, OUTPUT);
  pinMode(Exhaust_Fan_Pin, OUTPUT);
  pinMode(Outside_Light_Pin, OUTPUT);
  pinMode(Fault_LED_Pin, OUTPUT);
  pinMode(rtcs, OUTPUT);
  //pinMode(EthchipSelect, OUTPUT);

  Serial.begin (115200);
  md.init(); // Initialize Motor Driver
  Bridge.begin();

  #ifdef ServerDEBUG
    Serial.print(F("Starting SD.."));
  #endif

  // disable w5100 SPI while setting up SD
  pinMode(10, OUTPUT);                       // set the SS pin as an output (necessary!)
  digitalWrite(10, HIGH);                    // but turn off the W5100 chip!

  if(!SD.begin(4)) {
  #ifdef ServerDEBUG
    Serial.println(F("failed"));
  #endif
  }
  else {
  #ifdef ServerDEBUG
    Serial.println(F("ok"));
  #endif
  }

  server.listenOnLocalhost();
  server.begin();

  RTC_init();
  //day(1-31), month(1-12), year(0-99), hour(0-23), minute(0-59), second(0-59)
  //SetTimeDate(12,9,15,14,19,50); //This only needs to be set when the clock is new or not reading the correct time
}

/****************************************************************
* MAIN LOOP
*****************************************************************/

void loop() {
  if(!isInitialized) {
    isInitialized = true;
    coop_init();
  }
  
  BridgeClient client = server.accept();

  if (client) {
    process(client);
    client.stop();
  }

  // Check coop environment and adjust accordingly
  // Ex: If temp is too low, turn on heaters, if temp is too high, turn on fans.
  // Ex2: Open/Close door based on time of day.

  delay(50); 
}

/************************************************************************************************************/

void process(BridgeClient client) {
  String command = client.readStringUntil('/');

  if (command == "digital") {
    digitalCommand(client);
  }
  if (command == "analog") {
    analogCommand(client);
  }
  if (command == "mode") {
    modeCommand(client);
  }
  if (command == "settings") {
    settingsCommand(client);
  }
  
}

/**
 * Method to handle on/off commands for digital pins.
 * Pin will be the digital pin location on the arduino.
 * Value will be the command, 0 for off/close, 1 for on/open.
 */
void digitalCommand(BridgeClient client) {
  int pin, value;

  pin = client.parseInt();

  if (client.read() == '/') {
    value = client.parseInt();
    digitalWrite(pin, value);
  } 
  else {
    value = digitalRead(pin);
  }

  client.print(F("Pin D"));
  client.print(pin);
  client.print(F(" set to: "));
  client.println(value);

  String key = "D";
  key += pin;
  Bridge.put(key, String(value));
}

/**
 * Method to handle on/off commands for analog pins.
 * Pin will be the analog pin location on the arduino.
 * Value will be the command, 0 for off/close, 1 for on/open.
 */
void analogCommand(BridgeClient client) {
  int pin, value;

  pin = client.parseInt();

  if (client.read() == '/') {
    value = client.parseInt();
    analogWrite(pin, value);

    // Send feedback to client
    client.print(F("Pin D"));
    client.print(pin);
    client.print(F(" set to analog "));
    client.println(value);

    String key = "D";
    key += pin;
    Bridge.put(key, String(value));
  }
  else {
    value = analogRead(pin);

    client.print(F("Pin A"));
    client.print(pin);
    client.print(F(" reads analog "));
    client.println(value);

    String key = "A";
    key += pin;
    Bridge.put(key, String(value));
  }
}

/************************************************************************************************************/

void modeCommand(BridgeClient client) {
  int pin;

  // Read pin number
  pin = client.parseInt();

  // If the next character is not a '/' we have a malformed URL
  if (client.read() != '/') {
    client.println(F("error"));
    return;
  }

  String mode = client.readStringUntil('\r');

  if (mode == "input") {
    pinMode(pin, INPUT);
    // Send feedback to client
    client.print(F("Pin D"));
    client.print(pin);
    client.print(F(" configured as INPUT!"));
    return;
  }

  if (mode == "output") {
    pinMode(pin, OUTPUT);
    // Send feedback to client
    client.print(F("Pin D"));
    client.print(pin);
    client.print(F(" configured as OUTPUT!"));
    return;
  }

  client.print(F("error: invalid mode "));
  client.print(mode);
}

void settingsCommand(BridgeClient client) {
  return;
}

void coop_init() {
  CheckFaults();
  if (delayMilliSeconds(0, 2500)) {
    ReadIO();
  }

  // Temperature Control
  CheckCoopTemp();  // Get Coop Temperature
  //GetAnalogTemp(); debug at work only
  //CheckCoopTemp(); // Get A 2nd Temperature
  CoopTempControl();
  //WaterTempControl(); // Not Installed
  CheckAirQuality();  // Get Ammonia Reading and Control Exhaust Fan based on Ammonia Level or Coop Temperature

              /**********************
                    * DEBUG
              **********************/     
  //Serial.print("Door State:"), Serial.println(Door_State);
  //Serial.print("upbutton:"), Serial.println(DoorUpOvrd);
  //Serial.print("downbutton:"), Serial.println(DoorDownOvrd);
  //Serial.print("Door Open:"), Serial.println(Door_Open);
  //Serial.println(DoorUpSwitch);
  //Serial.print("Door Closed: "), Serial.println(Door_Closed);
  //Serial.println(DoorDownSwitch);
  //Serial.print("Door 2nd Chance: "), Serial.println(Door_2nd_chance);
  //Serial.print("Coop Temp: "), Serial.println(CoopTemp);
  ReadTimeDate();
  Serial.println(ReadTimeDate());
  Serial.println();

  OutsideLight();
  
  if (TimeDate[2] == OpenDoorHour && TimeDate[1] == OpenDoorMin) {
    OpenCoopDoor();
  }

  if (TimeDate[2] == CloseDoorHour && TimeDate[1] == CloseDoorMin) {
    CloseCoopDoor();
  }

  if (DoorUpOvrd == 1) {
    OverrideDoorOpen();
  }

  if (DoorDownOvrd == 1) {
    OverrideDoorClose();
  }
  
  /*if ((Door_State != 1) && (Door_2nd_chance == 1)) {
    SecondChanceDoorOpen();
  }

  if ((Door_State != 2) && (Door_2nd_chance == 2)) {
    SecondChanceDoorClose();
  }*/

  if ((TimeDate[5] >= 10) && (TimeDate[5] <= 2)) { // if the month is a winter month
    InsideLight();
  }
}

/****************************************************************
*Timer Code
*****************************************************************/

//Everything from here down comes after the main "loop()"

int delayHours(byte timerNumber, unsigned long delaytimeH) {  //Here we make it easy to set a delay in Hours
  delayMilliSeconds(timerNumber, delaytimeH * 1000 * 60 * 60);
}
int delayMinutes(byte timerNumber, unsigned long delaytimeM) {  //Here we make it easy to set a delay in Minutes
  delayMilliSeconds(timerNumber, delaytimeM * 1000 * 60);
}
int delaySeconds(byte timerNumber, unsigned long delaytimeS) {  //Here we make it easy to set a delay in Seconds
  delayMilliSeconds(timerNumber, delaytimeS * 1000);
}

int delayMilliSeconds(int timerNumber, unsigned long delaytime) {
  unsigned long timeTaken;
  if (timerState[timerNumber] == 0) { //If the timer has been reset (which means timer (state ==0) then save millis() to the same number timer,
    timer[timerNumber] = millis();
    timerState[timerNumber] = 1;    //now we want mark this timer "not reset" so that next time through it doesn't get changed.
  }
  if (millis() > timer[timerNumber]) {
    timeTaken = millis() + 1 - timer[timerNumber]; //here we see how much time has passed
  }
  else {
    timeTaken = millis() + 2 + (4294967295 - timer[timerNumber]); //if the timer rolled over (more than 48 days passed)then this line accounts for that
  }
  if (timeTaken >= delaytime) {        //here we make it easy to wrap the code we want to time in an "IF" statement, if not then it isn't and so doesn't get run.
    timerState[timerNumber] = 0; //once enough time has passed the timer is marked reset.
    return 1;                          //if enough time has passed the "IF" statement is true
  }
  else {                               //if enough time has not passed then the "if" statement will not be true.
    return 0;
  }
}

/****************************************************************
*Initialize Real Time Clock
*****************************************************************/
int RTC_init() {
  //pinMode(rtcs,OUTPUT); // chip select
  //pinMode(rtcs,OUTPUT); // chip select
  // start the SPI library:
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3); // both mode 1 & 3 should work
  //set control register
  digitalWrite(rtcs, LOW);
  SPI.transfer(0x8E);
  SPI.transfer(0x60); //60= disable Osciallator and Battery SQ wave @1hz, temp compensation, Alarms disabled
  digitalWrite(rtcs, HIGH);
  delay(10);
}

/****************************************************************
*SET Time and Date Real Time Clock
*****************************************************************/

int SetTimeDate(int d, int mo, int y, int h, int mi, int s) {
  int TimeDate [7] = {s, mi, h, 0, d, mo, y};
  for (int i = 0; i <= 6; i++) {
    if (i == 3)
      i++;
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

/****************************************************************
*READ Time and Date Real Time Clock
*****************************************************************/

String ReadTimeDate() {
  String temp;
  //int TimeDate [7]; //second,minute,hour,null,day,month,year
  for (int i = 0; i <= 6; i++) {
    if (i == 3)
      i++;
    SPI.setDataMode(SPI_MODE3);
    digitalWrite(rtcs, LOW);
    SPI.transfer(i + 0x00);
    unsigned int n = SPI.transfer(0x00);
    digitalWrite(rtcs, HIGH);
    int a = n & B00001111;
    if (i == 2) {
      int b = (n & B00110000) >> 4; //24 hour mode
      if (b == B00000010)
        b = 20;
      else if (b == B00000001)
        b = 10;
      TimeDate[i] = a + b;
    }
    else if (i == 4) {
      int b = (n & B00110000) >> 4;
      TimeDate[i] = a + b * 10;
    }
    else if (i == 5) {
      int b = (n & B00010000) >> 4;
      TimeDate[i] = a + b * 10;
    }
    else if (i == 6) {
      int b = (n & B11110000) >> 4;
      TimeDate[i] = a + b * 10;
    }
    else {
      int b = (n & B01110000) >> 4;
      TimeDate[i] = a + b * 10;
    }
  }

  temp.concat("20");
  temp.concat(TimeDate[6]);
  temp.concat("-") ;
  if (TimeDate[5] < 10) {
    temp.concat("0");
  }
  temp.concat(TimeDate[5]);
  temp.concat("-") ;
  if (TimeDate[4] < 10) {
    temp.concat("0");
  }
  temp.concat(TimeDate[4]);
  temp.concat("  ") ;
  if (TimeDate[2] < 10) {
    temp.concat("0");
  }
  temp.concat(TimeDate[2]);
  temp.concat(":") ;
  if (TimeDate[1] < 10) {
    temp.concat("0");
  }
  temp.concat(TimeDate[1]);
  temp.concat(":") ;
  if (TimeDate[0] < 10) {
    temp.concat("0");
  }
  temp.concat(TimeDate[0]);
  return (temp);
  SPI.setDataMode(SPI_MODE0);
}

/****************************************************************
* Read I/O
*****************************************************************/
void ReadIO() {
  DoorUpOvrd = digitalRead(Door_Up_Ovrd_Pin);
  DoorDownOvrd =digitalRead(Door_Down_Ovrd_Pin);
  DoorUpSwitch = digitalRead(Door_Up_Switch_Pin);
  DoorDownSwitch = digitalRead(Door_Down_Switch_Pin);
  UpdateDoorState();

                /**********************
                      * DEBUG
                **********************/  
  //Serial.println("I/O Updated");
  //Serial.print("Door Up Switch:"), Serial.println(DoorUpSwitch);
  //Serial.print("Door Up Button:"), Serial.println(DoorUpOvrd);
  //Serial.print("Door Down Switch:"), Serial.println(DoorDownSwitch);
  //Serial.print("Door Down Button:"), Serial.println(DoorDownOvrd);
  //Serial.print("2nd_Chance: "), Serial.println(Door_2nd_chance);
  //Serial.print("Door State: "), Serial.println(Door_State);
}

/****************************************************************
* Update Door State
*****************************************************************/
void UpdateDoorState() {
  // Door Not all the way closed or Open
  if ((DoorUpSwitch == 0) && (DoorDownSwitch == 0)) {
    if (delayMilliSeconds(2, 1000)) {
      (Door_State = DOOR_MOVING);
    }
  }
    // Door Completely Open
  if ((DoorUpSwitch == 1) && (DoorDownSwitch == 0)) {
    if (delayMilliSeconds(3, 1000)) {
      (Door_State = DOOR_OPEN);
    }
  }
  // Door Completely Closed
  if ((DoorUpSwitch == 0) && (DoorDownSwitch == 1)) {
    if (delayMilliSeconds(4, 1000)) {
      (Door_State = DOOR_CLOSED);
    }
  }
  
  switch (Door_State) {
    case 0:
      Serial.println("Door State: Door Has Not contacted either switch");
      break;
    case 1:
      Serial.println("Door State: Door is Open");
      break;
    case 2:
      Serial.println("Door State: Door is Closed");
      break;
  }  
}

/****************************************************************
*OPEN Coop Door Based on Time of Day
*****************************************************************/

void OpenCoopDoor() {
  // OPEN DOOR
  if (TimeDate[2] == OpenDoorHour && TimeDate[1] == OpenDoorMin && Door_State != 1) {
    if (delayMilliSeconds(5, 750)) {
      while (Door_State != 1) {
        md.setM1Speed(-400); // Turn on Motor
        //digitalWrite(Door_Open_Pin, HIGH); //Simulate Motor Output
        if (delayMilliSeconds(6, 750)) {
          ReadIO();   //Check Digital Signals and Update Variables
        }
        //Door_2nd_chance = 0;
      }
      md.setM1Speed(0); // Stop Motor
      //digitalWrite(Door_Open_Pin, LOW); //Simulate Motor Output
    }
  }
}

/****************************************************************
*OPEN Coop Door Based on Override Button
*****************************************************************/

void OverrideDoorOpen () {

  // OPEN DOOR
  if (DoorUpOvrd == 1 && Door_State != 1) {
    if (delayMilliSeconds(7, 250)) {
      while (DoorUpOvrd == 1 && Door_State != 1) {
        //OpenDoor();
        md.setM1Speed(-400); // Turn on Motor
        //digitalWrite(Door_Open_Pin, HIGH); //Simulate Motor Output
        if (delayMilliSeconds(8, 750)) {
          ReadIO();   //Check Digital Signals and Update Variables
        }  
      }
      md.setM1Speed(0);
      //digitalWrite(Door_Open_Pin, LOW); //Simulate Motor Output
    }
  }
}

/****************************************************************
*OPEN Coop Door Based on Inital Door Close Command and waiting 5 minutes, this lets any stragglers in
*****************************************************************/

void SecondChanceDoorOpen() {
  Serial.println("Second Chance Door Open");
  // 2nd CHANCE DOOR OPEN
  if (delayMilliSeconds(9, 60000)) {
    while (Door_State != 1) {
      //OpenDoor();
      md.setM1Speed(-400); // Turn on Motor
      //digitalWrite(Door_Open_Pin, HIGH); //Simulate Motor Output
      if (delayMilliSeconds(10, 750)) {
        ReadIO();   //Check Digital Signals and Update Variables
      }
    }
    Door_2nd_chance = 2;
    md.setM1Speed(0);
    //digitalWrite(Door_Open_Pin, LOW); //Simulate Motor Output
  }
}

/****************************************************************
*CLOSE Coop Door Based on Time of Day
*****************************************************************/

void CloseCoopDoor() {
  // CLOSE DOOR
  if (TimeDate[2] == CloseDoorHour && TimeDate[1] == CloseDoorMin && Door_State != 2) {
    if (delayMilliSeconds(11, 750)) {
      while (Door_State != 2) {
        md.setM1Speed(400); // Turn on Motor
        //digitalWrite(Door_Close_Pin, HIGH); //Simulate Motor Output
        if (delayMilliSeconds(12, 750)) {
          ReadIO();   //Check Digital Signals and Update Variables
        }
      }
      Door_2nd_chance = 1;
      md.setM1Speed(0); // Stop Motor
      //digitalWrite(Door_Close_Pin, LOW); //Simulate Motor Output
    }
  }
}

/****************************************************************
*CLOSE Coop Door Based on Override Button
*****************************************************************/

void OverrideDoorClose () {

  // CLOSE DOOR
  if (DoorDownOvrd == 1 && Door_State != 2) {
    if (delayMilliSeconds(13, 250)) {
      while (DoorDownOvrd == 1 && Door_State != 2) {
        //CloseDoor();
        md.setM1Speed(400); // Turn on Motor
        //digitalWrite(Door_Close_Pin, HIGH); //Simulate Motor Output
        if (delayMilliSeconds(14, 750)) {
          ReadIO();   //Check Digital Signals and Update Variables
        }  
      }
      md.setM1Speed(0);
      //digitalWrite(Door_Close_Pin, LOW); //Simulate Motor Output
    }
  }
}

/****************************************************************
*CLOSES Coop Door for final time of the evening
*****************************************************************/

void SecondChanceDoorClose() {
  Serial.println("Second Chance Door Close");
  // 2nd CHANCE DOOR CLOSE
  if (delayMilliSeconds(15,60000)) {
    while (Door_State != 2) {
      //CloseDoor();
      md.setM1Speed(400); // Turn on Motor
      //digitalWrite(Door_Close_Pin, HIGH); //Simulate Motor Output
      if (delayMilliSeconds(16, 750)) {
        ReadIO();   //Check Digital Signals and Update Variables
      }
    }
    Door_2nd_chance = 0;
    md.setM1Speed(0);
    //digitalWrite(Door_Close_Pin, LOW); //Simulate Motor Output
  }
}

/****************************************************************
*INDICATOR Lights Control for the Coop Door Status
*****************************************************************/

/*void Indlights() {
  // Green (turn just the green LED on)
  if (Door_Closed == 1) {
  digitalWrite(Red_Pin, LOW);
  digitalWrite(Green_Pin, HIGH);
  digitalWrite(Blue_Pin, LOW);
  }
  // Red (turn just the red LED on)
  if (Door_Open == 1) {
  digitalWrite(Red_Pin, HIGH);
  digitalWrite(Green_Pin, LOW);
  digitalWrite(Blue_Pin, LOW);
  }
}*/

/****************************************************************
*READ Temperature sensors and update temperature variables
*****************************************************************/

void CheckCoopTemp() {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
    //Serial.println("No more addresses.");
    //Serial.println();
    ds.reset_search();
    delay(50);
    return;
  }

  //Serial.print("ROM =");
  for ( i = 0; i < 2; i++) {
    //Serial.write(' ');
    //Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end

  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }

  celsius = (float)raw / 16.0;

  if (addr[1] == 94) { // 1st byte of the address of temperature probe
    WaterTemp = celsius * 1.8 + 32.0;
    Serial.print("Water Temperature = ");
    Serial.println(WaterTemp);
  }
  if (addr[1] == 174) { // 1st byte of the address of temperature probe
    //celsius = (float)raw / 16.0;
    CoopTemp = celsius * 1.8 + 32.0;
    Serial.print("Coop Temperature = ");
    Serial.println(CoopTemp);
  }
}

/****************************************************************
*COOP LOW TEMPERATURE Control
*****************************************************************/

void CoopTempControl() {
  if (FaultCode == 0) {
    if (CoopTemp <= 50) {
      digitalWrite(Ceiling_Heater_Pin, HIGH);
      Serial.println("Ceiling Heater ON");
    }
    else if (CoopTemp > 65) {
      digitalWrite(Ceiling_Heater_Pin, LOW);
      Serial.println("Ceiling Heater OFF");
    }
  }
}

/****************************************************************
*COOP WATER TEMPERATURE Control
*****************************************************************/

void WaterTempControl() {
  if (FaultCode == 0) {
    if (WaterTemp <= 40) {
      digitalWrite(Water_Heater_Pin, HIGH);
      Serial.println("Water Heater ON");
    }
    else if (WaterTemp > 60) {
      digitalWrite(Water_Heater_Pin, LOW);
      Serial.println("Water Heater OFF");
    }
  }
}

/****************************************************************
*READ Ammonia Sensor & CONTROL Air quality based on Temperature or Ammonia Level
*****************************************************************/

void CheckAirQuality() {
  // read the input on analog pin 2:
  int sensorValue = analogRead(A2);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  AmmoniaLevel = sensorValue * (5.0 / 1023.0);
  // print out the value you read:
  Serial.print("Air Sensor Reading:"), Serial.println(AmmoniaLevel);
  //delay(1000);

  //if (FaultCode==0){
  //Serial.print("KeepFanOn: "), Serial.println(KeepFanOn);
  //Serial.print("KeepFanOff: "), Serial.println(KeepFanOff);
  //if (AmmoniaLevel > 2.5 || CoopTemp >= 85 && KeepFanOff != 1) {
  if ((AmmoniaLevel > 2.5) || (CoopTemp >= 84)) {
    digitalWrite(Exhaust_Fan_Pin, HIGH);
    Serial.println("Exhaust Fan ON");
    KeepFanOn = 1;
  }
  //else if (AmmoniaLevel < .5 || CoopTemp < 75 && KeepFanOn != 1) {
  if ((AmmoniaLevel < .5 && CoopTemp < 80) || (CoopTemp < 80)) {
    digitalWrite(Exhaust_Fan_Pin, LOW);
    Serial.println("Exhaust Fan OFF");
    KeepFanOff = 1;
  }
  //}
}

/****************************************************************
*FLASH Red Fault LED
*****************************************************************/

void BlinkRedLED(int nr) // Flash Fault LED to indicate fault code specified above
{
  for (int i = 0; i < nr; i++)
  {
    digitalWrite(Fault_LED_Pin, HIGH);
    delay(200);
    digitalWrite(Fault_LED_Pin, LOW);
    delay(200);
  }
}

/****************************************************************
*FAULT STATUS

# of LED Flashes
1 - Temperature sensor is not working
2 - Motor Faulted
3 - Ammonia sensor is not working
4 - Real Time Clock does not have a valid time
5 - Door Open but should be closed
6 - Door Closed but should be open

*****************************************************************/

void CheckFaults () {

  if (CoopTemp == 0) {
    FaultCode = 1;
  }
  /*else if (WaterTemp == 0) {
    FaultCode = 2;
  }*/
  else if (AmmoniaLevel == 0) {
    FaultCode = 3;
  }
  else if (TimeDate[6] == 0) {
    FaultCode = 4;
  } else {
    FaultCode = 0;
  }
  //Serial.print("Fault Code: "), Serial.println(FaultCode);

  if (FaultCode > 0 && delayMilliSeconds(17, 5000)) {
    BlinkRedLED((int)(FaultCode));  // flash  red
  }
  //delay(1000);
}

/****************************************************************
*Outdoor Light Control
*****************************************************************/

void OutsideLight () {

  if ((Door_State == DOOR_CLOSED) || (TimeDate[2] >= CloseDoorHour && TimeDate[2] <= OpenDoorHour)) {
    digitalWrite(Outside_Light_Pin, HIGH);
    //Serial.println("Outside Light ON");
  }
  if ((Door_State == DOOR_OPEN) || (TimeDate[2] >= OpenDoorHour && TimeDate[2] <= CloseDoorHour)) {
    digitalWrite(Outside_Light_Pin, LOW);
    //Serial.println("Outside Light OFF");
  }
}

/****************************************************************
*Inside Light Control
*****************************************************************/

void InsideLight () {

  if ((TimeDate[2] >= 18) && (TimeDate[2] <= 20)) { // if the hour is between 6 & 8pm turn light on
    digitalWrite(Inside_Light_Pin, HIGH);
  }
  else {
    digitalWrite(Inside_Light_Pin, LOW);
  }
}

/****************************************************************
*Exhaust Fan On/Off Timer Control
*****************************************************************/

void ExhFan () {

  if (KeepFanOn == 1 && CoopTemp < CoopTemp_FanHighLimit) { // if the Fan has been turned on, wait 15 minutes before allowing it to turn off
    if (delayMinutes(13, 1)) {
      Serial.println("Timer to keep Fan on Finished");
      Serial.println("");
      KeepFanOn = 0;
    }
  }
  if (KeepFanOff == 1 && CoopTemp > CoopTemp_FanHighLimit ) { // if the Fan has been turned off, wait 15 minutes before allowing it to turn on
    if (delayMinutes(14, 1)) {
      Serial.println("Timer to keep Fan off Finished");
      Serial.println("");
      KeepFanOff = 0;
    }
  }
}


//*****************************************************************//
//

void GetAnalogTemp () {

  float voltage, degreesC;
  voltage = getVoltage(temperaturePin);
  degreesC = (voltage - 0.5) * 100.0;
  CoopTemp = degreesC * (9.0 / 5.0) + 32.0;

}


float getVoltage(int pin)
{
  return (analogRead(pin) * 0.004882814); //converts counts to voltage
}

//*****************************************************************//
