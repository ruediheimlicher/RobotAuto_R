/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Servo.h>

#include <Ticker.h> 
#include "elapsedMillis.h"
#include "expo.h"

// von ESP32 ROBOTAUTO

#define NUM_SERVOS 4
uint16_t          servomittearray[NUM_SERVOS] = {}; // Werte fuer Mitte

uint16_t maxwinkel = 180;


uint8_t buttonstatus = 0;
uint8_t tonindex = 0;
void playTon(int ton);
#define START_TON 0
#define LICHT_ON 1

uint8_t expolevel = 0;

uint16_t ubatt = 0;

int ledintervall = 1000;
Ticker timer;
elapsedMillis ledmillis;


struct ServoPins
{
  Servo servo;
  int servoPin;
  String servoName;
  int initialPosition;  
};
std::vector<ServoPins> servoPins = 
{
  { Servo(), 12 , "Dir", 90}, // Richtung
  { Servo(), 14 , "Pitch", 90}, // Gas
  { Servo(), 13 , "Elbow", 90},
  { Servo(), 15 , "Gripper", 90},
};

struct RecordedStep
{
  int servoIndex;
  int value;
  int delayInStep;
};
std::vector<RecordedStep> recordedSteps;

bool recordSteps = false;
bool playRecordedSteps = false;

unsigned long previousTimeInMilli = millis();



void writeServoValues(int servoIndex, int value)
{
  if (recordSteps)
  {
    RecordedStep recordedStep;       
    if (recordedSteps.size() == 0) // We will first record initial position of all servos. 
    {
      for (int i = 0; i < servoPins.size(); i++)
      {
        recordedStep.servoIndex = i; 
        recordedStep.value = servoPins[i].servo.read(); 
        recordedStep.delayInStep = 0;
        recordedSteps.push_back(recordedStep);         
      }      
    }
    unsigned long currentTime = millis();
    recordedStep.servoIndex = servoIndex; 
    recordedStep.value = value; 
    recordedStep.delayInStep = currentTime - previousTimeInMilli;
    recordedSteps.push_back(recordedStep);  
    previousTimeInMilli = currentTime;         
  }
  Serial.printf("pin: [%d] servoindex: [%d] value: [%d]  \n",servoPins[servoIndex].servoPin , servoIndex, value);
  servoPins[servoIndex].servo.write(value);   
}



//Structure example to receive data
//Must match the sender structure
typedef struct canal_struct 
{
  uint16_t lx;
  uint16_t ly;
  uint16_t rx;
  uint16_t ry;
 

  int x;
  int y;
} canal_struct;

//Create a struct_message called canaldata
canal_struct canaldata;

//callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) 
{
  // blink
  digitalWrite(13,!digitalRead(13));


  memcpy(&canaldata, incomingData, sizeof(canaldata));
  /*Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("x: ");
  Serial.print(canaldata.x);
  Serial.print(" ");
  */
  
   Serial.print(canaldata.lx);
  Serial.print(" ");
  Serial.print(map(canaldata.lx,0,4095, 0,255));
  //Serial.println();
  //Serial.print(" ");
  //Serial.print(map(canaldata.ly,0,4095, 0,255));
  
  int valueInt = map(canaldata.lx,0,4095, 0,180);
  Serial.print(" valueInt: ");
  Serial.print(valueInt);

  
  
  int expovalue = 0;
  if (valueInt > maxwinkel/2)
  {
     expovalue = maxwinkel/2 +  expoarray[expolevel][valueInt - maxwinkel/2];
  }
  else
  {
     expovalue = maxwinkel/2 -  expoarray[expolevel][maxwinkel/2 - valueInt ];
  }
  
   Serial.print(" expovalue: ");
  Serial.println(expovalue);
  
  writeServoValues(1, 90 + ((180 - expovalue) - 90)/1); // red Ausschlaege
  //  writeServoValues(1, 90 + ((180 - expovalue) - 90)/1);

}

void setUpPinModes()
{
  for (int i = 0; i < servoPins.size(); i++)
  {
    servoPins[i].servo.attach(servoPins[i].servoPin);
    servoPins[i].servo.write(servoPins[i].initialPosition);    
  }
 
  
}


 
void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  setUpPinModes();
  pinMode(12, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(13, OUTPUT);
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() 
{

}
