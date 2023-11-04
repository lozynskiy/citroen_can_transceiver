#include <mcp_can.h>
#include <SPI.h>
#include <EEPROM.h>
#include <DigiPotX9Cxxx.h>

struct CAN_PACKAGE {
  unsigned long id;
  byte dlc;
  byte data[8];
  int period;
  unsigned long lastMillis;
};

struct IGNITION {
  unsigned long id = 0x0F6;
  int byteNum = 0;
  byte byteValue = 0x08;
} IGNITION;

struct BUTTON {
  unsigned long id;
  int byteNum;
  byte byteValue;
  int resistance;
};

struct SCROLL {
  unsigned long id;
  int byteNum;
  int up;
  int down;
};

SCROLL SCROLL;//             = {0x0A2, 0, 20, 23};

BUTTON LIST               = {0x21F, 0, 0x01, 0};
BUTTON VOL_UP             = {0x21F, 0, 0x08, 3};
BUTTON VOL_DOWN           = {0x21F, 0, 0x04, 4};
BUTTON MUTE               = {0x21F, 0, 0x0C, 5};
BUTTON NEXT               = {0x21F, 0, 0x40, 6};
BUTTON PREVIUOS           = {0x21F, 0, 0x80, 7};

BUTTON SOURCE             = {0x0A2, 1, 0x04, 2};
BUTTON BACK               = {0x0A2, 1, 0x10, 8};
BUTTON HOME               = {0x0A2, 1, 0x08, 10};
BUTTON SCROLL_PRESSED     = {0x0A2, 1, 0xA0, 12};
BUTTON PHONE              = {0x0A2, 2, 0x80, 14};

BUTTON VOICE_ASSIST       = {0x221, 0, 0x01, 4};
BUTTON NIGHT_MODE         = {0x036, 3, 0x36, 17};

BUTTON WHEEL_BUTTON[] = {
  // VOL_UP, 
  // VOL_DOWN, 
  // MUTE, 
  // NEXT, 
  // PREVIUOS, 
  // BACK, 
  // HOME, 
  // SCROLL_PRESSED, 
  // PHONE, 
  // NIGHT_MODE,
  LIST,
  SOURCE, 
  VOICE_ASSIST
};

// button state
int BTN_PRESSED = -1;
int BTN_RELEASED = 32;

unsigned long btnPressedOn = 0;
int btnReleaseAfter = 150;
byte scrollPosition;

int buttonsCount = sizeof(WHEEL_BUTTON) / sizeof(BUTTON);

CAN_PACKAGE CAN_VOLUME = {0x1A5, 1, {0x14}, 500, 0};

CAN_PACKAGE CAN_PACKAGES[] = {
  CAN_VOLUME,                                                       // set volume
  {0x165, 4, {0xC0, 0x00, 0x40, 0x00}, 100, 0},                     // enable amplifier
  {0x1E5, 7, {0x3F, 0x3F, 0x43, 0x3F, 0x44, 0x47, 0x40}, 500, 0}    // set equalizer
};

int dataPackagesCount = sizeof(CAN_PACKAGES) / sizeof(CAN_PACKAGE);
bool useDynamicVolume = true;

int sleepDelay = 2000;
unsigned long sleepReceivedOn = 0;

byte can1ToCan0BlockedPackages[] = {0x1A5, 0x165, 0x1E5}; // block amplifier related packages received from CAN adapter
int can1ToCan0BlockedPackagesCount = sizeof(can1ToCan0BlockedPackages) / sizeof(byte);

unsigned long rxId;
byte len;
byte rxBuf[8];

DigiPot POTENTIOMETER(4,5,6);

MCP_CAN CAN0(9);                              // CAN0 interface usins CS on digital pin 2
MCP_CAN CAN1(10);                             // CAN1 interface using CS on digital pin 3

#define CAN0_INT 3    //define interrupt pin for CAN0 recieve buffer
#define CAN1_INT 2    //define interrupt pin for CAN1 recieve buffer

void setup()
{
  Serial.begin(115200);
  
  pinMode(CAN0_INT, INPUT_PULLUP);
  pinMode(CAN1_INT, INPUT_PULLUP);
  
  // init CAN0 bus, baudrate: 125k@8MHz
  if(CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK){
    Serial.print("CAN0: Init OK!\r\n");
    CAN0.setMode(MCP_NORMAL);
  } else Serial.print("CAN0: Init Fail!!!\r\n");
  
  // init CAN1 bus, baudrate: 125k@8MHz
  if(CAN1.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK){
    Serial.print("CAN1: Init OK!\r\n");
    CAN1.setMode(MCP_NORMAL);
  } else Serial.print("CAN1: Init Fail!!!\r\n");

  loadConfig();
}

void stringSplit(String * strs, String value, char separator){
  int count = 0;

  while (value.length() > 0)
  {
    int index = value.indexOf(separator);
    if (index == -1)
    {
      strs[count++] = value;
      break;
    }
    else
    {
      strs[count++] = value.substring(0, index);
      value = value.substring(index + 1);
    }
  }
}

unsigned long parseAsHex(String value){
  char hex[10];
  value.toCharArray(hex, 10);
  return strtoul (hex, NULL, 16);
}

void parseConfig(String config){
  String parts[10];
  stringSplit(parts, config, ';');

  int packageIndex = getPackageIndex(parseAsHex(parts[0]));

  if (packageIndex == -1){
    Serial.println("wrong value");
    return;
  }

  String dataParts[8];
  byte data[8];
  stringSplit(dataParts, parts[1], ' ');

  for (int i = 0; i < CAN_PACKAGES[packageIndex].dlc; i++){
    data[i] = parseAsHex(dataParts[i]);
  }

  memcpy(CAN_PACKAGES[packageIndex].data, data, 8);

  saveConfig();
}

int getPackageIndex(unsigned long id){
  int index = -1;

  for (int i = 0; i < dataPackagesCount; i++){
    if(id == CAN_PACKAGES[i].id){
      index = i;

      break;
    }
  }

  return index;
}

void saveConfig(){
  int address = 0;

  for (int i = 0; i < dataPackagesCount; i++){
    EEPROM.put(address, CAN_PACKAGES[i]);
    address += sizeof(CAN_PACKAGE);
  }

  Serial.println("config saved");
}

void loadConfig(){
  int address = 0;
  int index;

  CAN_PACKAGE loadedData;

  for (int i = 0; i < dataPackagesCount; i++){
    EEPROM.get(address, loadedData);

    index = getPackageIndex(loadedData.id);

    if (index >= 0){
      memcpy(CAN_PACKAGES[i].data, loadedData.data, 8);
    }

    if (loadedData.id == CAN_VOLUME.id){
      memcpy(CAN_VOLUME.data, loadedData.data, 8);
    }

    address += sizeof(CAN_PACKAGE);
  }

  Serial.println("config loaded");
}

void getConfig(){
  for (int i = 0; i < dataPackagesCount; i++){
    String data = "";

    for (int d = 0; d < CAN_PACKAGES[i].dlc; d++){
      data += " 0x";

      if (CAN_PACKAGES[i].data[d] < 16) 
        data += "0";
      
      data += String(CAN_PACKAGES[i].data[d], HEX);
    }

    data.trim();

    Serial.println("0x" + String(CAN_PACKAGES[i].id, HEX) + ";" + data);
  }
}

bool isForbidden(){
  for (int i = 0; i < can1ToCan0BlockedPackagesCount; i++){
    if (rxId == can1ToCan0BlockedPackages[i]) return true;
  }

  return false;
}

void sendData(){
  for (int i = 0; i < dataPackagesCount; i++){
    if(millis() - CAN_PACKAGES[i].lastMillis >= CAN_PACKAGES[i].period){
      if (useDynamicVolume && CAN_PACKAGES[i].id == CAN_VOLUME.id){
        CAN0.sendMsgBuf(CAN_VOLUME.id, 0, CAN_VOLUME.dlc, CAN_VOLUME.data);
      } else {
        CAN0.sendMsgBuf(CAN_PACKAGES[i].id, 0, CAN_PACKAGES[i].dlc, CAN_PACKAGES[i].data);
      }
      CAN_PACKAGES[i].lastMillis = millis();
    }
  }
}

void checkIgnition(){
  if (rxId == IGNITION.id){
    if ((rxBuf[IGNITION.byteNum] & IGNITION.byteValue) && (sleep == true || sleepReceivedOn != 0)){
      sleep = false;
      sleepReceivedOn = 0;

      Serial.println("ignition on"); 

      return;
    }
    
    if (sleep) return;

    if (!(rxBuf[IGNITION.byteNum] & IGNITION.byteValue) && sleep == false && sleepReceivedOn == 0){
      sleepReceivedOn = millis();
    }
  }

  if (sleepReceivedOn == 0 || sleep == true) return;

  // delay sleep
  if (millis() - sleepReceivedOn > sleepDelay && sleep == false){
    sleep = true;

    Serial.println("ignition off"); 
  }
}

void processKey(){
  for (int i = 0; i < buttonsCount; i++){
    if (rxId == WHEEL_BUTTON[i].id && rxBuf[WHEEL_BUTTON[i].byteNum] == WHEEL_BUTTON[i].byteValue){
      pressKey(WHEEL_BUTTON[i].resistance);
      rxBuf[WHEEL_BUTTON[i].byteNum] = 0x00;

      return;
    }
  }

  if (rxId == SCROLL.id){
    if (!scrollPosition){
      scrollPosition = rxBuf[SCROLL.byteNum];
    }

    if (scrollPosition != rxBuf[SCROLL.byteNum]){
      if (rxBuf[SCROLL.byteNum] > scrollPosition){
        pressKey(SCROLL.up);
      } else {
        pressKey(SCROLL.down);
      }
      scrollPosition = rxBuf[SCROLL.byteNum];
      rxBuf[SCROLL.byteNum] = 0x00;
    }

    return;
  }
  
  // if no press detected for more than period - release key
  if (millis() - btnPressedOn > btnReleaseAfter){
    pressKey(BTN_RELEASED);
  }
}

void pressKey(int key){
  btnPressedOn = millis();

  if (BTN_PRESSED != key)
  {
    BTN_PRESSED = key;
    POTENTIOMETER.set(BTN_PRESSED);

    Serial.println("pressed key: " + String(BTN_PRESSED));
  }
}

void processIncomingData (const char * data){
  String line = String(data);

  if (line == "get config"){
    getConfig();
  } else {
    parseConfig(line);
  }
}

void processIncomingByte (const byte inByte){
  const int serialMaximumInput = 50;
  static char inputLine [serialMaximumInput];
  static unsigned int position = 0;

  switch (inByte)
  {
    case '\n':
      inputLine [position] = 0;
      processIncomingData (inputLine);

      position = 0;  
      break;
    case '\r':
      break;
    default:
      if (position < (serialMaximumInput - 1))
        inputLine [position++] = inByte;
      break;
  }
}

void setDynamicVolume(){
  if (!useDynamicVolume){
    return;
  }
  
  if (rxId == CAN_VOLUME.id && (rxBuf[0] >= 0 && rxBuf[0] <= 30)){
    memcpy(CAN_VOLUME.data, rxBuf, 8);
  }
}

void processCan(){

  if(!digitalRead(CAN0_INT)){
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    checkIgnition();

    if (sleep) return;

    processKey();

    CAN1.sendMsgBuf(rxId, 0, len, rxBuf);
  }

  if (sleep) return;

  if (Serial.available()){
    processIncomingByte(Serial.read());
  }
  
  if(!digitalRead(CAN1_INT)){
    CAN1.readMsgBuf(&rxId, &len, rxBuf);

    setDynamicVolume();

    if(!isForbidden()){
      CAN0.sendMsgBuf(rxId, 0, len, rxBuf);
    }
  }

  sendData();
}

void loop(){
  processCan();
}
