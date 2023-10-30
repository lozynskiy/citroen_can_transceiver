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

struct BUTTON {
  byte id;
  int byteNum;
  byte byteValue;
  int resistance;
};

BUTTON VOL_UP             = {0x21F, 0, 0x08, 0};
BUTTON VOL_DOWN           = {0x21F, 0, 0x04, 2};
BUTTON MUTE               = {0x21F, 0, 0x0C, 3};
BUTTON NEXT               = {0x21F, 0, 0x40, 4};
BUTTON PREVIUOS           = {0x21F, 0, 0x80, 5};
BUTTON LIST               = {0x21F, 0, 0x01, 0};

BUTTON BACK               = {0x0A2, 1, 0x01, 7};
BUTTON HOME               = {0x0A2, 1, 0x01, 8};
BUTTON SOURCE             = {0x0A2, 1, 0x01, 2};
BUTTON SCROLL_PRESSED     = {0x0A2, 1, 0x01, 5};
BUTTON SCROLL_DOWN        = {0x0A2, 0, NULL, 6};
BUTTON SCROLL_UP          = {0x0A2, 0, NULL, 7};

BUTTON PHONE              = {0x0A2, 2, 0x80, 10};
BUTTON VOICE_ASSIST       = {0x221, 0, 0x01, 4};
BUTTON NIGHT_MODE         = {0x036, 3, 0x36, 17};

BUTTON WHEEL_BUTTON[3] = {
  // VOL_UP, 
  // VOL_DOWN, 
  // MUTE, 
  // NEXT, 
  // PREVIUOS, 
  LIST,
  // BACK, 
  // HOME, 
  SOURCE, 
  // SCROLL_PRESSED, 
  // SCROLL_DOWN, 
  // SCROLL_UP,
  // PHONE, 
  VOICE_ASSIST
  // NIGHT_MODE
};

CAN_PACKAGE dataPackages[3] = {
  {0x1A5, 1, {0x14}, 500, 0},                                       // set volume
  {0x165, 4, {0xC0, 0x00, 0x40, 0x00}, 100, 0},                     // enable amplifier
  {0x1E5, 7, {0x3F, 0x3F, 0x43, 0x3F, 0x44, 0x47, 0x40}, 500, 0}    // set equalizer
};

int dataPackagesCount = 3;


// buttons resistance configuration

/*

// int BTN_VOL_UP = 0;
// int BTN_VOL_DOWN = 2;
// int BTN_MUTE = 3;
// int BTN_NEXT = 4;
// int BTN_PREVIUOS = 5;
int BTN_LIST = 0;
// int BTN_BACK = 7;
// int BTN_HOME = 8;
int BTN_SOURCE = 2;
// int BTN_PHONE = 10;
int BTN_VOICE_ASSIST = 4;
// int BTN_NIGHT_MODE = 17;
int BTN_SCROLL_PRESSED = 5;
int BTN_SCROLL_DOWN = 6;
int BTN_SCROLL_UP = 7;

*/

// button state
int BTN_PRESSED = -1;
int BTN_RELEASED = 32;

unsigned long btnPressedOn = 0;
int btnReleaseAfter = 150;
byte scrollWheelPosition;

int sleepDelay = 3000;
unsigned long sleepReceivedOn = 0;
const int serialMaximumInput = 50;

unsigned long rxId;
byte len;
byte rxBuf[8];
bool sleep = false;

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

void stringSplit(String * strs, String value, char separator) {
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

  for (int i = 0; i < dataPackages[packageIndex].dlc; i++){
    data[i] = parseAsHex(dataParts[i]);
  }

  memcpy(dataPackages[packageIndex].data, data, 8);

  saveConfig();
}

int getPackageIndex(unsigned long id){
  int index = -1;

  for (int i = 0; i < dataPackagesCount; i++){
    if(id == dataPackages[i].id){
      index = i;

      break;
    }
  }

  return index;
}

void saveConfig(){
  int address = 0;

  for (int i = 0; i < dataPackagesCount; i++){
    EEPROM.put(address, dataPackages[i]);
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
      memcpy(dataPackages[i].data, loadedData.data, 8);
    }

    address += sizeof(CAN_PACKAGE);
  }

  Serial.println("config loaded");
}

void getConfig(){
  for (int i = 0; i < dataPackagesCount; i++){
    String data = "";

    for (int d = 0; d < dataPackages[i].dlc; d++){
      data += " 0x";

      if (dataPackages[i].data[d] < 16) 
        data += "0";
      
      data += String(dataPackages[i].data[d], HEX);
    }

    data.trim();

    Serial.println("0x" + String(dataPackages[i].id, HEX) + ";" + data);
  }
}

bool isForbidden(unsigned long id){
  if (id == 0x165) return true;   // enable amplifier
  if (id == 0x1E5) return true;   // equalizer
  if (id == 0x1A5) return true;   // volume

  return false;
}

void sendData(){
  for (int i = 0; i < dataPackagesCount; i++){
    if(millis() - dataPackages[i].lastMillis >= dataPackages[i].period){
      CAN0.sendMsgBuf(dataPackages[i].id, 0, dataPackages[i].dlc, dataPackages[i].data);
      dataPackages[i].lastMillis = millis();
    }
  }
}

void checkIgnition() {
  if (rxId == 0x0F6) {
    if ((rxBuf[0] & 0x08) && sleep == true) {
      sleep = false;
      sleepReceivedOn = 0;

      Serial.println("ignition on"); 

      return;
    }
    
    if (sleep) return;

    if ((rxBuf[0] & 0x08) && sleep == false && sleepReceivedOn == 0){
      sleepReceivedOn = millis();
    }
  }

  if (sleepReceivedOn == 0 || sleep == true) return;

  // delay sleep
  if (millis() - sleepReceivedOn > sleepDelay && sleep == false) {
    sleep = true;

    Serial.println("ignition off"); 
  }
}

void processKey() {

  if (!(rxId == 0x21F || rxId == 0x0A2 || rxId == 0x221 || rxId == 0x036)){
    return;
  }

  for (int i = 0; i < sizeof (WHEEL_BUTTON) / sizeof (BUTTON); i++){
    if (rxId == WHEEL_BUTTON[i].id && rxBuf[WHEEL_BUTTON[i].byteNum] == WHEEL_BUTTON[i].byteValue) {
      pressKey(WHEEL_BUTTON[i].resistance);
      rxBuf[WHEEL_BUTTON[i].byteNum] = 0x00;

      return;
    }
  }

  if (rxId == 0x0A2) {
    if (!scrollWheelPosition) {
      scrollWheelPosition = rxBuf[0];
    }

    if (scrollWheelPosition != rxBuf[0]) {
      if (rxBuf[0] > scrollWheelPosition){
        pressKey(SCROLL_UP.resistance);
      } else {
        pressKey(SCROLL_DOWN.resistance);
      }
      scrollWheelPosition = rxBuf[0];
      rxBuf[0] = 0x00;
    }
  }
  
  // if no press detected for more than period - release key
  if (millis() - btnPressedOn > btnReleaseAfter) {
    pressKey(BTN_RELEASED);
  }
}

void pressKey(int key) {
  btnPressedOn = millis();

  if (BTN_PRESSED != key)
  {
    BTN_PRESSED = key;
    POTENTIOMETER.set(BTN_PRESSED);

    Serial.println("pressed key: " + String(BTN_PRESSED));
  }
}

void processIncomingData (const char * data)
 {
  String line = String(data);

  if (line == "get config"){
    getConfig();
  } else {
    parseConfig(line);
  }
}

String processIncomingByte (const byte inByte)
{
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

  return String();
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
    processIncomingByte (Serial.read());
  }
  
  if(!digitalRead(CAN1_INT)){
    CAN1.readMsgBuf(&rxId, &len, rxBuf);

    if(!isForbidden(rxId))
    {
      CAN0.sendMsgBuf(rxId, 0, len, rxBuf);
    }
  }

  sendData();
}

void loop(){
  processCan();
}
