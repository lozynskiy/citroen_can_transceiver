/*
Equalizer config:
  ID: 1E5
  DLC: 7
  Data:
    0: balance  (36 - 48)
    1: fader    (36 - 48)
    2: bass     (36 - 48)
    3: -        (3f)
    4: treble   (36 - 48)
    5: ludness + speed
      false + false = 00
      true + false = 40
      true + true = 47
      false + true = 07
    6: preset: 
      40 - Lineral; 
      44 - Classic; 
      48 - Jazz; 
      4C - Rock/Pop; 
      54 - Techno; 
      50 - Vocal

Volume:
  ID: 1A5
  DLC: 1
  Data:
    0: 00 - 1E
*/

#include <mcp_can.h>
#include <SPI.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <MCP41_Simple.h>

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
  bool on = false;
  int offDelay = 2500;
  unsigned long switchedOffTime = 0;
  unsigned long switchedOnTime = 0;
} IGNITION;

struct BUTTON {
  unsigned long id;
  int byteNum;
  byte byteValue;
  int resistance;
};

struct SCROLL {
  unsigned long id;
  byte position;
  int byteNum;
  int up;
  int down;
};

SCROLL SCROLL;//             = {0x0A2, NULL, 0, 26, 29};

BUTTON LIST               = {0x21F, 0, 0x01, 2};
BUTTON VOL_UP             = {0x21F, 0, 0x08, 3};
BUTTON VOL_DOWN           = {0x21F, 0, 0x04, 6};
BUTTON MUTE               = {0x21F, 0, 0x0C, 8};
BUTTON NEXT               = {0x21F, 0, 0x40, 10};
BUTTON PREVIUOS           = {0x21F, 0, 0x80, 12};

BUTTON SOURCE             = {0x0A2, 1, 0x04, 4};
BUTTON BACK               = {0x0A2, 1, 0x10, 14};
BUTTON HOME               = {0x0A2, 1, 0x08, 16};
BUTTON SCROLL_PRESSED     = {0x0A2, 1, 0xA0, 18};
BUTTON PHONE              = {0x0A2, 2, 0x80, 21};

BUTTON VOICE_ASSIST       = {0x221, 0, 0x01, 6};
BUTTON NIGHT_MODE         = {0x036, 3, 0x36, 23};

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

unsigned long btnPressedOn = 0;
int btnReleaseDelay = 150;

int buttonsCount = sizeof(WHEEL_BUTTON) / sizeof(BUTTON);
int buttonState = 255;
int buttonReleased = 255;

CAN_PACKAGE CAN_VOLUME = {0x1A5, 1, {0x14}, 500, 0};
CAN_PACKAGE CAN_AMPLIFIER = {0x165, 4, {0xC0, 0xC0, 0x60, 0x00}, 100, 0};

CAN_PACKAGE CAN_PACKAGES[] = {
  CAN_VOLUME,                                                       // set volume
  CAN_AMPLIFIER,                                                    // enable amplifier
  {0x1E5, 7, {0x3F, 0x3F, 0x43, 0x3F, 0x44, 0x40, 0x40}, 500, 0}    // set equalizer config
};

int dataPackagesCount = sizeof(CAN_PACKAGES) / sizeof(CAN_PACKAGE);

// amplifier configs
bool useDynamicVolume = false;
int dynamicVolumeByteNum = 0;
int amplifierMaxVolume = 28;
int amplifierVolumeOffset = 5;
unsigned long amplifierPowerOnDelay = 2000;

// power down option
unsigned long lastActivityOn = 0;
unsigned long powerDownDelay = 5000;

unsigned long rxId;
byte len;
byte rxBuf[8];

MCP_CAN CAN0(9);                // CAN0 interface usins CS on digital pin 9
MCP_CAN CAN1(10);               // CAN1 interface using CS on digital pin 10
MCP41_Simple Potentiometer;

#define CAN0_INT 3              // define interrupt pin for CAN0 recieve buffer
#define CAN1_INT 2              // define interrupt pin for CAN1 recieve buffer
#define POT_CS 8                // digital potentiometer CS pin
#define MCP_2551_RS 4           // MCP2551 RS pin

void setup()
{
  // noInterrupts();
  // CLKPR = _BV(CLKPCE);  // enable change of the clock prescaler
  // CLKPR = _BV(CLKPS0);  // divide frequency by 2
  // interrupts();

  Serial.begin(38400);
  Potentiometer.begin(POT_CS);

  pinMode(CAN0_INT, INPUT);
  pinMode(CAN1_INT, INPUT);
  pinMode(MCP_2551_RS, OUTPUT);
  
  setupCanControllers();

  attachInterrupt(digitalPinToInterrupt(CAN0_INT), ISR_CAN, FALLING);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  loadConfig();
  Potentiometer.setWiper(buttonReleased); 
}

void setupCanControllers(){
  int connectionEstablishRetryCount = 3;

  // init CAN0 bus, baudrate: 125k@8MHz
  for (int i = 1; i <= connectionEstablishRetryCount; i ++){
    if(CAN0.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK){
      Serial.println("CAN0: Init OK!");
      CAN0.setMode(MCP_NORMAL);
      CAN0.setSleepWakeup(1);
      break;
    } else {
      // Serial.println("CAN0: Init Fail!!!, retry in 100 ms");
      delay(100);
    }
  }
  
  // init CAN1 bus, baudrate: 125k@8MHz
  for (int i = 1; i <= connectionEstablishRetryCount; i ++){
    if(CAN1.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK){
      Serial.println("CAN1: Init OK!");
      CAN1.setMode(MCP_NORMAL);
      break;
    } else {
      // Serial.println("CAN1: Init Fail!!!, retry in 100 ms");
      delay(100);
    }
  }

  // Pull the Rs pin of the MCP2551 transceiver low to enable it
  digitalWrite(MCP_2551_RS, LOW);
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
  for (int i = 0; i < dataPackagesCount; i++){
    if (rxId == CAN_PACKAGES[i].id) return true;
  }

  return false;
}

void sendData(){
  for (int i = 0; i < dataPackagesCount; i++){
    if(millis() - CAN_PACKAGES[i].lastMillis >= CAN_PACKAGES[i].period){

      if (millis() - IGNITION.switchedOnTime < amplifierPowerOnDelay && CAN_PACKAGES[i].id == CAN_AMPLIFIER.id){
        continue;
      }

      if (useDynamicVolume && CAN_PACKAGES[i].id == CAN_VOLUME.id){
        CAN0.sendMsgBuf(CAN_VOLUME.id, CAN_VOLUME.dlc, CAN_VOLUME.data);
      } else {
        CAN0.sendMsgBuf(CAN_PACKAGES[i].id, CAN_PACKAGES[i].dlc, CAN_PACKAGES[i].data);
      }

      CAN_PACKAGES[i].lastMillis = millis();
    }
  }
}

void checkIgnition(){
  if (rxId == IGNITION.id){
    if ((rxBuf[IGNITION.byteNum] & IGNITION.byteValue) && (IGNITION.on == false || IGNITION.switchedOffTime != 0)){
      // handle quick off/on case, set values only if previous ignition state is off for longer than IGNITION.offDelay
      if (IGNITION.on == false){
        IGNITION.on = true;
        IGNITION.switchedOnTime = millis();
      }
      
      IGNITION.switchedOffTime = 0;
      Serial.println("ignition on"); 

      return;
    }
    
    if (!IGNITION.on) return;

    if (!(rxBuf[IGNITION.byteNum] & IGNITION.byteValue) && IGNITION.on == true && IGNITION.switchedOffTime == 0){
      IGNITION.switchedOffTime = millis();
    }
  }

  if (IGNITION.switchedOffTime == 0 || IGNITION.on == false) return;

  // delay sleep
  if (millis() - IGNITION.switchedOffTime > IGNITION.offDelay && IGNITION.on == true){
    IGNITION.on = false;

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
    if (!SCROLL.position){
      SCROLL.position = rxBuf[SCROLL.byteNum];
    }

    if (SCROLL.position != rxBuf[SCROLL.byteNum]){
      if (rxBuf[SCROLL.byteNum] > SCROLL.position){
        pressKey(SCROLL.up);
      } else {
        pressKey(SCROLL.down);
      }
      SCROLL.position = rxBuf[SCROLL.byteNum];
      rxBuf[SCROLL.byteNum] = 0x00;

      return;
    }
  }
  
  // if no press detected for more than period - release key
  if (millis() - btnPressedOn > btnReleaseDelay){
    pressKey(buttonReleased);
  }
}

void pressKey(int key){
  btnPressedOn = millis();

  if (buttonState != key)
  {
    buttonState = key;

    Potentiometer.setWiper(key);
    Serial.println("pressed key: " + String(key));
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
  
  if (rxId == CAN_VOLUME.id && (rxBuf[dynamicVolumeByteNum] >= 0 && rxBuf[dynamicVolumeByteNum] <= amplifierMaxVolume)){
    
    byte volume = rxBuf[dynamicVolumeByteNum] + amplifierVolumeOffset;

    volume = rxBuf[dynamicVolumeByteNum] == 0 ? 0 : volume;
    volume = volume > amplifierMaxVolume ? amplifierMaxVolume : volume;

    CAN_VOLUME.data[dynamicVolumeByteNum] = volume;
  }
}

void processCan(){

  if(!digitalRead(CAN0_INT) && CAN0.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK){
    lastActivityOn = millis();

    // Serial.println("receive: " + String(rxId, HEX));

    checkIgnition();

    if (!IGNITION.on) return;

    processKey();

    CAN1.sendMsgBuf(rxId, len, rxBuf);
  }

  if (!IGNITION.on) return;

  if (Serial.available()){
    processIncomingByte(Serial.read());
  }
  
  if(!digitalRead(CAN1_INT) && CAN1.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK){

    // Serial.println("receive: " + String(rxId, HEX));

    setDynamicVolume();

    if(!isForbidden()){
      CAN0.sendMsgBuf(rxId, len, rxBuf);
    }
  }

  sendData();
}

void loop(){
  processCan();
  powerDown();
}

void powerDown(){
  if (millis() - lastActivityOn > powerDownDelay && digitalRead(CAN0_INT)) {

    // can was active, reset can controlers before sleep
    if (lastActivityOn > 0) {
      Serial.println("reset CAN");
      setupCanControllers();
    }

    Serial.println("power down");
    Serial.flush();

    CAN1.setMode(MCP_SLEEP);
    CAN0.setMode(MCP_SLEEP);

    // Pull the Rs pin of the MCP2551 transceiver high to enter sleep mode
    digitalWrite(MCP_2551_RS, HIGH);

    cli(); // Disable interrupts
    if(digitalRead(CAN0_INT)) // Make sure we haven't missed an interrupt between the digitalRead() above and now. If an interrupt happens between now and sei()/sleep_cpu() then sleep_cpu() will immediately wake up again
    {
      sleep_enable();
      sleep_bod_disable();
      sei();
      sleep_cpu();
      sleep_disable();
    }
    sei();

    Serial.println("wake up");

    // reset can controlers after sleep
    setupCanControllers();

    lastActivityOn = millis();
  }
}

static void ISR_CAN(){
}