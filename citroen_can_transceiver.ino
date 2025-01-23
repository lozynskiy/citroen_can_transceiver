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
    5: loudness + speed
      false + false = 00
      true + false = 40
      true + true = 47
      false + true = 07
    6: preset: 
      40 - Linear; 
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

const byte CONFIGURATION_VER = 1;

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
  bool on = true;
  int offDelay = 2500;
  unsigned long switchedOffTime = 0;
  unsigned long switchedOnTime = 0;
} IGNITION;

struct BUTTON {
  unsigned long id;
  int byteNum;
  byte byteValue;
  int shortPressTap;
  bool longPressEnabled;
  int longPressTap;
};

struct SCROLL {
  unsigned long id;
  byte position;
  int byteNum;
  int up;
  int down;
};

struct POTENTIOMETER {
  unsigned long setOn = 0;
  int resetDelay = 150;
  int currentState = 255;
  int resetState = 255;
};

struct PRESSED_BUTTON {
  int index = -1;
  unsigned long pressedOn;
  int longPressDuration = 500;
};

struct AMPLIFIER {
  bool useDynamicVolume = true;
  int dynamicVolumeByteNum = 0;
  int maxVolume = 29;
  int volumeOffset = 5;
  unsigned long powerOnDelay = 2000;
};

struct SCREEN_STATUS {
  unsigned long id = 0x036;
  int byteNum = 3;
  byte byteDisabledValue = 0x36;
  byte currentByte = 0x00;
  bool screenEnabled = true;
  int potentiometerTap = 23;
};

SCREEN_STATUS SCREEN_STATUS;
AMPLIFIER AMPLIFIER;
POTENTIOMETER POTENTIOMETER;
PRESSED_BUTTON PRESSED_BUTTON;
SCROLL SCROLL;//             = {0x0A2, NULL, 0, 26, 29};

BUTTON LIST               = {0x21F, 0, 0x01, 2, false, NULL};
BUTTON VOL_UP             = {0x21F, 0, 0x08, 3, false, NULL};
BUTTON VOL_DOWN           = {0x21F, 0, 0x04, 6, false, NULL};
BUTTON MUTE               = {0x21F, 0, 0x0C, 8, false, NULL};
BUTTON NEXT               = {0x21F, 0, 0x40, 10, true, 26};
BUTTON PREVIOUS           = {0x21F, 0, 0x80, 12, true, 29};

BUTTON SOURCE             = {0x0A2, 1, 0x04, 4, false, NULL};
BUTTON BACK               = {0x0A2, 1, 0x10, 14, false, NULL};
BUTTON HOME               = {0x0A2, 1, 0x08, 16, true, 23};
BUTTON SCROLL_PRESSED     = {0x0A2, 1, 0x20, 18, false, NULL};
BUTTON PHONE              = {0x0A2, 2, 0x80, 21, false, NULL};

BUTTON VOICE_ASSIST       = {0x221, 0, 0x01, 6, false, NULL};
// BUTTON NIGHT_MODE         = {0x036, 3, 0x36, 23, false, NULL};

BUTTON WHEEL_BUTTON[] = {
  // VOL_UP, 
  // VOL_DOWN, 
  // MUTE, 
  NEXT, 
  PREVIOUS, 
  // BACK, 
  // SCROLL_PRESSED, 
  // PHONE, 
  // NIGHT_MODE,
  HOME, 
  LIST,
  SOURCE, 
  VOICE_ASSIST
};

const int buttonsCount = sizeof(WHEEL_BUTTON) / sizeof(BUTTON);

CAN_PACKAGE CAN_VOLUME = {0x1A5, 1, {0x14}, 500, 0};
CAN_PACKAGE CAN_AMPLIFIER = {0x165, 4, {0xC0, 0xC0, 0x60, 0x00}, 100, 0};
// balance: 0; fader: -2; bass: +3; ..: 0; treble: +5; loudness + speed: true; false; preset: linear
CAN_PACKAGE CAN_EQUALIZER = {0x1E5, 7, {0x3F, 0x3D, 0x42, 0x3F, 0x44, 0x40, 0x40}, 500, 0};

CAN_PACKAGE CAN_PACKAGES[] = {
  CAN_VOLUME,       // set volume
  CAN_AMPLIFIER,    // enable amplifier
  CAN_EQUALIZER     // set equalizer config
};

const int dataPackagesCount = sizeof(CAN_PACKAGES) / sizeof(CAN_PACKAGE);

// struct CONFIG {
//   byte version = 1;
//   BUTTON WHEEL_BUTTON[buttonsCount];
//   CAN_PACKAGE CAN_PACKAGES[dataPackagesCount];
// } CONFIG;

// power down option
unsigned long lastActivityOn = 0;
unsigned long powerDownDelay = 5000;

unsigned long messageSentOn = 0;
const short messages = 4;
String delayedMessages[messages] = {"", "", "", ""};
int availableMessages = 0;
const int messageDelay = 50;

unsigned long rxId;
byte len;
byte rxBuf[8];

MCP_CAN CAN0(9);                // CAN0 interface usins CS on digital pin 9
MCP_CAN CAN1(10);               // CAN1 interface using CS on digital pin 10
MCP41_Simple Potentiometer;

#define CAN0_INT 3              // define interrupt pin for CAN0 receive buffer
#define CAN1_INT 2              // define interrupt pin for CAN1 receive buffer
#define POT_CS 8                // digital potentiometer CS pin
#define MCP_2551_RS 4           // MCP2551 RS pin
#define REMOTE 5                // Remote trigger pin

void setup(){
  Serial.begin(38400);
  Potentiometer.begin(POT_CS);

  pinMode(CAN0_INT, INPUT);
  pinMode(CAN1_INT, INPUT);
  pinMode(MCP_2551_RS, OUTPUT);
  pinMode(REMOTE, OUTPUT);
  
  setupCanControllers();

  attachInterrupt(digitalPinToInterrupt(CAN0_INT), ISR_CAN, FALLING);

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);

  loadConfig();
  Potentiometer.setWiper(POTENTIOMETER.resetState); 
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

  // Set remote to high to enable power
  digitalWrite(REMOTE, HIGH);
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

  // validate
  if (parts[1].length() != parts[2].toInt()) {
    Serial.println("wrong config");
    return;
  }

  String dataParts[9];
  stringSplit(dataParts, parts[1], ' ');

  int packageIndex = getPackageIndex(parseAsHex(parts[0]));

  if (packageIndex != -1) {
    byte data[8];

    for (int i = 0; i < CAN_PACKAGES[packageIndex].dlc; i++){
      data[i] = parseAsHex(dataParts[i]);
    }

    memcpy(CAN_PACKAGES[packageIndex].data, data, 8);

    return;
  }

  if (parts[0] == "0x000") {
    packageIndex = getPackageIndex(CAN_VOLUME.id);

    CAN_PACKAGES[packageIndex].data[0] = dataParts[1].toInt();

    AMPLIFIER.useDynamicVolume = dataParts[0] == "1";
    AMPLIFIER.maxVolume = dataParts[1].toInt();
    AMPLIFIER.volumeOffset = dataParts[2].toInt();

    return;
  }
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

  EEPROM.put(address, CONFIGURATION_VER);

  address += 1;

  for (int i = 0; i < dataPackagesCount; i++){
    EEPROM.put(address, CAN_PACKAGES[i]);
    address += sizeof(CAN_PACKAGE);
  }

  EEPROM.put(address, AMPLIFIER);
  address += sizeof(AMPLIFIER);

  Serial.println("config saved");
}

void loadConfig(){
  int address = 0;
  int index;

  CAN_PACKAGE canLoadedData;

  if (byte(EEPROM.read(0)) != CONFIGURATION_VER) {
    Serial.println("config is not loaded");
    return;
  }

  address += 1;

  for (int i = 0; i < dataPackagesCount; i++){
    EEPROM.get(address, canLoadedData);

    index = getPackageIndex(canLoadedData.id);

    if (index >= 0){
      memcpy(CAN_PACKAGES[i].data, canLoadedData.data, 8);
    }

    address += sizeof(CAN_PACKAGE);
  }

  EEPROM.get(address, AMPLIFIER);

  address += sizeof(AMPLIFIER);

  Serial.println("config loaded");
}

void getConfig(){
  int packageIndex = getPackageIndex(CAN_EQUALIZER.id);
  String data = "";

  //for (int i = 0; i < dataPackagesCount; i++){
    
    for (int d = 0; d < CAN_PACKAGES[packageIndex].dlc; d++){
      data += " 0x";

      if (CAN_PACKAGES[packageIndex].data[d] < 16) 
        data += "0";
      
      data += String(CAN_PACKAGES[packageIndex].data[d], HEX);
    }

    data.trim();

    addDelayedMessage("0x" + String(CAN_PACKAGES[packageIndex].id, HEX) + ";" + data + ";" + String(data.length()));

    data = String(AMPLIFIER.useDynamicVolume) + " " + String(AMPLIFIER.maxVolume) + 
      " " + String(AMPLIFIER.volumeOffset);

    addDelayedMessage("0x000;" + data + ";" + String(data.length()));

    //Serial.println("0x" + String(CAN_PACKAGES[packageIndex].id, HEX) + ";" + data + "|" +
    //  "0x000;" + String(AMPLIFIER.useDynamicVolume) + " " + String(AMPLIFIER.maxVolume) + 
    //  " " + String(AMPLIFIER.volumeOffset));
  //}
}

void addDelayedMessage(String message) {
  for (int i = 0; i < messages; i ++) {
    if (delayedMessages[i] != "") {
      continue;
    }

    delayedMessages[i] = message;
    availableMessages ++;
    
    return;
  }
}

void sendDelayedMessage() {

  if (availableMessages = 0 || millis() - messageSentOn < messageDelay) {
    return;
  }

  for (int i = 0; i < messages; i ++) {
    if (delayedMessages[i] == "") {
       continue;
     }

    Serial.println(delayedMessages[i]);
    delayedMessages[i] = "";
    availableMessages --;
    messageSentOn = millis();
      
    return;
  }
}

bool isForbiddenPackage(){
  for (int i = 0; i < dataPackagesCount; i++){
    if (rxId == CAN_PACKAGES[i].id) return true;
  }

  return false;
}

void sendData(){
  for (int i = 0; i < dataPackagesCount; i++){
    if(millis() - CAN_PACKAGES[i].lastMillis >= CAN_PACKAGES[i].period){

      if (CAN_PACKAGES[i].id == CAN_AMPLIFIER.id && millis() - IGNITION.switchedOnTime < AMPLIFIER.powerOnDelay){
        continue;
      }

      if (AMPLIFIER.useDynamicVolume && CAN_PACKAGES[i].id == CAN_VOLUME.id){
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

void setScreenStatus(){
  if (rxId != SCREEN_STATUS.id){
    return;
  }
    
  if (rxBuf[SCREEN_STATUS.byteNum] == SCREEN_STATUS.currentByte) {
    return;
  }

  if (rxBuf[SCREEN_STATUS.byteNum] == SCREEN_STATUS.byteDisabledValue || SCREEN_STATUS.currentByte == SCREEN_STATUS.byteDisabledValue){
    setPotentiometer(SCREEN_STATUS.potentiometerTap);
  }

  SCREEN_STATUS.currentByte = rxBuf[SCREEN_STATUS.byteNum];
}

void processWheelButton(){
  for (int i = 0; i < buttonsCount; i++){
    if (rxId == WHEEL_BUTTON[i].id) {
      if (rxBuf[WHEEL_BUTTON[i].byteNum] == WHEEL_BUTTON[i].byteValue) {
        pressButton(i);
        rxBuf[WHEEL_BUTTON[i].byteNum] = 0x00;
        return;
      } else {
        releaseButton(i);
      }
    }
  }

  if (rxId == SCROLL.id){
    if (!SCROLL.position){
      SCROLL.position = rxBuf[SCROLL.byteNum];
    }

    if (SCROLL.position != rxBuf[SCROLL.byteNum]){
      if (rxBuf[SCROLL.byteNum] > SCROLL.position){
        setPotentiometer(SCROLL.up);
      } else {
        setPotentiometer(SCROLL.down);
      }
      SCROLL.position = rxBuf[SCROLL.byteNum];
      rxBuf[SCROLL.byteNum] = 0x00;

      return;
    }
  }

  // if no press detected for more than period - reset potentiometer
  if (millis() - POTENTIOMETER.setOn > POTENTIOMETER.resetDelay){
    setPotentiometer(POTENTIOMETER.resetState);
  }
}

bool isButtonLongPress(int index) {
  if (WHEEL_BUTTON[index].longPressEnabled == true && (millis() - PRESSED_BUTTON.pressedOn) > PRESSED_BUTTON.longPressDuration){
    return true;
  }
  return false;
}

int getButtonTap(int index) {
  int tap  = POTENTIOMETER.resetState;
  if (isButtonLongPress(index)){
    tap = WHEEL_BUTTON[index].longPressTap;
  } else {
    tap = WHEEL_BUTTON[index].shortPressTap;
  }

  return tap;
}

void pressButton (int index){
  if (PRESSED_BUTTON.index != index){
    PRESSED_BUTTON.index = index;
    PRESSED_BUTTON.pressedOn = millis();
  } else if (isButtonLongPress(index) == true || WHEEL_BUTTON[index].longPressEnabled == false) {
    // hold button already reach long press state, set potentiometer value
    setPotentiometer(getButtonTap(index));
    //Serial.println("pressed: " + String(getButtonTap(index)) + ", duration: " + String(millis() - PRESSED_BUTTON.pressedOn));
  }
}

void releaseButton(int index) {
  int tap  = 0;
  if (PRESSED_BUTTON.index == index){
    tap = getButtonTap(index);
    // set potentiometer value in case there were no long press detected
    if (POTENTIOMETER.currentState == POTENTIOMETER.resetState){
      setPotentiometer(tap);
    }
    // Serial.println("released: " + String(tap) + ", duration: " + String(millis() - PRESSED_BUTTON.pressedOn));
    PRESSED_BUTTON.index = -1;
  }
}

void setPotentiometer(int tap){
  POTENTIOMETER.setOn = millis();

  // Serial.println("potentiometer current state: " + String(POTENTIOMETER.currentState) + ", received: " + String(tap));
  if (POTENTIOMETER.currentState != tap){
    POTENTIOMETER.currentState = tap;

    Potentiometer.setWiper(tap);
    //Serial.println("potentiometer tap: " + String(tap));
  }
}

void processIncomingData (const char * data){
  String line = String(data);

  if (line == "get config"){
    getConfig();
  } else if (line == "save config") {
    saveConfig();
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
  if (!AMPLIFIER.useDynamicVolume){
    return;
  }
  
  if (rxId == CAN_VOLUME.id && (rxBuf[AMPLIFIER.dynamicVolumeByteNum] >= 0 && rxBuf[AMPLIFIER.dynamicVolumeByteNum] <= AMPLIFIER.maxVolume)){
    
    byte volume = rxBuf[AMPLIFIER.dynamicVolumeByteNum] + AMPLIFIER.volumeOffset;

    volume = rxBuf[AMPLIFIER.dynamicVolumeByteNum] == 0 ? 0 : volume;
    volume = volume > AMPLIFIER.maxVolume ? AMPLIFIER.maxVolume : volume;

    CAN_VOLUME.data[AMPLIFIER.dynamicVolumeByteNum] = volume;
  }
}

void processCan(){

  if(!digitalRead(CAN0_INT) && CAN0.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK){
    lastActivityOn = millis();

    // if (rxId == 0x036) {
    //   Serial.println("receive: " + String(rxId, HEX) + ": " + String(rxBuf[0], HEX) + " " + String(rxBuf[1], HEX) + " " + String(rxBuf[2], HEX) + " " + String(rxBuf[3], HEX) + " " + String(rxBuf[4], HEX) + " " + String(rxBuf[5], HEX) + " " + String(rxBuf[6], HEX) + " " + String(rxBuf[7], HEX));
    // }

    checkIgnition();

    if (!IGNITION.on) return;

    processWheelButton();
    // setScreenStatus();

    CAN1.sendMsgBuf(rxId, len, rxBuf);
  }

  if (!IGNITION.on) return;

  if (Serial.available()){
    processIncomingByte(Serial.read());
  }
  
  sendDelayedMessage();

  if(!digitalRead(CAN1_INT) && CAN1.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK){

    // if (!(rxId == 0x165 || rxId == 0x3e5 || rxId == 0x1a5 || rxId == 0x1e5 || rxId == 0x167 || rxId == 0x1a9 || rxId == 0x1a3 || rxId == 0x164)) {
    // // 1a9, 1a3, 164, 
    // // if (rxId == 0x164) {
    //   Serial.println("receive: " + String(rxId, HEX) + ": " + String(rxBuf[0], HEX) + " " + String(rxBuf[1], HEX) + " " + String(rxBuf[2], HEX) + " " + String(rxBuf[3], HEX) + " " + String(rxBuf[4], HEX) + " " + String(rxBuf[5], HEX) + " " + String(rxBuf[6], HEX) + " " + String(rxBuf[7], HEX));
    // }

    setDynamicVolume();

    if(!isForbiddenPackage()){
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
  if (millis() - lastActivityOn > powerDownDelay && digitalRead(CAN0_INT) && IGNITION.on == false) {

    // can was active, reset can controllers before sleep
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

    // Set remote to low to disable power
    digitalWrite(REMOTE, LOW);

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

    // reset can controllers after sleep
    setupCanControllers();

    lastActivityOn = millis();
  }
}

static void ISR_CAN(){
}