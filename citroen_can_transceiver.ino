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

struct config {
  byte config_version = 2;

  int amplifier_volume_dynamic = 1;
  int amplifier_volume_max = 29;
  int amplifier_volume_offset = 5;
  int amplifier_volume_fixed = 29;

  int equalzer_balance = 0x3f;
  int equalzer_fader = 0x3f;
  int equalzer_bass = 0x42;
  int equalzer_treble = 0x43;
  int equalzer_loudness = 0x40;
  int equalzer_speed_dependent_volume = 0x0;
  int equalzer_preset = 0x40;

  int button_list_process = 1;
  int button_list_long_press = 1;

  int button_volume_up_process = 0;
  int button_volume_up_long_press = 0;

  int button_volume_down_process = 0;
  int button_volume_down_long_press = 0;

  int button_mute_process = 0;
  int button_mute_long_press = 0;

  int button_next_process = 1;
  int button_next_long_press = 1;

  int button_previous_process = 1;
  int button_previous_long_press = 1;

  int button_source_process = 1;
  int button_source_long_press = 0;

  int button_back_process = 0;
  int button_back_long_press = 0;

  int button_home_process = 1;
  int button_home_long_press = 1;

  int button_scroll_press_process = 0;
  int button_scroll_press_long_press = 0;

  int button_phone_process = 0;
  int button_phone_long_press = 0;
  
  int button_voice_assistant_process = 1;
  int button_voice_assistant_long_press = 0;

  int button_scroll_process = 0;
};

config CONFIG;

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
  bool process;
  int shortPressTap;
  bool longPressEnabled;
  int longPressTap;
};

struct SCROLL {
  unsigned long id;
  int byteNum;
  byte position;
  bool process;
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
  int potentiometerTap = 102;
};

SCREEN_STATUS SCREEN_STATUS;
AMPLIFIER AMPLIFIER;
POTENTIOMETER POTENTIOMETER;
PRESSED_BUTTON PRESSED_BUTTON;

SCROLL SCROLL             = {0x0A2, 0,    NULL, NULL, 36, 40};
BUTTON LIST               = {0x21F, 0,    0x01, NULL, 2,  NULL, 43};
BUTTON VOL_UP             = {0x21F, 0,    0x08, NULL, 3,  NULL, 48};
BUTTON VOL_DOWN           = {0x21F, 0,    0x04, NULL, 6,  NULL, 53};
BUTTON MUTE               = {0x21F, 0,    0x0C, NULL, 8,  NULL, 58};
BUTTON NEXT               = {0x21F, 0,    0x40, NULL, 10, NULL, 26};
BUTTON PREVIOUS           = {0x21F, 0,    0x80, NULL, 12, NULL, 29};
BUTTON SOURCE             = {0x0A2, 1,    0x04, NULL, 4,  NULL, 64};
BUTTON BACK               = {0x0A2, 1,    0x10, NULL, 14, NULL, 70};
BUTTON HOME               = {0x0A2, 1,    0x08, NULL, 16, NULL, 23};
BUTTON SCROLL_PRESSED     = {0x0A2, 1,    0x20, NULL, 18, NULL, 77};
BUTTON PHONE              = {0x0A2, 2,    0x80, NULL, 21, NULL, 85};
BUTTON VOICE_ASSIST       = {0x221, 0,    0x01, NULL, 33, NULL, 93};

BUTTON WHEEL_BUTTON[12] = {};

int buttonsCount = 12;

CAN_PACKAGE CAN_VOLUME      = {0x1A5, 1, {0x14}, 500, 0};
CAN_PACKAGE CAN_AMPLIFIER   = {0x165, 4, {0xC0, 0xC0, 0x60, 0x00}, 100, 0};
CAN_PACKAGE CAN_EQUALIZER   = {0x1E5, 7, {0x3F, 0x3D, 0x42, 0x3F, 0x44, 0x40, 0x40}, 500, 0};

CAN_PACKAGE CAN_PACKAGES[3] = {};

int dataPackagesCount = 3;

// power down option
unsigned long lastActivityOn = 0;
unsigned long powerDownDelay = 5000;

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
  setConfig();
  Potentiometer.setWiper(POTENTIOMETER.resetState); 
}

void setConfig(){

  AMPLIFIER.useDynamicVolume = CONFIG.amplifier_volume_dynamic == 1;
  AMPLIFIER.maxVolume = CONFIG.amplifier_volume_max;
  AMPLIFIER.volumeOffset = CONFIG.amplifier_volume_offset;

  CAN_VOLUME.data[0] = CONFIG.amplifier_volume_fixed;

  CAN_EQUALIZER.data[0] = CONFIG.equalzer_balance;
  CAN_EQUALIZER.data[1] = CONFIG.equalzer_fader;
  CAN_EQUALIZER.data[2] = CONFIG.equalzer_bass;
  CAN_EQUALIZER.data[4] = CONFIG.equalzer_treble;
  CAN_EQUALIZER.data[5] = CONFIG.equalzer_loudness + CONFIG.equalzer_speed_dependent_volume;
  CAN_EQUALIZER.data[6] = CONFIG.equalzer_preset;

  CAN_PACKAGES[0] = CAN_VOLUME;
  CAN_PACKAGES[1] = CAN_AMPLIFIER;
  CAN_PACKAGES[2] = CAN_EQUALIZER;

  SCROLL.process                  = CONFIG.button_scroll_process == 1;
  LIST.process                    = CONFIG.button_list_process == 1;
  LIST.longPressEnabled           = CONFIG.button_list_long_press == 1;
  VOL_UP.process                  = CONFIG.button_volume_up_process == 1;
  VOL_UP.longPressEnabled         = CONFIG.button_volume_up_long_press == 1;
  VOL_DOWN.process                = CONFIG.button_volume_up_process == 1;
  VOL_DOWN.longPressEnabled       = CONFIG.button_volume_up_long_press == 1;
  MUTE.process                    = CONFIG.button_mute_process == 1;
  MUTE.longPressEnabled           = CONFIG.button_mute_long_press == 1;
  NEXT.process                    = CONFIG.button_next_process == 1;
  NEXT.longPressEnabled           = CONFIG.button_next_long_press == 1;
  PREVIOUS.process                = CONFIG.button_previous_process == 1;
  PREVIOUS.longPressEnabled       = CONFIG.button_previous_long_press == 1;
  SOURCE.process                  = CONFIG.button_source_process == 1;
  SOURCE.longPressEnabled         = CONFIG.button_source_long_press == 1;
  BACK.process                    = CONFIG.button_back_process == 1;
  BACK.longPressEnabled           = CONFIG.button_back_long_press == 1;
  HOME.process                    = CONFIG.button_home_process == 1;
  HOME.longPressEnabled           = CONFIG.button_home_long_press == 1;
  SCROLL_PRESSED.process          = CONFIG.button_scroll_press_process == 1;
  SCROLL_PRESSED.longPressEnabled = CONFIG.button_scroll_press_long_press == 1;
  PHONE.process                   = CONFIG.button_phone_process == 1;
  PHONE.longPressEnabled          = CONFIG.button_phone_long_press == 1;
  VOICE_ASSIST.process            = CONFIG.button_voice_assistant_process == 1;
  VOICE_ASSIST.longPressEnabled   = CONFIG.button_voice_assistant_long_press == 1;

  WHEEL_BUTTON[0] = VOL_UP;
  WHEEL_BUTTON[1] = VOL_DOWN; 
  WHEEL_BUTTON[2] = MUTE; 
  WHEEL_BUTTON[3] = NEXT;
  WHEEL_BUTTON[4] = PREVIOUS; 
  WHEEL_BUTTON[5] = BACK;
  WHEEL_BUTTON[6] = SCROLL_PRESSED; 
  WHEEL_BUTTON[7] = PHONE; 
  WHEEL_BUTTON[8] = HOME; 
  WHEEL_BUTTON[9] = LIST;
  WHEEL_BUTTON[10] = SOURCE;
  WHEEL_BUTTON[11] = VOICE_ASSIST;
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

  EEPROM.put(address, CONFIG);

  Serial.println("config saved");
}

void loadConfig(){
  int address = 0;

  config loadedConfig;

  Serial.println("config version: " + String(EEPROM.read(address)));

  if(EEPROM.read(address) == CONFIG.config_version){
    EEPROM.get(address, loadedConfig);
    CONFIG = loadedConfig;
    Serial.println("config loaded");
  } else {
    Serial.println("config not loaded");
  }
}

void getConfig(){

  String config = "";

  config += "1:" + String(CONFIG.amplifier_volume_dynamic) + ";";
  config += "2:" + String(CONFIG.amplifier_volume_max) + ";";
  config += "3:" + String(CONFIG.amplifier_volume_offset) + ";";
  config += "4:" + String(CONFIG.amplifier_volume_fixed) + ";";
  config += "5:" + String(CONFIG.equalzer_balance) + ";";
  config += "6:" + String(CONFIG.equalzer_fader) + ";";
  config += "7:" + String(CONFIG.equalzer_bass) + ";";
  config += "8:" + String(CONFIG.equalzer_treble) + ";";
  config += "9:" + String(CONFIG.equalzer_loudness) + ";";
  config += "10:" + String(CONFIG.equalzer_speed_dependent_volume) + ";";
  config += "11:" + String(CONFIG.equalzer_preset) + ";";

  config += "12:" + String(CONFIG.button_list_process) + ";";
  config += "13:" + String(CONFIG.button_list_long_press) + ";";
  config += "14:" + String(CONFIG.button_volume_up_process) + ";";
  config += "15:" + String(CONFIG.button_volume_up_long_press) + ";";
  config += "16:" + String(CONFIG.button_volume_down_process) + ";";
  config += "17:" + String(CONFIG.button_volume_down_long_press) + ";";
  config += "18:" + String(CONFIG.button_mute_process) + ";";
  config += "19:" + String(CONFIG.button_mute_long_press) + ";";
  config += "20:" + String(CONFIG.button_next_process) + ";";
  config += "21:" + String(CONFIG.button_next_long_press) + ";";
  config += "22:" + String(CONFIG.button_previous_process) + ";";
  config += "23:" + String(CONFIG.button_previous_long_press) + ";";

  config += "24:" + String(CONFIG.button_source_process) + ";";
  config += "25:" + String(CONFIG.button_source_long_press) + ";";
  config += "26:" + String(CONFIG.button_back_process) + ";";
  config += "27:" + String(CONFIG.button_back_long_press) + ";";
  config += "28:" + String(CONFIG.button_home_process) + ";";
  config += "29:" + String(CONFIG.button_home_long_press) + ";";
  config += "30:" + String(CONFIG.button_scroll_press_process) + ";";
  config += "31:" + String(CONFIG.button_scroll_press_long_press) + ";";
  config += "32:" + String(CONFIG.button_phone_process) + ";";
  config += "33:" + String(CONFIG.button_phone_long_press) + ";";
  config += "34:" + String(CONFIG.button_voice_assistant_process) + ";";
  config += "35:" + String(CONFIG.button_voice_assistant_long_press) + ";";
  config += "36:" + String(CONFIG.button_scroll_process) + ";";

  Serial.println(config);

  return;

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
    if (rxId == WHEEL_BUTTON[i].id && WHEEL_BUTTON[i].process) {
      if (rxBuf[WHEEL_BUTTON[i].byteNum] == WHEEL_BUTTON[i].byteValue) {
        pressButton(i);
        rxBuf[WHEEL_BUTTON[i].byteNum] = 0x00;
        return;
      } else {
        releaseButton(i);
      }
    }
  }

  if (rxId == SCROLL.id && SCROLL.process){
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
  } else if (line.startsWith("set bass")) {
    String parts[10];
    stringSplit(parts, line, ':');

    Serial.println("Parsed: " + parts[0] + ", " + parts[1]);

    CONFIG.equalzer_bass = parts[1].toInt();

    setConfig();

    Serial.println("bass: " + String(CAN_EQUALIZER.data[2]));

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