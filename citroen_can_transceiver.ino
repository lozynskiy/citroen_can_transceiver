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
  bool remap;
  int shortPressTap;
  bool longPressEnabled;
  int longPressTap;
  int configValue;
};

struct SCROLL {
  unsigned long id;
  byte position;
  int byteNum;
  bool remap;
  int up;
  int down;
  int configValue;
};

struct POTENTIOMETER {
  unsigned long setOn = 0;
  int resetDelay = 150;
  int currentState = 255;
  int resetState = 255;
} POTENTIOMETER;

struct PRESSED_BUTTON {
  int index = -1;
  unsigned long pressedOn;
  int longPressDuration = 500;
} PRESSED_BUTTON;

struct AMPLIFIER {
  bool useDynamicVolume = true;
  int dynamicVolumeByteNum = 0;
  int maxVolume = 29;
  int volumeOffset = 5;
  unsigned long powerOnDelay = 2000;
  unsigned long canVolumeId = 0x1A5;
  unsigned long canAmplifierId = 0x165;
  unsigned long canEqualizerId = 0x1E5;
  byte dynamicVolume[1] = {0x14};
};

struct SCREEN_STATUS {
  unsigned long id = 0x036;
  int byteNum = 3;
  byte byteDisabledValue = 0x36;
  byte currentByte = 0x00;
  bool screenEnabled = true;
  int potentiometerTap = 23;
} SCREEN_STATUS;

const int buttonsCount = 12; //sizeof(WHEEL_BUTTON) / sizeof(BUTTON);
const int dataPackagesCount = 3; //sizeof(CAN_PACKAGES) / sizeof(CAN_PACKAGE);

struct CONFIG {
  byte version = 4;
  BUTTON WHEEL_BUTTON[buttonsCount] = {
    {0x21F, 0, 0x08, false, 3,  false,  33,     1},       // VOL_UP
    {0x21F, 0, 0x04, false, 0,  false,  36,     2},       // VOL_DOWN
    {0x21F, 0, 0x0C, false, 8,  false,  40,     4},       // MUTE
    {0x21F, 0, 0x40, true,  10, true,   26,     8},       // NEXT
    {0x21F, 0, 0x80, true,  12, true,   29,     16},      // PREVIOUS
    {0x0A2, 1, 0x10, false, 14, false,  43,     32},      // BACK
    {0x0A2, 1, 0x08, true,  16, true,   23,     64},      // HOME
    {0x21F, 0, 0x01, true,  2,  false,  48,     128},     // LIST
    {0x0A2, 2, 0x80, false, 21, false,  53,     256},     // PHONE
    {0x0A2, 1, 0x04, true,  4,  false,  NULL,   512},     // SOURCE
    {0x221, 0, 0x01, true,  6,  false,  NULL,   1024},    // VOICE_ASSIST
    {0x0A2, 1, 0x20, false, 18, false,  NULL,   2048}     // SCROLL_PRESSED
  };
  CAN_PACKAGE CAN_PACKAGES[dataPackagesCount] = {
    {0x1A5, 1, {0x14}, 500, 0},                                           // CAN_VOLUME
    {0x165, 4, {0xC0, 0xC0, 0x60, 0x00}, 100, 0},                         // CAN_AMPLIFIER
    // balance: 0; fader: -2; bass: +3; ..: 0; treble: +5; loudness + speed: true; false; preset: linear
    {0x1E5, 7, {0x3F, 0x3D, 0x42, 0x3F, 0x44, 0x40, 0x40}, 500, 0}       // CAN_EQUALIZER
  };
  SCROLL SCROLL = {0x0A2, NULL, 0, false, 58, 64, 4096};
  AMPLIFIER AMPLIFIER;
} CONFIG;

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

int getStringSum(String string) {
  int sum = 0;
  for (int i = 0; i < string.length(); i++) {
    sum += (int) string[i];
  }

  return sum;
}

void parseConfig(String config){
  String parts[10];
  stringSplit(parts, config, ';');

  // Serial.println("parsed sum: " + String(getStringSum(parts[1])) + ", received sum: " + String(parts[2]));

  // validate
  if (getStringSum(parts[1]) != parts[2].toInt() || getStringSum(parts[1]) == 0) {
    Serial.println("broken config");
    return;
  }

  String dataParts[9];
  stringSplit(dataParts, parts[1], ' ');

  int packageIndex = getPackageIndex(parseAsHex(parts[0]));

  if (packageIndex != -1) {
    byte data[8];

    for (int i = 0; i < CONFIG.CAN_PACKAGES[packageIndex].dlc; i++){
      data[i] = parseAsHex(dataParts[i]);
    }

    memcpy(CONFIG.CAN_PACKAGES[packageIndex].data, data, 8);

    return;
  }

  if (parts[0] == "0x000") {
    packageIndex = getPackageIndex(CONFIG.AMPLIFIER.canVolumeId);

    CONFIG.CAN_PACKAGES[packageIndex].data[0] = dataParts[1].toInt();

    CONFIG.AMPLIFIER.useDynamicVolume = dataParts[0] == "1";
    CONFIG.AMPLIFIER.maxVolume = dataParts[1].toInt();
    CONFIG.AMPLIFIER.volumeOffset = dataParts[2].toInt();

    return;
  }

  if (parts[0] == "0x001") {
    int remapData = dataParts[0].toInt();
    int longPressData = dataParts[1].toInt();

    for (int i = 0; i < buttonsCount; i++) {
      CONFIG.WHEEL_BUTTON[i].remap = (CONFIG.WHEEL_BUTTON[i].configValue & remapData) > 0;
      CONFIG.WHEEL_BUTTON[i].longPressEnabled = (CONFIG.WHEEL_BUTTON[i].configValue & longPressData) > 0;
    }
    
    CONFIG.SCROLL.remap = (CONFIG.SCROLL.configValue & remapData) > 0;

    return;
  }
}

int getPackageIndex(unsigned long id){
  int index = -1;

  for (int i = 0; i < dataPackagesCount; i++){
    if(id == CONFIG.CAN_PACKAGES[i].id){
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

  if (byte(EEPROM.read(0)) != CONFIG.version) {
    Serial.println("config is not loaded");
    return;
  }

  EEPROM.get(address, CONFIG);

  Serial.println("config loaded");
}

void getConfig(){
  int packageIndex = getPackageIndex(CONFIG.AMPLIFIER.canEqualizerId);
  String data = "";
  int remapButtonData = 0;
  int longPressButtonData = 0;

  //for (int i = 0; i < dataPackagesCount; i++){
    
    for (int d = 0; d < CONFIG.CAN_PACKAGES[packageIndex].dlc; d++){
      data += " 0x";

      if (CONFIG.CAN_PACKAGES[packageIndex].data[d] < 16) 
        data += "0";
      
      data += String(CONFIG.CAN_PACKAGES[packageIndex].data[d], HEX);
    }

    data.trim();

    addDelayedMessage("0x" + String(CONFIG.CAN_PACKAGES[packageIndex].id, HEX) + ";" + data + ";" + String(getStringSum(data)));

    data = String(CONFIG.AMPLIFIER.useDynamicVolume) + " " + String(CONFIG.AMPLIFIER.maxVolume) + 
      " " + String(CONFIG.AMPLIFIER.volumeOffset);

    addDelayedMessage("0x000;" + data + ";" + String(getStringSum(data)));

    for (int i = 0; i < buttonsCount; i++){
      remapButtonData += CONFIG.WHEEL_BUTTON[i].remap ? CONFIG.WHEEL_BUTTON[i].configValue : 0;
      longPressButtonData += CONFIG.WHEEL_BUTTON[i].longPressEnabled ? CONFIG.WHEEL_BUTTON[i].configValue : 0;
    }

    remapButtonData += CONFIG.SCROLL.remap ? CONFIG.SCROLL.configValue : 0;

    data = String(remapButtonData) + " " + String(longPressButtonData);

    addDelayedMessage("0x001;" + data + ";" + String(getStringSum(data)));
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
    if (rxId == CONFIG.CAN_PACKAGES[i].id) return true;
  }

  return false;
}

void sendData(){
  for (int i = 0; i < dataPackagesCount; i++){
    if(millis() - CONFIG.CAN_PACKAGES[i].lastMillis >= CONFIG.CAN_PACKAGES[i].period){

      if (CONFIG.CAN_PACKAGES[i].id == CONFIG.AMPLIFIER.canAmplifierId && millis() - IGNITION.switchedOnTime < CONFIG.AMPLIFIER.powerOnDelay){
        continue;
      }

      if (CONFIG.AMPLIFIER.useDynamicVolume && CONFIG.CAN_PACKAGES[i].id == CONFIG.AMPLIFIER.canVolumeId){
        CAN0.sendMsgBuf(CONFIG.CAN_PACKAGES[i].id, CONFIG.CAN_PACKAGES[i].dlc, CONFIG.AMPLIFIER.dynamicVolume);
      } else {
        CAN0.sendMsgBuf(CONFIG.CAN_PACKAGES[i].id, CONFIG.CAN_PACKAGES[i].dlc, CONFIG.CAN_PACKAGES[i].data);
      }

      CONFIG.CAN_PACKAGES[i].lastMillis = millis();
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
    if (rxId == CONFIG.WHEEL_BUTTON[i].id && CONFIG.WHEEL_BUTTON[i].remap == true) {
      if (rxBuf[CONFIG.WHEEL_BUTTON[i].byteNum] == CONFIG.WHEEL_BUTTON[i].byteValue) {
        pressButton(i);
        rxBuf[CONFIG.WHEEL_BUTTON[i].byteNum] = 0x00;
        return;
      } else {
        releaseButton(i);
      }
    }
  }

  if (rxId == CONFIG.SCROLL.id && CONFIG.SCROLL.remap == true){
    if (!CONFIG.SCROLL.position){
      CONFIG.SCROLL.position = rxBuf[CONFIG.SCROLL.byteNum];
    }

    if (CONFIG.SCROLL.position != rxBuf[CONFIG.SCROLL.byteNum]){
      if (rxBuf[CONFIG.SCROLL.byteNum] > CONFIG.SCROLL.position){
        setPotentiometer(CONFIG.SCROLL.up);
      } else {
        setPotentiometer(CONFIG.SCROLL.down);
      }
      CONFIG.SCROLL.position = rxBuf[CONFIG.SCROLL.byteNum];
      rxBuf[CONFIG.SCROLL.byteNum] = 0x00;

      return;
    }
  }

  // if no press detected for more than period - reset potentiometer
  if (millis() - POTENTIOMETER.setOn > POTENTIOMETER.resetDelay){
    setPotentiometer(POTENTIOMETER.resetState);
  }
}

bool isButtonLongPress(int index) {
  if (CONFIG.WHEEL_BUTTON[index].longPressEnabled == true
          && (millis() - PRESSED_BUTTON.pressedOn) > PRESSED_BUTTON.longPressDuration){
    return true;
  }
  return false;
}

int getButtonTap(int index) {
  int tap  = POTENTIOMETER.resetState;
  if (isButtonLongPress(index)){
    tap = CONFIG.WHEEL_BUTTON[index].longPressTap;
  } else {
    tap = CONFIG.WHEEL_BUTTON[index].shortPressTap;
  }

  return tap;
}

void pressButton (int index){
  if (PRESSED_BUTTON.index != index){
    PRESSED_BUTTON.index = index;
    PRESSED_BUTTON.pressedOn = millis();
  } else if (isButtonLongPress(index) == true || CONFIG.WHEEL_BUTTON[index].longPressEnabled == false) {
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
  if (!CONFIG.AMPLIFIER.useDynamicVolume){
    return;
  }
  
  if (rxId == CONFIG.AMPLIFIER.canVolumeId && (rxBuf[CONFIG.AMPLIFIER.dynamicVolumeByteNum] >= 0 && rxBuf[CONFIG.AMPLIFIER.dynamicVolumeByteNum] <= CONFIG.AMPLIFIER.maxVolume)) {
    int canVolumeIndex = getPackageIndex(CONFIG.AMPLIFIER.canVolumeId);

    byte volume = rxBuf[CONFIG.AMPLIFIER.dynamicVolumeByteNum] + CONFIG.AMPLIFIER.volumeOffset;

    volume = rxBuf[CONFIG.AMPLIFIER.dynamicVolumeByteNum] == 0 ? 0 : volume;
    volume = volume > CONFIG.AMPLIFIER.maxVolume ? CONFIG.AMPLIFIER.maxVolume : volume;

    CONFIG.AMPLIFIER.dynamicVolume[0] = volume;
  }
}

void processCan(){

  if(!digitalRead(CAN0_INT) && CAN0.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK){
    lastActivityOn = millis();

    // String data = "";
    // for (int i = 0; i < len; i++) {
    //   data += String(rxBuf[i], HEX) + " ";
    // }

    // Serial.println("receive: " + String(rxId, HEX) + ": " + data);

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

    // String data = "";
    // for (int i = 0; i < len; i++) {
    //   data += String(rxBuf[i], HEX) + " ";
    // }

    // Serial.println("receive: " + String(rxId, HEX) + ": " + data);

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