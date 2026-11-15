/*
  เครื่องจ่ายยา JaiYA — โค้ดเดิมของเครื่อง + รับค่าจาก ESP32-S3 CAM ทาง Serial2
  ส่วนที่เพิ่มมีเครื่องหมาย [เพิ่ม] ส่วนอื่นเหมือนโค้ดเดิมทุกบรรทัด

  การต่อสาย (เพิ่ม)
    ESP32-S3 CAM ขา 21 (TX)  ->  บอร์ดนี้ D32 (RX2)
    ESP32-S3 CAM ขา 14 (RX)  <-  บอร์ดนี้ D33 (TX2)   (ไม่ต่อก็ได้ ถ้าไม่ต้องการให้เว็บเห็นสถานะ)
    GND                      ->  GND
    ห้ามต่ออะไรที่ TXD / RXD

  สิ่งที่รับจากกล้อง
    {"cmd":"time","y":2026,"mo":11,"d":15,"h":8,"mi":0,"s":0,"src":"ntp"}  ตั้งเวลา RTC ตาม server (ทุก 10 นาที)
    {"cmd":"time",...}  จากเว็บตอนเชื่อมต่อ
    {"cmd":"test","floor":1-7,"slot":1-6}      = TEST <floor-1> <slot-1>  (ไปช่อง + ดันยา + เพลง)
    {"cmd":"dispense","floor":1-7,"slot":1-6}  เหมือน test
    {"cmd":"goto","floor":1-7,"slot":1-6}      ไปช่องอย่างเดียว ไม่ดันยา
    TEST <day 0-6> <slot 0-5>                  แบบเดียวกับพิมพ์ใน Serial Monitor
*/

#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h> 
#include <EEPROM.h>
#include <ESP32Servo.h> 
#include <ArduinoJson.h>   // [เพิ่ม] อ่านข้อความจากกล้อง (ArduinoJson v7)

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

RTC_DS3231 rtc; 
Servo pushServo; 

// --- ขา Pin สำหรับ ESP32 ---
#define VRX_PIN 35 
#define VRY_PIN 34 
#define SW_PIN  25
#define BUZ_PIN 26
#define SERVO_PIN 15 

// --- ขา Pin สำหรับ Motor (PWM) และ Sensor ---
#define MOT_X_IN1 12
#define MOT_X_IN2 13
#define MOT_Y_IN1 27
#define MOT_Y_IN2 14

#define SENSOR_X_PIN 16 
#define SENSOR_Y_PIN 17 

// [เพิ่ม] Serial2 ต่อกับ ESP32-S3 CAM (ใช้ขาที่โค้ดเดิมไม่ได้ใช้)
#define CAM_RX_PIN 32   // รับจากกล้องขา 21
#define CAM_TX_PIN 33   // ส่งไปกล้องขา 14
HardwareSerial CamSerial(2);
String camBuf;
bool moveOnly = false;           // คำสั่ง goto: ไปช่องอย่างเดียว ไม่ดันยา
unsigned long lastStatusMs = 0;

// --- ตั้งค่าความเร็วในการเคลื่อนที่ ---
#define HOMING_SPEED 70       
#define MOVE_SPEED 70        

// ==========================================
// ตั้งค่าเวลาในการเคลื่อนที่ไปแต่ละช่อง (มิลลิวินาที)
// นับจากตำแหน่ง Zero (เซนเซอร์ทำงาน) ไปยังช่องเป้าหมาย
// ==========================================
// แกน X (7 วัน) 
// ถ้าช่องแรก (SUN) อยู่ตำแหน่งเดียวกับจุด Zero พอดี ให้ใส่ 0
const unsigned long delay_X[7] = {
  0,     // SUN (X1) - สมมติว่าช่องแรกไม่ต้องขยับ
  600,  // MON (X2)
  1200,  // TUE (X3) - สมมติว่าระยะห่างไม่เท่ากันเป๊ะๆ ปรับแยกได้เลย
  1850,  // WED (X4)
  2400,  // THU (X5)
  3000,  // FRI (X6)
  3400   // SAT (X7)
}; 

// แกน Y (6 เวลา)
// ถ้าช่องแรกของแกน Y อยู่ตำแหน่งเดียวกับจุด Zero พอดี ให้ใส่ 0
const unsigned long delay_Y[6] = {
  0,     // Slot 1 (Y1)
  1000,  // Slot 2 (Y2)
  1900,  // Slot 3 (Y3)
  2600,  // Slot 4 (Y4)
  3500,  // Slot 5 (Y5)
  4600   // Slot 6 (Y6)
};

// --- ตั้งค่า EEPROM ---
#define EEPROM_SIZE 256
#define EEPROM_ADDR 1 
#define MAGIC_BYTE 0xAA 

// --- System States ---
enum State { HOME, MENU, SET_TIME, SET_ALARM, ALARM_RINGING, MOVING_TO_BOX };
State currentState = HOME;

// --- Joystick Variables ---
int menuIndex = 0;
bool buttonPressed = false;
bool joyMoved = false;

// --- Alarm Data Structure ---
struct AlarmTimer {
  uint8_t hour = 0;
  uint8_t minute = 0;
  bool active = false;
};
AlarmTimer alarms[7][6];

int setDay = 0, setSlot = 0, editField = 0; 
int rtcEditField = 0, setYear, setMonth, setDayRTC, setHour, setMinute;
int activeAlarmDay = -1;
int activeAlarmSlot = -1;

String daysOfWeek[7] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

// ==========================================
// เพลง Super Mario Bros. Theme
// ==========================================
const int alarm_notes[] = {
  659, 659, 0, 659, 0, 523, 659, 0, 784, 0, 0, 0, 392, 0, 0, 0,
  523, 0, 0, 392, 0, 0, 330, 0, 0, 440, 0, 494, 0, 466, 440, 0,
  392, 659, 784, 880, 0, 698, 784, 0, 659, 0, 523, 587, 494, 0, 0,
  523, 0, 0, 392, 0, 0, 330, 0, 0, 440, 0, 494, 0, 466, 440, 0,
  392, 659, 784, 880, 0, 698, 784, 0, 659, 0, 523, 587, 494, 0, 0,
  784, 740, 698, 622, 0, 659, 0, 415, 440, 523, 0, 440, 523, 587, 0,
  784, 740, 698, 622, 0, 659, 0, 1047, 0, 1047, 1047, 0, 0, 0
};
const int alarm_durations[] = {
  150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150,
  150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150,
  200, 200, 200, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150,
  150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150,
  200, 200, 200, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150,
  150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150,
  150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150
};
const int numNotes = sizeof(alarm_notes) / sizeof(alarm_notes[0]);

// ==========================================
// ฟังก์ชันควบคุม Servo อย่างสมูท
// ==========================================
void operateServoSmoothly() {
  int startPos = 0;   
  int targetPos = 90; 
  int speedDelay = 15; 
  
  for (int pos = startPos; pos <= targetPos; pos += 1) { 
    pushServo.write(pos);
    delay(speedDelay); 
  }
  
  delay(500); 
  
  for (int pos = targetPos; pos >= startPos; pos -= 1) { 
    pushServo.write(pos);
    delay(speedDelay); 
  }
}

// ==========================================
// ฟังก์ชันตรวจสอบคำสั่งจาก Serial
// ==========================================
void checkSerialCommands() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd.startsWith("TEST")) {
      int space1 = cmd.indexOf(' ');
      int space2 = cmd.lastIndexOf(' ');
      
      if (space1 != -1 && space2 != -1 && space1 != space2) {
        int dayTest = cmd.substring(space1 + 1, space2).toInt();
        int slotTest = cmd.substring(space2 + 1).toInt();
        
        if (dayTest >= 0 && dayTest <= 6 && slotTest >= 0 && slotTest <= 5) {
          Serial.print("Testing Target -> Day: "); Serial.print(daysOfWeek[dayTest]);
          Serial.print(" Slot: "); Serial.println(slotTest + 1);
          
          activeAlarmDay = dayTest;
          activeAlarmSlot = slotTest;
          currentState = MOVING_TO_BOX; 
        } else {
          Serial.println("Error: Day must be 0-6, Slot must be 0-5.");
        }
      } else {
        Serial.println("Format Error! Use: TEST <day> <slot> (e.g. TEST 0 2)");
      }
    }
  }
}

// ==========================================
// ฟังก์ชันควบคุมมอเตอร์ DC และ Homing
// ==========================================
void driveMotorX(int speed) { 
  if (speed > 0) {
    analogWrite(MOT_X_IN1, speed); analogWrite(MOT_X_IN2, 0);
  } else if (speed < 0) {
    analogWrite(MOT_X_IN1, 0); analogWrite(MOT_X_IN2, -speed);
  } else {
    analogWrite(MOT_X_IN1, 0); analogWrite(MOT_X_IN2, 0);
  }
}

void driveMotorY(int speed) { 
  if (speed > 0) {
    analogWrite(MOT_Y_IN1, speed); analogWrite(MOT_Y_IN2, 0);
  } else if (speed < 0) {
    analogWrite(MOT_Y_IN1, 0); analogWrite(MOT_Y_IN2, -speed);
  } else {
    analogWrite(MOT_Y_IN1, 0); analogWrite(MOT_Y_IN2, 0);
  }
}

void homingXY() {
  display.clearDisplay();
  display.setTextSize(1);
  
  display.setCursor(10, 20);
  display.println("Homing X...");
  display.display();
  
  while(digitalRead(SENSOR_X_PIN) == LOW) { driveMotorX(-HOMING_SPEED); }
  driveMotorX(0); 
  
  display.clearDisplay();
  display.setCursor(10, 20);
  display.println("Homing Y...");
  display.display();
  
  while(digitalRead(SENSOR_Y_PIN) == HIGH) { driveMotorY(-HOMING_SPEED); }
  driveMotorY(0); 
}

// ==========================================
// ฟังก์ชันเลื่อนไปหาช่องเป้าหมาย (อัปเดตการอ่านค่าแยกรายช่อง)
// ==========================================
void moveToBox(int dayIndex, int slotIndex) {
  homingXY(); 
  
  display.clearDisplay();
  display.setCursor(10, 20);
  display.print("Target: "); display.print(daysOfWeek[dayIndex]);
  display.print(" Slot:"); display.print(slotIndex + 1);
  display.display();
  
  // เคลื่อนที่แกน X ถ้าค่า delay ของช่องเป้าหมายมีค่ามากกว่า 0
  if(delay_X[dayIndex] > 0) {
    driveMotorX(MOVE_SPEED);
    delay(delay_X[dayIndex]); 
    driveMotorX(0);
  }
  
  // เคลื่อนที่แกน Y ถ้าค่า delay ของช่องเป้าหมายมีค่ามากกว่า 0
  if(delay_Y[slotIndex] > 0) {
    driveMotorY(MOVE_SPEED);
    delay(delay_Y[slotIndex]);
    driveMotorY(0);
  }
}

// ==========================================
// [เพิ่ม] รับคำสั่ง/เวลาจาก ESP32-S3 CAM ทาง Serial2
// ==========================================
void camLog(const String &msg) {            // ส่งข้อความกลับไปแสดงในเว็บ + Serial Monitor
  JsonDocument d;
  d["type"] = "log";
  d["msg"] = msg;
  String out;
  serializeJson(d, out);
  CamSerial.println(out);
  Serial.println(msg);
}

void camState(const char *state) {          // ให้โมเดล 3D ในเว็บขยับตาม
  if (activeAlarmDay < 0) return;
  JsonDocument d;
  d["floor"] = activeAlarmDay + 1;
  d["slot"] = activeAlarmSlot + 1;
  d["state"] = state;
  String out;
  serializeJson(d, out);
  CamSerial.println(out);
}

void camStatus(DateTime now) {              // เวลาเครื่อง ส่งไปเว็บทุก 2 วินาที
  char t[9];
  sprintf(t, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  JsonDocument d;
  d["type"] = "status";
  d["rtc"] = t;
  String out;
  serializeJson(d, out);
  CamSerial.println(out);
}

void startBox(int day, int slot, bool onlyMove) {
  if (currentState == ALARM_RINGING || currentState == MOVING_TO_BOX) {
    camLog("เครื่องกำลังทำงานอยู่ ลองใหม่อีกครั้ง");
    return;
  }
  activeAlarmDay = day;
  activeAlarmSlot = slot;
  moveOnly = onlyMove;
  currentState = MOVING_TO_BOX;
}

void handleCamLine(String line) {
  line.trim();
  if (!line.length()) return;

  if (line[0] != '{') {                     // ข้อความธรรมดา เช่น TEST 0 2
    String u = line;
    u.toUpperCase();
    if (u.startsWith("TEST")) {
      int d = -1, sl = -1;
      sscanf(u.c_str(), "TEST %d %d", &d, &sl);
      if (d >= 0 && d <= 6 && sl >= 0 && sl <= 5) startBox(d, sl, false);
      else camLog("รูปแบบ: TEST <day 0-6> <slot 0-5>");
    }
    return;
  }

  JsonDocument doc;
  if (deserializeJson(doc, line)) return;
  const char *cmd = doc["cmd"] | "";

  if (!strcmp(cmd, "time")) {               // เวลาจาก server (ผ่านกล้อง) หรือจากเว็บ
    DateTime t(doc["y"] | 2026, doc["mo"] | 1, doc["d"] | 1, doc["h"] | 0, doc["mi"] | 0, doc["s"] | 0);
    if (t.year() < 2024) return;
    long diff = (long)t.unixtime() - (long)rtc.now().unixtime();
    if (diff > 1 || diff < -1) {            // ปรับเฉพาะเมื่อคลาดเกิน 1 วินาที
      rtc.adjust(t);
      bool ntp = !strcmp(doc["src"] | "", "ntp");
      char t2[9];
      sprintf(t2, "%02d:%02d:%02d", t.hour(), t.minute(), t.second());
      camLog(String(ntp ? "ตั้งเวลาจาก server แล้ว " : "ตั้งเวลาตามเว็บแล้ว ") + t2);
    }
    return;
  }

  int f = doc["floor"] | 0, sl = doc["slot"] | 0;   // เว็บนับ 1-7 / 1-6
  bool okPos = f >= 1 && f <= 7 && sl >= 1 && sl <= 6;
  if (!strcmp(cmd, "test") || !strcmp(cmd, "dispense")) {
    if (okPos) startBox(f - 1, sl - 1, false);
  } else if (!strcmp(cmd, "goto")) {
    if (okPos) startBox(f - 1, sl - 1, true);
  }
  // คำสั่งอื่น (เช่น schedule) โค้ดเดิมยังไม่รองรับ ข้ามไป
}

void checkCamSerial() {
  while (CamSerial.available()) {
    char c = CamSerial.read();
    if (c == '\n') { handleCamLine(camBuf); camBuf = ""; }
    else if (c != '\r' && camBuf.length() < 2000) camBuf += c;
  }
}

void setup() {
  Serial.begin(115200);
  CamSerial.setRxBufferSize(2048);                                  // [เพิ่ม]
  CamSerial.begin(115200, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);      // [เพิ่ม]
  Wire.begin(21, 22);

  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(BUZ_PIN, OUTPUT);
  digitalWrite(BUZ_PIN, LOW);

  pinMode(MOT_X_IN1, OUTPUT);
  pinMode(MOT_X_IN2, OUTPUT);
  pinMode(MOT_Y_IN1, OUTPUT);
  pinMode(MOT_Y_IN2, OUTPUT);
  
  pinMode(SENSOR_X_PIN, INPUT); 
  pinMode(SENSOR_Y_PIN, INPUT);

  ESP32PWM::allocateTimer(0);
  pushServo.setPeriodHertz(50); 
  pushServo.attach(SERVO_PIN, 500, 2400); 
  pushServo.write(0); 

  if (!display.begin(0x3C, true)) {
    Serial.println(F("SH110X allocation failed"));
    for(;;);
  }
  display.display();
  delay(1000);
  display.clearDisplay();
  display.display();

  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.read(0) == MAGIC_BYTE) {
    EEPROM.get(EEPROM_ADDR, alarms);
    Serial.println("Loaded alarms from EEPROM");
  } else {
    Serial.println("Formatting EEPROM...");
    for(int i = 0; i < EEPROM_SIZE; i++) {
      EEPROM.write(i, 0);
    }
    EEPROM.write(0, MAGIC_BYTE);
    EEPROM.put(EEPROM_ADDR, alarms);
    EEPROM.commit();
  }

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    for(;;);
  }
  
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void loop() {
  DateTime now = rtc.now();

  checkCamSerial();                                   // [เพิ่ม] เวลา/คำสั่งจากกล้อง
  if (millis() - lastStatusMs >= 2000) {              // [เพิ่ม]
    lastStatusMs = millis();
    camStatus(now);
  }
  
  if (currentState != ALARM_RINGING && currentState != MOVING_TO_BOX) {
    checkSerialCommands();
    handleJoystick();
  }

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  switch (currentState) {
    case HOME:
      drawHome(now);
      checkAlarms(now);
      if (buttonPressed) currentState = MENU;
      break;

    case MENU:
      drawMenu();
      if (buttonPressed) {
        if (menuIndex == 0) currentState = HOME;
        else if (menuIndex == 1) { 
          currentState = SET_ALARM; editField = 0; 
        }
        else if (menuIndex == 2) { 
          setYear = now.year(); setMonth = now.month();
          setDayRTC = now.day(); setHour = now.hour();
          setMinute = now.minute();
          currentState = SET_TIME; rtcEditField = 0; 
        }
      }
      break;

    case SET_ALARM:
      drawSetAlarm();
      if (buttonPressed && editField == 5) {
        EEPROM.put(EEPROM_ADDR, alarms);
        EEPROM.commit();
        currentState = MENU;
      }
      break;

    case SET_TIME:
      drawSetTime();
      if (buttonPressed && rtcEditField == 5) {
        rtc.adjust(DateTime(setYear, setMonth, setDayRTC, setHour, setMinute, 0));
        currentState = MENU;
      }
      break;
      
    case MOVING_TO_BOX:
      camState("moving");                                         // [เพิ่ม]
      moveToBox(activeAlarmDay, activeAlarmSlot); 
      camState("ready");                                          // [เพิ่ม]
      if (moveOnly) {                                             // [เพิ่ม] goto: ไม่ดันยา ไม่เปิดเพลง
        moveOnly = false;
        camLog("ถึงช่องแล้ว " + daysOfWeek[activeAlarmDay] + " Slot " + String(activeAlarmSlot + 1));
        currentState = HOME;
        break;
      }
      camState("dispensing");                                     // [เพิ่ม]
      operateServoSmoothly();                     
      camLog("ดันยาแล้ว " + daysOfWeek[activeAlarmDay] + " Slot " + String(activeAlarmSlot + 1));  // [เพิ่ม]
      currentState = ALARM_RINGING;               
      break;

    case ALARM_RINGING:
      drawAlarmRinging();
      display.display(); 
      playAlarmSongOnce();
      currentState = HOME;
      buttonPressed = false;
      delay(300); 
      break;
  }
  
  if (currentState != ALARM_RINGING && currentState != MOVING_TO_BOX) {
    display.display();
  }
  
  buttonPressed = false; 
  delay(50);
}

void playAlarmSongOnce() {
  for (int i = 0; i < numNotes; i++) {
    if (alarm_notes[i] > 0) { tone(BUZ_PIN, alarm_notes[i]); } 
    else { noTone(BUZ_PIN); }
    
    unsigned long startT = millis();
    while (millis() - startT < alarm_durations[i]) {
      if (digitalRead(SW_PIN) == LOW) { 
        noTone(BUZ_PIN);
        while (digitalRead(SW_PIN) == LOW) { delay(10); } 
        return; 
      }
      delay(5);
    }
    
    noTone(BUZ_PIN); 
    
    startT = millis();
    while (millis() - startT < 30) {
       if (digitalRead(SW_PIN) == LOW) {
         while (digitalRead(SW_PIN) == LOW) { delay(10); }
         return; 
       }
       delay(5);
    }
  }
}

void handleJoystick() {
  static bool lastBtnState = HIGH;
  
  int x = analogRead(VRX_PIN);
  int y = analogRead(VRY_PIN);
  bool currentBtnState = digitalRead(SW_PIN);

  if (currentBtnState == LOW && lastBtnState == HIGH) {
    buttonPressed = true;
    tone(BUZ_PIN, 2000, 40); delay(50); 
  }
  lastBtnState = currentBtnState;

  if (!joyMoved) {
    bool moved = false;
    if (y > 3000) { processDir(1); moved = true; }      
    else if (y < 1000) { processDir(2); moved = true; } 
    else if (x < 1000) { processDir(3); moved = true; } 
    else if (x > 3000) { processDir(4); moved = true; } 
    
    if (moved) { joyMoved = true; tone(BUZ_PIN, 4000, 15); }
  }
  
  if (x >= 1200 && x <= 2800 && y >= 1200 && y <= 2800) { joyMoved = false; }
}

void processDir(int dir) { 
  if (currentState == MENU) {
    if (dir == 1) menuIndex = (menuIndex - 1 + 3) % 3;
    if (dir == 2) menuIndex = (menuIndex + 1) % 3;
  } 
  else if (currentState == SET_ALARM) {
    if (dir == 3) editField = (editField - 1 + 6) % 6; 
    if (dir == 4) editField = (editField + 1) % 6;     
    
    if (dir == 1 || dir == 2) {
      int step = (dir == 1) ? 1 : -1;
      if (editField == 0) setDay = (setDay + step + 7) % 7;
      if (editField == 1) setSlot = (setSlot + step + 6) % 6;
      if (editField == 2) alarms[setDay][setSlot].hour = (alarms[setDay][setSlot].hour + step + 24) % 24;
      if (editField == 3) alarms[setDay][setSlot].minute = (alarms[setDay][setSlot].minute + step + 60) % 60;
      if (editField == 4) alarms[setDay][setSlot].active = !alarms[setDay][setSlot].active;
    }
  }
  else if (currentState == SET_TIME) {
    if (dir == 3) rtcEditField = (rtcEditField - 1 + 6) % 6; 
    if (dir == 4) rtcEditField = (rtcEditField + 1) % 6;     
    
    if (dir == 1 || dir == 2) {
      int step = (dir == 1) ? 1 : -1;
      
      if (rtcEditField == 0) { 
        setDayRTC += step;
        if (setDayRTC < 1) setDayRTC = 31;
        if (setDayRTC > 31) setDayRTC = 1;
      }
      else if (rtcEditField == 1) { 
        setMonth += step;
        if (setMonth < 1) setMonth = 12;
        if (setMonth > 12) setMonth = 1;
      }
      else if (rtcEditField == 2) { 
        setYear += step;
        if (setYear < 2000) setYear = 2099;
        if (setYear > 2099) setYear = 2000;
      }
      else if (rtcEditField == 3) { setHour = (setHour + step + 24) % 24; }
      else if (rtcEditField == 4) { setMinute = (setMinute + step + 60) % 60; }
    }
  }
}

void checkAlarms(DateTime now) {
  int d = now.dayOfTheWeek();
  for (int i = 0; i < 6; i++) {
    if (alarms[d][i].active && alarms[d][i].hour == now.hour() && alarms[d][i].minute == now.minute() && now.second() == 0) {
      activeAlarmDay = d;    
      activeAlarmSlot = i;   
      currentState = MOVING_TO_BOX; 
    }
  }
}

void drawHome(DateTime now) {
  display.setTextSize(2); display.setCursor(15, 10);
  if(now.hour() < 10) display.print('0'); display.print(now.hour(), DEC); display.print(':'); 
  if(now.minute() < 10) display.print('0'); display.print(now.minute(), DEC); display.print(':');
  if(now.second() < 10) display.print('0'); display.print(now.second(), DEC);
  display.setTextSize(1); display.setCursor(10, 40); display.print(daysOfWeek[now.dayOfTheWeek()]); display.print(" ");
  if(now.day() < 10) display.print('0'); display.print(now.day(), DEC); display.print('/');
  if(now.month() < 10) display.print('0'); display.print(now.month(), DEC); display.print('/');
  display.print(now.year(), DEC);
}

void drawMenu() {
  display.setTextSize(1); display.setCursor(0, 0); display.println("--- MAIN MENU ---");
  String m[3] = {"Back to Clock", "Set Alarms", "Set RTC Time"};
  for (int i=0; i<3; i++) {
    display.setCursor(10, 20 + (i*10));
    if (menuIndex == i) display.print(">"); else display.print(" ");
    display.print(m[i]);
  }
}

void drawSetAlarm() {
  display.setTextSize(1); display.setCursor(0, 0); display.println("--- TIMER SETUP ---");
  display.setCursor(0, 15); if(editField == 0) display.print(">"); display.print("Day:"); display.print(daysOfWeek[setDay]);
  display.setCursor(64, 15); if(editField == 1) display.print(">"); display.print("S:"); display.print(setSlot + 1);
  display.setCursor(0, 30); if(editField == 2) display.print(">"); display.print("H:"); display.print(alarms[setDay][setSlot].hour);
  display.setCursor(64, 30); if(editField == 3) display.print(">"); display.print("M:"); display.print(alarms[setDay][setSlot].minute);
  display.setCursor(0, 45); if(editField == 4) display.print(">"); display.print("St:"); display.print(alarms[setDay][setSlot].active ? "ON" : "OFF");
  display.setCursor(64, 45); if(editField == 5) display.print(">[EXIT]"); else display.print(" [EXIT]");
}

void drawSetTime() {
  display.setTextSize(1); display.setCursor(0, 0); display.println("--- SET RTC TIME ---");
  display.setCursor(0, 20); if(rtcEditField == 0) display.print(">"); if(setDayRTC < 10) display.print("0"); display.print(setDayRTC); display.print("/");
  if(rtcEditField == 1) display.print(">"); if(setMonth < 10) display.print("0"); display.print(setMonth); display.print("/");
  if(rtcEditField == 2) display.print(">"); display.print(setYear);
  display.setCursor(0, 35); display.print("Time: "); if(rtcEditField == 3) display.print(">"); if(setHour < 10) display.print("0"); display.print(setHour); display.print(":");
  if(rtcEditField == 4) display.print(">"); if(setMinute < 10) display.print("0"); display.print(setMinute);
  display.setCursor(64, 50); if(rtcEditField == 5) display.print(">[SAVE]"); else display.print(" [SAVE]");
}

void drawAlarmRinging() {
  display.setTextSize(2); display.setCursor(10, 20); display.println("ALARM!");
  display.setTextSize(1); display.setCursor(10, 45); display.print(daysOfWeek[activeAlarmDay]); display.print(" Slot:"); display.println(activeAlarmSlot + 1);
}