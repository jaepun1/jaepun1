/*
  เครื่องจ่ายยา JaiYA — โค้ดเดิมของเครื่อง + เชื่อมกับเว็บแอปผ่าน ESP32-S3 CAM
  หลักการเดิมไม่เปลี่ยน: แกน X = วันในสัปดาห์ (SUN..SAT), แกน Y = ช่อง 1-6, Homing ทุกครั้ง, เซอร์โวดันยา, เพลงมาริโอ้
  ส่วนที่เพิ่ม/แก้มีป้าย [เพิ่ม] หรือ [แก้]

  การต่อสาย (ใช้ Serial เดิม)
    ESP32-S3 CAM ขา 21 (TX)  ->  บอร์ดนี้ RXD
    ESP32-S3 CAM ขา 14 (RX)  <-  บอร์ดนี้ TXD    (ให้เว็บเห็นสถานะ / สั่งถ่ายภาพ)
    GND                      ->  GND
    เซนเซอร์ตรวจยา Sharp GP2Y0A41SK0F (วัดระยะ 4-30 ซม.):
      สายแดง -> 5V, สายดำ -> GND, สายเหลือง -> VP (GPIO36)   (ไม่มีให้ตั้ง USE_PILL_SENSOR 0)
      ติดให้ห่างจุดที่กล่องยาตก 5-10 ซม. (ใกล้กว่า 4 ซม. วัดไม่ได้)
      จำค่าจากเว็บ: แท็บ SETTING ปุ่ม "จำค่าถาดว่าง" และ "จำค่าตอนมียา" (หรือ {"cmd":"calibrate","state":"empty"|"pill"|"reset"})
    ⚠️ ตอนอัปโหลดโค้ดลงบอร์ดนี้ ถอดสายที่ RXD และ TXD ออกก่อน แล้วค่อยเสียบกลับ

  รับจากเว็บ (ผ่านกล้อง)                         ส่งกลับไปเว็บ
    {"cmd":"time",...}      ตั้ง RTC                 {"type":"status","rtc":"08:00:05","floor":1,"slot":1,"pill":false,"ir":3500}
    {"cmd":"schedule",...}  ตั้งเวลายา              {"floor":1,"slot":1,"state":"moving"|"ready"|"dispensing"}
    {"cmd":"test",floor,slot}     ทดสอบครบวงจร       {"type":"taken","floor":1,"slot":1,"at":"2026-11-15T08:03:10"}
    {"cmd":"dispense",floor,slot} ไปช่อง+ดันยา+เพลง  {"type":"snap",...} ให้กล้องถ่ายภาพ, {"type":"test",...}, {"type":"log",...}
    {"cmd":"goto",floor,slot}     ไปช่องอย่างเดียว
    {"cmd":"ir"} {"cmd":"cancel"} {"cmd":"calibrate","state":"empty"|"pill"}   และ TEST <day 0-6> <slot 0-5> แบบเดิม
  ชั้น 1-7 ในเว็บ = วัน SUN..SAT (แกน X), ช่อง 1-6 = Slot 1-6 (แกน Y)
*/

#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h> 
#include <EEPROM.h>
#include <ESP32Servo.h> 
#include <ArduinoJson.h>   // [เพิ่ม] ArduinoJson v7

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

// [เพิ่ม] ประเภทงานที่เครื่องกำลังทำ (ต้องประกาศไว้ด้านบน ก่อนฟังก์ชันแรก)
enum DoseMode { DM_NONE, DM_SCHEDULED, DM_MANUAL, DM_TEST };
DoseMode doseMode = DM_NONE;

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
// [เพิ่ม] เชื่อมกับเว็บ: เซนเซอร์ตรวจยา, ติดตามมื้อยา, ส่งข้อความ JSON
// ==========================================
#define USE_PILL_SENSOR 1          // ไม่ได้ต่อเซนเซอร์ตรวจยา ให้เป็น 0
#define PILL_PIN 36                // VP
// ค่าเริ่มต้น (ใช้เมื่อยังไม่ได้จำค่า): Sharp GP2Y0A41SK0F วัตถุยิ่งใกล้ ค่ายิ่งสูง
const bool PILL_WHEN_LOW = false;
const int  PILL_THRESHOLD = 1500;  // เส้นแบ่ง มียา/ไม่มียา (0-4095)
const int  PILL_HYST = 150;
// ค่าที่จำจากเครื่องจริง (บันทึกใน EEPROM ตำแหน่ง 200 ไม่ทับตารางเวลา)
#define CAL_ADDR  200
#define CAL_MAGIC 0x5C
struct PillCal { uint8_t magic; uint8_t hasEmpty; uint8_t hasPill; uint16_t emptyVal; uint16_t pillVal; };
PillCal pillCal = {0, 0, 0, 0, 0};
const unsigned long PILL_DETECT_MS = 5000;           // รอยาตกลงถาดหลังดัน
const unsigned long TAKEN_DEBOUNCE_MS = 1500;        // ถาดว่างนานเท่านี้ = หยิบแล้ว
const unsigned long REMIND_MS = 5UL * 60 * 1000;     // เปิดเพลงซ้ำทุก 5 นาทีถ้ายายังอยู่
const uint8_t       MAX_RINGS = 3;                   // เล่นเพลงสูงสุดกี่รอบ (รวมรอบแรก)
const unsigned long MISSED_MS = 60UL * 60 * 1000;    // เกิน 1 ชม. = ไม่ได้รับยา
const unsigned long TEST_TIMEOUT_MS = 2UL * 60 * 1000;

bool moveOnly = false;            // goto: ไปช่องอย่างเดียว
bool doseActive = false;          // กำลังรอให้หยิบยา
bool pillSeen = false;
unsigned long doseStartMs = 0, lastRingMs = 0, clearSinceMs = 0, lastStatusMs = 0;
uint8_t rings = 0;
int posDay = 0, posSlot = 0;      // ตำแหน่งล่าสุด

void sendJson(JsonDocument &d) { String out; serializeJson(d, out); Serial.println(out); }

void webLog(const String &msg) {
  JsonDocument d; d["type"] = "log"; d["msg"] = msg; sendJson(d);
}

void webState(const char *state) {
  JsonDocument d; d["floor"] = activeAlarmDay + 1; d["slot"] = activeAlarmSlot + 1; d["state"] = state; sendJson(d);
}

void webSnap(const char *phase) {        // ให้กล้องถ่ายภาพ (มื้อจริง: dispensed/taken, ทดสอบ: test_...)
  JsonDocument d;
  d["type"] = "snap"; d["floor"] = activeAlarmDay + 1; d["slot"] = activeAlarmSlot + 1;
  if (doseMode == DM_SCHEDULED) d["phase"] = phase;
  else d["phase"] = strcmp(phase, "taken") == 0 ? "test_taken" : "test_dispensed";
  sendJson(d);
}

void webTest(const char *step, bool ok, const String &msg) {   // รายงานผลทดสอบทีละขั้น
  if (doseMode != DM_TEST) return;
  JsonDocument d;
  d["type"] = "test"; d["floor"] = activeAlarmDay + 1; d["slot"] = activeAlarmSlot + 1;
  d["step"] = step; d["ok"] = ok;
  if (msg.length()) d["msg"] = msg;
  sendJson(d);
}

// เซนเซอร์ Sharp ค่ากระเพื่อมเป็นจังหวะ (ทุก ~16 มิลลิวินาที): อ่าน 15 ครั้งในช่วง ~25 มิลลิวินาที แล้วใช้ค่ากลาง (median)
// ตัดค่าที่กระโดดทิ้ง จากนั้นเฉลี่ยกับค่าก่อนหน้าอีกชั้นให้นิ่งขึ้น
int pillRaw() {
  int v[15];
  for (int i = 0; i < 15; i++) { v[i] = analogRead(PILL_PIN); delayMicroseconds(1700); }
  for (int i = 1; i < 15; i++) {             // เรียงลำดับ (insertion sort)
    int x = v[i], j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
  static int smooth = -1;
  int med = v[7];
  smooth = (smooth < 0) ? med : (smooth * 2 + med) / 3;
  return smooth;
}

// เส้นแบ่งที่ใช้อยู่: จากค่าที่จำไว้ ถ้ามี ไม่อย่างนั้นใช้ค่าเริ่มต้น
void pillRule(int &thr, int &hyst, bool &whenLow) {
  int e = pillCal.emptyVal, p = pillCal.pillVal;
  if (pillCal.hasEmpty && pillCal.hasPill && abs(e - p) >= 100) {
    thr = (e + p) / 2; whenLow = p < e; hyst = abs(e - p) / 6; if (hyst < 30) hyst = 30;
  } else if (pillCal.hasEmpty) {               // จำแค่ถาดว่าง: มียา = ค่าสูงกว่าถาดว่างชัดเจน
    thr = e + 400; whenLow = false; hyst = 100;
  } else {
    thr = PILL_THRESHOLD; whenLow = PILL_WHEN_LOW; hyst = PILL_HYST;
  }
}

const uint8_t PILL_CONFIRM = 3;    // ต้องเห็นค่าฝั่งใหม่ติดกันกี่ครั้งถึงจะเปลี่ยนสถานะ (กันค่ากระโดด)

bool pillPresent() {
#if USE_PILL_SENSOR
  static bool state = false;
  static uint8_t streak = 0;
  int thr, hyst; bool whenLow;
  pillRule(thr, hyst, whenLow);
  int v = pillRaw();
  bool low = v < thr - hyst, high = v > thr + hyst;
  bool seesPill = whenLow ? low : high;
  bool seesEmpty = whenLow ? high : low;
  if ((!state && seesPill) || (state && seesEmpty)) {
    if (++streak >= PILL_CONFIRM) { state = !state; streak = 0; }
  } else {
    streak = 0;                       // ค่ายังอยู่ฝั่งเดิม หรืออยู่ช่วงกลาง
  }
  return state;
#else
  return false;
#endif
}

// จำค่าเซนเซอร์: state = "empty" (ถาดว่าง) / "pill" (มีกล่องยา) / "reset"
void calibratePill(const char *st) {
  if (!strcmp(st, "reset")) {
    pillCal = {CAL_MAGIC, 0, 0, 0, 0};
  } else {
    long sum = 0;
    for (int i = 0; i < 4; i++) { sum += pillRaw(); delay(50); }
    int v = sum / 4;
    pillCal.magic = CAL_MAGIC;
    if (!strcmp(st, "pill")) { pillCal.pillVal = v; pillCal.hasPill = 1; }
    else                     { pillCal.emptyVal = v; pillCal.hasEmpty = 1; }
  }
  EEPROM.put(CAL_ADDR, pillCal);
  EEPROM.commit();
  int thr, hyst; bool whenLow;
  pillRule(thr, hyst, whenLow);
  String msg = "จำค่าเซนเซอร์: ถาดว่าง=" + (pillCal.hasEmpty ? String(pillCal.emptyVal) : String("-")) +
               " มียา=" + (pillCal.hasPill ? String(pillCal.pillVal) : String("-")) +
               " เส้นแบ่ง=" + String(thr) + (whenLow ? " (มียา=ค่าต่ำ)" : " (มียา=ค่าสูง)");
  if (pillCal.hasEmpty && pillCal.hasPill && abs((int)pillCal.emptyVal - (int)pillCal.pillVal) < 100)
    msg += " ⚠️ สองค่าต่างกันน้อยเกินไป ตรวจตำแหน่งเซนเซอร์ (ห่างจุดที่กล่องตก 5-10 ซม.)";
  webLog(msg);
}

String slotName(int day, int slot) { return daysOfWeek[day] + " Slot " + String(slot + 1); }

void endDose() { doseActive = false; doseMode = DM_NONE; }

// เริ่มงาน: ไปช่อง (+ดันยา +เพลง)
void startJob(int day, int slot, DoseMode mode, bool onlyMove) {
  if (currentState == ALARM_RINGING || currentState == MOVING_TO_BOX) { webLog("เครื่องกำลังทำงานอยู่"); return; }
  if (doseActive) {                                   // ปิดงานเก่าที่ยังรอหยิบยา
    if (doseMode == DM_TEST) { webTest("taken", false, "ถูกแทรกด้วยคำสั่งใหม่"); webTest("done", false, ""); }
    endDose();
  }
  activeAlarmDay = day;
  activeAlarmSlot = slot;
  doseMode = mode;
  moveOnly = onlyMove;
  currentState = MOVING_TO_BOX;
}

// หลังดันยา: รอเซนเซอร์เจอยา แล้วสั่งถ่ายภาพที่ 1
void afterPush() {
  pillSeen = false;
  clearSinceMs = 0;
  rings = 0;
  doseStartMs = millis();
  doseActive = true;
#if USE_PILL_SENSOR
  while (millis() - doseStartMs < PILL_DETECT_MS) {
    if (pillPresent()) { pillSeen = true; break; }
    delay(20);
  }
  if (pillSeen) {
    webSnap("dispensed");
    webLog("ยาลงถาดแล้ว " + slotName(activeAlarmDay, activeAlarmSlot));
    webTest("pill", true, "ค่าเซนเซอร์ " + String(pillRaw()));
  } else {
    webLog("เซนเซอร์ไม่พบยาในถาด " + slotName(activeAlarmDay, activeAlarmSlot));
    webTest("pill", false, "ไม่พบยาภายใน 5 วินาที (ค่าเซนเซอร์ " + String(pillRaw()) + ")");
  }
#else
  webSnap("dispensed");                               // ไม่มีเซนเซอร์: ถ่ายภาพหลังดันเลย
  webTest("pill", false, "ไม่ได้เปิดใช้เซนเซอร์ตรวจยา");
#endif
}

// ติดตามมื้อยา: หยิบแล้ว / เตือนซ้ำ / ไม่ได้รับยา
void updateDose() {
  if (!doseActive) return;
  bool present = pillPresent();

  if (!pillSeen) {
    if (present) { pillSeen = true; webSnap("dispensed"); }        // ยามาถึงช้า
  } else if (!present) {
    if (!clearSinceMs) clearSinceMs = millis();
    if (millis() - clearSinceMs >= TAKEN_DEBOUNCE_MS) {
      if (doseMode == DM_SCHEDULED) {
        DateTime n = rtc.now();
        char at[20];
        sprintf(at, "%04d-%02d-%02dT%02d:%02d:%02d", n.year(), n.month(), n.day(), n.hour(), n.minute(), n.second());
        JsonDocument d;
        d["type"] = "taken"; d["floor"] = activeAlarmDay + 1; d["slot"] = activeAlarmSlot + 1; d["at"] = at;
        sendJson(d);
      } else {
        webLog("หยิบยาออกจากถาดแล้ว " + slotName(activeAlarmDay, activeAlarmSlot));
      }
      webSnap("taken");                                            // ภาพที่ 2
      webTest("taken", true, "");
      webTest("done", true, "");
      endDose();
      return;
    }
  } else {
    clearSinceMs = 0;
  }

  if (doseMode == DM_SCHEDULED && pillSeen && present && rings < MAX_RINGS &&
      millis() - lastRingMs >= REMIND_MS && currentState == HOME) {
    webLog("ยังไม่ได้หยิบยา เตือนครั้งที่ " + String(rings + 1));
    currentState = ALARM_RINGING;                                  // เปิดเพลงซ้ำ
  }

  if (doseMode == DM_TEST && millis() - doseStartMs >= TEST_TIMEOUT_MS) {
    webTest("taken", false, "ไม่ได้หยิบยาภายใน 2 นาที");
    webTest("done", false, "");
    endDose();
    return;
  }
  if (millis() - doseStartMs >= MISSED_MS) {
    webLog("ไม่ได้รับยา " + slotName(activeAlarmDay, activeAlarmSlot));
    endDose();
  }
}

void sendStatus(DateTime now) {
  char t[9];
  sprintf(t, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  JsonDocument d;
  d["type"] = "status"; d["rtc"] = t;
  d["floor"] = posDay + 1; d["slot"] = posSlot + 1;
#if USE_PILL_SENSOR
  d["pill"] = pillPresent(); d["ir"] = pillRaw();
#endif
  sendJson(d);
}

// ตารางเวลาจากเว็บ -> alarms[วันในสัปดาห์][ช่อง] (ใช้หลักการเดิม: เตือนซ้ำทุกสัปดาห์)
void applySchedule(JsonObject data) {
  AlarmTimer na[7][6];
  int days = 0, count = 0;
  for (JsonObject fo : data["floors"].as<JsonArray>()) {
    int y, m, dd;
    if (sscanf(fo["date"] | "", "%d-%d-%d", &y, &m, &dd) != 3) continue;
    int wd = DateTime(y, m, dd).dayOfTheWeek();
    days++;
    for (JsonObject so : fo["slots"].as<JsonArray>()) {
      int si = (so["slot"] | 0) - 1;
      int h, mi;
      if (si < 0 || si > 5) continue;
      if ((so["on"] | false) && sscanf(so["time"] | "", "%d:%d", &h, &mi) == 2) {
        na[wd][si].hour = h; na[wd][si].minute = mi; na[wd][si].active = true;
        count++;
      }
    }
  }
  memcpy(alarms, na, sizeof(alarms));
  EEPROM.put(EEPROM_ADDR, alarms);
  EEPROM.commit();
  webLog("รับตารางเวลา " + String(days) + " วัน " + String(count) + " มื้อแล้ว");
}

// ข้อความ JSON จากกล้อง/เว็บ
void handleJsonCommand(String line) {
  JsonDocument doc;
  if (deserializeJson(doc, line)) return;
  const char *cmd = doc["cmd"] | "";
  int f = doc["floor"] | 0, sl = doc["slot"] | 0;          // เว็บนับ 1-7 / 1-6
  bool okPos = f >= 1 && f <= 7 && sl >= 1 && sl <= 6;

  if (!strcmp(cmd, "time")) {
    DateTime t(doc["y"] | 2026, doc["mo"] | 1, doc["d"] | 1, doc["h"] | 0, doc["mi"] | 0, doc["s"] | 0);
    if (t.year() < 2024) return;
    long diff = (long)t.unixtime() - (long)rtc.now().unixtime();
    if (diff > 1 || diff < -1) {                              // ปรับเฉพาะเมื่อคลาดเกิน 1 วินาที
      rtc.adjust(t);
      webLog(strcmp(doc["src"] | "", "ntp") == 0 ? "ตั้งเวลาจาก server แล้ว" : "ตั้งเวลาตามเว็บแล้ว");
    }
  }
  else if (!strcmp(cmd, "schedule")) applySchedule(doc["data"].as<JsonObject>());
  else if (!strcmp(cmd, "test"))     { if (okPos) startJob(f - 1, sl - 1, DM_TEST, false); }
  else if (!strcmp(cmd, "dispense")) { if (okPos) startJob(f - 1, sl - 1, DM_MANUAL, false); }
  else if (!strcmp(cmd, "goto"))     { if (okPos) startJob(f - 1, sl - 1, DM_NONE, true); }
  else if (!strcmp(cmd, "ir")) {
#if USE_PILL_SENSOR
    int thr, hyst; bool whenLow;
    pillRule(thr, hyst, whenLow);
    webLog("ค่าเซนเซอร์ตรวจยา = " + String(pillRaw()) + (pillPresent() ? " (มียา)" : " (ไม่มียา)") +
           " · เส้นแบ่ง " + String(thr));
#else
    webLog("ไม่ได้เปิดใช้เซนเซอร์ตรวจยา");
#endif
  }
  else if (!strcmp(cmd, "calibrate")) calibratePill(doc["state"] | "empty");
  else if (!strcmp(cmd, "cancel")) {
    if (doseActive && doseMode != DM_SCHEDULED) {
      if (doseMode == DM_TEST) { webTest("taken", false, "ยกเลิกการทดสอบ"); webTest("done", false, ""); }
      endDose();
      webLog("ยกเลิกแล้ว");
    }
  }
  // ข้อความอื่นจากกล้อง (เช่น สถานะ Wi-Fi) ข้ามไป
}

// ==========================================
// ฟังก์ชันตรวจสอบคำสั่งจาก Serial
// ==========================================
void checkSerialCommands() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("{")) { handleJsonCommand(cmd); return; }   // [เพิ่ม] ข้อความจากกล้อง/เว็บ
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
          
          startJob(dayTest, slotTest, DM_MANUAL, false);   // [แก้] เดิม: ตั้ง activeAlarmDay/Slot แล้วไป MOVING_TO_BOX
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

void setup() {
  Serial.setRxBufferSize(2048);   // [เพิ่ม] ข้อความตารางเวลาจากเว็บยาว
  Serial.begin(115200);
  Serial.setTimeout(200);         // [เพิ่ม]
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
  pinMode(PILL_PIN, INPUT);                       // [เพิ่ม]
  analogSetPinAttenuation(PILL_PIN, ADC_11db);    // [เพิ่ม] อ่านได้ 0-3.3V

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
  EEPROM.get(CAL_ADDR, pillCal);                                  // [เพิ่ม] ค่าเซนเซอร์ที่จำไว้
  if (pillCal.magic != CAL_MAGIC) pillCal = {CAL_MAGIC, 0, 0, 0, 0};

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
  
  if (currentState != ALARM_RINGING && currentState != MOVING_TO_BOX) {
    checkSerialCommands();
    handleJoystick();
    updateDose();                                 // [เพิ่ม]
  }
  if (millis() - lastStatusMs >= 2000) {          // [เพิ่ม] สถานะให้เว็บทุก 2 วินาที
    lastStatusMs = millis();
    sendStatus(now);
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
      webTest("start", true, "");                                    // [เพิ่ม]
      webState("moving");                                        // [เพิ่ม]
      moveToBox(activeAlarmDay, activeAlarmSlot); 
      posDay = activeAlarmDay; posSlot = activeAlarmSlot;        // [เพิ่ม]
      webState("ready"); webTest("move", true, "");                  // [เพิ่ม]
      if (moveOnly) {                                            // [เพิ่ม] goto: ไม่ดันยา ไม่เปิดเพลง
        moveOnly = false; doseMode = DM_NONE;
        webLog("ถึงช่องแล้ว " + slotName(activeAlarmDay, activeAlarmSlot));
        currentState = HOME;
        break;
      }
      webState("dispensing");                                    // [เพิ่ม]
      operateServoSmoothly();                     
      webState("ready"); webTest("push", true, "");                  // [เพิ่ม]
      afterPush();                                               // [เพิ่ม] เซนเซอร์ + ภาพที่ 1
      webTest("ring", true, "");                                     // [เพิ่ม]
      currentState = ALARM_RINGING;               
      break;

    case ALARM_RINGING:
      drawAlarmRinging();
      display.display(); 
      rings++; lastRingMs = millis();                            // [เพิ่ม]
      playAlarmSongOnce();
      if (doseActive && doseMode == DM_TEST && !pillSeen) {      // [เพิ่ม] ทดสอบแต่ไม่พบยา: จบการทดสอบ
        webTest("taken", false, "ข้าม เพราะไม่พบยาในถาด");
        webTest("done", false, "");
        endDose();
      }
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
      if (doseActive && pillSeen && !pillPresent()) { noTone(BUZ_PIN); return; }   // [เพิ่ม] หยิบยาแล้ว หยุดเพลง
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
      startJob(d, i, DM_SCHEDULED, false);   // [แก้] เดิม: ตั้ง activeAlarmDay/Slot แล้วไป MOVING_TO_BOX
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