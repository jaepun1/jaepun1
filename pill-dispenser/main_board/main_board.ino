/*
  เครื่องจ่ายยา 7 ชั้น × 6 ช่อง — บอร์ดหลัก (ESP32 DevKit)
  ต่อยอดจากโปรเจกต์ Wokwi "Jaiya" (จอ OLED, RTC, จอยสติก, บัซเซอร์, เพลงมาริโอ้)

  การไหลของข้อมูล
    เว็บแอป  --WebSocket-->  ESP32-S3 CAM  --UART (JSON ทีละบรรทัด)-->  บอร์ดนี้
    บอร์ดนี้ --UART-->  ESP32-S3 CAM  --WebSocket-->  เว็บแอป

  คำสั่งจากเว็บ (รูปแบบเดียวกับที่เว็บแอปส่งอยู่แล้ว)
    {"cmd":"time","y":2026,"mo":11,"d":11,"h":8,"mi":0,"s":0}
    {"cmd":"schedule","data":{"floors":[{"floor":1,"date":"2026-11-11","slots":[{"slot":1,"on":true,"time":"07:00"},...]}]}}
    {"cmd":"goto","floor":3,"slot":4}        หมุนถาด + เลื่อนตัวดัน
    {"cmd":"dispense","floor":3,"slot":4}    หมุน + ดันยา
    {"cmd":"stop"}                           หยุดเพลง
    {"cmd":"ir"}                             ดูค่าเซนเซอร์ตรวจยา (ใช้ตั้งค่า)
    {"cmd":"test","floor":3,"slot":4}        ทดสอบครบวงจร: หมุน ดัน เซนเซอร์ ภาพ เพลง หยิบยา ภาพ
    {"cmd":"cancel"}                         ยกเลิกการทดสอบ

  ข้อความที่ส่งกลับไปเว็บ
    {"type":"status","rtc":"08:00:05","floor":3,"slot":4}     ทุก 2 วินาที
    {"floor":3,"slot":4,"state":"moving"|"ready"|"dispensing"}
    {"type":"taken","floor":3,"slot":4,"at":"2026-11-11T08:03:10"}
    {"type":"log","msg":"..."}
    {"type":"test","floor":3,"slot":4,"step":"move"|"push"|"pill"|"ring"|"taken"|"done","ok":true,"msg":"..."}
    {"type":"snap","floor":3,"slot":4,"phase":"dispensed"|"taken"}   (ส่งให้กล้องถ่ายภาพ ไม่ส่งต่อไปเว็บ)

  ทดสอบใน Wokwi (ไม่มีกล้อง): พิมพ์คำสั่ง JSON ข้างบนลงใน Serial Monitor ได้เลย
*/

#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>   // ArduinoJson v7
#include <Preferences.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

RTC_DS1307 rtc;            // ใช้ได้ทั้ง DS1307 และ DS3231 (เก็บเวลาในรีจิสเตอร์ตำแหน่งเดียวกัน) ไม่ต้องแก้โค้ด
// แหล่งเวลา: กล้อง ESP32-S3 CAM ดึงเวลาจากอินเทอร์เน็ต (NTP) แล้วส่งมาทุก 10 นาที / เว็บส่งมาตอนเชื่อมต่อ
// RTC ช่วยจำเวลาตอนไฟดับ ถ้าไม่มี RTC เครื่องจะใช้นาฬิกาในบอร์ดแทน และรอเวลาจากกล้อง
bool rtcOk = false;
uint32_t swBaseUnix = 0, swBaseMs = 0;   // นาฬิกาในบอร์ด (ใช้เมื่อไม่มี RTC)
bool timeValid = false;                  // ได้เวลาที่ถูกต้องแล้วหรือยัง

// ==========================================
// ขา Pin (ESP32 DevKit)
// ==========================================
// --- เดิมจากโปรเจกต์ Wokwi ---
#define VRX_PIN 34
#define VRY_PIN 35
#define SW_PIN  25
#define BUZ_PIN 26
// I2C: SDA 21, SCL 22 (OLED + RTC)

// --- เพิ่มใหม่ ---
#define CAM_RX_PIN   16   // รับจาก ESP32-S3 CAM (ต่อกับขา TX ของกล้อง)
#define CAM_TX_PIN   17   // ส่งไป ESP32-S3 CAM (ต่อกับขา RX ของกล้อง)
#define SERVO_TRAY   18   // หมุนถาดยาทั้งแท่งไปช่อง 1-6
#define SERVO_PUSH   19   // แขนดันยาออกจากช่อง
#define LIFT_IN1     32   // L298N IN1  เลื่อนตัวดันขึ้น
#define LIFT_IN2     33   // L298N IN2  เลื่อนตัวดันลง
#define LIFT_EN      23   // L298N ENA  ความเร็ว (PWM)
#define IR_PIN       36   // เซนเซอร์ตรวจยาในถาด EC-BOT PU (analog) ขา VP / ADC1
#define LED_OK       13   // LED เขียว: ต่อเว็บแอปแล้ว
#define LED_PILL     14   // LED แดง: มียาค้างในถาด
#define BTN_TEST     4    // ปุ่มเขียว: ทดสอบจ่ายยามื้อถัดไปทันที
#define BTN_STOP     5    // ปุ่มน้ำเงิน: หยุดเพลง

// ==========================================
// ค่าที่ปรับได้
// ==========================================
// เซนเซอร์ EC-BOT PU (LED ฟ้า + phototransistor) ให้ค่า analog: สะท้อนแสงมาก (สีขาว) = ค่าต่ำ, ไม่สะท้อน (สีดำ/ว่าง) = ค่าสูง
// ESP32 อ่านได้ 0-4095 (บอร์ด Arduino UNO อ่าน 0-1023 ค่าบนกล่อง ขาว <100, ดำ >800 จึงเท่ากับประมาณ ขาว <400, ดำ >3200 บน ESP32)
// ดูค่าจริงได้จากช่อง "ir" ในข้อความ status หรือส่งคำสั่ง {"cmd":"ir"} แล้วตั้ง IR_THRESHOLD ไว้กึ่งกลางระหว่าง "ถาดว่าง" กับ "มียา"
const bool IR_PILL_WHEN_LOW = true;   // ยาสีขาวบนพื้นถาดสีดำ = ค่าต่ำเมื่อมียา (ถ้ายาสีเข้มบนพื้นขาว ให้เป็น false)
const int  IR_THRESHOLD = 2000;       // เส้นแบ่ง มียา / ไม่มียา (0-4095)
const int  IR_HYST = 200;             // กันค่ากระพือรอบเส้นแบ่ง
const uint8_t  TRAY_ANGLE[6] = {0, 30, 60, 90, 120, 150};  // มุมเซอร์โวของช่อง 1-6
const uint8_t  PUSH_REST = 0, PUSH_OUT = 90;               // มุมแขนดันยา
const uint16_t LIFT_MS_PER_FLOOR = 800;   // เวลาที่มอเตอร์ใช้เลื่อน 1 ชั้น (ต้องจับเวลากับเครื่องจริง)
const uint8_t  LIFT_PWM = 200;            // ความเร็วมอเตอร์ 0-255
const bool     LIFT_HOME_ON_BOOT = true;  // เปิดเครื่องแล้วเลื่อนตัวดันลงชั้น 1 ก่อน

const uint32_t PILL_DETECT_MS   = 5000;            // รอให้ยาตกลงถาดหลังดัน
const uint32_t TAKEN_DEBOUNCE   = 1500;            // ถาดว่างนานเท่านี้ = หยิบแล้ว
const uint32_t REMIND_MS        = 5UL * 60 * 1000; // เปิดเพลงซ้ำทุก 5 นาทีถ้ายายังอยู่
const uint8_t  MAX_RINGS        = 3;               // เล่นเพลงสูงสุดกี่รอบ (รวมรอบแรก)
const uint32_t MISSED_MS        = 60UL * 60 * 1000;// เกิน 1 ชม. = ไม่ได้รับยา (ตรงกับเว็บ)
const uint8_t  GRACE_MIN        = 2;               // ถ้าเลยเวลาไม่เกินกี่นาทียังจ่ายให้ (เช่น ไฟดับชั่วครู่)
const uint32_t STATUS_MS        = 2000;
const uint32_t TEST_TIMEOUT_MS  = 2UL * 60 * 1000;// โหมดทดสอบ: รอหยิบยาไม่เกิน 2 นาที

// ==========================================
// System States (เดิม)
// ==========================================
enum State { HOME, MENU, SET_TIME, SET_ALARM, ALARM_RINGING };
State currentState = HOME;

int  menuIndex = 0;
bool buttonPressed = false;
bool joyMoved = false;

// ==========================================
// ตารางเวลา: 7 ชั้น (1 ชั้น = 1 วัน ตามวันที่จากเว็บ) × 6 ช่อง
// ==========================================
enum DoneState : uint8_t { DOSE_PENDING = 0, DOSE_DISPENSED = 1, DOSE_TAKEN = 2, DOSE_MISSED = 3 };
struct AlarmTimer {
  uint8_t hour = 0;
  uint8_t minute = 0;
  bool    active = false;
  uint8_t done = DOSE_PENDING;
};
struct FloorDay {
  char date[11] = "";        // "YYYY-MM-DD" ว่าง = ชั้นนี้ไม่ได้ใช้
  AlarmTimer slot[6];
};
FloorDay floors[7];
Preferences prefs;

int setFloor = 0;   // (เดิมคือ setDay) ชั้นที่กำลังแก้ด้วยจอยสติก
int setSlot = 0;
int editField = 0;
bool scheduleEdited = false;

int rtcEditField = 0;
DateTime tempTime;

String daysOfWeek[7] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

// ==========================================
// กลไก + มื้อยาที่กำลังติดตาม
// ==========================================
Servo servoTray, servoPush;
int curFloor = 1, curSlot = 1;   // ตำแหน่งตัวดัน / ช่องที่ตรงตัวดัน
int liftFloor = 1;

// โหมดของการจ่ายยา
enum DoseMode : uint8_t {
  DOSE_SCHEDULED,   // มื้อจริงตามตาราง: เพลง + เตือนซ้ำ + แจ้งเว็บว่ารับยาแล้ว
  DOSE_MANUAL,      // ปุ่ม "หมุนและดันยา" ในเว็บ: ไม่มีเพลง ไม่บันทึกลงตาราง
  DOSE_TEST         // คำสั่ง test: ทดสอบครบทุกขั้น รายงานผลทีละขั้นให้เว็บ ไม่บันทึกลงตาราง
};

struct Dose {
  bool     active = false;
  int      f = 0, s = 0;          // 1-based
  uint8_t  mode = DOSE_SCHEDULED;
  bool     scheduled = false;     // mode == DOSE_SCHEDULED
  bool     pillSeen = false;
  uint32_t startMs = 0, lastRingMs = 0, clearSinceMs = 0;
  uint8_t  rings = 0;
} dose;

// ==========================================
// การสื่อสาร
// ==========================================
HardwareSerial &CamSerial = Serial2;
String rxCam, rxUsb;
bool netWifi = false;
int  netClients = 0;
String netIp = "";
uint32_t lastNetMs = 0, lastStatusMs = 0;

// ==========================================
// เพลง Super Mario Bros. Theme (เดิม)
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

volatile bool stopRequested = false;   // คำสั่ง {"cmd":"stop"} จากเว็บ

// ---------- ประกาศฟังก์ชันล่วงหน้า ----------
void sendJson(JsonDocument &doc);
void sendLog(const String &msg);
void sendState(const char *state);
void sendStatus(const DateTime &now);
void requestPhoto(int f, int s, const char *phase);
String two(int v);
String hms(const DateTime &t);
String isoDate(const DateTime &t);
String isoDateTime(const DateTime &t);
void pollStream(Stream &st, String &buf);
void pollComms();
void handleLine(String &line);
bool validPos(int f, int s);
void loadSchedule();
void saveSchedule();
FloorDay *findDay(const char *date);
void applySchedule(JsonObject data);
void checkAlarms(DateTime now);
bool nextDose(const DateTime &now, int &nf, int &ns);
void moveLift(int f);
void moveTo(int f, int s);
void pushPill();
int  irRaw();
bool pillPresent();
DateTime clockNow();
void clockSet(const DateTime &t);
void startDose(int f, int s, uint8_t mode);
void testStep(const char *step, bool ok, const String &msg = "");
void endDose();
const char *photoPhase(const char *phase);
void ring();
void markDone(int f, int s, uint8_t st);
void updateDose();
void updateLeds();
bool shouldStopSong();
void playAlarmSongOnce();
void handleButtons();
void handleJoystick();
void processDir(int dir);
void drawHome(DateTime now);
void drawMenu();
void drawSetAlarm();
void drawSetTime();
void drawAlarmRinging();

// ==========================================
// setup / loop
// ==========================================
void setup() {
  Serial.begin(115200);
  CamSerial.setRxBufferSize(4096);   // ตารางเวลาเป็น JSON ยาว ~1.5 KB
  CamSerial.begin(115200, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);
  Wire.begin(21, 22);

  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(BUZ_PIN, OUTPUT);
  digitalWrite(BUZ_PIN, LOW);
  pinMode(IR_PIN, INPUT);
  analogSetPinAttenuation(IR_PIN, ADC_11db);   // อ่านได้เต็มช่วง 0-3.3V
  pinMode(BTN_TEST, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);
  pinMode(LED_OK, OUTPUT);
  pinMode(LED_PILL, OUTPUT);
  pinMode(LIFT_IN1, OUTPUT);
  pinMode(LIFT_IN2, OUTPUT);
  pinMode(LIFT_EN, OUTPUT);
  digitalWrite(LIFT_IN1, LOW);
  digitalWrite(LIFT_IN2, LOW);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();

  rtcOk = rtc.begin();
  if (!rtcOk) {
    Serial.println("ไม่พบ RTC ใช้นาฬิกาในบอร์ดแทน รอเวลาจากกล้อง/เว็บ");
  } else if (!rtc.isrunning()) {           // DS1307 ที่ยังไม่เคยตั้งเวลา
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  } else {
    timeValid = clockNow().year() >= 2024;  // RTC มีถ่านและเคยตั้งเวลาแล้ว
  }

  servoTray.setPeriodHertz(50);
  servoPush.setPeriodHertz(50);
  servoTray.attach(SERVO_TRAY, 500, 2400);
  servoPush.attach(SERVO_PUSH, 500, 2400);
  servoTray.write(TRAY_ANGLE[0]);
  servoPush.write(PUSH_REST);

  loadSchedule();

  if (LIFT_HOME_ON_BOOT) {
    liftFloor = 7;          // ไม่รู้ตำแหน่ง: เลื่อนลงให้สุดก่อน
    moveLift(1);
  }
  sendLog("บอร์ดหลักพร้อมทำงาน");
}

void loop() {
  DateTime now = clockNow();

  pollComms();
  updateDose();
  updateLeds();

  if (currentState != ALARM_RINGING) {
    handleJoystick();
    handleButtons();
  }
  // ตรวจเวลาจ่ายยาทุกหน้าจอ ยกเว้นตอนกำลังแก้ตารางด้วยจอยสติก
  if (currentState != ALARM_RINGING && currentState != SET_ALARM) {
    checkAlarms(now);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  switch (currentState) {
    case HOME:
      drawHome(now);
      if (buttonPressed) currentState = MENU;
      break;

    case MENU:
      drawMenu();
      if (buttonPressed) {
        if (menuIndex == 0) currentState = HOME;
        else if (menuIndex == 1) { currentState = SET_ALARM; editField = 0; }
        else if (menuIndex == 2) { currentState = SET_TIME; tempTime = now; rtcEditField = 0; }
      }
      break;

    case SET_ALARM:
      drawSetAlarm();
      if (buttonPressed && editField == 5) {
        if (scheduleEdited) { saveSchedule(); scheduleEdited = false; sendLog("แก้ตารางเวลาจากจอยสติก"); }
        currentState = MENU;
      }
      break;

    case SET_TIME:
      drawSetTime();
      if (buttonPressed) currentState = MENU;
      break;

    case ALARM_RINGING:
      drawAlarmRinging();
      display.display();

      playAlarmSongOnce(); // เล่นเพลงมาริโอ้

      currentState = HOME;
      buttonPressed = false;
      delay(300);
      break;
  }

  if (currentState != ALARM_RINGING) {
    display.display();
  }

  if (millis() - lastStatusMs >= STATUS_MS) {
    lastStatusMs = millis();
    sendStatus(now);
  }

  buttonPressed = false;
  delay(50);
}

// ==========================================
// การสื่อสาร JSON (ทีละบรรทัด)
// ==========================================
void sendJson(JsonDocument &doc) {
  String s;
  serializeJson(doc, s);
  CamSerial.println(s);
  Serial.println(s);       // ดูได้ใน Serial Monitor / Wokwi
}

void sendLog(const String &msg) {
  JsonDocument d;
  d["type"] = "log";
  d["msg"] = msg;
  sendJson(d);
}

void sendState(const char *state) {
  JsonDocument d;
  d["floor"] = curFloor;
  d["slot"] = curSlot;
  d["state"] = state;
  sendJson(d);
}

String two(int v) { return v < 10 ? "0" + String(v) : String(v); }
String hms(const DateTime &t) { return two(t.hour()) + ":" + two(t.minute()) + ":" + two(t.second()); }
String isoDate(const DateTime &t) { return String(t.year()) + "-" + two(t.month()) + "-" + two(t.day()); }
String isoDateTime(const DateTime &t) { return isoDate(t) + "T" + hms(t); }

void sendStatus(const DateTime &now) {
  JsonDocument d;
  d["type"] = "status";
  d["rtc"] = hms(now);
  d["floor"] = curFloor;
  d["slot"] = curSlot;
  d["pill"] = pillPresent();
  d["ir"] = irRaw();         // ค่าดิบของเซนเซอร์ ใช้ตั้ง IR_THRESHOLD
  d["clock"] = rtcOk ? "rtc" : "board";
  d["timeok"] = timeValid;
  sendJson(d);
}

void requestPhoto(int f, int s, const char *phase) {
  JsonDocument d;
  d["type"] = "snap";
  d["floor"] = f;
  d["slot"] = s;
  d["phase"] = phase;
  sendJson(d);
}

void pollStream(Stream &st, String &buf) {
  while (st.available()) {
    char c = st.read();
    if (c == '\n') {
      buf.trim();
      if (buf.length()) handleLine(buf);
      buf = "";
    } else if (c != '\r') {
      if (buf.length() < 6000) buf += c;
    }
  }
}

void pollComms() {
  pollStream(CamSerial, rxCam);
  pollStream(Serial, rxUsb);
  if (netWifi && millis() - lastNetMs > 10000) {   // กล้องเงียบไป = ถือว่าหลุด
    netWifi = false;
    netClients = 0;
  }
}

void handleLine(String &line) {
  if (line[0] != '{') return;
  JsonDocument doc;
  if (deserializeJson(doc, line)) { Serial.println("JSON ผิดรูปแบบ"); return; }

  const char *type = doc["type"] | "";
  if (!strcmp(type, "net")) {                     // สถานะ Wi-Fi จากกล้อง
    netWifi = doc["wifi"] | false;
    netClients = doc["clients"] | 0;
    netIp = (const char *)(doc["ip"] | "");
    lastNetMs = millis();
    return;
  }

  const char *cmd = doc["cmd"] | "";
  int f = doc["floor"] | 0, s = doc["slot"] | 0;

  if (!strcmp(cmd, "time")) {
    DateTime t(doc["y"] | 2026, doc["mo"] | 1, doc["d"] | 1, doc["h"] | 0, doc["mi"] | 0, doc["s"] | 0);
    bool ntp = !strcmp(doc["src"] | "", "ntp");      // กล้องส่งมาเองทุก 10 นาที
    long diff = (long)t.unixtime() - (long)clockNow().unixtime();
    if (!ntp || !timeValid || diff > 1 || diff < -1) {   // NTP: ปรับเฉพาะเมื่อเวลาคลาดเกิน 1 วินาที
      clockSet(t);
      sendLog(String(ntp ? "ตั้งเวลาจากอินเทอร์เน็ต (NTP) " : "ตั้งเวลาเครื่องตามเว็บแล้ว ") + hms(clockNow()));
    }
  } else if (!strcmp(cmd, "schedule")) {
    applySchedule(doc["data"].as<JsonObject>());
  } else if (!strcmp(cmd, "goto")) {
    if (validPos(f, s)) moveTo(f, s);
  } else if (!strcmp(cmd, "dispense")) {
    if (validPos(f, s)) startDose(f, s, DOSE_MANUAL);
  } else if (!strcmp(cmd, "test")) {
    if (validPos(f, s)) startDose(f, s, DOSE_TEST);
    else sendLog("คำสั่ง test ต้องระบุชั้น 1-7 และช่อง 1-6");
  } else if (!strcmp(cmd, "cancel")) {
    if (dose.active && !dose.scheduled) {
      stopRequested = true;
      if (dose.mode == DOSE_TEST) { testStep("taken", false, "ยกเลิกการทดสอบ"); testStep("done", false); }
      sendLog("ยกเลิกแล้ว");
      endDose();
    }
  } else if (!strcmp(cmd, "capture")) {
    // ปกติกล้องจัดการเอง ถ้ามาถึงบอร์ดนี้ (เช่นทดสอบใน Wokwi) ให้ขอภาพผ่าน UART
    requestPhoto(f, s, doc["phase"] | "manual");
  } else if (!strcmp(cmd, "ir")) {
    sendLog("ค่าเซนเซอร์ตรวจยา = " + String(irRaw()) + (pillPresent() ? " (มียา)" : " (ไม่มียา)"));
  } else if (!strcmp(cmd, "stop")) {
    stopRequested = true;
  }
}

bool validPos(int f, int s) { return f >= 1 && f <= 7 && s >= 1 && s <= 6; }

// ==========================================
// ตารางเวลา
// ==========================================
void loadSchedule() {
  prefs.begin("pill", true);
  if (prefs.getBytesLength("floors") == sizeof(floors)) {
    prefs.getBytes("floors", floors, sizeof(floors));
  }
  prefs.end();
}

void saveSchedule() {
  prefs.begin("pill", false);
  prefs.putBytes("floors", floors, sizeof(floors));
  prefs.end();
}

FloorDay *findDay(const char *date) {
  for (int i = 0; i < 7; i++)
    if (floors[i].date[0] && !strcmp(floors[i].date, date)) return &floors[i];
  return nullptr;
}

void applySchedule(JsonObject data) {
  FloorDay nf[7];
  int days = 0, slots = 0;
  for (JsonObject fo : data["floors"].as<JsonArray>()) {
    int fi = (fo["floor"] | 0) - 1;
    if (fi < 0 || fi > 6) continue;
    strlcpy(nf[fi].date, fo["date"] | "", sizeof(nf[fi].date));
    FloorDay *old = findDay(nf[fi].date);
    days++;
    for (JsonObject so : fo["slots"].as<JsonArray>()) {
      int si = (so["slot"] | 0) - 1;
      if (si < 0 || si > 5) continue;
      AlarmTimer &a = nf[fi].slot[si];
      int h = 0, m = 0;
      const char *t = so["time"] | "";
      a.active = (so["on"] | false) && sscanf(t, "%d:%d", &h, &m) == 2;
      a.hour = h;
      a.minute = m;
      if (a.active) slots++;
      // เว็บส่งตารางซ้ำทุกครั้งที่เชื่อมต่อ: เก็บสถานะเดิมไว้ถ้าวัน/เวลาไม่เปลี่ยน จะได้ไม่จ่ายซ้ำ
      if (old) {
        AlarmTimer &o = old->slot[si];
        if (o.active == a.active && o.hour == a.hour && o.minute == a.minute) a.done = o.done;
      }
    }
  }
  memcpy(floors, nf, sizeof(floors));
  saveSchedule();
  sendLog("รับตารางเวลา " + String(days) + " วัน " + String(slots) + " มื้อแล้ว");
}

// หาว่ามีมื้อไหนถึงเวลาจ่าย
void checkAlarms(DateTime now) {
  if (!timeValid) return;   // ยังไม่ได้เวลาจริง (ไม่มี RTC และยังไม่ได้เวลาจากกล้อง) อย่าเพิ่งจ่ายยา
  String today = isoDate(now);
  int nowMin = now.hour() * 60 + now.minute();
  for (int f = 0; f < 7; f++) {
    if (today != floors[f].date) continue;
    for (int s = 0; s < 6; s++) {
      AlarmTimer &a = floors[f].slot[s];
      if (!a.active || a.done != DOSE_PENDING) continue;
      int due = a.hour * 60 + a.minute;
      if (nowMin >= due && nowMin <= due + GRACE_MIN) {
        startDose(f + 1, s + 1, DOSE_SCHEDULED);
        return;            // ทีละมื้อ
      }
      if (nowMin > due + GRACE_MIN) {   // เลยเวลาไปนาน (เครื่องปิดอยู่) ไม่จ่ายย้อนหลัง
        a.done = DOSE_MISSED;
        saveSchedule();
        sendLog("ข้ามมื้อ ชั้น " + String(f + 1) + " ช่อง " + String(s + 1) + " เพราะเลยเวลาแล้ว");
      }
    }
  }
}

// มื้อถัดไปสำหรับแสดงบนจอ / ปุ่มทดสอบ
bool nextDose(const DateTime &now, int &nf, int &ns) {
  String today = isoDate(now);
  int nowMin = now.hour() * 60 + now.minute();
  String best = "";
  for (int f = 0; f < 7; f++) {
    if (!floors[f].date[0] || today > floors[f].date) continue;
    for (int s = 0; s < 6; s++) {
      AlarmTimer &a = floors[f].slot[s];
      if (!a.active || a.done != DOSE_PENDING) continue;
      if (today == floors[f].date && a.hour * 60 + a.minute < nowMin) continue;
      String key = String(floors[f].date) + " " + two(a.hour) + ":" + two(a.minute);
      if (best == "" || key < best) { best = key; nf = f + 1; ns = s + 1; }
    }
  }
  return best != "";
}

// ==========================================
// กลไก: หมุนถาด / เลื่อนตัวดัน / ดันยา
// ==========================================
void moveLift(int f) {
  int diff = f - liftFloor;
  if (diff == 0) return;
  digitalWrite(LIFT_IN1, diff > 0 ? HIGH : LOW);
  digitalWrite(LIFT_IN2, diff < 0 ? HIGH : LOW);
  analogWrite(LIFT_EN, LIFT_PWM);
  delay((uint32_t)abs(diff) * LIFT_MS_PER_FLOOR);
  digitalWrite(LIFT_IN1, LOW);
  digitalWrite(LIFT_IN2, LOW);
  analogWrite(LIFT_EN, 0);
  liftFloor = f;
}

void moveTo(int f, int s) {
  curFloor = f;
  curSlot = s;
  sendState("moving");
  servoTray.write(TRAY_ANGLE[s - 1]);
  delay(700);
  moveLift(f);
  sendState("ready");
}

void pushPill() {
  sendState("dispensing");
  servoPush.write(PUSH_OUT);
  delay(600);
  servoPush.write(PUSH_REST);
  delay(600);
  sendState("ready");
}

// อ่านค่าเฉลี่ย 8 ครั้ง ลดสัญญาณรบกวน
int irRaw() {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(IR_PIN);
  return sum / 8;
}

// ==========================================
// นาฬิกา: ใช้ RTC ถ้ามี ไม่มีก็ใช้นาฬิกาในบอร์ด
// ==========================================
DateTime clockNow() {
  if (rtcOk) return rtc.now();
  return DateTime(swBaseUnix + (millis() - swBaseMs) / 1000);
}

void clockSet(const DateTime &t) {
  if (rtcOk) rtc.adjust(t);
  swBaseUnix = t.unixtime();
  swBaseMs = millis();
  timeValid = true;
}

bool pillPresent() {
  static bool state = false;
  int v = irRaw();
  bool low = v < IR_THRESHOLD - IR_HYST, high = v > IR_THRESHOLD + IR_HYST;
  if (IR_PILL_WHEN_LOW ? low : high) state = true;
  else if (IR_PILL_WHEN_LOW ? high : low) state = false;
  return state;      // อยู่ในช่วงกลาง: คงสถานะเดิม
}

// ==========================================
// ติดตามมื้อยา: จ่าย -> ถ่ายภาพ -> เพลง -> เตือนซ้ำ -> หยิบแล้ว -> ถ่ายภาพ
// ==========================================
// รายงานผลการทดสอบทีละขั้นให้เว็บ (เฉพาะโหมดทดสอบ)
void testStep(const char *step, bool ok, const String &msg) {
  if (dose.mode != DOSE_TEST) return;
  JsonDocument d;
  d["type"] = "test";
  d["floor"] = dose.f;
  d["slot"] = dose.s;
  d["step"] = step;
  d["ok"] = ok;
  if (msg.length()) d["msg"] = msg;
  sendJson(d);
}

// โหมดทดสอบ/สั่งเองจากเว็บ ใช้ชื่อภาพต่างจากมื้อจริง เพื่อไม่ให้ไปทับภาพในตารางยา
const char *photoPhase(const char *phase) {
  if (dose.scheduled) return phase;
  return !strcmp(phase, "taken") ? "test_taken" : "test_dispensed";
}

void endDose() {
  dose.active = false;
}

void startDose(int f, int s, uint8_t mode) {
  if (dose.active) {   // มื้อก่อนยังไม่จบ: ปิดมื้อเก่าก่อน
    if (dose.scheduled) markDone(dose.f, dose.s, DOSE_MISSED);
    if (dose.mode == DOSE_TEST) { testStep("taken", false, "ถูกแทรกด้วยคำสั่งใหม่"); testStep("done", false); }
    sendLog("ปิดงานชั้น " + String(dose.f) + " ช่อง " + String(dose.s) + " เพราะมีงานใหม่");
  }
  dose = Dose();
  dose.f = f;
  dose.s = s;
  dose.mode = mode;
  dose.scheduled = (mode == DOSE_SCHEDULED);
  testStep("start", true);

  moveTo(f, s);
  testStep("move", true);
  pushPill();
  testStep("push", true);
  if (dose.scheduled) markDone(f, s, DOSE_DISPENSED);

  dose.active = true;
  dose.startMs = millis();

  // รอให้ยาตกลงถาด
  while (millis() - dose.startMs < PILL_DETECT_MS) {
    if (pillPresent()) { dose.pillSeen = true; break; }
    delay(20);
  }
  if (dose.pillSeen) {
    requestPhoto(f, s, photoPhase("dispensed"));          // ภาพที่ 1: ตอนยาลงถาด
    sendLog("ยาลงถาดแล้ว ชั้น " + String(f) + " ช่อง " + String(s));
    testStep("pill", true, "ค่าเซนเซอร์ " + String(irRaw()));
  } else {
    sendLog("เซนเซอร์ไม่พบยาในถาด ชั้น " + String(f) + " ช่อง " + String(s) + " ตรวจช่องยา/เซนเซอร์");
    testStep("pill", false, "ไม่พบยาภายใน " + String(PILL_DETECT_MS / 1000) + " วินาที (ค่าเซนเซอร์ " + String(irRaw()) + ")");
  }

  if (dose.mode == DOSE_TEST) {
    testStep("ring", true);
    ring();                       // เพลงเล่นเมื่อ loop() วนกลับไป
    if (!dose.pillSeen) {         // ไม่มียาในถาด: ทดสอบขั้นหยิบยาต่อไม่ได้
      testStep("taken", false, "ข้าม เพราะไม่พบยาในถาด");
      testStep("done", false);
      endDose();
    }
  } else if (dose.scheduled) {
    ring();
  }
}

void ring() {
  dose.rings++;
  dose.lastRingMs = millis();
  currentState = ALARM_RINGING;
}

void markDone(int f, int s, uint8_t st) {
  floors[f - 1].slot[s - 1].done = st;
  saveSchedule();
}

void updateDose() {
  if (!dose.active) return;
  bool present = pillPresent();

  if (!dose.pillSeen) {
    if (present) {              // ยามาถึงช้า
      dose.pillSeen = true;
      requestPhoto(dose.f, dose.s, photoPhase("dispensed"));
    }
  } else if (!present) {
    if (!dose.clearSinceMs) dose.clearSinceMs = millis();
    if (millis() - dose.clearSinceMs >= TAKEN_DEBOUNCE) {
      if (dose.scheduled) {     // มื้อจริงเท่านั้นที่แจ้งเว็บให้บันทึกว่ารับยาแล้ว
        DateTime now = clockNow();
        JsonDocument d;
        d["type"] = "taken";
        d["floor"] = dose.f;
        d["slot"] = dose.s;
        d["at"] = isoDateTime(now);
        sendJson(d);
        markDone(dose.f, dose.s, DOSE_TAKEN);
      } else {
        sendLog("หยิบยาออกจากถาดแล้ว (ทดสอบ) ชั้น " + String(dose.f) + " ช่อง " + String(dose.s));
      }
      requestPhoto(dose.f, dose.s, photoPhase("taken"));   // ภาพที่ 2: หลังหยิบยา
      testStep("taken", true);
      testStep("done", true);
      endDose();
      return;
    }
  } else {
    dose.clearSinceMs = 0;
  }

  // เปิดเพลงซ้ำถ้ายังมียาค้าง (มื้อจริงเท่านั้น)
  if (dose.scheduled && dose.pillSeen && present && dose.rings < MAX_RINGS &&
      millis() - dose.lastRingMs >= REMIND_MS && currentState == HOME) {
    sendLog("ยังไม่ได้หยิบยา เตือนครั้งที่ " + String(dose.rings + 1));
    ring();
  }

  if (dose.mode == DOSE_TEST && millis() - dose.startMs >= TEST_TIMEOUT_MS) {
    testStep("taken", false, "ไม่ได้หยิบยาภายใน 2 นาที");
    testStep("done", false);
    endDose();
    return;
  }

  if (millis() - dose.startMs >= MISSED_MS) {
    if (dose.scheduled) markDone(dose.f, dose.s, DOSE_MISSED);
    sendLog("ไม่ได้รับยา ชั้น " + String(dose.f) + " ช่อง " + String(dose.s));
    endDose();
  }
}

void updateLeds() {
  digitalWrite(LED_OK, netWifi && netClients > 0);
  digitalWrite(LED_PILL, pillPresent());
}

// ==========================================
// เพลง: เล่น 1 รอบ หยุดได้ด้วยปุ่ม หรือเมื่อหยิบยาแล้ว
// ==========================================

bool shouldStopSong() {
  if (digitalRead(SW_PIN) == LOW || digitalRead(BTN_STOP) == LOW) {
    noTone(BUZ_PIN);
    while (digitalRead(SW_PIN) == LOW || digitalRead(BTN_STOP) == LOW) delay(10);  // รอปล่อยมือ
    return true;
  }
  if (dose.active && dose.pillSeen && !pillPresent()) return true;  // หยิบยาแล้ว
  if (stopRequested) { stopRequested = false; return true; }
  return false;
}

void playAlarmSongOnce() {
  stopRequested = false;
  for (int i = 0; i < numNotes; i++) {
    if (alarm_notes[i] > 0) tone(BUZ_PIN, alarm_notes[i]);
    else noTone(BUZ_PIN);

    unsigned long startT = millis();
    while (millis() - startT < (unsigned long)alarm_durations[i]) {
      if (shouldStopSong()) { noTone(BUZ_PIN); return; }
      delay(5);
    }
    noTone(BUZ_PIN);

    startT = millis();
    while (millis() - startT < 30) {
      if (shouldStopSong()) return;
      delay(5);
    }
  }
}

// ==========================================
// ปุ่มเพิ่มเติม
// ==========================================
void handleButtons() {
  static bool lastTest = HIGH;
  bool t = digitalRead(BTN_TEST);
  if (t == LOW && lastTest == HIGH) {
    int f, s;
    if (nextDose(clockNow(), f, s)) {
      sendLog("กดปุ่มทดสอบ: จ่ายมื้อถัดไปทันที");
      startDose(f, s, DOSE_SCHEDULED);
    } else {
      tone(BUZ_PIN, 400, 200);   // ไม่มีมื้อที่ตั้งไว้
    }
  }
  lastTest = t;
}

// --- ฟังก์ชันอ่านค่า Joystick (เดิม) ---
void handleJoystick() {
  static bool lastBtnState = HIGH;

  int x = analogRead(VRX_PIN);
  int y = analogRead(VRY_PIN);
  bool currentBtnState = digitalRead(SW_PIN);

  if (currentBtnState == LOW && lastBtnState == HIGH) {
    buttonPressed = true;
    tone(BUZ_PIN, 2000, 40);
    delay(50);
  }
  lastBtnState = currentBtnState;

  if (!joyMoved) {
    bool moved = false;
    if (y < 1000) { processDir(1); moved = true; }      // UP
    else if (y > 3000) { processDir(2); moved = true; } // DOWN
    else if (x < 1000) { processDir(3); moved = true; } // LEFT
    else if (x > 3000) { processDir(4); moved = true; } // RIGHT

    if (moved) {
      joyMoved = true;
      tone(BUZ_PIN, 3000, 15);
    }
  }

  if (x > 1800 && x < 2300 && y > 1800 && y < 2300) {
    joyMoved = false;
  }
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
      AlarmTimer &a = floors[setFloor].slot[setSlot];
      if (editField == 0) setFloor = (setFloor + step + 7) % 7;
      if (editField == 1) setSlot = (setSlot + step + 6) % 6;
      if (editField == 2) { a.hour = (a.hour + step + 24) % 24; a.done = DOSE_PENDING; scheduleEdited = true; }
      if (editField == 3) { a.minute = (a.minute + step + 60) % 60; a.done = DOSE_PENDING; scheduleEdited = true; }
      if (editField == 4) { a.active = !a.active; a.done = DOSE_PENDING; scheduleEdited = true; }
    }
  }
}

// ==========================================
// หน้าจอ OLED
// ==========================================
void drawHome(DateTime now) {
  // เวลา
  display.setTextSize(2);
  display.setCursor(15, 0);
  display.print(hms(now));

  // วันที่
  display.setTextSize(1);
  display.setCursor(10, 20);
  display.print(daysOfWeek[now.dayOfTheWeek()]);
  display.print(" ");
  display.print(two(now.day())); display.print('/');
  display.print(two(now.month())); display.print('/');
  display.print(now.year(), DEC);

  // มื้อถัดไป
  display.setCursor(0, 33);
  int f, s;
  if (nextDose(now, f, s)) {
    AlarmTimer &a = floors[f - 1].slot[s - 1];
    display.print("Next ");
    if (strcmp(floors[f - 1].date, isoDate(now).c_str()) != 0) {
      display.print(String(floors[f - 1].date).substring(8, 10));
      display.print("/");
      display.print(String(floors[f - 1].date).substring(5, 7));
      display.print(" ");
    }
    display.print(two(a.hour) + ":" + two(a.minute));
    display.print(" F"); display.print(f);
    display.print("S"); display.print(s);
  } else {
    display.print("Next: -");
  }

  // เครือข่าย
  display.setCursor(0, 45);
  if (netWifi) {
    display.print(netIp);
    display.print(netClients > 0 ? " WEB" : "");
  } else {
    display.print("WiFi: --");
  }

  // ยาค้างในถาด
  if (pillPresent()) {
    display.fillRect(0, 55, 128, 9, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(2, 56);
    display.print(dose.active ? "PILL READY - TAKE IT" : "PILL IN TRAY");
    display.setTextColor(SSD1306_WHITE);
  }
}

void drawMenu() {
  display.setTextSize(1);
  display.setCursor(0, 0); display.println("--- MAIN MENU ---");

  String m[3] = {"Back to Clock", "Set Alarms", "Set RTC Time"};
  for (int i = 0; i < 3; i++) {
    display.setCursor(10, 20 + (i * 10));
    if (menuIndex == i) display.print(">"); else display.print(" ");
    display.print(m[i]);
  }
}

void drawSetAlarm() {
  AlarmTimer &a = floors[setFloor].slot[setSlot];
  display.setTextSize(1);
  display.setCursor(0, 0); display.println("--- TIMER SETUP ---");

  display.setCursor(0, 15);
  if (editField == 0) display.print(">");
  display.print("F"); display.print(setFloor + 1);
  display.print(" ");
  display.print(floors[setFloor].date[0] ? String(floors[setFloor].date).substring(5) : String("--"));

  display.setCursor(80, 15);
  if (editField == 1) display.print(">");
  display.print("S:"); display.print(setSlot + 1);

  display.setCursor(0, 30);
  if (editField == 2) display.print(">");
  display.print("H:"); display.print(a.hour);

  display.setCursor(64, 30);
  if (editField == 3) display.print(">");
  display.print("M:"); display.print(a.minute);

  display.setCursor(0, 45);
  if (editField == 4) display.print(">");
  display.print("St:"); display.print(a.active ? "ON" : "OFF");

  display.setCursor(64, 45);
  if (editField == 5) display.print(">[EXIT]");
  else display.print(" [EXIT]");
}

void drawSetTime() {
  display.setTextSize(1);
  display.setCursor(0, 0); display.println("--- RTC SETUP ---");
  display.setCursor(0, 20); display.println("Time syncs from web");
  display.setCursor(0, 40); display.println("Press Btn to EXIT");
}

void drawAlarmRinging() {
  display.setTextSize(2);
  display.setCursor(10, 10);
  display.println(dose.mode == DOSE_TEST ? "TEST" : "TAKE PILL");
  display.setTextSize(1);
  display.setCursor(10, 35);
  display.print("Floor "); display.print(dose.f);
  display.print("  Slot "); display.print(dose.s);
  display.setCursor(10, 50);
  display.println("Press Btn to Stop");
}
