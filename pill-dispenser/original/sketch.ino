#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

RTC_DS1307 rtc;

// --- ขา Pin สำหรับ ESP32 ---
#define VRX_PIN 34
#define VRY_PIN 35
#define SW_PIN  25
#define BUZ_PIN 26

// --- System States ---
enum State { HOME, MENU, SET_TIME, SET_ALARM, ALARM_RINGING };
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
AlarmTimer alarms[7][6]; // 7 วัน, 6 เวลาต่อวัน

int setDay = 0;
int setSlot = 0;
int editField = 0; 

int rtcEditField = 0;
DateTime tempTime;

String daysOfWeek[7] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

// ==========================================
// เพลง Super Mario Bros. Theme (ความยาว ~15-20 วิ)
// ==========================================
// ความถี่โน้ต (Hz)
const int alarm_notes[] = {
  659, 659, 0, 659, 0, 523, 659, 0, 784, 0, 0, 0, 392, 0, 0, 0,
  523, 0, 0, 392, 0, 0, 330, 0, 0, 440, 0, 494, 0, 466, 440, 0,
  392, 659, 784, 880, 0, 698, 784, 0, 659, 0, 523, 587, 494, 0, 0,
  523, 0, 0, 392, 0, 0, 330, 0, 0, 440, 0, 494, 0, 466, 440, 0,
  392, 659, 784, 880, 0, 698, 784, 0, 659, 0, 523, 587, 494, 0, 0,
  784, 740, 698, 622, 0, 659, 0, 415, 440, 523, 0, 440, 523, 587, 0,
  784, 740, 698, 622, 0, 659, 0, 1047, 0, 1047, 1047, 0, 0, 0
};
// ความยาวของโน้ตแต่ละตัว (จังหวะ x 150ms)
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

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(BUZ_PIN, OUTPUT);
  digitalWrite(BUZ_PIN, LOW);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    for(;;);
  }
  
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void loop() {
  DateTime now = rtc.now();
  
  if (currentState != ALARM_RINGING) {
    handleJoystick();
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

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
        else if (menuIndex == 1) { currentState = SET_ALARM; editField = 0; }
        else if (menuIndex == 2) { currentState = SET_TIME; tempTime = now; rtcEditField = 0; }
      }
      break;

    case SET_ALARM:
      drawSetAlarm();
      if (buttonPressed && editField == 5) {
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
  
  buttonPressed = false; 
  delay(50);
}

// --- ฟังก์ชันเล่นเพลง 1 รอบเต็มแบบไม่บล็อกปุ่ม ---
void playAlarmSongOnce() {
  for (int i = 0; i < numNotes; i++) {
    if (alarm_notes[i] > 0) {
      tone(BUZ_PIN, alarm_notes[i]);
    } else {
      noTone(BUZ_PIN); // เล่นเป็นเสียงเงียบ (จังหวะพัก)
    }
    
    // เล่นเสียงโน้ตตามระยะเวลาที่กำหนด พร้อมดักจับปุ่มกด
    unsigned long startT = millis();
    while (millis() - startT < alarm_durations[i]) {
      if (digitalRead(SW_PIN) == LOW) { // หากมีการกดปุ่ม
        noTone(BUZ_PIN);
        while (digitalRead(SW_PIN) == LOW) { delay(10); } // รอจนกว่าจะปล่อยมือ
        return; 
      }
      delay(5);
    }
    
    noTone(BUZ_PIN); 
    
    // ระยะห่างระหว่างโน้ตแต่ละตัว (30ms) 
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

// --- ฟังก์ชันอ่านค่า Joystick ---
void handleJoystick() {
  static bool lastBtnState = HIGH;
  
  int x = analogRead(VRX_PIN);
  int y = analogRead(VRY_PIN);
  bool currentBtnState = digitalRead(SW_PIN);

  if (currentBtnState == LOW && lastBtnState == HIGH) {
    buttonPressed = true;
    tone(BUZ_PIN, 2000, 40); // เสียงติ๊ดสั้นๆ ตอนกดปุ่ม
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
      tone(BUZ_PIN, 3000, 15); // เสียงติ๊กตอนเลื่อนเมนู
    }
  }
  
  if (x > 1800 && x < 2300 && y > 1800 && y < 2300) {
    joyMoved = false;
  }
}

// --- การจัดการทิศทางเมนู ---
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
}

// --- ตรวจสอบเวลา Alarm ---
void checkAlarms(DateTime now) {
  int d = now.dayOfTheWeek();
  for (int i = 0; i < 6; i++) {
    if (alarms[d][i].active && alarms[d][i].hour == now.hour() && alarms[d][i].minute == now.minute() && now.second() == 0) {
      currentState = ALARM_RINGING;
    }
  }
}

// --- ฟังก์ชันวาดหน้าจอ ---
void drawHome(DateTime now) {
  display.setTextSize(2);
  display.setCursor(15, 10);
  
  if(now.hour() < 10) display.print('0');
  display.print(now.hour(), DEC);
  display.print(':'); 
  
  if(now.minute() < 10) display.print('0');
  display.print(now.minute(), DEC);
  display.print(':');
  
  if(now.second() < 10) display.print('0');
  display.print(now.second(), DEC);
  
  display.setTextSize(1);
  display.setCursor(10, 40);
  display.print(daysOfWeek[now.dayOfTheWeek()]);
  display.print(" ");
  
  if(now.day() < 10) display.print('0');
  display.print(now.day(), DEC); display.print('/');
  
  if(now.month() < 10) display.print('0');
  display.print(now.month(), DEC); display.print('/');
  
  display.print(now.year(), DEC);
}

void drawMenu() {
  display.setTextSize(1);
  display.setCursor(0, 0); display.println("--- MAIN MENU ---");
  
  String m[3] = {"Back to Clock", "Set Alarms", "Set RTC Time"};
  for (int i=0; i<3; i++) {
    display.setCursor(10, 20 + (i*10));
    if (menuIndex == i) display.print(">"); else display.print(" ");
    display.print(m[i]);
  }
}

void drawSetAlarm() {
  display.setTextSize(1);
  display.setCursor(0, 0); display.println("--- TIMER SETUP ---");
  
  display.setCursor(0, 15);
  if(editField == 0) display.print(">");
  display.print("Day:"); display.print(daysOfWeek[setDay]);
  
  display.setCursor(64, 15);
  if(editField == 1) display.print(">");
  display.print("S:"); display.print(setSlot + 1);

  display.setCursor(0, 30);
  if(editField == 2) display.print(">");
  display.print("H:"); display.print(alarms[setDay][setSlot].hour);
  
  display.setCursor(64, 30);
  if(editField == 3) display.print(">");
  display.print("M:"); display.print(alarms[setDay][setSlot].minute);

  display.setCursor(0, 45);
  if(editField == 4) display.print(">");
  display.print("St:"); display.print(alarms[setDay][setSlot].active ? "ON" : "OFF");
  
  display.setCursor(64, 45);
  if(editField == 5) display.print(">[EXIT]");
  else display.print(" [EXIT]");
}

void drawSetTime() {
  display.setTextSize(1);
  display.setCursor(0, 0); display.println("--- RTC SETUP ---");
  display.setCursor(0, 20); display.println("Feature in progress");
  display.setCursor(0, 40); display.println("Press Btn to EXIT");
}

void drawAlarmRinging() {
  display.setTextSize(2);
  display.setCursor(10, 20);
  display.println("WAKE UP!");
  display.setTextSize(1);
  display.setCursor(10, 45);
  display.println("Press Btn to Stop");
}