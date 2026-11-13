/*
  เครื่องจ่ายยา — ESP32-S3 CAM (สะพานระหว่างเว็บแอปกับบอร์ดหลัก + กล้อง)

  - ต่อ Wi-Fi แล้วเปิด WebSocket ที่พอร์ต 81  (เว็บแอปเชื่อมที่ ws://<IP>:81 หรือ ws://pill.local:81)
  - คำสั่งจากเว็บ -> ส่งต่อให้บอร์ดหลักทาง UART (JSON ทีละบรรทัด)
  - ข้อความจากบอร์ดหลัก -> ส่งต่อให้เว็บทุกหน้าที่เชื่อมอยู่
  - {"type":"snap",...} จากบอร์ดหลัก และ {"cmd":"capture","floor":3,"slot":4,"phase":"test"} จากเว็บ -> ถ่ายภาพแล้วส่งเป็น
      {"type":"photo","floor":3,"slot":4,"phase":"dispensed","at":12345,"img":"data:image/jpeg;base64,..."}
      ถ้าถ่ายไม่สำเร็จ ส่ง {"type":"photo_error","floor":3,"slot":4,"phase":"test","msg":"..."}
  - ส่งสถานะเครือข่ายให้บอร์ดหลักทุก 3 วินาที (แสดง IP บนจอ OLED)
  - ดึงเวลาประเทศไทยจากอินเทอร์เน็ต (NTP) ส่งให้บอร์ดหลักตอนเปิดเครื่องและทุก 10 นาที
      {"cmd":"time","y":2026,"mo":11,"d":13,"h":8,"mi":0,"s":0,"src":"ntp"}

  บอร์ดที่ใช้: ESP32-S3 N16R8 CAM + OV2640 (แฟลช 16MB, PSRAM 8MB) ผังขาเหมือน Freenove ESP32-S3 CAM
  Arduino IDE > Tools
    Board: "ESP32S3 Dev Module"
    Flash Size: "16MB (128Mb)"
    PSRAM: "OPI PSRAM"
    Partition Scheme: "16M Flash (3MB APP/9.9MB FATFS)"
    USB CDC On Boot: "Disabled"  แล้วเสียบสาย USB ช่องที่เขียนว่า COM (หรือ UART) เพื่ออัปโหลดและดู Serial Monitor
      (ถ้าเสียบช่อง USB/OTG ให้ตั้ง USB CDC On Boot เป็น "Enabled" แทน)
  ไลบรารี: WebSockets (Markus Sattler), ArduinoJson (Benoit Blanchon) v7
*/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "esp_camera.h"
#include "mbedtls/base64.h"
#include <time.h>

// ==========================================
// ตั้งค่า Wi-Fi
// ==========================================
const char *WIFI_SSID = "YOUR_WIFI";
const char *WIFI_PASS = "YOUR_PASSWORD";
const char *HOSTNAME  = "pill";            // ws://pill.local:81
// ถ้าต่อ Wi-Fi ไม่ได้ จะเปิดฮอตสปอตของตัวเองแทน: ชื่อ PillDispenser รหัส 12345678 -> ws://192.168.4.1:81
const char *AP_SSID = "PillDispenser";
const char *AP_PASS = "12345678";

// ==========================================
// เวลาจากอินเทอร์เน็ต (NTP) ส่งให้บอร์ดหลัก
// ==========================================
const char *TZ_INFO = "ICT-7";            // เวลาประเทศไทย UTC+7
const char *NTP1 = "pool.ntp.org";
const char *NTP2 = "time.google.com";
const uint32_t TIME_SYNC_MS = 10UL * 60 * 1000;   // ส่งเวลาให้บอร์ดหลักทุก 10 นาที
bool ntpStarted = false;
uint32_t lastTimeSentMs = 0, lastTimeTryMs = 0;
bool timeSentOnce = false;

// ==========================================
// UART ไปบอร์ดหลัก (ไขว้ TX<->RX และต่อ GND ร่วม)
// ==========================================
#define BOARD_RX_PIN 14   // ต่อกับ GPIO17 (TX2) ของบอร์ดหลัก
#define BOARD_TX_PIN 21   // ต่อกับ GPIO16 (RX2) ของบอร์ดหลัก
HardwareSerial Board(1);

// ==========================================
// ขากล้อง: ESP32-S3 N16R8 CAM (OV2640) / Freenove ESP32-S3-WROOM CAM / ESP32-S3-EYE
// ตรงกับ CAMERA_MODEL_ESP32S3_EYE ใน camera_pins.h ของตัวอย่าง CameraWebServer
// ห้ามใช้ GPIO 35-37 (PSRAM แบบ OPI) และ 43-44 (USB-UART) ต่ออย่างอื่น
// ==========================================
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM    8
#define Y3_GPIO_NUM    9
#define Y2_GPIO_NUM    11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

WebSocketsServer ws(81);
bool camOk = false;
String rxBoard;
uint32_t lastNetMs = 0;

void sendLog(const String &msg, int client = -1);
void sendPhotoError(int floor, int slot, const char *phase, const String &msg);
void handleBoardLine(String &line);

// ==========================================
// กล้อง
// ==========================================
bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_VGA;     // 640x480 ภาพละ ~20-40 KB
  c.jpeg_quality = 14;              // 10-63 ตัวเลขมาก = ไฟล์เล็ก
  c.fb_count = 1;
  c.grab_mode = CAMERA_GRAB_LATEST;
  if (psramFound()) {
    c.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    c.fb_location = CAMERA_FB_IN_DRAM;
    c.frame_size = FRAMESIZE_QVGA;  // ไม่มี PSRAM ใช้ภาพเล็ก
  }
  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed 0x%x\n", err);
    return false;
  }
  return true;
}

// ถ่ายภาพแล้วส่งให้เว็บทุกหน้า
void snapAndSend(int floor, int slot, const char *phase) {
  if (!camOk) { sendPhotoError(floor, slot, phase, "กล้องไม่พร้อม (เริ่มต้นกล้องไม่สำเร็จ ตรวจสายแพกล้อง)"); return; }

  // ทิ้งเฟรมเก่าที่ค้างในบัฟเฟอร์ เพื่อให้ได้ภาพ ณ ตอนนี้จริง ๆ
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) esp_camera_fb_return(fb);
  fb = esp_camera_fb_get();
  if (!fb) { sendPhotoError(floor, slot, phase, "ถ่ายภาพไม่สำเร็จ"); return; }

  size_t b64Len = 0;
  mbedtls_base64_encode(nullptr, 0, &b64Len, fb->buf, fb->len);   // หาขนาด
  String head = "{\"type\":\"photo\",\"floor\":" + String(floor) + ",\"slot\":" + String(slot) +
                ",\"phase\":\"" + String(phase) + "\",\"at\":" + String(millis()) +
                ",\"img\":\"data:image/jpeg;base64,";
  const char *tail = "\"}";
  size_t total = head.length() + b64Len + strlen(tail) + 1;

  char *msg = (char *)(psramFound() ? ps_malloc(total) : malloc(total));
  if (!msg) { esp_camera_fb_return(fb); sendPhotoError(floor, slot, phase, "หน่วยความจำไม่พอสำหรับภาพ"); return; }

  memcpy(msg, head.c_str(), head.length());
  size_t written = 0;
  mbedtls_base64_encode((unsigned char *)msg + head.length(), b64Len, &written, fb->buf, fb->len);
  size_t pos = head.length() + written;
  memcpy(msg + pos, tail, strlen(tail));
  pos += strlen(tail);
  msg[pos] = 0;
  size_t jpgLen = fb->len;
  esp_camera_fb_return(fb);

  ws.broadcastTXT((uint8_t *)msg, pos);
  free(msg);
  Serial.printf("ส่งภาพ ชั้น %d ช่อง %d (%s) %u bytes\n", floor, slot, phase, (unsigned)jpgLen);
}

// ==========================================
// ส่งข้อความ
// ==========================================
void sendLog(const String &msg, int client) {
  JsonDocument d;
  d["type"] = "log";
  d["msg"] = "กล้อง: " + msg;
  String s;
  serializeJson(d, s);
  if (client >= 0) ws.sendTXT(client, s);
  else ws.broadcastTXT(s);
  Serial.println(s);
}

void sendPhotoError(int floor, int slot, const char *phase, const String &msg) {
  JsonDocument d;
  d["type"] = "photo_error";
  d["floor"] = floor;
  d["slot"] = slot;
  d["phase"] = phase;
  d["msg"] = msg;
  String s;
  serializeJson(d, s);
  ws.broadcastTXT(s);
  Serial.println(s);
}

// ส่งเวลาปัจจุบันให้บอร์ดหลัก (รูปแบบเดียวกับคำสั่ง time ของเว็บ)
bool sendTimeToBoard() {
  struct tm t;
  if (!getLocalTime(&t, 0) || t.tm_year + 1900 < 2024) return false;   // ยังไม่ได้เวลาจาก NTP
  JsonDocument d;
  d["cmd"] = "time";
  d["y"] = t.tm_year + 1900;
  d["mo"] = t.tm_mon + 1;
  d["d"] = t.tm_mday;
  d["h"] = t.tm_hour;
  d["mi"] = t.tm_min;
  d["s"] = t.tm_sec;
  d["src"] = "ntp";
  String s;
  serializeJson(d, s);
  Board.println(s);
  return true;
}

void sendNetToBoard() {
  JsonDocument d;
  d["type"] = "net";
  bool sta = WiFi.status() == WL_CONNECTED;
  d["wifi"] = sta || (WiFi.getMode() & WIFI_AP);
  d["ip"] = sta ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  d["clients"] = ws.connectedClients();
  String s;
  serializeJson(d, s);
  Board.println(s);
}

// ==========================================
// WebSocket จากเว็บแอป
// ==========================================
void onWs(uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[%u] เว็บเชื่อมต่อ\n", num);
      sendLog("เชื่อมต่อเครื่องจ่ายยาแล้ว", num);
      sendNetToBoard();
      break;

    case WStype_DISCONNECTED:
      Serial.printf("[%u] เว็บตัดการเชื่อมต่อ\n", num);
      sendNetToBoard();
      break;

    case WStype_TEXT: {
      String line((const char *)payload, len);
      line.replace("\n", " ");   // UART ใช้ขึ้นบรรทัดใหม่เป็นตัวแบ่งข้อความ
      JsonDocument doc;
      if (deserializeJson(doc, line)) { sendLog("ได้ข้อความที่ไม่ใช่ JSON", num); break; }
      const char *cmd = doc["cmd"] | "";
      if (!strcmp(cmd, "capture")) {             // กล้องจัดการเอง
        snapAndSend(doc["floor"] | 0, doc["slot"] | 0, doc["phase"] | "manual");
      } else {
        Board.println(line);                     // ส่งต่อให้บอร์ดหลัก
      }
      break;
    }
    default:
      break;
  }
}

// ==========================================
// UART จากบอร์ดหลัก
// ==========================================
void handleBoardLine(String &line) {
  if (line[0] != '{') return;
  JsonDocument doc;
  if (deserializeJson(doc, line)) return;
  const char *type = doc["type"] | "";
  if (!strcmp(type, "snap")) {
    snapAndSend(doc["floor"] | 0, doc["slot"] | 0, doc["phase"] | "dispensed");
    return;
  }
  ws.broadcastTXT(line);   // status / taken / log / moving / ready / dispensing
}

void pollBoard() {
  while (Board.available()) {
    char c = Board.read();
    if (c == '\n') {
      rxBoard.trim();
      if (rxBoard.length()) handleBoardLine(rxBoard);
      rxBoard = "";
    } else if (c != '\r' && rxBoard.length() < 4000) {
      rxBoard += c;
    }
  }
}

// ==========================================
// Wi-Fi
// ==========================================
void startWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("กำลังต่อ Wi-Fi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setSleep(false);
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    configTzTime(TZ_INFO, NTP1, NTP2);      // เริ่มดึงเวลาจากอินเทอร์เน็ต
    ntpStarted = true;
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("ต่อ Wi-Fi ไม่ได้ เปิดฮอตสปอต ");
    Serial.print(AP_SSID);
    Serial.print(" IP: ");
    Serial.println(WiFi.softAPIP());
  }
  if (MDNS.begin(HOSTNAME)) MDNS.addService("ws", "tcp", 81);
}

void setup() {
  Serial.begin(115200);
  Board.setRxBufferSize(4096);
  Board.begin(115200, SERIAL_8N1, BOARD_RX_PIN, BOARD_TX_PIN);

  camOk = initCamera();
  startWifi();

  ws.begin();
  ws.onEvent(onWs);
  ws.enableHeartbeat(15000, 3000, 2);   // ตัดหน้าเว็บที่ปิดไปแล้ว
  Serial.println("WebSocket พร้อมที่พอร์ต 81");
}

void loop() {
  ws.loop();
  pollBoard();

  if (millis() - lastNetMs > 3000) {
    lastNetMs = millis();
    sendNetToBoard();
    // หลุด Wi-Fi บ้าน: ลองต่อใหม่
    if (WiFi.getMode() == WIFI_STA && WiFi.status() != WL_CONNECTED) WiFi.reconnect();
  }

  // ส่งเวลาให้บอร์ดหลัก: ครั้งแรกทันทีที่ได้เวลา (ลองทุก 2 วินาที) จากนั้นทุก 10 นาที
  if (ntpStarted) {
    uint32_t wait = timeSentOnce ? TIME_SYNC_MS : 2000;
    uint32_t since = timeSentOnce ? millis() - lastTimeSentMs : millis() - lastTimeTryMs;
    if (since >= wait) {
      lastTimeTryMs = millis();
      if (sendTimeToBoard()) {
        if (!timeSentOnce) Serial.println("ได้เวลาจากอินเทอร์เน็ตแล้ว ส่งให้บอร์ดหลัก");
        timeSentOnce = true;
        lastTimeSentMs = millis();
      }
    }
  }
}
