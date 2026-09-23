#include <WiFi.h>
#include <Wire.h>
#include <RtcDS1302.h>
#include <LiquidCrystal_I2C.h>
#include "UbidotsEsp32Mqtt.h"

// --- KONFIGURASI WIFI & UBIDOTS ---
const char *WIFI_SSID     = "lida";
const char *WIFI_PASS     = "nanasmuda";
const char *UBIDOTS_TOKEN = "BBUS-8jFSRgyzzacIjpMOp4mgzRiMUNzZ9d";
const char *DEVICE_LABEL  = "solar-tracker";

// Label Variabel Ubidots
const char *VAR_MODE      = "mode-system"; 
const char *VAR_MOTOR_A   = "aktuator1";   
const char *VAR_MOTOR_B   = "aktuator2";   
const char *VAR_ARUS      = "arus";
const char *VAR_TEGANGAN  = "tegangan";
const char *VAR_DAYA      = "daya";

// --- KONFIGURASI PIN DRIVER ZK-5AD ---
// [0]=Motor A (IN1: 18, IN2: 19), [1]=Motor B (IN1: 25, IN2: 23)
const uint8_t MOTOR_PINS[2][2] = {{18, 19}, {25, 23}}; 

// --- KONFIGURASI PIN RTC DS1302 ---
#define DS1302_RST 4
#define DS1302_DAT 16
#define DS1302_CLK 17

ThreeWire myWire(DS1302_DAT, DS1302_CLK, DS1302_RST); 
RtcDS1302<ThreeWire> Rtc(myWire);

// --- KONFIGURASI LCD 16x2 I2C ---
LiquidCrystal_I2C lcd(0x27, 16, 2); 

// --- KONFIGURASI PIN SENSOR ANALOG ---
const int PIN_LDR_A    = 34;
const int PIN_LDR_B    = 35;
const int PIN_ARUS     = 32;
const int PIN_TEGANGAN = 33;

// --- PARAMETER SENSOR & LDR ---
#define CENTER_ADC 2048
#define TOLERANCE  300
#define R1 136800.0
#define R2 14570.0
const float SENSITIVITAS     = 0.066; // 66 mV/A (ACS712 30A)
const float TEGANGAN_PER_ADC = 3.3 / 4095.0;

// --- GLOBAL VARIABLES & OBJECTS ---
Ubidots ubidots(UBIDOTS_TOKEN);

int systemMode = 0; // 0 = Auto, 1 = Manual
int manualCmdA = 1; // 0 = Mundur, 1 = Stop, 2 = Maju
int manualCmdB = 1; 
float vOffset  = 1.65; // Offset default sebelum kalibrasi

unsigned long lastLcdMillis = 0;
unsigned long lastPublishMillis = 0;
const unsigned long PUBLISH_INTERVAL = 300000; // Kirim data tiap 5 Menit

// Fungsi Generik Kontrol Motor
void moveMotor(uint8_t index, int state) {
  digitalWrite(MOTOR_PINS[index][0], (state == 2) ? HIGH : LOW);
  digitalWrite(MOTOR_PINS[index][1], (state == 0) ? HIGH : LOW);
}

// Logika Mode Otomatis (Sensor LDR & Jam Operasional RTC)
void runAutoMode() {
  RtcDateTime now = Rtc.GetDateTime();
  
  if (now.Hour() >= 7 && now.Hour() < 17) {
    int adcA = analogRead(PIN_LDR_A);
    int adcB = analogRead(PIN_LDR_B);

    // Motor A
    if (adcA > (CENTER_ADC + TOLERANCE))      moveMotor(0, 2);
    else if (adcA < (CENTER_ADC - TOLERANCE)) moveMotor(0, 0);
    else                                      moveMotor(0, 1);

    // Motor B
    if (adcB > (CENTER_ADC + TOLERANCE))      moveMotor(1, 2);
    else if (adcB < (CENTER_ADC - TOLERANCE)) moveMotor(1, 0);
    else                                      moveMotor(1, 1);
  } else {
    moveMotor(0, 1);
    moveMotor(1, 1);
  }
}

// Callback Data MQTT Ubidots
void callback(char *topic, byte *payload, unsigned int length) {
  int val = atoi((char*)payload);
  if (strstr(topic, VAR_MODE))        systemMode = val;
  else if (strstr(topic, VAR_MOTOR_A)) manualCmdA = val;
  else if (strstr(topic, VAR_MOTOR_B)) manualCmdB = val;
}

// Pembacaan Arus DC
float bacaArus() {
  long total = 0;
  int sampel = 300;
  
  for (int i = 0; i < sampel; i++) {
    total += analogRead(PIN_ARUS);
    delayMicroseconds(500); 
  }
  
  float adcRata = total / (float)sampel;
  float teganganSensor = adcRata * TEGANGAN_PER_ADC;
  float arus = abs(teganganSensor - vOffset) / SENSITIVITAS;
  
  return (arus < 0.35) ? 0.0 : arus; // Threshold deadband
}

// Pembacaan Tegangan DC
float bacaTegangan() {
  long total = 0;
  for (int i = 0; i < 20; i++) {
    total += analogRead(PIN_TEGANGAN);
    delayMicroseconds(200);
  }
  float vOut = (total / 20.0) * TEGANGAN_PER_ADC;
  return vOut * ((R1 + R2) / R2);
}

// Update Tampilan LCD 16x2
void updateLCD(RtcDateTime now, float tegangan, float arus, float daya) {
  char baris0[17];
  char baris1[17];

  snprintf(baris0, sizeof(baris0), "%02d:%02d:%02d   %s", 
           now.Hour(), now.Minute(), now.Second(),
           (systemMode == 0) ? "AUTO" : "MANU");

  snprintf(baris1, sizeof(baris1), "%.1fV %.1fA %.1fW", tegangan, arus, daya);

  lcd.setCursor(0, 0);
  lcd.print(baris0);
  lcd.setCursor(0, 1);
  lcd.print(baris1);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // Inisialisasi Pin Motor
  for (int i = 0; i < 2; i++) {
    pinMode(MOTOR_PINS[i][0], OUTPUT);
    pinMode(MOTOR_PINS[i][1], OUTPUT);
    moveMotor(i, 1); // Stop motor saat awal
  }

  // Inisialisasi LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("  Solar Tracker ");
  lcd.setCursor(0, 1);
  lcd.print(" Connecting...  ");

  // Inisialisasi RTC DS1302
  Rtc.Begin();
  if (Rtc.GetIsWriteProtected()) Rtc.SetIsWriteProtected(false);
  if (!Rtc.GetIsRunning()) Rtc.SetIsRunning(true);

  // Koneksi Ubidots & WiFi
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();

  // Kalibrasi Sensor Arus (Setiap booting saat WiFi telah aktif)
  delay(1500); 
  long totalAdc = 0;
  for (int i = 0; i < 1000; i++) {
    totalAdc += analogRead(PIN_ARUS);
    delay(1);
  }
  vOffset = (totalAdc / 1000.0) * TEGANGAN_PER_ADC;
  
  Serial.print("vOffset Terkalibrasi: ");
  Serial.print(vOffset, 3);
  Serial.println(" V");

  // Subscribe Topik Ubidots
  ubidots.subscribeLastValue(DEVICE_LABEL, VAR_MODE);
  ubidots.subscribeLastValue(DEVICE_LABEL, VAR_MOTOR_A);
  ubidots.subscribeLastValue(DEVICE_LABEL, VAR_MOTOR_B);

  lcd.clear();
}

void loop() {
  // Reconnect otomatis jika koneksi MQTT terputus
  if (!ubidots.connected()) {
    ubidots.reconnect();
    ubidots.subscribeLastValue(DEVICE_LABEL, VAR_MODE);
    ubidots.subscribeLastValue(DEVICE_LABEL, VAR_MOTOR_A);
    ubidots.subscribeLastValue(DEVICE_LABEL, VAR_MOTOR_B);
  }
  ubidots.loop();

  // Eksekusi Mode (Auto / Manual)
  if (systemMode == 0) {
    runAutoMode();
  } else {
    moveMotor(0, manualCmdA);
    moveMotor(1, manualCmdB);
  }

  // Pembacaan Nilai Sensor
  RtcDateTime now = Rtc.GetDateTime();
  float arus = bacaArus();
  float tegangan = bacaTegangan();
  float daya = tegangan * arus;

  // Update LCD & Serial Monitor Setiap 1 Detik
  if (millis() - lastLcdMillis >= 1000) {
    lastLcdMillis = millis();

    updateLCD(now, tegangan, arus, daya);

    Serial.print("Waktu: "); Serial.print(now.Hour()); Serial.print(":"); Serial.print(now.Minute());
    Serial.print(" | V: "); Serial.print(tegangan, 2);
    Serial.print("V | I: "); Serial.print(arus, 2);
    Serial.print("A | P: "); Serial.print(daya, 2);
    Serial.print("W | Mode: "); Serial.println(systemMode == 0 ? "Auto" : "Manual");
  }

  // Pengiriman Data ke Ubidots Setiap 5 Menit
  if (millis() - lastPublishMillis >= PUBLISH_INTERVAL) {
    lastPublishMillis = millis();

    ubidots.add(VAR_ARUS, arus);
    ubidots.add(VAR_TEGANGAN, tegangan);
    ubidots.add(VAR_DAYA, daya);
    ubidots.publish(DEVICE_LABEL);

    Serial.println(">>> [UBIDOTS] Data berhasil dikirim ke cloud (5 Menit) <<<");
  }
}