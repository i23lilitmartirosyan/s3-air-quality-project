#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "SparkFun_SCD30_Arduino_Library.h"

// ----------------- SGP40 -----------------
#include "Adafruit_SGP40.h"
#include "VOCGasIndexAlgorithm.h"

// ----------------- Wi-Fi -----------------
const char* ssid = "tumoTeam";
const char* password = "hello2team";

// ----------------- API -------------------
const char* apiEndpoint = "https://0kvgs3r39c.execute-api.us-east-1.amazonaws.com/default/fromEsptoTimeStream";
const char* deviceId    = "ESP32_01";

// ----------------- BME280 ----------------
Adafruit_BME280 bme;
#define I2C_SDA 21
#define I2C_SCL 22

// ----------------- SCD30 -----------------
SCD30 scd30;

// ----------------- SPS30 -----------------
#define SPS30_ADDR 0x69
#define CMD_START_MEASUREMENT  0x0010
#define CMD_READ_DATA_READY    0x0202
#define CMD_READ_MEASUREMENT   0x0300

// ------------ SGP40 ------------
Adafruit_SGP40 sgp40;
VOCGasIndexAlgorithm voc_algorithm;

// ==========================================
// TIMING VARIABLES
// ==========================================
unsigned long previousMillisVOC = 0;   
unsigned long previousMillisAWS = 0;   

// READ VOC EVERY **3 SECONDS**
const long intervalVOC = 3000;         

// Upload every 30 seconds
const long intervalAWS = 300000;

// Global VOC variables
int32_t global_voc_index = 0;          
uint16_t global_sgp40_raw = 0;
float global_temp = 25.0;
float global_hum = 50.0;

// ==========================================

// SPS30 CRC-8
uint8_t sps30_crc8(const uint8_t *buf, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
  }
  return crc;
}

void i2cWrite16(uint16_t cmd) {
  Wire.beginTransmission(SPS30_ADDR);
  Wire.write(cmd >> 8);
  Wire.write(cmd & 0xFF);
  Wire.endTransmission();
}

bool i2cReadBytes(uint8_t *buf, size_t len) {
  Wire.requestFrom(SPS30_ADDR, len);
  size_t idx = 0;
  while (Wire.available() && idx < len) {
    buf[idx++] = Wire.read();
  }
  return (idx == len);
}

void startMeasurement() {
  Wire.beginTransmission(SPS30_ADDR);
  Wire.write(CMD_START_MEASUREMENT >> 8);
  Wire.write(CMD_START_MEASUREMENT & 0xFF);

  Wire.write(0x03);
  Wire.write(0x00);

  uint8_t crc = sps30_crc8((uint8_t[]){0x03, 0x00}, 2);
  Wire.write(crc);

  Wire.endTransmission();
  delay(3000);
}

bool dataReady() {
  i2cWrite16(CMD_READ_DATA_READY);
  uint8_t buf[3];
  if (!i2cReadBytes(buf, 3)) return false;
  return buf[1] == 0x01;
}

float parseFloatFromData(const uint8_t *packet) {
  uint8_t raw[4] = {packet[0], packet[1], packet[3], packet[4]};
  uint32_t val =
    ((uint32_t)raw[0] << 24) |
    ((uint32_t)raw[1] << 16) |
    ((uint32_t)raw[2] << 8 ) |
    (uint32_t)raw[3];

  float f;
  memcpy(&f, &val, sizeof(f));
  return f;
}

void readSPS30(float *outVals, int count) {
  i2cWrite16(CMD_READ_MEASUREMENT);
  const int bytesPerFloat = 6;
  const int totalBytes = count * bytesPerFloat;

  uint8_t buf[60];
  if (!i2cReadBytes(buf, totalBytes)) return;

  for (int i = 0; i < count; i++) {
    const uint8_t *p = buf + i * bytesPerFloat;
    if (sps30_crc8(p, 2) != p[2] || sps30_crc8(p + 3, 2) != p[5]) {
      outVals[i] = NAN;
    } else {
      outVals[i] = parseFloatFromData(p);
    }
  }
}

// ----------------- Wi-Fi -----------------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");

  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    Serial.print(".");
    retry++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi connected!");
  } else {
    Serial.println("\nFailed to connect to Wi-Fi");
  }
}

// ----------------- SETUP -----------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!bme.begin(0x76) && !bme.begin(0x77)) {
    Serial.println("BME280 not found!");
  } else
{
    Serial.println("BME280 initialized");
  }

  if (!scd30.begin()) {
    Serial.println("SCD30 not found!");
  } else {
    scd30.setMeasurementInterval(2);
    scd30.setAutoSelfCalibration(true);
    Serial.println("SCD30 initialized");
  }

  Serial.println("Starting SPS30…");
  startMeasurement();

  if (!sgp40.begin()) {
    Serial.println("SGP40 not found!");
  } else {
    Serial.println("SGP40 initialized");
  }

  // ❗ FIXED: remove INVALID function call
  float sampling = voc_algorithm.get_sampling_interval();
  Serial.print("VOC Sampling Interval: ");
  Serial.println(sampling);

  connectWiFi();
}

// ----------------- LOOP -----------------
void loop() {
  unsigned long currentMillis = millis();

  // ============================================================
  // TASK 1: VOC Every 3 Seconds
  // ============================================================
  if (currentMillis - previousMillisVOC >= intervalVOC) {
    previousMillisVOC = currentMillis;

    global_temp = bme.readTemperature();
    global_hum  = bme.readHumidity();

    global_sgp40_raw = sgp40.measureRaw(global_temp, global_hum);
    global_voc_index = voc_algorithm.process(global_sgp40_raw);
  }

  // ============================================================
  // TASK 2: UPLOAD Every 30 Seconds
  // ============================================================
  if (currentMillis - previousMillisAWS >= intervalAWS) {
    previousMillisAWS = currentMillis;

    if (WiFi.status() != WL_CONNECTED) connectWiFi();

    float pressure = bme.readPressure() / 100.0F;
    float altitude = bme.readAltitude(1013.25);

    float co2ppm = 0;
    if (scd30.dataAvailable()) {
      co2ppm = scd30.getCO2();
    }

    float pm[10] = {0};
    if (dataReady()) {
      readSPS30(pm, 10);
    }

    // ----------------- JSON Payload -----------------
    String payload = "{";
    payload += "\"temperature\":" + String(global_temp, 2) + ",";
    payload += "\"pressure\":"    + String(pressure,    2) + ",";
    payload += "\"humidity\":"    + String(global_hum,  2) + ",";
    payload += "\"altitude\":"    + String(altitude,    2) + ",";
    payload += "\"co2ppm\":"      + String(co2ppm,      2) + ",";

    payload += "\"pm1_0\":"       + String(pm[0], 2) + ",";
    payload += "\"pm2_5\":"       + String(pm[1], 2) + ",";
    payload += "\"pm4_0\":"       + String(pm[2], 2) + ",";
    payload += "\"pm10\":"        + String(pm[3], 2) + ",";
    payload += "\"nc0_5\":"       + String(pm[4], 2) + ",";
    payload += "\"nc1_0\":"       + String(pm[5], 2) + ",";
    payload += "\"nc2_5\":"       + String(pm[6], 2) + ",";
    payload += "\"nc4_0\":"       + String(pm[7], 2) + ",";
    payload += "\"nc10\":"        + String(pm[8], 2) + ",";
    payload += "\"typical_size\":"+ String(pm[9], 2) + ",";

    payload += "\"sgp40_raw\":"   + String(global_sgp40_raw) + ",";
    payload += "\"tvoc_index\":"  + String(global_voc_index) + ",";

    payload += "\"deviceId\":\""  + String(deviceId) + "\"";
    payload += "}";

    Serial.println(payload);

    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(apiEndpoint);
      http.addHeader("Content-Type", "application/json");

      int httpCode = http.POST(payload.c_str());
      if (httpCode > 0) {
        Serial.print("HTTP Code: "); Serial.println(httpCode);
        Serial.println(http.getString());
      } else {
        Serial.print("HTTP POST failed: "); Serial.println(httpCode);
      }
      http.end();
    }
  }
}