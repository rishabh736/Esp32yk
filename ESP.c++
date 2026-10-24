#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <PZEM004Tv30.h>
#include <time.h>
#include <math.h>

const char* WIFI_SSID = "Yantra Kavach";
const char* WIFI_PASSWORD = "yantrakavach@123";
const char* API_URL = "https://yantrakavach/api/machine-data";
const char* API_TOKEN = "";
const char* MACHINE_ID = "YK-MOTOR-001";

#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17
#define TEMP_PIN 4
#define SOUND_PIN 34
#define HALL_PIN 27

const float TEMP_WARNING_C = 60.0;
const float TEMP_CRITICAL_C = 80.0;
const float VIB_WARNING_MS2 = 2.0;
const float VIB_CRITICAL_MS2 = 4.0;
const float CURRENT_WARNING_A = 5.0;
const float CURRENT_CRITICAL_A = 7.0;
const float SOUND_WARNING_RMS = 300.0;
const float SOUND_CRITICAL_RMS = 600.0;

const unsigned long UPLOAD_INTERVAL_MS = 10000;
const unsigned long RPM_WINDOW_MS = 2000;
const int PULSES_PER_REVOLUTION = 1;

volatile unsigned long hallPulses = 0;

unsigned long lastUpload = 0;
unsigned long lastRpmCalculation = 0;
float rpm = NAN;

bool tempAvailable = false;
bool accelAvailable = false;

HardwareSerial pzemSerial(2);
PZEM004Tv30 pzem(pzemSerial, PZEM_RX_PIN, PZEM_TX_PIN);

OneWire oneWire(TEMP_PIN);
DallasTemperature tempSensor(&oneWire);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

struct MachineData {
  float temperature = NAN;
  float vibration = NAN;
  float current = NAN;
  float voltage = NAN;
  float power = NAN;
  float sound = NAN;
};

MachineData readings;

void IRAM_ATTR hallISR() {
  hallPulses++;
}

String getTimestamp() {
  struct tm timeinfo;

  if (!getLocalTime(&timeinfo, 100)) {
    return "";
  }

  char buffer[25];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buffer);
}

float readTemperature() {
  if (!tempAvailable) return NAN;

  tempSensor.requestTemperatures();
  float t = tempSensor.getTempCByIndex(0);

  if (t == DEVICE_DISCONNECTED_C || !isfinite(t) || t < -55 || t > 125) {
    return NAN;
  }

  return t;
}

float readVibrationRMS() {
  if (!accelAvailable) return NAN;

  const int N = 128;
  float ax[N], ay[N], az[N];
  sensors_event_t event;

  float meanX = 0, meanY = 0, meanZ = 0;

  for (int i = 0; i < N; i++) {
    accel.getEvent(&event);

    ax[i] = event.acceleration.x;
    ay[i] = event.acceleration.y;
    az[i] = event.acceleration.z;

    meanX += ax[i];
    meanY += ay[i];
    meanZ += az[i];

    delay(5);
  }

  meanX /= N;
  meanY /= N;
  meanZ /= N;

  double sumSquares = 0;

  for (int i = 0; i < N; i++) {
    float dx = ax[i] - meanX;
    float dy = ay[i] - meanY;
    float dz = az[i] - meanZ;

    sumSquares += dx * dx + dy * dy + dz * dz;
  }

  return sqrt(sumSquares / N);
}

float readSoundRMS() {
  const int N = 512;
  double sum = 0;
  double sumSquares = 0;

  for (int i = 0; i < N; i++) {
    int sample = analogRead(SOUND_PIN);
    sum += sample;
    sumSquares += (double)sample * sample;
    delayMicroseconds(100);
  }

  double mean = sum / N;
  double variance = (sumSquares / N) - (mean * mean);

  if (variance < 0) variance = 0;

  return sqrt(variance);
}

void updateRPM() {
  unsigned long now = millis();

  if (now - lastRpmCalculation < RPM_WINDOW_MS) return;

  noInterrupts();
  unsigned long pulses = hallPulses;
  hallPulses = 0;
  interrupts();

  unsigned long elapsed = now - lastRpmCalculation;
  lastRpmCalculation = now;

  if (elapsed > 0) {
    rpm = (pulses * 60000.0) /
          (PULSES_PER_REVOLUTION * elapsed);
  }
}

void readAllSensors() {
  readings.temperature = readTemperature();
  readings.vibration = readVibrationRMS();
  readings.sound = readSoundRMS();

  float v = pzem.voltage();
  float c = pzem.current();
  float w = pzem.power();

  readings.voltage = (isfinite(v) && v > 0) ? v : NAN;
  readings.current = (isfinite(c) && c >= 0) ? c : NAN;
  readings.power = (isfinite(w) && w >= 0) ? w : NAN;
}

String evaluateStatus() {
  bool warning = false;
  bool critical = false;
  bool sensorError = false;

  if (!isfinite(readings.temperature) ||
      !isfinite(readings.vibration) ||
      !isfinite(readings.current) ||
      !isfinite(readings.voltage) ||
      !isfinite(readings.power) ||
      !isfinite(readings.sound) ||
      !isfinite(rpm)) {
    sensorError = true;
  }

  if (isfinite(readings.temperature)) {
    if (readings.temperature >= TEMP_CRITICAL_C) critical = true;
    else if (readings.temperature >= TEMP_WARNING_C) warning = true;
  }

  if (isfinite(readings.vibration)) {
    if (readings.vibration >= VIB_CRITICAL_MS2) critical = true;
    else if (readings.vibration >= VIB_WARNING_MS2) warning = true;
  }

  if (isfinite(readings.current)) {
    if (readings.current >= CURRENT_CRITICAL_A) critical = true;
    else if (readings.current >= CURRENT_WARNING_A) warning = true;
  }

  if (isfinite(readings.sound)) {
    if (readings.sound >= SOUND_CRITICAL_RMS) critical = true;
    else if (readings.sound >= SOUND_WARNING_RMS) warning = true;
  }

  if (critical) return "critical";
  if (sensorError) return "sensor_error";
  if (warning) return "warning";

  return "ok";
}

void addFloatOrNull(JsonDocument& doc, const char* key, float value) {
  if (isfinite(value)) doc[key] = value;
  else doc[key] = nullptr;
}

void sendToAPI() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected");
    return;
  }

  readAllSensors();

  String timestamp = getTimestamp();

  if (timestamp.length() == 0) {
    Serial.println("Time not synchronized");
    return;
  }

  JsonDocument doc;

  doc["machine_id"] = MACHINE_ID;
  doc["timestamp"] = timestamp;

  addFloatOrNull(doc, "temperature_c", readings.temperature);
  addFloatOrNull(doc, "vibration_rms", readings.vibration);
  addFloatOrNull(doc, "current_a", readings.current);
  addFloatOrNull(doc, "voltage_v", readings.voltage);
  addFloatOrNull(doc, "power_w", readings.power);
  addFloatOrNull(doc, "sound_level", readings.sound);
  addFloatOrNull(doc, "rpm", rpm);

  doc["sensor_status"] = evaluateStatus();

  String payload;
  serializeJson(doc, payload);

  Serial.println(payload);

  HTTPClient http;
  http.setTimeout(5000);

  if (!http.begin(API_URL)) {
    Serial.println("HTTP initialization failed");
    return;
  }

  http.addHeader("Content-Type", "application/json");

  if (strlen(API_TOKEN) > 0) {
    http.addHeader("Authorization", String("Bearer ") + API_TOKEN);
  }

  int responseCode = http.POST(payload);

  if (responseCode > 0) {
    Serial.printf("HTTP response: %d\n", responseCode);
    Serial.println(http.getString());
  } else {
    Serial.printf("HTTP error: %s\n", http.errorToString(responseCode).c_str());
  }

  http.end();
}

void setup() {
  Serial.begin(115200);

  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);

  analogReadResolution(12);
  analogSetPinAttenuation(SOUND_PIN, ADC_11db);

  Wire.begin(21, 22);

  tempSensor.begin();
  tempAvailable = tempSensor.getDeviceCount() > 0;

  accelAvailable = accel.begin();

  if (accelAvailable) {
    accel.setRange(ADXL345_RANGE_16_G);
    accel.setDataRate(ADXL345_DATARATE_200_HZ);
  }

  pzemSerial.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();

  configTime(0, 0, "pool.ntp.org", "time.google.com");

  lastRpmCalculation = millis();

  Serial.println("YantraKavach initialized");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
  }

  updateRPM();

  if (millis() - lastUpload >= UPLOAD_INTERVAL_MS) {
    lastUpload = millis();
    sendToAPI();
  }

  delay(10);
}