#include "HardwareSerial.h"
#include "WString.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>

// ===== Configurations =====
#define DEVICE_ID 1
#define DEBUG_MODE 0
#define ACCELERATION_THRESHOLD 1
#define GYRO_THRESHOLD 0.5

// MQTT
const char *mqtt_server = ""; // broker URL
const int mqtt_port = 8883;
const char *INTERVAL_PUBLISH_TOPIC = "";
const char *EVENT_PUBLISH_TOPIC = "";
const char *mqtt_user = "";
const char *mqtt_password = "";
const int MQTT_RETRY_COOLDOWN = 2000; // in case mqtt fail to connect cool down (ms)

// root ca

const char *rootCACertificate = R"EOF(
-----BEGIN CERTIFICATE-----
Root Certificates goes here
-----END CERTIFICATE-----
)EOF";

// ===== END CONFIGURATION =====

// ===== GOLBAL =====
unsigned long now = millis();

WiFiClientSecure secureClient;
PubSubClient client(secureClient);

String getTimeString() {
  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo); // local time
  char buf[30];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buf);
}
int64_t getBsonUtcMillis() {
  time_t now = time(nullptr); // seconds since epoch
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts); // get precise time (sec + nsec)

  // Convert to milliseconds
  int64_t millis = static_cast<int64_t>(ts.tv_sec) * 1000 +
                   static_cast<int64_t>(ts.tv_nsec) / 1000000;

  return millis;
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(String(" try again in ") + MQTT_RETRY_COOLDOWN +
                     " milliseconds");

      delay(MQTT_RETRY_COOLDOWN);
    }
  }
}

// MPU Event struct
struct MpuEvent {
  sensors_event_t acceleration;
  sensors_event_t gyro;
};

// Globals

sensors_event_t a, g, t;
MpuEvent latest_event;
Adafruit_MPU6050 mpu;

const unsigned long SAMPLE_INTERVAL = 10;    // 10 ms sampling
const unsigned long AVERAGE_INTERVAL = 1000; // 1 s for averaging
const unsigned long OUTPUT_INTERVAL = 5000;  // 5 s output

unsigned long last_sample_time = 0;
unsigned long last_average_time = 0;
unsigned long last_output_time = 0;

// Accumulators
float acc_x_sum = 0, acc_y_sum = 0, acc_z_sum = 0;
float gyro_x_sum = 0, gyro_y_sum = 0, gyro_z_sum = 0;
int sample_count = 0;

// for calibration
float acc_x_calibrate_offset = 0, acc_y_calibrate_offset = 0,
      acc_z_calibrate_offset = 0;
float gyro_x_calibrate_offset = 0, gyro_y_calibrate_offset = 0,
      gyro_z_calibrate_offset = 0;

// Global variables to hold the latest calculated averages for output
float acc_x_avg_global = 0, acc_y_avg_global = 0, acc_z_avg_global = 0;
float gyro_x_avg_global = 0, gyro_y_avg_global = 0, gyro_z_avg_global = 0;

void sampleMPU6050() {
  mpu.getEvent(&a, &g, &t);

  acc_x_sum += a.acceleration.x;
  acc_y_sum += a.acceleration.y;
  acc_z_sum += a.acceleration.z;

  gyro_x_sum += g.gyro.x;
  gyro_y_sum += g.gyro.y;
  gyro_z_sum += g.gyro.z;

  sample_count++;
}

void collectData() {
  if (sample_count == 0){
    sampleMPU6050();
    return;
  }
  sampleMPU6050()
  acc_x_avg_global = (acc_x_sum / sample_count) - acc_x_calibrate_offset;
  acc_y_avg_global = (acc_y_sum / sample_count) - acc_y_calibrate_offset;
  acc_z_avg_global = (acc_z_sum / sample_count) - acc_z_calibrate_offset;

  gyro_x_avg_global = (gyro_x_sum / sample_count) - gyro_x_calibrate_offset;
  gyro_y_avg_global = (gyro_y_sum / sample_count) - gyro_y_calibrate_offset;
  gyro_z_avg_global = (gyro_z_sum / sample_count) - gyro_z_calibrate_offset;

  Serial.print("Acc X: ");
  Serial.print(acc_x_avg_global);
  Serial.print(", Y: ");
  Serial.print(acc_y_avg_global);
  Serial.print(", Z: ");
  Serial.println(acc_z_avg_global);

  Serial.print("Gyro X: ");
  Serial.print(gyro_x_avg_global);
  Serial.print(", Y: ");
  Serial.print(gyro_y_avg_global);
  Serial.print(", Z: ");
  Serial.println(gyro_z_avg_global);
  Serial.println();

  // clear sum
  acc_x_sum = acc_y_sum = acc_z_sum = 0;
  gyro_x_sum = gyro_y_sum = gyro_z_sum = 0;
  sample_count = 0;
}

void calibration(int samples = 100) {
  Serial.println("Calibrating... STAY STILL!!");

  float acc_x_sum_cal = 0, acc_y_sum_cal = 0, acc_z_sum_cal = 0;
  float gyro_x_sum_cal = 0, gyro_y_sum_cal = 0, gyro_z_sum_cal = 0;

  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&a, &g, &t);

    acc_x_sum_cal += a.acceleration.x;
    acc_y_sum_cal += a.acceleration.y;
    acc_z_sum_cal += a.acceleration.z;

    gyro_x_sum_cal += g.gyro.x;
    gyro_y_sum_cal += g.gyro.y;
    gyro_z_sum_cal += g.gyro.z;

    Serial.print(".");
    delay(10);
  }

  acc_x_calibrate_offset = acc_x_sum_cal / samples;
  acc_y_calibrate_offset = acc_y_sum_cal / samples;
  acc_z_calibrate_offset = acc_z_sum_cal / samples;

  gyro_x_calibrate_offset = gyro_x_sum_cal / samples;
  gyro_y_calibrate_offset = gyro_y_sum_cal / samples;
  gyro_z_calibrate_offset = gyro_z_sum_cal / samples;

  Serial.println();
  Serial.println("Calibration Done!");
  Serial.print("Acc offsets: ");
  Serial.print(acc_x_calibrate_offset);
  Serial.print(", ");
  Serial.print(acc_y_calibrate_offset);
  Serial.print(", ");
  Serial.println(acc_z_calibrate_offset);
  Serial.print("Gyro offsets: ");
  Serial.print(gyro_x_calibrate_offset);
  Serial.print(", ");
  Serial.print(gyro_y_calibrate_offset);
  Serial.print(", ");
  Serial.println(gyro_z_calibrate_offset);
}

void setup() {
  Serial.begin(9600);
  // Connect to WiFi
  WiFi.begin();
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  // Setup MQTT
  secureClient.setCACert(rootCACertificate);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // Sync time for TLS and timestamps
  configTime(0, 0, "pool.ntp.org");
  Serial.println("Time synchronized");
  Serial.println(" connected");
  Wire.begin(41, 40);
  delay(100);

  if (!mpu.begin(0x68)) {
    Serial.println("MPU6050 NOT FOUND!");
    while (1)
      delay(100);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  calibration();

  mpu.getEvent(&a, &g, &t);
  latest_event.acceleration = a;
  latest_event.gyro = g;
}
void sendEventMQTT(String change, String direction, float value,
                   boolean debug_mode = DEBUG_MODE) {
  String event = "";

  if (change == "acceleration") {
    if (direction == "X" && value > 0)
      event = "acceleration_left";
    else if (direction == "X" && value < 0)
      event = "acceleration_right";
    else if (direction == "Y" && value > 0)
      event = "acceleration_backward";
    else if (direction == "Y" && value < 0)
      event = "acceleration_farward";
    else if (direction == "Z" && value > 0)
      event = "acceleration_up";
    else if (direction == "Z" && value < 0)
      event = "acceleration_down";
  }

  if (change == "rotation") {
    if (direction == "X" && value > 0)
      event = "rotation_tilt_down";
    else if (direction == "X" && value < 0)
      event = "rotation_tilt_up";
    else if (direction == "Y" && value > 0)
      event = "rotation_roll_cw";
    else if (direction == "Y" && value < 0)
      event = "rotation_rool_ccw";
    else if (direction == "Z" && value > 0)
      event = "rotation_yaw_left";
    else if (direction == "Z" && value < 0)
      event = "rotation_yaw_right";
  }

  JsonDocument doc;
  doc["device_id"] = DEVICE_ID;
  doc["event"] = event;
  doc["type"] = change;
  doc["direction"] = direction;
  doc["value"] = value;
  doc["timestamp"] = getBsonUtcMillis();
  doc["timestring"] = getTimeString();

  char jsonString[256];
  serializeJson(doc, jsonString);

  client.publish(EVENT_PUBLISH_TOPIC, jsonString);
  Serial.printf("Data published to MQTT in topic %s", INTERVAL_PUBLISH_TOPIC);
  Serial.println(jsonString);
  client.loop();
}

void listenToEvent() {
  if (now - last_sample_time >= SAMPLE_INTERVAL) {
    last_sample_time = now;

    mpu.getEvent(&a, &g, &t);

    float acceleration_x_change = a.acceleration.x - latest_event.acceleration.acceleration.x;
    float acceleration_y_change = a.acceleration.y - latest_event.acceleration.acceleration.y;
    float acceleration_z_change = a.acceleration.z - latest_event.acceleration.acceleration.z;

    float gyro_x_change = g.gyro.x - latest_event.gyro.gyro.x;
    float gyro_y_change = g.gyro.y - latest_event.gyro.gyro.y;
    float gyro_z_change = g.gyro.z - latest_event.gyro.gyro.z;

    if (abs(acceleration_x_change) > ACCELERATION_THRESHOLD) {
      if (DEBUG_MODE) {
        Serial.println(acceleration_x_change);
      } else {
        sendEventMQTT("acceleration", "X", acceleration_x_change);
      }
    }

    if (abs(acceleration_y_change) > ACCELERATION_THRESHOLD) {
      if (DEBUG_MODE) {
        Serial.println(acceleration_y_change);
      } else {
        sendEventMQTT("acceleration", "Y", acceleration_y_change);
      }
    }

    if (abs(acceleration_z_change) > ACCELERATION_THRESHOLD) {
      if (DEBUG_MODE) {
        Serial.println(acceleration_z_change);
      } else {
        sendEventMQTT("acceleration", "Z", acceleration_z_change);
      }
    }

    if (abs(gyro_x_change) > GYRO_THRESHOLD) {
      if (DEBUG_MODE) {
        Serial.println(gyro_x_change);
      } else {
        sendEventMQTT("rotation", "X", gyro_x_change);
      }
    }

    if (abs(gyro_y_change) > GYRO_THRESHOLD) {
      if (DEBUG_MODE) {
        Serial.println(gyro_y_change);
      } else {
        sendEventMQTT("rotation", "Y", gyro_y_change);
      }
    }

    if (abs(gyro_z_change) > GYRO_THRESHOLD) {
      if (DEBUG_MODE) {
        Serial.println(gyro_z_change);
      } else {
        sendEventMQTT("rotation", "Z", gyro_z_change);
      }
    }

    latest_event.acceleration = a;
    latest_event.gyro = g;
  }
}

void print5SReport() {

  Serial.print("5s Report - Avg Acc X: ");
  Serial.println(acc_x_avg_global);
  Serial.print("5s Report - Avg Acc Y: ");
  Serial.println(acc_y_avg_global);
  Serial.print("5s Report - Avg Acc Z: ");
  Serial.println(acc_z_avg_global);

  Serial.print("5s Report - Avg Gyro X: ");
  Serial.println(gyro_x_avg_global);
  Serial.print("5s Report - Avg Gyro Y: ");
  Serial.println(gyro_y_avg_global);
  Serial.print("5s Report - Avg Gyro Z: ");
  Serial.println(gyro_z_avg_global);

  Serial.println("------------------------");
}

void calculateAveragesAndStore() {
  // Build JSON
  JsonDocument doc;
  doc["device_id"] = DEVICE_ID;
  doc["acc_x"] = acc_x_avg_global;
  doc["acc_y"] = acc_y_avg_global;
  doc["acc_z"] = acc_z_avg_global;
  doc["gyro_x"] = gyro_x_avg_global;
  doc["gyro_y"] = gyro_y_avg_global;
  doc["gyro_z"] = gyro_z_avg_global;
  doc["timestamp"] = getBsonUtcMillis();
  doc["timestring"] = getTimeString();

  char jsonString[256];
  serializeJson(doc, jsonString);

  // Publish to MQTT
  if (client.publish(INTERVAL_PUBLISH_TOPIC, jsonString)) {
    Serial.printf("Data published to MQTT in topic %s", INTERVAL_PUBLISH_TOPIC);
    Serial.println(jsonString);
    client.loop();
  } else {
    Serial.println("Failed to publish MQTT data");
  }
}

void setup() {
  Serial.begin(9600);
  // Connect to WiFi
  WiFi.begin();
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  // Setup MQTT
  secureClient.setCACert(rootCACertificate);
  client.setServer(mqtt_server, mqtt_port);

  // Sync time for TLS and timestamps
  configTime(0, 0, "pool.ntp.org");
  Serial.println("Time synchronized");
  Serial.println(" connected");
  Wire.begin(41, 40);
  delay(100);

  if (!mpu.begin(0x68)) {
    Serial.println("MPU6050 NOT FOUND!");
    while (1)
      delay(100);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  calibration();

  mpu.getEvent(&a, &g, &t);
  latest_event.acceleration = a;
  latest_event.gyro = g;
}

void loop() {
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin();
    Serial.print(".");
    delay(500);
  }

  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  now = millis();

  listenToEvent(); // this also include watch for event change

  if (now - last_average_time >= AVERAGE_INTERVAL) {
    last_average_time = now;
    collectData();
  }

  if (now - last_output_time >= OUTPUT_INTERVAL) {
    last_output_time = now;
    calculateAveragesAndStore();
    if (DEBUG_MODE) {
      print5SReport();
    }
  }
}