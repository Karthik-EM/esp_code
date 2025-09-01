
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>

Adafruit_MPU6050 mpu;

// Pin definitions
const int pirPin = 27;
const int rcwlPin = 26;
const int ledPin = 2;
const int buttonPin = 4;//calibration
const int wifi_blue = 19;
const int motion_yellow = 18;

float refAccelX, refAccelY, refAccelZ;
bool lastButtonState = HIGH;

// status variables
int motion_status = 0;
int last_motion = 0;
int tilt_status = 0;
int last_tilt = 0;


// Flask server URL (default)
String serverUrl = " http://192.168.43.192:5000/update";

void calibrate() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  refAccelX = a.acceleration.x;
  refAccelY = a.acceleration.y;
  refAccelZ = a.acceleration.z;
  Serial.println("Calibration done!");
}

void setup() {
  Serial.begin(115200);

  pinMode(pirPin, INPUT);
  pinMode(rcwlPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(wifi_blue, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  // --- MPU6050 setup ---
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip. Check wiring!");
    while (1) { delay(10); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
  calibrate();

  // --- WiFiManager setup ---
  WiFiManager wifiManager;

  // Custom parameter for Flask server URL
  WiFiManagerParameter custom_server_url("server", "Flask Server URL", serverUrl.c_str(), 100);
  wifiManager.addParameter(&custom_server_url);

  // autoConnect: try saved Wi-Fi, else start AP "ESP32-Setup"
  if (!wifiManager.autoConnect("ESP32-Setup")) {
    Serial.println("Failed to connect, restarting...");
    ESP.restart();
  }
  else
  {
    Serial.println("connected");
    digitalWrite(wifi_blue,1);
    delay(500);
    digitalWrite(wifi_blue,0);
  }

  // Read server URL from portal
  serverUrl = custom_server_url.getValue();
  Serial.print("Using Flask server: ");
  Serial.println(serverUrl);
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  int motionDetected1 = digitalRead(pirPin);
  int motionDetected2 = digitalRead(rcwlPin);

  // check button for recalibration
  bool buttonState = digitalRead(buttonPin);
  if (buttonState == LOW && lastButtonState == HIGH) {
    delay(50);
    calibrate();
  }
  lastButtonState = buttonState;

  // get accelerometer readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float curX = a.acceleration.x;
  float curY = a.acceleration.y;
  float curZ = a.acceleration.z;

  float dot = curX * refAccelX + curY * refAccelY + curZ * refAccelZ;
  float magCur = sqrt(curX * curX + curY * curY + curZ * curZ);
  float magRef = sqrt(refAccelX * refAccelX + refAccelY * refAccelY + refAccelZ * refAccelZ);

  float angle = acos(dot / (magCur * magRef)) * 180.0 / PI;

  Serial.print("Tilt Angle: ");
  Serial.println(angle);

  // Tilt logic
  last_tilt = tilt_status;
  tilt_status = (angle > 30.0) ? 1 : 0;

  // Motion logic
  last_motion = motion_status;
  motion_status = (motionDetected1 == HIGH && motionDetected2 == HIGH) ? 1 : 0;
  
  digitalWrite(ledPin, motion_status);
  digitalWrite(motion_yellow, motion_status);

  // Send data only if status changed
  if (last_motion != motion_status || last_tilt != tilt_status) {
    if (WiFi.status() == WL_CONNECTED) {
      digitalWrite(wifi_blue,0);
      HTTPClient http;
      http.begin(serverUrl);
      http.addHeader("Content-Type", "application/json");

      String jsonPayload = "{\"tilt_angle\":" + String(angle) + ",\"motion\":" + String(motion_status) + "}";
      int httpResponseCode = http.POST(jsonPayload);

      if (httpResponseCode > 0) {
        Serial.println("Message sent to server");
        Serial.println(http.getString());
      } else {
        Serial.print("Error sending POST: ");
        Serial.println(httpResponseCode);
      }
      http.end();
    } else {
      Serial.println("WiFi disconnected");
      digitalWrite(wifi_blue,1);
    }
  }

  delay(1000);
}
