#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <Preferences.h>

Adafruit_MPU6050 mpu;
Preferences preferences;

// Pin definitions
const int pirPin = 27;
const int rcwlPin = 17;
const int ledPin = 2;        
const int wifi_on = 19;
const int wifi_off = 32;
const int motion_yellow = 18;
const int buttonPin = 4;            // Calibration button
const int wificonfig_button = 5;

// State variables
float refAccelX, refAccelY, refAccelZ;
float refMag; // Optimization: Store reference magnitude
float last_angle = 0.0;

// Button Debouncing variables
int lastButtonState = HIGH;
int lastWifiButtonState = HIGH;

// Timing variables (Non-blocking delay)
unsigned long previousMillis = 0;
const long interval = 1000; // Run sensor logic every 1000ms (1 second)

int motion_status = 0;
int last_motion = 0;
int tilt_status = 0;

// Server URL
char serverUrlBuffer[100];
String serverUrl;

// -------------------------
// OPTIMIZED CALIBRATION
// -------------------------
void calibrate() {
  digitalWrite(wifi_on, 1);
  digitalWrite(wifi_off, 1);
  const int samples = 50;
  float sumX = 0, sumY = 0, sumZ = 0;
  sensors_event_t a, g, temp;

  Serial.println("Calibrating... keep sensor steady");
  delay(1000); 

  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&a, &g, &temp);
    sumX += a.acceleration.x;
    sumY += a.acceleration.y;
    sumZ += a.acceleration.z;
    delay(20);
  }

  refAccelX = sumX / samples;
  refAccelY = sumY / samples;
  refAccelZ = sumZ / samples;
  
  // Calculate reference magnitude once here
  refMag = sqrt(refAccelX*refAccelX + refAccelY*refAccelY + refAccelZ*refAccelZ);

  Serial.println("New reference saved!");
  Serial.print("Ref X: "); Serial.println(refAccelX);
  Serial.print("Ref Y: "); Serial.println(refAccelY);
  Serial.print("Ref Z: "); Serial.println(refAccelZ);
  
  digitalWrite(wifi_on, 0);
  digitalWrite(wifi_off, 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(pirPin, INPUT);
  pinMode(rcwlPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(wifi_on, OUTPUT);
  pinMode(wifi_off, OUTPUT);
  pinMode(motion_yellow, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(wificonfig_button, INPUT_PULLUP);

  // --- MPU6050 setup ---
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  calibrate(); 

  // --- PREFERENCES ---
  preferences.begin("my-app", false);
  String storedUrl = preferences.getString("server_url", "http://10.229.135.218:5000/update");
  storedUrl.toCharArray(serverUrlBuffer, 100);
  serverUrl = storedUrl;
  Serial.print("Loaded Server URL: "); Serial.println(serverUrl);

  // --- WiFiManager ---
  WiFiManager wifiManager;
  WiFiManagerParameter custom_server_url("server", "Flask Server URL", serverUrlBuffer, 100);
  wifiManager.addParameter(&custom_server_url);

  if (!wifiManager.autoConnect("ESP32-Setup")) {
    Serial.println("Failed to connect. Restarting...");
    ESP.restart();
  } else {
    Serial.println("Connected to WiFi");
    digitalWrite(wifi_on, 1);
    digitalWrite(wifi_off, 0);
  }

  String newUrl = custom_server_url.getValue();
  if (newUrl != storedUrl) {
    preferences.putString("server_url", newUrl);
    serverUrl = newUrl;
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // ==========================================
  // 1. FAST LOOP (Buttons & WiFi Status)
  // ==========================================
  
  // WiFi Config Button
  int readingWifi = digitalRead(wificonfig_button);
  if (readingWifi == LOW && lastWifiButtonState == HIGH) {
      delay(50); // debounce
      if(digitalRead(wificonfig_button) == LOW) {
        Serial.println("Starting WiFi config...");
        digitalWrite(wifi_on, 0);
        digitalWrite(wifi_off, 1);
        
        WiFiManager wifiManager;
        wifiManager.setBreakAfterConfig(true);
        serverUrl.toCharArray(serverUrlBuffer, 100);
        WiFiManagerParameter custom_server_url("server", "Flask Server URL", serverUrlBuffer, 100);
        wifiManager.addParameter(&custom_server_url);
        
        wifiManager.startConfigPortal("ESP32-Setup");
        
        String newUrl = custom_server_url.getValue();
        preferences.putString("server_url", newUrl);
        preferences.end();
        ESP.restart();
      }
  }
  lastWifiButtonState = readingWifi;

  // Calibration Button
  int readingCal = digitalRead(buttonPin);
  if (readingCal == LOW && lastButtonState == HIGH) {
     delay(50);
     if(digitalRead(buttonPin) == LOW) calibrate();
  }
  lastButtonState = readingCal;

  // WiFi LED Status
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(wifi_on, 1);
    digitalWrite(wifi_off, 0);
  } else {
    digitalWrite(wifi_on, 0);
    digitalWrite(wifi_off, 1);
  }

  // ==========================================
  // 2. SLOW LOOP (Sensors & Logging)
  // ==========================================
  // This runs every 1000ms (1 second)
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; 

    // --- READ SENSORS ---
    int motion1 = digitalRead(pirPin);
    int motion2 = digitalRead(rcwlPin);

    // --- RESTORED LOGGING ---
    Serial.print("pir: ");
    Serial.println(motion1);
    Serial.print("rcwl: ");
    Serial.println(motion2);
    
    // --- TILT MATH ---
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float curX = a.acceleration.x;
    float curY = a.acceleration.y;
    float curZ = a.acceleration.z;

    float dot = curX * refAccelX + curY * refAccelY + curZ * refAccelZ;
    float magCur = sqrt(curX*curX + curY*curY + curZ*curZ);
    
    float angle = 0.0;
    // Safety check to prevent NaN
    if (magCur * refMag != 0) {
      float cosine = dot / (magCur * refMag);
      if (cosine > 1.0) cosine = 1.0;
      if (cosine < -1.0) cosine = -1.0;
      angle = acos(cosine) * 180.0 / PI;
    }

    // --- RESTORED LOGGING ---
    Serial.print("Tilt Angle: ");
    Serial.println(angle);

    // --- LOGIC ---
    tilt_status = 0;

    // Logic: Trigger if angle > 30 AND changed by at least 2 degrees
    if(last_angle > 30 && angle < 30) {
       tilt_status = 1;
    }
    else if(angle > 30 && abs(angle - last_angle) > 2.0) {
       tilt_status = 1;
    }
    else {
       tilt_status = 0;
    }

    last_angle = angle;
    last_motion = motion_status;
    motion_status = (motion1 == HIGH && motion2 == HIGH);

    digitalWrite(ledPin, motion_status);
    digitalWrite(motion_yellow, motion_status);

    // --- SEND DATA ---
    if (last_motion != motion_status || tilt_status) {
       if (WiFi.status() == WL_CONNECTED) {
         digitalWrite(wifi_on, HIGH);
         digitalWrite(wifi_off, 0);

         HTTPClient http;
         http.begin(serverUrl);
         http.addHeader("Content-Type", "application/json");

         // Using snprintf for safe string formatting
         char payload[64];
         snprintf(payload, sizeof(payload), "{\"motion\":%d,\"tilt_angle\":%.2f}", motion_status, angle);
         
         int code = http.POST(payload);
         
         // RESTORED HTTP LOGGING
         if (code > 0) {
            Serial.println("Sent to server");
            Serial.println(http.getString());
         } else {
            Serial.print("HTTP Error: ");
            Serial.println(code);
         }
         http.end();
       } else {
          Serial.println("WiFi lost. Reconnecting...");
          WiFi.reconnect();
       }
    }
  } // End of 1 second interval
}
