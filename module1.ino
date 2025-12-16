#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <Preferences.h>  // <--- ADDED THIS

Adafruit_MPU6050 mpu;
Preferences preferences;  // <--- ADDED THIS (This fixes your error)

// Pin definitions
const int pirPin = 27;
const int rcwlPin = 17;

const int ledPin = 2;        
const int wifi_on = 19;
const int wifi_off = 32;
const int motion_yellow = 18;
const int buttonPin = 4;            // Calibration button
const int wificonfig_button = 5;

// state variables
float refAccelX, refAccelY, refAccelZ;
float last_angle;
bool lastButtonState = HIGH;
bool lastwifibuttonstate = HIGH;

int motion_status = 0;
int last_motion = 0;
int tilt_status = 0;
int last_tilt = 0;

// Flask server URL variables
char serverUrlBuffer[100]; // We will load this from memory now
String serverUrl;

// -------------------------
// NEW ADVANCED CALIBRATION
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

  Serial.println("New reference saved!");
  Serial.print("Ref X: "); Serial.println(refAccelX);
  Serial.print("Ref Y: "); Serial.println(refAccelY);
  Serial.print("Ref Z: "); Serial.println(refAccelZ);
  digitalWrite(wifi_on, 0);
  digitalWrite(wifi_off, 0);
}
// END CALIBRATION

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

  calibrate();  // FIRST TIME CALIBRATION

  // --- PREFERENCES (MEMORY) SETUP ---
  preferences.begin("my-app", false); 
  
  // Load saved URL. If nothing saved, use the default.
  // Note: I put your default IP here.
  String storedUrl = preferences.getString("server_url", "http://10.229.135.218:5000/update");
  
  // Convert String to char array for WiFiManager
  storedUrl.toCharArray(serverUrlBuffer, 100);
  
  serverUrl = storedUrl; // Update the main string variable
  Serial.print("Loaded Server URL: "); Serial.println(serverUrl);

  // --- WiFiManager setup ---
  WiFiManager wifiManager;
  
  // Display the currently stored URL in the text box
  WiFiManagerParameter custom_server_url("server", "Flask Server URL", serverUrlBuffer, 100);
  wifiManager.addParameter(&custom_server_url);

  if (!wifiManager.autoConnect("ESP32-Setup")) {
    Serial.println("Failed to connect. Restarting...");
    digitalWrite(wifi_on, 0);
    digitalWrite(wifi_off, 1);
    ESP.restart();
  } else {
    Serial.println("Connected to WiFi");
    digitalWrite(wifi_on, 1);
    digitalWrite(wifi_off, 0);
  }

  // Check if the URL changed during the initial setup portal (rare, but good to check)
  String newUrl = custom_server_url.getValue();
  if (newUrl != storedUrl) {
    Serial.println("URL changed during setup. Saving...");
    preferences.putString("server_url", newUrl);
    serverUrl = newUrl;
  }

  Serial.print("Final Server URL: "); Serial.println(serverUrl);
  Serial.print("ESP32 IP: "); Serial.println(WiFi.localIP());
}

void loop() {
  //wifi status
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(wifi_on, 1);
    digitalWrite(wifi_off, 0);
  } else {
    digitalWrite(wifi_on, 0);
    digitalWrite(wifi_off, 1);
  }
  
  // Runtime WiFi reconfig
  bool wifibuttonstate = digitalRead(wificonfig_button);
  if (wifibuttonstate == LOW && lastwifibuttonstate == HIGH) {

    delay(200);
    digitalWrite(wifi_on, 0);
    digitalWrite(wifi_off, 1);
    Serial.println("Starting WiFi config...");

    WiFiManager wifiManager;
    wifiManager.setBreakAfterConfig(true);
    
    // Pass the CURRENT URL buffer so the user sees what is currently saved
    // Convert current serverUrl String back to char array for the buffer
    serverUrl.toCharArray(serverUrlBuffer, 100);
    
    WiFiManagerParameter custom_server_url("server", "Flask Server URL", serverUrlBuffer, 100);
    wifiManager.addParameter(&custom_server_url);

    wifiManager.startConfigPortal("ESP32-Setup");

    // --- SAVE THE NEW VALUE ---
    String newUrl = custom_server_url.getValue();
    Serial.print("Saving new URL to flash memory: ");
    Serial.println(newUrl);
    
    // Save to permanent memory
    preferences.putString("server_url", newUrl);
    preferences.end(); // Close preferences

    Serial.println("Saved. Restarting to apply changes...");
    delay(1000);
    ESP.restart(); 
  }
  lastwifibuttonstate = wifibuttonstate;

  // ------------ CALIBRATION BUTTON ------------
  bool buttonState = digitalRead(buttonPin);
  if (buttonState == LOW && lastButtonState == HIGH) {
    delay(50);
    calibrate();
  }
  lastButtonState = buttonState;

  // ------------ MOTION READINGS ----------------
  int motion1 = digitalRead(pirPin);
  int motion2 = digitalRead(rcwlPin);
  Serial.print("pir: ");
  Serial.println(motion1);
  Serial.print("rcwl: ");
  Serial.println(motion2);

  // ------------ TILT CALCULATION ---------------
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float curX = a.acceleration.x;
  float curY = a.acceleration.y;
  float curZ = a.acceleration.z;

  float dot = curX * refAccelX + curY * refAccelY + curZ * refAccelZ;
  float magCur = sqrt(curX*curX + curY*curY + curZ*curZ);
  float magRef = sqrt(refAccelX*refAccelX + refAccelY*refAccelY + refAccelZ*refAccelZ);
  float angle = acos(dot / (magCur * magRef)) * 180.0 / PI;

  Serial.print("Tilt Angle: ");
  Serial.println(angle);
  if(last_angle>30 && angle<30)
  {
    tilt_status=1;
  }
  
  else if(angle>30 && last_angle!=angle)
  {
    tilt_status=1;
  }
  else
  {
    tilt_status=0;
  }
  last_angle=angle;
  last_motion = motion_status;
  motion_status = (motion1 == HIGH && motion2 == HIGH);

  digitalWrite(ledPin, motion_status);
  digitalWrite(motion_yellow, motion_status);

  // ------------ SEND ON CHANGE -----------------
  if (last_motion != motion_status || tilt_status)
   {

    String jsonPayload =
      "{\"motion\":" + String(motion_status) +
      ",\"tilt_angle\":" + String(angle) + "}";

    if (WiFi.status() == WL_CONNECTED)
     {
        digitalWrite(wifi_on, HIGH);
        digitalWrite(wifi_off, 0);

        HTTPClient http;
        http.begin(serverUrl); // Uses the global serverUrl string
        http.addHeader("Content-Type", "application/json");

        int code = http.POST(jsonPayload);

        if (code > 0) 
        {
          Serial.println("Sent to server");
          Serial.println(http.getString());
        } 
        else
        {
          Serial.print("HTTP Error: ");
          Serial.println(code);
        }
        http.end();

    } 
    else
     {
      Serial.println("WiFi lost. Reconnecting...");
      WiFi.reconnect();
    }
  }

  delay(1000);
}
