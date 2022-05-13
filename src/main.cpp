// #define SERVER
// #define UV
#define WARM

////// Calibration //////
const double maxAngle = 5;

////// Wifi server //////
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
const char* ssid = "equilibrio";
const char* password = "123456789";
AsyncWebServer server(80);

////// Wifi client //////
#include <HTTPClient.h>
const char* serverNameUV = "http://192.168.4.1/uv";
const char* serverNameWarm = "http://192.168.4.1/warm";
unsigned long previousMillis = 0;
const long interval = 200;

////// Accelerometer //////
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
double acceleration;

////// PWM //////
const int ledPin = 23;
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 16;

////// 220 dimmer //////
#include <RBDdimmer.h>//
#define outputPin  26
#define zerocross  27
dimmerLamp dimmer(outputPin, zerocross);

double getAcceleration() {
    mpu.getEvent(&a, &g, &temp);
    double y = a.acceleration.y;
    if (y > maxAngle) {
      y = maxAngle;
    }
    if (y < -maxAngle) {
      y = -maxAngle;
    }

    // debug
    // Serial.print("y:");
    // Serial.print(y);
    // Serial.print(", ");
    // Serial.print("result:");
    // Serial.print(y / 5 * 255);
    // Serial.println("");

    return y;
}

String readUV() {
  return String(map(acceleration, -maxAngle, maxAngle, 0, 65536));
}

String readWarm() {
  return String(map(acceleration, -maxAngle, maxAngle, 100, 0));
}

void setupWifiServer() {
  // Setting the ESP as an access point
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.on("/uv", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readUV().c_str());
  });

  server.on("/warm", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", readWarm().c_str());
  });

  // Start server
  server.begin();
}

void setupWifiClient() {
    WiFi.begin(ssid, password);
    Serial.println("Connecting");
    while(WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());
}

void setupPWM(){
  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled

  ledcAttachPin(ledPin, ledChannel);
}

void setupAccelerometer(){

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  //setupt motion detection
  // mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  // mpu.setMotionDetectionThreshold(1);
  // mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  // mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);
}

void setup220Dimmer() {
  pinMode(12, INPUT);
  dimmer.begin(NORMAL_MODE, ON); //dimmer initialisation: name.begin(MODE, STATE)
}

String httpGETRequest(const char* serverName) {
  HTTPClient http;

  // Your IP address with path or Domain name with URL path
  http.begin(serverName);

  // Send HTTP POST request
  int httpResponseCode = http.GET();

  String payload = "--";

  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}
 
void printAccel() {
    Serial.print("AccelX:");
    Serial.print(a.acceleration.x);
    Serial.print(",");
    Serial.print("AccelY:");
    Serial.print(a.acceleration.y);
    Serial.print(",");
    Serial.print("AccelZ:");
    Serial.print(a.acceleration.z);
    Serial.print(", ");
    Serial.print("GyroX:");
    Serial.print(g.gyro.x);
    Serial.print(",");
    Serial.print("GyroY:");
    Serial.print(g.gyro.y);
    Serial.print(",");
    Serial.print("GyroZ:");
    Serial.print(g.gyro.z);
    Serial.println("");
}

void updateBrightness(double y) {
    ledcWrite(ledChannel, y);
}

void setup() {

  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  #if defined(SERVER)
  setupWifiServer();
  setupAccelerometer();
  #endif

  #if defined(UV)
  setupWifiClient();
  setupPWM();
  #endif

  #if defined(WARM)
  setupWifiClient();
  setup220Dimmer();
  #endif
}

void loop(){
  // printAccel();
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {

    #if defined(UV) || defined(WARM)
    // Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED) {

      #if defined(UV)
      String uv = httpGETRequest(serverNameUV);
      Serial.println("UV: " + uv);
      updateBrightness(atof(uv.c_str()));
      #endif

      #if defined(WARM)
      String warm = httpGETRequest(serverNameWarm);
      Serial.println("Warm: " + warm);
      dimmer.setPower(static_cast<int>(atof(warm.c_str())));
      #endif

    }
    #endif

    #if defined(SERVER)
      acceleration = getAcceleration();
      Serial.println("Acceleration: " + String(acceleration, 2) + ", UV: " + readUV() + ", Warm: " + readWarm());
    #endif

    previousMillis = millis();
  }
}