////// Calibration //////
const double maxAngle = 5;

////// Accelerometer //////
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

////// PWM //////
const int ledPin = 23;
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 16;

void setupPWM(){
  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled

  ledcAttachPin(ledPin, ledChannel);
}

void setupAccelerometer(){
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

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

void setup(){
  setupPWM();
  setupAccelerometer();
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

double getAcceleration() {
    mpu.getEvent(&a, &g, &temp);
    double y = a.acceleration.y;
    if (y > maxAngle) {
      y = maxAngle;
    }
    if (y < 0) {
      y = 0;
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

void updateBrightness(double y) {
    ledcWrite(ledChannel, y / maxAngle * 65536);
    delay(15);
}

void loop(){
    //printAccel();

    double y = getAcceleration();
    updateBrightness(y);

}