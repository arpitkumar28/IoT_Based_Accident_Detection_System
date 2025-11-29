#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

MPU6050 mpu;

TinyGPSPlus gps;
SoftwareSerial gpsSerial(3, 4);  

SoftwareSerial sim800l(10, 11); 

#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9

#define BUZZER 5

#define TILT_THRESHOLD 30

String PHONE = "9142170211";

bool accidentSent = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  mpu.initialize();
  mpu.setSleepEnabled(false);

  gpsSerial.begin(9600);

  sim800l.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  Serial.println("System Started...");
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void buzzerAlarm() {
  digitalWrite(BUZZER, HIGH); 
}

float getTiltAngle() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float axg = ax / 16384.0;
  float ayg = ay / 16384.0;
  float azg = az / 16384.0;

  float pitch = atan2(axg, sqrt(ayg * ayg + azg * azg)) * 57.3;
  float roll  = atan2(ayg, sqrt(axg * axg + azg * azg)) * 57.3;

  return max(abs(pitch), abs(roll));
}

void sendSMS(String message) {
  sim800l.println("AT+CMGF=1");
  delay(500);
  sim800l.println("AT+CMGS=\"" + PHONE + "\"");
  delay(500);
  sim800l.println(message);
  delay(500);
  sim800l.write(26);
  delay(3000);
}

void callNumber() {
  sim800l.println("ATD" + PHONE + ";");
  delay(1000);
}
void sendAccidentAlert(float lat, float lng) {
  String msg = "ACCIDENT DETECTED!\nTilt above 30°\nLocation:\n";
  msg += "Lat: " + String(lat, 6) + "\nLng: " + String(lng, 6);

  sendSMS(msg);
  callNumber();
}

void loop() {

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  float tilt = getTiltAngle();

  Serial.print("Tilt Angle: ");
  Serial.println(tilt);

  bool accident = false;

  if (tilt > TILT_THRESHOLD) {
    Serial.println("ACCIDENT DETECTED due to HIGH TILT!");
    accident = true;
  }

  if (accident && !accidentSent) {
    stopMotors();
    buzzerAlarm(); 

    if (gps.location.isValid()) {
      float lat = gps.location.lat();
      float lng = gps.location.lng();
      sendAccidentAlert(lat, lng);
    } else {
      sendSMS("ACCIDENT! GPS not fixed yet!");
      callNumber();
    }

    accidentSent = true;
  }

  delay(200);
}
