#include <Servo.h>

Servo servoMotorX; // Servo za x-os (pan)
Servo servoMotorY; // Servo za y-os (tilt)

int servoPinX = 9;  // Pin za x-os servo
int servoPinY = 10; // Pin za y-os servo

int servoAngleX = 90; // inicijalna pozicija za x-os
int servoAngleY = 95; // inicijalna pozicija za y-os

void setup() {
  // Inicijaliziranje serijske komunikacije
  Serial.begin(9600);
  
  // pinovi za servo motore
  servoMotorX.attach(servoPinX);
  servoMotorY.attach(servoPinY);
  
  // Inicijalne pozicije
  servoMotorX.write(servoAngleX);
  servoMotorY.write(servoAngleY);
}

void loop() {
  // Provjera ako su podaci dostupni za čitanje
  if (Serial.available() > 0) {
    // Čita x i y podatke za kut okreta
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    if (commaIndex != -1) {
      int angleX = data.substring(0, commaIndex).toInt();
      int angleY = data.substring(commaIndex + 1).toInt();

      // provjerava ako je kut servo osi u valjanoj poziciji
      if (angleX >= 0 && angleX <= 180) {
        servoAngleX = angleX;
        servoMotorX.write(servoAngleX);
      }

      if (angleY >= 0 && angleY <= 180) {
        servoAngleY = angleY;
        servoMotorY.write(servoAngleY);
      }
    }
  } else {
    // Ako ne čita podatke vraća se na početnu poziciju (STOP)
    servoMotorX.write(servoAngleX);
    servoMotorY.write(servoAngleY);
  }
}
