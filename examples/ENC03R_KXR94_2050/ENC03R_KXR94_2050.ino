/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.
 
 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").
 
 Contact information
 -------------------
 
 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */
/*
 This example sketch is a demonstration of gyro scope and accelarator sensor with Kalman filter.
 This uses ENC-03R & KXR94-2050 (popular & low cost senseors in Japan.
 http://akizukidenshi.com/catalog/g/gK-04912/
 http://akizukidenshi.com/catalog/g/gM-05153/
 
 Katsumi ISHIDA
 Web      : http://isisredirect2hard.blogspot.jp/
 e-mail   :  isis331@gmail.com
*/

#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

static const int pin_gyrox = A0;
static const int pin_gyroy = A1;
static const int pin_accx = A3;
static const int pin_accy = A2;
static const int pin_accz = A4;

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
int accX, accY, accZ;
double gyroX, gyroY;
double gyroX_mean, gyroY_mean;

double accXangle, accYangle; // Angle calculate using the accelerometer
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter

uint32_t timer;


void setup() {  
  Serial.begin(115200);

  pinMode(pin_gyrox, INPUT);
  digitalWrite(pin_gyrox, HIGH);
  pinMode(pin_gyroy, INPUT);
  digitalWrite(pin_gyroy, HIGH);
  pinMode(pin_accx, INPUT);
  digitalWrite(pin_accx, HIGH);
  pinMode(pin_accy, INPUT);
  digitalWrite(pin_accy, HIGH);
  pinMode(pin_accz, INPUT);
  digitalWrite(pin_accz, HIGH);

  gyroX_mean = 0;
  gyroY_mean = 0;
  for (int i = 0; i < 1024; ++i) {
    gyroX_mean += analogRead(pin_gyrox);
    gyroY_mean += analogRead(pin_gyroy);
  }
  gyroX_mean /= 1024.;
  gyroY_mean /= 1024.;

  /* Set kalman and gyro starting angle */
  accX = analogRead(pin_accx) - 750;
  accY = analogRead(pin_accy) - 750;
  accZ = analogRead(pin_accz) - 750;
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;

  kalmanX.setAngle(accXangle); // Set starting angle
  kalmanY.setAngle(accYangle);
  gyroXangle = accXangle;
  gyroYangle = accYangle;
  compAngleX = accXangle;
  compAngleY = accYangle;

  timer = micros();
}

void loop() {
  uint32_t now = micros();
  double dt = (now - timer)/1000000.;

  /* Update all the values */
  accX = analogRead(pin_accx) - 750;
  accY = analogRead(pin_accy) - 750;
  accZ = analogRead(pin_accz) - 750;
  gyroX = analogRead(pin_gyrox) - gyroX_mean;
  gyroY = analogRead(pin_gyroy) - gyroY_mean;

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;

  double gyroXrate = 1.3 * gyroX;
  double gyroYrate = 1.3 * gyroY;
  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter  
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate()*dt); // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate()*dt);
  
  compAngleX = (0.93*(compAngleX+(gyroXrate * dt)))+(0.07 * accXangle); // Calculate the angle using a Complimentary filter
  compAngleY = (0.93*(compAngleY+(gyroYrate * dt)))+(0.07 * accYangle);

  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, dt); // Calculate the angle using a Kalman filter
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, dt);
  timer = now;


  /* Print Data */
 /*
  Serial.print(accX);
  Serial.print("\t");
  Serial.print(accY);
  Serial.print("\t");
  Serial.print(accZ);
  Serial.print("\t");

  Serial.print(gyroX);
  Serial.print("\t");
  Serial.print(gyroY); 
  Serial.print("\t");
  //  Serial.print(gyroZ);Serial.print("\t");
  Serial.print("\t");
 */
  Serial.print(accXangle);
  Serial.print("\t");
  Serial.print(gyroXangle);
  Serial.print("\t");
  Serial.print(compAngleX);
  Serial.print("\t");
  Serial.print(kalAngleX);
  Serial.print("\t");

  Serial.print("\t");

  Serial.print(accYangle);
  Serial.print("\t");
  Serial.print(gyroYangle);
  Serial.print("\t");
  Serial.print(compAngleY); 
  Serial.print("\t");
  Serial.print(kalAngleY);
  Serial.print("\t");

  //Serial.print(temp);Serial.print("\t");

  Serial.print("\r\n");
  delay(1);
}

