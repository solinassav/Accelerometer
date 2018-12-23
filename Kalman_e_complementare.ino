#include <SPI.h>
#include <Wire.h>
#define MPU 0x68
int timer;
float Pitch, Roll, comp_x, comp_y, kPitch, kRoll;
float AcX, AcY, AcZ, GyX, GyY, GyZ;
//variabili del filtro di Kalman, da notare che sono raddoppiate mentre sarebbe convenuto creare una classe per i filtro di kalman 
float Q_angle  =  0.001, Q_angleY = 0.001;
float Q_gyro   =  0.003, Q_gyroY = 0.003;
float R_angle  =  0.03, R_angleY = 0.03;
float x_angle = 0, x_angleY = 0;
float x_bias = 0, x_biasY = 0;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0, P_00Y = 0, P_01Y = 0, P_10Y = 0, P_11Y = 0;
float dt, y, S, dtY, yY, SY;
float K_0, K_1, K_0Y, K_1Y;
//scegliere il tipo di filtro KALMAN o COMPLEMENTARE o COMPLEMENTARE_KALMAN oppure i dati RAW
#define COMPLEMENTARE_KALMAN
void setup() {
  Serial.begin(115200);
  init_MPU();
  timer = micros();
  FunctionsMPU();
  comp_x = FunctionsPitchRoll(AcX, AcY, AcZ);
  comp_y = FunctionsPitchRoll(AcY, AcX, AcZ);
}

void loop() {
  FunctionsMPU();
  Pitch = FunctionsPitchRoll(AcX, AcY, AcZ);
  Roll = FunctionsPitchRoll(AcY, AcX, AcZ);
  
#ifdef KALMAN
 kPitch = kalmanCalculate(Pitch, GyY,  micros() - timer);
  kRoll = kalmanCalculateY(Roll, GyX,  micros() - timer);
 Serial.print(kPitch);
  Serial.print('\t');
  Serial.print(kRoll);
  Serial.print('\n');
#endif

#ifdef COMPLEMENTARE
  comp_x = 0.95 * (comp_x + (GyX * dt)) + 0.05 * Pitch;
  comp_y = 0.95 * (comp_y + (GyY * dt)) + 0.05 * Roll;
  Serial.print(comp_x,10);
  Serial.print('\t');
  Serial.print(comp_y,10);
  Serial.print('\n');
#endif

#ifdef COMPLEMENTARE_KALMAN
  kPitch = kalmanCalculate(Pitch, GyY,  micros() - timer);
  kRoll = kalmanCalculateY(Roll, GyX,  micros() - timer);
  comp_x = 0.95 * (comp_x + (GyX * dt)) + 0.05 * kPitch;
  comp_y = 0.95 * (comp_y + (GyY * dt)) + 0.05 * kRoll;
  Serial.print(comp_x);
  Serial.print('\t');
  Serial.print(comp_y);
  Serial.print('\n');
#endif

#ifdef RAW

#endif

  timer = micros();

}

void init_MPU() {

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);


}

void FunctionsMPU() {

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
  AcZ =  4.*AcZ / 65536.*9.81;
  AcX = 4.*AcX / 65536.*9.81;
  AcY = 4.*AcY / 65536.*9.81;

}

double FunctionsPitchRoll(double A, double B, double C) {

  double DatoA, DatoB, Value;
  DatoA = A;
  DatoB = (B * B) + (C * C);
  DatoB = sqrt(DatoB);

  Value = atan2(DatoA, DatoB);
  Value = Value * 180 / 3.14;

  return Value;

}
//newAngle è quello calcolato con le accelerazioni, newRate è la nuova velocità angolare presa(l'angolo è calcolato nel corpodella funzione, e loop time è il tempo da cui è iniziato il ciclo
//(il tempo di campionamento)
float kalmanCalculate(float newAngle, float newRate, int looptime) {

  dt = float(looptime) / 1000000;
  x_angle += dt * (newRate - x_bias);
  P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
  P_01 +=  - dt * P_11;
  P_10 +=  - dt * P_11;
  P_11 +=  + Q_gyro * dt;

  y = newAngle - x_angle;
  S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;

  x_angle +=  K_0 * y;
  x_bias  +=  K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;

  return x_angle;
}
float kalmanCalculateY(float newAngle, float newRate, int looptime) {

  dtY = float(looptime) / 1000000;
  x_angleY += dtY * (newRate - x_biasY);
  P_00Y +=  - dtY * (P_10Y + P_01Y) + Q_angleY * dtY;
  P_01Y +=  - dtY * P_11Y;
  P_10Y +=  - dtY * P_11Y;
  P_11Y +=  + Q_gyroY * dtY;

  yY = newAngle - x_angleY;
  SY = P_00Y + R_angleY;
  K_0Y = P_00Y / SY;
  K_1Y = P_10Y / SY;

  x_angleY +=  K_0Y * yY;
  x_biasY  +=  K_1Y * yY;
  P_00Y -= K_0Y * P_00Y;
  P_01Y -= K_0Y * P_01Y;
  P_10Y -= K_1Y * P_00Y;
  P_11Y -= K_1Y * P_01Y;

  return x_angleY;
}

float rotate(){}


