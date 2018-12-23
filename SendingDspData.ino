//
// Librerie
//
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
//
// Variabili
//
const int acceso = 3;
MPU6050 mpu;
double tc = 1000.00; //tempo di campionamento
int numeroTest = 10; //registreremo 100 file da 5000 campioni relativi le accellerazioni e 100 file
int n = 0; //numero campioni
int contaPerSpegnere_ = 0;
double offset_x = 0, offset_y = 0, offset_z = 0;//offset delle accelerazioni
double velx = 0, vely = 0, velz = 0; //velocità
double spostx = 0, sposty = 0, spostz = 0; //spostamento
float ypr[3]; //vettore yaw pitch e roll per calcolo rotazioni
double acx, acy, acz; //accelerazioni
int timer; //conto del tempo
//variabili funzionamento mpu
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
// Scelgo cosa mandare su seriale: Accelerazione (1) , Velocità(2), spostamento(3), angoli(4).
int setDati = 4;
//#define Numero_campioni se vuoi inviare a seriale il numero del campione togli questo commento
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;
bool lavora = true;
void dmpDataReady() {
  mpuInterrupt = true;
}
int i = 0, j = 0;
//
// Fine Variabili
//
//
// Setup
//
void setup() {
  Wire.begin();
  Wire.setClock(1000); // 1kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(115200);
mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
  }
  calcolaOffset(offset_x, offset_y, offset_z);
  timer = micros();
}
//
// Fine setup
//
//
// Loop
//
void loop() {
  
  stampaSeriale();
  attendiFinoTc();
}
//
// Fine loop
//
//
// Funzioni
//
void attendiFinoTc() {
  // Faccio in modo che il ciclo duri sempre il tempo di campionamento
  // Ricontrollando quanto tempo è passato ogni microsecondo fino a che non è passato un intervallo di campionamento intero
  n++;
  if (n == 5000) {
    timer = micros();
    n = 0;
  }
  while (micros() - timer < (n) * tc) {
    delayMicroseconds(1);
  }
}
//
// Funzione per il calcolo degi integrali
//
void integrale( double S, double tc, double * I ) {
  *I += S * (tc / 1000000.);

}
//
// Funzione per il calcolo degli offset
//
void calcolaOffset( double offset_x, double offset_y, double offset_z) {
  timer = micros();

  for (int i = 0; i < 1200; i++) {

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
    }
    else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    }
    if (i >= 200) {
      offset_x += (double(aaReal.x) / 16384.00000 ) * 9.80665;
      offset_y += (double(aaReal.y) / 16384.00000 ) * 9.80665;
      offset_z += (double(aaReal.z) / 16384.00000 ) * 9.80665;
    }
    while (micros() - timer < (i) * tc) {}
  }
  offset_x = offset_x / 1000. ;
  offset_y = offset_y / 1000. ;
  offset_z = offset_z / 1000. ;
}
//
// Funzione che aplica le matrici di rotazione ai dati in ingresso considerando come angoli di rotazionele angolazioni Yaw Pitch Roll del sensore
//
void rotate(double * x, double * y, double * z) {
  double x1, x2, x3, y1, y2, y3, z1, z2, z3;
  // Ruoto roll
  x1 = *x;
  y1 = *y * cos(ypr[2]) - *z * sin(ypr[2]);
  z1 = *y * sin(ypr[2]) + *z * cos(ypr[2]);
  // Ruoto pitch
  x2 = x1 * cos(ypr[1]) + z1 * sin(ypr[1]);
  y2 = y1;
  z2 = z1 * cos(ypr[1]) - x1 * sin(ypr[1]);
  // Ruoto yaw
  x3 = x2 * cos(ypr[0]) - y2 * sin(ypr[0]);
  y3 = x2 * sin(ypr[0]) + y2 * cos(ypr[0]);
  z3 = z2;
  *x = x3;
  *y = y3;
  *z = z3;
}
//
// Funzione che gestisce la stampa sulla porta seriale del set di dati indicato
//
void stampaSeriale() {
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();
  }
  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  }
  switch (setDati) {
    case 4:
      Serial.print((ypr[0] ), 5);
      Serial.print('\t');
      Serial.print(-(ypr[1] ), 5);
      Serial.print('\t');
      Serial.println(-(ypr[2] ), 5);
      break;
    case 1 :
      acx = (double(aaReal.x ) / 16384.00000 ) * 9.80665 - offset_x;
      acy = (double(aaReal.y ) / 16384.00000 ) * 9.80665 - offset_y ;
      acz = (double(aaReal.z ) / 16384.00000 ) * 9.80665 - offset_z;
      rotate(&acx, &acy, &acz);
      Serial.print(acx, 5);
      Serial.print('\t');
      Serial.print(acy, 5);
      Serial.print('\t');;
      Serial.println(acz, 5);
      break;
    case 2 :
      acx = (double(aaReal.x ) / 16384.00000 ) * 9.80665 - offset_x;
      acy = (double(aaReal.y ) / 16384.00000 ) * 9.80665 - offset_y ;
      acz = (double(aaReal.z ) / 16384.00000 ) * 9.80665 - offset_z;
      rotate(&acx, &acy, &acz);
      integrale(acx , tc, &velx);
      integrale(acy , tc, &vely);
      integrale(acz, tc, &velz);
      Serial.print(velx, 5);
      Serial.print(',');
      Serial.print(vely, 5);
      Serial.print('_');;
      Serial.println(velz, 5);
      break;
    case 3 :
      acx = (double(aaReal.x ) / 16384.00000 ) * 9.80665 - offset_x;
      acy = (double(aaReal.y ) / 16384.00000 ) * 9.80665 - offset_y ;
      acz = (double(aaReal.z ) / 16384.00000 ) * 9.80665 - offset_z;
      rotate(&acx, &acy, &acz);
      integrale(acx, tc, &velx);
      integrale(acy, tc, &vely);
      integrale(acz, tc, &velz);
      integrale(velx, tc, &spostx);
      integrale(vely, tc, &sposty);
      integrale(velz, tc, &spostz);
      Serial.print(spostx, 5);
      Serial.print(',');
      Serial.print(sposty, 5);
      Serial.print('_');;
      Serial.println(spostz, 5);
      break;
  }
}

void setupMpu() {
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
  }
  calcolaOffset(offset_x, offset_y, offset_z);
  timer = micros();
}
//
// Fine
//
