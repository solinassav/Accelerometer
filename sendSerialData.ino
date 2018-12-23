//
// Librerie
//
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
//
// Prototipi funzioni
//
void attendiFinoTc();
void calcolaOffset( double *offset_x, double *offset_y, double *offset_z);
void stampaSeriale(int setData);
void aggiornaN(int *n, int *setDati, bool *Lavora);
//
// Variabili
//
MPU6050 mpu;
double tc = 1000.00; //tempo di campionamento
int n = 0; //numero campioni
double offset_x = 0, offset_y = 0, offset_z = 0;//offset delle accelerazioni
double velx = 0, vely = 0, velz = 0; //velocitÃ 
double spostx = 0, sposty = 0, spostz = 0; //spostamento
float ypr[3]; //vettore yaw pitch e roll per calcolo rotazioni
double acx, acy, acz; //accelerazioni
int timer; //conto del tempo
// Variabili funzionamento mpu
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
// #define Numero_campioni//se deccomenti questo define ogni volta che scrivi su seriale la prima colonna contiene il numero di campioni
// Scelgo cosa mandare su seriale: Accelerazione (1) , VelocitÃ (2), spostamento(3), angoli(4).
int setDati = 4;//inizio inviando le accelerazioni
//#define Numero_campioni se vuoi inviare a seriale il numero del campione togli questo commento
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}
//
// Fine Variabili
//
//
// Setup
//
void setup() {
  // Inizializzo connessione I2C e seriale
  Wire.begin();
  Wire.setClock(1000); // 1kHz I2C clock.
  Serial.begin(115200);
  // Setup MPU6050
  setupMpu();
  //Calcolo offset e salvo il tempo iniziale del loop
  calcolaOffset();
  timer = micros();
}
//
// Fine setup
//
//
// Loop
//
void loop() {
  // Stampo i dati richiesti, aggiorno il conto dei campioni e aspetto fino al prossimo istante di campionamento
  stampaSeriale(setDati);
  aggiornaN(n);
  attendiFinoTc();

}
//
// Fine loop
//
//
// Funzioni
//
void attendiFinoTc() {
  //faccio in modo che il ciclo duri sempre il tempo di campionamento
  //ricontrollando quanto tempo Ã¨ passato ogni microsecondo fino a che non Ã¨ passato un intervallo di campionamento intero
  while (micros() - timer < (n) * tc) {
    delayMicroseconds(1);
  }
}
//
// Funzione per il calcolo degi integrali
//
void integrale( double S, double Tc, double *I ) {
  *I += S * (tc / 1000000.);

}
//
// Funzione per il calcolo degli offset
//
void calcolaOffset() {
  timer = micros();

  for (int i = 0; i < 1000; i++) {

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
    
      offset_x += (double(aaReal.x) / 16384.00000 ) * 9.80665;
      offset_y += (double(aaReal.y) / 16384.00000 ) * 9.80665;
      offset_z += (double(aaReal.z) / 16384.00000 ) * 9.80665;
    
    while (micros() - timer < (i) * tc) {}
  }
  offset_x = offset_x / 1000. ;
  offset_y = offset_y / 1000. ;
  offset_z = offset_z / 1000. ;
}
//
// Funzione che aplica le matrici di rotazione ai dati in ingresso considerando come angoli di rotazionele angolazioni Yaw Pitch Roll del sensore
//
void rotate(double *x, double *y, double *z) {
  double x1, x2, x3, y1, y2, y3, z1, z2, z3;
  //ruoto roll
  x1 = *x;
  y1 = *y * cos(ypr[2]) - *z * sin(ypr[2]);
  z1 = *y * sin(ypr[2]) + *z * cos(ypr[2]);
  //ruoto pitch
  x2 = x1 * cos(ypr[1]) + z1 * sin(ypr[1]);
  y2 = y1;
  z2 = z1 * cos(ypr[1]) - x1 * sin(ypr[1]);
  //ruoto yaw
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
void stampaSeriale(int setData) {
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
  switch (setData) {
    case 4:
      Serial.print((ypr[0] ), 5);
      Serial.print(',');
      Serial.print(-(ypr[1] ), 5);
      Serial.print('_');
      Serial.print(-(ypr[2] ), 5);
      Serial.print('\n');
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
      integrale(acx , tc, &velx);
      integrale(acy , tc, &vely);
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
//
// Funzione aggiornaN
//

void aggiornaN(int n) {
#ifdef Numero_campioni
  Serial.print(n);
  Serial.print('\t');
#endif
  n++;
  /*Per evitare che n diventi troppo grande a fine test riinizializzo il conto dei campioni,
    tanto poi per contare i campioni ci basta effettivamente contarli guardando la lunghezza del vettore dove li salviamo,
    n serve solo per regolare i tempo di campionamento*/
  if (n == 10 * 5000 - 1) {
    timer = micros();
    n = 0;
  }
}
//
// Fine
//
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
}


