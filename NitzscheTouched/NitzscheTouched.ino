#include "microRay.h"

// use your gains from WOK
#define KP 10.0
#define TV 0.005


// DON'T TOUCH

#include <math.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "DualVNH5019MotorShield.h"     // Bibliothek für den Motorcontroller


#define SW1 36        // Schalter 1 an Pin 36
#define SW2 37        // Schalter 2 an Pin 37

#define ENCA1 18       // Gelbes Kabel Motor1 in 18
#define ENCB1 28       // Weißes Kabel Motor1 in 28
#define ENCA2 19       // Gelbes Kabel Motor 2 in 19
#define ENCB2 30       // Weißes Kabel Motor 2 in 30



void oneStepForwardLeftTurn();
void oneStepForwardRightTurn();
void oneStepBackwardLeftTurn();
void oneStepBackwardRightTurn();


// Zählvariable des Encoders 1 (LINKS) -> in Fahrtrichtung oder wie?
volatile long EncoderPos1 = 0; 

// Zählvariable des Encoders 2 (RECHTS)
volatile long EncoderPos2 = 0; 

//Incremente pro Umdrehung
#define REV2INC 960.0 

// Position des Roboters im Welt-KS inkl Drehung um die Hochachse
// Berechnungen nur ganzzahlig, deshalb diese Einheiten
volatile long long worldX = 0; // in nm  !! NANO
volatile long long worldY = 0; // in nm  !! NANO
volatile long worldGamma = 0;   // in urad !! micro

// fixed size deltas
#define DELTA_X 127000
#define DELTA_Y 114
#define DELTA_GAMMA 1700
#define SPECIAL_PI 3141593   // micro PI


#define T_MOTION 3.0

#define DB(x) //Serial.println(x)
//#define DB(x)


#define OFFSET_PHI 0.01 //-0.015
#define OFFSET_PHIP -0.03

#define NM2SPEED 200.0
#define KV 0.0
#define ROL 0 // Friction Compensation Left
#define ROR 5 // Friction Compensation Right
#define MAX_PHI 0.35 // Switch off if phi exceeds this value

#define LBUF 1

#define PI_HALBE 1.570796
#define PI_ 3.1415927

unsigned int _time[LBUF];
int _phi[LBUF];
int _a[LBUF];
int _phiP[LBUF];


MPU6050 mpu;
DualVNH5019MotorShield md;              // Zugriff auf den Motorcontroller über md

//960 Inc per rev
void doENCA1() {
  static int a, b;
  b = digitalRead(ENCB1);//read B first, since A just changed and will change again only after B has changed
  a = digitalRead(ENCA1);
  if (a == HIGH) {
    if (b == HIGH){
      EncoderPos1++;
      deadReckonWheelLeftForwards();
    }
    else {
      EncoderPos1--;
      deadReckonWheelLeftBackwards();
    }
  }
  else {
    if (b == HIGH) {
      EncoderPos1--;
      deadReckonWheelLeftBackwards();
    }
    else {
      EncoderPos1++;
      deadReckonWheelLeftForwards();
    }
  }
}

void doENCA2() {
  static int a, b;
  b = digitalRead(ENCB2);//read B first, since A just changed and will change again only after B has changed
  a = digitalRead(ENCA2);
  if (a == HIGH) {
    if (b == HIGH) {
      EncoderPos2--;
      deadReckonWheelRightForwards();
    }
    else {
      EncoderPos2++;
      deadReckonWheelRightBackwards();
    }
  }
  else {
    if (b == HIGH) {
      EncoderPos2++;
      deadReckonWheelRightBackwards();
    }
    else {
      EncoderPos2--;
      deadReckonWheelRightForwards();
    }
  }
}

void deadReckonWheelRightForwards(){
  worldX += cos(worldGamma) * DELTA_X - sin(worldGamma) * DELTA_Y;
  worldY += sin(worldGamma) * DELTA_X + cos(worldGamma) * DELTA_Y;
  worldGamma += DELTA_GAMMA;
  
}

void deadReckonWheelRightBackwards(){
  worldX -= cos(worldGamma) * DELTA_X - sin(worldGamma) * DELTA_Y;
  worldY += sin(worldGamma) * DELTA_X + cos(worldGamma) * DELTA_Y;
  worldGamma -= DELTA_GAMMA;
}

void deadReckonWheelLeftForwards(){
  worldX += cos(worldGamma) * DELTA_X - sin(worldGamma) * DELTA_Y;
  worldY -= sin(worldGamma) * DELTA_X + cos(worldGamma) * DELTA_Y;
  worldGamma -= DELTA_GAMMA;
}

void deadReckonWheelLeftBackwards(){
  worldX -= cos(worldGamma) * DELTA_X - sin(worldGamma) * DELTA_Y;
  worldY -= sin(worldGamma) * DELTA_X + cos(worldGamma) * DELTA_Y;
  worldGamma += DELTA_GAMMA;
}

void correctGamma(){
  if (worldGamma >= SPECIAL_PI) {
    worldGamma -= SPECIAL_PI;
  }
  if (worldGamma <= 0) {
    worldGamma += SPECIAL_PI;
  }
}

void setupENC() {
  pinMode (ENCA1, INPUT); digitalWrite (ENCA1, HIGH);
  pinMode (ENCB1, INPUT); digitalWrite (ENCB1, HIGH);
  //  attachInterrupt(5, doEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA1), doENCA1, CHANGE);

  pinMode (ENCA2, INPUT); digitalWrite (ENCA2, HIGH);
  pinMode (ENCB2, INPUT); digitalWrite (ENCB2, HIGH);
  //  attachInterrupt(4, doEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2), doENCA2, CHANGE);
}

void setupMPU() {
  Wire.begin();
  mpu.initialize();
}




void setup() {
  ////Serial.begin(115200);            // Initialisierung der seriellen Schnittstelle zur Kommunikation mit PC

  microRayInit();

  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);

  setupMPU();
  md.init();

  setupENC();
}

int read_SW() {
  static int sw_ = 0;
  static int swState = -1;
  static unsigned long nextSwitchMillis;
  int s1, s2, sw;

  s1 = digitalRead(SW1);
  s2 = digitalRead(SW2);
  sw = s1 == HIGH ? 1 : s2 == HIGH ? 2 : 0;

  if (swState == -1) {
    swState = sw;
  }
  else {
  }
  if (sw != sw_) {
    nextSwitchMillis = millis() + 200;
  }
  if ((millis() > nextSwitchMillis) || (swState == -1)) swState = sw;
  sw_ = sw;
  return swState;
}


void dumpSingleBuffer(int idx) {
  //Serial.print(idx); //Serial.print("\t");
  //Serial.print(_time[idx]); //Serial.print("\t");
  //Serial.print(_a[idx]); //Serial.print("\t");
  //Serial.print(_phi[idx]); //Serial.print("\t");
  //Serial.print(_phiP[idx]); //Serial.print("\t");
  //Serial.print("\r\n");

}

void dumpBuffer(unsigned int k) {
  unsigned int idx;
  //Serial.println("------------------------------------------------");
  for (idx = k; idx < LBUF; idx++) {
    dumpSingleBuffer(idx);
  }
  for (idx = 0; idx < k; idx++) {
    dumpSingleBuffer(idx);
  }
}

int16_t ax, ay, az, px, py, pz;
void getMotion(float *phi, float *a, float *phiP, int *mm, int reset) {
  static unsigned long last = 0;
  unsigned long now;
  float h;
  float ax2, az2;
  static float phiF = 0; // Phi gefiltert (Beobachter)
  float phiA;  //Phi aus Beschleunigungen
  float phiPS; //phiP vom Sensor

  mpu.getMotion6(&ax, &ay, &az, &px, &py, &pz);
  now = micros();

  ax2 = (float)ax;
  az2 = (float)az;
  phiA = atan2(az2, ax2) - PI_HALBE - OFFSET_PHI;
  
  if ((last < 20000) || reset) {
    last = now;
    phiF = phiA;
    return;
  }
  *mm = (now - last);
  h = (float)(*mm) * 1.0e-6;
  last = now;


  phiPS = (float)py / 5000.0 - OFFSET_PHIP;

  phiF = ((phiF + h * phiPS) * T_MOTION + phiA * h) / (T_MOTION + h);

  *phi = phiF;
  *a = phiA;
  *phiP = phiPS;
}

void getVelo(float *v1, float *v2){
  static int first=1;
  static long enc1_last=0;
  static long enc2_last=0;
  long enc1,enc2;
  static float v1_last=0.0;
  static float v2_last=0.0;

  static unsigned long last = 0;
  unsigned long now;

  if (first){
    *v1=0;*v2=0;
    first = 0;
  }
  else {
    now=micros();
    enc1=EncoderPos1;
    enc2=EncoderPos2;
    *v1 = ((float)(enc1-enc1_last)/(float)(now-last)*1000.0 + 5.0*v1_last)/6.0;
    *v2 = ((float)(enc2-enc2_last)/(float)(now-last)*1000.0 + 5.0*v2_last)/6.0;

    v1_last = *v1;
    v2_last = *v2;
    last = now;
    enc1_last=enc1;
    enc2_last=enc2;
  }
}

float phiR, phi, phiP;
void control(unsigned int k, int reset, int active) {
  
  int mm;
  float u; //Reglerausgang
  int SPEEDL, SPEEDR, SPEED;
  static long enc1=0;
  static long enc2=0;

  float v1,v2;
  
  int modk;
  float ud=0;

#if 0 //Spruenge auf Moment
  modk = k%100;
  if (modk<20) ud=0.0;
  else if (modk<50) ud=0.5;
  else if (modk<70) ud=0.0;
  else ud=-0.5;
#endif

  SPEEDL = SPEEDR = 0;

  getMotion(&phi, &phiR, &phiP, &mm, reset);
  getVelo(&v1,&v2);
  
#if 0
  _a[k] = (int)(phiR * 10000);
  _phi[k] = (int)(phi * 10000);
  _phiP[k] = (int)(phiP * 10000);
#endif
#if 1
  _a[k] = (int)(v1 *   1000);
  _phi[k] = (int)(phi * 10000);
  _phiP[k] = (int)(v2 *1000);
#endif
  _time[k] = mm;

  if ((abs(phi) < MAX_PHI) && active) {
    u = -(phi + phiP * TV) * KP+ud+KV*(v1+v1)/2.0;
    SPEED = (int) (u * NM2SPEED);

    if (SPEED > 0) {
      SPEEDL = SPEED + ROL;
      SPEEDR = SPEED + ROR;
    }
    else {
      SPEEDL = SPEED - ROL;
      SPEEDR = SPEED - ROR;
    }
  }
  md.setM1Speed(SPEEDL);
  md.setM2Speed(SPEEDR);
#if 0
  //Serial.print(phi); //Serial.print("\t");
  //Serial.print(u); //Serial.print("\t");
  //Serial.print(active); //Serial.print("\t");
  //Serial.print(SPEEDL); //Serial.print("\t");
  //Serial.print(SPEEDR); //Serial.print("\t");
  //Serial.println("");
#endif
}

void stopMotors() {
  md.setM1Speed(0);
  md.setM2Speed(0);
}

void loop() {
  static unsigned int k = 0;
  static unsigned long cnt=0;
  static unsigned long start;
  static int done1 = 0;
  static int done3 = 0;
  int sw;

  cnt++;

  sw = read_SW();
  switch (sw)
  {
    case 1:
      stopMotors();
      done3 = 0;
      if (!done1) {
        dumpBuffer(k);
        done1 = 1;
      }
      break;

    default:
      done1 = 0;

      if (!done3) start = millis();
      //_time[k] = (unsigned int)(millis() - start);
      control(k, !done3, sw == 0);
      if ((cnt&0x1)==0){
        k++;
        if (k == LBUF) k = 0;
      }
      done3 = 1;
      break;
  }

  
  // Ausgabe der Position im Welt-KS
  mrWorldX = ((float)worldX) / 1000000000.0f;
  mrWorldY = ((float)worldX) / 1000000000.0f;
  mrWorldGamma = ((float)worldGamma) / 1000000.0f;

  // Ausgabe aller Werte des Bewegungssensors
  mrAlphaDeriv = px;
  mrBetaDeriv = py;
  mrGammaDeriv = pz;
  mrAX = ax;
  mrAY = ay;
  mrAZ = az;
  
  microRayCommunicate();
}

