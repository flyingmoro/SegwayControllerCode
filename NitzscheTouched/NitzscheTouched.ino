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

#define ENC1A 18       // Gelbes Kabel Motor1 in 18
#define ENC1B 28       // Weißes Kabel Motor1 in 28
#define ENC2A 19       // Gelbes Kabel Motor 2 in 19
#define ENC2B 30       // Weißes Kabel Motor 2 in 30

typedef struct {
    float x;
    float y;
    float gamma;
} WorldPosition;

// dont use this directly, instead use the function getWorldPosition()
volatile WorldPosition ISR_world_position = { 0, 0, 0 };


void deadReckonWheelEncoders(int);
WorldPosition worldPosition();

#define LEFT_WHEEL_FORWARDS 0
#define LEFT_WHEEL_BACKWARDS 1
#define RIGHT_WHEEL_FORWARDS 2
#define RIGHT_WHEEL_BACKWARDS 3

// fixed size deltas for dead reckoning world position
#define DELTA_X 0.00016362f
#define DELTA_Y 0.00000014511f
#define DELTA_GAMMA 0.0018f
#define SPECIAL_TWO_PI 6.283186f
#define R_RAD 0.05f  // Radradius in m

// Zählvariable des Encoders 1 (links in Fahrtrichtung gesehen)
volatile long EncoderPos1 = 0;

// Zählvariable des Encoders 2 (rechts in Fahrtrichtung gesehen)
volatile long EncoderPos2 = 0;

//Incremente pro Umdrehung
#define REV2INC 960.0



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


#define PI_HALBE 1.570796
#define PI_ 3.1415927

float phiR, phi, phiP;
int16_t ax, ay, az, px, py, pz;


MPU6050 mpu;
DualVNH5019MotorShield md;              // Zugriff auf den Motorcontroller über md

#define TESTPIN_ENC_1 25
#define TESTPIN_ENC_2 26

//960 Inc per rev
void doENC1A() {
  digitalWrite(TESTPIN_ENC_1, HIGH);
  static int a, b;
  b = digitalRead(ENC1B);//read B first, since A just changed and will change again only after B has changed
  a = digitalRead(ENC1A);
  if (a == HIGH) {
    if (b == HIGH){
      EncoderPos1++;
      deadReckonWheelEncoders(LEFT_WHEEL_FORWARDS);
      digitalWrite(TESTPIN_ENC_1, LOW);
    }
    else {
      EncoderPos1--;
      deadReckonWheelEncoders(LEFT_WHEEL_BACKWARDS);
      digitalWrite(TESTPIN_ENC_1, LOW);
    }
  }
  else {
    if (b == HIGH) {
      EncoderPos1--;
      deadReckonWheelEncoders(LEFT_WHEEL_BACKWARDS);
      digitalWrite(TESTPIN_ENC_1, LOW);
    }
    else {
      EncoderPos1++;
      deadReckonWheelEncoders(LEFT_WHEEL_FORWARDS);
      digitalWrite(TESTPIN_ENC_1, LOW);
    }
  }
}

void doENC2A() {
  digitalWrite(TESTPIN_ENC_2, HIGH);
  static int a, b;
  b = digitalRead(ENC2B);//read B first, since A just changed and will change again only after B has changed
  a = digitalRead(ENC2A);
  if (a == HIGH) {
    if (b == HIGH) {
      EncoderPos2--;
      deadReckonWheelEncoders(RIGHT_WHEEL_BACKWARDS);
      digitalWrite(TESTPIN_ENC_2, LOW);
    }
    else {
      EncoderPos2++;
      deadReckonWheelEncoders(RIGHT_WHEEL_FORWARDS);
      digitalWrite(TESTPIN_ENC_2, LOW);
    }
  }
  else {
    if (b == HIGH) {
      EncoderPos2++;
      deadReckonWheelEncoders(RIGHT_WHEEL_FORWARDS);
      digitalWrite(TESTPIN_ENC_2, LOW);
    }
    else {
      EncoderPos2--;
      deadReckonWheelEncoders(RIGHT_WHEEL_BACKWARDS);
      digitalWrite(TESTPIN_ENC_2, LOW);
    }
  }
}

float oldPhi = 0.0f;
void deadReckonWheelEncoders(int direction) {
    float tiltCorrection = (phi - oldPhi) * R_RAD;
    oldPhi = phi;

    float gammaOld = ISR_world_position.gamma;

    switch (direction) {
        case LEFT_WHEEL_FORWARDS:
            ISR_world_position.x += cos(gammaOld) * (DELTA_X - tiltCorrection);
            ISR_world_position.y += sin(gammaOld) * (DELTA_X - tiltCorrection);
            ISR_world_position.gamma -= DELTA_GAMMA;
            break;
        case LEFT_WHEEL_BACKWARDS:
            ISR_world_position.x -= cos(gammaOld) * (DELTA_X + tiltCorrection);
            ISR_world_position.y -= sin(gammaOld) * (DELTA_X + tiltCorrection);
            ISR_world_position.gamma += DELTA_GAMMA;
            break;
        case RIGHT_WHEEL_FORWARDS:
            ISR_world_position.x += cos(gammaOld) * (DELTA_X - tiltCorrection);
            ISR_world_position.y += sin(gammaOld) * (DELTA_X - tiltCorrection);
            ISR_world_position.gamma += DELTA_GAMMA;
            break;
        case RIGHT_WHEEL_BACKWARDS:
            ISR_world_position.x -= cos(gammaOld) * (DELTA_X + tiltCorrection);
            ISR_world_position.y -= sin(gammaOld) * (DELTA_X + tiltCorrection);
            ISR_world_position.gamma -= DELTA_GAMMA;
            break;
    }

    // constrain gamma
    while (ISR_world_position.gamma >= SPECIAL_TWO_PI) {
      ISR_world_position.gamma -= SPECIAL_TWO_PI;
    }
    while (ISR_world_position.gamma <= 0.0f) {
      ISR_world_position.gamma += SPECIAL_TWO_PI;
    }
}

// void deadReckonWheelRightForwards(){
//   long long tiltCorrection = KIPPWINKEL * R_RAD;
//   long long fullDeltaX = DELTA_X + tiltCorrection;
//   float gammaOld = (float)worldPosition().gamma / 1000000.0f;
//   ISR_world_position.x += cos(gammaOld) * fullDeltaX - sin(gammaOld) * DELTA_Y;
//   ISR_world_position.y += sin(gammaOld) * fullDeltaX + cos(gammaOld) * DELTA_Y;
//   ISR_world_position.gamma += DELTA_GAMMA;
//   constrainGamma();
// }
//
// void deadReckonWheelRightBackwards(){
//   float gammaOld = (float)worldPosition().gamma / 1000000.0f;
//   ISR_world_position.x -= cos(gammaOld) * DELTA_X - sin(gammaOld) * DELTA_Y;
//   ISR_world_position.y -= sin(gammaOld) * DELTA_X + cos(gammaOld) * DELTA_Y;
//   ISR_world_position.gamma -= DELTA_GAMMA;
//   constrainGamma();
// }
//
// void deadReckonWheelLeftForwards(){
//   float gammaOld = (float)worldPosition().gamma / 1000000.0f;
//   ISR_world_position.x += cos(gammaOld) * DELTA_X - sin(gammaOld) * DELTA_Y;
//   ISR_world_position.y += sin(gammaOld) * DELTA_X + cos(gammaOld) * DELTA_Y;
//   ISR_world_position.gamma -= DELTA_GAMMA;
//   constrainGamma();
// }
//
// void deadReckonWheelLeftBackwards(){
//   float gammaOld = (float)worldPosition().gamma / 1000000.0f;
//   ISR_world_position.x -= cos(gammaOld) * DELTA_X - sin(gammaOld) * DELTA_Y;
//   ISR_world_position.y -= sin(gammaOld) * DELTA_X + cos(gammaOld) * DELTA_Y;
//   ISR_world_position.gamma += DELTA_GAMMA;
//   constrainGamma();
// }

WorldPosition worldPosition() {
    WorldPosition wp;
    cli();
    wp.x = ISR_world_position.x;
    wp.y = ISR_world_position.y;
    wp.gamma = ISR_world_position.gamma;
    sei();
    return wp;
}

void setupENC() {
  pinMode (ENC1A, INPUT); digitalWrite (ENC1A, HIGH);
  pinMode (ENC1B, INPUT); digitalWrite (ENC1B, HIGH);
  //  attachInterrupt(5, doEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1A), doENC1A, CHANGE);

  pinMode (ENC2A, INPUT); digitalWrite (ENC2A, HIGH);
  pinMode (ENC2B, INPUT); digitalWrite (ENC2B, HIGH);
  //  attachInterrupt(4, doEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2A), doENC2A, CHANGE);
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
  pinMode(TESTPIN_ENC_1, OUTPUT);
  pinMode(TESTPIN_ENC_2, OUTPUT);

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



void getMotion(float *phi, float *a, float *phiP, int *mm) {
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

  if (last < 20000) {
    last = now;
    phiF = phiA;
    return;
  }
  *mm = (now - last);
  h = (float)(*mm) * 1.0e-6;
  last = now;


  phiPS = (float)py / 5000.0 - OFFSET_PHIP;

  phiF = ((phiF + h * phiPS) * T_MOTION + phiA * h) / (T_MOTION + h);

  cli();
  *phi = phiA;
  *a = phiA;
  *phiP = phiPS;
  sei();
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

void control(int active) {

  int mm;
  float u; //Reglerausgang
  int SPEEDL, SPEEDR, SPEED;
  static long enc1=0;
  static long enc2=0;

  float v1,v2;

  int modk;
  float ud=0;

  SPEEDL = SPEEDR = 0;

  getMotion(&phi, &phiR, &phiP, &mm);
  getVelo(&v1,&v2);

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
}

void stopMotors() {
  md.setM1Speed(0);
  md.setM2Speed(0);
}

unsigned long loopStart = 0;
void loop() {
  loopStart = micros();

  int sw;
  sw = read_SW();
  control(sw == 0);


  // Ausgabe der Position im Welt-KS
  mrWorldX = worldPosition().x;
  mrWorldY = worldPosition().y;
  mrWorldGamma = worldPosition().gamma * 180.0f / PI;

  // Ausgabe aller Werte des Bewegungssensors
  mrAlphaDeriv = px;
  mrBetaDeriv = py;
  mrGammaDeriv = pz;
  mrAX = ax;
  mrAY = ay;
  mrAZ = az;
  mrGamma = phi;

  microRayCommunicate();
}
