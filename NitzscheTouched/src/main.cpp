#include <Arduino.h>
#include "microRay.h"

// use your gains from WOK
#define KP 10.0
#define TV 0.005



// 1000ms default read timeout (modify with "I2Cdev::readTimeout = [ms];")

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




typedef struct WorldPosition {
    float x;
    float y;
    float gamma;
} WorldPosition;



WorldPosition worldPosition = { 0, 0, 0 };

void deadReckonWheelEncodersWithFloats();

int encoderChangeLeft();
int encoderChangeRight();



#define METERS_PER_ENCODER_STEP 0.000327f
#define NEG_PI -3.141  // in milli pi
#define TWO_PI 6.282  // in milli pi
#define R_RAD 0.050  // Radradius in metern
#define WHEEL_SPAN 0.184  // Radabstand in metern

// Zählvariable des Encoders 1 (links in Fahrtrichtung gesehen)
volatile int8_t ISR_EncoderPos1 = 0;
// Zählvariable des Encoders 2 (rechts in Fahrtrichtung gesehen)
volatile int8_t ISR_EncoderPos2 = 0;

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

float phiR = 0.0f, phi = 0.0f, phiP = 0.0f;
int16_t ax = 0, ay = 0, az = 0, px = 0, py = 0, pz = 0;


MPU6050 mpu;
DualVNH5019MotorShield md;              // Zugriff auf den Motorcontroller über md

#define TESTPIN_ENC_1 39
#define TESTPIN_ENC_2 41

//960 Inc per rev
void doENC1A() {
  digitalWrite(TESTPIN_ENC_1, HIGH);
  static uint8_t a, b, ab;
  b = digitalRead(ENC1B);//read B first, since A just changed and will change again only after B has changed
  a = digitalRead(ENC1A);

  if (a == HIGH) {
    if (b == HIGH){
      ISR_EncoderPos1++;
      // deadReckonWheelEncoders(LEFT_WHEEL_FORWARDS);
    }
    else {
      ISR_EncoderPos1--;
      // deadReckonWheelEncoders(LEFT_WHEEL_BACKWARDS);
    }
  }
  else {
    if (b == HIGH) {
      ISR_EncoderPos1--;
      // deadReckonWheelEncoders(LEFT_WHEEL_BACKWARDS);
    }
    else {
      ISR_EncoderPos1++;
      // deadReckonWheelEncoders(LEFT_WHEEL_FORWARDS);
    }
  }
  digitalWrite(TESTPIN_ENC_1, LOW);
}

void doENC2A() {

  digitalWrite(TESTPIN_ENC_2, HIGH);

  static int a, b;
  b = digitalRead(ENC2B);//read B first, since A just changed and will change again only after B has changed
  a = digitalRead(ENC2A);
  if (a == HIGH) {
    if (b == HIGH) {
      ISR_EncoderPos2--;
      // deadReckonWheelEncoders(RIGHT_WHEEL_BACKWARDS);
    }
    else {
      ISR_EncoderPos2++;
      // deadReckonWheelEncoders(RIGHT_WHEEL_FORWARDS);
    }
  }
  else {
    if (b == HIGH) {
      ISR_EncoderPos2++;
      // deadReckonWheelEncoders(RIGHT_WHEEL_FORWARDS);
    }
    else {
      ISR_EncoderPos2--;
      // deadReckonWheelEncoders(RIGHT_WHEEL_BACKWARDS);
    }
  }

  digitalWrite(TESTPIN_ENC_2, LOW);
}



void deadReckonWheelEncodersWithFloats() {
    static float xRLeft, xRRight, diffX, deltaForward, deltaGamma;

    // misleading, change functions encoderChange, because EncoderCounter will be reset inside
    xRLeft = encoderChangeLeft() * (float)METERS_PER_ENCODER_STEP;
    xRRight = encoderChangeRight() * (float)METERS_PER_ENCODER_STEP;

    diffX = (float)(xRRight - xRLeft);
    deltaGamma = diffX / WHEEL_SPAN;
    deltaForward = (xRLeft + xRRight) / 2.0;

    worldPosition.x += (cos(worldPosition.gamma) * deltaForward);
    worldPosition.y += (sin(worldPosition.gamma) * deltaForward);
    worldPosition.gamma += deltaGamma;

    // constrain gamma
    while (worldPosition.gamma > PI) {
      worldPosition.gamma -= TWO_PI;
    }
    while (worldPosition.gamma <= NEG_PI) {
      worldPosition.gamma += TWO_PI;
    }
}



int encoderChangeLeft() {
    static int8_t tempEncLeft;
    cli();
    tempEncLeft = ISR_EncoderPos1;
    ISR_EncoderPos1 = 0;
    sei();
    return tempEncLeft;
}

int encoderChangeRight() {
    static int8_t tempEncRight;
    cli();
    tempEncRight = ISR_EncoderPos2;
    ISR_EncoderPos2 = 0;
    sei();
    return tempEncRight;
}

void setupENC() {
  pinMode (ENC1A, INPUT);
  digitalWrite (ENC1A, HIGH);
  pinMode (ENC1B, INPUT);
  digitalWrite (ENC1B, HIGH);
  //  attachInterrupt(5, doEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1A), doENC1A, CHANGE);

  pinMode (ENC2A, INPUT);
  digitalWrite (ENC2A, HIGH);
  pinMode (ENC2B, INPUT);
  digitalWrite (ENC2B, HIGH);
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


  // md.setM1Speed(100);
  // md.setM2Speed(100);
  // delay(5000);

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
  *phi = phiF;
  *a = phiA;
  *phiP = phiPS;
  sei();
  mrBetaPS = phiPS;
  mrBetaFiltered = phiF;
}

void getVelo(float *v1, float *v2){
  static int first=1;
  static long enc1_last=0;
  static long enc2_last=0;
  long enc1 = 0, enc2 = 0;
  static float v1_last=0.0;
  static float v2_last=0.0;

  static unsigned long last = 0;
  unsigned long now = 0;

  if (first){
    *v1=0;
    *v2=0;
    first = 0;
  }
  else {
    now=micros();
    enc1=ISR_EncoderPos1;
    enc2=ISR_EncoderPos2;
    *v1 = ((float)(enc1-enc1_last)/(float)(now-last)*1000.0 + 5.0*v1_last)/6.0;
    *v2 = ((float)(enc2-enc2_last)/(float)(now-last)*1000.0 + 5.0*v2_last)/6.0;

    v1_last = *v1;
    v2_last = *v2;
    last = now;
    enc1_last=enc1;
    enc2_last=enc2;
  }
}

void control() {

  int mm;
  float u; //Reglerausgang
  int SPEEDL = 0, SPEEDR = 0, SPEED = 0;
  // static long enc1=0;
  // static long enc2=0;

  float v1 = 0.0f, v2 = 0.0f;

  // int modk;
  float ud=0.0f;

  getMotion(&phi, &phiR, &phiP, &mm);
  getVelo(&v1,&v2);

  if (abs(phi) < MAX_PHI) {
    u = -(phi + phiP * TV) * KP          +ud+KV*(v1+v2)/2.0;
    mrE = (float)u;
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
  else {
      mrE = 0.0f;
      SPEEDL = 0;
      SPEEDR = 0;
  }
  md.setM1Speed(SPEEDL);
  md.setM2Speed(SPEEDR);
  mrSpeed = SPEED;

}

void stopMotors() {
  md.setM1Speed(0);
  md.setM2Speed(0);
}

unsigned long loopStart = 0;
void loop() {
  loopStart = micros();
  deadReckonWheelEncodersWithFloats();
  control();


  // Ausgabe der Position im Welt-KS
  // mrWorldX = (float)worldPosition.x / 1000000.0f;
  // mrWorldY = (float)worldPosition.y / 1000000.0f;
  // mrWorldGamma = (float)worldPosition.gamma * 180.0f / PI / 1000.0f;
  mrWorldX = worldPosition.x * 1000.0f; // should give us millimeters
  mrWorldY = worldPosition.y * 1000.0f; // should give us millimeters
  mrWorldGamma = worldPosition.gamma * 180.0f / PI;

  // Ausgabe aller Werte des Bewegungssensors
  mrAlphaDeriv = px;
  mrBetaDeriv = py;
  mrGammaDeriv = pz;
  mrAX = ax;
  mrAY = ay;
  mrAZ = az;
  mrBetaA = phi;

  // Serial.println("duda");

  microRayCommunicate();
}
