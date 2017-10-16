// ======        PROGRAMMCODE SELBSTBALANCIERENDER ROBOTER        ====== //

//=== DEFINITION DER SCHALTER PINS ===//
#define Schalter1 36        // Schalter 1 an Pin 36
#define Schalter2 37        // Schalter 2 an Pin 37
#define ZyklusZeitPin 39

//=== DEFINITION DER MOTOR-ENCODER PINS ===//
#define EncoderPinA1 18       // Gelbes Kabel Motor1 in 18
#define EncoderPinB1 28       // Weißes Kabel Motor1 in 28
#define EncoderPinA2 19       // Gelbes Kabel Motor 2 in 19
#define EncoderPinB2 30       // Weißes Kabel Motor 2 in 30

//=== DEFINITIONEN DER REGLERPARAMETER ===//
#define KP1 4                 // Proportionalfaktor des Kipp-Reglers
#define Nm2Speed 190          // Umrechungsfaktor um von Nm auf Speedwert zu kommen
#define KD 0                  // Differentialfaktor des Kipp-Reglers
#define Tv 0.08               // Vorhaltzeit D-Anteil Kippregler
#define KI 0.0                // Integralfaktor
#define Tn 0.001              // Nachstellzeit I-Anteil Kippregler
#define IntegratorMax 0.5     // Grenze des Integralanteils
#define KP2 0.0               // Proportionalfaktor des Geschwindigeitsreglers
#define RO 0                  // PWM Offset zur Überwindung der Reibung in Motor und Getriebe
#define WinkelSoll 0          // Roboter soll immer aufrecht stehen
#define Balancepunkt 0.018    // Der gemessene Winkel in aufrechter Position
#define LimitAn  0.2
#define LimitAus 0.8

//=== DEFINITIONEN ULTRASCHALLSENSOR ===//
#define TriggerPin  26  // Ultraschall Trigger-Pin in 26
#define EchoPin     27  // Ultraschall Echo-Pin in 27
#define DistanzMax 400  // Maximale Distanz (in cm) auf die gepingt wird. Maximale Sensordistanz etwa bei 400-500cm.

//=== EINBINDUNG DER BIBLIOTHEKEN ===//
#include <DualVNH5019MotorShield.h>     // Bibliothek für den Motorcontroller
#include <MPU6050_6Axis_MotionApps20.h> // Bibliothek für Bewegungssensor MPU6050
#include <I2Cdev.h>                     // Bibliothek für Bussystem des MPU6050
#include <helper_3dmath.h>              // Bibliothek für mathematische Berechnungen des MPU6050
#include <NewPing.h>                    // Bibliothek für den Ultraschallsensor
MPU6050 mpu;                            // Teilte der Bibliothek mit welcher Sensor verwendet wird
DualVNH5019MotorShield md;              // Zugriff auf den Motorcontroller über md
NewPing sonar(TriggerPin, EchoPin, DistanzMax); // NewPing-Setup der Pins und der maximalen Distanz.

//=== MÖGLICHE UMSETZUNG DES GESCHWINDIGKEITSPROFIL (wird aktuell nicht verwendet)
float vVerlaufZeit[]={0, 3.0};    // Zeit-Vektor Geschwindigkeitsprofil
float vVerlaufGeschw[]={0, 4.0};  // Geschwindigkeits-Vektor Geschwindigkeitsprofil
float vVerlaufAnzahl = 2;         // Anzahl an Stützstellen des Geschwindigkeitsprofils

//=== GLOBALE VARIABLEN ZUM AUSLESEN VON STATUS & SENSORDATEN MIT MPU6050_6Axis_MotionApps20.h ===//
#define InterruptPin 3  // PIN 20 wird als Interrupt-Pin definiert
bool dmpReady = false;  // Wir auf true gesetzt, wenn die Initialisierung des DMP erfolgreich war
uint8_t mpuIntStatus;   // Besitzt das eigentliche Unterbrechungsstatusbyte des MPU
uint8_t devStatus;      // Gibt nach jeder Deviceoperation deren Status zurück (0 = success, !0 = error)
uint16_t PacketSize;    // Erwartete Größe der ausgelesenen Datenpakete (Standardwert ist 42 Bytes)
int SPEED;              // Variable für die Ansteuerung der Motoren

//=== GLOBALE VARIABLEN ENCODER ===//
volatile long EncoderPos1 = 0; // Zählvariable des Encoders 1 (LINKS)
volatile long EncoderPos2 = 0; // Zählvariable des Encoders 2 (RECHTS)

//=== INTERRUPT DETECTION ROUTINE DES BEWEGUNGSSENSORS ===//
volatile bool mpuInterrupt = false;    // Stellt fest, ob der MPU Interrupt-Pin auf 'HIGH' gesetzt wurde
void dmpDataReady()
{
    mpuInterrupt = true;
}

//=== FUNKTION MOTOR-ENCODER 1 (LINKS) ===//
void doEncoderA1()
{
  if (SPEED > 0)
  {
    if (digitalRead(EncoderPinA1) == HIGH)    // Falls an Kanal A low-to-high festgestellt wird
    {         // An Kanal B wird gecheckt in welche Richtung er dreht. Encoder dreht um
      if (digitalRead(EncoderPinB1) == LOW) EncoderPos1 = EncoderPos1 - 1;   // Gegen den Uhrzeigersinn
      else EncoderPos1 = EncoderPos1 + 1;     // Im Uhrzeigersinn
    }
    else      // Falls an Kanal A ein high-to-low festgestellt wird
    {
      if (digitalRead(EncoderPinB1) == LOW) EncoderPos1 = EncoderPos1 + 1;   // Im Uhrzeugersinn
      else EncoderPos1 = EncoderPos1 - 1;     // Gegen den Uhrzeigersinn
    }
  }
  else    // Falls Motor anders herum dreht
  {
    if (digitalRead(EncoderPinA1) == HIGH)    // low-to-high an Kanal A
    {
      if (digitalRead(EncoderPinB1) == LOW) EncoderPos1 = EncoderPos1 + 1;   // Gegen den Uhrzeigersinn
      else EncoderPos1 = EncoderPos1 - 1;     // Im Uhrzeigersinn
    }
    else     // high-to-low an Kanal A
    {
      if (digitalRead(EncoderPinB1) == LOW) EncoderPos1 = EncoderPos1 - 1;   // Im Uhrzeigersinn
      else EncoderPos1 = EncoderPos1 + 1;     // Gegen den Uhrzeigersinn
    }
  }// end if-Schleife
}// end doEncoderA1

//=== FUNKTION MOTOR-ENCODER 2 (RECHTS) ===//
void doEncoderA2()
{
  if (SPEED < 0)
  {
    if (digitalRead(EncoderPinA2) == HIGH)    // Falls an Kanal A low-to-high festgestellt wird
    {    // An Kanal B wird gecheckt in welche Richtung er dreht. Encoder dreht um
      if (digitalRead(EncoderPinB2) == LOW) EncoderPos2 = EncoderPos2 - 1;   // Gegen den Uhrzeigersinn
      else EncoderPos2 = EncoderPos2 + 1;     // Im Uhrzeigersinn
    }
    else      // Falls an Kanal A ein high-to-low festgestellt wird
    {
      if (digitalRead(EncoderPinB2) == LOW) EncoderPos2 = EncoderPos2 + 1;   // Im Uhrzeigersinn
      else EncoderPos2 = EncoderPos2 - 1;     // Gegen den Uhrzeigersinn
    }
  }
  else
  {
    if (digitalRead(EncoderPinA2) == HIGH)    // low-to-high an Kanal A
    {
      if (digitalRead(EncoderPinB2) == LOW) EncoderPos2 = EncoderPos2 + 1;   // Gegen den Uhrzeigersinn
      else EncoderPos2 = EncoderPos2 - 1;     // Im Uhrzeigersinn
    }
    else     // high-to-low an Kanal A
    {
      if (digitalRead(EncoderPinB2) == LOW) EncoderPos2 = EncoderPos2 - 1;    // Im Uhrzeigersinn
      else EncoderPos2 = EncoderPos2 + 1;     // Gegen den Uhrzeigersinn
     }
  }// end if-Schleife
}// end doEncoderA2

//=== FUNKTION MOTORFEHLER ===//
void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 Fehler");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 Fehler");
    while(1);
  }
}// end stopIfFault

// ================        SETUP - FUNKTION        ================ //
// ================        SETUP - FUNKTION        ================ //

void setup()
{
  Serial.begin(115200);            // Initialisierung der seriellen Schnittstelle zur Kommunikation mit PC
  Wire.begin();
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();                 // Initialisierung des Bewegungssensors
  Serial.println(F("ZyklusZeitPining device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();  // Laden und Konfigurieren des DMP

// Sicherstellung, dass Initialisierung funktioniert hat (Gibt in dem Fall 0 zurück)
  if (devStatus == 0)
  {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);        // Schaltet den DMP an, da er jetzt bereit ist
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(InterruptPin), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();  // Einschalten der Interrupt Detection
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;  // Setzt die DMP-Ready-Flagge, damit void loop() weiß, dass der DMP benutzt werden kann
    PacketSize = mpu.dmpGetFIFOPacketSize();  // Die Größe der Datenpakete wird ermittelt
  }
  else // FEHLER!
  {
    // 1 = Fehler beim Laden des Speichers
    // 2 = Fehler beim Update der DMP Konfiguration
    // (Falls die Schleife abbricht wird der Code in der Regel "1" sein)
    Serial.print(F("DMP Initialisierung fehlgeschlagen (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }// end if-Schleife zur Initialisierung des Bewegungssensors

  // Einstellen der Offsetwerte aufgrund Produktionssteuungen. Gemessen über einen separaten Programmcode
  mpu.setXGyroOffset(124);
  mpu.setYGyroOffset(40);
  mpu.setZGyroOffset(-54);
  mpu.setZAccelOffset(1504);

// INITIALISIERUNG ENCODER 1 (LINKS)
  pinMode (EncoderPinA1,INPUT); digitalWrite (EncoderPinA1,HIGH);
  pinMode (EncoderPinB1,INPUT); digitalWrite (EncoderPinB1,HIGH);
  attachInterrupt(5, doEncoderA1, CHANGE);

// INITIALISIERUNG ENCODER 2 (RECHTS)
  pinMode (EncoderPinA2,INPUT); digitalWrite (EncoderPinA2,HIGH);
  pinMode (EncoderPinB2,INPUT); digitalWrite (EncoderPinB2,HIGH);
  attachInterrupt(4, doEncoderA2, CHANGE);

  md.init(); // INITIALISIERUNG MOTOREN

  //== INITIALISIERUNG SCHALTER
  pinMode(Schalter1, INPUT);
  pinMode(Schalter2, INPUT);

}// end void setup()

// ============== HAUPTPROGRAMMSCHLEIFE =============//
// ============== HAUPTPROGRAMMSCHLEIFE =============//

void loop()
{
//=== DEFINITION LOKALER VARIABLEN ===//

  // Regelung
  float WinkelIst;            // Kippwinkel in rad
  float WinkelGrad;           // Kippwinkel in Grad
  float WinkelDifferenz;      // Regeldifferenz Kippregler
  float WinkelGeschw;
  float GeschwSoll;           // Muss eingegeben werden (Wert in rad/s erforderlich)
  float GeschwIst;            // Bekommen wir über die Encoder
  float GeschwDifferenz;      // Geschwindigkeitsdifferenz (Regeldiff. V-Regler)
  float MomentTeil1;          // Ergebnis Kippregler
  float MomentTeil2;          // Ergebnis Geschwindigkeitsregler
  float Integrator = 0;
  static int SPEEDAlt;
  static int Umgefallen;      // Markierung wird gesetzt wenn der Roboter umfällt

  // Encoder
  long PositionNeu1;              // Variable für aktuellen Positionswert Encoder 1
  long PositionNeu2;              // Variable für aktuellen Positionswert Encoder 2
  static long PositionAlt1 = 0;   // Variable für alten Positionswert Encoder 1
  static long PositionAlt2 = 0;   // Variable für alten Positionswert Encoder 2
  unsigned long ZeitNeu1;         // Variable für aktuelle Zeit (ms) Encoder 1
  unsigned long ZeitNeu2;         // Variable für aktuelle Zeit (ms) Encoder 2
  static unsigned long ZeitAlt1 = 0; // Variable für alte Zeit (ms) Encoder 1
  static unsigned long ZeitAlt2 = 0; // Variable für alte Zeit (ms) Encoder 1
  float Impulse1;                 // gezählte Impulse pro Zeitabschnitt
  float Impulse2;                 // gezählte Impulse pro Zeitabschnitt
  float UmdrehungenSek1;          // Umdrehungen pro Sekunde der Motorabtriebswelle
  float UmdrehungenSek2;          // Umdrehungen pro Sekunde der Motorabtriebswelle
  float UmdrehungenMin1;          // Umdrehungen pro Minute der Motorabtriebswelle
  float UmdrehungenMin2;          // Umdrehungen pro Minute der Motorabtriebswelle
  float dt;                       // jeweilige Zeitdifferenz

  // Schalter
  int SchalterZustand1 = 0;           // Variable zum Lesen des Zustands Schalter 1
  int SchalterZustand2 = 0;           // Variable zum Lesen des Zustands Schalter 2
  static unsigned int ZustandAlt = 0; // Um zu dedektieren, dass gerade der Schalter geändert wurde
  unsigned int Zustand = 0;           // Um zu dedektieren, dass gerade der Schalter geändert wurde
  int PinZustand = 0;                 // Toggelnder Pin

  // Geschwindigkeitsprofil
  static unsigned long ZeitStartZustand2; // Zeit beim Starten des Geschwindigkeitsprofils
  float t;                                // Zeit, die seit Starten des Geschwindigkeitsprofils vergangen ist

  // Bewegungssensor
  uint8_t FifoBuffer[64];   // Speicherpuffer für den FIFO Speicher
  uint16_t FifoCount;       // Anzahl aller Bytes, die aktuell im First-In-First-Out Speicher sind
  unsigned int NumGyroData; // Variable zur Überprüfung der Vollständigkeit eines Datenpakets
  float NumGyroDataFloat;   // Variable zur Überprüfung der Vollständigkeit eines Datenpakets
  unsigned int k;           // Zählvariable
  float YawPitchRoll[3];    // [z, y, x] Vector enthält Winkel: Gieren um Z, Kippen um Y, Rollen um X
  int16_t Gyros[3];         // [x, y, z] Vektor enthält WinkelGeschwen um X, Y, Z
  Quaternion q;             // [w, x, y, z] Enthält die Quaternionen
  VectorFloat gravity;      // [x, y, z] Vector zur Kompensation der Gravitation

  // Ultraschallsensor
  unsigned long MillisAktuell = 0;  // Vergangene ms seit Start des Programms
  unsigned long MillisAlt = 0;
  int ZeitIntervall = 30;           // Zeitabschnitt nach dem gepingt wird darf nicht kleiner als 29ms sein
  float Abstand;                    // Abstand des Ultraschallsensors zu einem Gegenstand

//=== TOGGELNDER PIN ===//

  PinZustand = digitalRead(ZyklusZeitPin);
  if (PinZustand == LOW) digitalWrite(ZyklusZeitPin, HIGH);
  else digitalWrite(ZyklusZeitPin, LOW);

//=== ZUSTAND DES SCHALTERS ===//

  SchalterZustand1 = digitalRead(Schalter1); // Liest den Zustand des Pins aus
  SchalterZustand2 = digitalRead(Schalter2);
  if (SchalterZustand1 == HIGH) Zustand = 1;
  else if (SchalterZustand2 == HIGH) Zustand = 2;
  else Zustand = 0;

  switch(Zustand) // Weißt GeschwSoll je nach Schalterstellung den passenden Wert zu.
  {
  case 1:
    GeschwSoll = 0;
    break; // case Zustand == 1

  case 2:         // Geschwindigkeitsprofil ODER Mittels Vektoren arbeiten (siehe Bsp. ganz oben)
    if (ZustandAlt!=2) ZeitStartZustand2 = millis();
    t = (float)(millis() - ZeitStartZustand2) / (float)1000; // float-Division
    if (t <= 1.5) GeschwSoll = 0;
    else if (t > 3 && t + 6) GeschwSoll = 0.5;
    else GeschwSoll = 0;
    break;  // case Zustand == 2
  } // end switch Zustand

//=== AUSLESEN DER SENSORDATEN & LADEN IN FIFO-PUFFER ===//

  if (!dmpReady) return;              // Sollte der DMP nicht bereit sein, soll er nichts machen
  mpuInterrupt = false;               // Zurücksetzen der Interrupt-Flagge
  mpuIntStatus = mpu.getIntStatus();  // Gibt Statusbyte des Sensors
  FifoCount = mpu.getFIFOCount();     // Anzahl der Werte, die im FIFO Speicher abgelegt werden

  if((mpuIntStatus & 0x10) || FifoCount >= 1024) //0x10 bedeutet 16 als Hexadezimalzahl, & bedeutet bitweise logisches UND
  {
    mpu.resetFIFO();                  // FIFO-Speicher (Zwischenspeicher) leeren
    Serial.println("FIFO OVERFLOW");
  }
  else if(mpuIntStatus & 0x02)        // Status in Ordnung // 0x02 bedeutet 2 als Hexadezimalzahl
  {
    NumGyroDataFloat = (float)FifoCount / (float)PacketSize;
    NumGyroData = FifoCount / PacketSize;
    if (abs(NumGyroDataFloat-(float)NumGyroData)<0.001) // nimmt nur Datenpakete, wenn der FIFO vollständige Datenpakete besitzt
    {
      for (k = 0; k < NumGyroData; k++) mpu.getFIFOBytes(FifoBuffer, PacketSize); // Das aktuellste Datenpaket wird in den Puffer geladen
    }
    else Serial.println(NumGyroDataFloat);
  }// end FIFO-IF-Schleife

  mpu.resetFIFO();                      // FIFO-Speicher leeren

  mpu.dmpGetQuaternion(&q, FifoBuffer); // Die Daten aus dem Puffer werden in die Bewegungsvektoren geschrieben
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(YawPitchRoll, &q, &gravity);
  mpu.dmpGetGyro(Gyros, FifoBuffer);

//===       ENCODER      ===//

  // ENCODER 1 (LINKS)
  PositionNeu1 = EncoderPos1;
  ZeitNeu1 = millis(); // Zeit wird in Millisekunden mitgezählt
  dt = (float)(ZeitNeu1-ZeitAlt1)/1000.0; // Zeitdiffenrenz; 1000: Umrechnung in Sekunden
  Impulse1 = (float)(PositionNeu1-PositionAlt1)/dt; // Anzahl Impulse = Wegunterschied / Zeitunterschied
  PositionAlt1 = PositionNeu1;
  ZeitAlt1 = ZeitNeu1;
  UmdrehungenSek1 = Impulse1/64.0/30.0*2.0;
  // Umrechnung der Impulse in Umdrehungen pro Sekunde
  // Faktor 64: Eine Umdrehung entspricht 64 Impulsen des Sensors
  // Faktor 30: Getriebeübersetzung des Motors
  // Faktor 2: Da nur Pin A des Encoders verwendet wird --> nur halbe Auflösung
  // Grund: Der frei werdende Interrupt-Pin am Arduino wird benötigt
  UmdrehungenMin1 = UmdrehungenSek1*60; // Umrechnen der Geschwindigkeit in Umdrehungen pro Minute

  // ENCODER 2 (RECHTS)
  PositionNeu2 = EncoderPos2;
  ZeitNeu2 = millis();
  dt = (float)(ZeitNeu2-ZeitAlt2)/1000.0;
  Impulse2 = (float)(PositionNeu2-PositionAlt2)/dt;
  PositionAlt2 = PositionNeu2;
  ZeitAlt2 = ZeitNeu2;
  UmdrehungenSek2 = Impulse2/64.0/30.0*2.0;
  UmdrehungenMin2 = UmdrehungenSek2*60;

  GeschwIst = (UmdrehungenSek1 + UmdrehungenSek2)*M_PI; // Eigentlich (UmdrehungenSek1 + UmdrehungenSek2)/2*2*M_PI
  if (abs(GeschwIst>1)) GeschwIst=(abs(GeschwIst)-1.0)*(SPEEDAlt>0 ? 1.0:-1.0);
  else GeschwIst = 0;

//=== ULTRASCHALLSENSOR ===//

  MillisAktuell = millis();
  if (MillisAktuell - MillisAlt >= ZeitIntervall)
  {
    // Wartet 50ms zwischen Pings (ca. 20 Pings/sek). 29ms sollte die kürzeste Zeit zwischen 2 Pings sein.
    MillisAlt = MillisAktuell;
    float uS = sonar.ping(); // Ping wird gesendet und Pingzeit in µS(uS) kommt zurück.
    Abstand = uS / US_ROUNDTRIP_CM; // Abstand in cm
  }
  if (Abstand < 5 && Abstand > 0) GeschwSoll = 0; // Lässt den Roboter stoppen, wenn dieser sich einem Hindernis nähert.

// === KASKADENREGELUNG & ANSTEUERUNG DER MOTOREN === //

  WinkelIst = YawPitchRoll[1] - Balancepunkt; // Kompensation der Nulllage
  WinkelGrad = WinkelIst*180.0/M_PI;          // Umrechnung des Kippwinkels von Radiant in Grad (dient nur dem besseren Verständnis)
  WinkelDifferenz = WinkelIst - WinkelSoll;   // Regeldifferenz Kippwinkel
  WinkelGeschw = (float)Gyros[1] * (-1.0) /180.0*M_PI; // Winkelgeschwindigkeit: Drehrate kommt mit falschem Vorzeichen und in °/s an

  Integrator = constrain (Integrator + WinkelDifferenz, -IntegratorMax, IntegratorMax); // I-Anteil

  // Kippregelung
  MomentTeil1 = KP1 * (WinkelDifferenz + Tv * WinkelGeschw) + KI * Integrator ;  // PID-Regler gemischte Schreibweise
  //MomentTeil1 = KP1 * (WinkelDifferenz + 1/Tn * Integrator + Tv * WinkelGeschw); // PID-Regler mit Tv und Tn
  //MomentTeil1 = KP1 * WinkelDifferenz + KI * Integrator + KD * WinkelGeschw;     // PID-Regler mit KI und KD

  GeschwDifferenz = GeschwSoll - GeschwIst;     // Regeldifferenz Geschwindigkeit

  // Geschwindigkeitsregelung
  MomentTeil2 = KP2 * GeschwDifferenz;          // P-Regler

  SPEED = Nm2Speed*(MomentTeil1 + MomentTeil2); // Berechnung des Speed-Wertes
  if (SPEED>0) SPEED+=RO;                       // Innere Reibung der Motoren und Getriebe kann kompensiert werden
  else if (SPEED<0) SPEED-=RO;
  SPEED = constrain (SPEED, -400, 400);         // Die Geschwindigkeit wird auf -400 bzw. 400 begrenzt

  md.setM1Speed(SPEED);                         // Ansteuerung der Motoren
  md.setM2Speed(SPEED);
  stopIfFault();                                // Fragt den Fehler-Pin des Motorcontrollers ab
  SPEEDAlt=SPEED;

  // Fällt der Roboter um, werden die Motoren gestoppt. Ist der Roboter aufgerichtet, werden die Motoren angesteuert.
  if (WinkelIst > LimitAus || WinkelIst < -LimitAus) Umgefallen = 1; //entspricht ca. 45 Grad
  if (Umgefallen && WinkelIst < LimitAn && WinkelIst > -LimitAn) Umgefallen = 0; //entspricht ca. 20 Grad
  if (Umgefallen)
  {
    md.setM1Brake(400);                         // Motoren werden sofort gestoppt
    md.setM2Brake(400);
    SPEEDAlt = 0;
    Integrator = 0;                             // Damit der Integrator einen frischen Wert hat
    Serial.println("Ich bin Umgefallen! Richte mich bitte wieder auf");
  }// end Umgefallen

//=== SERIELLE AUSGABEN BEI SCHALTER O ===//

  if (SchalterZustand1 == LOW && SchalterZustand2 == LOW) // SCHALTERSTELLUNG auf O
  {
    //Serial.print(YawPitchRoll[1],4);  Serial.print("\t");
    //Serial.print(WinkelIst,4);        Serial.print("\t");
    //Serial.print(WinkelGrad,2);       Serial.print("\t");
    Serial.print(WinkelDifferenz,4);  Serial.print("\t");
    Serial.print(WinkelGeschw,4);     Serial.print("\t");
    Serial.print(MomentTeil1);        Serial.print("\t");
    Serial.print(MomentTeil2);        Serial.print("\t");
    Serial.print(SPEED);              Serial.print("\t");
    Serial.print(PositionNeu1);       Serial.print("\t");
    //Serial.print(Impulse1);           Serial.print("\t");
    //Serial.print(UmdrehungenSek1);    Serial.print("\t");
    //Serial.print(UmdrehungenMin1);    Serial.print("\t");
    Serial.print(PositionNeu2);       Serial.print("\t");
    //Serial.print(Impulse2);           Serial.print("\t");
    //Serial.print(UmdrehungenSek2);    Serial.print("\t");
    //Serial.print(UmdrehungenMin2);    Serial.print("\t");
    Serial.print(GeschwIst);          Serial.print("\t");
    //Serial.print(Abstand); Serial.print("\t");//0 = Außerhalb Reichweite, kein Ping-Echo
    Serial.println(millis());
  }// end serial prints

  ZustandAlt = Zustand;

}// end void loop()
