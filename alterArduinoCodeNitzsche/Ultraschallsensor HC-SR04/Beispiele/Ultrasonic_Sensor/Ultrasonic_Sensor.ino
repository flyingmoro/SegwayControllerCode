//www.elegoo.com
//2016.06.13

#include <NewPing.h>

#define TRIGGER_PIN  26  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     27  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define buzzer 31

long MillisAlt = 0;
long ZeitIntervall = 80;
float Abstand;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  Serial.begin(9600); // Open serial monitor at 115200 baud to see ping results.
  pinMode(buzzer,OUTPUT);
}

void loop() {
 
  unsigned long MillisAktuell = millis();
   if (MillisAktuell - MillisAlt > ZeitIntervall)
   {
    MillisAlt = MillisAktuell;
    float uS = sonar.ping(); // Send ping, get ping time in microseconds (uS).
  Abstand = uS / US_ROUNDTRIP_CM;
  Serial.print("Ping: ");
  Serial.print(Abstand); // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
  Serial.println("cm");
  // Wait 500ms between pings (about 2 pings/sec). 29ms should be the shortest delay between pings.
   }
  if (Abstand < 4 && Abstand > 0)
  {

    digitalWrite(buzzer,HIGH);
   }  
  else
  {
    digitalWrite(buzzer,LOW);
    }
  
}
