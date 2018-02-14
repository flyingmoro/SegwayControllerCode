#include <NewPing.h>

//% Stephan - Hier ist Link: https://www.youtube.com/watch?v=6F1B_N6LuKw

#define Trig_Pin_1 50
#define Trig_Pin_2 48
#define Echo_Pin_1 50
#define Echo_Pin_2 48
#define Max_Distance 400

NewPing sonar1 (Trig_Pin_1, Echo_Pin_1, Max_Distance);
NewPing sonar2 (Trig_Pin_2, Echo_Pin_2, Max_Distance);

float duration1;
float duration2;
float distance1;
float distance2;
float soundsp;
float soundcm;
int interations = 3;

void setup() {
  
 Serial.begin(9600);
}

void loop() {

  

  delay (50);
  duration1 = sonar1.ping_median(interations);
  delay (50);
  duration2 = sonar2.ping_median(interations);
  
  distance1 = (duration1 / 2) * 0.0341;
  distance2 = (duration2 / 2) * 0.0341;

  Serial.print("US1 = ");
  Serial.print(distance1);
  Serial.print("cm; ");
  Serial.print("US2 = ");
  Serial.print(distance2);
  Serial.print("cm;");
  Serial.println();
}
