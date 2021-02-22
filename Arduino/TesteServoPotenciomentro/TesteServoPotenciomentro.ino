// MundoTudoFacil.com
// Pompílio Araújo Jr.

#include "Servo.h"

Servo ser; 
 
void setup()
{
  Serial.begin(9600);
  ser.attach(3); // Pino digital D3
}
 
void loop()
{
  int ang = analogRead(0); 
  ang = map(ang, 0, 1023, -20, 300);
  ser.write(ang); 
  Serial.println(ang);
  delay(20);
}
