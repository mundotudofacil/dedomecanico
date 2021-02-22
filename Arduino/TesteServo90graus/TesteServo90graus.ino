// MundoTudoFacil.com
// Pompílio Araújo Jr.

#include "Servo.h"

Servo ser; 
int ang = 100;
int limiteS = 180;
int limiteI = 100;
int iAlt = 1;
bool bDirecao = 1;

void setup()
{
  Serial.begin(9600);
  ser.attach(3); // Pino digital D3
  ser.write(ang);
}
 
void loop()
{
  ser.write(ang); 
  Serial.println(ang);

  if (bDirecao){
    ang = ang + iAlt;
    if (ang > limiteS) bDirecao = false;
  }
  else{
    ang = ang - iAlt;
    if (ang < limiteI) bDirecao = true;
  }
  delay(20);
}
