// MundoTudoFacil.com
// Pompílio Araújo Jr.

#include <Servo.h>

String readString;
Servo s;
int pin13 = 0;
int limValue;

void setup() {
  Serial.begin(9600);
  s.attach(3);
  s.write(20);
}

void loop() {
  while(!Serial.available()) {}
  // serial read section
  while (Serial.available())
  {
    delay(10);
    if (Serial.available() > 0)
    {
      char c = Serial.read(); 
      readString += c; 
    }
  }
  if (pin13 == 0){
    digitalWrite(13, HIGH);
    pin13 = 1;
  }else{
    digitalWrite(13, LOW);
    pin13 = 0;
  }

  //delay(100);
  if (readString.length() > 0)
  {
    //Serial.print("Arduino received: ");  
    Serial.print(readString);
    limValue = max(readString.toInt(), 0);
    limValue = min(limValue, 180);
    s.write(limValue);
    readString="";
  }
  readString="";

  Serial.flush();
  delay(30);
}
