#include <Arduino.h>

int count = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(13,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (count < 3)
  {
    /* code */
  
  
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13,LOW);
    delay(500);
    count++;
  }
  else
  {
    digitalWrite(13,LOW);
    delay(2000);
    count = 0;
  }
  


}