#include <Arduino.h>>

void setup(){
    pinMode(13,OUTPUT);
}
enum State{blink, off};
State LED_state = blink;
int prevmillis = 0;
int blinkCount = 0;
const long On = 500;
const long Off = 2000;

void loop(){
  switch (LED_state)
  {
  case blink:
    /* code */
    if (millis()-prevmillis >= On){
        prevmillis = millis();
      if (digitalRead(13) == HIGH)
      {
        digitalWrite(13,LOW);
        blinkCount++;
      }else{
        digitalWrite(13,HIGH);
      }

      if (blinkCount == 3)
      {
        LED_state = off;
        digitalWrite(13,LOW);
        prevmillis = millis();
        blinkCount =0;
      }
      
    }
    break;
  
  case off:
    if (millis()-prevmillis >=Off){
        LED_state = blink;
        prevmillis = millis();
      }
    break;
  }
}