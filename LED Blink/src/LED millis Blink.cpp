#include <Arduino.h>>

void setup(){
    pinMode(13,OUTPUT);
}

int LED_state = LOW;
int prevmillis = 0;
const long On = 3000;
const long Off = 2000;

void loop(){
  if (LED_state==HIGH)
  {
   if (millis()-prevmillis >= On){
        LED_state = LOW;
        digitalWrite(13,LED_state);
        prevmillis = millis();
    }
  }else{
    if (millis()-prevmillis >=Off){
        LED_state = HIGH;
        digitalWrite(13,LED_state);
        prevmillis = millis();
    }
  }
}