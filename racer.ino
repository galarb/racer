// ***** RACER ROBOT SOFTWARE ******* //
//          BY: Gal Arbel 2023
#include "car.h"
#include "clicli.h"
#include "camtrack.h"
double timeout = 400; //camera button activation default
bool togsw = false;
// Voltage sensing Pin: A3.
// Reflectedligh Pin: A2.
// Huskey camera on I2C.  
// neopixels reserved pin 6.
// RX, TX pins for HC-05/HC-06 :8, 9 
Car mycar(
  10, //servoPin
  2, //encoderPin
  3, //Button Pin
  11, //Ena on L298N
  4, //in1
  7, //in2
  80); //wheel diameter
clicli mycli(mycar);
//interrupt setup for button
void buttonPressed(){
  Serial.println("Camera Tracking ON");
  delay(100);
  togsw = true;   
  //while(1){car.run(1, 0, 0);}
}

void setup() {
  mycar.begin(115200);  
  //interrupt requests setup:
  pinMode(3, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(3),buttonPressed,FALLING); //pin3, on press

}

void loop() {
 //mycli.run();
 if(togsw){
  mycar.lcdswitch(true);
  for(int i = 0; i < timeout; i++){mycar.run(1,0,0);}
  mycar.steer(0);
  mycar.stop();
  togsw = false;
    }
 delay(10);

 mycar.checkbatlevel();
 //mycar.bt();
 //mycar.goencoder(1000, 100, 1, 0, 0);
 //Serial.println(mycar.getrefligh());
 //mycar.trackline(100, 34, 3, 0, 0); //speed, color, PID. color measurements revealed range between 20 and 40. 
}
