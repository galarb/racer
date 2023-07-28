#include "car.h"
#include "clicli.h"
#include "camtrack.h"

// Reflectedligh Pin on A1.
// Huskey camera on I2C.  
// neopixels reserved pin 6.
Car mycar(
  5, //servoPin
  7, //encoderPin
  3, //Ena on L298N
  4, //in1
  2, //in2
  80); //wheel diameter
clicli mycli(mycar);
Camtrack mycam(mycar);

void setup() {
  mycar.begin(115200);  
  mycam.begin(200);//sets the default tracking speed (0-255)
}

void loop() {
 mycli.run();
 mycam.run(1, 0, 0);
 delay(100);
 // mycar.steer(mycam.run(1, 0, 0));
 //mycar.goencoder(100, 100, 1, 0, 0);

 //Serial.println(mycar.getrefligh());
 //mycar.trackline(100, 34, 3, 0, 0); //speed, color, PID. color measurements revealed range between 20 and 40. 
}
