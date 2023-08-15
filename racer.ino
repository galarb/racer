#include "car.h"
#include "clicli.h"
#include "camtrack.h"

// Voltage sensing Pin on A0.
// Reflectedligh Pin on A1.
// Huskey camera on I2C.  
// neopixels reserved pin 6.
// RX, TX pins for HC-05/HC-06 :8, 9 
Car mycar(
  10, //servoPin
  7, //encoderPin...
  3, //Ena on L298N
  4, //in1
  2, //in2
  80); //wheel diameter
clicli mycli(mycar);

void setup() {
  mycar.begin(115200);  
 // mycam.begin(200);//sets the default tracking speed (0-255)

}

void loop() {
 mycli.run();
 delay(200);
 //mycar.checkbatlevel();
 mycar.bt();
 
 //mycar.steer(mycam.run(1, 0, 0));
 //mycar.goencoder(100, 100, 1, 0, 0);

 //Serial.println(mycar.getrefligh());
 //mycar.trackline(100, 34, 3, 0, 0); //speed, color, PID. color measurements revealed range between 20 and 40. 
}
