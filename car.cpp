#include "HardwareSerial.h"
#include "car.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include "camtrack.h"
#include <Adafruit_NeoPixel.h>

#define NEOPIXpin 6
#define NUMPIXELS 8 
#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

Servo SteeringServo; 
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXpin, NEO_GRB + NEO_KHZ800);

Car::Car(int servoPin, int encoderPin, int Ena, int in1, int in2, int wheelsize) {
 
  _servoPin = servoPin;
  _encoderPin = encoderPin;
  _Ena = Ena;
  _in1 = in1;
  _in2 = in2;
  _wheelsize = wheelsize;  
  
  pinMode(_encoderPin, INPUT_PULLUP); 
  pinMode(_Ena, OUTPUT); 
  pinMode(_in1, OUTPUT);
  pinMode(_in2, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

}

void Car::begin(double bdrate) {
  delay(30);
  Serial.begin(bdrate);      
  Serial.println("Started");
  Serial.print("in1 = ");  
  Serial.println(_in1);
  Serial.print("in2 = ");  
  Serial.println(_in2);
  Serial.print("enA = ");  
  Serial.println(_Ena);
  Serial.print("Encoder Pin = ");  
  Serial.println(_encoderPin);
  Serial.print("Steering Servo Pin = ");  
  Serial.println(_servoPin);
  Serial.println("Reflected light Pin = A1");
  Serial.println("Neopixels Pin = 6");  
  
  SteeringServo.attach(_servoPin);
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear();
  pixels.setBrightness(100);
  delay(30);

}

double Car::PIDcalc(double inp, int sp){
    Serial.println(steps);

   currentTime = millis();                //get current time
   elapsedTime = (double)(currentTime - previousTime)/1000; //compute time elapsed from previous computation (60ms approx). divide in 1000 to get in Sec
   //Serial.print(currentTime); //for serial plotter
   //Serial.println("\t"); //for serial plotter
   error = sp - inp;                                  // determine error
   cumError += error * elapsedTime;                   // compute integral
   rateError = (error - lastError)/elapsedTime;       // compute derivative deltaError/deltaTime
   if(rateError > 0.3 || rateError < -0.3){cumError = 0;}             // reset the Integral commulator when Proportional is doing the work

   double out = kp*error + ki*cumError + kd*rateError; //PID output               
   //Serial.println(cumError);
   lastError = error;                                 //remember current error
   previousTime = currentTime;                        //remember current time
   if(out > 254){out = 254;}    //limit the function for smoother operation
   if(out < -254){out = -254;}
   if(cumError > 255 || cumError < -255){cumError = 0; out = 0;} // reset the Integral commulator
   return out;                                        //the function returns the PID output value 
  
}
long Car::getSteps(){
  if(digitalRead(_encoderPin)){ //1 = obstruction, 0 = hole
     steps = steps +1; 
   }
  return steps;
}

long Car::goencoder(int clicks, int times, double KP, double KI, double KD){
  Serial.println(steps);

  kp = KP;
  ki = KI; 
  kd = KD;
  steps = 0; //reset the steps count
  for(int i = 0; i < times; i++) {
    int tempsteps = getSteps(); //input value
    int output = PIDcalc(tempsteps, clicks); //output value = the calculated error
    //lcdenshow(clicks, output, tempsteps);
    delay(30);
    if (output > 0){  // 
     move(output);
    }
    if (output < 0){  // 
     stop();
    } 
  }
 stop(); //reset the motor after moving
 return(steps * ((PI * _wheelsize) / 20));

}

void Car::move(int speed){
  if(speed >= 0 && speed < 255){
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
    analogWrite(_Ena, speed);
  }
  else if(speed < 0 && speed > -255){
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
    analogWrite(_Ena, abs(speed));
  }
  else{Serial.println("Wrong motor speed value");}
  
}

void Car::stop(){
  digitalWrite(_in1, HIGH);
  digitalWrite(_in2, HIGH);
  analogWrite(_Ena, 0);
}  

void Car::steer(int dir){
 int tempdir = map(dir, -100, 100, 10, 170); //90 is the absolute zero of my car
 SteeringServo.write(tempdir); 
 pixshow(dir);
}

long Car::getrefligh(){
  int x = analogRead(A1);
  return(x);
}
void Car::trackline(int speed, int color, double KP, double KI, double KD){
  kp = KP;
  ki = KI; 
  kd = KD;
  int tempcolor = getrefligh(); //input value
  int output = PIDcalc(tempcolor, color); //output value = the calculated error
  output = map(output, 0, 100, -100, 100);
  if(output > 100){output = 100;}
  if(output < -100){output = -100;}
  if(speed > 255){speed = 255;}
  if(speed < 0){speed =0;}
  //move(speed);
  steer(output);
}
void Car::gomm(long distancemm){ //covert to encoder clicks
  //one click is perimeter divided by 20 holes (in my encoder wheel)
  long clicks = distancemm / ((PI * _wheelsize) / 20);
  Serial.println(goencoder(clicks, 1000, 3, 2, 0));
}

void Car::pix(int red, int green, int blue){
 for(int i = 0; i < NUMPIXELS; i++){
   pixels.setPixelColor(i, red, green, blue);
   pixels.show();
 }
}
void Car::pixshow(int dir){
  pixels.clear();
  switch (dir) {
    case -100 ... -67: //from center to third left led 
       for(int i = 4; i > -1; i--){
        pixels.setPixelColor(i, 255, 0, 0);
        pixels.show();
       }
     break;
    case -66 ... -33: //from center to seconnd left led 
       for(int i = 4; i > 0; i--){
        pixels.setPixelColor(i, 255, 0, 0);
        pixels.show();
       }
     break;
    case -32 ... -1: 
       for(int i = 4; i > 1; i--){
        pixels.setPixelColor(i, 255, 0, 0);
        pixels.show();
       }
     break;
    default: //two center leds (dir = 0)
     for(int i = 3; i < 5; i++){
        pixels.setPixelColor(i, 255, 0, 0);
        pixels.show();
       }
     break;
    case 1 ... 33: 
       for(int i = 3; i < 6; i++){
        pixels.setPixelColor(i, 255, 0, 0);
        pixels.show();
       }
     break;
    case 34 ... 66: 
       for(int i = 3; i < 7; i++){
        pixels.setPixelColor(i, 255, 0, 0);
        pixels.show();
       }
     break;
    case 67 ... 100: 
       for(int i = 3; i < 8; i++){
        pixels.setPixelColor(i, 255, 0, 0);
        pixels.show();
       }
     break;

  }
}
