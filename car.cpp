#include "Stream.h"
#include "HardwareSerial.h"
#include "car.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include "camtrack.h"
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>

#define NEOPIXpin 6
#define NUMPIXELS 8 
#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

Servo SteeringServo; 
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXpin, NEO_GRB + NEO_KHZ800);
SoftwareSerial bluetoothSerial(8,9); // RX, TX pins for HC-05/HC-068, 9
LiquidCrystal_I2C lcd(0x27,16,2);

Car::Car(int servoPin, int encoderPin, int Ena, int in1, int in2, int wheelsize):mycam(*this) {
  
 // Camtrack x(*this);
  _servoPin = servoPin;
  _encoderPin = encoderPin;
  _Ena = Ena;
  _in1 = in1;
  _in2 = in2;
  _wheelsize = wheelsize;  
  _speed = 0;
  _direction = 0;
  _BTstatus = false;
  _onofsw  = false;

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
  pixels.begin(); // INITIALIZE NeoPixel strip object 
  pixels.clear();
  pixels.setBrightness(100);
  pix(0, 80, 80);
  delay(70);
  
  mycam.begin(200);//sets the default tracking speed (0-255)
  bluetoothSerial.begin(9600); // Bluetooth module baud rate

  lcd.init();  
  lcd.backlight();
  lcd.setCursor(5,0);
  lcd.print("Racer");
  lcd.setCursor(1, 1);
  lcd.print("Setup finished");
  pix(0, 0, 0);
  lcd.setBacklight(0);

}

void Car::bt(){
   //send data handler
  bluetoothSerial.print(_speed);//Value1
  bluetoothSerial.print("|");
  bluetoothSerial.print(_direction);//value2
  bluetoothSerial.print("|");
  bluetoothSerial.print(int(getrefligh()));
  bluetoothSerial.print("|");
  bluetoothSerial.print("Hi");//text message to app
  bluetoothSerial.print("|");
  bluetoothSerial.print(checkbatlevel());//text message to app
  bluetoothSerial.print("|");
  bluetoothSerial.print("10");//
  bluetoothSerial.print("|"); // bluetoothSerial.write(13); //this is \r=new line
  bluetoothSerial.println("");
  delay(10);
  //receive data
  if(bluetoothSerial.available()){
    char btdata = bluetoothSerial.read();
    switch(btdata){
      case '1'://blink
        pinMode(13, OUTPUT);
       for(int i = 0; i < 20; i++){
        digitalWrite(13,HIGH);
        delay(50);
        digitalWrite(13,LOW);
        delay(50);
        }
        break;
      case '2':
        //reserved
        break;
      case '3': //stop
        stop();
        delay(2000); 
        break;
      case '4': //increase speed
        _speed +=10;
        if(_speed > 254){_speed = 254;}
        move(_speed);
        break;
      case '5': //decrease speed
        _speed -=10;
        if(_speed < -254){_speed = -254;}
        move(_speed);
        break;  
      case '6': //steer right
        _direction +=10;
        steer(_direction);
        break;
      case '7': //steer left
        _direction -=10;
        steer(_direction);
        break;    
      case '8':
        _onofsw = true;
        break;
      case '9':
        _onofsw = false;
        stop();
        break;
      case 'a':
        lcd.backlight();
        break;    
      case 'b':
        lcd.setBacklight(0);
        break;   
       
    }
    _BTstatus = true;
   }
   else {
    _BTstatus = false;
  }
  //check camera operational status
  camswitch(_onofsw);
  
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

  kp = KP;
  ki = KI; 
  kd = KD;
  steps = 0; //reset the steps count
  for(int i = 0; i < times; i++) {
    Serial.println(steps);

    int tempsteps = getSteps(); //input value
    int output = PIDcalc(tempsteps, clicks); //output value = the calculated error
    lcdenshow(clicks, output, tempsteps);
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
 _speed = speed;
 ShowInfoLcd(_speed, _direction, _BTstatus);
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
 _direction = dir;
 ShowInfoLcd(_speed, _direction, _BTstatus);

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
void Car::lcdswitch(bool status){
  if(status){lcd.backlight();}
  else{lcd.setBacklight(0);}
}

void Car::ShowInfoLcd(int speed, int direction, int BTstatus){ 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("speed| dir | BT ");
  lcd.setCursor(1,1);
  lcd.print(speed);  
  lcd.setCursor(5,1);
  lcd.print("|"); 
  lcd.setCursor(7,1);
  lcd.print(direction); 
  lcd.setCursor(11,1);
  lcd.print("|");  
  lcd.setCursor(13,1);
  lcd.print(BTstatus); 
}

void Car::lcdenshow(int clicks, int output, int tempsteps){ 
  lcd.setBacklight(1);
  lcd.clear();
  lcd.setCursor(0, 0);
   //        0123456789012345 
  lcd.print(" SP | PID |  PV ");
  lcd.setCursor(0,1);
  lcd.print(clicks);  
  lcd.setCursor(4,1);
  lcd.print("|"); 
  lcd.setCursor(6,1);
  lcd.print(output); 
  lcd.setCursor(10,1);
  lcd.print("|");  
  lcd.setCursor(12,1);
  lcd.print(tempsteps); 
}
int Car::checkbatlevel(){
  //pixels.clear();
  smootheningfunction:
  int voltage = map(analogRead(A0), 0, 800, 0, 80); //Sampling input voltage through voltage divider
  int tempvoltage1 = map(analogRead(A0), 0, 800, 0, 80); //taking two measurements to reduce false alarms
  int tempvoltage2 = map(analogRead(A0), 0, 800, 0, 80); //more smoothing
  if(voltage == tempvoltage1 && voltage == tempvoltage2){  
    switch (voltage) {
      case 0 ... 59:
      for(int i = 0; i < 4; i++){
        pixels.setPixelColor(i, 255, 0, 0);
        }
      pixels.show();
      Serial.println("battery critically LOW!");

        break;
      case 60 ... 64:
        for(int i = 0; i < 4; i++){
          pixels.setPixelColor(i, 255, 0, 0);
          }
        pixels.setPixelColor(4, 0, 255, 0);
        pixels.show();
        Serial.println("battery LOW - change now");
        lcd.backlight();
        lcd.print("battery very low");
        break;
      case 65 ... 69:
        pixels.setPixelColor(4, 0, 255, 0);
        pixels.setPixelColor(5, 0, 255, 0);
        pixels.show();
        break;
      case 70 ... 77:
        pixels.setPixelColor(0, 255, 0, 0);
        pixels.setPixelColor(1, 255, 0, 0);
        pixels.setPixelColor(2, 255, 0, 0);
        pixels.setPixelColor(3, 255, 0, 0); 
        pixels.setPixelColor(4, 0, 255, 0);
        pixels.setPixelColor(5, 0, 255, 0);
        pixels.setPixelColor(6, 0, 0, 255);
        pixels.setPixelColor(7, 0, 0, 0);
        pixels.show();
        break;
      case 78 ... 84:
        pixels.setPixelColor(0, 255, 0, 0);
        pixels.setPixelColor(1, 255, 0, 0);
        pixels.setPixelColor(2, 255, 0, 0);
        pixels.setPixelColor(3, 255, 0, 0);     
        pixels.setPixelColor(4, 0, 255, 0);
        pixels.setPixelColor(5, 0, 255, 0);
        pixels.setPixelColor(6, 0, 0, 255);
        pixels.setPixelColor(7, 0, 0, 255);
        pixels.show();
        break;
    }
    return map(voltage, 50, 84, 0, 100);
  }  
  else{goto smootheningfunction;}
}
void Car::btreset(){
  bluetoothSerial.end();
  Serial.println("resetting BT");
  delay(1000);
  bluetoothSerial.begin(9600);
}
void Car::run(int kpp, int kii, int kdd){
  mycam.run(kpp, kii, kdd);
}

void Car::camswitch(bool onofsw){
 _onofsw = onofsw;
 if(_onofsw){
   run(1, 0, 0);
   _onofsw = true;
  }
  else{
   _onofsw = false;
  }
 }
