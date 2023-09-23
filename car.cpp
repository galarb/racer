#include "pins_arduino.h"
#include "car.h"
#include "WString.h"
#include "Stream.h"
#include "HardwareSerial.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include "camtrack.h"
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <Adafruit_PWMServoDriver.h>


#define NEOPIXpin 6
#define NUMPIXELS 8 
#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels
#define TimeoutCam 100 //default camera tracking cycles
Servo SteeringServo; 
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXpin, NEO_GRB + NEO_KHZ800);
SoftwareSerial bluetoothSerial(8, 9); // RX, TX pins for HC-05/HC-068, 9
LiquidCrystal_I2C lcd(0x27,16,2);
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);//object to represent PCA9685 at default I2C address  (0x40, more extensions up to 0x60)

volatile long Car::steps = 0; // Initialize the static member variable
String textmessage = "Racer Sais Hi";

Car::Car(int servoPin, int encoderPin, int buttonPin, int Ena, int in1, int in2, int wheelsize):mycam(*this) {
  
 // Camtrack x(*this);
  _servoPin = servoPin;
  _encoderPin = encoderPin;
  _buttonPin = buttonPin;
  _Ena = Ena;
  _in1 = in1;
  _in2 = in2;
  _wheelsize = wheelsize;  
  _speed = 0;
  _direction = 0;
  _BTstatus = false;
  _onofsw  = false;


  pinMode(_encoderPin, INPUT); 
  pinMode(_Ena, OUTPUT); 
  pinMode(_in1, OUTPUT);
  pinMode(_in2, OUTPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);

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
  Serial.println("Reflected light Pin = A2");
  Serial.println("Neopixels Pin = 6");  
  

  SteeringServo.attach(_servoPin);
  pixels.begin(); // INITIALIZE NeoPixel strip object 
  pixels.clear();
  pixels.setBrightness(100);
  pix(0, 80, 80);
  delay(70);

  mycam.begin(200);//sets the default tracking speed (0-255)

  bluetoothSerial.begin(9600); // Bluetooth module baud rate

  //Using Interupt to check when there is a chage from obstruction to hole in the encoder
  attachInterrupt(digitalPinToInterrupt(_encoderPin), Car::getSteps, FALLING); //on falling edge, perform getSteps()

  pca9685.begin();  
  pca9685.setPWMFreq(500);   // Set PWM Frequency(24-1526Hz). (default 500Hz)
 
  
  lcd.init();  
  lcd.backlight();
  lcd.setCursor(5,0);
  lcd.print("Racer");
  lcd.setCursor(1, 1);
  lcd.print("Setup finished");
  pix(0, 0, 0);
  lcd.setBacklight(0);
  Serial.println("Setup finished");
}

bool Car::camswitch(bool onofsw){
  _onofsw = onofsw;
  Serial.print("onofsw status = ");
  Serial.println(_onofsw);
  //while(1){run(1, 0, 0);}
  /*if(_onofsw){
   for(int i = 0; i < TimeoutCam; i++){
     run(1, 0, 0);
     }
   _onofsw = false;
  }
  else{
         run(0, 0, 0);

  }*/
  return (_onofsw);
 }

void Car::bt(){
  /// String senddata = "254|100|100|HI from Racer!!|100|10\0"; 
  String senddata = String(_speed)  + "|" + String(_direction) + "|" + String(getrefligh()) + "|" + String(textmessage) + "|" + String(checkbatlevel()) + "|" + "10" + "|\n";
  Serial.print(senddata);
  char dataBuffer[50];
  senddata.toCharArray(dataBuffer, sizeof(dataBuffer));
  for (int i = 0; dataBuffer[i] != '\0'; i++) {
   // bluetoothSerial.write(dataBuffer[i]);
    //Serial.print(dataBuffer[i]);
   }

  delay(10);
  
  /*
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
  delay(10);*/
  
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
        delay(1000); 
        break;
      case '4': //increase speed
        _speed +=30;
        if(_speed > 254){_speed = 254;}
        move(_speed);
        break;
      case '5': //decrease speed
        _speed -=30;
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
        lcd.clear();
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
  //  Serial.println(steps);

   currentTime = millis();                //get current time
   elapsedTime = (double)(currentTime - previousTime)/1000; //compute time elapsed from previous computation (60ms approx). divide in 1000 to get in Sec
   //Serial.print(currentTime); //for serial plotter
   //Serial.println("\t"); //for serial plotter
   error = sp - inp;                                  // determine error
   cumError += error * elapsedTime;                   // compute integral
   rateError = (error - lastError)/elapsedTime;       // compute derivative deltaError/deltaTime
   if(rateError > 0.3 || rateError < -0.3){cumError = 0;}             // reset the Integral commulator when Proportional is doing the work

   double out = kp*error + ki*cumError + kd*rateError; //PID output               

   lastError = error;                                 //remember current error
   previousTime = currentTime;                        //remember current time
   if(out > 254){out = 254;}    //limit the function for smoother operation
   if(out < -254){out = -254;}
   if(cumError > 255 || cumError < -255){cumError = 0; out = 0;} // reset the Integral commulator
   return out;                                        //the function returns the PID output value 
  
}

void Car::getSteps(){
     steps++; 
}

long Car::goencoder(long clicks, int times, double KP, double KI, double KD){
  unsigned long millisBefore;
  if (millis() - millisBefore > 500) {//non blocking check window for the Interrupt
      kp = KP/10;
      ki = KI; 
      kd = KD;
      steps = 0; //reset the steps count
      for(int i = 0; i < times; i++) {
        //Serial.println(steps);
        //input value is steps
        int output = PIDcalc(steps, clicks); //output value = the calculated error
        lcdenshow(clicks, output, steps);
        delay(30);
        if (output > 0){  // 
        move(output);
        }
        if (output < 0){  // 
        stop();
        } 
     millisBefore = millis();
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
 int tempdir = map(dir, -100, 100, 2100, 3350);  //2725 uSec Duty Cycle is my Zero heading
 
 //SteeringServo.write(tempdir); //I preffered to use the PCA9685 rather then rely on Arduino PWM signals, due to distortions caused when the BT is active. very strange. 
 pca9685.setPWM(8, 0, tempdir);    // Write to PCA9685. ״1״  at 0, 0 at "value"
 //Serial.println(tempdir);
 pixshow(dir);
 _direction = dir;
 //ShowInfoLcd(_speed, _direction, _BTstatus);

}

int Car::getrefligh(){
  int x = analogRead(A2);
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
  Serial.println(clicks);

  Serial.println(goencoder(clicks, 100, 100, 1, 0));
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
  lcd.setCursor(4,1);
  lcd.print(" |  "); 
  lcd.setCursor(7,1);
  lcd.print(direction); 
  lcd.setCursor(10,1);
  lcd.print(" | ");  
  lcd.setCursor(13,1);
  lcd.print(BTstatus); 

}

void Car::lcdenshow(int clicks, int output, int tempsteps){ 
 // lcd.setBacklight(1);
  lcd.clear();
  lcd.setCursor(0, 0);
   //        0123456789012345 
  lcd.print(" SP | PID |steps");
  lcd.setCursor(0,1);
  lcd.print(clicks);  
  lcd.setCursor(4,1);
  lcd.print("|"); 
  lcd.setCursor(6,1);
  lcd.print(output); 
  lcd.setCursor(10,1);
  lcd.print("|");  
  lcd.setCursor(12,1);
  lcd.print(steps); 
}
int Car::checkbatlevel(){
  //pixels.clear();
  smootheningfunction:
  int voltage = map(analogRead(A3), 0, 800, 0, 80); //Sampling input voltage through voltage divider
  int tempvoltage1 = map(analogRead(A3), 0, 800, 0, 80); //taking two measurements to reduce false alarms
  int tempvoltage2 = map(analogRead(A3), 0, 800, 0, 80); //more smoothing
  if(voltage == tempvoltage1 && voltage == tempvoltage2){  
    Serial.println(voltage);
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


byte i2c (int port, int value) { //writes a value to the i2c port
     value = map(value, 0, 255, 0, 4095);
     pca9685.setPWM(port, 0, value);    // Write to PCA9685. ״1״  at 0, 0 at "value"
}
