#include "HardwareSerial.h"
#include <Arduino.h>
#include "clicli.h"
#include "camtrack.h"
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
#include "Wire.h"

int xOrigin = 0;
int yOrigin = 0;
int xTarget = 0;
int yTarget = 0;
int _speed = 0;
Camtrack::Camtrack(Car &car):mycar(car) {}

HUSKYLENS huskylens;
//SoftwareSerial mySerial(10, 11); // RX, TX
//HUSKYLENS green line >> Pin 10; blue line >> Pin 11
void printResult(HUSKYLENSResult result);


void Camtrack::begin(int speed) {
  _speed = speed;
  Serial.begin(115200);
  Wire.begin();
 // mySerial.begin(9600);
  while (!huskylens.begin(Wire))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }
}
int Camtrack::run(int _kp, int _ki, int _kd){
    if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    if(!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    else if(!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
    else
    {
        //Serial.println(F("###########"));
        while (huskylens.available())
        {
            HUSKYLENSResult result = huskylens.read();
            printResult(result);
            kp = _kp;
            ki = _ki;
            kd = _kd;
            return(trackline()); 
        }    
    }
}
double Camtrack::PIDcalc(double inp, int sp){
   currentTime = millis();                //get current time
   elapsedTime = (double)(currentTime - previousTime)/1000; //compute time elapsed from previous computation (60ms approx). divide in 1000 to get in Sec
   //Serial.print(currentTime); //for serial plotter
   //Serial.println("\t"); //for serial plotter
   error = sp - inp;                                  // determine error
   cumError += error * elapsedTime;                   // compute integral
   rateError = (error - lastError)/elapsedTime;       // compute derivative deltaError/deltaTime
   double out = kp*error + ki*cumError + kd*rateError; //PID output               
   //Serial.println(cumError);
   lastError = error;                                 //remember current error
   previousTime = currentTime;                        //remember current time
   if(out > 254){out = 254;}    //limit the function for smoother operation
   if(out < -254){out = -254;}
   if(cumError > 255 || cumError < -255){cumError = 0; out = 0;} // reset the Integral commulator
   if(rateError > 0.3 || rateError < -0.3){cumError = 0;}             // reset the Integral commulator
   output = out;
   return out;                                        //the function returns the PID output value 
  
}
int Camtrack::trackline(){
  int tempdirection = xOrigin - xTarget; //delta x
  Serial.print("delta X = ");
  Serial.println(tempdirection);
  //PIDcalc(tempdirection, 0);//direction 0 is ahead
  mycar.steer(PIDcalc(tempdirection, 0));//
  if(output > 50 || output < -50){ //if a sharp turn, reduce speed
    mycar.move(150); //set this value according to behaviour
   }
   else {mycar.move(_speed);} //run at the default speed
  return(output); //the correction value  for steer()

}
void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    }
    else if (result.command == COMMAND_RETURN_ARROW){
       xOrigin = result.xOrigin;
       xTarget = result.xTarget;  
       yOrigin = result.yOrigin;
       yTarget = result.yTarget;
       //Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
    }
    else{
        Serial.println("Object unknown!");
    }
}
