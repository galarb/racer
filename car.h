/*                 ******************
              Class for managing my car Robot 
              Featuring: single motor drive with servo steering
                         Line tracking
                         accurate distance drive with encoder wheel 
                         PID function
                By: Gal Arbel 2023
                **********************                                   */
#ifndef CAR_H
#define CAR_H
#include "camtrack.h"

class Car { 
  Camtrack mycam;

  public:
    Car(int servoPin, int encoderPin, int buttonPin, int Ena, int in1, int in2, int wheelsize); 
    void begin(double bdrate);    
    long goencoder(long clicks, int times, double KP, double KI, double KD);
    static void getSteps();
    void move(int speed);
    void stop();
    void steer(int dir);
    int getrefligh();
    void trackline(int speed, int color, double KP, double KI, double KD);
    void gomm(long distancemm);
    void pix(int red, int green, int blue);
    void pixshow(int dir);
    void bt();
    void ShowInfoLcd(int speed, int direction, int BTstatus);
    void lcdenshow(int clicks, int output, int tempsteps);
    void lcdswitch(bool status);
    bool camswitch(bool onofsw);
    int checkbatlevel();
    void btreset();
    void run(int kpp, int kii, int kdd);


  private:
    int _servoPin;
    int _encoderPin;
    int _buttonPin;
    int _Ena; //speed
    int _in1;
    int _in2;
    int _reflightPin;
    int _wheelsize;
    int _speed;
    int _direction;
    int _refligh;
    bool _BTstatus;
    static volatile long steps; // Declare steps as a static member variable


    double PIDcalc(double inp, int sp);
    unsigned long currentTime;
    unsigned long previousTime;
    double elapsedTime;
    double error;
    double lastError;
    double input, output;
    double cumError, rateError;
    double kp = 0;
    double ki = 0; 
    double kd = 0;
    bool  _onofsw;
  


    
};


#endif 