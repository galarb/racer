#ifndef CAMTRACK_H
#define CAMTRACK_H

 class Camtrack {
  Car &mycar;

  public:
   Camtrack(Car &mycar);
   void begin(int speed);   //must be called from  void setup(), sets the default speed
   int run(int _kp, int _ki, int _kd); //must be in the loop. sets the default PID coefficients
   int trackline();
   double PIDcalc(double inp, int sp);

  private:
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


 };
#endif 