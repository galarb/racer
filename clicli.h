#ifndef CLICLI_H
#define CLICLI_H
#include "Car.h"

 class clicli {
  private:
  Car &mycar;
  int number;
  
  public:
   clicli(Car &mycar);
   void begin();   //must be called from  void setup()
   void run();   //must be called from  void loop()

 };
#endif 