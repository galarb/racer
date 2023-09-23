/*

                    CLI Software for Arduino

               A Simple Command Line Interface 
  Example functions:              
              Feature                |  CLI Usage
___________________________________________________
 Digial Write HIGH to a specific pin |  h (pin)
 Digial Write LOW to a specific pin  |  l (pin)
 Shoot command                       |  s (angle)
 Digital Read                        |  r (pin)
 Analog Read                         |  e (pin) 

               
      by Gal Arbel
      Oct 2022
*/


#include "clicli.h"
#include "Arduino.h"
#include "HardwareSerial.h"

const unsigned int MAX_MESSAGE_LENGTH = 64;

clicli::clicli(Car &car):mycar(car), number(7) {

}

void clicli::begin() {
  //
}
void clicli::run() {

// CLI - Messages from Terminal
  while (Serial.available() > 0) { 
   char message[MAX_MESSAGE_LENGTH];
   static unsigned int message_pos = 0;
   char inByte = Serial.read();   //Read the next available byte in the serial receive buffer
    if ( inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1) )
     {
     message[message_pos] = inByte;  //Add the incoming byte to our message
     message_pos++;
     }
     //Full message received...
     else
     {
      message[message_pos] = '\0';     //Add null character to string
      Serial.println(message);     //echo the message to terminal
        
      int command[4];
      int argindex = 0;
      char cmd;
      char delim[] = " ";
	     char tmpmsg[MAX_MESSAGE_LENGTH];
       strcpy(tmpmsg,message);
       message_pos = 0;
       message[message_pos] = '\0';     //Add null character to string

        char *ptr = strtok(tmpmsg, delim);
	      while(ptr != NULL)
	       {
		      //Serial.printf("'%s'\n", ptr);
          if (argindex == 0) {
            cmd = ptr[0];
          }
          command[argindex] = atoi(ptr);   
          //Serial.println(command[argindex]);
          argindex++;  
		      ptr = strtok(NULL, delim);
	       } 

      switch (cmd) {
       case 'h': //Set port to HIGH
        pinMode(command[1],OUTPUT);
        digitalWrite(command[1],HIGH);
        Serial.print("Pin "); 
        Serial.print(command[1]);   
        Serial.println(" is SET");   
        delay(1000);
        break;
        
       case 'l': // Set port to LOW
        pinMode(command[1],OUTPUT);
        digitalWrite(command[1],LOW);
        Serial.print("Pin "); 
        Serial.print(command[1]);   
        Serial.println(" is RESET");   
        delay(1000);
        break;
       
       case 'm': // move at speed 
        mycar.move(command[1]);
        Serial.print("running at: ");
        Serial.println(command[1]);
        break;

       case 'v': // test servo motor (angle)
        mycar.steer(command[1]);
        break;
       case 'z'://camera onoff switch
        if(command[1]){
          while(1){mycar.run(1, 0, 0);}
        }
        
        ;

        break;


       case 'd': // test encoder
        Serial.println(mycar.goencoder(command[1], command[2], 3, 0, 0));      //steps, times, P, I, D
        break;

       case 'k': // test encoder
        mycar.gomm(command[1]); 
        break;       

       case 'g': // test encoder
        mycar.goencoder(command[1], command[2], command[3], command[4], 0); 
        break;
       
       case 'f': //reflected light
        Serial.print("Reflected light = ");
        Serial.println(mycar.getrefligh());
        break;

       case 'r': // digital read
        pinMode(command[1],INPUT);
        Serial.print("Pin "); 
        Serial.print(command[1]);   
        Serial.print(" Value = "); 
        Serial.println(digitalRead(command[1]));   
        delay(1000);
        break;
       case 's':
        mycar.stop();
        break;
       case 'p':
        mycar.pix(command[1], command[2], command[3]);
        break;
       case 'b':
        for(int i=0; i<100; i++){
          mycar.checkbatlevel();
        }
        break;
       case 'a':
        mycar.lcdswitch(command[1]);
        break;
       case 't':
        mycar.btreset();
        break;

        case 'e': // analog read
        pinMode(command[1],INPUT);
        Serial.print("Pin "); 
        Serial.print(command[1]);   
        Serial.print(" Value = "); 
        Serial.println(analogRead(command[1]));   
        delay(1000);
        break;


       
       message_pos = 0;     //Reset for the next message
      }
   }
   delay (60); 
 } 
}
