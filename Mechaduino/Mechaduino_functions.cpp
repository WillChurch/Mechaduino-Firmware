// All Mechaduino functions are in this file

#include "Arduino.h"
#include <Wire.h>
#include "State.h"

extern void output(float,int);

/////////////////////////////////   oneStep    ///////////////////////////////
void oneStep() {

  if (dir == 0) {
    stepNumber += 1;
  }
  else {
    stepNumber -= 1;
  }
  // arduino mod does not wrap for negative....
  // step_state = ((((stepNumber) % 4) + 4) % 4);
  //output(1.8 * step_state, 128); //1.8 = 90/50

  output(1.8 * stepNumber, 128); //1.8 = 90/50

  delay(10);
}



void receiveEvent(int howMany)
{
  while (1 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    SerialUSB.print(c);
  }
  int x = Wire.read();    // receive byte as an integer
  SerialUSB.println(x);
  r = 0.1 * ((float)x);
}
