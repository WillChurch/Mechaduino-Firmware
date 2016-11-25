
/*
  -------------------------------------------------------------
  Mechaduino 0.1 Firmware  v0.1.1
  SAM21D18 (Arduino Zero compatible), AS5047 encoder, A4954 driver

  All Mechaduino related materials are released under the
  Creative Commons Attribution Share-Alike 4.0 License
  https://creativecommons.org/licenses/by-sa/4.0/

  Many thanks to Will Church, Marco Farrugia, and Kai Wolter.
  --------------------------------------------------------------
  
  Controlled via a SerialUSB terminal at 115200 baud.

  Implemented serial commands are:

 s  -  step
 d  -  dir
 p  -  print angle [step count] , [assumed angle] , [encoder reading]

 c  -  calibration routine
 e  -  check encoder diagnositics
 q  -  parameter query

 x  -  position mode
 v  -  velocity mode
 x  -  torque mode

 y  -  enable control loop
 n  -  disable control loop
 r  -  enter new setpoint

 k  -  edit controller gains
 m  -  print main menu

  ...see serialCheck() in Utils for more details

*/


#include "Utils.h"
#include "Parameters.h"
#include "State.h"
#include "analogFastWrite.h"
#include "FlashAsEEPROM.h"


//////////////////////////////////////
/////////////////SETUP////////////////
//////////////////////////////////////


void setup() {
  digitalWrite(13,HIGH);
  setupPins();  
  setupTCInterrupts();
  sineGen();
  
  
  SerialUSB.begin(115200);
  delay(3000);              //This delay seems to make it easier to establish a conncetion when the Mechaduino is configured to start in closed loop mode.
  serialMenu();
  setupSPI();
  digitalWrite(13,LOW);
  
  pinMode(3, OUTPUT);

  //  enableTCInterrupts();     //start in closed loop mode
  //  mode = 'x';


  // If the EEPROM is empty then isValid() is false
  if (!EEPROM.isValid()) {

    SerialUSB.println("EEPROM is empty, writing some example data:");
    SerialUSB.print("->");
    for (int i=0; i<20; i++) {
      EEPROM.write(i, 100.5+i);
      SerialUSB.print(" ");
      SerialUSB.print(100+i);
    }
    SerialUSB.println();

    // commit() saves all the changes to EEPROM, it must be called
    // every time the content of virtual EEPROM is changed to make
    // the change permanent.
    // This operation burns Flash write cycles and should not be
    // done too often. See readme for details:
    // https://github.com/cmaglie/FlashStorage#limited-number-of-writes
    EEPROM.commit();
    SerialUSB.println("Done!");

    SerialUSB.print("After commit, calling isValid() returns ");
    SerialUSB.println(EEPROM.isValid());

  } else {

    SerialUSB.println("EEPROM has been written.");
    SerialUSB.println("Here is the content of the first 20 bytes:");

    SerialUSB.print("->");
    for (int i=0; i<20; i++) {
      SerialUSB.print(" ");
      SerialUSB.print(EEPROM.read(i));
    }
    SerialUSB.println();

  }

}



//////////////////////////////////////
/////////////////LOOP/////////////////
//////////////////////////////////////

void loop()
{

  serialCheck();

  //r=0.1125*step_count; --- no longer need this adjust step angle in parameters.cpp



}
