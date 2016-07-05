#ifndef MECHADUINO_PINS_H
#define MECHADUINO_PINS_H

#include <variant.h>
#include <wiring_constants.h>

/* // @TODO Picture doesn't match!
  ____
      |
  0005|-> LED
  0007|-> pulse
  0009|-> IN_1
  0010|-> IN_2
  0012|-> IN_3
  0011|-> IN_4
      |->                   \
  0008|-> VREF_1             \___A4954
  0013|-> VREF_2            _/
     4|->
  0006|-> chipSelectPin
     2|
      |
   SCK|-> clockPin
  MOSI|-> MISO
  MISO|-> MOSI
  ____|

 */
const int IN_4 = 6;//11 - 1;
const int IN_3 = 5;//12 - 1;
const int VREF_2 = 4;//13 - 1;
const int VREF_1 = 9;//3;//8-1;
const int IN_2 = 7;//10 - 1;
const int IN_1 = 8;//9 - 1;
const int pulse = 13;//5;
const int ledPin = 13;//5; //LED connected to digital pin 13
const int chipSelectPin = A2;//5;//6; //output to chip select
const int step_pin  = 1;
const int dir_pin = 0;//2;

//const int sine_out = //3;				// pins for debugging waveforms
//const int encoder_out = //4;

#endif //MECHADUINO_PINS_H
