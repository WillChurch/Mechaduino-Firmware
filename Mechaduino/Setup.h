//
// Created by marco on 7/4/16.
//

#ifndef MECHADUINO_SETUP_H
#define MECHADUINO_SETUP_H

#include <wiring_digital.h>
#include <wiring_constants.h>
#include <WInterrupts.h>
#include <wiring_analog.h>
#include <SPI.h>
#include <USB/USBAPI.h>
#include <delay.h>

#define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.STATUS.bit.SYNCBUSY);

//////////////////////////////////////
//////////////////PINS////////////////
//////////////////////////////////////

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

void setupPins(voidFuncPtr stepInterruptFunction) {

  pinMode(VREF_2, OUTPUT);
  pinMode(VREF_1, OUTPUT);
  pinMode(IN_4, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(pulse, OUTPUT);
  pinMode(step_pin, INPUT);
  pinMode(dir_pin, INPUT);

  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);

  attachInterrupt(1, stepInterruptFunction, RISING);

  analogWrite(VREF_2, 64);
  analogWrite(VREF_1, 64);

  digitalWrite(IN_4, HIGH);
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_2, HIGH);
  digitalWrite(IN_1, LOW);

  pinMode(ledPin, OUTPUT); // visual signal of I/O to chip
  // pinMode(clockPin, OUTPUT); // SCK
  pinMode(chipSelectPin, OUTPUT); // CSn -- has to toggle high and low to signal chip to start data transfer
  //  pinMode(inputPin, INPUT); // SDA
}

void setupSPI() {

  SPISettings settingsA(400000, MSBFIRST, SPI_MODE1);             ///400000, MSBFIRST, SPI_MODE1);

  SPI.begin();    //AS5047D SPI uses mode=1 (CPOL=0, CPHA=1)
  SerialUSB.println("Begin...");
  delay(1000);
  SPI.beginTransaction(settingsA);

}

void setupTCInterrupts() {

  // Enable GCLK for TC4 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (unsigned int) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
  while (GCLK->STATUS.bit.SYNCBUSY);

  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // Set Timer counter Mode to 16 bits
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;   // Set perscaler
  WAIT_TC16_REGS_SYNC(TC5)


  TC5->COUNT16.CC[0].reg = 0x3E72; //0x4AF0;
  WAIT_TC16_REGS_SYNC(TC5)


  TC5->COUNT16.INTENSET.reg = 0;              // disable all interrupts
  TC5->COUNT16.INTENSET.bit.OVF = 1;          // enable overfollow
  TC5->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0

  // Enable InterruptVector
  NVIC_EnableIRQ(TC5_IRQn);

  // Enable TC
  //  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  //  WAIT_TC16_REGS_SYNC(TC5)
}

#endif //MECHADUINO_SETUP_H
