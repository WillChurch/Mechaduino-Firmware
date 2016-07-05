// multi file
/*

  SAM21D18 (Arduino Zero compatible), AS5047 encoder, A4954 driver

  Controlled via a SerialUSB terminal at 115200 baud.

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


  Implemented commands are:

  p  -  print [step count] , [assumed angle] , [encoder reading]

  c  -  clear step count & assumed angle

  s  -  step

  d  -  dir toggle

  z  -  seek zero position

  g  -  Go! steps around 400 times

  w  -  Same as go, but stores encoder angles to EEPROM

  r  -  returns EEPROM contents

  a  -  prompts user to enter angle

  y  -  sine sweep

*/

#include <math.h>
#include <avr/pgmspace.h>
#include <SPI.h>
#include <Wire.h>
#include "Mechaduino_functions.h"
#include "Setup.h"
#include "State.h"
#include "ParameterEditor.h"
#include "Utils.h"

ParameterEditor parameterEditor = ParameterEditor();

const PROGMEM float force_lookup[] = {
};


const int spr = 200; //  200 steps per revolution
const float aps = 360.0 / spr; // angle per step
const int cpr = 16384; //counts per rev

int dir = 1;		//initialize stepping mode variables
int step_state = 1;

long angle = 0; //holds processed angle value

float anglefloat = 0;

int a = 0;  // raw encoder value in closed loop and print_angle routine (should fix the latter to include LUT)

volatile long step_count = 0;  //For step/dir interrupt

volatile int interrupted = 0;

int stepNumber = 0; // step index for cal routine


//___________________________________

const float pi = 3.14159;
const int  half = 134;//128;

float new_angle = 0.0; //input angle
float current_angle = 0.0; //current angle
float diff_angle = 0.0;
int val1 = 0;
int val2 = 0;


int analogPin = 1;
int val = 0;

int aout = 0;


int readEncoder()           //////////////////////////////////////////////////////   READENCODER   ////////////////////////////
{
  long angleTemp;
  digitalWrite(chipSelectPin, LOW);

  //angle = SPI.transfer(0xFF);
  byte b1 = SPI.transfer(0xFF);
  byte b2 = SPI.transfer(0xFF);


  angleTemp = (((b1 << 8) | b2) & 0B0011111111111111);
  //  SerialUSB.println((angle & 0B0011111111111111)*0.02197265625);

  digitalWrite(chipSelectPin, HIGH);
  return angleTemp;
}

//////////////////////////////////////
/////////////////FUNCTIONS/////////////////////
//////////////////////////////////////

void TC5_Handler()
{

  //unedited/old:
  /*  // TcCount16* TC = (TcCount16*) TC3; // get timer struct
    if (TC5->COUNT16.INTFLAG.bit.OVF == 1) {  // A overflow caused the interrupt
        interrupted = 1;

      TC5->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
    //  irq_ovf_count++;                 // for debug leds
    }

    // if (TC->INTFLAG.bit.MC0 == 1) {  // A compare to cc0 caused the interrupt
    //  digitalWrite(pin_mc0_led, LOW);  // for debug leds
     // TC->INTFLAG.bit.MC0 = 1;    // writing a one clears the flag ovf flag
    // } */


  ///new
  // TcCount16* TC = (TcCount16*) TC3; // get timer struct
  if (TC5->COUNT16.INTFLAG.bit.OVF == 1) {  // A overflow caused the interrupt

    a = readEncoder();
    y = lookup_angle(a);

    if ((y - y_1) < -180.0) {
      wrap_count += 1;
    }
    else if ((y - y_1) > 180.0) {
      wrap_count -= 1;
    }
    //y_1 = y;  pushed lower

    yw = (y + (360.0 * wrap_count));

    switch (mode) {
      case Position:
        e = (r - yw);

        ITerm += (pKi * e);
        if (ITerm > 150) ITerm = 150;
        else if (ITerm < -150) ITerm = -150;

        u = ((pKp * e) + ITerm - (pKd * (yw - yw_1))); //ARDUINO library style
        //u = u+lookup_force(a)-20;
        //   u = u_1 + cA*e + cB*e_1 + cC*e_2;     //ppt linked in octave script

        //  u = 20*e;//
        break;

      case Velocity:
        e = (r - ((yw - yw_1) * 500));//416.66667)); degrees per Tc to rpm

        ITerm += (vKi * e);
        if (ITerm > 200) ITerm = 200;
        else if (ITerm < -200) ITerm = -200;

        u = ((vKp * e) + ITerm - (vKd * (yw - yw_1)));//+ lookup_force(a)-20; //ARDUINO library style
        break;

      case Torque:
        u = 1.0 * r ;//+ 1.7*(lookup_force(a)-20);
        break;

      default:
        u = 0;
        break;
    }

//
//    if (u > 0) {
//      PA = 1.8;
//    }
//    else {
//      PA = -1.8;
//    }
//
//    y += PA;

    if (u > 0) {
      y+=PA;
    }
    else {
      y -=PA;
    }

    if (u > 200) {                          //saturation limits max current command
      u = 200;
    }
    else if (u < -200) {
      u = -200;
    }

    U = abs(u);       //+lookup_force((((a-4213)%16384)+16384)%16384)-6); ///p);//+i);

    if (abs(e) < 0.1) {
      digitalWrite(pulse, HIGH);
      //  SerialUSB.println(r);
    }
    else  {
      digitalWrite(pulse, LOW);
    }

    output(-y, U);  //-y

    e_3 = e_2;
    e_2 = e_1;
    e_1 = e;
    u_3 = u_2;
    u_2 = u_1;
    u_1 = u;
    yw_1 = yw;
    y_1 = y;


    TC5->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
  }
}


void print_angle()                ///////////////////////////////////       PRINT_ANGLE   /////////////////////////////////
{
  a = 0;
  delay(100);
  // @TODO Suspicious
  a += readEncoder();
  delay(10);
  a += readEncoder();
  delay(10);
  a += readEncoder();
  delay(10);
  a += readEncoder();
  delay(10);
  a += readEncoder();
  delay(10);
  a += readEncoder();
  delay(10);
  a += readEncoder();
  delay(10);
  a += readEncoder();
  delay(10);
  a += readEncoder();
  delay(10);
  a += readEncoder();
  delay(10);
  a = a / 10;

  anglefloat = a * 0.02197265625;
  SerialUSB.print(stepNumber, DEC);
  SerialUSB.print(" , ");
  SerialUSB.print(stepNumber * aps, DEC);
  SerialUSB.print(" , ");
  SerialUSB.print(a, DEC);
  SerialUSB.print(" , ");
  SerialUSB.println(anglefloat, DEC);
}

void output(float theta, int effort) {                    //////////////////////////////////////////   OUTPUT   ///////////////////
  static int start = 0;
  static int finish = 0;
  static int intangle;
  static float floatangle;
  static int modangle;


  floatangle = (10000 * ( theta * 0.87266 + 2.3562) );//0.7854) );// 2.3562) );       //changed to 2.3 for NEMA23,NEMA17 dual..... opposite below
  //floatangle = (10000 * ( theta * 0.87266 + 0.7854) );

  intangle = (int)floatangle;
  //  modangle = (((intangle % 628) + 628) % 628);
  val1 = effort * lookup_sine(intangle);

  analogWrite(VREF_2, abs(val1));

  if (val1 >= 0)  {
    digitalWrite(IN_4, HIGH);
    //     PORTB |= (B00000001);
    digitalWrite(IN_3, LOW);
    //    PORTB &= ~(B00000010);

  }
  else  {
    digitalWrite(IN_4, LOW);
    //  PORTB &= ~(B00000001);
    digitalWrite(IN_3, HIGH);
    //    PORTB |= (B00000010);

  }

  floatangle = (10000 * (  theta * 0.8726646 + 0.7854) );//2.3562) );//0.7854) );
  //floatangle = (10000 * ( theta * 0.87266 + 2.3562) );

  intangle = (int)floatangle;
  // modangle = (((intangle % 628) + 628) % 628);
  val2 = effort * lookup_sine(intangle);

  analogWrite(VREF_1, abs(val2));

  if (val2 >= 0)  {
    digitalWrite(IN_2, HIGH);
    //     PORTB |= (B00000100);
    digitalWrite(IN_1, LOW);
    //     PORTB &= ~(B00001000);

  }
  else  {
    digitalWrite(IN_2, LOW);
    //   PORTB &= ~(B00000100);
    digitalWrite(IN_1, HIGH);
    //   PORTB |= (B00001000);

  }
}


void readEncoderDiagnostics()           //////////////////////////////////////////////////////   READENCODERDIAGNOSTICS   ////////////////////////////
{
  long angleTemp;
  digitalWrite(chipSelectPin, LOW);

  ///////////////////////////////////////////////READ DIAAGC (0x3FFC)
  SerialUSB.print("DIAAGC (0x3FFC)   ");

  SPI.transfer(0xFF);
  SPI.transfer(0xFC);
  digitalWrite(chipSelectPin, HIGH);

  delay(1);
  digitalWrite(chipSelectPin, LOW);

  byte b1 = SPI.transfer(0xC0);
  byte b2 = SPI.transfer(0x00);


  angleTemp = (((b1 << 8) | b2) & 0B1111111111111111);
  SerialUSB.print((angleTemp | 0B1110000000000000000 ), BIN);

  if (angleTemp & (1 << 14)) {
    SerialUSB.print("  Error occurred  ");
  }
  if (angleTemp & (1 << 11)) {
    SerialUSB.print("  MAGH  ");
  }
  if (angleTemp & (1 << 10)) {
    SerialUSB.print("  MAGL  ");
  }
  if (angleTemp & (1 << 9)) {
    SerialUSB.print("  COF  ");
  }
  if (angleTemp & (1 << 8)) {
    SerialUSB.print("  LF  ");
  }
  SerialUSB.println(" ");

  digitalWrite(chipSelectPin, HIGH);

  delay(1);

  digitalWrite(chipSelectPin, LOW);
  ///////////////////////////////////////////////READ ERRFL (0x0001)
  SerialUSB.print("ERRFL (0x0001)   ");

  SPI.transfer(0x40);
  SPI.transfer(0x01);
  digitalWrite(chipSelectPin, HIGH);

  delay(1);
  digitalWrite(chipSelectPin, LOW);

  b1 = SPI.transfer(0xC0);
  b2 = SPI.transfer(0x00);


  angleTemp = (((b1 << 8) | b2) & 0B1111111111111111);
  SerialUSB.print((angleTemp | 0B1110000000000000000 ), BIN);

  if (angleTemp & (1 << 14)) {
    SerialUSB.print("  Error occurred  ");
  }
  if (angleTemp & (1 << 2)) {
    SerialUSB.print("  parity error ");
  }
  if (angleTemp & (1 << 1)) {
    SerialUSB.print("  invalid register  ");
  }
  if (angleTemp & (1 << 0)) {
    SerialUSB.print("  framing error  ");
  }

  SerialUSB.println(" ");

  digitalWrite(chipSelectPin, HIGH);

  delay(1);
}


float lookup_force(int m)        /////////////////////////////////////////////////  LOOKUP_force   /////////////////////////////
{
  float b_out;
  //
  //  m = (0.01*(((m % 62832) + 62832) % 62832))+0.5;  //+0.5 for rounding
  //
  //  //SerialUSB.println(m);
  //
  //  if (m > 314) {
  //    m = m - 314;
  //    b_out = -pgm_read_float_near(force_lookup + m);
  //
  //  }
  //  else
  //  {
  b_out = pgm_read_float_near(force_lookup + m);
  //  }

  return b_out;
}


void stepInterrupt() {
  if (digitalRead(dir_pin))
  {
    step_count += 1;
  }
  else
  {
    step_count -= 1;
  }
}

void enableTCInterrupts() {
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    //Enable TC5
  WAIT_TC16_REGS_SYNC(TC5)                      //wait for sync
}


void disableTCInterrupts() {
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC5
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync
}

//////////////////////////////
// Command Interface


void commandW() {

  int encoderReading = 0;     //or float?  not sure if we can average for more res?
  int lastencoderReading = 0;
  int avg = 10;         //how many readings to average

  int iStart = 0;
  int jStart = 0;
  int stepNo = 0;

  int fullStepReadings[spr];
  int fullStep = 0;
  //  float newLookup[cpr];
  int ticks = 0;

  float lookupAngle = 0.0;

  encoderReading = readEncoder();
  dir = 1;
  oneStep();
  delay(500);

  if ((readEncoder() - encoderReading) < 0)
  {
    //dir = 0;
    SerialUSB.println("Wired backwards");
    return;
  }

  while (stepNumber != 0) {
    if (stepNumber > 0) {
      dir = 1;
    }
    else
    {
      dir = 0;
    }
    oneStep();
    delay(100);
  }
  dir = 1;
  for (int x = 0; x < spr; x++) {

    encoderReading = 0;
    delay(100);

    for (int reading = 0; reading < avg; reading++) {
      encoderReading += readEncoder();
      delay(10);
    }

    encoderReading = encoderReading / avg;

    anglefloat = encoderReading * 0.02197265625;
    fullStepReadings[x] = encoderReading;
    SerialUSB.println(fullStepReadings[x], DEC);
    oneStep();
  }
  SerialUSB.println(" ");
  SerialUSB.println("ticks:");
  SerialUSB.println(" ");
  for (int i = 0; i < spr; i++) {
    ticks = fullStepReadings[mod((i + 1), spr)] - fullStepReadings[mod((i), spr)];
    if (ticks < -15000) {
      ticks += cpr;

    }
    else if (ticks > 15000) {
      ticks -= cpr;
    }
    SerialUSB.println(ticks);

    if (ticks > 1) {
      for (int j = 0; j < ticks; j++) {
        stepNo = (mod(fullStepReadings[i] + j, cpr));
        // SerialUSB.println(stepNo);
        if (stepNo == 0) {
          iStart = i;
          jStart = j;
        }

      }
    }

    if (ticks < 1) {
      for (int j = -ticks; j > 0; j--) {
        stepNo = (mod(fullStepReadings[199 - i] + j, cpr));
        // SerialUSB.println(stepNo);
        if (stepNo == 0) {
          iStart = i;
          jStart = j;
        }
      }
    }
  }

  SerialUSB.println(" ");
  SerialUSB.println("newLookup:");
  SerialUSB.println(" ");

  for (int i = iStart; i < (iStart + spr); i++) {
    ticks = fullStepReadings[mod((i + 1), spr)] - fullStepReadings[mod((i), spr)];

    if (ticks < -15000) {
      ticks += cpr;
    }
    else if (ticks > 15000) {
      ticks -= cpr;
    }
    //SerialUSB.println(ticks);

    if (ticks > 1) {
      for (int j = jStart; j < (jStart + ticks); j++) {
        lookupAngle = 0.01 * mod(100 * (aps * i + (aps * j / ticks)), 36000.0);
        SerialUSB.print(lookupAngle);
        SerialUSB.print(" , ");
      }
    }
    else if (ticks < 1) {
      for (int j = jStart - ticks; j > (jStart); j--) {
        lookupAngle = 0.01 * mod(100 * (aps * (i) + (aps * (ticks + j) / ticks)), 36000.0);
        SerialUSB.print(lookupAngle);
        SerialUSB.print(" , ");
      }
    }
  }
  SerialUSB.println(" ");
}

void serialCheck() {

  if (SerialUSB.available()) {
    char inChar = (char)SerialUSB.read();

    switch (inChar) {
      case 'p':             //print
        print_angle();
        break;

      case 's':             //step
        oneStep();
        print_angle();
        break;

      case 'd':             //dir
        if (dir == 1) {
          dir = 0;
        }
        else {
          dir = 1;
        }
        break;

      case 'w':
        commandW();           //cal routine
        break;

      case 'e':
        readEncoderDiagnostics();   //encoder error?
        break;

      case 'y':
        enableTCInterrupts();      //enable closed loop
        break;

      case 'n':
        disableTCInterrupts();      //disable closed loop
        break;

      case 'r':             //new setpoint
        SerialUSB.println("Enter setpoint:");
            waitSerialUSB();
            r = SerialUSB.parseFloat();
        break;

      case 'x':
        mode = Mode::Position;
        break;

      case 'v':
        mode = Mode::Velocity;
        break;

      case 't':
        mode = Mode::Torque;
        break;

      case 'c':
        mode = Mode::Custom;
        break;

      case 'q':
        parameterEditor.parameterQuery();     // prints copy-able parameters
        break;

      case 'a':             //anticogging
        antiCoggingCal();
        break;

      case 'k':
          parameterEditor.parameterEditMain();
        break;

      default:
        break;
    }
  }
}

void antiCoggingCal() {
  SerialUSB.println(" -----------------BEGIN ANTICOGGING CALIBRATION!----------------");
  mode = Mode::Position;
  r = lookup_angle(1);
  enableTCInterrupts();
  delay(1000);

  for (int i = 1; i < 657; i++) {
    r = lookup_angle(i);
    SerialUSB.print(r, DEC);
    SerialUSB.print(" , ");
    delay(100);
    SerialUSB.println(u, DEC);
  }
  SerialUSB.println(" -----------------REVERSE!----------------");

  for (int i = 656; i > 0; i--) {
    r = lookup_angle(i);
    SerialUSB.print(r, DEC);
    SerialUSB.print(" , ");
    delay(100);
    SerialUSB.println(u, DEC);
  }
  SerialUSB.println(" -----------------DONE!----------------");
  disableTCInterrupts();
}


//////////////////////////////////////
// Arduino Main

void setup() {

    setupPins(stepInterrupt);
    setupSPI();
    setupTCInterrupts();

    SerialUSB.begin(115200);
    delay(500);

    //  enableTCInterrupts();     //start in closed loop mode
    //  mode = 'x';
    //
    //  Wire.begin(4);                // join i2c bus with address #8
    //  Wire.onReceive(receiveEvent); // register event

    //pinMode(10, OUTPUT);
    //pinMode(11, OUTPUT);
    //pinMode(12, OUTPUT);
}

void loop()
{
    //mode = 'x';

    serialCheck();

    // r=0.1125*step_count;

    // electronic gearing with analog
    //  val = 0;
    //  for (int i = 0; i < 10; i++){
    //    val += analogRead(analogPin);    // read the input pin
    //   // delay(1);
    //  }
    //
    //  SerialUSB.println(0.01*((float)val));             // debug value
    //  r = 0.01*((float)val);
    //
    //  aout = 10.0*yw;
    //
    //  if (aout > 1023){
    //    aout = 1023;
    //  }
    //  else if (aout < 0){
    //    aout = 0;
    //  }
    //
    //  analogWriteResolution(10);
    //  analogWrite(A0,aout);
    //

    //  for (int i = 0; i < 1024; i++){
    //    analogWrite(A0,i);
    //    delay(2);
    //  }

    //  // tp dispenser:
    //  mode = 'x';
    //
    //  digitalWrite(10, LOW);
    //  digitalWrite(11, LOW);
    //  digitalWrite(12, HIGH);
    //
    //  if (u > 70) {
    //    digitalWrite(10, LOW);
    //    digitalWrite(11, HIGH);
    //    digitalWrite(12, LOW);
    //    for (int i = 0; i < 2700; i++) {
    //      r -= 0.2;
    //      delay(1);
    //    }
    //      digitalWrite(10, HIGH);
    //      digitalWrite(11, LOW);
    //      digitalWrite(12, LOW);
    //    delay(750);
    //    r -= 45;
    //    delay(50);
    //    r += 135;
    //    delay(250);
    //    for (int i = 0; i < 1200; i++) {
    //      r -= 0.2;
    //      delay(1);
    //    }
    //  }

    //    mode = 'x';

    //  r = 90;
    //  delay(2000);
    //  r = 0;
    //  delay(500);
    //  r = -90;
    //  delay (1000);

    //    if (u>50){
    //      r = -45;
    //      delay(500);
    //    }
    //    else if (u< -50){
    //      r = 45;
    //      delay(500);
    //    }
    //
}