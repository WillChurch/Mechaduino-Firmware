/*
  SAM21D18 (Arduino Zero compatible), AS5047 encoder, A4954 driver

  Controlled via a SerialUSB terminal at 115200 baud.

*/

#include <avr/pgmspace.h>
#include <SPI.h>
#include "Setup.h"
#include "State.h"
#include "Controller.h"

Controller controller = Controller();

const PROGMEM float force_lookup[] = {
};


namespace Unused {
    int step_state = 1;
    long angle = 0; //holds processed angle value
    volatile int interrupted = 0;
    const float pi = 3.14159;
    const int half = 134; //128;
    float new_angle = 0.0; //input angle
    float current_angle = 0.0; //current angle
    float diff_angle = 0.0;
    int analogPin = 1;
    int val = 0;
    int aout = 0;
}

volatile long step_count = 0;  //For step/dir interrupt
int val1 = 0;
int val2 = 0;



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


//////////////////////////////
// Command Interface




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

    controller.serialCheck();

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