//Contains the definitions of the functions used by the firmware.


#include <SPI.h>
#include <Wire.h>

#include "Parameters.h"
#include "Controller.h"
#include "Utils.h"
#include "State.h"
#include "analogFastWrite.h"

void setupPins() {

  pinMode(VREF_2, OUTPUT);
  pinMode(VREF_1, OUTPUT);
  pinMode(IN_4, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_2, OUTPUT);
  pinMode(IN_1, OUTPUT);

  pinMode(step_pin, INPUT);
  pinMode(dir_pin, INPUT);

  pinMode(chipSelectPin, OUTPUT); // CSn -- has to toggle high and low to signal chip to start data transfer

  pinMode(ledPin, OUTPUT); //
  pinMode(3, OUTPUT); //

  // pinMode(clockPin, OUTPUT); // SCL    for I2C
  // pinMode(inputPin, INPUT); // SDA


  attachInterrupt(1, stepInterrupt, RISING);
  attachInterrupt(dir_pin, dirInterrupt, CHANGE);

  analogFastWrite(VREF_2, 0.33 * uMAX);
  analogFastWrite(VREF_1, 0.33 * uMAX);

  digitalWrite(IN_4, HIGH);
  digitalWrite(IN_3, LOW);
  digitalWrite(IN_2, HIGH);
  digitalWrite(IN_1, LOW);

}

void setupSPI() {

  SPISettings settingsA(10000000, MSBFIRST, SPI_MODE1);             ///400000, MSBFIRST, SPI_MODE1);

  SPI.begin();    //AS5047D SPI uses mode=1 (CPOL=0, CPHA=1)
  SerialUSB.println("Beginning SPI communication with AS5047 encoder...");
  delay(1000);
  SPI.beginTransaction(settingsA);

}


void stepInterrupt() {
  if (dir) r += stepangle;
  else r -= stepangle;
}

void dirInterrupt() {
  if (REG_PORT_IN0 & PORT_PA11) dir = false; // check if dir_pin is HIGH
  else dir = true;
}

void output(float theta, int effort) {
  static volatile int angle_1;
  static volatile int angle_2;
  static volatile int v_coil_A;
  static volatile int v_coil_B;

  static int sin_coil_A;
  static int sin_coil_B;
  static int phase_multiplier = 10 * spr / 4;

  //REG_PORT_OUTCLR0 = PORT_PA09; for debugging/timing

<<<<<<< HEAD
  angle_1 = mod((phase_multiplier * theta) , 3600);
  angle_2 = mod((phase_multiplier * theta)+900 , 3600);
  
  sin_coil_A = sin_1[angle_1];
  if (sin_coil_A > 1024) {
    sin_coil_A = sin_coil_A - 65536;
  }
=======
  floatangle = (10000 * ( theta * angle_multiplier + 2.3562) );//0.7854) );// 2.3562) );       //changed to 2.3 for NEMA23,NEMA17 dual..... opposite below
  //floatangle = (10000 * ( theta * 0.87266 + 0.7854) );
>>>>>>> refs/remotes/jcchurch13/multi-file

  sin_coil_B = sin_1[angle_2];
  if (sin_coil_B > 1024) {
    sin_coil_B = sin_coil_B - 65536;
  }

  v_coil_A = ((effort * sin_coil_A) / 1024);
  v_coil_B = ((effort * sin_coil_B) / 1024);

  analogFastWrite(VREF_1, abs(v_coil_A));
  analogFastWrite(VREF_2, abs(v_coil_B));

  if (v_coil_A >= 0)  {
    REG_PORT_OUTSET0 = PORT_PA21;     //write IN_2 HIGH
    REG_PORT_OUTCLR0 = PORT_PA06;     //write IN_1 LOW
  }
  else  {
    REG_PORT_OUTCLR0 = PORT_PA21;     //write IN_2 LOW
    REG_PORT_OUTSET0 = PORT_PA06;     //write IN_1 HIGH
  }

<<<<<<< HEAD
  if (v_coil_B >= 0)  {
    REG_PORT_OUTSET0 = PORT_PA20;     //write IN_4 HIGH
    REG_PORT_OUTCLR0 = PORT_PA15;     //write IN_3 LOW
=======




  floatangle = (10000 * (  theta * angle_multiplier + 0.7854) );//2.3562) );//0.7854) );
  //floatangle = (10000 * ( theta * 0.87266 + 2.3562) );

  intangle = (int)floatangle;
  // modangle = (((intangle % 628) + 628) % 628);
  val2 = effort * lookup_sine(intangle);

  analogFastWrite(VREF_1, abs(val2));

  if (val2 >= 0)  {
    digitalWrite(IN_2, HIGH);
    //     PORTB |= (B00000100);
    digitalWrite(IN_1, LOW);
    //     PORTB &= ~(B00001000);

>>>>>>> refs/remotes/jcchurch13/multi-file
  }
  else  {
    REG_PORT_OUTCLR0 = PORT_PA20;     //write IN_4 LOW
    REG_PORT_OUTSET0 = PORT_PA15;     //write IN_3 HIGH
  }
  //  REG_PORT_OUTSET0 = PORT_PA09;    for debugging/timing
}



void calibrate() {   /// this is the calibration routine

  int encoderReading = 0;     //or float?  not sure if we can average for more res?
  int lastencoderReading = 0;
  int avg = 10;               //how many readings to average

  int iStart = 0;     //encoder zero position index
  int jStart = 0;
  int stepNo = 0;
  
  int fullStepReadings[spr];
    
  int fullStep = 0;
  int ticks = 0;
  float lookupAngle = 0.0;
  SerialUSB.println("Beginning calibration routine...");

  encoderReading = readEncoder();
  dir = true;
  oneStep();
  delay(500);

  if ((readEncoder() - encoderReading) < 0)   //check which way motor moves when dir = true
  {
    SerialUSB.println("Wired backwards");    // rewiring either phase should fix this.  You may get a false message if you happen to be near the point where the encoder rolls over...
    return;
  }

  while (stepNumber != 0) {       //go to step zero
    if (stepNumber > 0) {
      dir = true;
    }
    else
    {
      dir = false;
    }
    oneStep();
    delay(100);
  }
  dir = true;
  for (int x = 0; x < spr; x++) {     //step through all full step positions, recording their encoder readings

    encoderReading = 0;
    delay(100);                         //moving too fast may not give accurate readings.  Motor needs time to settle after each step.

    for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
      encoderReading += readEncoder();
      delay(10);
    }
    encoderReading = encoderReading / avg;

    anglefloat = encoderReading * 0.02197265625;    //encoderReading * 360/16384
    fullStepReadings[x] = encoderReading;
   // SerialUSB.println(fullStepReadings[x], DEC);      //print readings as a sanity check
    SerialUSB.print(100.0*x/spr,1);
    SerialUSB.println("%");
    
    oneStep();
  }
 // SerialUSB.println(" ");
 // SerialUSB.println("ticks:");                        //"ticks" represents the number of encoder counts between successive steps... these should be around 82 for a 1.8 degree stepper
 // SerialUSB.println(" ");
  for (int i = 0; i < spr; i++) {
    ticks = fullStepReadings[mod((i + 1), spr)] - fullStepReadings[mod((i), spr)];
    if (ticks < -15000) {
      ticks += cpr;

    }
    else if (ticks > 15000) {
      ticks -= cpr;
    }
   // SerialUSB.println(ticks);

    if (ticks > 1) {                                    //note starting point with iStart,jStart
      for (int j = 0; j < ticks; j++) {
        stepNo = (mod(fullStepReadings[i] + j, cpr));
        // SerialUSB.println(stepNo);
        if (stepNo == 0) {
          iStart = i;
          jStart = j;
        }

      }
    }

    if (ticks < 1) {                                    //note starting point with iStart,jStart
      for (int j = -ticks; j > 0; j--) {
        stepNo = (mod(fullStepReadings[spr - 1 - i] + j, cpr));
        // SerialUSB.println(stepNo);
        if (stepNo == 0) {
          iStart = i;
          jStart = j;
        }

      }
    }

  }




  SerialUSB.println(" ");                   //The code below generates the lookup table by intepolating between full steps and mapping each encoder count to a calibrated angle
  SerialUSB.println("newLookup:");          //The lookup table is too big to store in volatile memory, so we must generate and print it on the fly
  SerialUSB.println(" ");                   // in the future, we hope to be able to print this directly to non-volatile memory


  for (int i = iStart; i < (iStart + spr + 1); i++) {
    ticks = fullStepReadings[mod((i + 1), spr)] - fullStepReadings[mod((i), spr)];

    if (ticks < -15000) {           //check if current interval wraps over encoder's zero positon
      ticks += cpr;
    }
    else if (ticks > 15000) {
      ticks -= cpr;
    }
    //Here we print an interpolated angle corresponding to each encoder count (in order)
    if (ticks > 1) {              //if encoder counts were increasing during cal routine...

      if (i == iStart) { //this is an edge case
        for (int j = jStart; j < ticks; j++) {
          lookupAngle = 0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0);
          SerialUSB.print(lookupAngle);
          SerialUSB.print(" , ");
        }
      }

      else if (i == (iStart + spr)) { //this is an edge case
        for (int j = 0; j < jStart; j++) {
          lookupAngle = 0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0);
          SerialUSB.print(lookupAngle);
          SerialUSB.print(" , ");
        }
      }
      else {                        //this is the general case
        for (int j = 0; j < ticks; j++) {
          lookupAngle = 0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0);
          SerialUSB.print(lookupAngle);
          SerialUSB.print(" , ");
        }
      }



    }

    else if (ticks < 1) {             //similar to above... for case when encoder counts were decreasing during cal routine
      if (i == iStart) {
        for (int j = - ticks; j > (jStart); j--) {
          lookupAngle = 0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / float(ticks))), 360000.0);
          SerialUSB.print(lookupAngle);
          SerialUSB.print(" , ");
        }
      }
      else if (i == iStart + spr) {
        for (int j = jStart; j > 0; j--) {
          lookupAngle = 0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / float(ticks))), 360000.0);
          SerialUSB.print(lookupAngle);
          SerialUSB.print(" , ");
        }
      }
      else {
        for (int j = - ticks; j > 0; j--) {
          lookupAngle = 0.001 * mod(1000 * (aps * (i) + (aps * ((ticks + j)) / float(ticks))), 360000.0);
          SerialUSB.print(lookupAngle);
          SerialUSB.print(" , ");
        }
      }

    }


  }
  SerialUSB.println(" ");


}


void serialCheck() {        //Monitors serial for commands.  Must be called in routinely in loop for serial interface to work.

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
        if (dir) {
          dir = false;
        }
        else {
          dir = true;
        }
        break;

      case 'w':                //old command
        calibrate();           //cal routine
        break;
        
      case 'c':
        calibrate();           //cal routine
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
        while (SerialUSB.available() == 0)  {}
        r = SerialUSB.parseFloat();
        SerialUSB.println(r);
        break;

      case 'x':
        mode = 'x';           //position loop
        break;

      case 'v':
        mode = 'v';           //velocity loop
        break;

      case 't':
        mode = 't';           //torque loop
        break;

      case 'h':               //hybrid mode
        mode = 'h';
        break;

      case 'q':
        parameterQuery();     // prints copy-able parameters
        break;

      case 'a':             //anticogging
        antiCoggingCal();
        break;

      case 'k':
        parameterEditmain();
        break;

      case 'm':
        serialMenu();
        break;


      default:
        break;
    }
  }

}


void parameterQuery() {         //print current parameters in a format that can be copied directly in to Parameters.cpp
  SerialUSB.println(' ');
  SerialUSB.println("----Current Parameters-----");
  SerialUSB.println(' ');
  SerialUSB.println(' ');

  SerialUSB.print("volatile float Ts = ");
  SerialUSB.print(Ts, DEC);
  SerialUSB.println(";");
  SerialUSB.println(' ');

  SerialUSB.print("volatile float pKp = ");
  SerialUSB.print(pKp);
  SerialUSB.println(";");

  SerialUSB.print("volatile float pKi = ");
  SerialUSB.print(pKi);
  SerialUSB.println(";");

  SerialUSB.print("volatile float pKd = ");
  SerialUSB.print(pKd);
  SerialUSB.println(";");

  SerialUSB.println(' ');

  SerialUSB.print("volatile float vKp = ");
  SerialUSB.print(vKp);
  SerialUSB.println(";");

  SerialUSB.print("volatile float vKi = ");
  SerialUSB.print(vKi / Ts);
  SerialUSB.println(" * Ts;");

  SerialUSB.print("volatile float vKd = ");
  SerialUSB.print(vKd * Ts);
  SerialUSB.println(" / Ts;");

  SerialUSB.println(' ');

  SerialUSB.println("const PROGMEM float lookup[] = {");
  for (int i = 0; i < 16384; i++) {
    SerialUSB.print(lookup_angle(i));
    SerialUSB.print(", ");
  }
  SerialUSB.println("");
  SerialUSB.println("};");

}


float lookup_angle(int n)
{
  float a_out;
  a_out = pgm_read_float_near(lookup + n);
  return a_out;
}

void oneStep() {           /////////////////////////////////   oneStep    ///////////////////////////////

  if (!dir) {
    stepNumber += 1;
  }
  else {
    stepNumber -= 1;
  }
<<<<<<< HEAD
=======
  // step_state = ((((stepNumber) % 4) + 4) % 4); // arduino mod does not wrap for negative....

  //output(1.8 * step_state, 128); //1.8 = 90/50

  output(aps * stepNumber, 50); //1.8 = 90/50
>>>>>>> refs/remotes/jcchurch13/multi-file

  //output(1.8 * stepNumber, 64); //updata 1.8 to aps..., second number is control effort
  output(aps * stepNumber, (int)(0.33 * uMAX));
  delay(10);
}

int readEncoder()           //////////////////////////////////////////////////////   READENCODER   ////////////////////////////
{
  long angleTemp;
  // REG_PORT_OUTCLR0 = PORT_PB08;  //
  digitalWrite(chipSelectPin, LOW);

  byte b1 = SPI.transfer(0xFF);
  byte b2 = SPI.transfer(0xFF);

  angleTemp = (((b1 << 8) | b2) & 0B0011111111111111);
  //  SerialUSB.println((angle & 0B0011111111111111)*0.02197265625);

  // REG_PORT_OUTSET0 = PORT_PB08;  //
  digitalWrite(chipSelectPin, HIGH);
  return angleTemp;
}



void readEncoderDiagnostics()           //////////////////////////////////////////////////////   READENCODERDIAGNOSTICS   ////////////////////////////
{
  long angleTemp;
  digitalWrite(chipSelectPin, LOW);

  ///////////////////////////////////////////////READ DIAAGC (0x3FFC)
  SerialUSB.println("------------------------------------------------");

  SerialUSB.println("Checking AS5047 diagnostic and error registers");
  SerialUSB.println("See AS5047 datasheet for details");
  SerialUSB.println(" ");
  ;

  SPI.transfer(0xFF);
  SPI.transfer(0xFC);

  digitalWrite(chipSelectPin, HIGH);
  delay(1);
  digitalWrite(chipSelectPin, LOW);

  byte b1 = SPI.transfer(0xC0);
  byte b2 = SPI.transfer(0x00);

  SerialUSB.print("Check DIAAGC register (0x3FFC) ...  ");
  SerialUSB.println(" ");

  angleTemp = (((b1 << 8) | b2) & 0B1111111111111111);
  SerialUSB.println((angleTemp | 0B1110000000000000000 ), BIN);

  if (angleTemp & (1 << 14))    SerialUSB.println("  Error occurred  ");

  if (angleTemp & (1 << 11))    SerialUSB.println("  MAGH - magnetic field strength too high, set if AGC = 0x00. This indicates the non-linearity error may be increased");

  if (angleTemp & (1 << 10))    SerialUSB.println("  MAGL - magnetic field strength too low, set if AGC = 0xFF. This indicates the output noise of the measured angle may be increased");

  if (angleTemp & (1 << 9))     SerialUSB.println("  COF - CORDIC overflow. This indicates the measured angle is not reliable");

  if (angleTemp & (1 << 8))     SerialUSB.println("  LF - offset compensation completed. At power-up, an internal offset compensation procedure is started, and this bit is set when the procedure is completed");

  if (!((angleTemp & (1 << 14)) | (angleTemp & (1 << 11)) | (angleTemp & (1 << 10)) | (angleTemp & (1 << 9))))  SerialUSB.println("Looks good!");
  SerialUSB.println(" ");


  digitalWrite(chipSelectPin, HIGH);
  delay(1);
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(0x40);
  SPI.transfer(0x01);
  digitalWrite(chipSelectPin, HIGH);

  delay(1);
  digitalWrite(chipSelectPin, LOW);

  b1 = SPI.transfer(0xC0);
  b2 = SPI.transfer(0x00);


  SerialUSB.print("Check ERRFL register (0x0001) ...  ");
  SerialUSB.println(" ");



  angleTemp = (((b1 << 8) | b2) & 0B1111111111111111);
  SerialUSB.println((angleTemp | 0B1110000000000000000 ), BIN);

  if (angleTemp & (1 << 14)) {
    SerialUSB.println("  Error occurred  ");
  }
  if (angleTemp & (1 << 2)) {
    SerialUSB.println("  parity error ");
  }
  if (angleTemp & (1 << 1)) {
    SerialUSB.println("  invalid register  ");
  }
  if (angleTemp & (1 << 0)) {
    SerialUSB.println("  framing error  ");
  }
  if (!((angleTemp & (1 << 14)) | (angleTemp & (1 << 2)) | (angleTemp & (1 << 1)) | (angleTemp & (1 << 0))))  SerialUSB.println("Looks good!");

  SerialUSB.println(" ");

  digitalWrite(chipSelectPin, HIGH);


  delay(1);

}


void print_angle()                ///////////////////////////////////       PRINT_ANGLE   /////////////////////////////////
{
  a = 0;
  delay(100);
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


void receiveEvent(int howMany)
{
  while (1 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    SerialUSB.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  SerialUSB.println(x);         // print the integer
  r = 0.1 * ((float)x);
}

int mod(int xMod, int mMod) {
  return (xMod % mMod + mMod) % mMod;
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

//float lookup_sine(int m)        /////////////////////////////////////////////////  LOOKUP_SINE   /////////////////////////////
//{
//  float b_out;
//  m = (0.01 * (((m % 62832) + 62832) % 62832)) + 0.5; //+0.5 for rounding
//  if (m > 314) {
//    m = m - 314;
//    b_out = -pgm_read_float_near(sine_lookup + m);
//  }
//  else b_out = pgm_read_float_near(sine_lookup + m);
//
//  return b_out;
//}


void setupTCInterrupts() {  // configure the controller interrupt

  // Enable GCLK for TC4 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
  while (GCLK->STATUS.bit.SYNCBUSY);

  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // Set Timer counter Mode to 16 bits
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;   // Set perscaler
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CC[0].reg = (int)( round(48000000 / Fs)); //0x3E72; //0x4AF0;
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.INTENSET.reg = 0;              // disable all interrupts
  TC5->COUNT16.INTENSET.bit.OVF = 1;          // enable overfollow
  TC5->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0

  NVIC_SetPriority(TC5_IRQn, 1);              //Set interrupt priority

  // Enable InterruptVector
  NVIC_EnableIRQ(TC5_IRQn);

  // Enable TC
  //  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  //  WAIT_TC16_REGS_SYNC(TC5)
}

void enableTCInterrupts() {   //enables the controller interrupt ("closed loop mode")
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    //Enable TC5
  WAIT_TC16_REGS_SYNC(TC5)                      //wait for sync
}

void disableTCInterrupts() {  //disables the controller interrupt ("closed loop mode")
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC5
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync
}


void antiCoggingCal() {       //This is still under development...  The idea is that we can calibrate out the stepper motor's detent torque by measuring the torque required to hold all possible positions.
  SerialUSB.println(" -----------------BEGIN ANTICOGGING CALIBRATION!----------------");
  mode = 'x';
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





void parameterEditmain() {

  SerialUSB.println();
  SerialUSB.println("Edit parameters:");
  SerialUSB.println();
  SerialUSB.println("p ----- proportional loop");
  SerialUSB.println("v ----- velocity loop");
  SerialUSB.println("o ----- other");
  SerialUSB.println("q ----- quit");
  SerialUSB.println();

  while (SerialUSB.available() == 0)  {}
  char inChar2 = (char)SerialUSB.read();

  switch (inChar2) {
    case 'p':
      {
        parameterEditp();
      }
      break;

    case 'v':
      {
        parameterEditv();
      }
      break;

    case 'o':
      {
        parameterEdito();
      }
      break;
    default:
      {}
      break;



  }
}

void parameterEditp() {


  SerialUSB.println("Edit position loop gains:");
  SerialUSB.println();
  SerialUSB.print("p ----- pKp = ");
  SerialUSB.println(pKp, DEC);
  SerialUSB.print("i ----- pKi = ");
  SerialUSB.println(pKi, DEC);
  SerialUSB.print("d ----- pKd = ");
  SerialUSB.println(pKd, DEC);
  SerialUSB.println("q ----- quit");
  SerialUSB.println();

  while (SerialUSB.available() == 0)  {}
  char inChar3 = (char)SerialUSB.read();

  switch (inChar3) {
    case 'p':
      {
        SerialUSB.println("pKp = ?");
        while (SerialUSB.available() == 0)  {}
        pKp = SerialUSB.parseFloat();
        SerialUSB.print("new pKp = ");
        SerialUSB.println(pKp, DEC);
      }
      break;
    case 'i':
      {
        SerialUSB.println("pKi = ?");
        while (SerialUSB.available() == 0)  {}
        pKi = SerialUSB.parseFloat();
        SerialUSB.print("new pKi = ");
        SerialUSB.println(pKi, DEC);
      }
      break;
    case 'd':
      {
        SerialUSB.println("pKd = ?");
        while (SerialUSB.available() == 0)  {}
        pKd = SerialUSB.parseFloat();
        SerialUSB.print("new pKd = ");
        SerialUSB.println(pKd, DEC);
      }
      break;
    default:
      {}
      break;
  }
}

void parameterEditv() {
  SerialUSB.println("Edit velocity loop gains:");
  SerialUSB.println();
  SerialUSB.print("p ----- vKp = ");
  SerialUSB.println(vKp, DEC);
  SerialUSB.print("i ----- vKi = ");
  SerialUSB.println(vKi, DEC);
  SerialUSB.print("d ----- vKd = ");
  SerialUSB.println(vKd, DEC);
  SerialUSB.println("q ----- quit");
  SerialUSB.println();

  while (SerialUSB.available() == 0)  {}
  char inChar4 = (char)SerialUSB.read();

  switch (inChar4) {
    case 'p':
      {
        SerialUSB.println("vKp = ?");
        while (SerialUSB.available() == 0)  {}
        vKp = SerialUSB.parseFloat();
        SerialUSB.print("new vKp = ");
        SerialUSB.println(vKp, DEC);
      }
      break;
    case 'i':
      {
        SerialUSB.println("vKi = ?");
        while (SerialUSB.available() == 0)  {}
        vKi = SerialUSB.parseFloat();
        SerialUSB.print("new vKi = ");
        SerialUSB.println(vKi, DEC);
      }
      break;
    case 'd':
      {
        SerialUSB.println("vKd = ?");
        while (SerialUSB.available() == 0)  {}
        vKd = SerialUSB.parseFloat();
        SerialUSB.print("new vKd = ");
        SerialUSB.println(vKd, DEC);
      }
      break;
    default:
      {}
      break;
  }
}

void parameterEdito() {


  SerialUSB.println("Edit other parameters:");
  SerialUSB.println();
  SerialUSB.print("p ----- PA = ");
  SerialUSB.println(PA, DEC);
  SerialUSB.println();


  while (SerialUSB.available() == 0)  {}
  char inChar3 = (char)SerialUSB.read();

  switch (inChar3) {
    case 'p':
      {
        SerialUSB.println("PA = ?");
        while (SerialUSB.available() == 0)  {}
        PA = SerialUSB.parseFloat();
        SerialUSB.print("new PA = ");
        SerialUSB.println(PA, DEC);
      }

      break;
    default:
      {}
      break;
  }
}



void hybridControl() {        //still under development

  static int missed_steps = 0;
  static float iLevel = 0.6;  //hybrid stepping current level.  In this mode, this current is continuous (unlike closed loop mode). Be very careful raising this value as you risk overheating the A4954 driver!
  static float rSense = 0.15;

<<<<<<< HEAD
  if (yw < r - aps) {
    missed_steps -= 1;
  }
  else if (yw > r + aps) {
=======
  a = readEncoder();
  y = lookup_angle(a);
  if ((y - y_1) < -180.0) {
    wrap_count += 1;
  }
  else if ((y - y_1) > 180.0) {
    wrap_count -= 1;
  }
  y_1 = y; 

  yw = (y + (360.0 * wrap_count));
  
  if (yw < 0.1125*step_count-aps) {
    missed_steps -= 1;
  }
  else if (yw > 0.1125*step_count+aps) {
>>>>>>> refs/remotes/jcchurch13/multi-file
    missed_steps += 1;
  }

  output(0.1125 * (-(r - missed_steps)), (255 / 3.3) * (iLevel * 10 * rSense));

}

void positionControl() {
  e = (r - yw);

  ITerm += (pKi * e);                             //Integral wind up limit
  if (ITerm > 150) ITerm = 150;
  else if (ITerm < -150) ITerm = -150;

  u = ((pKp * e) + ITerm - (pKd * (yw - yw_1))); //ARDUINO library style PID controller
}

void velocityControl() {
  e = (r - ((yw - yw_1) * Fs * 0.16666667)); //error in degrees per rpm (sample frequency in Hz * (60 seconds/min) / (360 degrees/rev) )

  ITerm += (vKi * e);                 //Integral wind up limit
  if (ITerm > 200) ITerm = 200;
  else if (ITerm < -200) ITerm = -200;

  u = ((vKp * e) + ITerm - (vKd * (yw - yw_1)));//+ lookup_force(a)-20; //ARDUINO library style PID controller
}

void torqueControl() {
  u = 1.0 * r ;     //directly set control effort
}

void serialMenu() {
  SerialUSB.println("");
  SerialUSB.println("");
  SerialUSB.println("----- Mechaduino 0.1 -----");
  //SerialUSB.print("Identifier: ");
  //SerialUSB.println(identifier);
  SerialUSB.println("");
  SerialUSB.println("");
  SerialUSB.println("Main menu");
  SerialUSB.println("");
  SerialUSB.println(" s  -  step");
  SerialUSB.println(" d  -  dir");
  SerialUSB.println(" p  -  print angle");
  SerialUSB.println("");
  SerialUSB.println(" c  -  write new calibration table");
  SerialUSB.println(" e  -  check encoder diagnositics");
  SerialUSB.println(" q  -  parameter query");
  SerialUSB.println("");
  SerialUSB.println(" x  -  position mode");
  SerialUSB.println(" v  -  velocity mode");
  SerialUSB.println(" x  -  torque mode");
  SerialUSB.println("");
  SerialUSB.println(" y  -  enable control loop");
  SerialUSB.println(" n  -  disable control loop");
  SerialUSB.println(" r  -  enter new setpoint");
  SerialUSB.println("");
  // SerialUSB.println(" j  -  step response");
  SerialUSB.println(" k  -  edit controller gains");
  SerialUSB.println(" m  -  print main menu");
  // SerialUSB.println(" f  -  get max loop frequency");
  SerialUSB.println("");
}
void sineGen() {
  for (int x = 0; x <= 3600; x++) {
    sin_1[x] = round(1024.0 * sin((3.14159265358979 * ((x * 0.1 / 180.0) + 0.25))));
   // sin_2[x] = round(1024.0 * sin((3.14159265358979 * ((x * 0.1 / 180.0) + 0.75))));    
  }
   //   for (int x = 0; x<=3600;x++){     //print out commutation table
   //   SerialUSB.println(sin_1[x]);
   //   SerialUSB.print(",");
   //   SerialUSB.print(sin_2[x]);
   // }

}



//void stepResponse() {     // not done yet...
//  enableTCInterrupts();     //start in closed loop mode
//  mode = 'x';
//  r = 0;
//  print_yw = true;
//  delay(100);
//  r = 180.0;
//  delay(400);
//  print_yw = false;
//  disableTCInterrupts();
//
//}


