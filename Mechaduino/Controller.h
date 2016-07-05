//
// Created by marco on 7/4/16.
//

#ifndef MECHADUINO_CONTROLLER_H
#define MECHADUINO_CONTROLLER_H

#include <wiring_digital.h>
#include <delay.h>
#include <SPI.h>
#include "State.h"
#include "ParameterEditor.h"
#include "Utils.h"
#include "Controller.h"
#include "Pins.h"
#include "Mechaduino_functions.h"

/*
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
class Controller {
    static const int spr = 200; //  200 steps per revolution
    static constexpr float aps = 360.0 / spr; // angle per step
    static const int cpr = 16384; //counts per rev

    ParameterEditor parameterEditor = ParameterEditor();
    float anglefloat = 0;

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

    void readEncoderDiagnostics()           //////////////////////////////////////////////////////   READENCODERDIAGNOSTICS   ////////////////////////////
    {
        long angleTemp;
        digitalWrite(CHIP_SELECT, LOW);

        ///////////////////////////////////////////////READ DIAAGC (0x3FFC)
        SerialUSB.print("DIAAGC (0x3FFC)   ");

        SPI.transfer(0xFF);
        SPI.transfer(0xFC);
        digitalWrite(CHIP_SELECT, HIGH);

        delay(1);
        digitalWrite(CHIP_SELECT, LOW);

        byte b1 = SPI.transfer(0xC0);
        byte b2 = SPI.transfer(0x00);


        angleTemp = (((b1 << 8) | b2) & 0B1111111111111111);
        SerialUSB.print((angleTemp | 0B1110000000000000000), BIN);

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

        digitalWrite(CHIP_SELECT, HIGH);

        delay(1);

        digitalWrite(CHIP_SELECT, LOW);
        ///////////////////////////////////////////////READ ERRFL (0x0001)
        SerialUSB.print("ERRFL (0x0001)   ");

        SPI.transfer(0x40);
        SPI.transfer(0x01);
        digitalWrite(CHIP_SELECT, HIGH);

        delay(1);
        digitalWrite(CHIP_SELECT, LOW);

        b1 = SPI.transfer(0xC0);
        b2 = SPI.transfer(0x00);


        angleTemp = (((b1 << 8) | b2) & 0B1111111111111111);
        SerialUSB.print((angleTemp | 0B1110000000000000000), BIN);

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

        digitalWrite(CHIP_SELECT, HIGH);

        delay(1);
    }


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

        if ((readEncoder() - encoderReading) < 0) {
            //dir = 0;
            SerialUSB.println("Wired backwards");
            return;
        }

        while (stepNumber != 0) {
            if (stepNumber > 0) {
                dir = 1;
            }
            else {
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

    void enableTCInterrupts() {
        TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    //Enable TC5
        WAIT_TC16_REGS_SYNC(TC5)                      //wait for sync
    }


    void disableTCInterrupts() {
        TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC5
        WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync
    }

public:
    void serialCheck() {
        if (!SerialUSB.available()) {
            return;
        }
        char inChar = (char) SerialUSB.read();

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
};


#endif //MECHADUINO_CONTROLLER_H
