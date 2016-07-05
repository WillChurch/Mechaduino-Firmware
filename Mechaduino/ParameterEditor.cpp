
#include "ParameterEditor.h"
#include <USB/USBAPI.h>
#include "State.h"
#include "Utils.h"

void ParameterEditor::parameterEditMain() {

    SerialUSB.println();
    SerialUSB.println("Edit parameters:");
    SerialUSB.println();
    SerialUSB.println("p ----- proportional loop");
    SerialUSB.println("v ----- velocity loop");
    SerialUSB.println("o ----- other");
    SerialUSB.println("q ----- quit");
    SerialUSB.println();

    waitSerialUSB();
    char inChar2 = (char) SerialUSB.read();

    switch (inChar2) {
        case 'p':
            parameterEditp();
            break;

        case 'v':
            parameterEditv();
            break;

        case 'o':
            parameterEdito();
            break;

        default:
            break;
    }
}

void ParameterEditor::parameterQuery() {
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

    SerialUSB.print("cvolatile float vKp = ");
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

void ParameterEditor::parameterEdito() {
    SerialUSB.println("Edit other parameters:");
    SerialUSB.println();
    SerialUSB.print("p ----- PA = ");
    SerialUSB.println(PA, DEC);
    SerialUSB.println();

    waitSerialUSB();
    char inChar3 = (char) SerialUSB.read();

    switch (inChar3) {
        case 'p':
            SerialUSB.println("PA = ?");
            waitSerialUSB();
            PA = SerialUSB.parseFloat();
            break;

        default:
            break;
    }
}

void ParameterEditor::parameterEditv() {
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

    waitSerialUSB();
    char inChar4 = (char) SerialUSB.read();

    switch (inChar4) {
        case 'p': {
            SerialUSB.println("vKp = ?");
            waitSerialUSB();
            vKp = SerialUSB.parseFloat();
        }
            break;
        case 'i': {
            SerialUSB.println("vKi = ?");
            waitSerialUSB();
            vKi = SerialUSB.parseFloat();
        }
            break;
        case 'd': {
            SerialUSB.println("vKd = ?");
            waitSerialUSB();
            vKd = SerialUSB.parseFloat();
        }
            break;
        default: {
        }
            break;
    }
}

void ParameterEditor::parameterEditp() {
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

    waitSerialUSB();
    char inChar3 = (char) SerialUSB.read();

    switch (inChar3) {
        case 'p': {
            SerialUSB.println("pKp = ?");
            waitSerialUSB();
            pKp = SerialUSB.parseFloat();
        }
            break;
        case 'i': {
            SerialUSB.println("pKi = ?");
            waitSerialUSB();
            pKi = SerialUSB.parseFloat();
        }
            break;
        case 'd': {
            SerialUSB.println("pKd = ?");
            waitSerialUSB();
            pKd = SerialUSB.parseFloat();
        }
            break;
        default: {
        }
            break;
    }
}