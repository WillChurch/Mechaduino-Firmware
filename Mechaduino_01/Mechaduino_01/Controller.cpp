//Contains TC5 Controller definition


#include <SPI.h>

#include "State.h" 
#include "Utils.h"
#include "Parameters.h"


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

	
	//PROCESS WRAPAROUND
    if ((y - y_1) < -180.0) {
      wrap_count += 1;
    }
    else if ((y - y_1) > 180.0) {
      wrap_count -= 1;
    }
    //y_1 = y;  pushed lower

    yw = (y + (360.0 * wrap_count));
    vw = (yw-yw_1)/Ts ; 

    switch (mode) {

      case 'x':
        e = (r - yw);

        ITerm += (pKi * e);
        if (ITerm > 150) ITerm = 150;
        else if (ITerm < -150) ITerm = -150;

        u = ((pKp * e) + ITerm - (pKd * (yw - yw_1))); //ARDUINO library style
        //u = u+lookup_force(a)-20;
        //   u = u_1 + cA*e + cB*e_1 + cC*e_2;     //ppt linked in octave script

        //  u = 20*e;//

        break;

      case 'v':
        SerialUSB.println("");    
    		//xhat = F * xhat_1 ; Calculate prediction
    		xhat1 = ((F11 * xb1) + ( F12 * xb2));
    		xhat2 = ((F21 * xb1) + ( F22 * xb2));
       
        SerialUSB.println(yw);
        SerialUSB.println(xhat1);
        SerialUSB.println(vw);
    		SerialUSB.println(xhat2);
       
    		//P = F * P_1 * F^T + Q ; Calculate Covariance of Prediction (Error)
    		P11 = (F11*((F11*Pb11)+(F12*Pb21)) + F12*((F11*Pb12)+(F12*Pb22))) + Q11; 
    		P12 = (F21*((F11*Pb11)+(F12*Pb21)) + F22*((F11*Pb12)+(F12*Pb22))) + Q12; 
    		P21 = (F11*((F21*Pb11)+(F22*Pb21)) + F12*((F21*Pb12)+(F22*Pb22))) + Q21;
    		P22 = (F21*((F21*Pb11)+(F22*Pb21)) + F22*((F21*Pb12)+(F22*Pb22))) + Q22; 
    		
    		//K = P * (P + R)^-1 ; Calculate Kalman Gains 
    		K11 = (P11*(P22+R22)-P12*(P21+R21))/(((P22+R22)*(P11+R11)) - ((P21+R21)*(P12+R12))); 
    		K12 = (P12*(P11+R11)-P11*(P12+R12))/(((P22+R22)*(P11+R11)) - ((P21+R21)*(P12+R12)));
    		K21 = (P21*(P22+R22)-P22*(P21+R21))/(((P22+R22)*(P11+R11)) - ((P21+R21)*(P12+R12)));
    		K22 = (P22*(P11+R11)-P21*(P12+R12))/(((P22+R22)*(P11+R11)) - ((P21+R21)*(P12+R12)));
			
    		SerialUSB.println(P11);
        SerialUSB.println(P12);
        SerialUSB.println(P21);
        SerialUSB.println(P22);        
                      
    		//xb = xhat + K * (y - xhat) ; Combine prediction and measurement for best guess. 
    		xb1 = xhat1 + (K11*(yw - xhat1) + K12*(vw - xhat2));
    		xb2 = xhat2 + (K21*(yw - xhat1) + K22*(vw - xhat2));
        SerialUSB.println(xb1);
        SerialUSB.println(xb2);
    		
    		//Pb = P - K * P ; Calculate Covariance of Best Guess (Error)
    		Pb11 = P11 - (K11*P11 + K12*P21);
    		Pb12 = P12 - (K11*P12 + K12*P22);
    		Pb21 = P21 - (K21*P11 + K22*P21);
    		Pb22 = P22 - (K21*P12 + K22*P22);

        e = r - xb2; 									        //ERROR, DEGREES/SEC, Will be changed to RPM eventually. 
  
        ITerm += (vKi * e);								    //ADD TO RUNNING INTEGRAL ERROR TERM
		
        if (ITerm > 200) ITerm = 200;					
        else if (ITerm < -200) ITerm = -200;

        u = ((vKp * e) + ITerm - (vKd * (xb1 - xhat1))); 	//SUM PID CONTROL EFFORTS 
        
        SerialUSB.println(u);
        break;

      case 't':
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



    if (u > uMAX) {                          //saturation limits max current command
      u = uMAX;
    }
    else if (u < -uMAX) {
      u = -uMAX;
    }



    U = abs(u);       //+lookup_force((((a-4213)%16384)+16384)%16384)-6); ///p);//+i);
	
	
	//IF CONTROL EFFORT IS BELOW THRESHOLD, ILLUMINATE LED
    if (abs(e) < 0.1) {
      digitalWrite(pulse, HIGH);
     //   SerialUSB.println(r);
    }
    else  {
      digitalWrite(pulse, LOW);
    }

    output(-y, U);  //-y
	
	
	//END OF LOOP, SHIFT PARAMETERS BACK ONE TIMESTEP
	  e_3 = e_2;
    e_2 = e_1;
    e_1 = e;
    u_3 = u_2;
    u_2 = u_1;
    u_1 = u;
    yw_1 = yw;
    y_1 = y;
    vw_1 = vw;


    TC5->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
  }


}
