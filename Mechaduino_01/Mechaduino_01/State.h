//Contains the declaration of the state variables for the control loop  

#ifndef __STATE_H__
#define __STATE_H__

//interrupt vars

extern volatile float ei;
extern volatile int U;  		//control effort (abs)
extern volatile float r;  		//setpoint
extern volatile float y;  		//measured angle
extern volatile float y_1; 		//last measured angle 
extern volatile float yw; 		//measured angle with wrap count
extern volatile float yw_1;		//last measured angle with wrap count 
extern volatile float vw;
extern volatile float vw_1;

extern volatile float p;  		// proportional effort
extern volatile float i;  		// integral effort
extern volatile float PA;  		//

extern volatile float u;  		//real control effort (not abs)
extern volatile float e;  		//e = r-y (error)
extern volatile float u_1;		//last control effort (not abs)
extern volatile float e_1;		//last error
extern volatile float u_2;	
extern volatile float e_2;
extern volatile float u_3;
extern volatile float e_3;

//Kalman Parameters//

//Predicted State
extern volatile float xhat1;   //Position prediction
extern volatile float xhat2;   //Velocity prediction

//State Best Guess
extern volatile float xb1;
extern volatile float xb2;

//Prediction Matrix 
extern volatile float F11; 
extern volatile float F12; 
extern volatile float F21; 
extern volatile float F22; 

//Prediction Uncertainty
extern volatile float P11;
extern volatile float P12;
extern volatile float P21;
extern volatile float P22;

//Best Guess Uncertainty
extern volatile float Pb11;
extern volatile float Pb12;
extern volatile float Pb21;
extern volatile float Pb22;

//Kalman Gain
extern volatile float K11; 
extern volatile float K12; 
extern volatile float K21; 
extern volatile float K22; 

//Process Error Term 
extern volatile float Q11;		
extern volatile float Q12;		
extern volatile float Q21;		
extern volatile float Q22;

//Observation Error Term
extern volatile float R11;		
extern volatile float R12;		
extern volatile float R21;		
extern volatile float R22;		



extern volatile long counter;
extern volatile long wrap_count;

extern volatile float ITerm; 	//INTEGRAL TERM FOR PID CONTROL: Incremental sum of (e times Ki) 
extern volatile int uMAX;		//MAX CONTROL EFFORT, AS DEFINED BY MAX CURRENT LIMIT 

extern volatile char mode;		//MODE SELECTION

extern int dir;
extern int stepNumber;
extern void output(float,int);




//___________________________________

extern const float pi;		//THIS IS PI 
extern const int  half;		//128;

extern float new_angle; 	//input angle
extern float current_angle; //current angle
extern float diff_angle;
extern int val1;
extern int val2;


#endif
