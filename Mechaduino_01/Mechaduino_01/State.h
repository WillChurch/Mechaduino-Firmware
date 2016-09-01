//Contains the declaration of the state variables for the control loop  

#ifndef __STATE_H__
#define __STATE_H__

//interrupt vars

extern volatile float ei;
extern volatile int U;  	//control effort (abs)
extern volatile float r;  	//setpoint
extern volatile float y;  	//measured angle
extern volatile float y_1; 	//last measured angle 
extern volatile float yw; 	//measured angle with wrap count
extern volatile float yw_1; //last measured angle with wrap count 	

extern volatile float p;  	// proportional effort
extern volatile float i;  	// integral effort
extern volatile float PA;  	//

extern volatile float u;  	//real control effort (not abs)
extern volatile float e;  	//e = r-y (error)
extern volatile float u_1;	//last control effort (not abs)
extern volatile float e_1;	//last error
extern volatile float u_2;	
extern volatile float e_2;
extern volatile float u_3;
extern volatile float e_3;

extern volatile long counter;
extern volatile long wrap_count;

extern volatile float ITerm; 	//INTEGRAL TERM FOR PID CONTROL: Incremental sum of (e times Ki) 
extern volatile int uMAX;		//MAX CONTROL EFFORT, AS DEFINED BY MAX CURRENT LIMIT 

extern volatile char mode;

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
