//Contains the declaration of the state variables for the control loop  


//interrupt vars

volatile float ei = 0.0;
volatile int U = 0;  			//control effort (abs)
volatile float r = 0.0;  		//setpoint
volatile float y = 0.0;  		//measured angle
volatile float y_1 = 0;			//last measured angle
volatile float yw = 0.0;		//measured angle, with wrap count 
volatile float yw_1 = 0.0;		//last measured angle, with wrap count 

volatile float p = 0.0;  		// proportional effort
volatile float i = 0.0;  		// integral effort
volatile float PA = 1.8;  		//

volatile float u = 0.0;  		//real control effort (not abs)
volatile float e = 0.0;  		//e = r-y (error)
volatile float u_1 = 0.0;		//last control effort (not abs) 
volatile float e_1 = 0.0;		//last error 
volatile float u_2 = 0.0;
volatile float e_2 = 0.0;
volatile float u_3 = 0.0;
volatile float e_3 = 0.0;

//Kalman Parameters//

//Predicted State
volatile float xhat1 = 0.0;   //Position prediction
volatile float xhat2 = 0.0;   //Velocity prediction
volatile float xhat1_1 = 0.0;   
volatile float xhat2_1 = 0.0; 

//State Best Guess
volatile float xb1 = 0.0;
volatile float xb2 = 0.0;
volatile float xb1_1 = 0.0; 

//Prediction Matrix 
volatile float F11 = 1.0; 
volatile float F12 = 0.0003333333; 	//This will be Ts, placeholder for now. 
volatile float F21 = 0.0; 
volatile float F22 = 1.0; 

//Prediction Uncertainty
volatile float P11 = 0.0;
volatile float P12 = 0.0;
volatile float P21 = 0.0;
volatile float P22 = 0.0;
volatile float P11_1 = 0.0;
volatile float P12_1 = 0.0;
volatile float P21_1 = 0.0;
volatile float P22_1 = 0.0;

//Best Guess Uncertainty
volatile float Pb11 = 0.0;
volatile float Pb12 = 0.0;
volatile float Pb21 = 0.0;
volatile float Pb22 = 0.0;

//Kalman Gain
volatile float K11 = 0.0; 
volatile float K12 = 0.0; 
volatile float K21 = 0.0; 
volatile float K22 = 0.0; 

//Process Error Term 
volatile float Q11 = 0.01;		
volatile float Q12 = 0.0;		
volatile float Q21 = 0.0;		
volatile float Q22 = 0.01;
		
//Observation Error Term
volatile float R11 = 0.01;		
volatile float R12 = 0.0;		
volatile float R21 = 0.0;		
volatile float R22 = 0.01;		


volatile long counter = 0;
volatile long wrap_count = 0;


const float iMAX = 1.0;				//CURRENT LIMIT PARAMETER
const float rSense = 0.150;			//SENSE RESISTOR VALUE, OHMS

volatile int uMAX = (255/3.3)*(iMAX*10*rSense);		//MAX CONTROL EFFORT, AS DEFINED BY MAX CURRENT LIMIT 

volatile float ITerm;				//INTEGRAL TERM FOR PID CONTROL: Incremental sum of (e times Ki)

volatile char mode;


//___________________________________

const float pi = 3.14159;			//THIS IS PI
const int  half = 134;//128;

float new_angle = 0.0; 				//input angle
float current_angle = 0.0; 			//current angle
float diff_angle = 0.0;
int val1 = 0;
int val2 = 0;
