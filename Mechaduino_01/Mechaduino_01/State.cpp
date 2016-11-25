  //Contains the declaration of the state variables for the control loop  


//interrupt vars

volatile float ei = 0.0;
volatile int U = 0;  //control effort (abs)
volatile float r = 0.0;  //setpoint
volatile float y = 0.0;  // measured angle
volatile float yw = 0.0;
volatile float yw_1 = 0.0;
volatile float e = 0.0;  // e = r-y (error)
volatile float p = 0.0;  // proportional effort
volatile float i = 0.0;  // integral effort
<<<<<<< HEAD
volatile float PA = 1.8;  // Phase advance...1.8 for 200 steps per rev, 0.9 for 400
=======
volatile float PA = 1.8;// Phase advance...1.8 for 200 steps per rev, 0.9 for 400
>>>>>>> refs/remotes/jcchurch13/multi-file

volatile float u = 0.0;  //real control effort (not abs)
volatile float u_1 = 0.0;   //value of u at previous time step, etc...
volatile float e_1 = 0.0;
volatile float u_2 = 0.0;
volatile float e_2 = 0.0;
volatile float u_3 = 0.0;
volatile float e_3 = 0.0;
volatile long counter = 0;

volatile long wrap_count = 0;  //keeps track of how many revolutions the motor has gone though (so you can command angles outside of 0-360)
volatile float y_1 = 0;



const float iMAX = 1.0;  //Be careful adjusting this.  While the A4954 driver is rated for 2.0 Amp peak currents, it cannot handle these currents continuously.  Depending on how you operate the Mechaduino, you may be able to safely raise this value...please refer to the A4954 datasheet for more info
const float rSense = 0.150;

volatile int uMAX = (255/3.3)*(iMAX*10*rSense); //1023 for 10 bit, must also edit analogFastWrite

volatile float ITerm;

volatile char mode;
volatile bool dir = true;  
//___________________________________

const float pi= 3.14159265359;
const int  half = 134;//128;

//float new_angle = 0.0; //input angle
//float current_angle = 0.0; //current angle
//float diff_angle = 0.0;

int val1 = 0;
int val2 = 0;

bool print_yw = false;

