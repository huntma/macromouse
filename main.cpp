#include "mbed.h"

DigitalOut onled(PC_11);

Serial pc(PA_9, PA_10); //set serial

//motor pins
PwmOut leftF(PC_7); //for rat pa7
PwmOut leftR(PC_6); //for rat pb6
PwmOut rightR(PC_9); //for rat: pc7     
PwmOut rightF(PC_8); //for rat: pb10

//set motor encoders
InterruptIn encoderLeftR(PA_0);
InterruptIn encoderLeftF(PA_1);
InterruptIn encoderRightR(PA_2);
InterruptIn encoderRightF(PA_3);

//IR receivers and corresponding pins (directions relative to forward-facing rat)
AnalogIn ir_r1(PC_3);       //far left
AnalogIn ir_r2(PC_2);       //mid left
AnalogIn ir_r3(PC_1);       //mid right
AnalogIn ir_r4(PC_0);       //far right

//IR emitters and corresponding pins
DigitalOut ir_e1(PB_13);    //far left 
DigitalOut ir_e2(PB_12);    //mid left
DigitalOut ir_e3(PB_1);    ///mid right
DigitalOut ir_e4(PB_0);    ///far right

Timer timer;
//Ticker Systicker;

//constants for the code
volatile unsigned long pulsesRight = 0;
volatile unsigned long pulsesLeft = 0;
int errorPulse = 1;
double Kp = 0.00015;   // arbitrary values
double Kd = 0.02;
double Ki = 0.5;
double speedChange = 0;     //holds P,I,D fn return values
int integral = 0;
double maxCorrect = 0.1;    //limits correction values
double maxSpeed = 0.2;     //limits motor speed
double minSpeed = 0.05;     //need this?
double speedR = 0.1;        //for setting motor speed; accumulates
double speedL = 0.1;
int cnt=0;                  //for D
int prevError = 0;  //for D
int turning = 0;

float farLeft;
float midLeft;
float midRight;
float farRight;

void incrementRight(){  // increments right pulses
	pulsesRight++;
	return;
}
void incrementLeft(){  //increments left pulses
	pulsesLeft++;
	return;
}

void stop() {    // stops mouse (turns motors to 0)
	rightF = 0;
	rightR = 0;
	leftF = 0;
	leftR = 0;
	return;
}

void go(int cells) {
	setRight(speedR)
}

void turnR90() {   // turns 90 deg right
	turning = 1;
	int temp = pulsesRight;
	int temp2 = pulsesLeft;
	pulsesRight = 0;
	pulsesLeft = 0;
	
	midRight = ir_r3.read();
	while(pulsesLeft<1000 && midRight > 0.35){
		pc.printf("pulsesLeft:%d              midRight:%f\r\n", pulsesLeft, midRight);
		rightF.write(0.1);
		rightR.write(0);
		leftF.write(0);
		leftR.write(0.1);
		midRight = ir_r3.read();
	}
	pulsesRight = temp;
	pulsesLeft = temp2;
	stop();
	turning = 0;
	return;
}

void turnL90() {   // turns 90 deg right
	turning = 1;
	int temp = pulsesRight;
	int temp2 = pulsesLeft;
	pulsesRight = 0;
	pulsesLeft = 0;
	midLeft = ir_r2.read();
	while(pulsesRight<1000 && midLeft > 0.35){
		pc.printf("pulsesRight:%d\r\n", pulsesRight);
		rightF.write(0);
		rightR.write(0.1);
		leftF.write(0.1);
		leftR.write(0);
		midLeft = ir_r2.read();
	}
	pulsesRight = temp;
	pulsesLeft = temp2;
	stop();
	turning = 0;
	return;
}

/*
//Not in use yet
void forward(int cells) {   // moves forward num of cells
	int temp = pulsesRight;
	pulsesRight = 0;
	int n = 2500*cells;
	while(pulsesRight<n){
		rightF = 0.1;
		rightR = 0;
		leftF = 0.1;
		leftR = 0;
	}
	pulsesRight = temp + pulsesRight;
	stop();
	return;
}*/

void setLeft(double speed){    // safely writes motor speed
	if(speed > maxSpeed)
		leftF.write(maxSpeed);
	else if(speed < minSpeed)
		leftF.write(minSpeed);
	else
		leftF.write(speed);
	return;
}

void setRight(double speed){
	if(speed > maxSpeed)
		rightF.write(maxSpeed);
	else if(speed < minSpeed)
		rightF.write(minSpeed);
	else
		rightF.write(speed);
	return;
}

void resetSpeed(double speed){    // resets forward speeds to input
	rightF.write(speed);
	rightR.write(0);
	leftF.write(speed);
	leftR.write(0);
	return;
}

double P_Controller(int error) // multiplies by Kp
{
	double correction = Kp * error;  // check if this is the right constant (1 pulse = 1 rotation of inner shaft, = ____ wheel rotations)
		if (abs(correction) > maxCorrect) // check with max
			correction = maxCorrect;
	return correction;
}

double D_Controller(int error) //calc D correction
{
	if(cnt==0) {    //once per 10 cycles
		int dError = error - prevError; //fairly small #
		double correction = Kd * dError;
		prevError = error;
		if (abs(correction) > maxCorrect) // check with max
			correction = maxCorrect;     
		return correction;
	}
	cnt++;
	cnt = cnt%10;
	return 0;
}

double I_Controller(int error)
{
	integral += error;
	double correction = Ki * integral;
	integral /= 2; //decay factor is 2
	return correction;
}

//needs changes. maybe into systick, but not checking walls and turning should be main loop
void aheadTest()
{
	//pc.printf("test farLeft: %f         farRight:%f\r\n", farLeft, farRight);
	midRight = ir_r3.read();
	midLeft = ir_r2.read();
	if(midRight >= 0.325 || midLeft >= 0.325){
		farRight = ir_r4.read();
		farLeft = ir_r1.read();
		if (farRight > farLeft+0.1) {
			turnL90();
		}
		else if(0.1+farLeft > farRight){
			turnR90();
		}
	} 
	return;
}

void systick() {
	
	//errorPulse = pulsesLeft - pulsesRight;

	//--------IR PID----------- just avoid walls
	
	//systick will run while turning so turn off ir pid while turning.
	if(!turning) {
		if(midRight > 0.1) //approaching right wall
			speedR += 0.3 * midRight;   //avg midRight a couple cm away is 0.2
		else if(farLeft > 0.1)
			speedL += 0.3 * midLeft;
	}

	//encoder PID , may not need
	/*
	else{
		//PROPORTIONAL
		speedChange = P_Controller(errorPulse); //can be neg or pos
		
		//DERIVATIVE
		speedChange += D_Controller(errorPulse); //when P neg, D is pos; this line not tested

		//INTEGRAL
		//speedChange += I_Controller(errorPulse);
	  
		speedR += speedChange; //if speedChange neg (left is faster), speedR increases
		speedL -= speedChange;

		if (errorPulse == 0) { //if no error go normal speed
			speedR = 0.2;
			speedL = 0.2;
		}
	}*/
}

int main() {
	pc.baud(9600);
	onled = 1;
	encoderRightF.rise(&incrementRight);
	encoderRightF.fall(&incrementRight);
	encoderLeftF.rise(&incrementLeft);
	encoderLeftF.fall(&incrementLeft);
	//not touching R pins of motors yet

	float offsetL;
	float offsetML;
	float offsetMR;
	float offsetR;

	for(int i=0; i<9; i++) {
		ir_e1 = 1;
		offsetL  += ir_r1.read();
		ir_e1 = 0;
		ir_e2 = 1;
		offsetML += ir_r2.read();
		ir_e2 = 0;
		ir_e3 = 1;
		offsetMR += ir_r3.read();
		ir_e3 = 0;
		ir_e4 = 1;
		offsetR  += ir_r4.read();
		ir_e4 = 0;
	}
	offsetL  /= 10;
	offsetML /= 10;
	offsetMR /= 10;
	offsetR  /= 10;
	
	leftF.write(0);
	leftR.write(0);

	rightF.write(0);
	rightR.write(0);
	
	while (1) {
		ir_e1 = 1;
		farLeft = ir_r1.read() - offsetL;
		ir_e1 = 0;
		
		ir_e2 = 1;
		midLeft = ir_r2.read() - offsetML;
		ir_e2 = 0;
		
		ir_e3 = 1;
		midRight = ir_r3.read() - offsetMR;
		ir_e3 = 0;

		ir_e4 = 1;
		farRight = ir_r4.read() - offsetR;
		ir_e4 = 0;
					
	if(!turning) {
		if(midRight > 0.1) { //approaching right wall
			speedR += 0.3 * midRight;
			speedL -= 0.1 * midRight;
			//setRight(speedR);
			//pc.printf("speedR : %f\n", speedR);
			speedR = 0; speedL = 0;
		} else if(farLeft > 0.1) {
			speedL += 0.3 * midLeft;
			speedR -= 0.1 * midLeft;
			//setLeft(speedL);
			//pc.printf("speedL: %f\n", speedL);
			speedL = 0; speedR = 0;
		}
	}

	//pc.printf("%f %f %f %f %f %f\n", farLeft, midLeft, midRight, farRight, speedL, speedR );
			
	//pc.printf("\n");
	pc.printf("pulsesLeft:%d pulsesRight:%d \n", pulsesLeft, pulsesRight);
  }
}
