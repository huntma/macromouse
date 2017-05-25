#include "mbed.h"

DigitalOut onled(PC_11); //indicator light

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
AnalogIn rL(PC_3);       //far left
AnalogIn rML(PC_2);       //mid left
AnalogIn rMR(PC_1);       //mid right
AnalogIn rR(PC_0);       //far right

//IR emitters and corresponding pins
DigitalOut eL(PB_13);    //far left 
DigitalOut eML(PB_12);    //mid left
DigitalOut eMR(PB_1);    ///mid right
DigitalOut eR(PB_0);    ///far right

float offsetL;
float offsetML;
float offsetMR;
float offsetR;

class cell
{
	public:
		cell(int d)
		{
			dist = d;
			n=0;s=0;e=0;w=0;
		}
		int dist;
		bool n,s,e,w;	//north south east west
};
/*
int mapdim=16;
for(i=0; i<mapdim; i++)
	for(j=0; j<mapdim; j++)
		map[i][j] = new cell(mapdim + 
*/
cell map[16][16] = {
{15,14,13,12,11,10,9,8,8,9,10,11,12,13,14,15},
{14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14},
{13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13},
{12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12},
{11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11},
{10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10},
{9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9},
{8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8},
{8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8},
{9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9},
{10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10},
{11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11},
{12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12},
{13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13},
{14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14},
{15,14,13,12,11,10,9,8,8,9,10,11,12,13,14,15}
};

int x=0;
int y=0;
int curbr=2; //1 for north, 2 for east, 3 south, 4 west

Timer timer;
Ticker Systicker;

//constants for the code
double Kp = 0.00015;   // arbitrary values
double Kd = 0.02;
double Ki = 0.5;

double maxCorrect = 0.1;   //limits correction values
double maxSpeed = 0.1;     //limits motor speed
double minSpeed = 0.05;

volatile unsigned long pulsesRight = 0; //why volatile?
volatile unsigned long pulsesLeft = 0;
int errorPulse = 1;
int integral = 0;
double speedR = 0.1;        //holds motor speed
double speedL = 0.1;
int cnt=0;                  //for D
int prevError = 0;  //for D

float farLeft; //hold IR readings
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

void turn(int gobr) {
	int rtt = gobr - curbr;	//will be either 0, 1, 2, 3, -1, -2, -3
	//neg -> counterclock; pos -> clock
	if(rtt == 0)
		return;
	//change -3 to +1 and +3 to -1
	if(rtt == -3)
		rtt = 1;
	else if(rtt == 3)
		rtt = -1;
		
	pulsesRight = 0;
	pulsesLeft = 0;
	if(rtt > 0) { //right turns
		int cnt = rtt * 100;
		while(pulsesLeft < cnt){
			rightF.write(0);
			rightR.write(0.05);
			leftR.write(0);
			leftF.write(0.05);
		}
	}
	else { 		//left turns
		int cnt = rtt * -100;
		while(pulsesRight < cnt){
			rightR.write(0);
			rightF.write(0.05);
			leftF.write(0);
			leftR.write(0.05);
		}
	}
	curbr = gobr; //set current bearing to new bearing
	stop();
	return;
}

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

void go(int cells) {
	int cnt = cells * 146; //TODO change 146
	pulsesRight = 0;
	pulsesLeft = 0;
	while(midLeft < 0.1 && midRight < 0.1 && pulsesRight < cnt && pulsesLeft < cnt) {
		setRight(speedR);
		setLeft(speedL);
	}
	switch(curbr) {
		case 1:
			y++;		
		case 2:
			x++;
		case 3:
			y--;
		case 4:
			x--;
	}
	rightF = 0;
	leftF = 0;
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


void systick() {

	//--------IR PID----------- 
	eL = 1;
	farLeft = rL.read() - offsetL;
	eL = 0;
	
	eML = 1;
	midLeft = rML.read() - offsetML;
	eML = 0;
	
	eMR = 1;
	midRight = rMR.read() - offsetMR;
	eMR = 0;

	eR = 1;
	farRight = rR.read() - offsetR;
	eR = 0;
					
	if(midRight > 0.1) { //approaching right wall
		speedR += 0.2 * midRight;
		speedL -= 0.2 * midRight;
		//setRight(speedR);
		//setLeft(speedL);
		//pc.printf("speedR : %f\n", speedR);
	} else if(farLeft > 0.1) { //may need to make more sensitive
		speedL += 0.2 * midLeft;
		speedR -= 0.2 * midLeft;
		//setLeft(speedL);
		//setRight(speedR);
		//pc.printf("speedL: %f\n", speedL);
	}

}

int main() {
	pc.baud(9600);
	Systicker.attach_us(&systick, 5000);
	onled = 1;
	encoderRightF.rise(&incrementRight);
	encoderRightF.fall(&incrementRight);
	encoderLeftF.rise(&incrementLeft);
	encoderLeftF.fall(&incrementLeft);
	//not use R pins of motors yet
	

	for(int i=0; i<9; i++) {
		eL = 1;
		offsetL  += rL.read();
		eL = 0;
		eML = 1;
		offsetML += rML.read();
		eML = 0;
		eMR = 1;
		offsetMR += rMR.read();
		eMR = 0;
		eR = 1;
		offsetR  += rR.read();
		eR = 0;
	}
	offsetL  /= 10;
	offsetML /= 10;
	offsetMR /= 10;
	offsetR  /= 10;

	turn(3);
	turn(4);
	turn(2);

	while (1) {
	/*
	//find adjacent lower dist cell
	int gobr = 1;
	int min = map[x][y].dist;
	if(y > 0 && !map[x][y-1].s && map[x][y-1].dist < min) //check north
		gobr = 1;
	if(x < 15 && !map[x+1][y].w && map[x+1][y].dist < min) //check east
		gobr = 2;
	if(y < 15 && !map[x][y+1].n && map[x][y+1].dist < min) //check south
		gobr = 3;
	if(x > 0 && !map[x-1][y].e && map[x-1][y].dist < min) //check west
		gobr = 4;
	*/	
	//turn(gobr);
	//go(1);//TODO check
	
	
	//if blocked
		//place a wall
		//flood	

	//	pc.printf("%f %f %f %f %f %f\n", farLeft, midLeft, midRight, farRight, speedL, speedR );
			
	//pc.printf("\n");
	//pc.printf("pulsesLeft:%d pulsesRight:%d \n", pulsesLeft, pulsesRight);
  }
}
