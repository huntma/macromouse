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
			dist=d;
			n=0;e=0;
		}
		int dist;
		bool n,e;	//north east; only two avoid redundancy
};
/*
int mapdim=16;
for(i=0; i<mapdim; i++)
	for(j=0; j<mapdim; j++)
		map[i][j] = new cell(mapdim + 
*/
int THRESH = 0.2;
int STHRESH = 0.1;
int MAPSIZE = 16;
cell map[16][16] = {
{15,14,13,12,11,10,9,8,8,9,10,11,12,13,14,15},
{14,13,12,11,10, 9,8,7,7,8, 9,10,11,12,13,14},
{13,12,11,10, 9, 8,7,6,6,7, 8, 9,10,11,12,13},
{12,11,10, 9, 8, 7,6,5,5,6, 7, 8, 9,10,11,12},
{11,10, 9, 8, 7, 6,5,4,4,5, 6, 7, 8, 9,10,11},
{10, 9, 8, 7, 6, 5,4,3,3,4, 5, 6, 7, 8, 9,10},
{ 9, 8, 7, 6, 5, 4,3,2,2,3, 4, 5, 6, 7, 8, 9},
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
		int cnt = rtt * 107;
		while(pulsesLeft < cnt){
			rightF.write(0);
			rightR.write(0.05);
			leftR.write(0);
			leftF.write(0.05);
		}
	}
	else { 		//left turns
		int cnt = rtt * -107;
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
	bool wall = 0;
	pulsesRight = 0;
	pulsesLeft = 0;
	while(pulsesRight < cnt && pulsesLeft < cnt) {
		if(midLeft > THRESH || midRight > THRESH) {
			rightF = 0;
			leftF = 0;
			wall = 1;
			break;
		}
		setRight(speedR);
		setLeft(speedL);
	}
	if(wall)
		placeWall();
	else	
		switch(curbr) {
			case 1:
				y++;		
				break;
			case 2:
				x++;
				break;
			case 3:
				y--;
				break;
			case 4:
				x--;
				break;
		}
	rightF = 0;
	leftF = 0;
	return;
}


void printMap()
{
for(int i=0; i<MAPSIZE-1; i++)
	for(int j=0; j<MAPSIZE-1; j++)
		


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
					
	if(midRight > STHRESH) { //approaching right wall
		speedR += 0.2 * midRight;
		speedL -= 0.2 * midRight;
		//setRight(speedR);
		//setLeft(speedL);
		//pc.printf("speedR : %f\n", speedR);
	} else if(farLeft > STHRESH) { //may need to make more sensitive
		speedL += 0.2 * midLeft;
		speedR -= 0.2 * midLeft;
		//setLeft(speedL);
		//setRight(speedR);
		//pc.printf("speedL: %f\n", speedL);
	}

}

void placeWall()
{
	switch(curbr) { //case 1,2 no danger
		case 1:
			map[x][y].n = 1;
			return;
		case 2:
			map[x][y].e = 1;
			return;
		case 3:
			if(y < MAPSIZE-1) map[x][y+1].n = 1;
			return;
		case 4:
			if(x > 0) map[x-1][y].e = 1;
			return;
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

	
	while (1) {
	//check and place walls if possible
	if(farLeft > THRESH || farRight > THRESH)
		placeWall();		
	
	//flood

	//find adjacent lower dist cell
	
	int gobr = curbr;
	int min = map[x][y].dist;
	if(y > 0 && !map[x][y].n && map[x][y-1].dist < min) //check north
		gobr = 1;
	if(x < MAPSIZE-1 && !map[x][y].e && map[x+1][y].dist < min) //check east
		gobr = 2;
	if(y < MAPSIZE-1 && !map[x][y+1].n && map[x][y+1].dist < min) //check south
		gobr = 3;
	if(x > 0 && !map[x-1][y].e && map[x-1][y].dist < min) //check west
		gobr = 4;
	turn(gobr);
	go(1);
	wait(0.5);
	

	//	pc.printf("%f %f %f %f %f %f\n", farLeft, midLeft, midRight, farRight, speedL, speedR );
			
	//pc.printf("\n");
	//pc.printf("pulsesLeft:%d pulsesRight:%d \n", pulsesLeft, pulsesRight);
  }
}
