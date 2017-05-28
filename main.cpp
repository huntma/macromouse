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

float offsetL=0;
float offsetML=0;
float offsetMR=0;
float offsetR=0;

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
int THRESH = 0.45;
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
int curbr=1; //0 for north, 1 for east, 2 south, 3 west

Timer timer;
//Ticker Systicker;

//constants for the code
double Kp = 0.00015;   // arbitrary values
double Kd = 0.02;
double Ki = 0.5;

double maxCorrect = 0.1;   //limits correction values
double maxSpeed = 0.2;     //limits motor speed
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

void placeWall(char c) //l for left, r for right, f for front
{
	int dir = curbr;
	if(c == 'f'){}		//choose front
	else if(c == 'l') //choose left
		dir = (dir+3)%4;
	else							//choose right
		dir = (dir+1)%4;	
	switch(dir) { //case 1,2 no danger
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

void rstSpeed()
{
	speedR = 0.1;
	speedL = 0.1;
}

void ir() {
	//--------IR PID----------- need 60us to turn on and off
	eL = 1;
	wait_us(70);
	farLeft = rL.read() - offsetL;
	eL = 0;
	
	eMR = 1;
	wait_us(70);
	midRight = rMR.read() - offsetMR;
	eMR = 0;

	eML = 1;
	wait_us(70);
	midLeft = rML.read() - offsetML;
	eML = 0;

	eR = 1;
	wait_us(70);
	farRight = rR.read() - offsetR;
	eR = 0;

	if(midRight > STHRESH) { //approaching right wall
		speedR += 0.5 * midRight;
		speedL -= 0.5 * midRight;
		//pc.printf("speedR : %f\n", speedR);
	} if(midLeft > STHRESH) { //may need to make more sensitive
		speedL += 0.5 * midLeft;
		speedR -= 0.5 * midLeft;
		//pc.printf("speedL: %f\n", speedL);
	}
	return;
}

void go(int cells) {
	int cnt = cells * 200; //TODO change
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
		rstSpeed();
	}
	if(wall)	//if we missed a wall and tried to go into it
		placeWall('f');
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
	int dim = MAPSIZE*2;
	for(int j=0; j<dim; j++) {
		for(int i=0; i<dim; i++) {
			if(j%2 == 0) { //jth row even, print horiz walls
				if(i%2 == 0) //ith col even, print dots
					pc.printf(".");
				else {				 //ith col odd, print north wall if exists
					if(j<dim && map[i/2][j/2].n) pc.printf("__"); //check bottom
					else pc.printf(" ");
				}
			}
			else {				//jth row odd, print vert walls
				if(i%2 == 0) { //ith col even, print walls
					if(i/2 > 0 && map[i/2-1][j/2].e) pc.printf("|");//check leftest
					else pc.printf(" ");
				}
				else				//ith col odd, print space
					pc.printf("%d",map[i/2][j/2].dist);
			}
		}
		pc.printf("\n");
		}
	return;
}

int main() {
	pc.baud(9600);
	//Systicker.attach_us(&systick, 50000);
	onled = 1;
	encoderRightF.rise(&incrementRight);
	encoderRightF.fall(&incrementRight);
	encoderLeftF.rise(&incrementLeft);
	encoderLeftF.fall(&incrementLeft);
	//not use R pins of motors yet

	for(int i=0; i<1; i++) {
		eL = 1;
		wait_us(100);
		offsetL = rL.read();
		eL = 0;
		
		eMR = 1;
		wait_us(100);
		offsetMR = rMR.read();
		eMR = 0;

		eML = 1;
		wait_us(100);
		offsetML = rML.read();
		eML = 0;

		eR = 1;
		wait_us(100);
		offsetR = rR.read();
		eR = 0;
	}
	while (1) {
	ir();	
	setLeft(speedL);
	setRight(speedR);
	pc.printf("%f %f %f %f %f %f\n", farLeft, midLeft, midRight, farRight, speedL, speedR );
	rstSpeed();
/*
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
*/
//	pc.printf("%f %f %f %f\n", offsetL, offsetML, offsetMR, offsetR);
			
	//pc.printf("\n");
	//pc.printf("pulsesLeft:%d pulsesRight:%d \n", pulsesLeft, pulsesRight);
  }
}
