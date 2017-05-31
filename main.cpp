#include "mbed.h"
DigitalOut onled(PC_11); //indicator light
Serial pc(PA_9, PA_10); //set serial

//motor pins
PwmOut leftF(PC_7); //for rat pa7
PwmOut leftR(PC_6); //for rat pb6
PwmOut rightR(PC_9); //for rat: pc7     
PwmOut rightF(PC_8); //for rat: pb10

//set motor encoders
InterruptIn encLeftA(PA_0);
InterruptIn encLeftB(PA_1);
InterruptIn encRightA(PA_2);
InterruptIn encRightB(PA_3);

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
int STHRESH = 0;
int MAPSIZE = 6;

cell map[6][6] = {
{5,4,3,3,4,5},
{4,3,2,2,3,4},
{3,2,1,1,2,3},
{3,2,1,1,2,3},
{4,3,2,2,3,4},
{5,4,3,3,4,5},
};
/*
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
};*/

int x=0;
int y=0;
int curbr=1; //0 for north, 1 for east, 2 south, 3 west

Timer timer;

double maxSpeed = 0.2;     //limits motor speed
double minSpeed = 0.07;		//motors stall at 0.05

volatile unsigned long encRight = 0; //why volatile?
volatile unsigned long encLeft = 0;
double speedR, speedL;	//holds motor speeds
int cnt=0;                  //for D
int prevError = 0;  //for D

float farLeft, midLeft, midRight, farRight;

//for each pair AB, B lvl determines if rise and falls of A are fwd or rv
void riseRight(){  // right pulses
	if(encRightB == 0)
		encRight++;
	else
		encRight--;
	return;
}
void fallRight() {
	if(encRightB == 1)
		encRight++;
	else
		encRight--;
	return;
}
void riseLeft(){  //TODO encLeftA not reading!
	if(encLeftA == 1)
		encLeft++;
	else
		encLeft--;
	return;
}
void fallLeft() {
	if(encLeftA == 0)
		encLeft++;
	else
		encLeft--;
	return;
}

void stop() {    // stops mouse (turns motors to 0)
	rightF = 0;
	rightR = 0;
	leftF = 0;
	leftR = 0;
	return;
}

void incPos() {
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
	return;
}

void setLeft(double speed, int dir){    // safely writes motor speed
	if(dir == 0) { //rev
		if(speed > maxSpeed)
			leftR.write(maxSpeed);
		else if(speed < minSpeed)
			leftR.write(minSpeed);
		else
			leftR.write(speed);
	}
	else {
		if(speed > maxSpeed)
			leftF.write(maxSpeed);
		else if(speed < minSpeed)
			leftF.write(minSpeed);
		else
			leftF.write(speed);
	}
	return;
}
void setRight(double speed, int dir){    // safely writes motor speed
	if(dir == 0) { //rev
		if(speed > maxSpeed)
			rightR.write(maxSpeed);
		else if(speed < minSpeed)
			rightR.write(minSpeed);
		else
			rightR.write(speed);
	}
	else {
		if(speed > maxSpeed)
			rightF.write(maxSpeed);
		else if(speed < minSpeed)
			rightF.write(minSpeed);
		else
			rightF.write(speed);
	}
	return;
}

void turn(int gobr) {
	int rtt = gobr - curbr;	//will be either 0, 1, 2, 3, -1, -2, -3
	//neg -> counterclock; pos -> clock
	if(rtt == 0) //don't need turn
		return;
	//change -3 to +1 and +3 to -1
	if(rtt == -3)
		rtt = 1;
	else if(rtt == 3)
		rtt = -1;
	encRight = 0;
	encLeft = 0;
	int cnt = rtt*107;
	int error = cnt - encRight; //if rtt neg, will turn left and encRight neg
	while(1){
		error = cnt + encRight;
		if(error > 8) { //turn more; TODO adjust
			rightF.write(0);
			setRight(error*0.001,0); //use set fn to safeguard max speed
			leftR.write(0);
			setLeft(error*0.001,1);
			pc.printf("%d\n", error);
		}
		else if(error < -8) {	//turn back
			rightR.write(0);
			setRight(error*0.001,1);
			leftF.write(0);
			setLeft(error*0.001,0);
		}
		else { //just right
		  break;
		}
	}
	curbr = gobr; //set bearing to new bearing
	stop();
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

void ir() {
	//need 60us to turn on and off
	eL = 1; wait_us(70);
	farLeft = rL.read() - offsetL;
	eL = 0;
	
	eMR = 1; wait_us(70);
	midRight = rMR.read() - offsetMR;
	eMR = 0;

	eML = 1; wait_us(70);
	midLeft = rML.read() - offsetML;
	eML = 0;

	eR = 1; wait_us(70);
	farRight = rR.read() - offsetR;
	eR = 0;
	speedR = 0.1; speedL = 0.1;
	if(midRight > STHRESH) { //approaching right wall
		speedR += 0.4 * midRight;
		speedL -= 0.2 * midRight;
//		int d = prevmidRight - midRight; //pos if improving, want to leave decrease
//		speedR -= d*Kd;
//		speedL += 
	} if(midLeft > STHRESH) { //may need to make more sensitive
		speedL += 0.4 * midLeft;
		speedR -= 0.2 * midLeft;
	}
	return;
}

void go(int cells) {
	int cnt = cells * 200; //TODO change
	bool wall = 0;
	encRight = 0;
	encLeft = 0;
	while(encRight < cnt && encLeft < cnt) {
		if(midLeft > THRESH || midRight > THRESH) {
			rightF = 0;
			leftF = 0;
			wall = 1;
			break;
		}
		setRight(speedR,1);
		setLeft(speedL,1);
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
	encRightA.rise(&riseRight);
	encRightA.fall(&fallRight);
//	encLeftA.rise(&riseLeft);
//	encLeftA.fall(&fallLeft);
	encLeftB.rise(&riseLeft);
	encLeftB.fall(&fallLeft);

	for(int i=0; i<1; i++) {
		eL = 1; wait_us(100);
		offsetL = rL.read();
		eL = 0;
		
		eMR = 1; wait_us(100);
		offsetMR = rMR.read();
		eMR = 0;

		eML = 1; wait_us(100);
		offsetML = rML.read();
		eML = 0;

		eR = 1; wait_us(100);
		offsetR = rR.read();
		eR = 0;
	}
	turn(2);wait(1);turn(3);
	while (1) {
	//PSEUDO
	//read ir
	//check front
		//stop and turn
	//if new cell
		//check need to turn
		//place wall
		//flood
		//blink led
	//run motor with P
/*	
		ir();	
		if(farLeft > 0.3 || farRight > 0.3) {
			stop();
			encLeft=0;encRight=0;
			stop();
			//placeWall('f');
			//flood
			//turn to next
			turn((curbr + rand()) %4);
		}
		//if new cell
			//check need to turn
			//place wall
		setLeft(speedL,1);
		setRight(speedR,1);
*/		
	//	pc.printf("%f %f %f %f %f %f (%d,%d)\n", farLeft, midLeft, midRight, farRight, speedL, speedR, x, y );
//		if(encRight > 200 || encLeft > 200) {
//			stop();
//			incPos();
//			//walls
//			//flood
//			encRight = 0;
//			encLeft = 0;
//		}
		/*
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
	//pc.printf("\n");
	pc.printf("encLeft:%d encRight:%d \n", encLeft, encRight);
//pc.printf("%f %f %f %f %f %f\n", offsetL, offsetML, offsetMR, offsetR, speedL, speedR );
		
	}  
}
