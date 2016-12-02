
#include "mbed.h"

DigitalOut led1(LED1);

PwmOut rightF(PA_7);
PwmOut rightR(PB_6);

PwmOut leftR(PC_7);
PwmOut leftF(PB_10);


InterruptIn encoderRightR(PB_3);
InterruptIn encoderRightF(PA_15);
InterruptIn encoderLeftR(PA_1);
InterruptIn encoderLeftF(PC_4);

//IR receivers and corresponding pins (directions relative to forward-facing rat)
AnalogIn ir_r1(PC_0);       //far-left
AnalogIn ir_r2(PC_1);       //Middle-left
AnalogIn ir_r3(PA_4);       //Middle-right
AnalogIn ir_r4(PA_0);       //far-right
//IR emitters and corresponding pins
DigitalOut ir_e1(PB_7);     //far-left
DigitalOut ir_e2(PB_0);     //Middle-left
DigitalOut ir_e3(PC_11);    //Middle-right
DigitalOut ir_e4(PC_10);    //far-right

Timer timer;

volatile unsigned long pulsesRight = 0;
volatile unsigned long pulsesLeft = 0;
int errorPulse = 1;
double Kp = 0.00015;   // arbitrary values
double Kd = 0.02;
double Ki = 0.5;
double speedChange = 0;     //holds P,I,D fn return values
int integral = 0;
double maxCorrect = 0.1;    //limits correction values
double maxSpeed = 0.3;     //limits motor speed
double minSpeed = 0.05;     //need this?
double speedR = 0.1;        //for setting motor speed; accumulates
double speedL = 0.1;
int cnt=0;                  //for D
int prevError = 0;  //for D


    float baseMidRight = 0;
    float baseMidLeft = 0;
    float baseFarRight = 0;
    float baseFarLeft = 0;



Serial pc(PA_9, PA_10); //set serial

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
    wait(0.1);
    return;
}

void turnR90() {   // turns 90 deg right
    int temp = pulsesRight;
    int temp2 = pulsesLeft;
    pulsesRight = 0;
    pulsesLeft = 0;
    //int n = 2500*cells;
    while(pulsesRight<300){
        rightF = 0.1;
        rightR = 0;
        leftF = 0;
        leftR = 0.1;
    }
    pulsesRight = temp;
    pulsesLeft = temp2;
    stop();
    return;
}

void turnL90() {   // turns 90 deg right
    int temp = pulsesRight;
    int temp2 = pulsesLeft;
    pulsesRight = 0;
    pulsesLeft = 0;
    //int n = 2500*cells;
    while(pulsesLeft<300){
        rightF = 0;
        rightR = 0.1;
        leftF = 0.1;
        leftR = 0;
    }
    pulsesRight = temp;
    pulsesLeft = temp2;
    stop();
    return;
}

/*
//Not in use
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

int speedLeft(double speed){    // writes the speed of left motor (checks if valid)
    if (speed < maxSpeed){
        leftF.write(speed);
        return 1;
    }
    return 0;
}

int speedRight(double speed){    // writes speed of right motor (checks if valid)
    if (speed < maxSpeed){
        rightF.write(speed);
        return 1;
    }
    return 0;
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
        double dt = timer.read_us();    //for once/10 cycles, dt = ~20
        //pc.printf("time: %f\n", dt);
        double correction = Kd * dError;
        timer.reset();
        prevError = error;
        //double correction = Kd*dError/dt;
        //pc.printf("derror: %f\n", correction);  //gives ~0.X
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


void aheadTest(float midRight, float midLeft, float farRight, float farLeft)
{
    if(midRight >= 0.825 || midLeft >= 0.825){
        pc.printf("hi\r\n");
        stop();

        if (farRight > farLeft) {
            pc.printf("turnLeft\r\n");
            turnL90();
        }
        else {
            pc.printf("turnRight\r\n");
            turnR90();
        }
    } 

    return;
}

int main() {
    
	wait(1);
	
    encoderRightF.rise(&incrementRight);
    encoderRightF.fall(&incrementRight);
    encoderLeftF.rise(&incrementLeft);
    encoderLeftF.fall(&incrementLeft);
    pc.baud(9600);

    int i = 0;

    while(i < 100){	//if trying to avg 100, need +=
        ir_e4 = 1;
        ir_e3 = 1;
        ir_e2 = 1;
        ir_e1 = 1;
        baseMidRight += ir_r3.read();
        baseMidLeft += ir_r2.read();
        baseFarRight += ir_r4.read();
        baseFarLeft += ir_r1.read();
        i++;
    }

    baseMidLeft /= 100;
    baseMidRight /= 100;
    baseFarRight /= 100;
    baseFarLeft /= 100;
	

    while(1) {

        timer.start();
        led1 = 1;
        
        ir_e4 = 1;
        ir_e3 = 1;
        ir_e2 = 1;
        ir_e1 = 1;

        float midRight = ir_r3.read();
        float midLeft = ir_r2.read();
        float farRight = ir_r4.read();
        float farLeft = ir_r1.read();

        aheadTest(midRight, midLeft, farRight, farLeft);
        errorPulse = pulsesLeft - pulsesRight; //if errorPulse negative: left is faster than right

        //when front sensors go over 0.2, ignore sides

        //--------IR PID-----------

        float changeMidRight = midRight - baseMidRight;
        float changeMidLeft = midLeft - baseMidLeft;
        float changeFarRight = farRight - baseFarRight;
        float changeFarLeft = farLeft - baseFarLeft;

        int enc_pid = 1;	//toggles encoder or IR PID
		
		if((changeMidRight + changeMidLeft)*0.5 > 0.1) {	//only use IR when not approach corner
			if(changeFarRight > 0.1){ //approaching right wall
				speedR += 0.1;
				pulsesRight=0;
				pulsesLeft=0;
				enc_pid=0;
			}else if(changeFarLeft > 0.1){
				speedL += 0.1;
				pulsesRight=0;
				pulsesLeft=0;
				enc_pid=0;
			}

		}

        if(enc_pid) {
			//PROPORTIONAL
			speedChange = P_Controller(errorPulse); //can be neg or pos
			
			//DERIVATIVE
			speedChange += D_Controller(errorPulse); //when P neg, D is pos; this line not tested

			//INTEGRAL
			//speedChange += I_Controller(errorPulse);
		  
			speedR -= speedChange; //if speedChange neg (left is faster), speedR increases
			speedL += speedChange;

			 if (errorPulse == 0) { //if no error go normal speed
				speedR = 0.2;
				speedL = 0.2;
			}
		}
               
        speedLeft(speedL);
        speedRight(speedR);
        
        pc.printf("%d\r\n", errorPulse);

        timer.stop();
        
    }
}
