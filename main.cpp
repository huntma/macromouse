#include "mbed.h"

DigitalOut led1(LED1);

PwmOut leftF(PC_7); //for rat pa7
PwmOut leftR(PC_6); //for rat pb6

PwmOut rightR(PC_9); //for rat: pc7     
PwmOut rightF(PC_8); //for rat: pb10

InterruptIn encoderLeftR(PA_1);
InterruptIn encoderLeftF(PA_0);

InterruptIn encoderRightR(PA_2);
InterruptIn encoderRightF(PA_3);

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
Ticker Systicker;

volatile unsigned long pulsesRight = 0;
volatile unsigned long pulsesLeft = 0;
int errorPulse = 1;
double Kp = 0.00015;   // arbitrary values
double Kd = 0.02;
double Ki = 0.5;
double speedChange = 0;     //holds P,I,D fn return values
int integral = 0;
double maxCorrect = 0.1;    //limits correction values
double maxSpeed = 0.5;     //limits motor speed
double minSpeed = 0.1;     //need this?
double speedR = 0.1;        //for setting motor speed; accumulates
double speedL = 0.1;
int cnt=0;                  //for D
int prevError = 0;  //for D
int turning = 0;

float midRight = ir_r3.read();
float midLeft = ir_r2.read();
float farRight = ir_r4.read();
float farLeft = ir_r1.read();

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
    wait(0.05);
    farRight = ir_r4.read();
    farLeft = ir_r1.read();
    return;
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

void speedLeft(double speed){    // writes the speed of left motor (checks if valid)
    if ((speed < maxSpeed) && (speed > minSpeed)){
        leftF.write(speed);
        return;
    }else if(speed > maxSpeed){
        leftF.write(maxSpeed);
        return;
    }else if(speed < minSpeed){
        leftF.write(minSpeed);
        return;
    }else{
        leftF.write(minSpeed);
    }
    return;
}

void speedRight(double speed){    // writes speed of right motor (checks if valid)
    if ((speed < maxSpeed) && (speed > minSpeed)){
        rightF.write(speed);
        return;
    }else if(speed > maxSpeed){
        rightF.write(maxSpeed);
        return;
    }else if(speed < minSpeed){
        rightF.write(minSpeed);
        return;
    }else{
        rightF.write(minSpeed);
    }
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

void aheadTest()
{
    pc.printf("test farLeft: %f         farRight:%f\r\n", farLeft, farRight);
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
    /*
    //pc.printf("%f\r %f\r %f\r %f\r\n", farLeft, midLeft, midRight, farLeft);

    errorPulse = pulsesLeft - pulsesRight; //if errorPulse negative: left is faster than right

    //--------IR PID-----------
    
    midRight = ir_r3.read();
    midLeft = ir_r2.read();
    farRight = ir_r4.read();
    farLeft = ir_r1.read();
    
    if(!turning) {
    
    if(farLeft > 0.6 || farRight > 0.6) {  //only use IR when not approach corner
        if(farRight > 0.6){ //approaching right wall
            speedChange -= 0.1 * midRight;  //need to add proportional
//          pulsesRight=0;
//          pulsesLeft=0;
        }else if(farLeft > 0.6){
            speedChange += 0.1 * midLeft;
//          pulsesRight=0;
//          pulsesLeft=0;
        }   
        speedR -= speedChange;
        speedL += speedChange;
    }

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
    }
           
    speedLeft(speedL);
    speedRight(speedR);
    
    //pc.printf("%f\r %f\r\n", speedL, speedR);
    }
    
    speedL = 0;
    speedR = 0;

*/
}

int main() {
    
    encoderRightF.rise(&incrementRight);
    encoderRightF.fall(&incrementRight);

    encoderLeftF.rise(&incrementLeft);
    encoderLeftF.fall(&incrementLeft);

    pc.baud(9600);
        
    leftF.write(0.1);
    leftR.write(0);

    rightF.write(0.05);
    rightR.write(0);
    
    while (1) {
        pc.printf("pulsesLeft:%d pulsesRight:%d \n", pulsesLeft, pulsesRight);
    }
}