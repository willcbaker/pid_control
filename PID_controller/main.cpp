/*
 A PID controller for Linear Actuators
 Author: William Baker    2015
 */

#include "PID_Motor.h"
#include <stdio.h>
#include <stdlib.h>     /* srand, rand */
#include <unistd.h>

//#include <Servo.h>

#define BAUD_RATE  9600
#define DEBUG        1

//used for testing only
#define TEST_SWEEP   1
#define SWEEP_STEP   40
#define SIMULATE     1
#define TOLERANCE    5

#define MOTOR_LEFT_DIRECTION_PIN   5
#define MOTOR_RIGHT_DIRECTION_PIN  6
#define MOTOR_LEFT_PWM_PIN         8
#define MOTOR_RIGHT_PWM_PIN        9
#define MOTOR_LEFT_FEEDBACK_PIN    A1
#define MOTOR_RIGHT_FEEDBACK_PIN   A2

#define MOTOR_LIMIT   255   //max motor speed
#define MIN_FEEDBACK  0     //min encoder/resistor value
#define MAX_FEEDBACK  1023  //max encoder/resistor value

//PID coefficients
#define KP  2//start x, increase until oscilation
#define KD  0//start 0, increase as necesary
#define KI  0//start 0, increase as necesary
/*
 Use of the integral term can be problematic. In PID control systems there is a
 problem referred to as “windup”, which occurs as follows. The integral term sums the error
 history over time. If a system starts far from the final desired set point, the initial errors will be
 large, and the integral term will quickly grow very large. This accumulated integral usually
 produces a dominating effect which prevents the system from quickly achieving the set point. To
 avoid this problem, a number of methods have been employed. The scheme shown in the code
 above is to “zero out” the integral term unless the error is sufficiently small. Specifically, the test
 here is to check if the current error is less than some test value. In the code above the test value is
 called IntThresh. Including this test allows the integral term to operate only after the system has
 approached close to the final set point. The integral term then acts to remove any small residual
 error so that the system may converge to the final set point.
 source: http://www.maelabs.ucsd.edu/mae156alib/control/PID-Control-Ardunio.pdf
 */
#define INT_THRESHOLD 30

long timeLast;
float integral_Right, integral_Left;
int lastActual_Right, lastActual_Left;
//Servo motorLeft;
//Servo motorRight;

//FOR TEST/SIMULATION ONLY:
int desired_value = 0;//initial condition
int sweep_direction = 1;
int lastActuation_Right,lastActuation_Left;

void setup() {
    //Serial.begin(BAUD_RATE);
    //motorLeft.attach(MOTOR_LEFT_PWM_PIN);
    //motorRight.attach(MOTOR_RIGHT_PWM_PIN);
    
    if(DEBUG){
        printf("Starting PID Controller...\n");//
    }
    
    //timeLast = timeNow();
    
}


//A function to return the error
int getError(int set,int actual){
    return set - actual;
}

int constrain(int val, int min, int max){
    if(val < min)
        return min;
    if(val > max)
        return max;
    return val;
}

//this is a pretty bad simulation
int simulate(int &last, int actuation){
    float random_factor = actuation*float(rand() % 5 + 1)/10.0;
    return constrain(last+random_factor,MIN_FEEDBACK,MAX_FEEDBACK);
}

float PIDcontrol(int setPoint, int actual, int &previousActual, int deltaTime, float &integral){
    //-------------------- Calculates the PID drive value --------------------
    int error = getError(setPoint,actual);
    //integration
    if (abs(error) < INT_THRESHOLD){ // prevent integral 'windup'
        integral += error * deltaTime; // accumulate the error integral
    }
    else {
        integral=0; // zero it if out of bounds
    }
    //derivation
    float derivative = (previousActual - actual) / deltaTime ;
    previousActual = actual;
    
    //actuation = kP*Error + kI*Σ Error + kD * dP/dT
    return KP * error + KI * integral + KD * derivative;
}
//A function to actuate the motors to the desired set point
void actuate(int setPoint) {
    
    int actualRight,actualLeft;
    if(SIMULATE){
        actualRight = simulate(lastActual_Right,lastActuation_Right);
        actualLeft  = simulate(lastActual_Left,lastActuation_Left);
    }else{
        //actualRight = analogReadAverage(MOTOR_RIGHT_FEEDBACK_PIN,5);
        //actualLeft = analogReadAverage(MOTOR_LEFT_FEEDBACK_PIN,5);
    }
    
    int deltaTime = 1;//long deltaTime = timeNow()-timeLast;
    
    int actuate_Left = PIDcontrol(setPoint,actualLeft,lastActual_Left,deltaTime,integral_Left) + 0.5;
    //setMotor(motorLeft,MOTOR_RIGHT_DIRECTION_PIN,actuate_Left);
    
    int actuate_Right = PIDcontrol(setPoint,actualRight,lastActual_Right,deltaTime,integral_Right) + 0.5;
    //setMotor(motorRight,MOTOR_RIGHT_DIRECTION_PIN,actuate_Right);
    
    if(DEBUG){
        printf("Setting actuators to: %d\n",setPoint);
        printf("Positions:[%d],[%d]\n",actualLeft,actualRight);
        printf("Errors:[%d],[%d]\n",getError(setPoint,actualLeft),getError(setPoint,actualRight));
        printf("Actuations:[%d],[%d]\n",actuate_Left,actuate_Right);
        printf("----------------------\n");
    }
    lastActuation_Left=actuate_Left;
    lastActuation_Right=actuate_Right;
    //timeLast=timeNow();
}


void loop(){
    //
    if(TEST_SWEEP){//a test sweep
        if(getError(desired_value,lastActual_Left) < TOLERANCE &&
           getError(desired_value,lastActual_Right) < TOLERANCE){
            desired_value += SWEEP_STEP * sweep_direction;
            if(desired_value > MAX_FEEDBACK || desired_value < MIN_FEEDBACK){
                sweep_direction *= -1;
                if(DEBUG){
                    printf("Reversing SWEEP Direction\n");//
                }
            }
        }
    }
    actuate(desired_value);
}
int main(){
    //setup();
    PID_Motor motor(KP,KI,KD);
    printf("Starting Position: %d\n",motor.getPosition());
    motor.setValue(100);
    while(1){
        //loop();
        motor.update();
        printf("===============\n");
        sleep(0.5f);
    }
}