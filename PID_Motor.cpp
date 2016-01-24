//
//  PID_Motor.cpp
//  PID_controller
//
//  Created by Will Baker on 4/25/15.
//  Copyright (c) 2015 Will Baker. All rights reserved.
//

#include "PID_Motor.h"

PID_Motor::PID_Motor(int _P, int _I, int _D){
    //
    KP=_P;
    KI=_I;
    KD=_D;
}

PID_Motor::PID_Motor(){KP=1;KI=0;KD=0;};
void PID_Motor::setKP(int P){KP=P;};
void PID_Motor::setKI(int I){KI=I;};
void PID_Motor::setKD(int D){KD=D;};

int PID_Motor::setValue(int val){
    desiredVal=val;
    return 0;
}
int PID_Motor::calculateError(){
    error=desiredVal-currentVal;
    return error;
}
int PID_Motor::calculatePosition(){
    //calculate the position here
    currentVal = simulate();
    return currentVal;
}
int PID_Motor::getPosition(){
    return currentVal;
}
int PID_Motor::getError(){
    return error;
}
int PID_Motor::PID_control(){
    long deltaTime = getDeltaT();
    //integration
    if (abs(error) < integral_threshold){ // prevent integral 'windup'
        integral += error * deltaTime; // accumulate the error integral
    }
    else {
        integral=0; // zero it if out of bounds
    }
    //derivation
    float derivative = (lastVal - currentVal) / deltaTime ;
    
    lastVal = currentVal;
    //actuation = kP*Error + kI*Σ Error + kD * dP/dT
    return KP * error + KI * integral + KD * derivative;
}
int PID_Motor::simulate(){
    float random_factor = lastAct*float(rand() % 5 + 1)/25.0;
    int val = lastVal+int(random_factor+0.5);
    printf("random_factor=%d + %d = %d\n",int(random_factor+0.5),lastVal,val);
    if(val > 1023)
        return 1023;
    if(val < 0)
        return 0;
    return val;
}
long PID_Motor::getDeltaT(){
    return 1;//time_now-time_last;
}
void PID_Motor::update(){
    calculatePosition();
    calculateError();
    printf("SetPoint[%d]\n",desiredVal);
    printf("Position[%d]\n",currentVal);
    printf("Error[%d]\n",error);
    lastAct = PID_control();
    printf("Actuating[%d]\n",lastAct);
}