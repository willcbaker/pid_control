//
//  PID_Motor.h
//  PID_controller
//
//  Created by Will Baker on 4/25/15.
//  Copyright (c) 2015 Will Baker. All rights reserved.
//

#ifndef __PID_controller__PID_Motor__
#define __PID_controller__PID_Motor__

#include <stdio.h>
#include <stdlib.h>

class PID_Motor{
private:
    int KP;
    int KI;
    int KD;
    int lastVal=0;
    int desiredVal;
    int currentVal;
    int error;
    int lastAct;
    double integral;
    long time_last;
    long time_now;
    int integral_threshold;
    int PID_control();
    int simulate();
    long getDeltaT();
    int calculateError();
    int calculatePosition();
public:
    PID_Motor(int P, int I, int D);
    PID_Motor();
    void setKP(int P);
    void setKI(int I);
    void setKD(int D);
    int getError();
    int getPosition();
    int setValue(int val);
    void update();
};

#endif /* defined(__PID_controller__PID_Motor__) */
