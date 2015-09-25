
// Copyright Alten AB, Linköping 2015
//
// Authors: Sören Molander, Joel Nordh, Pierre Östlund
//
//
#ifndef __GLOBALS_H__
#define __GLOBALS_H__

// Time out for line follower track lost 
const int LF_TIME_OUT = 3000; // in ms 

// States and events for state machines

// Line follower (LF) states
namespace SM_LF {
    const int ST_LF_START_ROBOT = 0;
    const int ST_LF_TRACK = 1;
    const int ST_LF_STOP_ROBOT = 2;
    const int ST_LF_TURN_ROBOT = 3;
    const int ST_LF_HALT_ROBOT = 4;
    const int ST_ANY = -10;
// Line follower events
    const int EV_LF_TRACK_LOST = 0;
    const int EV_LF_TRACK_FOUND = 1;
    const int EV_LF_TIME_OUT = 2;
    const int EV_ANY = -20;
    static int LF_EVENT;
    static int LF_STATE;
}; 




namespace SM_CONTROL {
// Top level Control states
    const int ST_MANUAL_CONTROL = 5;
    const int ST_LINE_FOLLOW_CONTROL = 6;

// Control events
    const int EV_MANUAL_CONTROL = 3;
    const int EV_LINE_FOLLOW_CONTROL = 4;
    static int CONTROL_EVENT;
    static int CONTROL_STATE;
};


// Math
const double DEG2RAD = 3.141592653589793/180.0;
const double RAD2DEG = 1.0/DEG2RAD;

#endif
