// Copyright Alten AB, Linköping 2015
//
// Authors: Sören Molander, Joel Nordh, Pierre Östlund
//


#ifndef __STATEMACHINE__
#define __STATEMACHINE__ 

#include "MotorControl.h"
#include "ImageProc.h"
#include <stdio.h>
#include <iostream>



// The Line Follower SM. Note: The order and semantics of actions is defined in the main program! 
namespace SM_LF { // Line follower state machine

    // The basic SM element: State, Event, Transition unction
    // The transition function accepts a list of points to objects
    // that dispatches events, which in turn triggers state changes
    typedef int (*transFunc)(std::vector<void*>&);
    struct tTransition {
        int st; // state
        int ev; // event
        transFunc fn; // action
        tTransition(int state, int event, transFunc func) :
            st(state),
            ev(event),
            fn(func) {}
    };

    
    int NOF_TRANSITIONS;
    int INIT_STATE;
    std::vector<tTransition> trans;
    int TRANS_COUNT;
    
// Start state
    int StartRobot(std::vector<void*>& actions) {
        GetPosition* getpos = reinterpret_cast<GetPosition*>(actions[1]);
        getpos->getPositionsOnLine(0,0);
        std::cout << "Start state" << std::endl;
        return ST_LF_START_ROBOT;
    }
    
// Track and drive    
    int Track(std::vector<void*>& actions) {
        MotorControl* motor_control = reinterpret_cast<MotorControl*>(actions[0]);
        GetPosition* getpos = reinterpret_cast<GetPosition*>(actions[1]);
        getpos->getPositionsOnLine(0,0);
        double err = getpos->getErr();
        motor_control->drive(err,getpos->signalStatus());
        std::cout << "Track state" << std::endl;
        return ST_LF_TRACK;
    }

// Stop and look for track
    int StopRobot(std::vector<void*>& actions) {
        MotorControl* motor_control = reinterpret_cast<MotorControl*>(actions[0]);
        GetPosition* getpos = reinterpret_cast<GetPosition*>(actions[1]);
        motor_control->halt();
        std::cout << "Stop robot and look for track" << std::endl;
#ifdef __linux__
        sleep(500);
        
#else
        Sleep(500);
#endif
        getpos->getPositionsOnLine(0,0);
        std::cout << "Stop Robot" << std::endl;
        return ST_LF_TURN_ROBOT;
    }

// Turn robot    
    int TurnRobot(std::vector<void*>& actions) {
        MotorControl* motor_control = reinterpret_cast<MotorControl*>(actions[0]);
        GetPosition* getpos = reinterpret_cast<GetPosition*>(actions[1]);
        int direction = (getpos->getErr() < 0 ? motor_control->LEFT : motor_control->RIGHT);
        std::cout << "Turn robot" << std::endl;
#ifdef __linux_
        int* mpu_data;
        
        _arduino->get_mpu_data(mpu_data);
        turnAngle = 30 * DEG2RAD;
        getNav->calcFusedAngles(mpu_data);
        motor_control->turn(getpos->getErr(),direction, turnAngle);
#else        
        motor_control->turn(getpos->getErr(),direction, motor_control->getParams()->TURN_DURATION);
#endif
#ifdef __linux__
        sleep(200);
#else
        Sleep(200);
#endif
        getpos->getPositionsOnLine(0,0);
        return ST_LF_TURN_ROBOT;
    }
    
// Halt tracking
    int HaltRobot(std::vector<void*>& actions) {
        MotorControl* motor_control = reinterpret_cast<MotorControl*>(actions[0]);
        GetPosition* getpos = reinterpret_cast<GetPosition*>(actions[1]);
        motor_control->halt();
        // getpos->getPositionsOnLine(0,0);
        std::cout << "Halt robot" << std::endl;
        return ST_LF_START_ROBOT;
    }

    transFunc EXE_Start = &StartRobot;
    transFunc EXE_Track = &Track;
    transFunc EXE_StopRobot = &StopRobot;
    transFunc EXE_TurnRobot = &TurnRobot;
    
    
    void setupTransitions() {
        trans.clear();
        tTransition t0(ST_LF_START_ROBOT,  EV_LF_TRACK_LOST,  EXE_Start);
        tTransition t1(ST_LF_START_ROBOT,  EV_LF_TRACK_FOUND, EXE_Track);
        tTransition t2(ST_LF_START_ROBOT,  EV_LF_TIME_OUT,    EXE_Start);
        
        tTransition t3(ST_LF_TRACK,        EV_LF_TRACK_FOUND, EXE_Track);
        tTransition t4(ST_LF_TRACK,        EV_LF_TRACK_LOST,  EXE_StopRobot);
        
        tTransition t5(ST_LF_STOP_ROBOT,   EV_LF_TRACK_LOST,  EXE_TurnRobot);
        
        tTransition t6(ST_LF_TURN_ROBOT,   EV_LF_TRACK_LOST,  EXE_StopRobot);
        tTransition t7(ST_LF_TURN_ROBOT,   EV_LF_TRACK_FOUND, EXE_Track);
        tTransition t8(ST_LF_TURN_ROBOT,   EV_LF_TIME_OUT,    EXE_Start);
        
        trans.push_back(t0);
        trans.push_back(t1);
        trans.push_back(t2);
        trans.push_back(t3);
        trans.push_back(t4);
        trans.push_back(t5);
        trans.push_back(t6);
        trans.push_back(t7);
        trans.push_back(t8);
        // trans.push_back(t9);
        // trans.push_back(t10);
        INIT_STATE = ST_LF_START_ROBOT;
        NOF_TRANSITIONS = trans.size();
    }

    void stateMachine(int event, int& state, std::vector<void*>& actions) {
        bool done = false;
        int i;
        while (state != ST_LF_HALT_ROBOT && !done) {
            for (i = 0; i < NOF_TRANSITIONS && !done; i++) {
                if ((state == trans[i].st) || (SM_LF::ST_ANY == trans[i].st)) {
                    if ((event == trans[i].ev) || (SM_LF::EV_ANY == trans[i].ev)) {
                        state = (trans[i].fn)(actions);
                        done = true;
                    }
                }
            }
        }
    }
        
    
}; // namespace SM_LF





// This is the main top level control SM
namespace SM_CONTROL {
    typedef int (*transFunc)(std::vector<void*>&);
    struct tTransition {
        int st; // state
        int ev; // event
        transFunc fn; // action
        tTransition(int state, int event, transFunc func) :
            st(state),
            ev(event),
            fn(func) {}
    };
    int INIT_STATE;
    std::vector<tTransition> trans;
    int TRANS_COUNT;
    // Used to initate states
    bool INIT_LF_CONTROL;
    bool INIT_MANUAL_CONTROL;
    
    // Add manual control here
    int ManualControl(std::vector<void*>& actions) {
        INIT_LF_CONTROL = true;
        // Game control stuff here
        if (INIT_MANUAL_CONTROL) {
            // Stuff to initiate manual control
            INIT_MANUAL_CONTROL = false;
        }
        std::cout << "State: Manual Control" << std::endl;
        return ST_MANUAL_CONTROL;
    }

    // Line following SM
    int LineFollowControl(std::vector<void*>& actions) {
        GetPosition* getpos = reinterpret_cast<GetPosition*>(actions[1]);
        // getpos->getPositionsOnLine(0,0);
        INIT_MANUAL_CONTROL = true;
        if (INIT_LF_CONTROL) { // Go to initial LF state
            SM_LF::LF_EVENT = SM_LF::EV_LF_TRACK_LOST;
            SM_LF::LF_STATE = SM_LF::INIT_STATE;
            INIT_LF_CONTROL = false;
        } else
            SM_LF::LF_EVENT = getpos->IMG_EVENT;
        std::cout << "State: LF control" << std::endl;
        SM_LF::stateMachine(SM_LF::LF_EVENT,SM_LF::LF_STATE,actions);
        return SM_CONTROL::ST_LINE_FOLLOW_CONTROL;
    }

    transFunc EXE_ManualControl = &ManualControl;
    transFunc EXE_LineFollowControl = &LineFollowControl;

    
    void setupTransitions() {
        INIT_LF_CONTROL = true;
        trans.clear();
        tTransition t0(ST_MANUAL_CONTROL,       EV_MANUAL_CONTROL,       EXE_ManualControl);
        tTransition t1(ST_MANUAL_CONTROL,       EV_LINE_FOLLOW_CONTROL,  EXE_LineFollowControl);
        
        tTransition t2(ST_LINE_FOLLOW_CONTROL,  EV_MANUAL_CONTROL,       EXE_ManualControl);
        tTransition t3(ST_LINE_FOLLOW_CONTROL,  EV_LINE_FOLLOW_CONTROL,  EXE_LineFollowControl);
        
        trans.push_back(t0);
        trans.push_back(t1);
        trans.push_back(t2);
        trans.push_back(t3);
        INIT_STATE = ST_LINE_FOLLOW_CONTROL;
        TRANS_COUNT = trans.size();
    }

    
    void stateMachine(int event, int& state, std::vector<void*>& actions) {
        bool done = false;
        int i;
        while (!done) {
            for (i = 0; i < TRANS_COUNT && !done; i++) {
                if ((state == trans[i].st) || (SM_LF::ST_ANY == trans[i].st)) {
                    if ((event == trans[i].ev) || (SM_LF::EV_ANY == trans[i].ev)) {
                        state = (trans[i].fn)(actions);
                        done = true;
                    }
                }
            }
        }
    }

}; // namespace SM_CONTROL



#endif
