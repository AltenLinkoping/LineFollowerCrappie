// Copyright Alten AB, Linköping 2015
//
// Authors: Sören Molander, Joel Nordh, Pierre Östlund
//

#ifndef __MOTOR_CONTROL__
#define __MOTOR_CONTROL__

#include <string>
#include <fstream>
#include <iostream>
#include "tools.h"
#include "navigation.h"

#ifdef __linux__
#include "serial.h"
#endif

// PID parameters
struct MotorControlParams {
    double I;
    double Kp;
    double Kd;
    double Ki;
    double Ts;
    double INTEGRAL_ERROR_THRESH;
    int MAX_MOTOR_SPEED;
    int MIN_MOTOR_SPEED;
    int MAX_SPEED_DIFF;
    int DESIRED_SPEED;
    int TURN_DURATION;
    bool logData;
    int MOTOR_AMPLIFICATION;
    MotorControlParams() :
        I(0),
        Kp(1),
        Kd(0),
        Ki(0),
        Ts(1.0/30.0),
        INTEGRAL_ERROR_THRESH(1E10),
        MAX_MOTOR_SPEED(255),
        MIN_MOTOR_SPEED(50),
        DESIRED_SPEED(100),
        MAX_SPEED_DIFF(60),
        TURN_DURATION(700),
        logData(false),
        MOTOR_AMPLIFICATION(200) {}
};



// Motor commands
class MotorControl {
private:
    std::string logfile;
    std::ofstream _ofs;
    MotorControlParams* _motorparam;
    bool _first_time;
    MeasureTime _timer;
#ifdef __linux__
  ArduinoSerial* _arduino;
#endif
  double _PID_regulate_position(double err, double Ts, double err_thresh);
  void _scaleToMotorCommand(double error, int desired_speed, int scale_factor, int& left_motor, int& right_motor);
  void _driveMotors(int left, int right);
    void logDataToFile(double time, double err, double pid_signal, int left, int right);
 public:
    int LEFT;
    int RIGHT;
#ifdef __linux__
    MotorControl(MotorControlParams*, ArduinoSerial*);
#else
    MotorControl(MotorControlParams*);
#endif
    ~MotorControl();
    void drive(double err, bool signalStatus);
    void halt();
    void turn(double err, int direction, int duration); // turn during duration (ms)
    void turn(double err, int direction, double desired_angle); // Turn using navdata
    MotorControlParams* getParams() const {return _motorparam;}
};


#endif
