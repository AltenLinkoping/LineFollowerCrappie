// Copyright Alten AB, Linköping 2015
//
// Authors: Sören Molander, Joel Nordh, Pierre Östlund
//

#include <math.h>
#include <stdio.h>
#include <fstream>
#include <algorithm>
#include <iostream>
#include "MotorControl.h"
#include "tools.h"

#ifdef __linux__
#include "serial.h"
#endif

using namespace std;


#ifdef __linux__
MotorControl::MotorControl(MotorControlParams* mparm, ArduinoSerial* arduino) : 
    _motorparam (mparm),
    LEFT(1),
    RIGHT(-1),
    _arduino(arduino) {
    if (_motorparam->logData) {
        _ofs.open("controldata.txt");
    }
    }
#else
MotorControl::MotorControl(MotorControlParams* mparm) : 
    _motorparam (mparm) {
    _first_time = true;
    if (_motorparam->logData) {
        _ofs.open("controldata.txt");
    }
    
}
#endif


MotorControl::~MotorControl() {
    if (_motorparam->logData)
        _ofs.close();
    if (!_motorparam)
        delete _motorparam;
}

// PID of error signal
// Error is normalized to [-1..1]
double MotorControl::_PID_regulate_position(double err, double Ts, double err_thresh) {
    static double prev_err;
    double Kp = _motorparam->Kp;
    double Ki = _motorparam->Ki;
    double Kd = _motorparam->Kd;
    double D, u;
    if (_first_time) {
        _first_time = false;
        prev_err = 0;
    }



    D = (err-prev_err)/Ts;
    if (fabs(err) < err_thresh) // prevent windup
        _motorparam->I += err*Ts;
    else
        _motorparam->I = 0;
    u = Kp*err + Ki*_motorparam->I + Kd*D;
    prev_err = err;
    return u;
}


// Error is normalized to [-1..1], rescale to Motor input
void MotorControl::_scaleToMotorCommand(double error,
                                        int desired_speed,
                                        int scale_factor,
                                        int& left_motor,
                                        int& right_motor) {
    double EXPONENT = 0.2;
    int scaled_error = int(error*scale_factor);
    int MAX_SPEED_DIFF = _motorparam->MAX_SPEED_DIFF;
    int left_signal = desired_speed + scaled_error;
    int right_signal = desired_speed - scaled_error;
    
    if (scaled_error < 0) {
        left_motor  = max(left_signal,desired_speed - MAX_SPEED_DIFF);
        right_motor = min(right_signal,desired_speed + MAX_SPEED_DIFF);
    } else {
        left_motor  = min(left_signal,desired_speed + MAX_SPEED_DIFF);
        right_motor = max(right_signal,desired_speed - MAX_SPEED_DIFF);
    }
}


void MotorControl::_driveMotors(int left, int right) {
#ifdef __linux__    
    _arduino->engine_forward(_arduino->LEFT_MOTOR, left);
    _arduino->engine_forward(_arduino->RIGHT_MOTOR, right);
#else    
    cout << "Motor: left, right = " << left << ", " << right << endl;
#endif 
    
}

    
void MotorControl::drive(double error, bool signalStatus) {
    int right_motor, left_motor;
    double pidout;
   
    // int SCALE_FACTOR =  _motorparam->MAX_MOTOR_SPEED;
    int SCALE_FACTOR =  _motorparam->MOTOR_AMPLIFICATION;
    
    if (signalStatus) {
        pidout = _PID_regulate_position(error, _motorparam->Ts, _motorparam->INTEGRAL_ERROR_THRESH);
        _scaleToMotorCommand(pidout, _motorparam->DESIRED_SPEED, SCALE_FACTOR,left_motor, right_motor);
        _driveMotors(left_motor, right_motor);
        logDataToFile(_timer.timeLapsed(), error, pidout, left_motor, right_motor);
    }
}

void MotorControl::halt() {
#ifdef __linux__    
  _arduino->engine_halt(_arduino->LEFT_MOTOR); 
  _arduino->engine_halt(_arduino->RIGHT_MOTOR);
#else    
  cout << "Halting motor!" << endl;
#endif   
}



// Temporary interface, should use absoluate turn angle from rate sensors
void MotorControl::turn(double error, int direction, int duration) { // duration in ms
#ifdef __linux__   
  MeasureTime T;
  int SCALED_ERROR =  _motorparam->MOTOR_AMPLIFICATION;
  double pidout = _PID_regulate_position(error, _motorparam->Ts, _motorparam->INTEGRAL_ERROR_THRESH);
  int scaled_error = int(error*SCALED_ERROR);
  int MAX_SPEED_DIFF = _motorparam->MAX_SPEED_DIFF;
  int signal =  (fabs(scaled_error) < MAX_SPEED_DIFF ? scaled_error : MAX_SPEED_DIFF);

  // while (T.timeLapsed()*1000 < duration) {
  //     _arduino->engine_backwards(_arduino->LEFT_MOTOR, signal); 
  //     _arduino->engine_forward(_arduino->RIGHT_MOTOR, signal);
  // }

  
  if (direction <= LEFT) {
      while (T.timeLapsed()*1000 < duration) {
          _arduino->engine_backwards(_arduino->LEFT_MOTOR, signal); 
          _arduino->engine_forward(_arduino->RIGHT_MOTOR, signal);
      }
  } else if (direction >= RIGHT) {
      while (T.timeLapsed()*1000 < duration) {
          _arduino->engine_forward(_arduino->LEFT_MOTOR, signal);
          _arduino->engine_backwards(_arduino->RIGHT_MOTOR, signal);
      }
  }

#else    
cout << "Turning!" << endl;
#endif   
}


// Interface using real navdata
void MotorControl::turn(double error, int direction, double desired_angle) { 
#ifdef __linux__   
  MeasureTime T;
  int SCALED_ERROR =  _motorparam->MOTOR_AMPLIFICATION;
  double pidout = _PID_regulate_position(error, _motorparam->Ts, _motorparam->INTEGRAL_ERROR_THRESH);
  int scaled_error = int(error*SCALED_ERROR);
  int MAX_SPEED_DIFF = _motorparam->MAX_SPEED_DIFF;
  int signal =  (scaled_error < MAX_SPEED_DIFF ? scaled_error : MAX_SPEED_DIFF);
  double angle = 0.0;
  int* mpu_data;
  NavigationData navdata;
  SimpleNavigation snav;
  INavigation* nav = &snav;  
  
  while (angle < desired_angle) {
      _arduino->get_mpu_data(mpu_data);
      nav->calcFusedAngles(mpu_data);
      navdata = nav->getNavData();
      angle += double(navData.rate[0]*T.timeLapsed()); // Yaw channel
      if (direction <= LEFT) {          
          _arduino->engine_backwards(_arduino->LEFT_MOTOR, signal); 
          _arduino->engine_forward(_arduino->RIGHT_MOTOR, signal);
      } else {
          _arduino->engine_forward(_arduino->LEFT_MOTOR, signal);
          _arduino->engine_backwards(_arduino->RIGHT_MOTOR, signal);
      }
  }
  
#else    
cout << "Turning!" << endl;
#endif   
}



void MotorControl::logDataToFile(double time, double raw_error, double pid_output, int left, int right) {
    if (_motorparam->logData) {
        _ofs.precision(8);
        _ofs << time << ", " << raw_error << " ," << pid_output << ", " << left << ", " << right << endl;
    }
}
