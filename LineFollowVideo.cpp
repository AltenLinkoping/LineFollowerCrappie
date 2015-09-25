// Copyright Alten AB, Linköping 2015
//
// Authors: Sören Molander, Joel Nordh, Pierre Östlund
//

#include "globals.h"
#include "ImageProc.h"
#include "MotorControl.h"
#include "tools.h"
#include <math.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "statemachine.h"

#ifdef __linux__
#include "serial.h"
#endif

using namespace cv;
using namespace std;



bool INTERACTIVE_MODE = true;

// slidebar color segmentation callbacks
void updateCallbacks(GetPosition& getPos, int lowh, int highh, int lows, int highs, int lowv, int highv) {
    getPos._hsvparam->iLowH  = lowh;
    getPos._hsvparam->iHighH = highh;
    getPos._hsvparam->iLowS  = lows;
    getPos._hsvparam->iHighS = highs;
    getPos._hsvparam->iLowV  = lowv;
    getPos._hsvparam->iHighV = highv;
}


void on_manual_control(int apa, void* userdata) {
}


void callback(int apa, void* userdata) {
    GetPosition* getPos = reinterpret_cast<GetPosition*>(userdata);
    getPos->getPositionsOnLine(0,0);
}


bool test_manual_control(double time_in_ms, double period_in_ms) {
    bool manual_control;
    double time_in_period = fmod(time_in_ms, period_in_ms);
    manual_control = (time_in_period < period_in_ms/2 ? false : true);
    return manual_control;
}

// Main loop
int main(int argc, char** argv ) {
    // Set up state machine
    
    VideoCapture cap(0);
    if ( !cap.isOpened() ) {
        cout << "Cannot open the web cam" << endl;
        exit(-1);
    }

    ImageProcParams*  imageProcParams = new ImageProcParams;
    HSVParams* hsvParams = new HSVParams;
    GetPosition getPos(imageProcParams, hsvParams);

#ifdef __linux__
    ArduinoSerial* arduino = new ArduinoSerial;
    MotorControlParams* motorControlParams = new MotorControlParams;
    MotorControl motorControl(motorControlParams, arduino);
#else
    MotorControlParams* motorControlParams = new MotorControlParams;
    MotorControl motorControl(motorControlParams);
#endif
    
    cap.set(CV_CAP_PROP_FRAME_WIDTH, imageProcParams->VIDEO_WIDTH);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, imageProcParams->VIDEO_HEIGHT);

    int lowh =  getPos._hsvparam->iLowH;
    int highh = getPos._hsvparam->iHighH;
    int lows =  getPos._hsvparam->iLowS;
    int highs = getPos._hsvparam->iHighS;
    int lowv =  getPos._hsvparam->iLowV;
    int highv = getPos._hsvparam->iHighV;

    int control_min = 0;
    int control_max = 1;
    int control_slider = 0;
    
    if (INTERACTIVE_MODE) {
        namedWindow("Display Image", WINDOW_AUTOSIZE );
        namedWindow("Control",0);        
        namedWindow("Dz Image", WINDOW_AUTOSIZE );
        createTrackbar("Man override", "Control", &control_slider, control_max,on_manual_control);
        createTrackbar("LowH", "Control",&lowh, 179, callback,(void*)(&getPos));//Hue (0 - 179)
        createTrackbar("HighH", "Control", &highh, 179,callback,(void*)(&getPos)); 
        createTrackbar("LowS", "Control", &lows, 255,callback,(void*)(&getPos)); //Satuation (0-255)
        createTrackbar("HighS", "Control", &highs, 255,callback,(void*)(&getPos));
        createTrackbar("LowV", "Control", &lowv, 255,callback,(void*)(&getPos)); // Value (0..255)
        createTrackbar("HighV", "Control", &highv, 255,callback,(void*)(&getPos));
    }
    
    
    Mat image, imageCrop;
    double cropFactor = 0.5;
    MeasureTime getTime;
    MeasureTime timer;
    bool manual_control = false;
    int test_time_period = 5000; // ms

    vector<void*> actions;
    actions.push_back((void*)(&motorControl));
    actions.push_back((void*)(&getPos));

    SM_LF::setupTransitions();
    SM_CONTROL::setupTransitions();
    SM_CONTROL::CONTROL_STATE = SM_CONTROL::INIT_STATE;
    // LF_EVENT = EV_LF_TIME_OUT;
    // LF_STATE = SM_LF::INIT_STATE;
    while (true) {
        bool ok = cap.read(image);
        if (!ok) {
            cout << "Cannot read a frame from video stream" << endl;
            break;
        }
        getPos.cropImageVertically(imageProcParams, cropFactor, image,imageCrop);
        getPos.setImage(imageCrop);

         // manual_control = test_manual_control(timer.timeLapsed()*1000,test_time_period);
         // CONTROL_EVENT = (manual_control ? EV_MANUAL_CONTROL : EV_LINE_FOLLOW_CONTROL);
         // cout << "manual control = " << manual_control << endl;
        if (control_slider > 0)
            SM_CONTROL::CONTROL_EVENT = SM_CONTROL::EV_MANUAL_CONTROL;
        else
            SM_CONTROL::CONTROL_EVENT = SM_CONTROL::EV_LINE_FOLLOW_CONTROL;
        SM_CONTROL::stateMachine(SM_CONTROL::CONTROL_EVENT, SM_CONTROL::CONTROL_STATE, actions);
        // cout << "Manual control= " << MANUAL_CONTROL << endl;
        // cout << "control slider = " << control_slider << endl;
        // SM_LF::stateMachine(LF_EVENT, LF_STATE, actions);

        // motorControl.drive(getPos.getErr(), getPos.signalStatus());
        // if (!getPos.signalStatus())
        //     motorControl.halt();
        if (INTERACTIVE_MODE) {
            imshow("Dz Image", getPos.getImParam()->dist_u8);
            imshow("Display Image", getPos.getImParam()->image);
            updateCallbacks(getPos,lowh,highh,lows,highs,lowv,highv);
        }
         // getTime.updateTime();
        // cout << "\r framrate = " << getTime.getFPS() << ", time lapsed = " << getTime.timeLapsed();

        
        if (waitKey(10) == 27) {
            cap.release();
            break; 
        }
    }
    
    
    return 0;
}
    
