// Copyright Alten AB, Linköping 2015
//
// Authors: Sören Molander, Joel Nordh, Pierre Östlund
//

#ifndef __IMAGE_PROC__
#define __IMAGE_PROC__

#include "tools.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


// Colour segmentation parameters. Currently adapted for yellow/red segmentation
struct HSVParams {
    int iLowH;
    int iHighH;
    int iLowS;
    int iHighS;
    int iLowV;
    int iHighV;
    HSVParams() : 
        iLowH(30),
        iHighH(179),
        iLowS(50),
        iHighS(255),
        iLowV(180),
        iHighV(255) {}
};



// Derivative filter coeffs, line following positions, video extent
struct ImageProcParams {
    cv::Mat imgHSV, imgThresh, imgThreshS8,dist, dist_u8, image;
    double NEXT_POS, CURRENT_POS;
    int VIDEO_WIDTH;
    int VIDEO_HEIGHT;
    double ALLOWED_POS_DEV;
    int SHIFT_IN_PIXELS;
    std::vector<int> kernel;
    cv::Point sceneCenter;
    double tanAng;
    double ALLOWED_SCENE_CENTER_DIFF;
    int MIRROR_ERROR;
    ImageProcParams() {
        kernel.push_back(-1);
        kernel.push_back(0);
        kernel.push_back(1);
        NEXT_POS = 0.3; // relative position on line, perc of image
                           // height. 0= of image,1 = bottom
                           // of image
        MIRROR_ERROR = -1; // change polarity of error 
        CURRENT_POS = 0.8; // top of image = 0, bottom = 1
        VIDEO_WIDTH = 320; // desired video size 
        VIDEO_HEIGHT = int(floor(VIDEO_WIDTH*3/4));
        ALLOWED_POS_DEV = 3.0; // large values, high curvature of line allowed
        SHIFT_IN_PIXELS = 10; // shift up/down for robust x-derivative
                              // calc.
        sceneCenter = cv::Point(VIDEO_WIDTH/2,VIDEO_HEIGHT/2);
        tanAng = VIDEO_HEIGHT/double(VIDEO_WIDTH); // Not used for now
        ALLOWED_SCENE_CENTER_DIFF = 0.4; // If deviation from scene
                                         // center too large, change
                                         // control strategy
    }
};

// Image sensor processing to find line
class GetPosition {
public:
    HSVParams* _hsvparam;
    cv::Point prev_current, prev_next;
    int IMG_EVENT;
private:
    bool _timeConsistency; 
    bool _first_time;
    bool _signal_ok;
    double _poserr; // in pixels
    ImageProcParams* _imparam;
    cv::Point _extractPosition(cv::Mat&, double normHeight);
    cv::Point _averagePositions(cv::Mat&, double normHeight,bool& ok);
    double _normalizedSceneCenterDiff(const cv::Point& p, const cv::Point& center);
    bool _onVerticalEdge(const cv::Point&, const cv::Point&);
    MeasureTime _timer;
public:
    GetPosition(ImageProcParams*, HSVParams*);
    ~GetPosition();
    void cropImageVertically(ImageProcParams* imparam, double cropFactor,cv::Mat& in, cv::Mat& out);
    void setImage(cv::Mat&);
    void getPositionsOnLine(int, void*); // callback function from cv::trackbar
    double getErr() const {return _poserr;}
    bool signalStatus() const {return _signal_ok;}
    void drawDirection(cv::Point&, cv::Point&);
    ImageProcParams* getImParam() {return _imparam;}
    double L2norm(const cv::Point& p1, const cv::Point& p2);
    cv::Point average(const cv::Point& p1, const cv::Point& p2);
};




#endif
