// Copyright Alten AB, Linköping 2015
//
// Authors: Sören Molander, Joel Nordh, Pierre Östlund
//
//

#include "globals.h"
#include "ImageProc.h"
#include <math.h>
#include <stdio.h>
#include <algorithm>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#ifdef __linux__
#include "serial.h"
#endif



using namespace std;
using namespace cv;

GetPosition::GetPosition(ImageProcParams* imparam, HSVParams* hsvpar) :
    _imparam(imparam),
    _hsvparam(hsvpar) {
    _poserr = 0.0;
    _first_time = true;
    }

void GetPosition::setImage(Mat& image) {
    _imparam->image = image;
}

GetPosition::~GetPosition() {
    if (!_hsvparam)
        delete _hsvparam;
    if (!_imparam)
        delete _imparam;
}


Point GetPosition::average(const Point& p1, const Point& p2) {
    Point aver((p1.x+p2.x)/2,(p1.y+p2.y)/2);
    return aver;
}

double GetPosition::L2norm(const cv::Point& p1, const cv::Point& p2) {
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}
         

// Get normalized diff between center and point along x-axis
double GetPosition::_normalizedSceneCenterDiff(const Point& p, const Point& center) {
    int x_diff = p.x-center.x;
    double err = double(x_diff) / double(_imparam->VIDEO_WIDTH/2);
    return err;
}


// take 1D derivative filter in x-direction for a given row. Extract
// min max positions and calculate the center position. Assumes good
// edges :-)
Point GetPosition::_extractPosition(Mat& image, double normHeight) {
    Point out;
    vector<double> filter(image.cols,0);
    // assert(_imparam.kernel.size() > 0);
    int current_row = int(image.rows*normHeight);
    int nof_cols = image.cols;
    double sum;
    Scalar intensity;
    int kernel_radius = int(floor(_imparam->kernel.size()/2));
    int j=0;
    for ( int c=kernel_radius;c<nof_cols-kernel_radius;c++) {
        sum = 0.0;
        for ( int k=-kernel_radius;k<=kernel_radius;k++) {
            intensity = image.at<signed char>(current_row,c+k);
            sum += _imparam->kernel[k+kernel_radius]*intensity.val[0];
        }
        filter[j++] = sum;
    }
     int max_ind = distance(filter.begin(),max_element(filter.begin(),filter.end()));
     int min_ind = distance(filter.begin(),min_element(filter.begin(),filter.end()));
     return Point((max_ind+min_ind)/2, current_row);
}


// At least 2 points have to agree and be < threshold
Point GetPosition::_averagePositions(Mat& image, double normHeight, bool& ok) {
    Point outP;
    int shift = _imparam->SHIFT_IN_PIXELS; // shift in pixels
    double normShift = shift/(double)image.rows;
    Point p = _extractPosition(image,normHeight);
    Point pl = _extractPosition(image,normHeight+normShift);
    Point ph = _extractPosition(image,normHeight-normShift);
    double dpl, dph, dlh;
    dpl = L2norm(pl,p)/(double)shift;
    dph = L2norm(p,ph)/(double)(shift);
    dlh = L2norm(pl,ph)/(double)(2*shift);
    ok = false;
    
    if ((dpl < _imparam->ALLOWED_POS_DEV) && (dph < _imparam->ALLOWED_POS_DEV)) {
        ok = true;
        outP = p;
    } else if ((dpl < _imparam->ALLOWED_POS_DEV) && (dlh < _imparam->ALLOWED_POS_DEV)) {
        ok = true;
        outP = p;
    } else if ((dph < _imparam->ALLOWED_POS_DEV) && (dlh < _imparam->ALLOWED_POS_DEV)) {
        ok = true;
        outP = p;
    }
    return outP;
}


bool GetPosition::_onVerticalEdge(const Point& p1, const Point& p2) {
    return (p1.x == 0) || (p2.x == 0);
}
    

void GetPosition::drawDirection(Point& p1, Point& p2) {
    circle(_imparam->image, p1, 5, Scalar(0,255,0), 2, 8);
    circle(_imparam->image, p2, 5, Scalar(0,255,0), 2, 8);
    line(_imparam->image, p1, p2, Scalar (110,220,0), 2);
}


// Public method for image processing
// 1. Color segment image using HSV (currently red line)
// 2. Threshold image (optional step: morphological open and close)
// 3. robust x-derivative (shift up and down to comensate for holes)
// 4. Error signal is the normalized difference of next
// and current x-positions on line. Combinations are possible such as the sum of different errors
void GetPosition::getPositionsOnLine(int, void*) {
    // openCV functions
    bool ok1, ok2;
    Point p1,p2;
    double sceneCenterError, lineFollowError;
    _poserr = 0.0;
    if (_first_time) {
        prev_current = Point(_imparam->VIDEO_WIDTH/2,_imparam->VIDEO_HEIGHT/2);
        prev_next = Point(_imparam->VIDEO_WIDTH/2,_imparam->VIDEO_HEIGHT/2+20);
        _timeConsistency = true;
        _first_time = false;
    }
    cvtColor(_imparam->image, _imparam->imgHSV, COLOR_RGB2HSV);
    inRange(_imparam->imgHSV,Scalar(_hsvparam->iLowH,_hsvparam->iLowS,_hsvparam->iLowV),
            Scalar(_hsvparam->iHighH,_hsvparam->iHighS,_hsvparam->iHighV),_imparam->imgThresh);
    
    _imparam->imgThresh.convertTo(_imparam->dist_u8,CV_8U);
    _imparam->imgThresh.convertTo(_imparam->imgThreshS8,CV_8S); // derivative uses both neg andpos values
    
    p1 = _averagePositions(_imparam->imgThreshS8,_imparam->CURRENT_POS,ok1); // x-derivatives
    p2 = _averagePositions(_imparam->imgThreshS8,_imparam->NEXT_POS,ok2);
    _signal_ok = true;
     // For robustness require that the distance between points on line < vertical_dist*factor
    double lineDist = L2norm(p1,p2);
    double verticalDist  = _imparam->imgThresh.rows*(_imparam->CURRENT_POS-_imparam->NEXT_POS);
    
    
    if  (lineDist < verticalDist *_imparam->ALLOWED_POS_DEV &&
         (ok1 && ok2) && !_onVerticalEdge(p1,p2) &&
         _timeConsistency) {

        drawDirection(p1,p2);
        Point aver = average(p1,p2);
        sceneCenterError = _normalizedSceneCenterDiff(p1,_imparam->sceneCenter);
        lineFollowError = double((p2.x - p1.x)/(double)_imparam->VIDEO_WIDTH);

       // _poserr =  lineFollowError;
        // _poserr = sceneCenterError + lineFollowError;
        // _poserr = sceneCenterError*_imparam->MIRROR_ERROR;
        _poserr = sceneCenterError*_imparam->MIRROR_ERROR;
        _timer.resetTime();
        // if (fabs(sceneCenterError) > _imparam->ALLOWED_SCENE_CENTER_DIFF) {
        //     cout << "SCENE CENTER PID, sceneCenterError = " << sceneCenterError << endl;
        //     _poserr = sceneCenterError;
        // } else { // regulate on direction
        //     cout << "FOLLOW LINE PID, sceneCenterError = " << sceneCenterError << endl;
        //     _poserr = lineFollowError*_imparam->MIRROR_ERROR;
        // }
        prev_current = p1;
        prev_next = p2;
        IMG_EVENT = SM_LF::EV_LF_TRACK_FOUND;
        cout << "getPositionsOnLine: EV_TRACK_FOUND" << endl;
    } else { // if dropout, use previous positions
        _timer.updateTime();
        // _poserr = double((prev_current.x - _imparam->VIDEO_WIDTH/2)/(double)_imparam->VIDEO_WIDTH);
        Point aver = average(prev_current,prev_next);
        sceneCenterError = _normalizedSceneCenterDiff(aver,_imparam->sceneCenter);
        _poserr = sceneCenterError*_imparam->MIRROR_ERROR;
        IMG_EVENT = SM_LF::EV_LF_TRACK_LOST;
        cout << "getPositionsOnLine: EV_TRACK_LOST" << endl;
        // cout << "Timer " << _timer.timeLapsed() << endl;
        if (_timer.timeLapsed()*1000 > LF_TIME_OUT) { // wait for TIME_OUT (s) then stop motors
            _signal_ok = false;
            IMG_EVENT = SM_LF::EV_LF_TIME_OUT;
            cout << "getPositionsOnLine: EV_TIME_OUT" << endl;
            // cout << "getPositionsOnLine: EV_TRACK_LOST" << endl;
        }
            
    }
    if (IMG_EVENT == SM_LF::EV_LF_TRACK_FOUND)
        _timeConsistency = (L2norm(prev_current,p1) < 50 && L2norm(prev_next, p2) < 50);
    else
        _timeConsistency = true;
}

// Vertical cropping of image, cropFactor = [0..1]. 0=no cropping, 0.7
// = 70 % of rows removed
void GetPosition::cropImageVertically(ImageProcParams* imparam, double cropFactor,Mat& in, Mat& out) {
    int imwidth = in.cols;
    int imheight = in.rows;
    Rect roi(0,int(imheight*cropFactor),imwidth-1,int(imheight*(1-cropFactor)-1));
    Mat cropped_ref(in,roi);
    cropped_ref.copyTo(out);
}
