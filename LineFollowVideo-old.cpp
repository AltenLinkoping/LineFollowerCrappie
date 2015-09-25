#include <stdio.h>
#include <algorithm>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

Mat image, imageGray, imgHSV, imgThresh;

int iLowH=50;
int iHighH=179;
int iLowS=50;
int iHighS=255;
int iLowV=120;  
int iHighV=255;
bool first_time = true;
double I = 0;
double NEXT_POS = 0.6; // relative position on line, perc of image height. zero = ULC = URC
double CURRENT_POS = 0.9;
int VIDEO_WIDTH = 640;
int VIDEO_HEIGHT = int(floor(VIDEO_WIDTH*4/4));
double ALLOWED_POS_DEV = 2.0; // large values, less curvature of line allowed
// PID control params
double Kp = 0.5;
double Kd = 0.02;
double Ki = 4;
double Ts = 1/20;
int INTEGRAL_ERROR_THRESH = 200;
int MAX_MOTOR_SPEED= 200; // 255 is max
int MIN_MOTOR_SPEED = 0;
int DESIRED_SPEED = 100;
Point prev_cur_point, prev_next_point;


Point goal_position(int(VIDEO_WIDTH/2),int(VIDEO_HEIGHT/2));


int binaryCount(Mat& bin) {
    return countNonZero(bin);
}

double L2norm(const Point& p1, const Point& p2) {
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
}



// take 1D derivative filter in x-direction for a given row. Extract
// min max positions and calculate the center position. Assumes good
// edges :-)
Point extractPosition(const Mat& image, double normHeight, vector<int>& kernel) {
    // assert(image.channels() == 1);
    vector<double> filter(image.cols,0);
    int current_row = int(image.rows*normHeight);
    int nof_cols = image.cols;
    double sum = 0;
    Scalar intensity;
    int kernel_radius = int(floor(kernel.size()/2));
    int j=0;
    for ( int c=kernel_radius;c<nof_cols-kernel_radius;c++) {
        sum = 0.0;
        for ( int k=-kernel_radius;k<=kernel_radius;k++) {
            intensity = image.at<uchar>(current_row,c+k);
            sum += kernel[k+kernel_radius]*intensity.val[0];
        }
        filter[j++] = sum;
    }
    int max_ind = distance(filter.begin(),max_element(filter.begin(),filter.end()));
    int min_ind = distance(filter.begin(),min_element(filter.begin(),filter.end()));
    return Point((max_ind+min_ind)/2, current_row);
}

// At least 2 points have to agree and be < threshold
Point averagePositions(const Mat& image, double normHeight, vector<int>& kernel, bool& ok) {
    int shift = 10;
    double normShift = shift/(double)image.rows;
    Point p = extractPosition(image, normHeight, kernel);
    Point pl = extractPosition(image, (normHeight+normShift), kernel);
    Point ph = extractPosition(image, (normHeight-normShift), kernel);
    Point outP;
    double dpl, dph, dlh;
    dpl = L2norm(pl,p)/(double)shift;
    dph = L2norm(p,ph)/(double)(shift);
    dlh = L2norm(pl,ph)/(double)(2*shift);
    ok = false;
    
    if ((dpl < ALLOWED_POS_DEV) && (dph < ALLOWED_POS_DEV)) {
        ok = true;
        outP = p;
    } else if ((dpl < ALLOWED_POS_DEV) && (dlh < ALLOWED_POS_DEV)) {
        ok = true;
        outP = p;
    } else if ((dph < ALLOWED_POS_DEV) && (dlh < ALLOWED_POS_DEV)) {
        ok = true;
        outP = p;
    }
        
    return outP;
}


// Regulate position (e.g. width in the middle of the image)
// Tuning hints: "good" values for Kp ~ 0.5, Kd ~ 0.02, Ki ~ 3
double PID_regulate_position(double err, double Ts, double err_thresh) {
    static double prev_err;
    double D, u;
    if (first_time)
        prev_err = 0;
    else
        first_time = false;
    D = (err-prev_err)/Ts;
    if (fabs(err) < err_thresh) // prevent windup
        I += err*Ts;
    else
        I = 0;
    u = Kp*err + Ki*I + Kd*D;
    prev_err = err;
    return u;
}


void scaleToMotorCommand(int error, int& left_motor, int& right_motor, int desired_speed) {
    if (error < 0) {
        left_motor = std::max(std::min(desired_speed + error,MAX_MOTOR_SPEED),MIN_MOTOR_SPEED);
        right_motor = std::min(desired_speed, MAX_MOTOR_SPEED);
    } else if (error > 0) {
        right_motor = std::min(desired_speed, MAX_MOTOR_SPEED);
        left_motor = std::max(std::min(desired_speed - error,MAX_MOTOR_SPEED),MIN_MOTOR_SPEED);
    }
}


void driveMotors(int left, int right) {
    

}


// Callback for 
void on_trackbar(int, void*) {
    Mat dist_u8, imgThreshS8;
    vector<int> filter;
    filter.push_back(-1); 
    filter.push_back(0);
    filter.push_back(1);
    inRange(imgHSV,Scalar(iLowH,iLowS,iLowV),Scalar(iHighH,iHighS,iHighV),imgThresh);
    
    erode(imgThresh,imgThresh, getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
    dilate(imgThresh,imgThresh, getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
    dilate(imgThresh,imgThresh, getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
    erode(imgThresh,imgThresh, getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
    
    imgThresh.convertTo(dist_u8,CV_8U);
    imgThresh.convertTo(imgThreshS8,CV_8S);
    bool ok1, ok2;
    Point p1 = averagePositions(imgThreshS8,CURRENT_POS, filter, ok1);
    Point p2 = averagePositions(imgThreshS8,NEXT_POS, filter, ok2);

    // Require that the distance between points on line < factor
    if ((L2norm(p1,p2) < imgThresh.rows*(CURRENT_POS-NEXT_POS)*ALLOWED_POS_DEV) &&
        (ok1 && ok2)) {
        circle(image, p1, 5, Scalar(0,255,0), 2, 8);
        circle(image, p2, 5, Scalar(0,255,0), 2, 8);
        line(image, p1, p2, Scalar (110,220,0), 2);
        double errWidth = p1.x - p2.x;
    }
    
    imshow("Dz Image", dist_u8);
    imshow("Display Image", image);
    
    // imshow("Dx",Dx);
}


int main(int argc, char** argv ) {
    VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT);

    
    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }

    // namedWindow("Control", CV_WINDOW_AUTOSIZE);
    namedWindow("Control");

    createTrackbar("LowH", "Control", &iLowH, 179, on_trackbar); //Hue (0 - 179)
    createTrackbar("HighH", "Control", &iHighH, 179, on_trackbar);
    
    createTrackbar("LowS", "Control", &iLowS, 255, on_trackbar); //Saturation (0 - 255)
    createTrackbar("HighS", "Control", &iHighS, 255, on_trackbar);
    
    createTrackbar("LowV", "Control", &iLowV, 255, on_trackbar);//Value (0 - 255)
    createTrackbar("HighV", "Control", &iHighV, 255, on_trackbar);
    
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    namedWindow("Dz Image", WINDOW_AUTOSIZE );

    while (true) {
        bool ok = cap.read(image);
        // resize(image,image,Size(),0.4,0.4,INTER_LINEAR);
        
        if (!ok) {
            cout << "Cannot read a frame from video stream" << endl;
            break;
        }
        cvtColor(image, imgHSV, COLOR_RGB2HSV);
        on_trackbar(0,0);
        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            break; 
        }

    }

    return 0;
}
