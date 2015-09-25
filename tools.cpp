// Copyright Alten AB, Link�ping 2015
//
// Authors: S�ren Molander, Joel Nordh, Pierre �stlund
//

#include "tools.h"

MeasureTime::MeasureTime() {
    _counter = 0;
    gettimeofday(&start,NULL);
}

void MeasureTime::updateTime() {
    ++_counter;
}

void MeasureTime::resetTime() {
    _counter = 0;
    gettimeofday(&start,NULL);
}


double MeasureTime::getFPS() {
    gettimeofday(&end,NULL);
    double diff = (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec)/1.0E6;
    return _counter / double(diff);
}
// Time lapsed in s
double MeasureTime::timeLapsed() {
    gettimeofday(&end,NULL);
    double diff = (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec)/1.0E6;
    // cout << "time = " << diff << endl;
    return diff;
}

