// Copyright Alten AB, Link�ping 2015
//
// Authors: S�ren Molander, Joel Nordh, Pierre �stlund
//

#ifndef __TOOLS__
#define __TOOLS__

#include "stdio.h"
#include "mytime.h"

class MeasureTime {
private:
    struct timeval start;
    struct timeval end;
    double _sec, _fps;
    int _counter;
public:
    MeasureTime();
    void measureTime();
    double getFPS();
    void updateTime();
    void resetTime();
    double timeLapsed();
};



#endif
