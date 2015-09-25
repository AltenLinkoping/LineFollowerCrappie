// Copyright Alten AB, Linköping 2015
//
// Authors: Sören Molander, Joel Nordh, Pierre Östlund
//

#include "navigation.h"

using namespace std;

/////////////////////////////////////////////////////////////////////////
/////////// Simple body coordinate navigation ///////////////////////////
/////////////////////////////////////////////////////////////////////////


// Assume Tait-Bryan convention, yaw, pitch, roll.
// Yaw = rotation along z-axis, pitch = rotation along y, roll = rotation along x
//
// Use experiments to figure out the different axes and the indeces in mpu_data vector
SimpleNavigation::SimpleNavigation() : 
    indPitch(3),
    indYaw(4),
    indRoll(5),
    indAccx(0),
    indAccy(1),
    indAccz(2) {
    _filterCoeff = _navDataBodyRaw.tau / (_navDataBodyRaw.tau + _Ts);
    _Ts = _navDataBodyRaw.Ts;
}

// raw vector from MPU 6050 chip:
// mpu_data[0] = accx
// mpu_data[1] = accy
// mpu_data[2] = accz
// mpu_data[3] = pitch
// mpu_data[4] = roll
// mpu_data[5] = yaw
// Integrate rotation angles from gyro rates. This will contain large drifts
// also need to figure out which angles are yaw, pitch, roll
void SimpleNavigation::_calcRawAngles(int* mpu_data) {
    int gs = _navDataBodyRaw.GYRO_SENSITIVITY;
    double yaw = (double(mpu_data[indYaw] / gs))*_Ts;
    double pitch = (double(mpu_data[indPitch] / gs))*_Ts;
    double roll = (double(mpu_data[indRoll] / gs))*_Ts;
    _navDataBodyRaw.rate[0] =  (double(mpu_data[indYaw] / gs));
    _navDataBodyRaw.rate[1] =  (double(mpu_data[indPitch] / gs));
    _navDataBodyRaw.rate[2] =  (double(mpu_data[indRoll] / gs));

    _navDataBodyRaw.acc[0] =  (double(mpu_data[indAccx] / gs));
    _navDataBodyRaw.acc[1] =  (double(mpu_data[indAccy] / gs));
    _navDataBodyRaw.acc[2] =  (double(mpu_data[indAccz] / gs));
    
    _navDataBodyRaw.angle[0] += yaw; 
    _navDataBodyRaw.angle[1] += pitch;
    _navDataBodyRaw.angle[2] += roll;

}

// calculate instantaneous body angles from body accelerations (noisy)
// Since angles are relative, the physical units are divided out.
// Assumes NED system, navchip is aligned z-axis orthogonal to gravity vector
// X-axis is forwards, y-axis to the left of the forward direction 
void SimpleNavigation::_calcAccAngles(int* mpu_data) {
    int ax = mpu_data[indAccx];
    int ay = mpu_data[indAccy];
    int az = mpu_data[indAccz];
    _navDataBodyRawFromAcc.angle[0] = atan2(sqrt(ax*ax+ay*ay),az); // yaw
    _navDataBodyRawFromAcc.angle[1] = atan2(ay,sqrt(ax*ax+az*az)); // pitch
    _navDataBodyRawFromAcc.angle[2] = atan2(ax,sqrt(ay*ay+az*az)); // roll
}


// Fuse angle and gyro rate angles with a simple "complementary" filter (experimental)
void SimpleNavigation::_calcMixedAngles(int* mpu_data) {
    int AccMagnitude = int(sqrt(mpu_data[indAccx]*mpu_data[indAccx]+
                                mpu_data[indAccy]*mpu_data[indAccy]+
                                mpu_data[indAccz]*mpu_data[indAccz]));
    if (AccMagnitude < _navDataBodyRaw.ACC_SENSITIVITY) {
        double yawAcc = _navDataBodyRawFromAcc.angle[0];
        _navDataBodyRaw.angle[0] = _navDataBodyRaw.angle[0]*_filterCoeff + yawAcc*(1-_filterCoeff);
        
        double pitchAcc = _navDataBodyRawFromAcc.angle[1];
        _navDataBodyRaw.angle[1] = _navDataBodyRaw.angle[1]*_filterCoeff + pitchAcc*(1-_filterCoeff);
        
        double rollAcc = _navDataBodyRawFromAcc.angle[2];
        _navDataBodyRaw.angle[2] = _navDataBodyRaw.angle[2]*_filterCoeff + rollAcc*(1-_filterCoeff);
    }
    _navDataBodyMixed = _navDataBodyRaw;
}

void SimpleNavigation::calcFusedAngles(int* mpu_data) {
    _calcRawAngles(mpu_data);
    _calcAccAngles(mpu_data);
    _calcMixedAngles(mpu_data);
}

void SimpleNavigation::calcWorldState(int* mpu_data) {
// empty for now
}

/////////////////////////////////////////////////////////////////////////
/////////// Advanced Kalman filter for navigation in world coordinates //
/////////////////////////////////////////////////////////////////////////

// Statevector, local body 2D coords: 
// theta (angle)
// gyro bias
// gyro rate (z-direction)
// x pos
// y pos
// speed
//
// measurements: accelerations and gyro rates. 
// 
//


// KalmanNavigation::KalmanNavigation(double Ts) {
//     _Ts=Ts;
// }

// void KalmanNavigation::_toPhysicalAcc(int* mpu_data) {
//     int as = navData.ACC_SENSITIVITY;
//     navData.acc[0] = mpu_data[0]/as;
//     navData.acc[1] = mpu_data[1]/as;
//     navData.acc[2] = mpu_data[2]/as;
// }

// void KalmanNavigation::_toPhysicalRates(int* mpu_data) {
//     int gs = navData.GYRO_SENSITIVITY;
//     navData.rate[0] = mpu_data[0]/gs;
//     navData.rate[1] = mpu_data[1]/gs;
//     navData.rate[2] = mpu_data[2]/gs;
// }

// TranslationState KalmanNavigation::predict(TranslationState& X,
//                                            CovarianceMatrix& P,
//                                            int* mpu_data) {
// //  TBI
// }

// // rates, accelerations and bias
// KalmanNavigation::measure(TranslationState& translation_state,
//                           RotationState& rotation_state,
//                           int* mpu_data) {
// // TBI    
// }

// void SimpleNavigation::calcFusedAngles(int* mpu_data) {
// // TBI
// }

// void SimpleNavigation::calcWorldState(int* mpu_data) {
// // TBI
// }






