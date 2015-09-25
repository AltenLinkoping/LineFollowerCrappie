// Copyright Alten AB, Linköping 2015
//
// Authors: Sören Molander, Joel Nordh, Pierre Östlund
//

#ifndef __NAVIGATION__
#define __NAVIGATION__

#include <math.h>
#include <vector>
#include <stdio.h>

// #include <Eigen/Core>
// #include <Eigen/Geometry>

// Raw data in body coords
// Need details of registers of MPU chip to set sensitivity
struct NavigationData {
    double tau; 
    double Ts;
    int ACC_SENSITIVITY;
    int GYRO_SENSITIVITY;
    std::vector<double> angle; // yaw, pitch, roll
    std::vector<double> rate; // yaw, pitch, roll
    std::vector<double> acc;
    std::vector<double> vel;
    std::vector<double> pos;
    std::vector<double> acc_bias;
    std::vector<double> rate_bias;
    NavigationData() :
        angle(3,0),
        acc(3,0),
        vel(3,0),
        pos(3,0),
        Ts(0.005), // From MPU 6050 spec
        acc_bias(3,0.01),
        rate_bias(3,0.1) {
        tau = 0.075; // respone time
        ACC_SENSITIVITY = 8192; // +/- 4g
        GYRO_SENSITIVITY = 65535; // 500 deg/s 

    }
};

// raw vector from MPU 6050 chip (mpu_data):
// mpu_data[0] = accx
// mpu_data[1] = accy
// mpu_data[2] = accz
// mpu_data[3] = temp
// mpu_data[4] = pitch
// mpu_data[5] = roll
// mpu_data[6] = yaw

// Navigation interface
class INavigation {
public:
    INavigation() {}
    virtual void calcFusedAngles(int* mpu_data) = 0;
    virtual void calcWorldState(int* mpu_data) = 0;
    virtual NavigationData getNavData() const = 0;
};


// Simplest possible filter: complementary fusion filter, use acc and rates to get angles
// assumes MPU chip is aligned along the coordinate axes, z-axis aligned along gravity vector
class SimpleNavigation: public INavigation {
private:
    double _Ts;
    void _toPhysicalAcc(int* mpu_data);
    void _toPhysicalRates(int* mpu_data);
    NavigationData _navDataBodyMixed; // Mixed angle solution, persistent data for filtering
    NavigationData _navDataBodyRaw; // Mixed angle solution, persistent data for filtering
    NavigationData _navDataBodyRawFromAcc; // Mixed angle solution, persistent data for filtering
    NavigationData _worldData; // world data solution
    double _filterCoeff; // for the mixing complementary filter
    void _calcRawAngles(int* mpu_data); // accx, accy, accz, rate_x, rate_y, rate_z;
    void _calcMixedAngles(int* mpu_data); // accx, accy, accz, rate_x, rate_y, rate_z;
    void _calcAccAngles(int* mpu_data);
    int indPitch, indYaw, indRoll; // indices in mpu_data
    int indAccx, indAccy, indAccz; // indices in mpu_data
public:
    SimpleNavigation();
    void calcFusedAngles(int* mpu_data);
    void calcWorldState(int* mpu_data);
    NavigationData getNavData() const {return _navDataBodyMixed;}
};


// typedef Eigen::Matrix<double,9,
// typedef Eigen::Matrix<double,6,1> State;
// typedef Eigen::Matrix<double,6,6> Dynamics;
// typedef Eigen::Matrix<double,3,3> RotationMatrix;
// typedef Eigen::Matrix<double,4,3> MeasurentMatrix;
// typedef Eigen::Matrix<double,6,6> CovarianceMatrix;
// typedef Eigen::Matrix<double,6,6> ProcessNoiseMatrix;
// typedef Eigen::Matrix<double,6,6> MesurementNoiseMatrix;

// class KalmanNavigation: public INavigation{
// private:
//     double _Ts;
//     RotationState q;
//     QuaternionState S;
//     QuaternionStatePrime Sp;
//     Dynamics F; 
//     state X;
//     MeasurementMatrix H;
//     RotationMatrix Rot;
//     processNoiseVector n;
//     CovarianceMatrix P;
//     ProcessNoiseMatrix Q;
//     MeasurementNoiseMatrix R;
// public:
//     NavigationData navData;
//     Eigen::Matrix
//     Eigen::Matrix3f Rot;
//     Eigen::Quaternion q;
//     TranslationState stateX;
//     RotationState stateQ;
//     KalmanNavigation(double Ts);
//     void calcStateAngles(int* mpu_data);
//     void calcWorldState(int* mpu_data);

// };



#endif
