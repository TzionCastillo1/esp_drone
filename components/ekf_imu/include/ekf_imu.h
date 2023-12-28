#ifndef _ekf_imu_H_
#define _ekf_imu_H_

//#include "ekf.h"
#include "Eigen/Dense"

/**
 * @brief This class is used to process and calculate attitude from IMU sensors.
 * 
 *  This class uses a state vector with the following 4 values:
 *  X[0...3] - attitude quaternion
 * 
 *  It is deisnged to be used with a 6 axis IMU.
 *  As a result, there will be significant yaw drift 
 * 
 */
class ekf_imu:
{
    public:
    ekf_imu::ekf_imu();
    ~ekf_imu();

    /**
     * @brief 
     * In this function we will initialize the filter with our inital expected state and
     * initial expected state error covariance matrix
     * @param accel0 
     * @param P0
     */
    void Init(Eigen::Vector7f state0_, Eigen::Matrix7f P0, Eigen::Matrix7f Q, Eigen::Matrix3f R);

    Eigen::Matrix<float, 3, 7> skew(Eigen::Vector7f state_);

    Eigen::Matrix7f setStateMatrix(Eigen::Vector7f state_);

    Eigen::Matrix<float, 3, 7> setMeasurementMatrixJac(Eigen::Vector4f q_);

    // Method to run prediction subroutine
    /** 
     * In this method the: 
     * (1) State Matrix evaluation function will be called
     * (2) Next state will be predicted using the State Matrix from the previous step
     * (3) Next Estimate Error COvariance Matrix will be predicted using the State 
     * Matrix from the previous step 
     * (4) Measurement matrix Jacobian will be evaluated at the state prediction from step 2
    */
    void predict();


    // Method to run update subroutine
    /**
     * In this method the: 
     * (1) Kalman gain matrix will be calculated using the most recent Estimate Error 
     * Covariance matrix & the most recent measurement matrix Jacobian 
     * (2) The updated state estimation will be calculated using the previous state 
     * prediction, kalman gain matrix, measurement vector & the taylor expansion of the
     * measurement matrix using the most recent evaluated measurement matrix Jacobian
     * 
     */
    void update();

    //Initial reference values of accelerometer


    private:
    //Note an appended underscore indicates a vector (i.e x_ is a vector x)
    Eigen::Vector7f state_;
    Eigen::Vector3f measurement_;
    Eigen::vector3f control_;
    Eigen::Matrix7f P;
    Eigen::Matrix7f Q;
    Eigen::Matrix3f R; 


};