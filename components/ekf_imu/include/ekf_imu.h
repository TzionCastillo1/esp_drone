#ifndef _ekf_imu_H_
#define _ekf_imu_H_

//#include "ekf.h"
#include <eigen3/Eigen/Dense>
#include <cmath>

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
class ekf_imu
{
    public:
        typedef Eigen::Matrix<float, 7, 7> Matrix7f;
        typedef Eigen::Matrix<float, 7,1> Vector7f;
        typedef Eigen::Matrix<float, 3, 7> Matrix3x7f;
        typedef Eigen::Matrix<float, 7, 3> Matrix7x3f;

        float TaitBryan[3];

        ekf_imu();
        ~ekf_imu();

        /**
         * @brief 
         * In this function we will initialize the filter with our inital expected state and
         * initial expected state error covariance matrix
         * @param accel0 
         * @param P0
         */
        void init(Vector7f state0_, Matrix7f P0, Matrix7f Q, Eigen::Matrix3f R, void* offsets);

        /* TODO: change this input vector to me more generic*/
        Eigen::Matrix<float, 4, 3> skew(Eigen::Vector4f q_);

        Eigen::Matrix4f skewOmega(Eigen::Vector4f w_);
        //Matrix7f setStateMatrix(Vector7f state_, float dt);

        Matrix3x7f setMeasurementMatrixJac(Eigen::Vector4f q_);

        void threeNorm(Eigen::Vector3f* vec3_);

        void qNorm(Vector7f* state_);

        // Method to run prediction subroutine
        /** 
         * In this method the: 
         * (1) State Matrix evaluation function will be called
         * (2) Next state will be predicted using the State Matrix from the previous step
         * (3) Next Estimate Error COvariance Matrix will be predicted using the State 
         * Matrix from the previous step 
         * (4) Measurement matrix Jacobian will be evaluated at the state prediction from step 2
        */
        void predict(float control[3], float dt);


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
        void update(float measurement[3]);

        //Initial reference values of accelerometer

        void quat2TaitBryan(float* taitBryanAngles);

        void printState();

    private:
    //Note an appended underscore indicates a vector (i.e x_ is a vector x)
    // TODO: Create a typedef for size 7 vector & size 7 square matrix or use the full instantiation everywhere these are needed.
        Vector7f state_;
        Eigen::Vector3f measurement_;
        Eigen::Vector3f control_;
        Matrix7f P;
        Matrix7f Q;
        Eigen::Matrix3f R; 
        float*  offsets;


};

#endif