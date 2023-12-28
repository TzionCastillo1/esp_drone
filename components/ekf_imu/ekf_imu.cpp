#include "ekf_imu.h"
//#include "Eigen/Dense"


ekf_imu::ekf_imu()
{
}

ekf_imu::~ekf_imu()
{
}

void Init(Eigen::Vector7f state0_, Eigen::Matrix7f P0, Eigen::Matrix7f Q, Eigen::Matrix3f R)
{
    //the initial estimate for state needs to be in the form of a quaternion
    // aligned with the NED world frame. Does this just mean that I will start with 
    // a quaternion with "no rotation", and assume this points to no rotation in the 
    // NED world frame?
    this->state_ = state0_;
    // Init estimation error covariance
    this->P = P0;
    // Init process covariance
    this->Q = Q;
    // Init measurement covariance
    this->R = R;
}

Eigen::Matrix<float, 4, 3> skew (Eigen::Vector7f q_){
    Eigen::Matrix<float, 4, 3> skew << -q_(1,0), -q_(2,0), -q_(3,0),
                                        q_(0,0), -q_(3,0), -q_(2,0),
                                        q_(3,0), q_(0,0), -q_(1,0),
                                        -q_(2,0), q_(1,0), q_(0,0);
    return skew;
}

Eigen::Matrix7f setStateMatrix(Eigen::Vector7f state_, float dt){
    Eigen::Matrix7f F = Eigen::Matrix7f::Identity();
    //Eigen::Matrix<float, 4, 3> Sq =
    F.block<4,3>(0,4) = (-dt/2) * (skew(state_.block<4,0>(0,0)));

}

Eigen::Matrix<float, 3, 7> setMeasurementMatrixJac(Eigen::Vector7f state_){}

void predict(float* control, float dt){
    //Need to read in the gyro readings for the 'control' vector
    this->control_ = Map<Vector3f>(control);/*array to vector*/
    //Need to decide where dt is stored & how it is measured
    //F = setStateMatrix(this->state_, dt);
    Eigen::Matrix7f F = Eigen::Matrix7f::Identity();
    Eigen::Matrix<float, 4, 3> Sq = (dt/2) * skew(state_.block<4,0>(0,0));
    F.block<4,3>(0,4) = -Sq;
    this->P = (F * P * F.transpose() + this->Q);
    Eigen::Matrix<float, 7,3> G = Eigen::Matrix<float, 7, 3>::Zero();
    G.block<4,3>(0,0) = Sq;
    predictedState_ = F*(this->state_) + G*this->control_;
}

/*
dsps::Mat ekf_imu7states::StateXdot(dspm::Mat &x, float *u)
{
    //omega represented by w for brevity
    float wx = u[0] - x(4,0);
    float wy = u[1] - x(5,0);
    float wz = u[2] - x(6,0);

    float w[] = {wx, wy, wz};
    dspm::Mat q = dspm::Mat(x.data, 4, 1);

    // qdot = Q * omega
    dspm::Mat Omega = 0.5 * SkewSym4x4(x.data, 4, 1);
    dspm::Mat qdot = Omega * q;
    dspm::Mat Xdot(this->NUMX, 1);
    Xdot *= 0;
    Xdot.Copy(qdot, 0, 0);
    
    return Xdot;
}

void ekf_imu7states::LinearizeFG(dspm::Mat &x, float *u)
{
    // subtract gyro biases
    float w[3] = {(u[0] - x(4,0)), (u[1] - x(5,0)), (u[2] - x(6,0))};

    this->F *= 0;
    this->G *= 0;


    /**
     * {0, -w[1], -w[2], -w[3];
     *  w[1],  0,  w[3], -w[2];
     *  w[2], -w[3],  0,  w[1];
     *  w[3], w[2], -w[1], 0  ;} 
     * 
     /
    F.Copy(0.5*ekf::SkewSym4x4(w), 0, 0);

    /**
     * Only performs q product with the first 4 elements of x
    /

    dspm::Mat dq = -0.5 * qProduct(x.data);
    //Removing first column since w[0] = 0;
    dspm::Mat dq_q = dq.Get(0,4,1,3);

    G.Copy(dq_q, 0, 0);

    F.Copy(dq_q, 0, 4);
}
*/