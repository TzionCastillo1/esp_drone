#include "ekf_imu.h"
//#include "Eigen/Dense"


ekf_imu::ekf_imu()
{
}

ekf_imu::~ekf_imu()
{
}

void ekf_imu::Init(Vector7f state0_, Matrix7f P0, Matrix7f Q, Eigen::Matrix3f R)
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

Eigen::Matrix<float, 4, 3> ekf_imu::skew(Eigen::Vector4f q_){
    Eigen::Matrix<float, 4, 3> S_q;
    S_q <<  -q_(1,0), -q_(2,0), -q_(3,0),
            q_(0,0), -q_(3,0), -q_(2,0),
            q_(3,0), q_(0,0), -q_(1,0),
            -q_(2,0), q_(1,0), q_(0,0);
    return S_q;
}

/*
ekf_imu::Matrix7f ekf_imu::setStateMatrix(Vector7f state_, float dt){
    ekf_imu::Matrix7f F = Matrix7f::Identity();
    //Eigen::Matrix<float, 4, 3> Sq =
    F.block<4,3>(0,4) = (-dt/2) * (skew(state_.block<4,0>(0,0)));
    return F;
}
*/

ekf_imu::Matrix3x7f ekf_imu::setMeasurementMatrixJac(Eigen::Vector4f q_){
    //Needs to be updated
    return Matrix3x7f::Zero();
}

// It may be worth it in the future to change this function to not take state_ as an input
// since it is a member function and can access the state_ variable directly. Maybe not, because we
// may want to use this function with other quaternions found in the control loop for example
ekf_imu::Vector7f qNorm(ekf_imu::Vector7f state_){
    // A unit quaternion is just a quaternion where each element is divided by 
    // the length of the whole quaternion vector. For our rotation quaternion to remain a purely
    // rotational quaternion, it must remain a unit quaternion
    float q_length = 0;
    for(int i = 0; i < 4; i++){
        q_length+= state_(i,0) * state_(i,0);
    }
    q_length = std::sqrt(q_length);
    state_.block<4,1>(0,0) = state_.block<4,1>(0,0) / q_length;
    return state_;
}


void ekf_imu::predict(float* control, float dt){
    //Need to read in the gyro readings for the 'control' vector
    this->control_ = Eigen::Map<Eigen::Vector3f>(control);/*array to vector*/
    //Need to decide where dt is stored & how it is measured
    //F = setStateMatrix(this->state_, dt);
    Matrix7f F = Matrix7f::Identity();
    Eigen::Matrix<float, 4, 3> S_q = (dt/2) * skew(state_.block<4,1>(0,0));
    F.block<4,3>(0,4) = -S_q;
    this->P = (F * P * F.transpose() + this->Q);
    Matrix7x3f G = Matrix7x3f::Zero();
    G.block<4,3>(0,0) = S_q;
    this->state_ = F*this->state_ + G*this->control_;
    this->state_ = qNorm(this->state_);

}

// A similar note as for the qNorm function applies to this one
void quat2TaitBryan(ekf_imu::Vector7f state_, float* taitBryanAngles){
    // Calculating Roll
    float sinr_cosp = 2 * (state_(0,0) * state_(1,0) + state_(2,0) * state_(3,0));
    float cosr_cosp = 1 - 2 * (state_(1,0) * state_(1,0) + state_(2,0) * state_(2,0));
    taitBryanAngles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // Calculating Pitch
    float sinp = std::sqrt(1 + 2 * (state_(0,0) * state_(2,0) - state_(1,0) * state_(3,0)));
    float cosp = std::sqrt(1 - 2 * (state_(0,0) * state_(2,0) - state_(1,0) * state_(3,0)));
    taitBryanAngles[1] = std::atan2(sinp, cosp) - M_PI / 2;

    // Calculting Yaw
    float siny_cosp = 2 * (state_(0,0) * state_(3,0) + state_(1,0) * state_(2,0));
    float cosy_cosp = 1 - 2 * (state_(2,0) * state_(2,0) + state_(3,0) * state_(3,0));
    taitBryanAngles[2] = std::atan2(siny_cosp, cosy_cosp); 
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



     * {0, -w[1], -w[2], -w[3];
     *  w[1],  0,  w[3], -w[2];
     *  w[2], -w[3],  0,  w[1];
     *  w[3], w[2], -w[1], 0  ;} 
     * 
     /
    F.Copy(0.5*ekf::SkewSym4x4(w), 0, 0);

     * Only performs q product with the first 4 elements of x
    /

    dspm::Mat dq = -0.5 * qProduct(x.data);
    //Removing first column since w[0] = 0;
    dspm::Mat dq_q = dq.Get(0,4,1,3);

    G.Copy(dq_q, 0, 0);

    F.Copy(dq_q, 0, 4);
}
*/