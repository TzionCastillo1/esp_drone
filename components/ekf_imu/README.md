**This library controls the initialization and execution of an extended kalman filter for use with a 6 DOF IMU (3 DOF Accelerometer, 3 DOF Gyroscope).

The library works in the following steps:

1 - An initial estimate, and estimation error, proccess noise, and sensor noise covariance matrices are given to the library.
2 - When a gyro reading is received, a prediction of the state at the next time step is formulated using the state equation. The prediction for the estimation error covariance is also evaluated using the jacobian of the state equation.
3 - When an accelerometer reading is received, a set of 'expected' readings is calculated using the state prediction from the previous step and the measurement equation. The kalman gain is calculated using the predicted error covariance and the jacobian of the measurement equation. The state quaternion is then normalized

How can this be built so that it can be fit easily into multiple types of overarching routines (RTOS, superloop, etc.)? 

EKF_base Class:
    private variables:
        estimate error covariance P
        process noise covariance Q
        sensor noise covariance R
        state vector x
        measurement vector y
    public variables:
        none?
    functions:
        initialize -> initialized with a struct pointer that points to a struct containing P0,x0, Q, R.
        predict -> supply 'control' inputs to filter & predict as described above.
        estimate -> supply measurement readings and perform estimation as described in step 3.
        get_orientation -> provide Euler angle representation of rotation.

EKF_utils Class:
