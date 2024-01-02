#include <stdio.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_dsp.h"
#include "dsp_platform.h"
#include "driver/timer.h"

#include "mat_inv.h"
#include "ICM20608.h"
#include "VL53L1X_api.h"
#include "ekf_imu.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <std_msgs/msg/header.h>
#include <diagnostic_msgs/msg/diagnostic_status.h>
#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

extern "C" void app_main();

uint16_t vl53l1x = 0x29;
const char* TAG = "DEBUG";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define TIMER_DIVIDER (16) // Hardware timer clock dividier
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // convert counter value to seconds

#define GYRO_SCALER (.0153)
#define ACC_SCALER (19.6/32767)

#define KALMAN_PERIOD 100 // Period in milliseconds

uint8_t device_id = 1;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
//geometry_msgs__msg__TwistStamped send_twist;

std_msgs__msg__Header header;
diagnostic_msgs__msg__DiagnosticStatus diagnosticstatus;
geometry_msgs__msg__TwistStamped recv_twist;
std_msgs__msg__Int32 msg;

typedef struct {
    int16_t* motion;
    ekf_imu* EKF_IMU;
} ekf_obj;


volatile bool sensorReady = false;

//class EKF {
//public:
//    
//} 

void init_I2C()
{
    
    ESP_LOGI(TAG, "INITIALIZING I2C");
    i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num  = 10,         // select GPIO specific to your project
    .scl_io_num = 11,         // select GPIO specific to your project
    .sda_pullup_en = GPIO_PULLUP_DISABLE,
    .scl_pullup_en = GPIO_PULLUP_DISABLE,
    //.master.clk_speed = 400000,  // select frequency specific to your project
    .clk_flags = 0,                         // you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here
    };
    conf.master.clk_speed = 400000;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));

    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

void diagnostic_timer_callback(rcl_timer_t * timer, int64_t last_call_time){
    RCLC_UNUSED(last_call_time);
    ESP_LOGI(TAG, "pub call back");

    if (timer != NULL) {
        diagnostic_msgs__msg__DiagnosticStatus__init(&diagnosticstatus);
        diagnosticstatus.level = 0;
        sprintf(diagnosticstatus.name.data, "test");
        diagnosticstatus.name.size = strlen(diagnosticstatus.name.data);
        sprintf(diagnosticstatus.message.data, "testmsg");
        diagnosticstatus.message.size = strlen(diagnosticstatus.message.data);
        sprintf(diagnosticstatus.hardware_id.data, "%d", device_id);
        diagnosticstatus.hardware_id.size = strlen(diagnosticstatus.hardware_id.data);
        RCSOFTCHECK(rcl_publish(&publisher, (const void *) &diagnosticstatus, NULL));
    }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
    RCLC_UNUSED(last_call_time);
    if(timer != NULL){
        printf("Publishing: %d\n", (int) msg.data);
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
        msg.data++;
    }
}

void twist_subscription_callback(const void * msgin){
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;
    /** TODO: Feed twist to motor controller 
     *  Only paying attention to linear components and angular.z
     *  TODO: Determine a forwards direction based on direction of IMU
    */
   /*
    msg->linear.x;
    msg->linear.y;
    msg->linear.z;
    msg->angular.x;
    msg->angular.y;
    msg->angular.z;
    */
}

//previous estimate
//float q[4];
//prediction for next state
//float q_bar[4];

/*
void add_mat(float* A, float* B, float* C){

    uint8_t size_row = sizeof(A)/sizeof(A[0]);
    uint8_t size_col = sizeof(A[0])/sizeof(A[0][0]);
    
    for(int i = 0; i < size_col ; i++){
        for(int j = 0; j < size_row){
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}
*/
/*
void ekf_predict(float omega[3], float dt){
    float q_temp[4]

    const float A[4][7] = {{1,1,1,1, dt/2 * q[1], -dt/2 * q[2], dt/2 * q[3]},
                    {1,1,1,1, dt/2 * -[0], dt/2 * q[3], dt/2 * -q[2]},
                    {1,1,1,1, dt/2 * -q[3], dt/2 * -q[0], dt/2 * q[1]},
                    {1,1,1,1, dt/2 * q[2], dt/2 * -q[1], dt/2 * -q[0]}};

    const float B[4][4] = {{dt/2 * -q[1], dt/2 * -q[2], dt/2 * -q[3]},
                    {dt/2 * q[0], dt/2 * -q[3], dt/2 * q[2]},
                    {dt/2 * q[3], dt/2 * q[0], dt/2 * -q[1]},
                    {dt/2 * -q[2], dt/2 * q[1], dt/2 * q[0]}};

    dspm_mult_f32_ae32(&A, q, q_temp, 4,7,1);
    dspm_mult_f32_ae32(B,omega, q_bar, 4,3,1);

    add_mat(q_temp, q_bar, q_bar);


}
*/
//class KalmanFilter {
//    public:
//        G_m = dspm::Mat::eye(12);
//};

void estimate_control_task(void * arg)
{
    /** TODO: Create PID Loops for x, y, & z velocities & yaw angular velocity
     *        These loops can wrap loops for angular position control (r,p)
     *        These loops can be ran independantly and sum to create the true motor output
     *        Maybe need to check for control saturation?
     * 
    */

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(KALMAN_PERIOD);
    BaseType_t xWasDelayed;
    xLastWakeTime = xTaskGetTickCount();
    ekf_obj* EKF_Obj = (ekf_obj*) arg;
    int16_t* motion = (int16_t*) EKF_Obj->motion;
    //EKF_IMU is the pointer that refers to the ekf object
    ekf_imu* EKF_IMU = (ekf_imu*) EKF_Obj->EKF_IMU;
    //Convert gyro from 500 d/s scale to actual units (rad/s?)
    //Convert accel from 2g scal eto actual units (m/s/s)
    float gyro[3];
    float accel[3];

    while(1){
        xWasDelayed = xTaskDelayUntil(&xLastWakeTime, xFrequency);
        for(int i = 0; i < 3; i++){
            gyro[i] = static_cast<float>(motion[3+i]) * GYRO_SCALER;
            //gygy[i] = motion[3+i] * GYRO_SCALER;
            accel[i] = static_cast<float>(motion[i]) * ACC_SCALER;
            //std::cout<< "gyro:" << gyro[3 + i] << std::endl;
        }
        EKF_IMU->predict(gyro, (float) (KALMAN_PERIOD*.001));
        //EKF_IMU->update();
        EKF_IMU->printState();

    }


}



void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// Create init_options.
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif
	// Setup support structure.
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// Create node.
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "drone_thing_pub_sub", "", &support));

	// Create publisher.

	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),/*ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus),*/
		"test_int_publisher"));

	// Create subscriber.
	/*RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
		"twiststamped_subscriber"));
    */
	// Create timer.
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		diagnostic_timer_callback));

	// Create executor.
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	// Add timer and subscriber to executor.
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	//RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_twist, &twist_subscription_callback, ON_NEW_DATA));

	// Spin forever.
	//send_twist.
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100000);
	}

	// Free resources.
	//RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

static bool IRAM_ATTR sensor_timer_cb(void* args)
{
    BaseType_t high_task_awoken = pdFALSE;
    sensorReady = true;
    return high_task_awoken == pdFALSE;   
}

static void sensor_timer_init(timer_group_t group, timer_idx_t timer, timer_autoreload_t auto_reload, uint8_t timer_interval_sec)
{
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = auto_reload,
        .divider = TIMER_DIVIDER,
    }; // default clock source is APB

    timer_init(group, timer, &config);
    // sets timer counter's initial value to below. It will also be set to this value
    // on auto reload on alarm
    timer_set_counter_value(group, timer, 0);
    timer_set_alarm_value(group, timer, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(group, timer);
    timer_isr_callback_add(group, timer, sensor_timer_cb, NULL, 0);

    timer_start(group, timer);
}

//static bool read_sensor_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata, void* )

void app_main(void)
{
    ESP_LOGI(TAG, "Beginning");
    init_I2C();
    VL53L1X_ERROR Status = 0;
    /*Platform Initialization code here */
        /* Wait for device booted */
    uint8_t state = 0;
    uint8_t dataReady = 0;
    uint8_t RangeStatus;
    uint16_t Distance;
    uint16_t SensorID;
    char str[80];
    int16_t motion[6];
    while(state == 0){
        Status = VL53L1X_BootState(vl53l1x, &state);
        ESP_LOGI(TAG, "Status: %u", state);
        vTaskDelay(20/portTICK_PERIOD_MS);
    }

    VL53L1X_GetSensorId(vl53l1x, &SensorID);
    ESP_LOG_BUFFER_HEX(TAG, &SensorID, 2);

    /* Sensor Initialization */
    Status = VL53L1X_SensorInit(vl53l1x);

        /* Modify the default configuration */
    Status = VL53L1X_SetInterMeasurementInMs(vl53l1x, 100);

    //Status = VL53L1X_SetOffset();
    /* enable the ranging*/
    Status = VL53L1X_StartRanging(vl53l1x);
    
    ICM_20608 imu;

    ICM20608_init(&imu, I2C_NUM_0, ICM20608_DEFAULT_ADDR, 10000);
    //int16_t ax, ay, az;
    //int16_t gx, gy, gz;
    //ICM20608_getMotion(&imu, &ax, &ay, &az, &gx, &gy, &gz);

    //ESP_LOGI(TAG, "ICM20608:");
    //ESP_LOG_BUFFER_HEX(TAG, &ax, 2);

    /* Create timer for periodically polling sensors */
    sensor_timer_init(TIMER_GROUP_0, TIMER_0, TIMER_AUTORELOAD_EN, 1);
    /*gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    */

    ekf_imu::Matrix7f Q = ekf_imu::Matrix7f::Identity();
    Q *= .1;

    ekf_imu::Matrix7f P0 = ekf_imu::Matrix7f::Identity();
    P0 *= .1;

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    R *= .1;

    ekf_imu::Vector7f state0_ = {1,0,0,0,0,0,0};

    ekf_imu EKF;
    ekf_obj EKF_Obj;
    EKF_Obj.motion = (int16_t*) &motion;
    EKF_Obj.EKF_IMU = &EKF;

    EKF.init(state0_, P0, Q, R);

    ESP_ERROR_CHECK(uros_network_interface_initialize());
    /* ranging loop */
    /*xTaskCreate(micro_ros_task,
            "uros_task",
            16000,
            NULL,
            5,
            NULL);
    */
    xTaskCreate(estimate_control_task,
            "estimate_control_task",
            64000,
            (void*) &EKF_Obj,
            4,
            NULL);


    while(1){
        /*while(dataReady == 0)
        {
            Status = VL53L1X_CheckForDataReady(vl53l1x, &dataReady);
        }
        */
        if (sensorReady == true)
        {
            //dataReady = 0;
            Status = VL53L1X_GetRangeStatus(vl53l1x, &RangeStatus);
            Status = VL53L1X_GetDistance(vl53l1x, &Distance);
            Status = VL53L1X_ClearInterrupt(vl53l1x);
           /* ESP_LOGI("TOF TEST:" , "Distance (mm): %d", Distance);
            
            sprintf(str, "%d", Distance);
            std::cout << str;
            ESP_LOGI("TOF TEST:" , "Range Status: %u", RangeStatus);
            */
            ICM20608_getMotion(&imu, motion);               
            /*std::cout << "Gyro motion:";
            for(auto x:motion){
                std::cout << x << std::endl;
            }
            */
            //ESP_LOGI(TAG, "IMU vals: %d", Imu_vals.ax);
        }
        //vTaskDelay(1000/portTICK_PERIOD_MS);
    }

}