#include <stdio.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "ICM20608.h"
#include "VL53L1X_api.h"

uint16_t vl53l1x = 0x29;
const char* TAG = "DEBUG";

void init_I2C()
{
    
    ESP_LOGI(TAG, "INITIALIZING I2C");
    i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num  = 10,         // select GPIO specific to your project
    .sda_pullup_en = GPIO_PULLUP_DISABLE,
    .scl_io_num = 11,         // select GPIO specific to your project
    .scl_pullup_en = GPIO_PULLUP_DISABLE,
    .master.clk_speed = 400000,  // select frequency specific to your project
    .clk_flags = 0,                          // you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));

    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

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
    while(state == 0){
        Status = VL53L1X_BootState(vl53l1x, &state);
        ESP_LOGI(TAG, "Status: %u", state);
        vTaskDelay(20/portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "0");

    VL53L1X_GetSensorId(vl53l1x, &SensorID);
    ESP_LOG_BUFFER_HEX(TAG, &SensorID, 2);

    /* Sensor Initialization */
    Status = VL53L1X_SensorInit(vl53l1x);
    ESP_LOGI(TAG, "1");

        /* Modify the default configuration */
    Status = VL53L1X_SetInterMeasurementInMs(vl53l1x, 100);
        ESP_LOGI(TAG, "2");

    //Status = VL53L1X_SetOffset();
    /* enable the ranging*/
    Status = VL53L1X_StartRanging(vl53l1x);
        ESP_LOGI(TAG, "3");

    /* ranging loop */
    while(1){
        while(dataReady == 0)
        {
            Status = VL53L1X_CheckForDataReady(vl53l1x, &dataReady);
        }
            ESP_LOGI(TAG, "4");

        dataReady = 0;
        Status = VL53L1X_GetRangeStatus(vl53l1x, &RangeStatus);
        Status = VL53L1X_GetDistance(vl53l1x, &Distance);
        Status = VL53L1X_ClearInterrupt(vl53l1x);
        ESP_LOGI("TOF TEST:" , "Distance (mm): %d", Distance);
        ESP_LOGI("TOF TEST:" , "Range Status: %u", RangeStatus);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

}