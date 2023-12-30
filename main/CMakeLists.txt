idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash 
                            driver 
                            esp_event 
                            esp_netif 
                            esp_wifi 
                            icm_20608_espidf_component 
                            micro_ros_espidf_component 
                            vl53l1x_esp_idf_component 
                            mat_inv
                            eigen)