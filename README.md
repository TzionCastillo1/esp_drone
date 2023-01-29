# ESP_DRONE

ESP_drone is intended as a low-cost, small form-factor, micro-ros enabled quadcopter. It is particularily aimed towards prototyping multi-agent systems for education purposes.

## How it works
The ESP_drone features an ESP32s3 as it's name would suggest. This enables very easy integration into the existing ROS2 ecosystem, thanks to the work done by the micro-ros team. Currently, microros is enabled only through UDP, but could be enabled through UART with some work for debugging purposes.

The ESP_drone has an InvenSense 20608-g 6-axis IMU, and a STMicroelectronics vl503lx for pose estimation. The board is designed to use 4x 8520 Brushed DC motor, as well as DJI Tello propellors.

# IMPORTANT NOTE:
DO NOT PLUG IN THE USB AND BATTERY AT THE SAME TIME. THERE IS A FLAW IN THE DESIGN THAT COULD ALLOW FOR A SMALL AMOUNT OF CURRENT TO FLOW INTO THE BATTERY

## How to use
TODO: I'm not really sure atm, this will have to be updated
TODO: CHANGE PROJECT NAME

