# Water_Level_and_Flow_Rate_Monitoring_System
This arduino sketch uses the RS485 modbus communication with the A02/A20 ultrasonic sensor that measures the height of the water level flowing through the 6 gates when opened. and sends it to the android device and the processed data from android to the google sheets using WiFi.

The RS485-UART TTL converter is used to connect the Ultrasonic sensor with the controller. The RS485 must be connected to the same voltage as that of the microcontroller operating voltage to meet the logic level requirement of the converter and the  controller to communicate. 
