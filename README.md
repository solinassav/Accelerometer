# Accelerometer
It takes the raw data from mpu6050, sends it to the PC via the serial port, applies the Kalman filter and the complementary filter. 
Or it takes the data processed by the internal dsp to the sensor (the mpu is a microcontroller!). The data sent in the second method are good but will need further processing on the PC.
