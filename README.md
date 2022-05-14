# Self-Balancing-Robot
## Description
This is a fuzzy logic based Self-Balancing robot project. We tried to balance the robot using a fuzzy controller.
The tilt angle is measured using the MPU6050 accelerometer and gyroscope sensor, it is coupled with a Kalman filter on the controller to estimate the tilt angle theta of the robot.
The mathematical model of the robot was derived and the unknown parameters of the DC motors where estimated using MATLAB.
#
![IMG_2561](https://user-images.githubusercontent.com/69092782/168445610-61b3493a-85fa-40f7-91e9-5bcaeddf0081.JPG)
#
## Circuit Design
![Picture1](https://user-images.githubusercontent.com/69092782/168446348-d126c6b9-a120-4417-bd64-f14053804b78.png)
## Tools Used
- MATLAB
- Arduino IDE
## Libraries Used
- [Embedded Fuzzy Logic Library (eFLL)](https://github.com/alvesoaj/eFLL)
- [Kalman filter](https://github.com/TKJElectronics/KalmanFilter)
- Other Arduino built-in Libraries(I2c, SPI)
## Authors and acknowledgment
This robot is built by **Ziad Saoud**, **Roy Sawaya**, and **Chadi Kallas**.

