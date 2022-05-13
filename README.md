# GPS_Tracking_System

The current repository represents my Capstone project developed to university in order to accomplish a Automotive Tracking System for a car fleet used by the local police in Espírito Santo, Brazil. 

The explanation on the files are as follows:
- GPSKalmanFilter: This is the developed library for this project. It can be embedded in many devices used in Arduino enviroment. Later it is interesting to adapt it for several diferent MCU architecture. The link to the repository itself is: https://github.com/gabrielbklopes/GPSKalmanFilter
- HMC5883L is the library to access the magnetometer. Some changes were applied.
- Haversine is the library that calculates the distance between two points in the globe and the bearing angle.
- MPU6050 access the sensor that incorporates a gyroscope and a accelerometer. Several changes had to be applied in order to math the current project.
- arduino-lmic-master is responsible for the LoRa communication. Be aware this is a highly complicated library and can be trick to understand and work with. Also it was not exactly made for ESP32 MCU. Good luck in case you need to apply changes inside.
- GPSFilter.ino is the program itself. It covers the LoRa communication, the gps aquisition, the sensors first calibration and applies the Madgwick fuction to calculate the system attitude. In the future this function can migrate to a library or even to the Kalman Filter library.
- KFSimulationGPS.ipynb is a Google Colab notebook that covers the Kalman Filter applied in datasets obtained from the embedded system data.
- PathSimulationGPS.ipynb simulates the car path using trigonometric functions for the position, velocity and acceleration. This is a good kickoff point to start understanding the functioning of the system itself.
- calibration_mag.ino is used to calibrate the magnetometer against Hard and Soft Iron interference.

In case of futher questions about this project fill free to reach me out on LinkedIn: https://www.linkedin.com/in/gabrielbklopes/

Notice that this project doesn't work perfectly and I stoped it when the system presented a ok behavior. Many improviments are necessary but this is already a good start.

Gabriel Lopes!
