# 003_Walking One Leg Flexion
Measure and train left or right hip, knee, and ankle flexion angles during walking.

### Nodes Required: 6 
- Sensing (4): 
  - foot (top, switch pointing forward), 
  - shank (lateral, switch pointing up), 
  - thigh (lateral, switch pointing up), 
  - pelvis (back center, switch pointing up) 
- Feedback (2): feedback_min, feedback_max

## Algorithm & Calibration
### Algorithm Information
The raw quaternions from the IMU are converted to Euler angles, and the roll angle is extracted using well established mathematical principles. If you'd like to learn more about quaternion to Euler angles calculations, we suggest starting with this Wikipedia page: [Conversion between quaternions and Euler angles](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)

### Calibration Process:
The angle calculated is the global roll angle for the IMU. This must be aligned with the segment. The app will take the starting position to determine yaw offset.

## Description of Data in Downloaded File
### Calculated Fields
- time (sec): time since trial start
- step_count: steps of walking
- Gait_Phase: gait phase of either left foot or the right foot as selected in the app configuration. 
  - 0 is “Early stance”; 
  - 1 is “Middle stance” ; 
  - 2 is “Late stance”; 
  - 3 is  “Swing” 
- feedback_max: feedback status for feedback_max node. 
  - 0 is “feedback off”
  - 1 is - “feedback on” 
- feedback_min: feedback status for feedback_min node. 
  - 0 is “feedback off”
  - 1 is “feedback on”

### Sensor Raw Data Values 
Please Note: Each of the columns listed below will be repeated for each sensor
- SensorIndex: index of raw sensor data
- AccelX/Y/Z (m/s^2): raw acceleration data
- GyroX/Y/Z (deg/s): raw gyroscope data
- MagX/Y/Z (μT): raw magnetometer data
- Quat1/2/3/4: quaternion data (Scaler first order)
- Sampletime: timestamp of the sensor value
- Package: package number of the sensor value

# Development and App Processing Loop
The best place to start with developing an or modifying an app, is the [SageMotion Documentation](http://docs.sagemotion.com/index.html) page.