# Trajectory-Estimation

**Description**: In this project, you are provided with measured IMU data,
GPS data and ground truth trajectory data. Please apply appropriate
algorithms to estimate trajectory based on IMU data alone, GPS data alone,
IMU and GPS data together and then evaluate your algorithms by
comparing your results with the ground truth qualitatively(e.g. plot
trajectories) and quantitatively(e.g. RMSE, etc).
This dataset is just a simple simulation of a ship sailing on the sea, whose
trajectory is basically just a straight line, so you won’t see dramatic
change in the data and fancy turns in the trajectory.
The provided measured **IMU data** includes:
 data from 3D gyro ( unit: radius per second)
 data from 3D accelerometer( unit: meter per squared second)
o acceleration in z direction includes gravity

all are measured in the robot’s local coordinate system
The provided measured **GPS data** includes:
 position information ( longitude, altitude and height)
 velocity information ( velocity in x and y and z direction)
 all are measured in Earth coordinate system
The provided ground truth data includes:

**3D location data**: longitude, latitude and height.
Multi-sensor data fusion algorithms include but not limited to:
 Kalman Filter(KF);
 Extended Kalman Filter(EKF);
 Unscented Kalman Filter(UKF);
 Cubage Kalman Filter(CKF)
 H-infinity Filter
 Particle Filter
 Neural Networks
 ......
The work you may need to do includes but not limited to:

use initial orientation and initial position information to build
**transformation matrix**.

o initial orientation: roll, pitch, yaw are all 0;
o initial position: [0.1609; 4.2062; 0]
o initial position:[0; 0; 0]
transform IMU data
in local frame to global frame with the
transformation matrix, and then eliminate gravity from z direction.

transform from global frame to the Earth coordinate system with
Earth parameters:

o Earth radius:Re=6378137.0
o Earth elliptic rate:f=1/298.257
o Earth's angular velocity(地球自转角速度):Wie=7.29211506e-5
o gravity acceleration:g=[0; 0; -9.7803698]
fuse information from IMU and GPS ( in the same coordinate
system) with filters, get estimated trajectory.

compare and analysis your experimental results qualitatively(e.g.
plot trajectories) and quantitatively(e.g. RMSE, etc).
