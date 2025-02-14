# Drone-Pose-Estimation

<img src='jg.gif'>

### Problem Statement
Given an onboard downward-facing camera, a mat of AprilTags, and an IMU (Inertial Measurement Unit), the task is to compute the pose and velocity of the drone.

### Methodology

1. **AprilTag Detection and Position Calculation**  
   - Detect AprilTags within the camera’s downward-facing view.
   - associate the detected Tag with the position in the world coordinate system.

2. **Pose Estimation (Position and Orientation)**  
   - Estimate the drone’s pose (position and orientation) using the homography matrix.
   - Apply Singular Value Decomposition (SVD) to obtain the best estimate.

3. **Feature Detection and Motion Tracking**  
   - Use FAST (Features from Accelerated Segment Test) for feature detection.
   - Track the motion of these features using the KLT (Kanade-Lucas-Tomasi) optical flow algorithm.

4. **Velocity Estimation**  
   - Formulate the motion field equation to estimate the drone’s velocity in the camera frame.

5. **Outlier Rejection with RANSAC**  
   - Use the RANSAC (Random Sample Consensus) algorithm to reject outliers from the data.
   - Refine the estimate of the drone’s velocity for more accurate results.

6. **Fusion with IMU Data Using UKF**  
   - Use the Unscented Kalman Filter (UKF) to fuse the velocity and pose estimates from the camera with data from the IMU for better overall accuracy.

This process integrates both vision-based methods and IMU readings to achieve robust pose and velocity estimation for the drone in a dynamic environment.

<img src='part2_data2.png'>
<img src='part2_data1.png'>
<img src='part1_data_1.png'>
<img src='part1_data_2.png'>

### Tools
MATLAB




3. Transformation matrix equation:
   
$$\begin{bmatrix}
c_p W \\
c_\omega W
\end{bmatrix}
=$$ $$\begin{bmatrix}
R_C^B & -R_B^C S(r_{BC}^B) \\
0 & R_B^C
\end{bmatrix}
\begin{bmatrix}
B \omega_B^W \\
B \omega_B^W
\end{bmatrix}$$
 


