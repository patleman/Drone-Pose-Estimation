# Drone-Pose-Estimation

<img src='final_dp.gif'>

### Problem Statement
Given an onboard downward-facing camera, a mat of AprilTags, and an IMU (Inertial Measurement Unit), the task is to compute the pose and velocity of the drone.

### Methodology

   <img src='state.png'>
   
1. **AprilTag Detection and Position Calculation**  
   - Detect AprilTags within the camera’s downward-facing view.
   - associate the detected Tag with the position in the world coordinate system.

2. **Pose Estimation (Position and Orientation)**  
   - Estimate the drone’s pose (position and orientation) using the homography matrix.
     <img src='Part1_eq1.png'>
     <img src='Part1_eq2.png'>
     <img src='Part1_eq3.png'>
     <img src='Part1_eq4.png'>

3. **Feature Detection and Motion Tracking**  
   - Use FAST (Features from Accelerated Segment Test) for feature detection.
   - Track the motion of these features using the KLT (Kanade-Lucas-Tomasi) optical flow algorithm.

4. **Velocity Estimation**  
   - Formulate the motion field equation to estimate the drone’s velocity in the camera frame.
     <img src='Part2_eq1.png'>
     <img src='Part2_eq2.png'>
     

5. **Outlier Rejection with RANSAC**  
   - Use the RANSAC (Random Sample Consensus) algorithm to reject outliers from the data.
   - Refine the estimate of the drone’s velocity for more accurate results.
     <img src='Part2_eq3.png'>

6. **Fusion with IMU Data Using UKF**  
   - Use the Unscented Kalman Filter (UKF) to fuse the velocity and pose estimates from the camera with data from the IMU for better overall accuracy.
     <img src='prediction.png'>
     <img src='update_1.png'>
     <img src='update_2.png'>
     <img src='measurement_model_2.png'> 

This process integrates both vision-based methods and IMU readings to achieve robust pose and velocity estimation for the drone in a dynamic environment.

### Result

<img src='part2_data2.png'>
<img src='part2_data1.png'>
<img src='part1_data_1.png'>
<img src='part1_data_2.png'>

### Tools
MATLAB







