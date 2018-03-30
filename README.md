# Extended Kalman Filter 

In this project we utilized a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

![alt text](https://github.com/jwangjie/SDC-Extended-Kalman-Filter-Project/blob/master/EKF.GIF)

For project instructions, check [this](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project). 

## Overview of a Kalman Filter: Initialize, Predict, Update
![alt text](https://github.com/jwangjie/SDC-Extended-Kalman-Filter-Project/blob/master/data/The%20Sensor%20Fusion%20Flow%20using%20Kalman%20Filter%20algorithm.png)
There are three main steps for programming a Kalman filter:

* initializing Kalman filter variables
* predicting where our object is going to be after a time step Δt
* updating where our object is based on sensor measurements

Then the prediction and update steps repeat themselves in a loop.

To measure how well our Kalman filter performs, root mean squared error comparing the Kalman filter results with the provided ground truth was calculated.

These three steps (initialize, predict, update) plus calculating RMSE encapsulate the entire extended Kalman filter project.

#### Files in the [src Folder](https://github.com/jwangjie/SDC-Extended-Kalman-Filter-Project/tree/master/src)

* main.cpp - communicates with the Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE
* FusionEKF.cpp - initializes the filter, calls the predict function, calls the update function
* kalman_filter.cpp- defines the predict function, the update function for lidar, and the update function for radar
* tools.cpp- function to calculate RMSE and the Jacobian matrix

#### How the Files Relate to Each Other
Here is a brief overview of what happens when you run the code files:
1.	Main.cpp reads in the data and sends a sensor measurement to FusionEKF.cpp
2.	FusionEKF.cpp takes the sensor data and initializes variables and updates variables. The Kalman filter equations are not in this file. FusionEKF.cpp has a variable called ekf_, which is an instance of a KalmanFilter class. The ekf_ will hold the matrix and vector values. You will also use the ekf_instance to call the predict and update equations.
3.	The KalmanFilter class is defined in kalman_filter.cpp and kalman_filter.h. You will only need to modify 'kalman_filter.cpp', which contains functions for the prediction and update steps.

---

## Kalman Filter Algorithm

There are many intuitive materials taking about the Kalman Filter, thus it’s not necessary to add one more. Here are some key points to myself.

The process flow can be shown as

<p align="center">
  <img src="https://upload.wikimedia.org/wikipedia/commons/a/a5/Basic_concept_of_Kalman_filtering.svg" 
       width="600px" height="300px"/>
</p>


#### The State Transition Function

<p align="center">
  <img src="https://github.com/jwangjie/SDC-Extended-Kalman-Filter-Project/blob/master/data/KF1.png" 
       width="700px" height="380px"/>
  <img src="https://github.com/jwangjie/SDC-Extended-Kalman-Filter-Project/blob/master/data/The%20General%20Control%20System%20Block-Diagram.png" 
       width="600px" height="300px"/>
</p>

#### Prediction

<p align="center">
  <img src="https://github.com/jwangjie/SDC-Extended-Kalman-Filter-Project/blob/master/data/KF%20Prediction.png" 
       width="460px" height="300px"/>
  <img src="https://github.com/jwangjie/SDC-Extended-Kalman-Filter-Project/blob/master/data/State%20and%20Convariance.png" 
       width="460px" height="260px"/>
</p>

* x is the mean state vector. For an extended Kalman filter, the mean state vector contains information about the object's position and velocity that you are tracking. It is called the "mean" state vector because position and velocity are represented by a Gaussian distribution with mean x.
* P is the state covariance matrix, which contains information about the uncertainty of the object's position and velocity. You can think of it as containing standard deviations.
* k represents time steps. So x_k refers to the object's position and velocity vector at time k.
* The notation k+1∣k refers to the prediction step. At time k+1, you receive a sensor measurement. Before taking into account the sensor measurement to update your belief about the object's position and velocity, you predict where you think the object will be at time k+1. You can predict the position of the object at k+1 based on its position and velocity at time k. Hence x_(k+1∣k) means that you have predicted where the object will be at k+1 but have NOT YET taken the sensor measurement into account.
* x_(k+1) means that you have now predicted where the object will be at time k+1 and then used the sensor measurement to update the object's position and velocity.

#### Math
<p align="center">
  <img src="https://github.com/jwangjie/SDC-Extended-Kalman-Filter-Project/blob/master/data/Kalman%20Filter%20Equaitons.png" width="460px" height="300px"/>
  <img src="https://github.com/jwangjie/SDC-Extended-Kalman-Filter-Project/blob/master/data/KF2.png" 
width="700px" height="220px"/>
</p>

* x - state vector
* P - uncertainty covariance matrix of state x (process covariance)
* z - measurement vector 
* R - uncertainty covariance matrix of sensor that produces z (measurement covariance)
* F - update matrix - used to get predicted x - based on time elapsed and assumed dynamic model being tracked
* H - extraction matrix - used to extract the hypothetical measurement if state x is correct and the sensor is perfect
* Q - noise covariance matrix - adds uncertainty to the process covariance
* S - 'innovation' covariance that combines process covariance and measurement covariance
* y - difference between the actual measurement and the predicted measurement 
* K - Kalman gain - contains information on how much weight to place on the current prediction and current observed measurement that will result the final fused updated state vector and process covariance matrix computed from P (process covariance), H (extraction), R (measurement covariance)

#### Reference
1.	[Lesson 6.4 Estimation Problem Refresh](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/3612b91d-9c33-47ad-8067-a572a6c93837/concepts/81d536e6-4f6f-4970-94a2-eec7f0a20595#) 
2.	[Lesson 7.8 File Structure]( https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/295dfb84-91ad-4887-87b0-3c0b635684b5) 
3.	[Kalman Filter Design ](https://classroom.udacity.com/courses/cs373/lessons/48723604/concepts/486836600923)
4.	[Sensor Fusion and Object Tracking using an Extended Kalman Filter Algorithm](https://medium.com/@mithi/object-tracking-and-fusing-sensor-measurements-using-the-extended-kalman-filter-algorithm-part-1-f2158ef1e4f0)
5.	[Understanding the Basis of the Kalman Filter Via a Simple and Intuitive Derivation](http://www.cl.cam.ac.uk/~rmf25/papers/Understanding%20the%20Basis%20of%20the%20Kalman%20Filter.pdf)
6.	[Kalman Filter Matrices](https://www.udacity.com/wiki/cs373/kalman-filter-matrices)
7.  [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter)
8.  [Kalman Filter, Extended Kalman Filter, Unscented Kalman Filter](https://medium.com/@kastsiukavets.alena/kalman-filter-extended-kalman-filter-unscented-kalman-filter-dbbd929f83c5)



