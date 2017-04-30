## Extended Kalman Filter

### Fuse LIDAR and radar to track object

---


[//]: # (Image References)
[NIS]: ./images/NIS.png
[plot]: ./images/plot.png
[zoom1]: ./images/zoom1.png
[zoom2]: ./images/zoom2.png


### How to use it

download this project:

`git clone 'thisrepo'`

change directory:

`cd thisrepo/build`

compile the project (optional)

`cmake .. && make`

run the executable with input and output files specified:

`./UnscentedKF ../data/obj_pose-laser-radar-synthetic-input.txt ../data/output.txt`

then visualize `output.txt` using `ukf-visualization.ipynb`

### Accuracy result
#### For input data 'obj_pose-laser-radar-synthetic-input.txt':

Accuracy - RMSE:
[0.0730622, 0.0843373, 0.349426, 0.263418]

Result:
![alt text][plot]

The zoomed views:
![alt text][zoom1]

![alt text][zoom2]

###  Algorithm details and discussion

This project is built in Oo style following the sample pipeline provided by Udacity.

Different initializtoin settings were tried and the following setting was finally selected:
* The first measurements are used to initialize the x and y positions in the state vector and the other three elements: speed, yaw angle and yaw rate are initialized to 0. 
* The covariance matrices are initialized to

`0.1, 0, 0, 0, 0,

0, 0.1, 0, 0, 0,

0, 0, 1, 0, 0,

0, 0, 0, 1, 0,

0, 0, 0, 0, 1;`

* The standard deviation of longitudinal acceleration in the process noise is set to 3m/s^2. This means I expect the longitudinal acceleration to be between −6m/s^2​​ and +6m/s^2 about 95% of the time.
* The standard deviation of yaw acceleration in the process noise is set to 1rad/s^2. This means I expect the yaw acceleration to be between −2rad/s^2​​ and +2rad/s^2 about 95% of the time.

Those initilization settings have been verified by plotting the NIS score.

![alt text][NIS]

The 95 percentile value of Chi-squre distribution is 7.8 for degree of freedom = 3 (radar case) and 6.0 for degree of freedom =2 (lidar case). As shown in the figure above, our estimation of the process noise is appropriate. 

* The algorithm uses different update functions based on the type of measurements (LIDAR or radar).
