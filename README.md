# Model Predictive Controller

## 1. Introduction
This project implements a Model Predictive Controller in C++ to generate throttle and steer values for a vehicle to navigate a test track in a simulator. This project was completed as part of Term 2 of Udacity's Self-Driving Car Nanodegree program.

## 2. Project Environment
The project was built using the Ubuntu 16-04 bash shell in Windows 10. Instructions to set this up can be found [here](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/). The following dependencies need to be in place to build and execute the project.

* [Udacity SDC Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases)
* cmake >= 3.5
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
* gcc/g++ >= 5.4
* uWebSocketIO (installed via the [install-ubuntu.sh](https://github.com/shazraz/Extended-Kalman-Filter/blob/master/install-ubuntu.sh) script) 
* Ipopt and CppAD. Instructions [here](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md)

The project consists primarily of the following files located in the [src](https://github.com/shazraz/MPC-Controller/tree/master/src) folder:

* [main.cpp](https://github.com/shazraz/MPC-Controller/blob/master/src/main.cpp): Interfaces with the simulator using uWebSocketIO to recieve vehicle state values to generate the steer and throttle values.
* [MPC.cpp](https://github.com/shazraz/MPC-Controller/blob/master/src/MPC.cpp): Implements the model predictive controller.

Once the environment is ready, the code can be tested as follows:

1. Launch the simulator and select the MPC project
2. Execute ```./mpc``` from the build directory
3. Click Start in the simulator

## 3. Model Predictive Control
Model predictive control uses the kinematic model to predict the position of the vehicle along a prediction horizon, T, consisting of N steps of duration dt. The state and actuators at each time step are calculated to minimize the controller cost function. The actuations for the next time step are applied to the vehicle and the prediction is performed again. 

The state of the vehicle consists of <x, y, psi, v, cte, epsi> defined as follows:

* x: x-coordinate of vehicle position
* y: y-coordinate of vehicle position
* psi: vehicle headed
* v: vehicle velocity magnitude
* cte: cross track error (variation from reference line)
* epsi: error in vehicle heading

The kinematic model defines the transition of the state as follows:

* x_t+1 = x_t + v_t * cos(psi_t) * dt
* y_t+1 = y_t + v_t * sin(psi_t) * dt
* psi_t+1 = psi_t + v_t/Lf * delta * dt
* v_t+1 = v_t + a * dt
* cte_t+1 = cte_t + v_t * sin(epsi_t) * dt
* epsi_t+1 = epsi_t + v_t/Lf * delta * dt

where <a, delta> are the actutations to be determined by minimizing the cost function for the model predictive controller.

## 4. Cost Function
The cost function used for the model consists of the weighted sum of the squared error values for the following terms:

* The cross track error of the vehicle;  `CTE x CTE_weight`
* The error in vehicle heading; `EPSI x EPSI_weight`
* The difference between the vehicle velocity and reference velocity; `(V - Vref) x V_weight`
* The magnitude of the steering angle; `delta x delta_weight`
* The magnitude of the acceleration term; `a x a_weight`
* The rate of change of steering angle; `(delta_t+1 - delta_t) x delta_rate_weight`
* The rate of change of acceleration; `(a_t+1 - a_t) x a_rate_weight`

## 5. Latency
The implemented controller simulates a latency of 100ms between the calculation and implementation of the actuations to approximate the real-world delay of applying actuations to a vehicle. Unlike a PID controller, the MPC is capable of accounting for this delay in the model.

In this case, the vehicle state is transitioned using the kinematic model defined above in the vehicles frame of reference (i.e. vehicle at the origin with zero heading). The transitioned state is then fed into the MPC solver. This can be found in lines 145-158 of main.cpp.

## 6. Tuning & Results
The hyper-parameters to be tuned include the weights of all the cost function terms as well as the number of timesteps, N, and timesstep interval, dt, of the prediction horizon.

The parameter N determines the number of timesteps over which the cost function will be optimized and increases the total computation time required to solve for the actuations. A large N without the appropriate computational resources results in instability of the controller and large oscillations are observed in the vehicle movement. 

The parameter dt determines the interval between actuations. A large dt results in actuations being applied further apart in time resulting in larger deviations from the reference trajectory which also leads to larger actuations and instability of the controller which manifests as oscillations around the reference trajectory.

For the hardware being used, an optimal value of N and dt were 10 and 0.1 respectively resulting in a prediction horizon of 1 second. In addition, the simulator had to be run in the fastest graphics mode and the smallest resolution to achieve the best performance of the controller.

The weights were also tuned so that large actuations and a large rate of change of actuations were both heavily penalized. This resulted in a smooth steering response and a tendency to not oversteer to compensate for the CTE or EPSI. In addition, it was noted that a large weight on either the CTE or EPSI would cause oscillations around the reference trajectory which would result in instability at high speeds. Therefore, these weights were dropped as the reference speed of the controller was increased. Finally, the weight of the velocity difference was gradually increased until the vehicle reached its reference velocity consistently. The final values of all hyperparameters are summarized below.

* N: 10
* dt: 0.1
* CTE_weight: 3
* EPSI_weight: 3
* V_weight: 100
* delta_weight: 2500
* a_weight: 100
* delta_rate_weight: 4500
* a_rate_weight: 250
* Reference velocity: 60 mph
