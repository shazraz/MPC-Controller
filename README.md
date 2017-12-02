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
