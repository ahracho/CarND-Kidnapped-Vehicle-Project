# **Kidnapped Vehicle Project**

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


## Overview

This repository is for the project of **Udacity Nanodegree - Self-driving Car Engineer : Kidnapped Vehicle Proejct**.  It is forked from https://github.com/udacity/CarND-Kidnapped-Vehicle-Project).  


The goals / steps of this project are the following:
* Understand how Particle Filter works in Localization
* Implement Particle Filter in C++


## Environment Setting
I used Bash on Windows 10 for code complie.  

- C++ compile dependency / run `install-ubuntu.sh`
~~~sh
sudo apt-get update
sudo apt-get install git libuv1-dev libssl-dev gcc g++ cmake make
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make 
sudo make install
cd ../..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets
~~~

- code compile
~~~sh
cd CarND-Kidnapped-Vehicle-Project
cd build
cmake ..
make
./particle_filter
~~~

## File Structure
- `main.cpp` - communicates with the Simulator receiving data measurements, calls a functions for particle filter
- `particle_filter.cpp` - initializes the particles, calls the predict function and calls the updateWeight function
- `helper_function.h` - functions for easy calculation
