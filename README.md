# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project an Extended Kalman Filter was used to track various objects given noisy lidar and radar measurement data. The EKF was able to succesfully fuse the two different kinds of measurements together even though lidar was in cartesian coordinates and radar was in polar. Although radar was not as accurate spatially as lidar, it could directly measure the velocity of objects. 

## EKF Performance on data
#### sample-laser-radar-measurement-data-1.txt
Accuracy - RMSE:

0.070975

0.0656733

0.564486

0.551459

#### sample-laser-radar-measurement-data-2.txt
Accuracy - RMSE:

0.18372

0.189987

0.473033

0.789295

#### Extra Data Generated From Matlab
Accuracy - RMSE:

0.0674286

0.0800124

0.22187

0.254647

### Visuals

All three data samples can be visually inspected by using the included ekf-visualization jupyter notebook file.
Also additonal data can be generated from the included ekf_data_generator.m file.

### Exploration

As an experiment RMSE values were compared when using both lidar/radar or only one or the other.

RMSE Lidar ONLY

.112 

.142 

.457

.388                 

RMSE Radar ONLY

.182

.212

.387

.336

RMSE Lidar+Radar

.151

.117

.41

.33

---

## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/12dd29d8-2755-4b1b-8e03-e8f16796bea8)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

