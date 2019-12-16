# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

# Performance Evaluation

## MP6 
- For Perforamnce Stats look at the folder data/ it's in the data.csv file 
- For images look at the folder data/images
- knn matching was used with k=2 as the selectorType and the matcher type was set to MAT_BF

## MP7 
### Both total points and points within the ROI were counted for each detector type 
- The most matching points were found with : 
1. FAST (approx. 400 per image pair)
2. BRISK (254-297 per image pair)

- The fewest matching points were found with : 
1. HARRIS (always less than 100 per image pair)
2. ORB and SHITOMASI (both were around the 100-130 range for an image pair)

## MP9
- FAST was mostly the fastest detector (about 1-2 ms) while also producing the most matches 
- the descriptors BRIEF (1ms), ORB (1ms) and SIFT(about 10-11 ms) all performed pretty well and all offer real time Performance when combined with FAST
- Therefore the first 3 options therfore are BRIEF, ORB and SIFT descriptors combined with a FAST detector 
- SIFT however should be avoided for a commercial application if possible due to the patent that goes along with it, this does not pose a problem in the project though 
