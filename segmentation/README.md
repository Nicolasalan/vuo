# Color segmentation

This repository is a package in ROS 1 that implements a color segmenter. This package has an interface with ROS to visualize the segmentation and a service so that it only shows the centroid of the segmentation.

**Requirements**

Before executing the package, you first have to calibrate the segmentation for your use case, for that I use the [repository](https://github.com/abhisavaliya/hsv_calibration/).

* ROS Noetic
* Ubuntu 20.04
* Python 3.8
* Numpy
* Imutils
* Opencv-python 4.6

```
pip install hsv-calibration 
```
Once you calibrate the package, run the commands.
```
git clone https://github.com/Nicolasalan/Color-Segmentation.git
cd Color-segmentation
pip3 install -r requirements.txt
```
After installing the package, compile with the command below.
```
cd src
source devel/setup.bash
catkin_make
```
**To start the node and service.**

Starts the segmentation service:
```
rosrun Color-segmentation blob_detection
```
Start the view node:
```
rosrun Color-segmentation blob_visualize
```

