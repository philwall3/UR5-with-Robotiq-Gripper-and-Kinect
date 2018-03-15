# common-sensors

A collection of commonly used sensors: urdf files and a few tools

This package includes a variety of sensor models imported from different ROS packages available throughout the web.
I picked the best / most suitable models and copied them to this package, so they are all available in one location.
Several changes to the individual models were made, e.g. to share certain common properties, add new properties, gazebo
tags, model refinements and adding extended models (e.g. a stand for the Xtion).

Includes: 

* Kinect sensor based on [turtlebot_description](http://wiki.ros.org/turtlebot_description)
* Xtion sensor based on [robotnik sensors](https://github.com/RobotnikAutomation/robotnik_sensors/)
* Hokuyo 04lx based on [robotnik_sensors](https://github.com/RobotnikAutomation/robotnik_sensors/)
* Hokuyo utm30lx based on [hector_sensors_description](http://wiki.ros.org/hector_sensors_description) 
* from Uni Texas: copied the FootprintFilter and NanToInfFilter. Here is their [LICENSE](https://github.com/utexas-bwi/segbot/blob/devel/LICENSE).

This package has the aim of grouping several simple sensor models such that they can be used in the other packages without
the need to introduce a large set of ROS package dependencies just because of the URDF models of the sensors.
Further, most of the urdf models have been improved, but are still based on the originals referenced in the list above.

### Dependencies

- [laser_filters](http://wiki.ros.org/laser_filters)
- [openni_launch](http://wiki.ros.org/openni_launch)


**Install mandatory dependencies**

```
sudo apt-get install \
    ros-<distro>-laser-filters \
    ros-<distro>-openni-launch
```

**Install common-sensors**

Add the git repository to your catkin workspace:

```
cd <your-catkin-ws>/src
git clone https://github.com/JenniferBuehler/common-sensors.git
```

*Hint*: Alternatively to cloning the repositry directly into the catkin source folder, you
may also clone the repositories elsewhere and then create a softlink to the main folders
in your catkin source directory:    
``ln -s <path to common-sensors>`` 

**Compile**
 
To compile, you can now use catkin\_make as usual:

```
cd ..
catkin_make
```
