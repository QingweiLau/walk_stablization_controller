# walk_stablization_controller
This is a biped walking stabilizer package based on [Choreonoid](https://github.com/s-nakaoka/choreonoid) for [thormang3](http://www.robotis.us/thormang3/), 

### **Plese note that this work is still undergoing, and any comments or advise is welcome.**

## Dependencies


* Choreonoid

Please refer to [choreonoid.org](https://choreonoid.org/en/) for the environment setup, you can find the standard installation procedure [here](https://choreonoid.org/en/manuals/latest/install/install.html), or with choreonoid_rosplugin [here](https://choreonoid.org/en/manuals/latest/wrs2018/teleoperation-ros.html). This package is with the latter one and use `catkin` for build. I personnaly recommend [Ubuntu 1604LTS](http://releases.ubuntu.com/16.04/) for choreonoid for now.

* ROS

Since choreonoid rosplugin depends on ROS, please install ROS first. 
 for Ubuntu 1604, it is [Kinetic](http://wiki.ros.org/kinetic/Installation) .


## Package structure

*  walk_stablization_controller
    * include
    * projects      // project file for chorenoid 
      * model      // model file of thormang3 and simulation environment
    * src
    * CMakeLists.txt
    * package.xml

## Installation

Once you have set up your catkin workspace and successfully build all four packages `choreonoid`,`choreonoid_rosplugin`,`choreonoid_ros_samples`, and `choreonoid_joy`,

  ```
  cd /to/your/catkin_ws/src/dir
  catkin build walk_stablization_controller
  ```

## Acknowledgment

This work is under the supervision of [Dr. Shuuji Kajita](https://staff.aist.go.jp/s.kajita/index-e.html), to whom I pay all my repect and gratitude, and all colleagues from [HRG](https://unit.aist.go.jp/is/humanoid/index.html), AIST have helped me a lot.