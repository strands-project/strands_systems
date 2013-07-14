strands_systems
===============

## General ROS setup on UBUNTU

Make sure you have all the required software installed:

1. make sure you have all the ROS packages:
  * basic build stuff: `sudo apt-get install cmake git  zlib1g-dev  git-cvs`
  * ROS basic installation: `sudo apt-get install ros-groovy-desktop-full python-rosinstall` 
  * addtional packages needed for [webtools](https://github.com/strands-project/strands_webtools): 
      ```
      sudo apt-get install ros-groovy-rosbridge-suite ros-groovy-robot-pose-publisher ros-groovy-tf2-web-republisher ros-groovy-mjpeg-server

      ```
1. some basic requirements 
  * for your desktop, install MORSE first: [Getting started with Morse and ROS on Ubuntu](https://github.com/strands-project/strands_morse/wiki/MORSE-on-Ubuntu)
  * for the robot, make sure that MIRA is installed
1. install [wstool](http://ros.org/wiki/wstool): `sudo pip install wstool` (this is only to be done once)

## create a catkin workspace for different systems 

1. create your workspace, e.g `mkdir catkin_ws`, and then change into it, e.g. `cd catkin_ws`
1. get all the repositories of a system using `wstool init src <add-your-system-url-here>`
  * currently we have the following wstool configs available
      1. desktop-full: run 
           ```
           wstool init src https://raw.github.com/strands-project/strands_systems/master/strands_rosinstall/strands-desktop-full.yaml
           
           ```
      1. scitos (for the actual robot): 
           ```
           wstool init src https://raw.github.com/strands-project/strands_systems/master/strands_rosinstall/strands-scitos.yaml

           ```
  * see [here](https://github.com/strands-project/strands_systems/tree/master/strands_rosinstall) for other system definitions understood by wstool
1. initialse the catkin workspace: `catkin_init_workspace src`
1. build the workspace: `catkin_make`

If in the future you need to update it simply do the following (it will run `git pull` in all of them), 
  1. `cd src`
  1. `wstool update`
  1. voila, you have an up-to-date system


A little [video](http://ascii.io/a/3882) illustrates the actual process nicely
