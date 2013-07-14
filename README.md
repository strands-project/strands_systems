strands_systems
===============

## Fresh install of STRANDS stable on your system

The following are instruction to set up a clean STRANDS system with all packages that are considered stable. This is not to setup your own development tree, but the base system on which you can overlay your own workspace with your own developments. In other words: Never edit anything directly unter `/opt/strands`.

1. make sure you have all the ROS and other useful packages:
  * basic build stuff: `sudo apt-get install cmake git  zlib1g-dev  git-cvs wget`
  * ROS basic installation: `sudo apt-get install ros-groovy-desktop-full python-rosinstall` 
  * addtional packages needed for [webtools](https://github.com/strands-project/strands_webtools): 
      ```
      sudo apt-get install ros-groovy-rosbridge-suite ros-groovy-robot-pose-publisher ros-groovy-tf2-web-republisher ros-groovy-mjpeg-server

      ```
1. create a user `strands`: `sudo adduser strands` (choose a password)
1. make new user admin: ` sudo adduser strands sudo`
1. create a directory to contain the strands stable installation: `sudo mkdir -p /opt/strands`
1. make strands the owner of that directory: `sudo chown -R strands:strands /opt/strands`
1. login as user `strands`: `su strands`
1. change into the new directory: `cd /opt/strands`
1. on the desktop installation install MORSE
    1. run `wget https://gist.github.com/cburbridge/5782900/raw/8a4c01f579b8e5ebe68205fc73274172a2a52534/setup.sh` to get the MORSE installer script
    1. install MORSE: `bash setup.sh`
1. on the robot make sure that MIRA is installed
1. install [wstool](http://ros.org/wiki/wstool): `sudo pip install wstool` (this is only to be done once)
1. create your workspace, e.g `mkdir strands_catkin_ws`, and then change into it, e.g. `cd strands_catkin_ws`
1. get all the repositories for either the robot or the desktop version:
      1. desktop: run 
           ```
           wstool init src https://raw.github.com/strands-project/strands_systems/master/strands_rosinstall/strands-desktop-full.yaml
           
           ```
      1. scitos (for the actual robot): 
           ```
           wstool init src https://raw.github.com/strands-project/strands_systems/master/strands_rosinstall/strands-scitos.yaml

           ```
1. initialse the catkin workspace: `catkin_init_workspace src`
1. build the workspace: `catkin_make`
1. in your own `~/.bashrc` configure your shell to use the stuff in `/opt/strands` by adding the following at the end:
    ```
    # configure MORSE
    [ -f /opt/strands/.bashrc ] && source /opt/strands/.bashrc
    # configure the STRANDS catkin workspace
    [ -f /opt/strands/strands_catkin_ws/devel/setup.bash ] && source /opt/strands/strands_catkin_ws/devel/setup.bash

    ```




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
