strands_systems
===============

## Maintaining a stable STRANDS installation

### Fresh install of STRANDS stable on your system

The following are instruction to set up a clean STRANDS system with all packages that are considered stable. This is not to setup your own development tree, but the base system on which you can overlay your own workspace with your own developments. In other words: Never edit anything directly unter `/opt/strands`.

1. make sure you have all the ROS and other useful packages:
  * basic build stuff: `sudo apt-get install cmake git  zlib1g-dev  git-cvs wget python-pip`
  * ROS basic installation: `sudo apt-get install ros-groovy-desktop-full python-rosinstall` 
  * addtional packages needed for [webtools](https://github.com/strands-project/strands_webtools): 
      ```
      sudo apt-get install ros-groovy-rosbridge-suite ros-groovy-robot-pose-publisher ros-groovy-tf2-web-republisher ros-groovy-mjpeg-server

      ```
1. create a user `strands`: `sudo adduser strands` (choose a password)
1. make new user admin: ` sudo adduser strands sudo`
1. create a directory to contain the strands stable installation: `sudo mkdir -p /opt/strands`
1. make strands the owner of that directory: `sudo chown -R strands:strands /opt/strands`
1. login as user `strands`: `su -l strands`
1. configure the `strands` user for ROS by putting the following at the end of the `.bashrc` file:
    ```
    [ -f /opt/ros/groovy/setup.bash ] && source /opt/ros/groovy/setup.bash    
    ```
1. change into the new directory: `cd /opt/strands`
1. on the desktop installation install MORSE
    1. run `wget https://gist.github.com/cburbridge/5782900/raw/8a4c01f579b8e5ebe68205fc73274172a2a52534/setup.sh` to get the MORSE installer script
    1. install MORSE: `bash setup.sh`
1. on the robot make sure that MIRA is installed
1. install [wstool](http://ros.org/wiki/wstool): `sudo pip install wstool`
1. create your workspace, e.g `mkdir /opt/strands/strands_catkin_ws`, and then change into it, e.g. `cd /opt/strands/strands_catkin_ws`
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
    
### update the stable installation
If you need to update it simply do the following (it will run `git pull` in all of them), 
  1. `cd /opt/strands/strands_catkin_ws/src`
  1. `wstool update` (gets everything from github)
  1. `cd ..` and build it: `catkin_make`

### updating MORSE
As user `strands` run:

 1. `cd /opt/strands/src/morse`
 1. `git pull`
 1. `cd build`
 1. `make install`


## Setup your local development workspace
This assumes you ahve a full current installation of the STRANDS system (e.g. in `/opt/strands` as described above). You checkout and create your own packages to work on in your local development workspace as described here. In ROS terms that is you *overlay* your own workspace

1. create your workspace, e.g `mkdir catkin_ws`, and then change into it, e.g. `cd catkin_ws`
1. initialse the catkin workspace: `mkdir src; cd src; catkin_init_workspace; cd ..`
1. get or create packages to work on, either do
    1. **initialise from a whole system using wstool:**  run `wstool init src <add-your-system-url-here>`, e.g. using
           ```
           wstool init src https://raw.github.com/strands-project/strands_systems/master/strands_rosinstall/strands-desktop-full.yaml
           
           ```
        * see [here](https://github.com/strands-project/strands_systems/tree/master/strands_rosinstall) for other system definitions understood by wstool
    1. **create your own catkin package to start working in:** `cd src`, followed by `catkin_create_pkg <your package> <your deps>`
    1. **clone a specific repository into src:** `cd src`, followed by e.g. `git clone https://github.com/strands-project/scitos_apps.git`
1. make sure you have configured the shell for the "underlay" workspace (i.e. `/opt/strands/devel/setup.bash`), read http://ros.org/wiki/catkin/Tutorials/workspace_overlaying to learn more about overlaying
1. in `catkin_ws` run `catkin_make`
1. make sure you source the config of your own workspace: `source devel/setup.bash`


A little [video](http://ascii.io/a/3882) illustrates the actual process nicely

### ROSBUILD workspace
For some packages that are not catkinised yet you need to set up an extra ROSBUILD workspace. Read the details [here](http://ros.org/wiki/catkin/Tutorials/using_rosbuild_with_catkin). The following assumes you have created a catkin workspace as illustrated above.
1. create your ROSBUILD workspace, e.g `mkdir ros_ws`, and then change into it, e.g. `cd ros_ws`
1. initialse the ROSBUILD workspace from the catkin workspace, e.g. `rosws init . ~/catkin_ws/devel`
1. configure your workspace to include other repositories, e.g. `wstool set gmapping --git https://github.com/ros-perception/slam_gmapping.git` or `wstool merge https://raw.github.com/LCAS/uol-openday-robot/master/uol_rosinstall/uol-cob.yaml`
1. get the latest from the repositories: `rosws update gmapping`
1. make source you configure your shell to use this workspace: `source setup.bash`
