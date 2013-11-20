strands_systems
===============

## Maintaining a stable STRANDS installation

The following instructions are thought as a kind of "best practice" when installing a strands system (from scratch). You might want to deviate from this recipe, but then don't blame it on me (Marc) if it doesn't work as expected ;-)

### Fresh install of STRANDS stable on your system

The following are instruction to set up a clean STRANDS system with all packages that are considered stable. This is not to setup your own development tree, but the base system on which you can overlay your own workspace with your own developments. In other words: Never edit anything directly unter `/opt/strands`.

1. make sure you have all the ROS and other useful packages:
  * basic build stuff: `sudo apt-get install cmake git  zlib1g-dev  git-cvs wget python-pip`
  * Adding the ROS repository: 

           
           sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros-latest.list'

           

  * Adding the key of the ROS repository: `wget http://packages.ros.org/ros.key -O - | sudo apt-key add -`
  * Getting the package list from the ROS repository: `sudo apt-get update`
  * ROS basic installation: `sudo apt-get install ros-groovy-desktop-full python-rosinstall` 
  * UPDATE: ~~additional packages needed for [webtools](https://github.com/strands-project/strands_webtools)~~ is no longer needed. The `rosdep` calls takes care of this.
  * Initialise rosdep if not done before: `sudo rosdep init`
  * update rosdep: `rosdep update`

1. SUGGESTED: use a dedicated user and directory to install the stable system (*justification:*  Having a dedicated STRANDS user ensures that other people don't overwrite the stable system install or mess with it in an inappropriate way. Normal users shouldn't be allowed to write the system installation; e.g. on the robot). The suggested directory is `/opt/strands` and the user is `strands`. You may also choose to install the system in your own user space, e.g. `~/strands_stable`, then you have to substitute `/opt/strands` for your own path in the following.
  1. create a user `strands`: `sudo adduser strands` 
  1. make new user admin: `sudo adduser strands sudo` (*justification:* This makes it possible to install packages during the installation, you *may* want to remove strands from the `sudo` group if you want that extra bit of security).
  1. login as user `strands`: `su -l strands` 
  1. configure the `strands` user for ROS by putting the following at the end of the `~/.bashrc` file:
      ```
      [ -f /opt/ros/groovy/setup.bash ] && source /opt/ros/groovy/setup.bash    
      ```
     and then `source ~/.bashrc` to make it active immediately.
  1. create a directory to contain the strands stable installation: `sudo mkdir -p /opt/strands` (*justification:* we don't want to install it in the user's home, but system-wide)
  1. make strands the owner of that directory: `sudo chown -R strands:strands /opt/strands`
  1. change into the new directory: `cd /opt/strands`
1. install hardware specific dependencies or make sure the are installed (MIRA or MORSE)
  * MORSE: in your installation directory (here: `/opt/strands`) on the desktop:
      1. run `wget https://raw.github.com/strands-project/strands_systems/master/setup.sh` to get the MORSE installer script
      1. install MORSE: `bash setup.sh` (this will install MORSE directly from github in the latest master version and also make sure all other required packages are installed)
  * MIRA: on the robot make sure that MIRA is installed
1. install [wstool](http://ros.org/wiki/wstool): `sudo apt-get install python-wstool`  (*justification:* this is needed to manage the repositories)
1. create your workspace, ~~e.g.~~ `mkdir /opt/strands/strands_catkin_ws`, and then change into it, ~~e.g.~~ `cd /opt/strands/strands_catkin_ws`
1. get all the repositories for either the robot or the desktop version:
      * desktop: run 
           ```
           wstool init src https://raw.github.com/strands-project/strands_systems/master/strands_rosinstall/strands-desktop-full.yaml
           
           ```
      * scitos (for the actual robot): 
           ```
           wstool init src https://raw.github.com/strands-project/strands_systems/master/strands_rosinstall/strands-scitos-full.yaml

           ```
1. initialise the catkin workspace: `cd src; catkin_init_workspace; cd ..` 
1. install all dependencies automatically: `rosdep install --from-paths src --ignore-src --rosdistro groovy -y -r`
1. UPDATE: build the workspace: ~~catkin_make~~ `catkin_make -DCMAKE_BUILD_TYPE=Release` _The build type argument prevents the generation of debug symbols and uses -O3 optimization._
1. now your system is ready to go. Please follow the instructions below to get started: "Using an existing STRANDS installation and developing in it"    



### update the stable installation
If you need to update it simply do the following as user `strands`.
If you want to update your stable workspace with a version that has successfully been tested on jenkins, look here: https://github.com/strands-project/strands_ci/wiki#updating-a-workspace-with-software-versions-that-have-been-successfully-tested-on-jenkins. You can add the script that is used there to your crontab to automatically update your system to the latest stable over night. 
Alternatively, you can follow the steps below.

  1. `cd /opt/strands/strands_catkin_ws/src`
  1. `wstool merge URL` (where <strong>URL</strong> is whatever your system rosinstall configuration was that you chose earlier; this will make sure that you still have all the different repositories that belong to desktop-full installed, even if new ones are added. Usually you will see *Merge caused no change, no new elements found*, which is fine if the rosinstall config hasn't changed). For example:
           
           wstool merge https://raw.github.com/strands-project/strands_systems/master/strands_rosinstall/strands-desktop-full.yaml
           
  1. `wstool update` (gets everything from github, basically this runs `git pull` for you in all the repositories)
  1. UPDATE: `cd ..` and build it: ~~catkin_make~~ `catkin_make -DCMAKE_BUILD_TYPE=Release` _The build type argument prevents the generation of debug symbols and uses -O3 optimization._
 
You may consider to put this in a cronjob for your user `strands` to update every night after a successful jenkins build. This will make sure that you always have the latest installation.

### adding a library to your python3.3 installation, e.g. numpy:

 1. cd ${workspace}/tmp && git clone https://github.com/numpy/numpy.git
 1. cd numpy && git checkout v1.7.1
 1. ${workspace}/bin/python3.3 setup.py install

where `${workspace}` is, e.g., `/opt/strands`.

### updating MORSE
As user `strands` run:

 1. `cd /opt/strands/src/morse`
 1. `git pull`
 1. `cd build`
 2.   
           cmake -DCMAKE_INSTALL_PREFIX=${workspace} -DPYMORSE_SUPPORT=ON -DPYTHON_EXECUTABLE=${workspace}/bin/python3.3 -DBUILD_ROS_SUPPORT=ON ..
           
    where `${workspace}` should be set to your installation location (e.g. `/opt/strands`)
 1. `make install`

### migrating your MORSE installation from morse-simulator/morse to strands-project/morse
*This is only necessary if you have previously used the official MORSE repository*
As user `strands` run:

 1. `cd /opt/strands/src/morse`
 1. `git remote add strands-morse https://github.com/strands-project/morse.git`
 1. `git pull strands-morse master`
 1. `cd build`
 1. 
 cmake -DCMAKE_INSTALL_PREFIX=${workspace} -DPYMORSE_SUPPORT=ON -DPYTHON_EXECUTABLE=${workspace}/bin/python3.3 -DBUILD_ROS_SUPPORT=ON ..

    where `${workspace}` should be set to your installation location (e.g. `/opt/strands`) 
 1. `make install`


## Using an existing STRANDS installation and developing in it
* in your own `~/.bashrc` configure your shell to use the stuff in `/opt/strands` by adding the following at the end:
    ```
    # configure MORSE
    [ -f /opt/strands/.bashrc ] && source /opt/strands/.bashrc
    # configure the STRANDS catkin workspace
    [ -f /opt/strands/strands_catkin_ws/devel/setup.bash ] && source /opt/strands/strands_catkin_ws/devel/setup.bash

    ```
* run `source ~/.basrc` or start a new terminal to make sure your evironment is set up properly

### Setup your local CATKIN development workspace
This assumes you have a full current installation of the STRANDS system (e.g. in `/opt/strands` as described above). You checkout and create your own packages to work on in your local development workspace as described here. In ROS terms that is you *overlay* your own workspace. By overlaying, your packages hide the system one with the same name. E.g., if you are working on your own `strands_morse` than fork/branch `strands_morse` on github and clone it into your *own* workspace that you overlay over the system workspace. Everything you have in your own workspace will "overrule" the system stuff. But you will always have a clean system workspace (in `/opt/strands` if you followed the above).

1. create your workspace, ~~e.g~~ `mkdir catkin_ws`, and then change into it, ~~e.g.~~ `cd catkin_ws`
1. initialse the catkin workspace: `mkdir src; cd src; catkin_init_workspace; cd ..`
1. get or create packages to work on, either do
    1. **initialise from a whole system using wstool:**  run `wstool init src <add-your-system-url-here>`, e.g. using
           ```
           wstool init src https://raw.github.com/strands-project/strands_systems/master/strands_rosinstall/strands-desktop-full.yaml
           
           ```
        * see [here](https://github.com/strands-project/strands_systems/tree/master/strands_rosinstall) for other system definitions understood by wstool
    1. **create your own catkin package to start working in:** `cd src`, followed by `catkin_create_pkg <your package> <your deps>`
    1. **clone a specific repository:** `cd src`, followed by e.g. `wstool set scitos_apps --git https://github.com/strands-project/scitos_apps.git`, then run `wstool update` to clone the repository just set up
1. make sure you have configured the shell for the "underlay" workspace (i.e. `/opt/strands/devel/setup.bash`), read http://ros.org/wiki/catkin/Tutorials/workspace_overlaying to learn more about overlaying
1. in `catkin_ws` run `catkin_make`
1. make sure you source the config of your own workspace: `source devel/setup.bash`

A little [video](http://ascii.io/a/3882) illustrates a similar process 

#### Eclipse

If you want to use eclipse, you can have catkin create eclipse config files for you running

    ```
    cd ~/catkin_ws
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"
    source ~/catkin_ws/devel/setup.bash
    eclipse
    ```

Then you import the project into eclipse from `catkin_ws/build` using the import tool in eclipse (choose "existing project")


### ROSBUILD workspace
For some packages that are not catkinised yet you need to set up a separate ROSBUILD workspace. Read the details [here](http://ros.org/wiki/catkin/Tutorials/using_rosbuild_with_catkin). The following assumes you have created a catkin workspace as illustrated above.

1. create your ROSBUILD workspace, e.g `mkdir ros_ws`, and then change into it, e.g. `cd ros_ws`
1. initialse the ROSBUILD workspace from the catkin workspace, e.g. `rosws init . ~/catkin_ws/devel`
1. configure your workspace to include other repositories, e.g. `wstool set gmapping --git https://github.com/ros-perception/slam_gmapping.git` or `wstool merge https://raw.github.com/LCAS/uol-openday-robot/master/uol_rosinstall/uol-cob.yaml`
1. get the latest from the repositories: `rosws update gmapping`
1. make source you configure your shell to use this workspace: `source setup.bash`
1. run `rosmake` in any package you want to compile 

Now your shell is configured to find all packages *both* in your catkin workspace and your ROSBUILD workspace
