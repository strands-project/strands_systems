strands_systems
===============

## create a catkin workspace for different systems 

1. install [wstool](http://ros.org/wiki/wstool): `sudo pip install wstool` (this is only to be done once)
1. create your workspace: `mkdir catkin_ws`
1. get all the repositories of a system using `wstool init src <add-your-system-url-here>`
  * currently we have the following wstool configs available
      1. desktop: run `wstool init src https://raw.github.com/strands-project/strands_systems/master/strands_rosinstall/strands-desktop.yaml`
      1. scitos (for the actual robot): `wstool init src https://raw.github.com/strands-project/strands_systems/master/strands_rosinstall/strands-scitos.yaml`
  * see [here](https://github.com/strands-project/strands_systems/tree/master/strands_rosinstall) for other system definitions understood by wstool
1. initialse the catkin workspace: `catkin_init_workspace src`
1. build the workspace: `catkin_make`
1. to update all repositories from github (run `git pull` in all of them), 
  1. `cd src`
  1. `wstool update`
  1. voila, you have an up-to-date system


A little [video](http://ascii.io/a/3882) illustrates the actual process nicely
