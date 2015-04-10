^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.10 (2015-04-10)
-------------------
* Adding parmaters to start mary on different machine.
* Contributors: Christian Dondrup

0.0.9 (2015-03-24)
------------------
* Setting default of with_human_aware to true
  Now that https://github.com/strands-project/strands_hri/pull/91 is merged, human aware navigation dynamically subscribes and unsubscribes from the ppl perception pipeline. this means:
  * If the edge uses human aware navigation, everything works as expected. While the robot is driving it subscribes to the ppl perception. If not goal is active it unsubscribes.
  * If the edge uses move_base, human ware navigation is unsubscribed from the ppl perception, not causing any processing.
  * If the ppl perception is not running, human aware navigation behaves like move_base + some gazing behaviour
  Therefore, having it running by default does not cause additional load on the CPU if it is not used but it prevents confusion if it is used in the edge but not running.
* Adding strands_hri to package xml of strands_bringup.
* Added human_aware_navigation and dependencies to strands_navigation.launch. See `#101 <https://github.com/strands-project/strands_systems/issues/101>`_. Default behaviour is unchanged. To run with human_aware_navigation, start with .
* indigo-0.0.8
* Contributors: Chris Burbridge, Christian Dondrup

0.0.7 (2015-01-09)
------------------
* Include pc_monitor in robot bringup.
  Includes the now released pc_monitor in the strands_robot launch file.
* removing config for monitored nav (its already a default and it looks like it doesnt work)
* Contributors: Bruno Lacerda, Chris Burbridge

0.0.6 (2014-11-21)
------------------
* Make the subsampling parameters configurable.
* Same parameters as in movebase
* Contributors: Lucas Beyer, Nils Bore

0.0.5 (2014-11-20)
------------------
* Pass-through the EBC port for docking.
  DEFAULT TO NONE, ONLY ONE ROBOT HAS A LIGHT IN THERE.
  Feels good to shout once in the morning.
* Contributors: Lucas Beyer

0.0.4 (2014-11-19)
------------------
* Ability to include site-specific movebase parameters.
* Contributors: Lucas Beyer

0.0.3 (2014-11-19)
------------------
* Fixing argument rename
  This is wrong because both @nilsbore and me made PRs in parallel, so he didn't see my changes and I didn't see his.
* Update README.md
* Merge pull request `#91 <https://github.com/strands-project/strands_systems/issues/91>`_ from lucasb-eyer/hydro-devel
  Changes needed by `strands-project/strands_movebase#14 <https://github.com/strands-project/strands_movebase/issues/14>`_
* Added some more movebase parameters to strands_navigation
* Update to changes in strands_navigation/movebase
* Cleanup, while I'm at it.
* Contributors: Jaime Pulido Fentanes, Lucas Beyer, Marc Hanheide, Nils Bore

0.0.2 (2014-11-14)
------------------
* adding changes to launch files example start script and new strands_mapping and strands_ui launch files
* splitting strands_cameras from strands robot
* Fixing `#86 <https://github.com/strands-project/strands_systems/issues/86>`_
  Amidoinitrite?
* Contributors: Jaime Pulido Fentanes, Lucas Beyer

0.0.1 (2014-11-12)
------------------
* removing head user
* Update README.md
* Create README.md
* bug fixes
* Creating strands_bringup removing strands_bob, strands_linda, strands_rosie, strands_uol_sim and strands_wernee
* Contributors: Jaime Pulido Fentanes
