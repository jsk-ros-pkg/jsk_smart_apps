^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_smart_gui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2015-11-27)
------------------
* CMakeLists.txt: remove find_package which is not actually used
* [jsk_smart_apps] use kinect_head instead of openni following pr2 default naming
* [jsk_smart_gui] add missing deps jsk_android_gui for tablet_receiver.l
* no more rosmake
* put find_package roseus down in jsk_pr2_core_apps; revert unexpectedly deleted manifest.xml
* catkinize local_app_manager and its deps; still jsk_android_gui_api9 is not set deps correctly
* add euslisp_INCLUDE_DIRS
* remove unused package when build
* remove jsk_maps from build_depend
  Former-commit-id: d56ebf413e7c8069613b0db8ab032a200aafdd9f
* Add cmake_modules
  Former-commit-id: a82d5f68cbd28cf1abe527d54c02941b0e66a493
* add falign-functions=16 for compiling
  Former-commit-id: 655ff8264bd1e3b967ed7743ced07bcfd5372fa9
* Add include path of euslisp
* Install android SDK on travis
* catkinize jsk_smart_gui
* add .gitignore for jsk_smart_gui
* Merge remote-tracking branch 'origin/master' into use-tf2
* use tf2 to decrease CPU load
* remove object_snapshotter dependency for `#17 <https://github.com/jsk-ros-pkg/jsk_smart_apps/issues/17>`_
* added manipulation callback for tablet
* add jsk_rosjava_messages
* add :spin-once to tablet actions
* add (ros::load-ros-manifest object_snapshotter) to get_template.l
* comment out object_snapshotter
* fix for groovy
* update demo actions
* slow down the speed of tablet/cmd_vel
* added joystick cb to jsk_smart_gui, default is nil, click manipulation->switchjoy to enable joystick mode
* removed tablet_marker, usr tablet_marker_array, fixed bug of / counts
* temporary backup for migration
* added auto launch file generator for reusing template
* updated switchsensor function , also assoc camera_info
* updated open-fridge function same to detect_cans demo
* updated for visualizing, sending apeal button and task lists
* updated visualize
* get template origin coords for memorizing tasks
* added set_template function
* updated mux switch for avoid miss switching
* changed msg type from tablet to stringstamped
* added stop action and navigation tools from tablet, using actionlib
* removed ray_coords message , check cam-method-name, added new dependancy : object_snapshotter
* separated get_spots.l, added get_template.l for memorizing tasks
* auto script for making spots yaml file from eng2-scene.l , for android
* we dont use jsk_pcl_ros/SwithTopic anymore , just user mux to select the right topic
* added patch script for BGR conversion on android SDK by murase
* updated spot function for move_base
* typo and fixed camera switcher name
* added switch sensor callback for changeing camera device and 3Dsensor device
* added tablet msg definition
* added show3dline function
* fixed launch for those havenot put roseus on PATH
* forgot to add srv
* removed dependancy for ipad_gui
* added load-manfest for ipad_gui
* added dependency
* added launch file for tablet receiver
* added jsk_smart_gui package, contains scripts for tablet receiver
* Contributors: Hiroyuki Mikita, Kei Okada, Ryohei Ueda, Yohei Kakiuchi, Yuki Furuta, Yuto Inagaki, Haseru Chen, Shohei Fujii, Kazuto Murase
