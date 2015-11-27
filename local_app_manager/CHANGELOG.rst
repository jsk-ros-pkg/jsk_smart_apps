^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package local_app_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2015-11-27)
------------------
* CMakeLists.txt: remove find_package which is not actually used
* no more rosmake
* catkinize local_app_manager and its deps; still jsk_android_gui_api9 is not set deps correctly
* catkinize local_app_manager
* remove local demo_tray
* remove loca-pr2-demo/demo_tray
* add ROS_NOBUILD, because local_app_manager does not require compilation
* add local_app_manager, which is required by JSK PR2 system
* Contributors: Kei Okada, Ryohei Ueda, Yuki Furuta, Yuto Inagaki
