jsk-ros-ios
===========

This is the project of creating iOS applications that enable to work on ROS environment.

The prebuilt frameworks are available on `binaries` directory. (currently supports all 64bit devices/simulators)

**NOTE**

Currently sample project (available on `xcode_project`) does not yet work.

## How to build frameworks

1. build ros core for iOS

```
sh build.sh
```

All frameworks will be built on `ros/frameworks` directory.

2. create your xcode project

Open xcode, create your own project, then save it to this top directory.

3. add ros related frameworks to your project

This procedure can be done only in xcode.

4. Have fun!

You can build your app at xcode.

## How to generate own ros package to framework

```
cd /path/to/ros-ios/ros
sh message_gen.sh -f /path/to/hoge_msgs /path/to/depend1_msgs /path/to/depend2_msgs ...
```

This will build `hoge_msgs.framework` which depends of `depend1_msgs` and `depend2_msgs` (it's also 
possible to generate a simple folder with the -d option instead of the -f).

iOS demo applications
---------------------

It's an Xcode project which can be found in the xcode_project directory.
