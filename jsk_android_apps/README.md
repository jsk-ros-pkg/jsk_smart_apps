jsk_android_apps [![Circle CI](https://circleci.com/gh/jsk-ros-pkg/jsk_smart_apps.svg?style=svg)](https://circleci.com/gh/jsk-ros-pkg/jsk_smart_apps)
================

## For users

Download Android Apps from [Google Play](https://play.google.com/store/apps/developer?id=JSK+Robotics+Laboratory)

Description of App pages tells how to use these apps, but you can also find them from direct link.

- [android_camera_viewer](https://raw.githubusercontent.com/jsk-ros-pkg/jsk_smart_apps/master/jsk_android_apps/android_camera_viewer/src/main/play/en-US/listing/fulldescription)
- [android_image_view](https://raw.githubusercontent.com/jsk-ros-pkg/jsk_smart_apps/master/jsk_android_apps/android_image_view/src/main/play/en-US/listing/fulldescription)
- [android_sensor_message](https://raw.githubusercontent.com/jsk-ros-pkg/jsk_smart_apps/master/jsk_android_apps/android_sensor_message/src/main/play/en-US/listing/fulldescription)
- [android_voice_message](https://raw.githubusercontent.com/jsk-ros-pkg/jsk_smart_apps/master/jsk_android_apps/android_voice_message/src/main/play/en-US/listing/fulldescription)

### Tips

#### Generate QR code to tell ROS_MASTER_URI to the android apps

Inputting ROS_MASTER_URI whenever you launch the app may painful. You can generate QR code for your ROS_MASTER_URI as follows:

```
sudo apt-get install qrencode imagemagick ros-$ROS_DISTRO-jsk-tools
source /opt/ros/indigo/setup.bash
rossetip
export ROS_MASTER_URI=http://$ROS_IP:11311
qrencode -o /tmp/master-$$.png -s 8 $ROS_MASTER_URI
convert /tmp/master-$$.png -background white -extent 264x274 -pointsize 18 -fill black -gravity center -draw "text 0,122 '$ROS_MASTER_URI'"  ros_master_uri.png
```

#### Use ROS_IP

When android app connect to the node running on your computer, the android device must resolve the hostname of your machine. If it is not the case, you need to set [ROS_IP](http://wiki.ros.org/ROS/EnvironmentVariables#ROS_IP.2BAC8-ROS_HOSTNAME) using IP address.

`rossetip` command in [jsk_tools](https://github.com/jsk-ros-pkg/jsk_common/blob/master/jsk_tools/README.md) package may help to set appropriate `ROS_IP`.

## For Developers

### Generate apk in CircleCI

Any Pull Request generate apk and store them as an artifacts, go and check at the CircleCI page
(https://circleci.com/gh/jsk-ros-pkg/jsk_smart_apps)

Once the PR is merged to master and had new version, Meta information and apk is automatically upload to the Google Play Store.

### Working on your Computer

#### Pre-requests

- If you installed android SDK and set ANDROID_HOME environment variable, then `catkin build jsk_android_apps` will compile apk

#### How to generate apk from command line

1. Generate debug version of apk

        roscd jsk_android_apps
        ./gradlew assembleDebug
        find -iname '*-debug.apk' -print 
        
2. Generate signed version of apk

   put your `key.json` file under `jsk_android_apps` and run following commands:

        roscd jsk_android_apps
        ./gradlew assembleRelease
        PASSWORD='89....x2' ./gradlew assembleRelease
        find -iname '*-release.apk' -print 

#### How to update information on GooglePlay store

  see [Play Store Metadata](https://github.com/Triple-T/gradle-play-publisher/blob/master/README.md#credentials) for more information

        PASSWORD='89....x2' ./gradlew publishListingRelease

#### How to send apk to GooglePlay store

        PASSWORD='89....x2' ./gradlew publishApkRelease

