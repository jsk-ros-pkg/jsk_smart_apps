jsk_android_apps [![Circle CI](https://circleci.com/gh/jsk-ros-pkg/jsk_smart_apps.svg?style=svg)](https://circleci.com/gh/jsk-ros-pkg/jsk_smart_apps)
================

## Pre-requests

- If you installed android SDK and set ANDROID_HOME environment variable, then `catkin build jsk_android_apps` will compile apk

## How to generate apk from command line

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

## How to generate apk in CircleCI

Any Pull Request generate apk and store them as an artifacts, go and check at the CircleCI page
(https://circleci.com/gh/jsk-ros-pkg/jsk_smart_apps)

## How to send apk to GooglePlay store

        PASSWORD='89....x2' ./gradlew publishApkRelease

## How to update information on GooglePlay store

  see [Play Store Metadata](https://github.com/Triple-T/gradle-play-publisher/blob/master/README.md#credentials) for more information

        PASSWORD='89....x2' ./gradlew publishListingRelease
