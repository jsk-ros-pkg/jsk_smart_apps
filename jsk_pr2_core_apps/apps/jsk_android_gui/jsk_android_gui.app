display: JSK ANDROID GUI
description: Control pan and tilt angles of PR2's head, direction and velocity of PR2's base.
platform: PR2
launch: jsk_pr2_core_apps/jsk_android_gui.launch
interface:  jsk_pr2_core_apps/jsk_android_gui.interface
icon:  jsk_pr2_core_apps/jsk_android_gui.png
clients:
- type: android
  manager:
    api-level: 9
    intent-action: ros.android.jskandroidgui.JskAndroidGui
  app: 
    gravityMode: 0
    camera_topic: /tablet/marked/image_rect_color/compressed_throttle
