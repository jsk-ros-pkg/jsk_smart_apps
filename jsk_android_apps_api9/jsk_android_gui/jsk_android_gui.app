display: JSK ANDROID GUI
description: Control pan and tilt angles of PR2's head, direction and velocity of PR2's base.
platform: PR2
launch: jsk_android_gui/start_gui.launch
interface:  jsk_android_gui/app.interface
icon:  jsk_android_gui/jsk_android_gui.jpg
clients:
- type: android
  manager:
    api-level: 9
    intent-action: ros.android.jskandroidgui.JskAndroidGui
  app: 
    gravityMode: 0
    camera_topic: /camera/marked/image_rect_color/compressed_throttle
