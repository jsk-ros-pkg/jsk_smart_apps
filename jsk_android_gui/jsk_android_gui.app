display: JSK ANDROID GUI
description: Control pan and tilt angles of PR2's head, direction and velocity of PR2's base.
platform: PR2
launch: jsk_android_gui_api9/start_gui.launch
interface:  jsk_android_gui_api9/app.interface
icon:  jsk_android_gui_api9/jsk_android_gui.jpg
clients:
- type: android
  manager:
    api-level: 9
    intent-action: org.ros.android.jskAndroidGui.JskAndroidGui
  app: 
    gravityMode: 0
    camera_topic: /tablet/marked/image_rect_color/compressed_throttle
