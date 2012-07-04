display: HRP-4C Pan Tilt
description: Control pan and tilt angles of HRP-4C robot.
platform: hrp4
launch: hrp4_pan_tilt/hrp4c.launch
interface:  hrp4_pan_tilt/app.interface
icon:  hrp4_pan_tilt/hrp4c.jpeg
clients:
- type: android
  manager:
    api-level: 9
    intent-action: ros.android.pantilt.PanTilt
  app: 
    gravityMode: 0
    camera_topic: /image_raw
