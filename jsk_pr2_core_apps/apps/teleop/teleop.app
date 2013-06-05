display: Teleop
description: A really nice application for my robot.
platform: pr2
launch: jsk_pr2_core_apps/teleop.launch
interface: jsk_pr2_core_apps/teleop.interface
icon: jsk_pr2_core_apps/teleop.png
clients:
 - type: android
   manager:
     api-level: 13
     intent-action: org.ros.android.teleop.MainActivity