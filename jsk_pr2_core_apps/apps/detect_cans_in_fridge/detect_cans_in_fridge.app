display: Refrigerator Demo
description: A really nice application for my robot.
platform: pr2
launch: jsk_pr2_core_apps/detect_cans_in_fridge.launch
interface: jsk_pr2_core_apps/detect_cans_in_fridge.interface
icon: jsk_pr2_core_apps/detect_cans_in_fridge.png
clients:
- type: multitask
  targets:
    - georgia
    - wonda
    - iemon
    - mountain_dew