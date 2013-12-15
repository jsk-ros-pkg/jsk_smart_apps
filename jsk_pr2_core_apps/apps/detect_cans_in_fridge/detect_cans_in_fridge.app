display: Refrigerator Demo
description: A really nice application for my robot.
platform: pr2
launch: jsk_pr2_core_apps/detect_cans_in_fridge.launch
interface: jsk_pr2_core_apps/detect_cans_in_fridge.interface
icon: jsk_pr2_core_apps/detect_cans_in_fridge.png
clients:
- type: multitask
  manager:
    target_1: georgia
    target_2: wonda
    target_3: iemon
    target_4: mountain_dew