ezrassor_joy_translator
---------------------
![Build Badge](https://github.com/FlaSpaceInst/ezrassor_joy_translator/workflows/Build/badge.svg) ![Style Badge](https://img.shields.io/badge/Code%20Style-black-000000.svg)

The `ezrassor_joy_translator` reads messages from a joy topic and translates those messages into commands for the EZRASSOR. These commands are made up of different actions that control various aspects of the robot. Actions are ultimately published on individual action topics.

This node supports several different controller configs. New configs should go in the `config/` directory.

The joy translator is one of several optional controller packages available for the EZRASSOR.

topics
------
```
joy_translator <- /joy

joy_translator -> /wheel_actions
joy_translator -> /front_arm_actions
joy_translator -> /back_arm_actions
joy_translator -> /front_drum_actions
joy_translator -> /back_drum_actions
joy_translator -> /routine_actions
```

usage
-----
```
command:
  ros2 launch ezrassor_joy_translator joy_translator.py [argument:=value]

optional arguments:
  device_id         joystick device id (default: 0)
  controller        desired controller configuration (default: xbox-360-controller.yaml)
```

examples
--------
Launch a joy translator with the default device and controller:
```
ros2 launch ezrassor_joy_translator joy_translator.py
```

Launch a joy translator with a device ID of `1` (corresponding to `/dev/input/js1`):
```
ros2 launch ezrassor_joy_translator joy_translator.py device_id:=1
```

Launch a joy translator using the Nintendo Switch controller configuration:
```
ros2 launch ezrassor_joy_translator joy_translator.py \
  controller:=nintendo-switch-controller.yaml
```
