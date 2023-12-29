# KR_GCS_user_interface_ROS2

## Overview
- KR_GCS_user_interface_ROS2 is the software that includes a GUI that can run KR-DATC of KORAS Robotics. The user can receive the status of KR-DATC through the topic of ROS2, and command through the service of ROS2.

---
## Compatible Operating Systems
- Ubuntu 22.04 (Jammy Jellyfish)
    - ROS2 Humble build system

---
## Dependencies
- Boost
- libmodbus
- Qt
- Feather-icon
- Noto Sans

If ROS2 Humble was installed correctly, most dependencies would have been installed except libmodbus. To install libmodbus, follow these steps.
```shell
sudo apt update
sudo apt install libmodbus-dev
```

---
## Warning
- KR-DATC is a tooling system that changes and uses various grippers such as pneumatic grippers, 2-finger grippers, and 3-finger grippers. Therefore, there are many different precautions from using general grippers, so please read KORAS Robotics' DATC manual before using this software.
- Homepage: http://korasrobotics.com

---
## Build & Run
```shell
$ cd <path_to_ROS2_workspace>
$ colcon build
$ source install/setup.bash
$ ros2 run kr_gcs_ui kr_gcs_ui
```

---
## Troubleshooting
- This section lists solutions to a set of possible errors which can happen when using the KR_GCS_user_interface_ROS2.
#### qt.qpa.plugin error
- If you have seen an error such as **"qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "~/platforms:" even though it was found."** when you run the released KR_GCS_user_interface_ROS2 on Ubuntu 22.04, you can resolve it in the following way.
```shell
$ sudo apt update
$ sudo apt install pyqt5-dev*
```

---
## Communication Using ROS2 Topic and Service

#### ROS2 Topic
- Topic name: /grp_state
- Type: grp_control_msg/msg/GripperMsg
- Frequency: about 50Hz

| Variable Name       | Data Type | Value
| ----                | ----      | ----
| motor_position      | int16_t   | Position of the motor (deg)
| motor_current       | int16_t   | Current of the motor (mA)
| motor_velocity      | int16_t   | Velocity of the motor (rpm)
| finger_position     | uint16_t  | Finger position of the DATC (0 ~ 1000 (0: closed & 1000: open))
| motor_enabled       | boolean   | 0: False, 1: True
| gripper_initialized | boolean   | 0: False, 1: True
| position_ctrl_mode  | boolean   | 0: False, 1: True
| velocity_ctrl_mode  | boolean   | 0: False, 1: True
| current_ctrl_mode   | boolean   | 0: False, 1: True
| grp_opened          | boolean   | 0: False, 1: True
| grp_closed          | boolean   | 0: False, 1: True
| motor_fault         | boolean   | 0: False, 1: True

#### ROS2 Service
- Please refer to the DATC manual for a detailed description of each function.

**Description of each service**
Service Name        | Function                                       | Type
----                | ----                                           | ----
motor_enable        | Motor enable                                   | grp_control_msg::srv::Void
motor_disable       | Motor disable                                  | grp_control_msg::srv::Void
modbus_slave_change | Change connected modbus slave                  | grp_control_msg::srv::SingleInt
set_modbus_addr     | Set new address number of DATC                 | grp_control_msg::srv::SingleInt
set_finger_pos      | Control the finger position                    | grp_control_msg::srv::SingleInt
set_motor_torque    | Set the maximum torque (position control mode) | grp_control_msg::srv::SingleInt
set_motor_speed     | Set the maximum speed (position control mode)  | grp_control_msg::srv::SingleInt
motor_stop          | Stop the DATC                                  | grp_control_msg::srv::Void
gripper_initialize  | Initialize gripper                             | grp_control_msg::srv::Void
grp_open            | Gripper open                                   | grp_control_msg::srv::Void
grp_close           | Gripper close                                  | grp_control_msg::srv::Void
vacuum_grp_on       | Vacuum gripper on                              | grp_control_msg::srv::Void
vacuum_grp_off      | Vacuum gripper off                             | grp_control_msg::srv::Void
motor_vel_ctrl      | Control the velocity of the motor              | grp_control_msg::srv::PosVelCurCtrl
motor_cur_ctrl      | Control the current of the motor               | grp_control_msg::srv::PosVelCurCtrl

**Structure of each service**
| Service Name        | Request Variables (data type)   | Value
| ----                | ----                    | ----
| motor_enable        | -                       | -
| motor_disable       | -                       | -
| modbus_slave_change | value (int16_t)         | 1 ~ 200
| set_modbus_addr     | value (int16_t)         | 1 ~ 200
| set_finger_pos      | value (int16_t)         | 0 ~ 1000 (0: closed & 1000: open)
| set_motor_torque    | value (int16_t)         | 50 ~ 100 (unit: %)
| set_motor_speed     | value (int16_t)         | 0 ~ 100 (unit: %)
| motor_stop          | -                       | -
| gripper_initialize  | -                       | -
| grp_open            | -                       | -
| grp_close           | -                       | -
| vacuum_grp_on       | -                       | -
| vacuum_grp_off      | -                       | -
| motor_vel_ctrl      | velocity (int16_t)      | 100 ~ 1000 or -1000 ~ -100 (unit: rpm)
|                     | ~~duration (uint16_t)~~ | ~~10 ~ 10000 (ms)~~
| motor_cur_ctrl      | current (int16_t)       | -1200 ~ 1200 (unit: mA)
|                     | ~~duration (uint16_t)~~ | ~~10 ~ 10000 (ms)~~

---
## Contact
E-mail: software@korasrobotics.com

Homepage: http://korasrobotics.com