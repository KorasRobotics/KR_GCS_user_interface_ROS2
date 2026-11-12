# KR_GCS_user_interface_ROS2

## Overview
- KR_GCS_user_interface_ROS2 is the software that includes a GUI that can run KR-MATC of KORAS Robotics. The user can receive the status of KR-MATC through the topic of ROS2, and command through the service of ROS2.
- The GUI is the same as KR_GCS_user_interface (Windows / Ubuntu) except for the TCP tab, which is replaced by the ROS2 topic and services below.

---
## Compatible Operating Systems
- Ubuntu 22.04 (Jammy Jellyfish)
    - ROS2 Humble build system

---
## Dependencies
- Boost v1.74
- libmodbus v3.1.6 or later
- Qt 5.15 (installed with ROS2 Humble desktop) or Qt 6
- Feather-icon
- Noto Sans

If ROS2 Humble was installed correctly, most dependencies would have been installed except libmodbus. To install libmodbus and the other dependencies, follow these steps.
```shell
sudo apt update
sudo apt install libmodbus-dev libboost-dev qtbase5-dev libqt5svg5
```

---
## Warning
- KR-MATC is a tooling system that changes and uses various grippers such as pneumatic grippers, 2-finger grippers, and 3-finger grippers. Therefore, there are many different precautions from using general grippers, so please read KORAS Robotics' KR-MATC user manual before using this software.
- Homepage: http://korasrobotics.com

---
## Build & Run
```shell
$ cd <path_to_ROS2_workspace>
$ colcon build
$ source install/setup.bash
$ ros2 run kr_gcs_ui kr_gcs_ui
```
- Select the serial port, baud rate and slave address in the Modbus tab and press Start. The topic is published while the MATC responds, and the services send commands while the Modbus RTU connection is open.
- To use the serial port without sudo, add the user to the dialout group and log in again.
```shell
$ sudo usermod -aG dialout $USER
```
- To run one KR_GCS_user_interface_ROS2 for each MATC, give each a namespace. The topic and services are then under the namespace (e.g. /matc1/grp_state).
```shell
$ ros2 run kr_gcs_ui kr_gcs_ui --ros-args -r __ns:=/matc1
```
- Closing the window or pressing Ctrl+C in the terminal stops the program. The motor is disabled before the Modbus connection is closed.

---
## Troubleshooting
- This section lists solutions to a set of possible errors which can happen when using the KR_GCS_user_interface_ROS2.
#### qt.qpa.plugin error
- If you have seen an error such as **"qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "~/platforms:" even though it was found."** when you run the released KR_GCS_user_interface_ROS2 on Ubuntu 22.04, you can resolve it in the following way.
```shell
$ sudo apt update
$ sudo apt install pyqt5-dev*
```
#### Icons are not displayed
- The icons are SVG files. Install the Qt SVG module.
```shell
$ sudo apt install libqt5svg5
```

---
## Communication Using ROS2 Topic and Service
- Node name: MATC_Control_Interface (with a namespace, e.g. /matc1/MATC_Control_Interface)

#### ROS2 Topic
- Topic name: /grp_state
- Type: grp_control_msg/msg/GripperMsg
- Frequency: about 50Hz (published only while the MATC responds)
- The first 12 variables are the same as in the earlier version of KR_GCS_user_interface_ROS2. The variables after them were added for KR-MATC v3.

| Variable Name          | Data Type | Value
| ----                   | ----      | ----
| motor_position         | int16_t   | Position of the motor (deg)
| motor_current          | int16_t   | Current of the motor (mA)
| motor_velocity         | int16_t   | Velocity of the motor (rpm)
| finger_position        | uint16_t  | Finger position of the MATC (0 ~ 1000 ‰ of the stroke (0: closed & 1000: open), also on MATC v3. In mm: finger_position × gripper_stroke_mm / 1000)
| motor_enabled          | boolean   | 0: False, 1: True
| gripper_initialized    | boolean   | 0: False, 1: True
| position_ctrl_mode     | boolean   | 0: False, 1: True
| velocity_ctrl_mode     | boolean   | 0: False, 1: True
| current_ctrl_mode      | boolean   | 0: False, 1: True
| grp_opened             | boolean   | 0: False, 1: True
| grp_closed             | boolean   | 0: False, 1: True
| motor_fault            | boolean   | 0: False, 1: True
| states                 | uint16_t  | Status of the MATC (see States of MATC)
| voltage                | uint16_t  | Voltage of the MATC (V)
| fault_now              | uint16_t  | Current STM motor fault code
| fault_occurred         | uint16_t  | Latched STM motor fault code
| gripper_direction      | boolean   | Gripper direction flag
| object_grasp           | boolean   | Object grasp detection flag
| grip_hold              | boolean   | Grip hold flag
| slip_detected          | boolean   | Slip detection flag (always false on MATC v3 — reserved bit)
| gobj_result            | uint8_t   | Grip result (MATC v3, states bit 12–13): 0 moving, 1 object detected while closing, 2 object detected while opening, 3 at requested position (no object). Always 0 on older firmware
| grip_force_n           | uint16_t  | Calculated grip force while gripping (N, MATC v3)
| comm_overrun           | boolean   | Communication overrun, latched until error_clear (MATC v3: states bit 14, older firmware: bit 12)
| eep_save_fail          | boolean   | EEPROM save fail (MATC v3, states bit 15), latched until error_clear or the next successful save
| gripper_connected      | boolean   | Gripper physically connected (MATC v3; always false on older firmware, which has no connection signal)
| gripper_id             | uint16_t  | Connected gripper ID (MATC v3; 128 = none)
| gripper_stroke_mm      | uint16_t  | Stroke of the connected gripper (mm, MATC v3)
| gripper_force_n        | uint16_t  | Maximum force of the connected gripper (N, MATC v3)
| gripper_payload_x10    | uint16_t  | Payload of the connected gripper (x0.1 kg, MATC v3)
| legacy_firmware        | boolean   | Older firmware (registers 0~19) is connected. set_finger_pos and set_motor_torque use the older units (see below)
| device_model           | uint16_t  | Connected controller model (10: MATC-10, 20: MATC-20 (GL60), 30: MATC-30 (GL80)), firmware v3.01~. 0 if not reported
| fw_version             | uint16_t  | Firmware version (301 = v3.01), firmware v3.01~. 0 if not reported
| max_speed_rpm          | uint16_t  | Limit applied to motor_vel_ctrl for the connected model (rpm)
| cur_full_scale_ma      | uint16_t  | Limit applied to motor_cur_ctrl for the connected model (mA)
| elangle_calibrated     | boolean   | Electrical angle is calibrated (firmware v3.01~)
| elangle_model_mismatch | boolean   | The stored electrical angle is from another model and was rejected (firmware v3.01~)

**States of MATC**
| Bit   | Status                 | Value
| ----  | ----                   | ----
| 0     | Motor Enable           | 0: False, 1: True
| 1     | Gripper Initialize     | 0: False, 1: True
| 2     | Motor Position Control | 0: False, 1: True
| 3     | Motor Velocity Control | 0: False, 1: True
| 4     | Motor Current Control  | 0: False, 1: True
| 5     | Gripper Open           | 0: False, 1: True
| 6     | Gripper Close          | 0: False, 1: True
| 7     | Gripper Direction      | 0: Normal, 1: Reverse
| 8     | Object Grasp           | 0: False, 1: True
| 9     | Motor Fault            | 0: False, 1: True
| 10    | Grip Hold              | 0: False, 1: True
| 11    | Slip Detected          | 0 (reserved on MATC v3)
| 12-13 | Grip Object Status     | 0: Moving, 1: Object (closing), 2: Object (opening), 3: At target (MATC v3). Older firmware: bit 12 = Communication Overrun
| 14    | Communication Overrun  | 0: False, 1: True (latched until Error Clear)
| 15    | EEPROM Save Fail       | 0: False, 1: True (latched until Error Clear or next successful save)

#### ROS2 Service
- Please refer to the KR-MATC user manual for a detailed description of each function.
- "successed" is true when the command was sent to the MATC. It does not mean that the motion has finished; check /grp_state for the result. It is false when the Modbus RTU connection is not open or the value cannot be sent.

**Description of each service**
Service Name         | Function                                       | Type
----                 | ----                                           | ----
motor_enable         | Motor enable                                   | grp_control_msg::srv::Void
motor_disable        | Motor disable                                  | grp_control_msg::srv::Void
modbus_slave_change  | Change connected modbus slave                  | grp_control_msg::srv::SingleInt
set_modbus_addr      | Set new address number of MATC                 | grp_control_msg::srv::SingleInt
set_finger_pos       | Control the finger position                    | grp_control_msg::srv::SingleInt
set_motor_torque     | Set the grip force (position control mode)     | grp_control_msg::srv::SingleInt
set_motor_speed      | Set the maximum speed (position control mode)  | grp_control_msg::srv::SingleInt
motor_stop           | Stop the MATC                                  | grp_control_msg::srv::Void
gripper_initialize   | Initialize gripper                             | grp_control_msg::srv::Void
grp_open             | Gripper open                                   | grp_control_msg::srv::Void
grp_close            | Gripper close                                  | grp_control_msg::srv::Void
vacuum_grp_on        | Vacuum gripper on                              | grp_control_msg::srv::Void
vacuum_grp_off       | Vacuum gripper off                             | grp_control_msg::srv::Void
motor_pos_ctrl       | Control the position of the motor              | grp_control_msg::srv::PosVelCurCtrl
motor_vel_ctrl       | Control the velocity of the motor              | grp_control_msg::srv::PosVelCurCtrl
motor_cur_ctrl       | Control the current of the motor               | grp_control_msg::srv::PosVelCurCtrl
error_clear          | Clear the error and the latched warnings       | grp_control_msg::srv::Void
impedance_on         | Impedance control on                           | grp_control_msg::srv::Void
impedance_off        | Impedance control off                          | grp_control_msg::srv::Void
set_impedance_params | Set the impedance parameters                   | grp_control_msg::srv::ImpedanceParams
write_gripper_map    | Write a gripper map entry (MATC v3)            | grp_control_msg::srv::WriteGripperMap
read_gripper_map     | Read a gripper map entry (MATC v3)             | grp_control_msg::srv::ReadGripperMap
save_gripper_map     | Save the gripper map to EEPROM (MATC v3)       | grp_control_msg::srv::Void
set_modbus_baudrate  | Change the baud rate of the MATC and reconnect (MATC v3) | grp_control_msg::srv::SetModbusBaudrate

**Structure of each service**
| Service Name         | Request Variables (data type)   | Value
| ----                 | ----                    | ----
| motor_enable         | -                       | -
| motor_disable        | -                       | -
| modbus_slave_change  | value (int16_t)         | 1 ~ 247
| set_modbus_addr      | value (int16_t)         | 1 ~ 247
| set_finger_pos       | value (int16_t)         | 0 ~ stroke (mm, 0: closed; MATC v3). Older firmware: 0 ~ 1000 (‰, 0: closed & 1000: open)
| set_motor_torque     | value (int16_t)         | Target grip force (1 ~ 1000 N, MATC v3). Older firmware: 25 ~ 100 (% of rated torque)
| set_motor_speed      | value (int16_t)         | 1 ~ 100 (unit: %)
| motor_stop           | -                       | -
| gripper_initialize   | -                       | -
| grp_open             | -                       | -
| grp_close            | -                       | -
| vacuum_grp_on        | -                       | -
| vacuum_grp_off       | -                       | -
| motor_pos_ctrl       | position (int16_t)      | Target position (unit: deg)
|                      | duration (uint16_t)     | 10 ~ 10000 (ms)
| motor_vel_ctrl       | velocity (int16_t)      | 100 ~ max_speed_rpm or -max_speed_rpm ~ -100 (unit: rpm)
|                      | ~~duration (uint16_t)~~ | ~~10 ~ 10000 (ms)~~
| motor_cur_ctrl       | current (int16_t)       | -cur_full_scale_ma ~ cur_full_scale_ma (unit: mA)
|                      | ~~duration (uint16_t)~~ | ~~10 ~ 10000 (ms)~~
| error_clear          | -                       | -
| impedance_on         | -                       | - (Gripper Initialize is sent 0.1 s later, as with the button in the GUI)
| impedance_off        | -                       | - (Gripper Initialize is sent 0.1 s later, as with the button in the GUI)
| set_impedance_params | slave_num (int16_t)     | Slave number (1 ~ 247)
|                      | stiffness_level (int16_t) | 1 ~ 10
| write_gripper_map    | index (uint16_t)        | 0 ~ 127
|                      | stroke_mm (uint16_t)    | Stroke (mm)
|                      | force_n (uint16_t)      | Force (N)
|                      | payload_x10 (uint16_t)  | Payload (x0.1 kg)
| read_gripper_map     | index (uint16_t)        | 0 ~ 127. The response has index, stroke_mm, force_n and payload_x10 of the entry
| save_gripper_map     | -                       | - (returns after about 1.5 s; false if the EEPROM save failed (states bit 15))
| set_modbus_baudrate  | baudrate (int32_t)      | 9600, 19200, 38400, 57600, 115200 (bps). The response has baudrate (the baud rate KR_GCS_user_interface_ROS2 is connected at after the call) and result (see below)

- Values out of range are limited to the range before they are sent, except the slave addresses (modbus_slave_change, set_modbus_addr), the map index and the baud rate, which return false.
- write_gripper_map, read_gripper_map, save_gripper_map and set_modbus_baudrate exist only on MATC v3. While older firmware is connected, KR_GCS_user_interface_ROS2 does not send them and returns false.
- set_modbus_baudrate saves the new baud rate in the MATC (EEPROM) and reconnects KR_GCS_user_interface_ROS2 at it. This takes about 2 seconds; the topic pauses and other service calls wait in the meantime. If the MATC does not answer at the new baud rate, KR_GCS_user_interface_ROS2 returns to the previous one. Other masters on the same bus (PLC etc.) must also use the new baud rate.
    - result: "changed", "unchanged" (already at that baud rate), "reverted" (the MATC kept the previous baud rate), "no_response" (no answer at either baud rate) or "not_sent" (not connected, invalid value, older firmware, or another reconnection in progress). If there is no result within 30 s, result is "timeout". successed is true for "changed" and "unchanged".

#### Communication test using ROS2 CLI
```shell
# Status
$ ros2 topic echo /grp_state
# Gripper initialize
$ ros2 service call /gripper_initialize grp_control_msg/srv/Void
# Set finger position (MATC v3: 40 mm)
$ ros2 service call /set_finger_pos grp_control_msg/srv/SingleInt "{value: 40}"
# Read gripper map entry 6
$ ros2 service call /read_gripper_map grp_control_msg/srv/ReadGripperMap "{index: 6}"
# Change the baud rate of the MATC to 57600 bps (KR_GCS_user_interface_ROS2 reconnects at 57600 bps)
$ ros2 service call /set_modbus_baudrate grp_control_msg/srv/SetModbusBaudrate "{baudrate: 57600}"
```

---
## Contact
E-mail: software@korasrobotics.com

Homepage: http://korasrobotics.com