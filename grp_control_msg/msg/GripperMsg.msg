int16 motor_position
int16 motor_current
int16 motor_velocity
uint16 finger_position

bool motor_enabled
bool gripper_initialized
bool position_ctrl_mode
bool velocity_ctrl_mode
bool current_ctrl_mode
bool grp_opened
bool grp_closed
bool motor_fault

# KR-MATC v3: fields added in this version (the fields above are unchanged from the earlier version)
uint16 states                 # status word (register 10)
uint16 voltage                # supply voltage (V)
uint16 fault_now              # current motor fault code
uint16 fault_occurred         # latched motor fault code
bool gripper_direction        # status bit 7
bool object_grasp             # status bit 8
bool grip_hold                # status bit 10
bool slip_detected            # status bit 11 (always false on MATC v3)
uint8 gobj_result             # MATC v3, status bit 12-13: 0 moving, 1 object (closing), 2 object (opening), 3 at target
uint16 grip_force_n           # MATC v3: calculated grip force while gripping (N)
bool comm_overrun             # latched until error_clear (MATC v3: bit 14, older firmware: bit 12)
bool eep_save_fail            # MATC v3, bit 15: latched until error_clear or the next successful save
bool gripper_connected        # MATC v3: gripper physically connected
uint16 gripper_id             # MATC v3: connected gripper ID (128 = none)
uint16 gripper_stroke_mm      # MATC v3: stroke of the connected gripper (mm)
uint16 gripper_force_n        # MATC v3: maximum force of the connected gripper (N)
uint16 gripper_payload_x10    # MATC v3: payload of the connected gripper (x0.1 kg)
bool legacy_firmware          # older firmware (registers 0~19): set_finger_pos in permille, set_motor_torque in %
uint16 device_model           # 10: MATC-10, 20: MATC-20 (GL60), 30: MATC-30 (GL80), 0: not reported
uint16 fw_version             # 301 = v3.01, 0: not reported
uint16 max_speed_rpm          # limit applied to motor_vel_ctrl (rpm)
uint16 cur_full_scale_ma      # limit applied to motor_cur_ctrl (mA)
bool elangle_calibrated       # firmware v3.01~: electrical angle calibrated
bool elangle_model_mismatch   # firmware v3.01~: stored electrical angle is from another model and was rejected