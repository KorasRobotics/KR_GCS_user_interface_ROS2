/**
 * @file datc_ctrl.cpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief
 * @version 1.0
 * @date 2023-11-03
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "datc_ctrl.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

// 조회 명령(217/221) 응답 대기 — queryCommand 참고
const int kQueryPollMs         = 20;     // reg 24~27 재읽기 간격
const int kQueryTimeoutMs      = 1000;   // 응답 대기 상한
const int kQuerySameEchoWaitMs = 100;    // 직전 응답이 같은 번호였을 때 최소 대기

DatcCtrl::DatcCtrl() {
}

DatcCtrl::~DatcCtrl() {
}

bool DatcCtrl::modbusInit(const char *port_name, uint16_t slave_address, int baudrate) {
    if (slave_address < kModbusSlaveAddrMin || slave_address > kModbusSlaveAddrMax) {
        COUT("\"modbusInit\" function error. Check the input slave address.");
        return false;
    }

    resetDeviceInfo();
    return mbc_.modbusInit(port_name, slave_address, baudrate);
}

bool DatcCtrl::modbusRelease() {
    mbc_.modbusRelease();
    resetDeviceInfo();
    return true;
}

bool DatcCtrl::modbusSlaveChange(uint16_t slave_addr) {
    if (slave_addr < kModbusSlaveAddrMin || slave_addr > kModbusSlaveAddrMax) {
        COUT("\"modbusSlaveChange\" function error. Check the input slave address.");
        return false;
    }

    resetDeviceInfo();
    return mbc_.slaveChange(slave_addr);
}

bool DatcCtrl::motorEnable() {
    return command(DATC_COMMAND::MOTOR_ENABLE);
}

bool DatcCtrl::motorStop() {
    return command(DATC_COMMAND::MOTOR_STOP);
}

bool DatcCtrl::motorEmergencyStop() {
    return command(DATC_COMMAND::MOTOR_EMERGENCY_STOP);
}

bool DatcCtrl::motorDisable() {
    return command(DATC_COMMAND::MOTOR_DISABLE);
}

bool DatcCtrl::resetPosition() {
    return command(DATC_COMMAND::RESET_POSITION);
}

bool DatcCtrl::errorClear() {
    return command(DATC_COMMAND::ERROR_CLEAR);
}

bool DatcCtrl::systemReset() {
    return command(DATC_COMMAND::SYSTEM_RESET);
}

bool DatcCtrl::setModbusAddr(uint16_t slave_addr) {
    if (slave_addr < kModbusSlaveAddrMin || slave_addr > kModbusSlaveAddrMax) {
        COUT("\"setModbusAddr\" function error. Check the input slave address.");
        return false;
    }

    return command(DATC_COMMAND::CHANGE_MODBUS_ADDRESS, slave_addr);
}

bool DatcCtrl::changeElectricalAngle(int16_t elec_angle_deg) {
    return command(DATC_COMMAND::CHANGE_ELECTRICAL_ANGLE, (uint16_t) elec_angle_deg);
}

bool DatcCtrl::setModbusBaudrate(int baudrate) {
    switch (baudrate) {
        case 9600: case 19200: case 38400: case 57600: case 115200:
            break;
        default:
            COUT("\"setModbusBaudrate\" function error. Use 9600/19200/38400/57600/115200.");
            return false;
    }
    return command(DATC_COMMAND::CHANGE_MODBUS_BAUDRATE, (uint16_t) (baudrate / 100));
}

bool DatcCtrl::probe() {
    vector<uint16_t> reg;
    return mbc_.recvData(kDatcStatusRegAddr, 1, reg);
}

bool DatcCtrl::grpInitialize() {
    return command(DATC_COMMAND::GRIPPER_INITIALIZE);
}

bool DatcCtrl::grpInitialize2(int16_t offset_deg) {
    return command(DATC_COMMAND::GRIPPER_INITIALIZE2, (uint16_t) offset_deg);
}

bool DatcCtrl::grpOpen() {
    return command(DATC_COMMAND::GRIPPER_OPEN);
}

bool DatcCtrl::grpClose() {
    return command(DATC_COMMAND::GRIPPER_CLOSE);
}

bool DatcCtrl::grpSetDirection(bool reverse_direction) {
    return command(DATC_COMMAND::GRIPPER_SET_DIRECTION, reverse_direction ? 1 : 0);
}

bool DatcCtrl::setFingerPos(uint16_t finger_pos) {
    string error_prefix = "[Set Finger Position]";

    if (finger_pos < kFingerPosMin) {
        printf("[Error] %s Invalid range of finger position ( < %d)\n", error_prefix.c_str(), kFingerPosMin);
        finger_pos = kFingerPosMin;
    } else if (finger_pos > kFingerPosMax) {
        printf("[Error] %s Invalid range of finger position ( > %d)\n", error_prefix.c_str(), kFingerPosMax);
        finger_pos = kFingerPosMax;
    }

    return command(DATC_COMMAND::SET_FINGER_POSITION, finger_pos);
}

bool DatcCtrl::motorVelCtrl(int16_t vel) {
    string error_prefix = "[Motor Velocity Control]";

    // 모델별 상한(장치 보고값). 상한을 넘는 지령은 펌웨어(MCSDK)가 거부해 무반응이 되므로 여기서 클램프한다.
    const int vel_max = getVelMaxRpm();
    const int vel_min = std::min<int>(kVelMin, vel_max);

    if (abs(vel) < vel_min) {
        printf("[Error] %s Invalid range of speed ( < %d)\n", error_prefix.c_str(), vel_min);
        vel = (vel >= 0) ? vel_min : -vel_min;
    } else if (abs(vel) > vel_max) {
        printf("[Error] %s Invalid range of speed ( > %d)\n", error_prefix.c_str(), vel_max);
        vel = (vel >= 0) ? vel_max : -vel_max;
    }

    return command(DATC_COMMAND::MOTOR_VELOCITY_CONTROL, vel, 500); // duration no longer works.
}

bool DatcCtrl::motorCurCtrl(int16_t cur) {
    string error_prefix = "[Motor Current Control]";

    const int cur_max = getCurMaxMa();   // 모델별 정격(장치 보고값)

    if (abs(cur) > cur_max) {
        printf("[Error] %s Invalid range of current ( > %d)\n", error_prefix.c_str(), cur_max);
        cur = (cur >= 0) ? cur_max : -cur_max;
    }

    return command(DATC_COMMAND::MOTOR_CURRENT_CONTROL, cur, 500); // duration no longer works.
}

bool DatcCtrl::motorPosCtrl(int16_t pos_deg, uint16_t duration) {
    string error_prefix = "[Motor Position Control]";
    checkDurationRange(error_prefix, duration);
    return command(DATC_COMMAND::MOTOR_POSITION_CONTROL, pos_deg, duration);
}

bool DatcCtrl::vacuumGrpOn() {
    return command(DATC_COMMAND::VACUUM_GRIPPER_ON);
}

bool DatcCtrl::vacuumGrpOff() {
    return command(DATC_COMMAND::VACUUM_GRIPPER_OFF);
}

bool DatcCtrl::setMotorTorque(uint16_t force_n) {
    // 그립 파지력 지령 [N]. 펌웨어(MB_SET_TORQUE)가 그리퍼 종류별로 N→전류 환산한다.
    // (이전: 모터 토크 백분율 50~100 %) TV1로 그대로 전송, 스케일링 없음.
    string error_prefix = "[Set Grip Force]";

    if (force_n < kGripForceMin) {
        printf("[Error] %s Grip force is too low ( < %d N)\n", error_prefix.c_str(), kGripForceMin);
        force_n = kGripForceMin;
    } else if (force_n > kGripForceMax) {
        printf("[Error] %s Grip force is too high ( > %d N)\n", error_prefix.c_str(), kGripForceMax);
        force_n = kGripForceMax;
    }

    return command(DATC_COMMAND::SET_MOTOR_TORQUE, force_n);
}

bool DatcCtrl::setMotorSpeed (uint16_t speed_ratio) {
    string error_prefix = "[Set Motor Speed]";

    if (speed_ratio < kSpeedRatioMin) {
        printf("[Error] %s Motor speed is too low ( < %d)\n", error_prefix.c_str(), kSpeedRatioMin);
        speed_ratio = kSpeedRatioMin;
    } else if (speed_ratio > kSpeedRatioMax) {
        printf("[Error] %s Motor speed is too high ( > %d)\n", error_prefix.c_str(), kSpeedRatioMax);
        speed_ratio = kSpeedRatioMax;
    }

    return command(DATC_COMMAND::SET_MOTOR_SPEED, speed_ratio);
}

bool DatcCtrl::setMaxValue(int16_t max_vel, int16_t max_acc) {
    return command(DATC_COMMAND::SET_MAX_VALUE, (uint16_t) max_vel, (uint16_t) max_acc);
}

bool DatcCtrl::setPositionPidGain(int16_t kp, int16_t ki, int16_t kd) {
    return command(DATC_COMMAND::SET_POSITION_PID_GAIN, (uint16_t) kp, (uint16_t) ki, (uint16_t) kd);
}

bool DatcCtrl::setCurrentPiGain(int16_t kp, int16_t ki, int16_t kd) {
    return command(DATC_COMMAND::SET_CURRENT_PI_GAIN, (uint16_t) kp, (uint16_t) ki, (uint16_t) kd);
}

bool DatcCtrl::setVelocityPiGain(int16_t kp, int16_t ki, int16_t kd) {
    return command(DATC_COMMAND::SET_VELOCITY_PI_GAIN, (uint16_t) kp, (uint16_t) ki, (uint16_t) kd);
}

bool DatcCtrl::readDatcData() {
    static map<uint16_t, pair<bool*, string>> status_info;

    if (status_info.size() == 0) {
        // Bit → bool field, display string
        status_info.insert({0,  make_pair(&status_.enable,            "Motor Enable")});
        status_info.insert({1,  make_pair(&status_.initialize,        "Gripper Initialize")});
        status_info.insert({2,  make_pair(&status_.motor_pos_ctrl,    "Motor Position Control")});
        status_info.insert({3,  make_pair(&status_.motor_vel_ctrl,    "Motor Velocity Control")});
        status_info.insert({4,  make_pair(&status_.motor_cur_ctrl,    "Motor Current Control")});
        status_info.insert({5,  make_pair(&status_.grp_open,          "Gripper Open")});
        status_info.insert({6,  make_pair(&status_.grp_close,         "Gripper Close")});
        status_info.insert({7,  make_pair(&status_.gripper_direction,  "Gripper Direction Reversed")});
        status_info.insert({8,  make_pair(&status_.object_grasp,       "Object Grasp")});
        status_info.insert({9,  make_pair(&status_.fault,              "Motor Fault")});
        status_info.insert({10, make_pair(&status_.grip_hold,          "Grip Hold")});
        status_info.insert({11, make_pair(&status_.slip_detected,      "Slip Detected")});
    }

    // Single read: regs 10–23
    //   [0-7]  : motor/gripper status (regs 10–17)
    //   [8-9]  : reserved             (regs 18–19)
    //   [10-13]: gripper ID & specs   (regs 20–23, MATC v3)
    //
    // 적응형 읽기 (견고화):
    //   - MATC v3 펌웨어(REG_HOLDING_NREGS=28)는 reg 20-23(그리퍼 정보)까지 존재 → 14개 읽기
    //   - 구버전 펌웨어(status만 존재)는 14개 읽기 시 "Illegal Data Address"(예외 02) 발생
    //
    // 단순 1회 실패로 영구히 8개로 latch되면, 일시적 통신 오류(CRC/타임아웃) 때문에
    // 그리퍼 정보가 영영 안 보이는 문제가 생긴다. 따라서:
    //   1) 14개 읽기가 "연속 3회" 실패해야 8개 모드로 전환한다 (일시 오류 무시).
    //   2) 8개 모드로 폴백한 뒤에도 주기적으로(약 2초마다) 14개 읽기를 재시도하여,
    //      펌웨어/장치가 교체되면 자동으로 그리퍼 정보 읽기를 복구한다.
    //   3) 14개 읽기가 Illegal Data Address(예외 02)로 연속 3회 거부되면 구버전 펌웨어(레지스터 0~19)로 본다
    //      (status_.legacy_map → GUI가 위치·파지력 단위를 바꾼다). 시간 초과·CRC 같은 일시 오류로는 판정하지 않는다.
    //   4) 14개가 읽혀도 reg 20이 그리퍼 ID 범위(0~128)를 벗어나면 구버전 펌웨어로 본다. 레지스터를 20번 뒤로
    //      늘린 구버전 진단판(reg 20 = 0xD1A6 표식)이 이 경우다.
    const  int      kFullFailThreshold   = 3;                // 연속 3회 실패 시 폴백
    const  int      kFallbackRetryPeriod = 100;              // ~2초(50Hz) 마다 14개 재시도

    if (read_mode_reset_.exchange(false)) {   // 연결·슬레이브가 바뀜 → 처음부터 다시 판정
        read_reg_num_       = kDatcFullReadNum;
        full_read_fail_cnt_ = 0;
        illegal_addr_cnt_   = 0;
        fallback_retry_cnt_ = 0;
        status_.legacy_map  = false;
    }

    // 8개 모드일 때 주기적으로 14개 읽기 복구를 시도
    if (read_reg_num_ == kDatcStatusRegNum) {
        if (++fallback_retry_cnt_ >= kFallbackRetryPeriod) {
            fallback_retry_cnt_ = 0;
            read_reg_num_       = kDatcFullReadNum;  // 14개로 복귀 시도
            full_read_fail_cnt_ = 0;
        }
    }

    vector<uint16_t> reg;
    int  err = 0;
    bool ok  = mbc_.recvData(kDatcStatusRegAddr, read_reg_num_, reg, &err);

    if (!ok && read_reg_num_ == kDatcFullReadNum) {
        // 14개 읽기 실패 — 연속 실패 횟수 누적 (주소 거부는 따로 센다)
        illegal_addr_cnt_ = (err == EMBXILADD) ? illegal_addr_cnt_ + 1 : 0;
        if (illegal_addr_cnt_ >= kFullFailThreshold && !status_.legacy_map) {
            status_.legacy_map = true;
            COUT("[readDatcData] Registers 18-23 rejected (illegal data address). "
                 "Legacy firmware: finger position 0-1000 permille, grip force 50-100 %.");
        }
        if (++full_read_fail_cnt_ >= kFullFailThreshold) {
            read_reg_num_ = kDatcStatusRegNum;  // 8 (status only)로 폴백
            COUT("[readDatcData] Full read (14 regs) failed 3x. "
                 "Falling back to status-only (8 regs). Gripper info unavailable.");
        }
        // 폴백 직후 이번 사이클은 8개로 즉시 재시도
        ok = mbc_.recvData(kDatcStatusRegAddr, kDatcStatusRegNum, reg);
    } else if (ok && read_reg_num_ == kDatcFullReadNum) {
        full_read_fail_cnt_ = 0;  // 성공 시 실패 카운터 리셋
        illegal_addr_cnt_   = 0;
        // V3의 reg 20은 그리퍼 ID(0~127, 128 = 없음)다. 그보다 크면 V3 배치가 아니다 — 구버전 진단판은
        // 20~41번에 수신 명령 표·거부 카운터를 두고 reg 20에 0xD1A6을 쓴다.
        const bool foreign_map = (reg[10] > 128);
        if (foreign_map && !status_.legacy_map) {
            printf("[readDatcData] Register 20 = 0x%04X is not a gripper ID — legacy firmware with extra registers%s. "
                   "Finger position 0-1000 permille, grip force 50-100 %%.\n",
                   reg[10], (reg[10] == 0xD1A6) ? " (field diagnostic map)" : "");
        }
        status_.legacy_map = foreign_map;   // false: reg 18~23이 있는 V3 펌웨어
    }

    if (ok) {
        // --- Status registers (10–17) ---
        uint16_t status    = reg[0];
        status_.states     = status;
        status_.motor_pos  = (int16_t) reg[1];
        status_.motor_cur  = (int16_t) reg[2];
        status_.motor_vel  = (int16_t) reg[3];
        status_.finger_pos = reg[4];
        status_.fault_now      = reg[5];
        status_.fault_occurred = reg[6];
        status_.voltage        = reg[7];

        // 구버전 펌웨어의 18~23번은 V3와 뜻이 다르다(진단판은 14개가 읽혀도 명령 카운터·진단 표) — 쓰지 않는다
        const bool v3_map = !status_.legacy_map;

        // --- Gripper connection (register 18) — MATC v3 ---
        // CONNECT_CHECK(PG0): 1 = 그리퍼 물리적 연결, 0 = 미연결
        // reg[8] = 홀딩 reg 18. 14개 읽기일 때만 존재(구버전 8개 읽기엔 없음).
        if (v3_map && reg.size() >= 9) {
            status_.gripper_connected = (reg[8] != 0);
        } else {
            status_.gripper_connected = false;  // 구버전 펌웨어: 연결 감지 미지원
        }
        // reg[9] = 홀딩 reg 19: 파지 중 계산 파지력 [N] (MATC v3, 14워드 읽기일 때만 존재)
        status_.grip_force_calc_n = (v3_map && reg.size() >= 10) ? reg[9] : 0;

        // --- Gripper info registers (20–23) ---
        // MATC v3 펌웨어에서만 지원. 구버전(현재 KUGRP_Firmware 포함)에서는 기본값 유지.
        if (v3_map && reg.size() >= 14) {
            status_.gripper_id          = reg[10];
            status_.gripper_stroke_mm   = reg[11];
            status_.gripper_force_n     = reg[12];
            status_.gripper_payload_x10 = reg[13];
        } else {
            // 구버전 펌웨어: gripper info 미지원. 기본값(128 = no gripper) 유지.
            status_.gripper_id          = 128;
            status_.gripper_stroke_mm   = 0;
            status_.gripper_force_n     = 0;
            status_.gripper_payload_x10 = 0;
        }

        // --- Parse status bits ---
        status_.status_str = "---";
        status_.warn_str.clear();

        for (int i = 0; i < 16; i++) {
            if (status_info.find(i) != status_info.end()) {
                if (status & (0x01 << i)) {
                    *status_info[i].first = true;
                    status_.status_str = status_info[i].second;
                } else {
                    *status_info[i].first = false;
                }
            }
        }

        if (v3_map) {
            // gObj 파지 결과 (bit 12-13, MATC v3): 0=이동중 1=물체감지(닫는중) 2=물체감지(여는중) 3=목표위치 도달
            status_.gobj_result = (uint8_t) ((status >> 12) & 0x03);

            // 래치 경고 (bit 14-15, MATC v3): 에러 클리어(9) 전까지 유지된다
            status_.comm_overrun  = (status & (0x01 << 14)) != 0;
            status_.eep_save_fail = (status & (0x01 << 15)) != 0;
        } else {
            // 구버전 펌웨어(KUGRP·대동·진단판)는 bit 12가 통신 오버런(ORE) 래치다(에러 클리어로 해제).
            // 파지 결과(V3 bit 12-13)와 EEPROM 저장 실패(bit 15)는 없다.
            status_.gobj_result   = 0;
            status_.comm_overrun  = (status & (0x01 << 12)) != 0;
            status_.eep_save_fail = false;
        }

        if (status_.fault) {
            status_.status_str = "Motor Fault";
        } else if (status_.gobj_result == 1) {
            status_.status_str = "Object Detected (Closing)";
        } else if (status_.gobj_result == 2) {
            status_.status_str = "Object Detected (Opening)";
        } else if (status_.slip_detected) {
            status_.status_str = "Slip Detected";
        } else if (status_.grip_hold) {
            status_.status_str = "Grip Hold";
        } else if (status_.object_grasp) {
            status_.status_str = "Object Grasp";
        } else if (!status_.enable) {
            status_.status_str = "Motor Disabled";
        } else if (status_.gobj_result == 3) {
            status_.status_str = "At Requested Position";
        } else if (status_.grp_close) {
            status_.status_str = "Gripper Close";
        } else if (status_.grp_open) {
            status_.status_str = "Gripper Open";
        } else if (status_.motor_cur_ctrl) {
            status_.status_str = "Motor Current Control";
        } else if (status_.motor_vel_ctrl) {
            status_.status_str = "Motor Velocity Control";
        } else if (status_.motor_pos_ctrl) {
            status_.status_str = "Motor Position Control";
        } else if (status_.initialize) {
            status_.status_str = "Gripper Initialize";
        } else if (status_.enable) {
            status_.status_str = "Motor Enable";
        }

        // 경고는 동작 상태(status_str)와 따로 모은다 — GCS는 Mode 아래 둘째 줄에 표시
        auto addWarn = [this](const char* warn) {
            if (!status_.warn_str.empty()) status_.warn_str += " | ";
            status_.warn_str += warn;
        };

        // 장치 정보(regs 28–33, v3.01~): 연결 직후 읽고 이후 약 1초마다 플래그 갱신
        pollDeviceInfo();
        {
            const DeviceInfo info = getDeviceInfo();
            // 전기각 오프셋이 없으면(또는 다른 모델 것이라 거부되면) Enable이 거부된다 — 원인을 보여준다
            if (info.valid && info.elangleModelMismatch()) {
                addWarn("Elec. Angle: Other Model (Recalibrate)");
            } else if (info.valid && !info.elangleCalibrated()) {
                addWarn("Elec. Angle Not Calibrated");
            }
        }

        // 래치 경고 (Error Clear로 해제)
        if (status_.eep_save_fail) {
            addWarn("EEPROM Save Fail");
        }
        if (status_.comm_overrun) {
            addWarn("Comm Overrun");
        }

        flag_modbus_recv_err_ = false;
        return true;
    } else {
        flag_modbus_recv_err_ = true;
        return false;
    }
}

bool DatcCtrl::checkDurationRange(string error_prefix, uint16_t &duration) {
    if (duration < kDurationMin) {
        printf("[Error] %s Duration is too short ( < %dms)\n", error_prefix.c_str(), kDurationMin);
        duration = kDurationMin;
        return false;
    } else if (duration > kDurationMax) {
        printf("[Error] %s Duration is too long ( > %dms)\n", error_prefix.c_str(), kDurationMax);
        duration = kDurationMax;
        return false;
    }

    return true;
}

bool DatcCtrl::rejectOnLegacy(uint16_t cmd) {
    if (status_.legacy_map && isV3OnlyCommand(cmd)) {
        COUT("[Command " << cmd << "] Not supported by older firmware (registers 0-19). Not sent.");
        return true;
    }
    return false;
}

bool DatcCtrl::command(DATC_COMMAND cmd, uint16_t value_1, uint16_t value_2, uint16_t value_3) {
    if (rejectOnLegacy((uint16_t) cmd)) {
        return false;
    }
    switch (cmd) {
        case DATC_COMMAND::MOTOR_ENABLE:
            return SEND_CMD(cmd);

        case DATC_COMMAND::MOTOR_STOP:
            return SEND_CMD(cmd);

        case DATC_COMMAND::MOTOR_EMERGENCY_STOP:
            return SEND_CMD(cmd);

        case DATC_COMMAND::MOTOR_DISABLE:
            return SEND_CMD(cmd);

        case DATC_COMMAND::RESET_POSITION:
            return SEND_CMD(cmd);

        case DATC_COMMAND::ERROR_CLEAR:
            return SEND_CMD(cmd);

        case DATC_COMMAND::SYSTEM_RESET:
            return SEND_CMD(cmd);

        case DATC_COMMAND::MOTOR_POSITION_CONTROL:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1, value_2}));

        case DATC_COMMAND::MOTOR_VELOCITY_CONTROL:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1, value_2}));

        case DATC_COMMAND::MOTOR_CURRENT_CONTROL:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1, value_2}));

        case DATC_COMMAND::CHANGE_MODBUS_ADDRESS:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1}));

        case DATC_COMMAND::CHANGE_MODBUS_BAUDRATE:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1}));

        case DATC_COMMAND::CHANGE_ELECTRICAL_ANGLE:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1}));

        case DATC_COMMAND::GRIPPER_SET_DIRECTION:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1}));

        case DATC_COMMAND::GRIPPER_INITIALIZE:
            return SEND_CMD(cmd);

        case DATC_COMMAND::GRIPPER_INITIALIZE2:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1}));

        case DATC_COMMAND::GRIPPER_OPEN:
            return SEND_CMD(cmd);

        case DATC_COMMAND::GRIPPER_CLOSE:
            return SEND_CMD(cmd);

        case DATC_COMMAND::SET_FINGER_POSITION:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1}));

        case DATC_COMMAND::VACUUM_GRIPPER_ON:
            return SEND_CMD(cmd);

        case DATC_COMMAND::VACUUM_GRIPPER_OFF:
            return SEND_CMD(cmd);

        case DATC_COMMAND::IMPEDANCE_ON:
            return SEND_CMD(cmd);

        case DATC_COMMAND::IMPEDANCE_OFF:
            return SEND_CMD(cmd);

        case DATC_COMMAND::SET_IMPEDANCE_PARAMS:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1, value_2}));

        case DATC_COMMAND::SET_IMPEDANCE_MBK:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1, value_2, value_3}));

        case DATC_COMMAND::SET_MAX_VALUE:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1, value_2}));

        case DATC_COMMAND::SET_POSITION_PID_GAIN:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1, value_2, value_3}));

        case DATC_COMMAND::SET_MOTOR_TORQUE:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1}));

        case DATC_COMMAND::SET_MOTOR_SPEED:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1}));

        case DATC_COMMAND::SET_CURRENT_PI_GAIN:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1, value_2, value_3}));

        case DATC_COMMAND::SET_VELOCITY_PI_GAIN:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1, value_2, value_3}));

        // Gripper mapping table (MATC v3)
        case DATC_COMMAND::WRITE_MAP_ENTRY:
            // payload_x10은 reg 4에 실리므로 writeMapEntry를 사용할 것 (여기서는 reg 1~3만 전송)
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1, value_2, value_3}));

        case DATC_COMMAND::READ_MAP_ENTRY:
            // value_1=index; result is placed in regs 24-27 by firmware
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1}));

        case DATC_COMMAND::SAVE_MAP:
            return SEND_CMD(cmd);

        // Encoder harmonic compensation (MATC v3)
        case DATC_COMMAND::SET_ENC_HARM:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1, value_2, value_3}));

        case DATC_COMMAND::SAVE_ENC_HARM:
            return SEND_CMD(cmd);

        case DATC_COMMAND::READ_ENC_HARM:
            // value_1=k; result is placed in regs 24-27 by firmware
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1}));

        // Encoder harmonic auto calibration (MATC v3.02~)
        case DATC_COMMAND::AUTO_ENC_HARM:
            // value_1 = 1 시작, 0 취소 — reg 1을 항상 함께 써야 이전 명령의 값이 남아 취소가 시작으로 읽히지 않는다
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1}));

        case DATC_COMMAND::READ_AUTO_ENC_HARM:
            // value_1=k; result is placed in regs 24-27 by firmware
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1}));

        // Integrated gripper fixed ID (MATC v3.04~)
        case DATC_COMMAND::SET_FIXED_GRIPPER_ID:
            // value_1 = 0~126 저장, 128 = 해제 — 결과는 reg 34(저장된 값)·상태 bit15(저장 실패)
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1}));

        default:
            COUT("Error: Undefined command.");
            return false;
    }
}

// Dev ui related functions
bool DatcCtrl::customCmd(uint16_t cmd, uint16_t value_1, uint16_t value_2, uint16_t value_3) {
    return SEND_CMD_VECTOR(vector<uint16_t> ({cmd, value_1, value_2, value_3}));
}

// Gripper mapping table — MATC v3
bool DatcCtrl::writeMapEntry(uint16_t index, uint16_t stroke_mm,
                              uint16_t force_n, uint16_t payload_x10) {
    if (index > 127) {
        COUT("[writeMapEntry] index must be 0–127");
        return false;
    }
    if (rejectOnLegacy((uint16_t) DATC_COMMAND::WRITE_MAP_ENTRY)) {
        return false;
    }
    // 16-bit protocol: reg1=index, reg2=stroke_mm, reg3=force_n, reg4=payload_x10.
    // (구 8비트 패킹 폐기 — 대형 그리퍼 600N/30kg 지원). 펌웨어 MB_WRITE_MAP_ENTRY가 reg4 직접 읽음.
    return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) DATC_COMMAND::WRITE_MAP_ENTRY,
                                              index, stroke_mm, force_n, payload_x10}));
}

bool DatcCtrl::readMapEntry(uint16_t index) {
    if (index > 127) {
        COUT("[readMapEntry] index must be 0–127");
        return false;
    }
    return command(DATC_COMMAND::READ_MAP_ENTRY, index);
}

bool DatcCtrl::queryMapEntry(uint16_t index, GripperMapEntry &entry) {
    if (index > 127) {
        COUT("[queryMapEntry] index must be 0–127");
        return false;
    }
    vector<uint16_t> resp;
    if (!queryCommand(DATC_COMMAND::READ_MAP_ENTRY, index, resp)) {
        return false;
    }
    entry.index       = resp[0];
    entry.stroke_mm   = resp[1];
    entry.force_n     = resp[2];
    entry.payload_x10 = resp[3];
    return true;
}

bool DatcCtrl::saveGripperMap() {
    return command(DATC_COMMAND::SAVE_MAP);
}

bool DatcCtrl::readStatusWord(uint16_t &status) {
    vector<uint16_t> reg;
    if (!mbc_.recvData(kDatcStatusRegAddr, 1, reg) || reg.empty()) {
        return false;
    }
    status = reg[0];
    return true;
}

// --- 장치 정보 (regs 28–33, MATC v3.01~) ---
bool DatcCtrl::readDeviceInfoNow(DeviceInfo &info) {
    if (status_.legacy_map) {
        return false;   // 구버전 펌웨어: 28번 이후가 읽혀도(진단판) 장치 정보가 아니다
    }
    vector<uint16_t> reg;
    if (!mbc_.recvData(kDeviceInfoRegAddr, kDeviceInfoRegNum, reg) || reg.size() < kDeviceInfoRegNum) {
        return false;
    }
    info = DeviceInfo();
    info.valid             = true;
    info.model             = reg[0];
    info.fw_version        = reg[1];
    info.max_speed_rpm     = reg[2];
    info.cur_full_scale_ma = reg[3];
    info.flags             = reg[4];
    info.auto_cal          = reg[5];

    // reg 34: 저장된 일체형 그리퍼 고정 ID (v3.04~). 이전 펌웨어에는 이 주소가 없어 읽지 않는다.
    if (info.fw_version >= kFixedGripperIdMinFw) {
        vector<uint16_t> fixed;
        if (mbc_.recvData(kFixedGripperIdRegAddr, 1, fixed) && !fixed.empty()) {
            info.fixed_gripper_id = fixed[0];
        }
    }
    return true;
}

bool DatcCtrl::setFixedGripperId(uint16_t id) {
    if ((id > kFixedGripperIdMax) && (id != kFixedGripperIdNone)) {
        COUT("[Fixed Gripper ID] Invalid ID (0~126 to store, 128 to clear).");
        return false;
    }
    return command(DATC_COMMAND::SET_FIXED_GRIPPER_ID, id);
}

DeviceInfo DatcCtrl::getDeviceInfo() {
    unique_lock<mutex> lg(mutex_info_);
    return device_info_;
}

void DatcCtrl::resetDeviceInfo() {
    unique_lock<mutex> lg(mutex_info_);
    device_info_   = DeviceInfo();
    info_fail_cnt_ = 0;
    info_poll_cnt_ = 0;
    read_mode_reset_ = true;   // 레지스터 수(V3 / 구버전)도 다시 판정
}

// readDatcData(폴링 스레드)에서 호출. 연결 직후엔 성공할 때까지 매 주기 시도하고, 연속 3회 실패하면
// 이전 펌웨어(장치 정보 없음)로 보고 더 묻지 않는다. 읽은 뒤에는 약 1초마다 다시 읽어 플래그를 갱신한다.
void DatcCtrl::pollDeviceInfo() {
    const int kInfoPollPeriod = 50;   // readDatcData 50 Hz 기준 약 1 s
    const int kInfoFailLimit  = 3;
    {
        unique_lock<mutex> lg(mutex_info_);
        if (status_.legacy_map && !device_info_.unsupported) {
            device_info_ = DeviceInfo();   // 판정 전에 읽힌 값이 있으면 버린다
            device_info_.unsupported = true;
        }
        if (device_info_.unsupported) {
            return;
        }
        if (device_info_.valid && ++info_poll_cnt_ < kInfoPollPeriod) {
            return;
        }
        info_poll_cnt_ = 0;
    }

    DeviceInfo info;
    const bool ok = readDeviceInfoNow(info);

    unique_lock<mutex> lg(mutex_info_);
    if (ok) {
        device_info_   = info;
        info_fail_cnt_ = 0;
    } else if (!device_info_.valid && ++info_fail_cnt_ >= kInfoFailLimit) {
        device_info_.unsupported = true;
        COUT("[DeviceInfo] Registers 28-33 not available — firmware before v3.01. Using default limits.");
    }
}

uint16_t DatcCtrl::getVelMaxRpm() {
    const DeviceInfo info = getDeviceInfo();
    if (info.valid && info.max_speed_rpm > 0) {
        return std::min<uint16_t>(kVelMax, info.max_speed_rpm);
    }
    return kVelMax;
}

uint16_t DatcCtrl::getCurMaxMa() {
    const DeviceInfo info = getDeviceInfo();
    if (info.valid && info.cur_full_scale_ma > 0) {
        return info.cur_full_scale_ma;
    }
    return kCurMax;
}

// 조회 명령(217/221)을 보내고 응답(reg 24~27)을 받는다.
// 펌웨어는 Modbus 쓰기 응답을 먼저 보내고 그리퍼 태스크(약 1 ms 주기)에서 명령을 처리해 reg 24~27을 채우므로,
// 쓰기 직후 바로 읽으면 이전 응답이 보일 수 있다. reg 24가 요청 번호를 되돌려줄 때까지 다시 읽는다.
// 직전 응답이 이미 같은 번호였다면 갱신 여부를 구분할 수 없으므로 최소 대기 시간을 둔다.
// ※ reg 24~27을 직접 써서 지우면 안 된다 — 펌웨어는 어느 주소에 쓰든 reg 0의 명령을 다시 실행한다.
bool DatcCtrl::queryCommand(DATC_COMMAND cmd, uint16_t value_1, vector<uint16_t> &resp) {
    using namespace std::chrono;

    vector<uint16_t> prev;
    const bool same_as_prev = mbc_.recvData(kQueryRespRegAddr, kQueryRespRegNum, prev) &&
                              prev.size() == kQueryRespRegNum && prev[0] == value_1;

    if (!command(cmd, value_1)) {
        return false;
    }

    const auto start = steady_clock::now();
    while (steady_clock::now() - start < milliseconds(kQueryTimeoutMs)) {
        std::this_thread::sleep_for(milliseconds(kQueryPollMs));
        if (same_as_prev && steady_clock::now() - start < milliseconds(kQuerySameEchoWaitMs)) {
            continue;
        }
        if (mbc_.recvData(kQueryRespRegAddr, kQueryRespRegNum, resp) &&
            resp.size() == kQueryRespRegNum && resp[0] == value_1) {
            return true;
        }
    }
    COUT("[queryCommand] No response in regs 24-27 for command " + to_string((int) cmd));
    return false;
}

// Encoder harmonic compensation — MATC v3
bool DatcCtrl::setEncoderHarmonic(uint16_t k, int16_t a, int16_t b) {
    if (k < 1 || k > kEncHarmCount) {
        COUT("[setEncoderHarmonic] k must be 1–4 (use clearEncoderHarmonics to clear)");
        return false;
    }
    // 펌웨어는 진폭 상한을 넘는 계수를 응답 없이 무시하므로 여기서 먼저 거른다
    if ((int32_t) a * a + (int32_t) b * b > kEncHarmLimit * kEncHarmLimit) {
        COUT("[setEncoderHarmonic] amplitude sqrt(A^2+B^2) must be <= 2048 (11.25 deg)");
        return false;
    }
    return command(DATC_COMMAND::SET_ENC_HARM, k, (uint16_t) a, (uint16_t) b);
}

bool DatcCtrl::clearEncoderHarmonics() {
    return command(DATC_COMMAND::SET_ENC_HARM, 0, 0, 0);
}

bool DatcCtrl::saveEncoderHarmonics() {
    return command(DATC_COMMAND::SAVE_ENC_HARM);
}

bool DatcCtrl::queryEncoderHarmonic(uint16_t k, EncHarmEntry &entry) {
    if (k < 1 || k > kEncHarmCount) {
        COUT("[queryEncoderHarmonic] k must be 1–4");
        return false;
    }
    vector<uint16_t> resp;
    if (!queryCommand(DATC_COMMAND::READ_ENC_HARM, k, resp)) {
        return false;
    }
    entry.k              = resp[0];
    entry.a              = (int16_t) resp[1];
    entry.b              = (int16_t) resp[2];
    entry.active         = (resp[3] & 0x1) != 0;
    entry.last_save_ok   = (resp[3] & 0x2) != 0;
    entry.last_save_fail = (resp[3] & 0x4) != 0;
    return true;
}

// Encoder harmonic auto calibration — MATC v3.02~
string autoCalErrorText(uint8_t code) {
    switch (code) {
        case 1:  return "모터가 Enable 상태가 아닙니다. Enable 후 다시 실행하세요.";
        case 2:  return "툴(그리퍼)이 장착되어 있습니다. 툴을 분리하고 실행하세요.";
        case 3:  return "전기각이 캘리브레이션되지 않았습니다. 전기각 설정(51) 후 실행하세요.";
        case 4:  return "임피던스 모드가 켜져 있습니다. 끄고 실행하세요.";
        case 5:  return "다른 동작 중입니다. 동작이 끝난 뒤 실행하세요.";
        case 6:  return "이동 중 막혔습니다(위치 오차 30도 초과). 모터 축이 자유롭게 도는지 확인하세요.";
        case 7:  return "이동이 제때 끝나지 않았습니다.";
        case 8:  return "등속 구간 데이터가 부족합니다.";
        case 9:  return "보정 후 오차가 충분히 줄지 않아 원래 계수로 되돌렸습니다.";
        case 10: return "계산된 계수가 한도(11.25도)를 넘었습니다.";
        case 11: return "EEPROM 저장에 실패했습니다. 계수는 적용된 상태이니 'EEPROM 저장(220)'을 다시 눌러 보세요.";
        case 12: return "보정 중 모터가 정지했습니다(고장 또는 Disable).";
        default: return "알 수 없는 오류 (코드 " + to_string(code) + ")";
    }
}

bool DatcCtrl::startAutoEncHarmonic() {
    return command(DATC_COMMAND::AUTO_ENC_HARM, 1);
}

bool DatcCtrl::cancelAutoEncHarmonic() {
    return command(DATC_COMMAND::AUTO_ENC_HARM, 0);
}

bool DatcCtrl::readAutoEncHarmonicStatus(uint16_t &status) {
    vector<uint16_t> reg;
    if (!mbc_.recvData(kAutoCalRegAddr, 1, reg) || reg.empty()) {
        return false;
    }
    status = reg[0];
    return true;
}

bool DatcCtrl::queryAutoEncHarmonicResult(uint16_t k, AutoEncHarmResult &result) {
    if (k < 1 || k > kEncHarmCount) {
        COUT("[queryAutoEncHarmonicResult] k must be 1–4");
        return false;
    }
    vector<uint16_t> resp;
    if (!queryCommand(DATC_COMMAND::READ_AUTO_ENC_HARM, k, resp)) {
        return false;
    }
    result.k           = resp[0];
    result.before_mdeg = resp[1];
    result.after_mdeg  = resp[2];
    result.status      = resp[3];
    return true;
}

// Impedance related functions
bool DatcCtrl::impedanceOn() {
    return command(DATC_COMMAND::IMPEDANCE_ON);
}

bool DatcCtrl::impedanceOff() {
    return command(DATC_COMMAND::IMPEDANCE_OFF);
}

bool DatcCtrl::setImpedanceParams(int16_t slave_num, int16_t stiffness_level) {
    string error_prefix = "[Set Impedance M]";

    if (slave_num < kModbusSlaveAddrMin) {
        printf("[Error] %s slave_num is too small ( < %d)\n", error_prefix.c_str(), kModbusSlaveAddrMin);
        slave_num = kModbusSlaveAddrMin;
    } else if (slave_num > kModbusSlaveAddrMax) {
        printf("[Error] %s slave_num is too large ( > %d)\n", error_prefix.c_str(), kModbusSlaveAddrMax);
        slave_num = kModbusSlaveAddrMax;
    }

    if (stiffness_level < 1) {
        printf("[Error] %s stiffness_level is too small ( < %d)\n", error_prefix.c_str(), 1);
        stiffness_level = 1;
    } else if (stiffness_level > 10) {
        printf("[Error] %s stiffness_level is too large ( > %d)\n", error_prefix.c_str(), 10);
        stiffness_level = 10;
    }

    return command(DATC_COMMAND::SET_IMPEDANCE_PARAMS, slave_num, stiffness_level);
}

bool DatcCtrl::setImpedanceMBK(int16_t mass, int16_t damping, int16_t stiffness) {
    return command(DATC_COMMAND::SET_IMPEDANCE_MBK,
                   (uint16_t) mass,
                   (uint16_t) damping,
                   (uint16_t) stiffness);
}

