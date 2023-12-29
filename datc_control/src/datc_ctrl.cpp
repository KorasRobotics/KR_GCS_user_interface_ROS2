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

DatcCtrl::DatcCtrl() {
}

DatcCtrl::~DatcCtrl() {
}

bool DatcCtrl::modbusInit(const char *port_name, uint16_t slave_address) {
    return mbc_.modbusInit(port_name, slave_address);
}

bool DatcCtrl::modbusRelease() {
    mbc_.modbusRelease();
    return true;
}

bool DatcCtrl::modbusSlaveChange(uint16_t slave_addr) {
    return mbc_.slaveChange(slave_addr);
}

bool DatcCtrl::motorEnable() {
    return command(DATC_COMMAND::MOTOR_ENABLE);
}

bool DatcCtrl::motorStop() {
    return command(DATC_COMMAND::MOTOR_STOP);
}

bool DatcCtrl::motorDisable() {
    return command(DATC_COMMAND::MOTOR_DISABLE);
}

bool DatcCtrl::setModbusAddr(uint16_t slave_addr) {
    // TODO: modbus addr 범위 지정 필요
    if (slave_addr < 1 || slave_addr >= 100) {
        COUT("\"setModbusAddr\" function error. Check the input slave address.");
        return false;
    }

    return command(DATC_COMMAND::CHANGE_MODBUS_ADDRESS, slave_addr);
}

bool DatcCtrl::grpInitialize() {
    return command(DATC_COMMAND::GRIPPER_INITIALIZE);
}

bool DatcCtrl::grpOpen() {
    return command(DATC_COMMAND::GRIPPER_OPEN);
}

bool DatcCtrl::grpClose() {
    return command(DATC_COMMAND::GRIPPER_CLOSE);
}

bool DatcCtrl::setFingerPos(uint16_t finger_pos) {
    string error_prefix = "[Set Finger Position]";

    if (finger_pos < kFingerPosMin) {
        printf("%s Invalid range of finger position ( < %d)", error_prefix.c_str(), kFingerPosMin);
        finger_pos = kFingerPosMin;
    } else if (finger_pos > kFingerPosMax) {
        printf("%s Invalid range of finger position ( > %d)", error_prefix.c_str(), kFingerPosMax);
        finger_pos = kFingerPosMax;
    }

    return command(DATC_COMMAND::SET_FINGER_POSITION, finger_pos);
}

bool DatcCtrl::motorVelCtrl(int16_t vel) {
    string error_prefix = "[Motor Velocity Control]";

    if (abs(vel) < kVelMin) {
        printf("%s Invalid range of speed ( < %d)", error_prefix.c_str(), kVelMin);
        vel = (vel >= 0) ? kVelMin : -kVelMin;
    } else if (abs(vel) > kVelMax) {
        printf("%s Invalid range of speed ( > %d)", error_prefix.c_str(), kVelMax);
        vel = (vel >= 0) ? kVelMax : -kVelMax;
    }

    return command(DATC_COMMAND::MOTOR_VELOCITY_CONTROL, vel, 500); // duration no longer works.
}

bool DatcCtrl::motorCurCtrl(int16_t cur) {
    string error_prefix = "[Motor Current Control]";

    if (abs(cur) > kCurMax) {
        printf("%s Invalid range of current ( > %d)", error_prefix.c_str(), kCurMax);
        cur = (cur >= 0) ? kCurMax : -kCurMax;
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

bool DatcCtrl::setMotorTorque(uint16_t torque_ratio) {
    string error_prefix = "[Set Motor Torque]";

    if (torque_ratio < kTorqueRatioMin) {
        printf("%s Motor torque is too low ( < %d)", error_prefix.c_str(), kTorqueRatioMin);
        torque_ratio = kTorqueRatioMin;
    } else if (torque_ratio > kTorqueRatioMax) {
        printf("%s Motor torque is too high ( > %d)", error_prefix.c_str(), kTorqueRatioMax);
        torque_ratio = kTorqueRatioMax;
    }

    return command(DATC_COMMAND::SET_MOTOR_TORQUE, torque_ratio);
}

bool DatcCtrl::setMotorSpeed (uint16_t speed_ratio) {
    string error_prefix = "[Set Motor Speed]";

    if (speed_ratio < kSpeedRatioMin) {
        printf("%s Motor torque is too low ( < %d)", error_prefix.c_str(), kSpeedRatioMin);
        speed_ratio = kSpeedRatioMin;
    } else if (speed_ratio > kSpeedRatioMax) {
        printf("%s Motor torque is too high ( > %d)", error_prefix.c_str(), kSpeedRatioMax);
        speed_ratio = kSpeedRatioMax;
    }

    return command(DATC_COMMAND::SET_MOTOR_SPEED, speed_ratio);
}

bool DatcCtrl::readDatcData() {
    static map<uint16_t, pair<bool*, string>> status_info;

    if (status_info.size() == 0) {
        // Bit, Value, Status 순서
        status_info.insert({0, make_pair(&status_.enable        , "Motor Enable")});
        status_info.insert({1, make_pair(&status_.initialize    , "Gripper Initialize")});
        status_info.insert({2, make_pair(&status_.motor_pos_ctrl, "Motor Position Control")});
        status_info.insert({3, make_pair(&status_.motor_vel_ctrl, "Motor Velocity Control")});
        status_info.insert({4, make_pair(&status_.motor_cur_ctrl, "Motor Current Control")});
        status_info.insert({5, make_pair(&status_.grp_open      , "Gripper Open")});
        status_info.insert({6, make_pair(&status_.grp_close     , "Gripper Close")});
        status_info.insert({9, make_pair(&status_.fault         , "Motor Fault")});
    }

    // Read input register //
    uint16_t reg_addr = 10;
    uint16_t reg_num  = 8;
    vector<uint16_t> reg;

    if (mbc_.recvData(reg_addr, reg_num, reg)) {
        uint16_t status    = reg[0];
        status_.states     = status;
        status_.motor_pos  = (int16_t) reg[1];
        status_.motor_cur  = (int16_t) reg[2];
        status_.motor_vel  = (int16_t) reg[3];
        status_.finger_pos = reg[4];
        status_.voltage    = reg[7];

        status_.status_str = "---";

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

        if (!status_.enable) {
            status_.status_str = "Motor Disabled";
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
        printf("%s Duration is too short ( < %dms)", error_prefix.c_str(), kDurationMin);
        duration = kDurationMin;
        return false;
    } else if (duration > kDurationMax) {
        printf("%s Duration is too long ( > %dms)", error_prefix.c_str(), kDurationMax);
        duration = kDurationMax;
        return false;
    }

    return true;
}

bool DatcCtrl::command(DATC_COMMAND cmd, uint16_t value_1, uint16_t value_2) {
    switch (cmd) {
        case DATC_COMMAND::MOTOR_ENABLE:
            return SEND_CMD(cmd);

        case DATC_COMMAND::MOTOR_STOP:
            return SEND_CMD(cmd);

        case DATC_COMMAND::MOTOR_DISABLE:
            return SEND_CMD(cmd);

        case DATC_COMMAND::MOTOR_POSITION_CONTROL:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1, value_2}));

        case DATC_COMMAND::MOTOR_VELOCITY_CONTROL:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1, value_2}));

        case DATC_COMMAND::MOTOR_CURRENT_CONTROL:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1, value_2}));

        case DATC_COMMAND::CHANGE_MODBUS_ADDRESS:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1}));

        case DATC_COMMAND::GRIPPER_INITIALIZE:
            return SEND_CMD(cmd);

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

        case DATC_COMMAND::SET_MOTOR_TORQUE:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1}));

        case DATC_COMMAND::SET_MOTOR_SPEED:
            return SEND_CMD_VECTOR(vector<uint16_t> ({(uint16_t) cmd, value_1}));

        default:
            COUT("Error: Undefined command.");
            return false;
    }
}
