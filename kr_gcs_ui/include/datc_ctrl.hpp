/**
 * @file datc_ctrl.hpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief
 * @version 1.0
 * @date 2023-11-03
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef DATC_CTRL_HPP
#define DATC_CTRL_HPP

#include "modbus_comm.hpp"
#include <map>

#define CMD_ADDR 0

#define SEND_CMD_VECTOR(...) mbc_.sendData(CMD_ADDR, __VA_ARGS__)
#define SEND_CMD(...) mbc_.sendData(CMD_ADDR, (uint16_t) __VA_ARGS__)

using namespace std;

const uint16_t kDurationMin    = 10;
const uint16_t kDurationMax    = 10000;
const uint16_t kFingerPosMin   = 0;
const uint16_t kFingerPosMax   = 10000;
const uint16_t kTorqueRatioMin = 50;
const uint16_t kTorqueRatioMax = 100;
const uint16_t kSpeedRatioMin  = 0;
const uint16_t kSpeedRatioMax  = 100;

const uint16_t kVelMin =  100;
const uint16_t kVelMax =  900;
const uint16_t kCurMax = 1200;

enum class DATC_COMMAND {
    MOTOR_ENABLE           = 1,
    MOTOR_STOP             = 2,
    MOTOR_DISABLE          = 4,
    MOTOR_POSITION_CONTROL = 5,
    MOTOR_VELOCITY_CONTROL = 6,
    MOTOR_CURRENT_CONTROL  = 7,
    CHANGE_MODBUS_ADDRESS  = 50,
    GRIPPER_INITIALIZE     = 101,
    GRIPPER_OPEN           = 102,
    GRIPPER_CLOSE          = 103,
    SET_FINGER_POSITION    = 104,
    VACUUM_GRIPPER_ON      = 106,
    VACUUM_GRIPPER_OFF     = 107,
    SET_MOTOR_TORQUE       = 212,
    SET_MOTOR_SPEED        = 213,
};

struct DatcStatus {
    string status_str;

    bool enable         = false;
    bool initialize     = false;
    bool motor_pos_ctrl = false;
    bool motor_vel_ctrl = false;
    bool motor_cur_ctrl = false;
    bool grp_open       = false;
    bool grp_close      = false;
    bool fault          = false;

    int16_t motor_pos = 0;
    int16_t motor_vel = 0;
    int16_t motor_cur = 0;
    uint16_t finger_pos = 0;
    uint16_t voltage    = 0;
    uint16_t states     = 0;
};

class DatcCtrl {
public:
    DatcCtrl();
    ~DatcCtrl();

    bool modbusInit(const char *port_name, uint16_t slave_address);
    bool modbusRelease();
    bool modbusSlaveChange(uint16_t slave_addr);

    bool motorEnable();
    bool motorStop();
    bool motorDisable();

    bool setModbusAddr(uint16_t slave_addr);

    bool grpInitialize();
    bool grpOpen();
    bool grpClose();

    // Datc control
    bool setFingerPos(uint16_t finger_pos);
    bool motorVelCtrl(int16_t vel);
    bool motorCurCtrl(int16_t cur);
    bool motorPosCtrl(int16_t pos_deg, uint16_t duration);

    bool vacuumGrpOn();
    bool vacuumGrpOff();

    bool setMotorTorque(uint16_t torque_ratio);
    bool setMotorSpeed (uint16_t speed_ratio);

    bool readDatcData();
    DatcStatus getDatcStatus() {return status_;}
    bool getConnectionState() {return mbc_.getConnectionState();}
    bool getModbusRecvErr() {return flag_modbus_recv_err_;}

    uint16_t getSlaveAddr() {return mbc_.getSlaveAddr();}

protected:
    bool checkDurationRange(string error_prefix, uint16_t &duration);
    bool command(DATC_COMMAND cmd, uint16_t value_1 = 0, uint16_t value_2 = 0);

    ModbusComm mbc_;
    DatcStatus status_;

    bool flag_modbus_recv_err_ = false;
};

#endif // DATC_CTRL_HPP
