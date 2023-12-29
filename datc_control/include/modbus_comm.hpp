/**
 * @file modbus_comm.hpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief
 * @version 1.0
 * @date 2023-11-03
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef MODBUS_COMM_HPP
#define MODBUS_COMM_HPP

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(WIN64) || defined(_WIN64) || defined(__WIN64__)
#include "modbus-rtu.h"
#include <unistd.h>
#else
#include <modbus/modbus-rtu.h>
#endif

#include <mutex>
#include <iostream>
#include <vector>

#define BAUDRATE      38400
#define DEBUG_MODE    false
#define DATA_BIT      8
#define STOP_BIT      1
#define PARITY_MODE   'N'

using namespace std;

#define COUT(...) cout << __VA_ARGS__ << endl

class ModbusComm {
public:
    ModbusComm() {}
    ~ModbusComm() {
        modbusRelease();
    }

    bool modbusInit(const char *port_name, uint16_t slave_addr) {
        unique_lock<mutex> lg(mutex_comm_);

        mb_ = modbus_new_rtu(port_name, BAUDRATE, PARITY_MODE, DATA_BIT, STOP_BIT);

        modbus_rtu_set_serial_mode(mb_, MODBUS_RTU_RS485);
        modbus_rtu_set_rts_delay  (mb_, 300);
        modbus_set_debug          (mb_, DEBUG_MODE);

        if (mb_ == NULL) {
            fprintf(stderr, "Unable to create the libmodbus context\n");
            return false;
        }

        if (modbus_set_slave(mb_, slave_addr) == -1) {
            fprintf(stderr, "server_id= %d Invalid slave ID: %s\n", slave_addr, modbus_strerror(errno));
            modbus_free(mb_);
            return false;
        }

        if (modbus_connect(mb_) == -1) {
            fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
            modbus_free(mb_);
            return false;
        }

        slave_num_ = slave_addr;
        connection_state_ = true;
        COUT("Modbus communication initiated");

        return true;
    }

    void modbusRelease() {
        slave_num_ = 0;
        connection_state_ = false;

        unique_lock<mutex> lg(mutex_comm_);

        modbus_close(mb_);
        modbus_free (mb_);
        COUT("Modbus released");
    }

    bool slaveChange(uint16_t slave_addr) {
        connection_state_ = false;

        unique_lock<mutex> lg(mutex_comm_);

        if (modbus_set_slave(mb_, slave_addr) == -1) {
            fprintf(stderr, "server_id= %d Invalid slave ID: %s\n", slave_addr, modbus_strerror(errno));
            modbus_close(mb_);
            modbus_free (mb_);
            connection_state_ = false;
            return false;
        }

        usleep(10000);
        printf("Modbus slave address changed to %d\n", slave_addr);
        slave_num_ = slave_addr;
        connection_state_ = true;

        return true;
    }

    bool sendData(int reg_addr, vector<uint16_t> data) {
        if (!connection_state_) {
            COUT("Modbus communication is not enabled.");
            return false;
        }

        unique_lock<mutex> lg(mutex_comm_);

        uint16_t register_number = data.size();

        if (register_number == 1) {
            if (modbus_write_register(mb_, reg_addr, data[0]) == -1) {
                fprintf(stderr, "Failed to modbus write register %d : %s\n", reg_addr, modbus_strerror(errno));
                return false;
            }
        } else if (modbus_write_registers(mb_, reg_addr, register_number, &data[0]) == -1) {
            fprintf(stderr, "Failed to modbus write register %d : %s\n", reg_addr, modbus_strerror(errno));
            return false;
        }

        return true;
    }

    bool sendData(int reg_addr, uint16_t data) {
        if (!connection_state_) {
            COUT("Modbus communication is not enabled.");
            return false;
        }

        unique_lock<mutex> lg(mutex_comm_);

        if (modbus_write_register(mb_, reg_addr, data) == -1) {
            fprintf(stderr, "Failed to modbus write register %d : %s\n", reg_addr, modbus_strerror(errno));
            return false;
        } else {
            return true;
        }
    }

    bool recvData(int reg_addr, int nb, vector<uint16_t> &data) {
        if (!connection_state_) {
            COUT("Modbus communication is not enabled.");
            return false;
        }

        unique_lock<mutex> lg(mutex_comm_);

        uint16_t data_temp[nb];

        if (modbus_read_registers(mb_, reg_addr, nb, data_temp) == -1) {
            mutex_comm_.unlock();
            fprintf(stderr, "Failed to read input registers! : %s\n", modbus_strerror(errno));
            return false;
        }

        data.clear();

        for (auto i : data_temp) {
            data.push_back(i);
        }

        return true;
    }

    bool getConnectionState() {return connection_state_;}

    uint16_t getSlaveAddr() {return slave_num_;}

private:
    mutex mutex_comm_;
    modbus_t *mb_;

    bool connection_state_ = false;

    uint16_t slave_num_ = 0;
};

#endif // MODBUS_COMM_HPP
