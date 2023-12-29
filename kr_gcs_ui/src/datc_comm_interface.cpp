/**
 * @file datc_comm_interface.cpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief
 * @version 1.0
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "datc_comm_interface.hpp"

const uint kFreq = 50;

DatcCommInterface::DatcCommInterface(int argc, char **argv) {
    rclcpp::init(argc, argv);
    nh_ = rclcpp::Node::make_shared("DATC_Control_Interface");

    // Publisher
    publisher_grp_state_ = nh_->create_publisher<GripperMsg> ("grp_state", 1000);

    // Server
    // srv_modbus_init_release_ = nh_->create_service<SingleBoolean>("modbus_init_release",
    //                            [&] (const shared_ptr<SingleBoolean::Request> req, shared_ptr<SingleBoolean::Response> res) {
    //                                req;
    //                            });

    srv_motor_enable_ = nh_->create_service<Void>("motor_enable",
                        [&] (const shared_ptr<Void::Request> req, shared_ptr<Void::Response> res) {
                            res->successed = motorEnable();
                        });

    srv_motor_disable_ = nh_->create_service<Void>("motor_disable",
                         [&] (const shared_ptr<Void::Request> req, shared_ptr<Void::Response> res) {
                             res->successed = motorDisable();
                         });

    srv_modbus_slave_change_ = nh_->create_service<SingleInt>("modbus_slave_change",
                               [&] (const shared_ptr<SingleInt::Request> req, shared_ptr<SingleInt::Response> res) {
                                   res->successed = modbusSlaveChange((uint) req->value);
                               });

    srv_set_modbus_addr_ = nh_->create_service<SingleInt>("set_modbus_addr",
                           [&] (const shared_ptr<SingleInt::Request> req, shared_ptr<SingleInt::Response> res) {
                               res->successed = setModbusAddr((uint) req->value);
                           });

    srv_set_finger_pos_ = nh_->create_service<SingleInt>("set_finger_pos",
                          [&] (const shared_ptr<SingleInt::Request> req, shared_ptr<SingleInt::Response> res) {
                              res->successed = setFingerPos((uint) req->value);
                          });

    srv_set_motor_torque_ = nh_->create_service<SingleInt>("set_motor_torque",
                            [&] (const shared_ptr<SingleInt::Request> req, shared_ptr<SingleInt::Response> res) {
                                res->successed = setMotorTorque((uint) req->value);
                            });

    srv_set_motor_speed_ = nh_->create_service<SingleInt>("set_motor_speed",
                           [&] (const shared_ptr<SingleInt::Request> req, shared_ptr<SingleInt::Response> res) {
                               res->successed = setMotorSpeed((uint) req->value);
                           });

    srv_motor_stop_ = nh_->create_service<Void>("motor_stop",
                      [&] (const shared_ptr<Void::Request> req, shared_ptr<Void::Response> res) {
                          res->successed = motorStop();
                      });

    srv_grp_initialize_ = nh_->create_service<Void>("gripper_initialize",
                          [&] (const shared_ptr<Void::Request> req, shared_ptr<Void::Response> res) {
                              res->successed = grpInitialize();
                          });

    srv_grp_open_ = nh_->create_service<Void>("grp_open",
                    [&] (const shared_ptr<Void::Request> req, shared_ptr<Void::Response> res) {
                        res->successed = grpOpen();
                    });

    srv_grp_close_ = nh_->create_service<Void>("grp_close",
                     [&] (const shared_ptr<Void::Request> req, shared_ptr<Void::Response> res) {
                         res->successed = grpClose();
                     });

    srv_vacuum_grp_on_ = nh_->create_service<Void>("vacuum_grp_on",
                         [&] (const shared_ptr<Void::Request> req, shared_ptr<Void::Response> res) {
                             res->successed = vacuumGrpOn();
                         });

    srv_vacuum_grp_off_ = nh_->create_service<Void>("vacuum_grp_off",
                          [&] (const shared_ptr<Void::Request> req, shared_ptr<Void::Response> res) {
                              res->successed = vacuumGrpOff();
                          });

    // srv_motor_pos_ctrl_ = nh_->create_service<PosVelCurCtrl>("motor_pos_ctrl",
    //                       [&] (const shared_ptr<PosVelCurCtrl::Request> req, shared_ptr<PosVelCurCtrl::Response> res) {
    //                           res->successed = motorPosCtrl(req->position, req->duration);
    //                       });

    srv_motor_vel_ctrl_ = nh_->create_service<PosVelCurCtrl>("motor_vel_ctrl",
                          [&] (const shared_ptr<PosVelCurCtrl::Request> req, shared_ptr<PosVelCurCtrl::Response> res) {
                              res->successed = motorVelCtrl(req->velocity);
                          });

    srv_motor_cur_ctrl_ = nh_->create_service<PosVelCurCtrl>("motor_cur_ctrl",
                          [&] (const shared_ptr<PosVelCurCtrl::Request> req, shared_ptr<PosVelCurCtrl::Response> res) {
                              res->successed = motorCurCtrl(req->current);
                          });

    COUT("DATC ros interface init.");
    start();
}

DatcCommInterface::~DatcCommInterface() {
    modbusRelease();
}

bool DatcCommInterface::init(const char *port_name, uint slave_address) {
    return modbusInit(port_name, slave_address);
}

void DatcCommInterface::pubTopic() {
    if (getConnectionState()) {
        DatcStatus datc_status = getDatcStatus();
        grp_control_msg::msg::GripperMsg msg;

        msg.motor_position  = datc_status.motor_pos;
        msg.motor_velocity  = datc_status.motor_vel;
        msg.motor_current   = datc_status.motor_cur;
        msg.finger_position = datc_status.finger_pos;

        msg.motor_enabled       = datc_status.enable;
        msg.gripper_initialized = datc_status.initialize;
        msg.position_ctrl_mode  = datc_status.motor_pos_ctrl;
        msg.velocity_ctrl_mode  = datc_status.motor_vel_ctrl;
        msg.current_ctrl_mode   = datc_status.motor_cur_ctrl;
        msg.grp_opened          = datc_status.grp_open;
        msg.grp_closed          = datc_status.grp_close;
        msg.motor_fault         = datc_status.fault;

        publisher_grp_state_->publish(msg);
    }
}

// Main loop
void DatcCommInterface::run() {
    double period = 1 / (double) kFreq;

    timespec time_prev, time_current;
    clock_gettime(CLOCK_MONOTONIC, &time_prev);

    while(rclcpp::ok()) {
        clock_gettime(CLOCK_MONOTONIC, &time_current);

        double dt = (time_current.tv_sec - time_prev.tv_sec) + ((time_current.tv_nsec - time_prev.tv_nsec) * 0.000000001);

        if(dt >= period) {
            if (getConnectionState()) {
                rclcpp::spin_some(nh_);

                if (mbc_.getConnectionState()) {
                    readDatcData();
                    pubTopic();
                }
            }

            time_prev = time_current;
        }
    }

    motorDisable();
    modbusRelease();

    Q_EMIT rclcpp::shutdown();
}

