/**
 * @file datc_comm_interface.hpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief
 * @version 1.0
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef DATC_COMM_INTERFACE_HPP
#define DATC_COMM_INTERFACE_HPP

#include "datc_ctrl.hpp"
#include <rclcpp/rclcpp.hpp>
#include <QThread>

#include "grp_control_msg/msg/gripper_msg.hpp"

#include "grp_control_msg/srv/pos_vel_cur_ctrl.hpp"
#include "grp_control_msg/srv/gripper_command.hpp"
#include "grp_control_msg/srv/single_boolean.hpp"
#include "grp_control_msg/srv/single_int.hpp"
#include "grp_control_msg/srv/void.hpp"

using namespace std;
using namespace grp_control_msg::srv;
using namespace grp_control_msg::msg;

class DatcCommInterface : public QThread, public DatcCtrl {
    Q_OBJECT

public:
    DatcCommInterface(int argc, char **argv);
    virtual ~DatcCommInterface();

Q_SIGNALS:
    void rosShutdown();

public:
	bool init(const char *port_name, uint slave_address);

private:
    shared_ptr<rclcpp::Node> nh_;

    // Publisher
    rclcpp::Publisher<GripperMsg>::SharedPtr publisher_grp_state_;

    // Server
    // rclcpp::Service<SingleBoolean>::SharedPtr srv_modbus_init_release_;
    rclcpp::Service<Void>::SharedPtr srv_motor_enable_;
    rclcpp::Service<Void>::SharedPtr srv_motor_disable_;

    rclcpp::Service<SingleInt>::SharedPtr srv_modbus_slave_change_;
    rclcpp::Service<SingleInt>::SharedPtr srv_set_modbus_addr_;
    rclcpp::Service<SingleInt>::SharedPtr srv_set_finger_pos_;
    rclcpp::Service<SingleInt>::SharedPtr srv_set_motor_torque_;
    rclcpp::Service<SingleInt>::SharedPtr srv_set_motor_speed_;

    rclcpp::Service<Void>::SharedPtr srv_motor_stop_;
    rclcpp::Service<Void>::SharedPtr srv_grp_initialize_;
    rclcpp::Service<Void>::SharedPtr srv_grp_open_;
    rclcpp::Service<Void>::SharedPtr srv_grp_close_;
    rclcpp::Service<Void>::SharedPtr srv_vacuum_grp_on_;
    rclcpp::Service<Void>::SharedPtr srv_vacuum_grp_off_;

    // rclcpp::Service<PosVelCurCtrl>::SharedPtr srv_motor_pos_ctrl_;
    rclcpp::Service<PosVelCurCtrl>::SharedPtr srv_motor_vel_ctrl_;
    rclcpp::Service<PosVelCurCtrl>::SharedPtr srv_motor_cur_ctrl_;


    bool is_enable_            = false;
    bool modbus_connect_state_ = false;
    bool read_mode_            = true;

    mutex mutex_com_;
    mutex mutex_var_;

	void run();

    void pubTopic();

    bool checkValue();
};

#endif // DATC_COMM_INTERFACE_HPP
