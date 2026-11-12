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
#include <atomic>
#include <condition_variable>
#include <thread>
#include <QThread>

#include "grp_control_msg/msg/gripper_msg.hpp"

#include "grp_control_msg/srv/pos_vel_cur_ctrl.hpp"
#include "grp_control_msg/srv/gripper_command.hpp"
#include "grp_control_msg/srv/single_boolean.hpp"
#include "grp_control_msg/srv/single_int.hpp"
#include "grp_control_msg/srv/void.hpp"
#include "grp_control_msg/srv/impedance_params.hpp"
#include "grp_control_msg/srv/write_gripper_map.hpp"
#include "grp_control_msg/srv/read_gripper_map.hpp"
#include "grp_control_msg/srv/set_modbus_baudrate.hpp"

using namespace std;
using namespace grp_control_msg::srv;
using namespace grp_control_msg::msg;

// ROS2판 통신 인터페이스. 일반판(TCP)의 datc_comm_interface와 바꿔 끼우는 파일이고, 나머지 소스(datc_ctrl,
// main_window, ui)는 일반판과 같다. main_window는 이 헤더가 넣는 rclcpp(RCLCPP__RCLCPP_HPP_)를 보고 TCP 탭을 뺀다.
//   - 폴링 스레드(run, 50 Hz): 상태를 읽어 /grp_state로 발행한다.
//   - 서비스 스레드(spinRos): ROS2 서비스 요청을 처리한다. 일반판 TCP 명령 스레드에 해당한다.
class DatcCommInterface : public QThread, public DatcCtrl {
    Q_OBJECT

public:
    DatcCommInterface(int argc, char **argv);
    virtual ~DatcCommInterface();

Q_SIGNALS:
    void rosShutdown();
    // set_modbus_baudrate(52). 장치가 속도를 바꾸면 GCS도 새 속도로 다시 연결해야 하므로
    // UI 스레드(MainWindow::remoteSetDeviceBaudrate)가 처리한다
    void deviceBaudrateRequested(int baudrate);

public:
    bool init(const char *port_name, uint16_t slave_address, int baudrate);

    // set_modbus_baudrate 처리 결과 — 기다리던 서비스 스레드가 응답으로 돌려준다.
    // baudrate = 처리 뒤 GCS가 연결된 속도, result = changed / unchanged / reverted / no_response / not_sent
    void reportDeviceBaudrate(bool ok, int baudrate, const string &result);
    bool isPollingPaused() {return poll_pause_.load() > 0;}

    // UI 스레드가 재연결·속도 검색처럼 버스를 연달아 쓰는 동안 50 Hz 폴링을 멈춘다(중첩 가능).
    // 폴링 스레드는 응답이 없으면 한 주기에 읽기를 2~3번(각각 응답 시간 초과까지) 연달아 하며
    // mutex_comm_을 거의 놓지 않는다. Windows 뮤텍스는 공정하지 않아 UI 스레드의 probe가 락을
    // 얻지 못하고, Release에서는 사실상 영원히 기다렸다(속도 자동 검색 '응답 없음', 2026-10-19).
    // pausePolling은 진행 중인 주기가 끝날 때까지 기다린다(최대 약 1.5 s). PollingPause로 쓴다.
    // ROS2판에서는 멈춘 동안 서비스 요청도 처리하지 않고 쌓아 두었다가, 다시 시작하면 처리한다.
    void pausePolling();
    void resumePolling();

private:
    shared_ptr<rclcpp::Node> nh_;

    // Publisher
    rclcpp::Publisher<GripperMsg>::SharedPtr publisher_grp_state_;

    // Server
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

    rclcpp::Service<PosVelCurCtrl>::SharedPtr srv_motor_pos_ctrl_;
    rclcpp::Service<PosVelCurCtrl>::SharedPtr srv_motor_vel_ctrl_;
    rclcpp::Service<PosVelCurCtrl>::SharedPtr srv_motor_cur_ctrl_;

    // KR-MATC
    rclcpp::Service<Void>::SharedPtr            srv_error_clear_;
    rclcpp::Service<Void>::SharedPtr            srv_impedance_on_;
    rclcpp::Service<Void>::SharedPtr            srv_impedance_off_;
    rclcpp::Service<ImpedanceParams>::SharedPtr srv_set_impedance_params_;
    rclcpp::Service<WriteGripperMap>::SharedPtr srv_write_gripper_map_;
    rclcpp::Service<ReadGripperMap>::SharedPtr  srv_read_gripper_map_;
    rclcpp::Service<Void>::SharedPtr            srv_save_gripper_map_;
    rclcpp::Service<SetModbusBaudrate>::SharedPtr srv_set_modbus_baudrate_;

    // set_modbus_baudrate 결과 넘겨받기 (UI 스레드 → 서비스 스레드)
    mutex              mutex_baud_;
    condition_variable cv_baud_;
    bool               baud_done_   = false;
    bool               baud_ok_     = false;
    int                baud_now_    = 0;
    string             baud_result_;

    std::thread       ros_thread_;
    std::atomic<bool> flag_ros_stop_{false};       // 서비스 스레드 종료 — 소멸자가 폴링보다 먼저 멈춘다
    std::atomic<bool> flag_program_stop_{false};
    std::atomic<int>  poll_pause_{0};   // > 0이면 폴링 주기를 건너뛰고 서비스 요청도 미룬다
    mutex             mutex_poll_;      // 폴링 한 주기 동안 잡는다 — pausePolling이 주기 끝을 기다린다

    void run();
    void spinRos();
    void pubTopic();
};

// 범위 안에서 폴링을 멈춘다: PollingPause pause(datc_interface_);
class PollingPause {
public:
    explicit PollingPause(DatcCommInterface *d) : d_(d) { d_->pausePolling(); }
    ~PollingPause() { d_->resumePolling(); }
    PollingPause(const PollingPause &) = delete;
    PollingPause &operator=(const PollingPause &) = delete;

private:
    DatcCommInterface *d_;
};

#endif // DATC_COMM_INTERFACE_HPP
