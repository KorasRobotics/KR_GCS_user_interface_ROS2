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
#include <QCoreApplication>

const uint16_t kFreq = 50;

DatcCommInterface::DatcCommInterface(int argc, char **argv) {
    rclcpp::init(argc, argv);
    nh_ = rclcpp::Node::make_shared("MATC_Control_Interface");

    // Publisher
    publisher_grp_state_ = nh_->create_publisher<GripperMsg> ("grp_state", 1000);

    // Server
    srv_motor_enable_ = nh_->create_service<Void>("motor_enable",
                        [&] (const shared_ptr<Void::Request>, shared_ptr<Void::Response> res) {
                            COUT("[Service called] motor_enable");
                            res->successed = motorEnable();
                        });

    srv_motor_disable_ = nh_->create_service<Void>("motor_disable",
                         [&] (const shared_ptr<Void::Request>, shared_ptr<Void::Response> res) {
                             COUT("[Service called] motor_disable");
                             res->successed = motorDisable();
                         });

    srv_modbus_slave_change_ = nh_->create_service<SingleInt>("modbus_slave_change",
                               [&] (const shared_ptr<SingleInt::Request> req, shared_ptr<SingleInt::Response> res) {
                                   COUT("[Service called] modbus_slave_change, input: " << (uint) req->value);
                                   res->successed = modbusSlaveChange((uint) req->value);
                               });

    srv_set_modbus_addr_ = nh_->create_service<SingleInt>("set_modbus_addr",
                           [&] (const shared_ptr<SingleInt::Request> req, shared_ptr<SingleInt::Response> res) {
                               COUT("[Service called] set_modbus_addr, input: " << (uint) req->value);
                               res->successed = setModbusAddr((uint) req->value);
                           });

    // MATC v3: 0 ~ 그리퍼 스트로크 [mm]. 구버전 펌웨어: 0 ~ 1000 [‰] (/grp_state의 legacy_firmware)
    srv_set_finger_pos_ = nh_->create_service<SingleInt>("set_finger_pos",
                          [&] (const shared_ptr<SingleInt::Request> req, shared_ptr<SingleInt::Response> res) {
                              COUT("[Service called] set_finger_pos, input: " << (uint) req->value);
                              res->successed = setFingerPos((uint) req->value);
                          });

    // MATC v3: 파지력 [N]. 구버전 펌웨어: 정격 토크의 [%]
    srv_set_motor_torque_ = nh_->create_service<SingleInt>("set_motor_torque",
                            [&] (const shared_ptr<SingleInt::Request> req, shared_ptr<SingleInt::Response> res) {
                                COUT("[Service called] set_motor_torque, input: " << (uint) req->value);
                                res->successed = setMotorTorque((uint) req->value);
                            });

    srv_set_motor_speed_ = nh_->create_service<SingleInt>("set_motor_speed",
                           [&] (const shared_ptr<SingleInt::Request> req, shared_ptr<SingleInt::Response> res) {
                               COUT("[Service called] set_motor_speed, input: " << (uint) req->value);
                               res->successed = setMotorSpeed((uint) req->value);
                           });

    srv_motor_stop_ = nh_->create_service<Void>("motor_stop",
                      [&] (const shared_ptr<Void::Request>, shared_ptr<Void::Response> res) {
                          COUT("[Service called] motor_stop");
                          res->successed = motorStop();
                      });

    srv_grp_initialize_ = nh_->create_service<Void>("gripper_initialize",
                          [&] (const shared_ptr<Void::Request>, shared_ptr<Void::Response> res) {
                              COUT("[Service called] gripper_initialize");
                              res->successed = grpInitialize();
                          });

    srv_grp_open_ = nh_->create_service<Void>("grp_open",
                    [&] (const shared_ptr<Void::Request>, shared_ptr<Void::Response> res) {
                        COUT("[Service called] grp_open");
                        res->successed = grpOpen();
                    });

    srv_grp_close_ = nh_->create_service<Void>("grp_close",
                     [&] (const shared_ptr<Void::Request>, shared_ptr<Void::Response> res) {
                         COUT("[Service called] grp_close");
                         res->successed = grpClose();
                     });

    srv_vacuum_grp_on_ = nh_->create_service<Void>("vacuum_grp_on",
                         [&] (const shared_ptr<Void::Request>, shared_ptr<Void::Response> res) {
                             COUT("[Service called] vacuum_grp_on");
                             res->successed = vacuumGrpOn();
                         });

    srv_vacuum_grp_off_ = nh_->create_service<Void>("vacuum_grp_off",
                          [&] (const shared_ptr<Void::Request>, shared_ptr<Void::Response> res) {
                              COUT("[Service called] vacuum_grp_off");
                              res->successed = vacuumGrpOff();
                          });

    srv_motor_pos_ctrl_ = nh_->create_service<PosVelCurCtrl>("motor_pos_ctrl",
                          [&] (const shared_ptr<PosVelCurCtrl::Request> req, shared_ptr<PosVelCurCtrl::Response> res) {
                              COUT("[Service called] motor_pos_ctrl, input: " << req->position << ", " << req->duration);
                              res->successed = motorPosCtrl(req->position, req->duration);
                          });

    // 속도·전류는 연결된 모델의 한계(/grp_state의 max_speed_rpm, cur_full_scale_ma)로 제한된다
    srv_motor_vel_ctrl_ = nh_->create_service<PosVelCurCtrl>("motor_vel_ctrl",
                          [&] (const shared_ptr<PosVelCurCtrl::Request> req, shared_ptr<PosVelCurCtrl::Response> res) {
                              COUT("[Service called] motor_vel_ctrl, input: " << req->velocity);
                              res->successed = motorVelCtrl(req->velocity);
                          });

    srv_motor_cur_ctrl_ = nh_->create_service<PosVelCurCtrl>("motor_cur_ctrl",
                          [&] (const shared_ptr<PosVelCurCtrl::Request> req, shared_ptr<PosVelCurCtrl::Response> res) {
                              COUT("[Service called] motor_cur_ctrl, input: " << req->current);
                              res->successed = motorCurCtrl(req->current);
                          });

    // KR-MATC
    srv_error_clear_ = nh_->create_service<Void>("error_clear",
                       [&] (const shared_ptr<Void::Request>, shared_ptr<Void::Response> res) {
                           COUT("[Service called] error_clear");
                           res->successed = errorClear();
                       });

    // GUI의 Impedance On/Off 버튼과 같이 모드를 바꾼 뒤 0.1 s 후 그리퍼를 다시 초기화(101)한다
    srv_impedance_on_ = nh_->create_service<Void>("impedance_on",
                        [&] (const shared_ptr<Void::Request>, shared_ptr<Void::Response> res) {
                            COUT("[Service called] impedance_on");
                            const bool ok = impedanceOn();
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                            res->successed = grpInitialize() && ok;
                        });

    srv_impedance_off_ = nh_->create_service<Void>("impedance_off",
                         [&] (const shared_ptr<Void::Request>, shared_ptr<Void::Response> res) {
                             COUT("[Service called] impedance_off");
                             const bool ok = impedanceOff();
                             std::this_thread::sleep_for(std::chrono::milliseconds(100));
                             res->successed = grpInitialize() && ok;
                         });

    srv_set_impedance_params_ = nh_->create_service<ImpedanceParams>("set_impedance_params",
                                [&] (const shared_ptr<ImpedanceParams::Request> req, shared_ptr<ImpedanceParams::Response> res) {
                                    COUT("[Service called] set_impedance_params, input: " << req->slave_num << ", "
                                         << req->stiffness_level);
                                    res->successed = setImpedanceParams(req->slave_num, req->stiffness_level);
                                });

    // 그리퍼 맵(216~218)은 MATC v3 전용 — 구버전 펌웨어에 연결돼 있으면 보내지 않고 false를 돌려준다
    srv_write_gripper_map_ = nh_->create_service<WriteGripperMap>("write_gripper_map",
                             [&] (const shared_ptr<WriteGripperMap::Request> req, shared_ptr<WriteGripperMap::Response> res) {
                                 COUT("[Service called] write_gripper_map, input: " << req->index << ", " << req->stroke_mm
                                      << ", " << req->force_n << ", " << req->payload_x10);
                                 res->successed = writeMapEntry(req->index, req->stroke_mm, req->force_n, req->payload_x10);
                             });

    // 217을 보내고 응답(reg 24~27)에 요청한 index가 돌아올 때까지 확인한 뒤 돌려준다
    srv_read_gripper_map_ = nh_->create_service<ReadGripperMap>("read_gripper_map",
                            [&] (const shared_ptr<ReadGripperMap::Request> req, shared_ptr<ReadGripperMap::Response> res) {
                                COUT("[Service called] read_gripper_map, input: " << req->index);
                                GripperMapEntry entry;
                                res->successed   = queryMapEntry(req->index, entry);
                                res->index       = entry.index;
                                res->stroke_mm   = entry.stroke_mm;
                                res->force_n     = entry.force_n;
                                res->payload_x10 = entry.payload_x10;
                            });

    // 펌웨어는 저장 결과를 상태 워드 bit 15(EEPROM Save Fail)로만 알려 준다. GUI 맵 임포트처럼 저장(약 0.4 s)이
    // 끝나 상태가 갱신될 시간을 기다린 뒤 bit 15를 확인한다 — 약 1.5 s 걸리고, 그동안 다른 서비스 요청은 기다린다.
    srv_save_gripper_map_ = nh_->create_service<Void>("save_gripper_map",
                            [&] (const shared_ptr<Void::Request>, shared_ptr<Void::Response> res) {
                                COUT("[Service called] save_gripper_map");
                                uint16_t status = 0;
                                res->successed = saveGripperMap();
                                if (res->successed) {
                                    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                                    res->successed = readStatusWord(status) && !(status & (0x01 << 15));
                                }
                            });

    // 장치 통신속도 변경(52, MATC v3) + GCS 재연결. GCS의 연결 설정(Modbus 탭)도 바뀌므로 UI 스레드
    // (MainWindow::remoteSetDeviceBaudrate)가 '장치 속도 변경' 버튼과 같은 절차로 처리하고, 이 스레드는 결과가 올 때까지
    // (보통 약 2 s, 실패하면 10 s 안팎, 최대 30 s) 기다렸다가 응답한다. 그동안 다른 서비스 요청은 기다린다.
    srv_set_modbus_baudrate_ = nh_->create_service<SetModbusBaudrate>("set_modbus_baudrate",
                               [&] (const shared_ptr<SetModbusBaudrate::Request> req, shared_ptr<SetModbusBaudrate::Response> res) {
                                   COUT("[Service called] set_modbus_baudrate, input: " << req->baudrate);
                                   unique_lock<mutex> lk(mutex_baud_);
                                   baud_done_ = false;
                                   lk.unlock();
                                   Q_EMIT deviceBaudrateRequested(req->baudrate);

                                   const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
                                   lk.lock();
                                   while (!baud_done_ && !flag_ros_stop_ && rclcpp::ok() &&
                                          (std::chrono::steady_clock::now() < deadline)) {
                                       cv_baud_.wait_for(lk, std::chrono::milliseconds(100));
                                   }
                                   res->successed = baud_done_ && baud_ok_;
                                   res->baudrate  = baud_done_ ? baud_now_ : 0;
                                   res->result    = baud_done_ ? baud_result_ : string("timeout");
                               });

    COUT("MATC ROS2 interface init. (node: MATC_Control_Interface, topic: grp_state)");
    ros_thread_ = std::thread(&DatcCommInterface::spinRos, this);
}

DatcCommInterface::~DatcCommInterface() {
    // 서비스 스레드를 먼저 멈춘다 — 처리 중인 요청이 폴링 스레드의 마지막 모터 정지·Modbus 해제와 겹치지 않게
    flag_ros_stop_ = true;
    if (ros_thread_.joinable()) {
        ros_thread_.join();
    }

    flag_program_stop_ = true;
    wait(1000);

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    modbusRelease();
}

bool DatcCommInterface::init(const char *port_name, uint16_t slave_address, int baudrate) {
    if (!modbusInit(port_name, slave_address, baudrate)) {
        return false;
    }

    COUT("MATC interface init.");

    return true;
}

void DatcCommInterface::reportDeviceBaudrate(bool ok, int baudrate, const string &result) {
    {
        unique_lock<mutex> lk(mutex_baud_);
        baud_ok_     = ok;
        baud_now_    = baudrate;
        baud_result_ = result;
        baud_done_   = true;
    }
    cv_baud_.notify_all();
}

void DatcCommInterface::pubTopic() {
    const DatcStatus status = getDatcStatus();
    const DeviceInfo info   = getDeviceInfo();
    GripperMsg msg;

    msg.motor_position  = status.motor_pos;
    msg.motor_velocity  = status.motor_vel;
    msg.motor_current   = status.motor_cur;
    msg.finger_position = status.finger_pos;

    msg.motor_enabled       = status.enable;
    msg.gripper_initialized = status.initialize;
    msg.position_ctrl_mode  = status.motor_pos_ctrl;
    msg.velocity_ctrl_mode  = status.motor_vel_ctrl;
    msg.current_ctrl_mode   = status.motor_cur_ctrl;
    msg.grp_opened          = status.grp_open;
    msg.grp_closed          = status.grp_close;
    msg.motor_fault         = status.fault;

    // KR-MATC — 일반판 TCP 상태 JSON과 같은 값
    msg.states            = status.states;
    msg.voltage           = status.voltage;
    msg.fault_now         = status.fault_now;
    msg.fault_occurred    = status.fault_occurred;
    msg.gripper_direction = status.gripper_direction;
    msg.object_grasp      = status.object_grasp;
    msg.grip_hold         = status.grip_hold;
    msg.slip_detected     = status.slip_detected;
    msg.gobj_result       = status.gobj_result;
    msg.grip_force_n      = status.grip_force_calc_n;
    msg.comm_overrun      = status.comm_overrun;
    msg.eep_save_fail     = status.eep_save_fail;

    msg.gripper_connected   = status.gripper_connected;
    msg.gripper_id          = status.gripper_id;
    msg.gripper_stroke_mm   = status.gripper_stroke_mm;
    msg.gripper_force_n     = status.gripper_force_n;
    msg.gripper_payload_x10 = status.gripper_payload_x10;
    msg.legacy_firmware     = status.legacy_map;

    // 장치 정보 (regs 28–33, MATC v3.01~). 0 = 미지원(이전 펌웨어) 또는 아직 읽기 전
    msg.device_model           = info.valid ? info.model : 0;
    msg.fw_version             = info.valid ? info.fw_version : 0;
    msg.max_speed_rpm          = getVelMaxRpm();   // 속도 제어(6) 상한 — 실제로 적용하는 값
    msg.cur_full_scale_ma      = getCurMaxMa();    // 전류 제어(7) 상한
    msg.elangle_calibrated     = info.valid && info.elangleCalibrated();
    msg.elangle_model_mismatch = info.valid && info.elangleModelMismatch();

    publisher_grp_state_->publish(msg);
}

// 서비스 스레드 — 일반판의 TCP 명령 스레드(recvCommand)에 해당한다.
// 폴링이 멈춘 동안(UI의 재연결·속도 검색)은 요청을 처리하지 않고 기다리게 한다.
void DatcCommInterface::spinRos() {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(nh_);

    while (!flag_ros_stop_ && rclcpp::ok()) {
        if (poll_pause_.load() > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        executor.spin_once(std::chrono::milliseconds(10));
    }

    // Ctrl+C 등으로 ROS2가 먼저 끝나면 프로그램도 닫는다(통신이 멈춘 창만 남지 않게)
    if (!flag_ros_stop_) {
        COUT("ROS2 shutdown - closing KR_GCS");
        Q_EMIT rosShutdown();
        QMetaObject::invokeMethod(QCoreApplication::instance(), [] { QCoreApplication::quit(); }, Qt::QueuedConnection);
    }
}

// 폴링 일시정지 (헤더 설명 참고). 카운터를 올린 뒤 mutex_poll_을 한 번 잡아, 이미 시작된 주기가
// 끝나기를 기다린다. 그 뒤로 폴링 스레드는 카운터를 보고 주기를 건너뛰므로 락 경쟁이 없다.
void DatcCommInterface::pausePolling() {
    ++poll_pause_;
    unique_lock<mutex> lg(mutex_poll_);
}

void DatcCommInterface::resumePolling() {
    if (poll_pause_.load() > 0) {
        --poll_pause_;
    }
}

// Main loop
void DatcCommInterface::run() {
    auto cycleFn([&] () {
        if (poll_pause_.load() > 0) {
            return;
        }
        unique_lock<mutex> lg(mutex_poll_);
        if (poll_pause_.load() > 0) {   // 락을 얻는 사이 일시정지됐을 수 있다
            return;
        }
        if (mbc_.getConnectionState()) {
            // 읽기에 성공한 주기에만 발행한다 — 장치가 응답하지 않으면 토픽이 끊긴다
            if (readDatcData() && rclcpp::ok()) {
                pubTopic();
            }
        }
    });

    const std::chrono::duration<double> period(1 / (double) kFreq);

    while(!flag_program_stop_) {
        auto time_start = std::chrono::steady_clock::now();

        cycleFn();

        // Calculate the time elapsed and sleep for the remaining time
        auto time_end = std::chrono::steady_clock::now();
        auto time_elapsed = time_end - time_start;

        if (time_elapsed < period) {
            std::this_thread::sleep_for(period - time_elapsed);
        } else {
            // 주기가 늘어진 경우(시간 초과·저속 통신)에도 틈을 둬, 명령을 보내려고 mutex_comm_을
            // 기다리는 다른 스레드가 락을 얻을 수 있게 한다
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    motorDisable();
    modbusRelease();
}
