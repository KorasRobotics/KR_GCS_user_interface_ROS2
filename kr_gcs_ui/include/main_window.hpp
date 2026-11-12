/**
 * @file main_window.hpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief
 * @version 1.0
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QTimer>
#include <QElapsedTimer>
#include <QLabel>
#include <QLineEdit>
#include <QComboBox>
#include <QList>
#include <QMainWindow>
#include <QShortcut>
#include <QKeySequence>

#include <iostream>
#include <math.h>

#include "datc_comm_interface.hpp"
#include "ui_main_window.h"
#include "custom_widget.hpp"
#include "firmware_updater.hpp"

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(WIN64) || defined(_WIN64) || defined(__WIN64__)
#include <windows.h>
#include <setupapi.h>
#else
#include <fstream>
#include <dirent.h>
#include <cstring>
#endif

using namespace std;

namespace gripper_ui {

enum class WidgetSeq {
    MODBUS_WIDGET         = 0,
    DATC_CTRL_WIDGET      = 1,
    ADVANCED_CTRL_WIDGET  = 2,
    IMPEDANCE_CTRL_WIDGET = 3,
    DEV_TAB_WIDGET        = 4,
    TCP_WIDGET            = 5,
};

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, bool &success, QWidget *parent = 0);
    ~MainWindow();

public Q_SLOTS:
    void timerCallback();

    // Enable & disable
    void datcEnable();
    void datcDisable();

    // Datc control
    void datcFingerPosCtrl();
    void datcFingerPosCtrl2();
    void datcMotorVelCtrl();
    void datcMotorCurCtrl();

    void datcInit();
    void datcOpen();
    void datcClose();
    void datcStop();
    void datcVacuumGrpOn();
    void datcVacuumGrpOff();

    void datcSetTorque();
    void datcSetSpeed();

    // Impedance related functions
    void datcImpedanceOn();
    void datcImpedanceOff();
    void datcSetImpedanceParams();

    // Modbus RTU related
    void initModbus();
    void releaseModbus();
    void changeSlaveAddress();
    void setSlaveAddr();
    void setDeviceBaudrate();    // 장치(펌웨어) Modbus 통신속도 변경 + 새 속도로 자동 재연결
    void autoDetectBaudrate();   // 장치 통신속도 자동 검색 (속도를 모를 때 복구용)

    // Repeat test (Modbus 탭) — 열기/닫기 반복 시험
    void repeatStart();
    void repeatStop();
    void repeatReset();

    // Dev ui related
    void dev_setGainP();
    void dev_setGainV();
    void dev_setGainC();
    void dev_customCmd();
    void dev_resetPos();
    void dev_setElecAngle();
    void dev_importGripperMap();   // 그리퍼 매핑표 CSV 일괄 임포트 → writeMapEntry + saveGripperMap

    // Firmware update (Dev tab)
    void dev_fwBrowse();
    void dev_fwStart();
    void dev_fwCancel();
    void dev_fwOnLog(QString msg);
    void dev_fwOnProgress(int pct);
    void dev_fwOnFinished(bool ok, QString msg);

#ifndef RCLCPP__RCLCPP_HPP_
    // TCP comm. related functions
    void startTcpComm();
    void stopTcpComm();
#endif
    // 원격 장치 통신속도 변경(TCP 명령 52, ROS2 set_modbus_baudrate): 확인 창 없이 변경 + 새 속도로 재연결
    void remoteSetDeviceBaudrate(int baudrate);

    // Auto mapping function
    void on_pushButton_select_modbus_clicked();
    void on_pushButton_select_datc_ctrl_clicked();
    void on_pushButton_select_adv_clicked();
    void on_pushButton_select_tcp_clicked();
    void on_pushButton_select_imped_ctrl_clicked();
    void on_pushButton_modbus_refresh_clicked();

    // Btn for dev tab
    void on_pushButton_logo_clicked();
    void on_pushButton_hidden_clicked();

    // Serial port find function
    std::vector<std::string> getSerialPortLists();

private:
    Ui::MainWindow *ui_;

    ModbusWidget        *modbus_widget_;
    DatcCtrlWidget      *datc_ctrl_widget_;
    TcpWidget           *tcp_widget_;
    AdvancedCtrlWidget  *advanced_ctrl_widget_;
    ImpedanceCtrlWidget *impedance_ctrl_widget_;
    DevTabWidget        *dev_tab_widget_;

    QString menu_btn_active_str_, menu_btn_inactive_str_;
    QString btn_active_str_, btn_inactive_str_;

    QTimer *timer_;
    DatcCommInterface *datc_interface_;

    bool dev_tab_accessibility_;

    FirmwareUpdater *fw_updater_;
    void buildCalibrationTab();   // Dev 탭 "Encoder / Map" 서브탭: 엔코더 보정(219~221)·맵 조회(217)·Error Clear
    bool fwCheckModel(const QString &bin, bool device_alive);   // 업데이트 파일과 장치 모델 비교 (v3.01~)
    void fwAbortStart(const QString &reason);                  // 업데이트 시작 전 중단 → UI 복구
    QLabel *label_dev_info_ = nullptr;   // Modbus 탭: 연결된 장치 모델·FW 버전·제어 한계·전기각 상태
    QComboBox       *combo_dev_baud_ = nullptr;   // 장치(펌웨어) 통신속도 선택
    bool reconnectAt(int baudrate);               // 지정 속도로 재연결 후 응답 확인
    // 장치 통신속도 변경 공통 절차(52 전송 → 새 속도로 재연결, 안 되면 이전 속도로) — '장치 속도 변경' 버튼과
    // 원격 명령(TCP 52, ROS2 set_modbus_baudrate)이 쓴다
    enum class BaudChange { CHANGED, REVERTED, NO_RESPONSE, NOT_SENT };
    BaudChange changeDeviceBaudrate(int new_baud, int old_baud);
    bool baud_change_busy_ = false;               // 속도 변경·자동 검색 중(확인 창 포함) — 그사이 온 원격 52는 보내지 않는다
    void dev_fwVerifyApp(int baud, int tries_left);   // 펌웨어 업데이트 후 새 앱의 Modbus 응답 확인
    bool fw_reset_sent_ = false;                      // 이번 업데이트에서 SYSTEM_RESET을 보냈는지
    int  fw_app_baud_   = 115200;                     // 업데이트 전 앱 Modbus 통신속도

    // 구버전 펌웨어(DatcStatus::legacy_map): 손가락 위치·파지력 칸을 % 단위로 바꿔 쓴다
    void     applyFingerUnits(bool legacy);           // 위치(mm↔%)·파지력(N↔%) 칸 단위·범위 전환 (바뀔 때만)
    uint16_t fingerPosCommand(double value) const;    // 위치 칸 값 → 명령 104 값 (V3: mm, 구버전: % → ‰)
    bool     finger_units_legacy_ = false;            // 현재 위치·파지력 칸이 구버전(%) 단위인지
    // 구버전 펌웨어에 없는 V3 전용 기능(isV3OnlyCommand, 부트로더 업데이트)의 위젯을 막는다 (바뀔 때만)
    void     applyLegacyFeatureLock(bool legacy);
    QList<QWidget *> v3_only_widgets_;                // Modbus 탭 통신속도 변경·맵 임포트, Encoder / Map 탭 그룹
    bool     legacy_feature_lock_ = false;            // 지금 막혀 있는지 (펌웨어 업데이트 시작 버튼도 따름)

    // Repeat test — timerCallback(100 ms)마다 repeatStep으로 한 단계씩 진행
    enum class RepeatState { IDLE, CMD_OPEN, WAIT_OPEN, OPEN_DELAY, CMD_CLOSE, WAIT_CLOSE, CLOSE_DELAY };
    void repeatStep(const DatcStatus &status);
    void repeatHalt(const QString &error_text, const QString &log_msg);
    void repeatLog(const QString &msg);
    uint32_t repeatTarget() const;   // Target Count (0 = 무제한)
    void repeatUpdateCount();        // Repeat Count 표시: "n" 또는 "n / 목표"
    void repeatSetStatus(const QString &text, bool done = false);   // Error Code 칸 (done = 초록색)
    bool          repeat_running_    = false;
    RepeatState   repeat_state_      = RepeatState::IDLE;
    bool          repeat_open_done_  = false;   // 이번 사이클 열림 완료
    bool          repeat_close_done_ = false;   // 이번 사이클 닫힘 완료
    uint32_t      repeat_count_      = 0;       // 완료한 열림→닫힘 사이클 수
    QElapsedTimer repeat_timer_;                // 현재 단계에 들어온 뒤 경과 시간
    QElapsedTimer repeat_progress_timer_;       // 열림/닫힘 대기 중 마지막으로 움직인 뒤 경과 시간
    uint16_t      repeat_progress_pos_ = 0;     // 마지막으로 움직였다고 본 위치 [‰]
    uint16_t      repeat_prev_pos_   = 0;       // 직전 틱의 핑거 위치 [‰]
    uint16_t      repeat_close_start_pos_ = 0;  // 닫기 지령 시점 핑거 위치 [‰]
    int           repeat_stable_cnt_ = 0;       // 닫는 중 정지 상태가 이어진 틱 수
};

}
#endif // ur_ui_MAIN_WINDOW_H
