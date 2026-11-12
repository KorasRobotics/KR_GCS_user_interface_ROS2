/**
 * @file main_window.cpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief Implementation for the qt gui_->
 * @version 1.0
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "main_window.hpp"

#include <QFileDialog>
#include <QFileInfo>
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QPushButton>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QThread>
#include <QApplication>
#include <QPlainTextEdit>
#include <QScrollBar>
#include <QTime>
#include <QGroupBox>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QProgressBar>
#include <QElapsedTimer>
#include <QScrollArea>
#include <QScopeGuard>
#include <memory>

using namespace Qt;

namespace gripper_ui {

// Repeat test 판정값 — 반복시험용 GCS(kr_gcs_user_interface_v2_repeat, 2026-07-03 빌드)와 동일.
// 단, 원본의 고정 5 s 도달 제한은 MATC-20/30(최대속도 50 rad/s, MATC-10의 절반)이나 낮은 속도 설정에서
// 정상 이동도 넘겨 버려, "위치가 3 s 동안 안 움직이면" 타임아웃으로 바꿨다(전체 상한 120 s).
// finger_pos는 ‰(0=닫힘, 1000=열림), 전류는 mA(reg12), 속도는 rpm(reg13).
namespace {
const qint64   kRepeatStallTimeoutMs = 3000;   // 열림/닫힘 대기 중 이 시간 동안 위치 변화가 없으면 타임아웃
const int      kRepeatProgressPosTol = 5;      // 5 ‰ 이상 움직여야 진행으로 인정
const qint64   kRepeatMoveTimeoutMs  = 120000; // 열기/닫기 지령 후 전체 상한
const uint16_t kRepeatOpenPosMin    = 980;   // 98 % 이상 → 열림 도달
const uint16_t kRepeatClosePosMax   = 30;    // 3 % 이하 → (물체 없이) 닫힘 완료
const qint64   kRepeatSettleMs      = 300;   // 닫기 지령 후 이 시간부터 정지 판정
const int      kRepeatStablePosTol  = 2;     // 틱 간 위치 변화 ≤ 2 ‰
const int      kRepeatStableVelRpm  = 5;     // |속도| ≤ 5 rpm
const int      kRepeatGraspCurMa    = 200;   // |전류| ≥ 200 mA → 물체 파지
const int      kRepeatStableTicks   = 3;     // 정지 상태가 3틱(≈300 ms) 이어지면 확정
const int      kRepeatGraspMinTravel = 20;   // 닫기 시작 위치에서 2 % 이상 닫혀야 파지로 인정
}

MainWindow::MainWindow(int argc, char **argv, bool &success, QWidget *parent) : QMainWindow(parent) {
    ui_ = new Ui::MainWindow();

    modbus_widget_         = new ModbusWidget(this);
    datc_ctrl_widget_      = new DatcCtrlWidget(this);
    tcp_widget_            = new TcpWidget(this);
    advanced_ctrl_widget_  = new AdvancedCtrlWidget(this);
    impedance_ctrl_widget_ = new ImpedanceCtrlWidget(this);
    dev_tab_widget_        = new DevTabWidget(this);

    ui_->setupUi(this);

    //setWindowIcon(QIcon(":/images/icon.png"));
    datc_interface_ = new DatcCommInterface(argc, argv);

    // Stacked widget
    ui_->stackedWidget->addWidget(modbus_widget_);
    ui_->stackedWidget->addWidget(datc_ctrl_widget_);
    ui_->stackedWidget->addWidget(advanced_ctrl_widget_);
    ui_->stackedWidget->addWidget(impedance_ctrl_widget_);
    ui_->stackedWidget->addWidget(dev_tab_widget_);

#ifndef RCLCPP__RCLCPP_HPP_
    ui_->stackedWidget->addWidget(tcp_widget_);
#endif

    ui_->stackedWidget->setCurrentIndex((int) WidgetSeq::MODBUS_WIDGET);

    // GUI setting
    menu_btn_active_str_   = "background-color:#FFFFFF;color:#000000";
    menu_btn_inactive_str_ = "background-color:#888888;color:#FFFFFF";

    btn_active_str_   = "background-color:#FFFFFF;color:#888888;border-style:outset;border-width:7;";
    btn_inactive_str_ = "background-color:#BBBBBB;color:#888888;border-style:inset;border-width:7;";

    // Initial value setting
    int finger_pos_init_value = 50;
    int torque_init_value = 10;     // 그리퍼 미연결 기본 파지력 [N] (5~10N 범위의 100%=10N)
    int speed_init_value  = 75;

    datc_ctrl_widget_->ui_.horizontalSlider_finger_pos->setValue(finger_pos_init_value);
    datc_ctrl_widget_->ui_.verticalSlider_torque->setValue      (torque_init_value);
    datc_ctrl_widget_->ui_.verticalSlider_speed ->setValue      (speed_init_value);

    datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setValue((double) finger_pos_init_value);
    datc_ctrl_widget_->ui_.doubleSpinBox_torque->setValue    ((double) torque_init_value);
    datc_ctrl_widget_->ui_.doubleSpinBox_speed->setValue     ((double) speed_init_value);

    // Initial values setting of impedance ctrl widget
    impedance_ctrl_widget_->ui_.horizontalSlider_finger_pos->setValue(finger_pos_init_value);
    impedance_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setValue((double) finger_pos_init_value);

    // 핑거 위치를 mm 단위로 표시 (0mm=닫힘, 그리퍼 stroke mm=열림). 범위는 그리퍼 연결 시
    // stroke로 갱신(updateStatus). 분해능 1mm. 펌웨어가 mm를 stroke로 정규화해 이동.
    datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setSuffix(" mm");
    datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setDecimals(0);
    datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setSingleStep(1.0);
    impedance_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setSuffix(" mm");
    impedance_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setDecimals(0);
    impedance_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setSingleStep(1.0);

    // 핑거 위치 기본 범위 0~100mm (그리퍼 연결 시 stroke로 갱신). 시작 시(미연결) 비활성 —
    // updateStatus가 연결 상태에 따라 토글한다.
    datc_ctrl_widget_->ui_.horizontalSlider_finger_pos->setRange(0, 100);
    datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setRange(0.0, 100.0);
    impedance_ctrl_widget_->ui_.horizontalSlider_finger_pos->setRange(0, 100);
    impedance_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setRange(0.0, 100.0);
    datc_ctrl_widget_->ui_.horizontalSlider_finger_pos->setEnabled(false);
    datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setEnabled(false);
    datc_ctrl_widget_->ui_.pushButton_set_position->setEnabled(false);
    impedance_ctrl_widget_->ui_.horizontalSlider_finger_pos->setEnabled(false);
    impedance_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setEnabled(false);

    // 그립 파지력을 N 단위로 표시/지령 (이전: 토크 %). 사용 범위는 정격 파지력의 50~100%이며
    // 그리퍼 연결 시 [50%·정격, 정격]N으로 갱신(updateStatus). 펌웨어가 종류별로 N→전류 환산.
    datc_ctrl_widget_->ui_.doubleSpinBox_torque->setSuffix(" N");
    datc_ctrl_widget_->ui_.doubleSpinBox_torque->setDecimals(0);
    datc_ctrl_widget_->ui_.doubleSpinBox_torque->setSingleStep(1.0);

    // 시작 시(그리퍼 미연결) 파지력 컨트롤 비활성 — updateStatus가 연결 상태에 따라 토글한다.
    datc_ctrl_widget_->ui_.verticalSlider_torque->setEnabled(false);
    datc_ctrl_widget_->ui_.doubleSpinBox_torque->setEnabled(false);
    datc_ctrl_widget_->ui_.pushButton_set_torque->setEnabled(false);

    // Combo box setting
    modbus_widget_->ui_.comboBox_serial_port->setEnabled(true);
    modbus_widget_->ui_.comboBox_serial_port->lineEdit()->setAlignment(Qt::AlignCenter);
    auto font = modbus_widget_->ui_.comboBox_serial_port->lineEdit()->font();
    font.setPointSize(14);
    font.setBold(true);
    modbus_widget_->ui_.comboBox_serial_port->lineEdit()->setFont(font);

    on_pushButton_modbus_refresh_clicked();

    modbus_widget_->ui_.comboBox_baudrate->setEnabled(true);
    modbus_widget_->ui_.comboBox_baudrate->addItems({"9600", "19200", "38400", "57600", "115200"});
    modbus_widget_->ui_.comboBox_baudrate->setCurrentIndex(4);

    // Check box setting
    QString checkbox_qstr = "QCheckBox::indicator {width:25px; height: 25px;}";

    tcp_widget_->ui_.checkBox_tcp_send_status->setStyleSheet(checkbox_qstr);

    checkbox_qstr = "QCheckBox::indicator {width:20px; height: 20px;}";

    advanced_ctrl_widget_->ui_.checkBox_motor_speed_reverse  ->setStyleSheet(checkbox_qstr);
    advanced_ctrl_widget_->ui_.checkBox_motor_current_reverse->setStyleSheet(checkbox_qstr);

    // Label
    advanced_ctrl_widget_->ui_.label_motor_speed  ->setText("(100 % : " + QString::number(kVelMax) + " rpm)");
    advanced_ctrl_widget_->ui_.label_motor_current->setText("(100 % : " + QString::number(kCurMax) + " mA)");

    // DATC control related btn
    QObject::connect(datc_ctrl_widget_->ui_.pushButton_cmd_enable  , SIGNAL(clicked()), this, SLOT(datcEnable()));
    QObject::connect(datc_ctrl_widget_->ui_.pushButton_cmd_disable , SIGNAL(clicked()), this, SLOT(datcDisable()));
    QObject::connect(datc_ctrl_widget_->ui_.pushButton_set_position, SIGNAL(clicked()), this, SLOT(datcFingerPosCtrl()));

    QObject::connect(datc_ctrl_widget_->ui_.pushButton_cmd_initialize    , SIGNAL(clicked()), this, SLOT(datcInit()));
    QObject::connect(datc_ctrl_widget_->ui_.pushButton_cmd_grp_open      , SIGNAL(clicked()), this, SLOT(datcOpen()));
    QObject::connect(datc_ctrl_widget_->ui_.pushButton_cmd_grp_close     , SIGNAL(clicked()), this, SLOT(datcClose()));
    QObject::connect(datc_ctrl_widget_->ui_.pushButton_cmd_grp_stop      , SIGNAL(clicked()), this, SLOT(datcStop()));
    QObject::connect(datc_ctrl_widget_->ui_.pushButton_cmd_grp_vacuum_on , SIGNAL(clicked()), this, SLOT(datcVacuumGrpOn()));
    QObject::connect(datc_ctrl_widget_->ui_.pushButton_cmd_grp_vacuum_off, SIGNAL(clicked()), this, SLOT(datcVacuumGrpOff()));

    QObject::connect(datc_ctrl_widget_->ui_.pushButton_set_torque, SIGNAL(clicked()), this, SLOT(datcSetTorque()));
    QObject::connect(datc_ctrl_widget_->ui_.pushButton_set_speed , SIGNAL(clicked()), this, SLOT(datcSetSpeed()));

    // Advanced control related btn
    QObject::connect(advanced_ctrl_widget_->ui_.pushButton_cmd_enable  , SIGNAL(clicked()), this, SLOT(datcEnable()));
    QObject::connect(advanced_ctrl_widget_->ui_.pushButton_cmd_disable , SIGNAL(clicked()), this, SLOT(datcDisable()));

    QObject::connect(advanced_ctrl_widget_->ui_.pushButton_cmd_initialize    , SIGNAL(clicked()), this, SLOT(datcInit()));
    QObject::connect(advanced_ctrl_widget_->ui_.pushButton_cmd_grp_open      , SIGNAL(clicked()), this, SLOT(datcOpen()));
    QObject::connect(advanced_ctrl_widget_->ui_.pushButton_cmd_grp_close     , SIGNAL(clicked()), this, SLOT(datcClose()));
    QObject::connect(advanced_ctrl_widget_->ui_.pushButton_cmd_grp_stop      , SIGNAL(clicked()), this, SLOT(datcStop()));
    QObject::connect(advanced_ctrl_widget_->ui_.pushButton_cmd_grp_vacuum_on , SIGNAL(clicked()), this, SLOT(datcVacuumGrpOn()));
    QObject::connect(advanced_ctrl_widget_->ui_.pushButton_cmd_grp_vacuum_off, SIGNAL(clicked()), this, SLOT(datcVacuumGrpOff()));

    QObject::connect(advanced_ctrl_widget_->ui_.pushButton_set_motor_speed  , SIGNAL(clicked()), this, SLOT(datcMotorVelCtrl()));
    QObject::connect(advanced_ctrl_widget_->ui_.pushButton_set_motor_current, SIGNAL(clicked()), this, SLOT(datcMotorCurCtrl()));

    // Modbus RTU related btn
    QObject::connect(modbus_widget_->ui_.pushButton_modbus_start, SIGNAL(clicked()), this, SLOT(initModbus()));
    QObject::connect(modbus_widget_->ui_.pushButton_modbus_stop , SIGNAL(clicked()), this, SLOT(releaseModbus()));
    QObject::connect(modbus_widget_->ui_.pushButton_modbus_slave_change  , SIGNAL(clicked()), this, SLOT(changeSlaveAddress()));
    QObject::connect(modbus_widget_->ui_.pushButton_modbus_set_slave_addr, SIGNAL(clicked()), this, SLOT(setSlaveAddr()));

    // Repeat test (Modbus 탭 오른쪽 열)
    QObject::connect(modbus_widget_->ui_.pushButton_repeat_start, SIGNAL(clicked()), this, SLOT(repeatStart()));
    QObject::connect(modbus_widget_->ui_.pushButton_repeat_reset, SIGNAL(clicked()), this, SLOT(repeatReset()));
    QObject::connect(modbus_widget_->ui_.pushButton_repeat_stop , SIGNAL(clicked()), this, SLOT(repeatStop()));
    // QOverload: Qt5에는 valueChanged(const QString &)도 있다 (ROS2판은 Qt5로도 빌드)
    QObject::connect(modbus_widget_->ui_.doubleSpinBox_repeat_target, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                     this, [this](double) { repeatUpdateCount(); });
    QObject::connect(modbus_widget_->ui_.pushButton_repeat_command_status_clear, SIGNAL(clicked()),
                     modbus_widget_->ui_.plainTextEdit_repeat_command_status, SLOT(clear()));

    // Impedance control related btn
    QObject::connect(impedance_ctrl_widget_->ui_.pushButton_cmd_impedance_on       , SIGNAL(clicked()), this, SLOT(datcImpedanceOn()));
    QObject::connect(impedance_ctrl_widget_->ui_.pushButton_cmd_impedance_off      , SIGNAL(clicked()), this, SLOT(datcImpedanceOff()));
    QObject::connect(impedance_ctrl_widget_->ui_.pushButton_set_impedance_params   , SIGNAL(clicked()), this, SLOT(datcSetImpedanceParams()));

    QObject::connect(impedance_ctrl_widget_->ui_.pushButton_cmd_enable  , SIGNAL(clicked()), this, SLOT(datcEnable()));
    QObject::connect(impedance_ctrl_widget_->ui_.pushButton_cmd_disable , SIGNAL(clicked()), this, SLOT(datcDisable()));
    QObject::connect(impedance_ctrl_widget_->ui_.pushButton_set_position, SIGNAL(clicked()), this, SLOT(datcFingerPosCtrl2()));

    QObject::connect(impedance_ctrl_widget_->ui_.pushButton_cmd_initialize    , SIGNAL(clicked()), this, SLOT(datcInit()));
    QObject::connect(impedance_ctrl_widget_->ui_.pushButton_cmd_grp_open      , SIGNAL(clicked()), this, SLOT(datcOpen()));
    QObject::connect(impedance_ctrl_widget_->ui_.pushButton_cmd_grp_close     , SIGNAL(clicked()), this, SLOT(datcClose()));
    QObject::connect(impedance_ctrl_widget_->ui_.pushButton_cmd_grp_stop      , SIGNAL(clicked()), this, SLOT(datcStop()));
    QObject::connect(impedance_ctrl_widget_->ui_.pushButton_cmd_grp_vacuum_on , SIGNAL(clicked()), this, SLOT(datcVacuumGrpOn()));
    QObject::connect(impedance_ctrl_widget_->ui_.pushButton_cmd_grp_vacuum_off, SIGNAL(clicked()), this, SLOT(datcVacuumGrpOff()));

    // Dev ui related btn
    QObject::connect(dev_tab_widget_->ui_.pushButton_set_p_gain    , SIGNAL(clicked()), this, SLOT(dev_setGainP()));
    QObject::connect(dev_tab_widget_->ui_.pushButton_set_v_gain    , SIGNAL(clicked()), this, SLOT(dev_setGainV()));
    QObject::connect(dev_tab_widget_->ui_.pushButton_set_c_gain    , SIGNAL(clicked()), this, SLOT(dev_setGainC()));
    QObject::connect(dev_tab_widget_->ui_.pushButton_custom_cmd    , SIGNAL(clicked()), this, SLOT(dev_customCmd()));
    QObject::connect(dev_tab_widget_->ui_.pushButton_reset_position, SIGNAL(clicked()), this, SLOT(dev_resetPos()));
    QObject::connect(dev_tab_widget_->ui_.pushButton_set_elec_angle, SIGNAL(clicked()), this, SLOT(dev_setElecAngle()));

    // 장치(펌웨어) Modbus 통신속도 변경 줄 — Modbus 탭, 슬레이브 주소 설정 줄 아래에 코드로 추가(.ui 미변경).
    {
        QHBoxLayout *row = new QHBoxLayout();
        QLabel *lbl = new QLabel("장치 통신속도", modbus_widget_);
        combo_dev_baud_ = new QComboBox(modbus_widget_);
        combo_dev_baud_->addItems({"9600", "19200", "38400", "57600", "115200"});
        combo_dev_baud_->setCurrentIndex(4);
        QPushButton *btnSet  = new QPushButton("장치 속도 변경", modbus_widget_);
        QPushButton *btnScan = new QPushButton("속도 자동 검색", modbus_widget_);
        btnSet->setMinimumHeight(30);
        btnScan->setMinimumHeight(30);
        row->addWidget(lbl);
        row->addWidget(combo_dev_baud_, 1);
        row->addWidget(btnSet, 1);
        row->addWidget(btnScan, 1);

        QVBoxLayout *vl = modbus_widget_->ui_.verticalLayout;
        const int idx = (vl != nullptr) ? vl->indexOf(modbus_widget_->ui_.horizontalLayout) : -1;
        if ((vl != nullptr) && (idx >= 0)) {
            vl->insertLayout(idx + 1, row);
        } else if (vl != nullptr) {
            vl->addLayout(row);
        }
        QObject::connect(btnSet,  SIGNAL(clicked()), this, SLOT(setDeviceBaudrate()));
        QObject::connect(btnScan, SIGNAL(clicked()), this, SLOT(autoDetectBaudrate()));
        v3_only_widgets_ << combo_dev_baud_ << btnSet;   // 52는 V3 전용 (자동 검색은 읽기만 하므로 구버전도 된다)

        // 연결된 장치 정보(모델·FW 버전·제어 한계·전기각 상태, regs 28–33) — 통신속도 줄 바로 아래
        label_dev_info_ = new QLabel("장치 정보: -", modbus_widget_);
        label_dev_info_->setWordWrap(true);
        if ((vl != nullptr) && (idx >= 0)) {
            vl->insertWidget(idx + 2, label_dev_info_);
        } else if (vl != nullptr) {
            vl->addWidget(label_dev_info_);
        }
    }

    // 그리퍼 매핑표 CSV 임포트 버튼 — Modbus 탭 하단에 코드로 추가(.ui 미변경).
    {
        QPushButton *importBtn = new QPushButton("그리퍼 맵 CSV 임포트 (일괄 쓰기+저장)", modbus_widget_);
        importBtn->setMinimumHeight(30);
        if (modbus_widget_->ui_.verticalLayout != nullptr) {
            modbus_widget_->ui_.verticalLayout->addWidget(importBtn);
        } else if (modbus_widget_->layout() != nullptr) {
            modbus_widget_->layout()->addWidget(importBtn);
        }
        QObject::connect(importBtn, SIGNAL(clicked()), this, SLOT(dev_importGripperMap()));
        v3_only_widgets_ << importBtn;   // 216·218은 V3 전용
    }

    // Firmware update (Dev tab) — bootloader-based .bin upload
    fw_updater_ = new FirmwareUpdater(this);
    QObject::connect(dev_tab_widget_->ui_.pushButton_fw_browse, SIGNAL(clicked()), this, SLOT(dev_fwBrowse()));
    QObject::connect(dev_tab_widget_->ui_.pushButton_fw_start , SIGNAL(clicked()), this, SLOT(dev_fwStart()));
    QObject::connect(dev_tab_widget_->ui_.pushButton_fw_cancel, SIGNAL(clicked()), this, SLOT(dev_fwCancel()));
    QObject::connect(fw_updater_, &FirmwareUpdater::log     , this, &MainWindow::dev_fwOnLog     , Qt::QueuedConnection);
    QObject::connect(fw_updater_, &FirmwareUpdater::progress, this, &MainWindow::dev_fwOnProgress, Qt::QueuedConnection);
    QObject::connect(fw_updater_, &FirmwareUpdater::finished, this, &MainWindow::dev_fwOnFinished, Qt::QueuedConnection);

    buildCalibrationTab();

#ifndef RCLCPP__RCLCPP_HPP_
    // TCP socket commiunication related btn
    QObject::connect(tcp_widget_->ui_.pushButton_tcp_start, SIGNAL(clicked()), this, SLOT(startTcpComm()));
    QObject::connect(tcp_widget_->ui_.pushButton_tcp_stop , SIGNAL(clicked()), this, SLOT(stopTcpComm()));
#else
    ui_->pushButton_select_tcp->setHidden(true);
#endif
    // 원격 장치 통신속도 변경(TCP 명령 52, ROS2 set_modbus_baudrate)은 새 속도로 다시 연결해야 하므로 UI 스레드에서 처리한다
    QObject::connect(datc_interface_, &DatcCommInterface::deviceBaudrateRequested,
                     this, &MainWindow::remoteSetDeviceBaudrate, Qt::QueuedConnection);

    timer_ = new QTimer(this);
    connect(timer_, SIGNAL(timeout()), this, SLOT(timerCallback()));
    timer_->start(100); // msec

    datc_interface_->start();
    success = true;

    dev_tab_accessibility_ = false;

    // Ctrl+Shift+D 단축키로 Dev Tab 바로 진입 (숨김 버튼 5회 + 로고 3회 클릭 대체)
    auto *dev_shortcut = new QShortcut(QKeySequence("Ctrl+Shift+D"), this);
    connect(dev_shortcut, &QShortcut::activated, this, [this]() {
        dev_tab_accessibility_ = true;
        ui_->stackedWidget->setCurrentIndex((int) WidgetSeq::DEV_TAB_WIDGET);
        ui_->pushButton_select_modbus    ->setStyleSheet(menu_btn_inactive_str_);
        ui_->pushButton_select_datc_ctrl ->setStyleSheet(menu_btn_inactive_str_);
        ui_->pushButton_select_adv       ->setStyleSheet(menu_btn_inactive_str_);
        ui_->pushButton_select_tcp       ->setStyleSheet(menu_btn_inactive_str_);
        ui_->pushButton_select_imped_ctrl->setStyleSheet(menu_btn_inactive_str_);
    });
}

MainWindow::~MainWindow() {
    if(datc_interface_ != NULL) {
        datc_interface_->~DatcCommInterface();
    }

    if(timer_ != NULL) {
        delete timer_;
    }
}

void MainWindow::timerCallback() {
    static std::function setButtonStyle([=] (QPushButton *btn) {
        bool is_activated = btn->isEnabled();
        btn->setStyleSheet(is_activated ? btn_active_str_ : btn_inactive_str_);
    });

    DatcStatus datc_status = datc_interface_->getDatcStatus();
    const bool conn   = datc_interface_->getConnectionState();
    // 구버전 펌웨어(레지스터 0~19): 그리퍼 정보·연결 감지·파지력 피드백이 없고, 위치·파지력을 %로 지령한다
    const bool legacy = conn && datc_status.legacy_map;
    applyFingerUnits(legacy);
    applyLegacyFeatureLock(legacy);

    // Display — motor/gripper feedback
    // Finger position 피드백을 mm로 표시 (0mm=닫힘, stroke mm=열림). finger_pos(reg4)=permille(0~1000).
    // 구버전 펌웨어는 stroke를 알려 주지 않으므로 열림 비율(%)로 표시한다.
    if (conn && datc_status.gripper_stroke_mm > 0) {
        double finger_mm = (double) datc_status.finger_pos / 1000.0 * (double) datc_status.gripper_stroke_mm;
        ui_->lineEdit_monitor_finger_position->setText(QString::number(finger_mm, 'f', 1) + " mm");
    } else if (legacy) {
        ui_->lineEdit_monitor_finger_position->setText(QString::number(datc_status.finger_pos / 10.0, 'f', 1) + " %");
    } else {
        ui_->lineEdit_monitor_finger_position->setText("- mm");
    }
    ui_->lineEdit_monitor_current        ->setText(QString::number(datc_status.motor_cur) + " mA");
    ui_->lineEdit_monitor_grip_force     ->setText(legacy ? QString("- N")   // 구버전: reg 19 없음
                                                          : QString::number(datc_status.grip_force_calc_n) + " N");

    // Gripper ID / physical specs (MATC v3 registers 20-23)
    // gripper_id == 128 이면 그리퍼 미장착
    if (!conn || datc_status.gripper_id == 128) {
        ui_->lineEdit_gripper_id     ->setText("N/A");
        ui_->lineEdit_gripper_stroke ->setText("- mm");
        ui_->lineEdit_gripper_force  ->setText("- N");
        ui_->lineEdit_gripper_payload->setText("- kg");

        if (legacy) {
            // 구버전 펌웨어: 그리퍼 정보는 없지만 위치(0~100 %)·파지력(50~100 %) 지령은 된다.
            // 단위와 범위는 applyFingerUnits가 맞춘다.
            datc_ctrl_widget_->ui_.horizontalSlider_finger_pos->setEnabled(true);
            datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setEnabled(true);
            datc_ctrl_widget_->ui_.pushButton_set_position->setEnabled(true);
            impedance_ctrl_widget_->ui_.horizontalSlider_finger_pos->setEnabled(true);
            impedance_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setEnabled(true);
            datc_ctrl_widget_->ui_.verticalSlider_torque->setEnabled(true);
            datc_ctrl_widget_->ui_.doubleSpinBox_torque->setEnabled(true);
            datc_ctrl_widget_->ui_.pushButton_set_torque->setEnabled(true);
        } else {
            // 그리퍼 미연결: 핑거 위치 컨트롤 비활성 + 기본 범위 0~100mm.
            if (datc_ctrl_widget_->ui_.horizontalSlider_finger_pos->maximum() != 100) {
                datc_ctrl_widget_->ui_.horizontalSlider_finger_pos->setRange(0, 100);
                datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setRange(0.0, 100.0);
                impedance_ctrl_widget_->ui_.horizontalSlider_finger_pos->setRange(0, 100);
                impedance_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setRange(0.0, 100.0);
            }
            datc_ctrl_widget_->ui_.horizontalSlider_finger_pos->setEnabled(false);
            datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setEnabled(false);
            datc_ctrl_widget_->ui_.pushButton_set_position->setEnabled(false);
            impedance_ctrl_widget_->ui_.horizontalSlider_finger_pos->setEnabled(false);
            impedance_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setEnabled(false);

            // 그리퍼 미연결: 파지력 컨트롤 비활성 + 기본 범위 5~10N (5N=50%, 10N=100%).
            if (datc_ctrl_widget_->ui_.verticalSlider_torque->maximum() != 10) {
                datc_ctrl_widget_->ui_.verticalSlider_torque->setRange(5, 10);
                datc_ctrl_widget_->ui_.doubleSpinBox_torque->setRange(5.0, 10.0);
            }
            datc_ctrl_widget_->ui_.verticalSlider_torque->setEnabled(false);
            datc_ctrl_widget_->ui_.doubleSpinBox_torque->setEnabled(false);
            datc_ctrl_widget_->ui_.pushButton_set_torque->setEnabled(false);
        }
    } else {
        ui_->lineEdit_gripper_id     ->setText(QString::number(datc_status.gripper_id));
        ui_->lineEdit_gripper_stroke ->setText(QString::number(datc_status.gripper_stroke_mm) + " mm");
        ui_->lineEdit_gripper_force  ->setText(QString::number(datc_status.gripper_force_n) + " N");
        ui_->lineEdit_gripper_payload->setText(
            QString::number((double) datc_status.gripper_payload_x10 / 10.0, 'f', 1) + " kg");

        // 핑거 위치 범위를 그리퍼 stroke(mm)로 가변 적용 (0=닫힘, stroke=열림). stroke 바뀔 때만
        // 갱신(매 틱 부작용 방지). stroke 미정의(예: 진공)면 위치 지령 불가 → 비활성 + 기본 0~100mm.
        const int stroke = (int) datc_status.gripper_stroke_mm;
        if (stroke > 0) {
            if (datc_ctrl_widget_->ui_.horizontalSlider_finger_pos->maximum() != stroke) {
                datc_ctrl_widget_->ui_.horizontalSlider_finger_pos->setRange(0, stroke);
                datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setRange(0.0, (double) stroke);
                impedance_ctrl_widget_->ui_.horizontalSlider_finger_pos->setRange(0, stroke);
                impedance_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setRange(0.0, (double) stroke);
            }
            datc_ctrl_widget_->ui_.horizontalSlider_finger_pos->setEnabled(true);
            datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setEnabled(true);
            datc_ctrl_widget_->ui_.pushButton_set_position->setEnabled(true);
            impedance_ctrl_widget_->ui_.horizontalSlider_finger_pos->setEnabled(true);
            impedance_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setEnabled(true);
        } else {
            if (datc_ctrl_widget_->ui_.horizontalSlider_finger_pos->maximum() != 100) {
                datc_ctrl_widget_->ui_.horizontalSlider_finger_pos->setRange(0, 100);
                datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setRange(0.0, 100.0);
                impedance_ctrl_widget_->ui_.horizontalSlider_finger_pos->setRange(0, 100);
                impedance_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setRange(0.0, 100.0);
            }
            datc_ctrl_widget_->ui_.horizontalSlider_finger_pos->setEnabled(false);
            datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setEnabled(false);
            datc_ctrl_widget_->ui_.pushButton_set_position->setEnabled(false);
            impedance_ctrl_widget_->ui_.horizontalSlider_finger_pos->setEnabled(false);
            impedance_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setEnabled(false);
        }

        // 그립 파지력: 정격의 50%~100% 구간만 사용. 범위 [50%·정격, 정격]N으로 설정.
        // 정격이 바뀔 때만 갱신. 펌웨어가 종류별로 N→전류 환산(평행=선형, 표준형=각도 의존).
        const int rated_force = (int) datc_status.gripper_force_n;
        if (rated_force > 0) {
            if (datc_ctrl_widget_->ui_.verticalSlider_torque->maximum() != rated_force) {
                const int force_min = (rated_force + 1) / 2;   // 50% (반올림)
                datc_ctrl_widget_->ui_.verticalSlider_torque->setRange(force_min, rated_force);
                datc_ctrl_widget_->ui_.doubleSpinBox_torque->setRange((double) force_min, (double) rated_force);
                // 범위를 바꾸면 값이 하한(50%)으로 붙어 Set이 정격의 절반을 보낸다. 펌웨어 기본값
                // (212 미지령 = 정격 전류로 파지)과 같게 정격(100%)에서 시작한다.
                datc_ctrl_widget_->ui_.verticalSlider_torque->setValue(rated_force);
                datc_ctrl_widget_->ui_.doubleSpinBox_torque->setValue((double) rated_force);
            }
            // 그리퍼 장착 → 파지력 컨트롤 활성화
            datc_ctrl_widget_->ui_.verticalSlider_torque->setEnabled(true);
            datc_ctrl_widget_->ui_.doubleSpinBox_torque->setEnabled(true);
            datc_ctrl_widget_->ui_.pushButton_set_torque->setEnabled(true);
        } else {
            // 장착됐으나 파지력 정격 미정의(예: 바이스) → 파지력 지령 불가, 비활성 + 기본 5~10N.
            if (datc_ctrl_widget_->ui_.verticalSlider_torque->maximum() != 10) {
                datc_ctrl_widget_->ui_.verticalSlider_torque->setRange(5, 10);
                datc_ctrl_widget_->ui_.doubleSpinBox_torque->setRange(5.0, 10.0);
            }
            datc_ctrl_widget_->ui_.verticalSlider_torque->setEnabled(false);
            datc_ctrl_widget_->ui_.doubleSpinBox_torque->setEnabled(false);
            datc_ctrl_widget_->ui_.pushButton_set_torque->setEnabled(false);
        }
    }

    // Gripper connection status (CONNECT_CHECK / PG0, holding reg 18)
    if (!conn) {
        ui_->lineEdit_gripper_connect->setText("- ");
        ui_->lineEdit_gripper_connect->setStyleSheet("color:#888888;");
    } else if (legacy) {
        // 구버전 펌웨어는 연결 감지 신호(reg 18)가 없어 알 수 없다 — "Disconnected"로 보이지 않게 한다
        ui_->lineEdit_gripper_connect->setText("-");
        ui_->lineEdit_gripper_connect->setStyleSheet("color:#888888;");
    } else if (datc_status.gripper_connected) {
        ui_->lineEdit_gripper_connect->setText("Connected");
        ui_->lineEdit_gripper_connect->setStyleSheet("color:#188A18; font-weight:bold;");
    } else {
        ui_->lineEdit_gripper_connect->setText("Disconnected");
        ui_->lineEdit_gripper_connect->setStyleSheet("color:#B00000; font-weight:bold;");
    }

    // 연결된 장치 정보와 모델별 제어 한계 (regs 28–33, v3.01~) — 바뀔 때만 갱신
    {
        const DeviceInfo info = datc_interface_->getDeviceInfo();
        const int vel_max = datc_interface_->getVelMaxRpm();
        const int cur_max = datc_interface_->getCurMaxMa();

        QString text;
        if (!conn) {
            text = "장치 정보: -";
        } else if (legacy) {
            text = QString("장치 정보 없음 — 구버전 펌웨어: 위치·파지력 % 단위 (기본 한계: 속도 ≤ %1 rpm, 전류 100% = %2 mA)")
                       .arg(vel_max).arg(cur_max);
        } else if (info.valid) {
            const QString elangle = info.elangleModelMismatch() ? "다른 모델 값 → 재캘리브 필요"
                                  : (info.elangleCalibrated() ? "OK" : "미캘리브 → Dev 탭에서 전기각 설정 필요");
            // 일체형 그리퍼 고정 ID (reg 34, v3.04~) — 저장돼 있을 때만 붙인다
            QString fixed;
            if (info.hasFixedId()) {
                fixed = QString(" · 고정 ID %1%2").arg(info.fixed_gripper_id)
                            .arg(info.fixedIdInUse() ? "" : "(핀 ID 우선)");
            }
            text = QString("장치: %1 · FW %2 · 속도 제어 ≤ %3 rpm · 전류 100% = %4 mA · 전기각 %5%6")
                       .arg(QString::fromStdString(matcModelName(info.model)))
                       .arg(QString::fromStdString(matcFwVersionStr(info.fw_version)))
                       .arg(vel_max).arg(cur_max).arg(elangle).arg(fixed);
        } else if (info.unsupported) {
            text = QString("장치 정보 없음 — v3.01 이전 펌웨어 (기본 한계: 속도 ≤ %1 rpm, 전류 100% = %2 mA)")
                       .arg(vel_max).arg(cur_max);
        } else {
            text = "장치 정보 확인 중...";
        }
        if ((label_dev_info_ != nullptr) && (label_dev_info_->text() != text)) {
            label_dev_info_->setText(text);
        }

        static int last_vel_max = -1;
        static int last_cur_max = -1;
        if (vel_max != last_vel_max) {
            last_vel_max = vel_max;
            advanced_ctrl_widget_->ui_.label_motor_speed->setText("(100 % : " + QString::number(vel_max) + " rpm)");
        }
        if (cur_max != last_cur_max) {
            last_cur_max = cur_max;
            advanced_ctrl_widget_->ui_.label_motor_current->setText("(100 % : " + QString::number(cur_max) + " mA)");
        }
    }

    static uint16_t last_gripper_id = 0xFFFF;
    if (conn && datc_status.gripper_id != last_gripper_id) {
        last_gripper_id = datc_status.gripper_id;
        if (datc_status.gripper_id == 128) {
            COUT("[Gripper] None detected");
        } else {
            printf("[Gripper] ID:%u  Stroke:%umm  Force:%uN  Payload:%.1fkg\n",
                   datc_status.gripper_id,
                   datc_status.gripper_stroke_mm,
                   datc_status.gripper_force_n,
                   (double) datc_status.gripper_payload_x10 / 10.0);
        }
    }

    // Comm. status check
    const bool is_modbus_connected = datc_interface_->getConnectionState();

    modbus_widget_->ui_.pushButton_modbus_start->setEnabled(!is_modbus_connected);
    modbus_widget_->ui_.pushButton_modbus_stop ->setEnabled(is_modbus_connected);
    modbus_widget_->ui_.pushButton_modbus_set_slave_addr->setEnabled(is_modbus_connected);
    modbus_widget_->ui_.pushButton_modbus_slave_change->setEnabled(is_modbus_connected);

    setButtonStyle(modbus_widget_->ui_.pushButton_modbus_start);
    setButtonStyle(modbus_widget_->ui_.pushButton_modbus_stop);
    setButtonStyle(modbus_widget_->ui_.pushButton_modbus_set_slave_addr);
    setButtonStyle(modbus_widget_->ui_.pushButton_modbus_slave_change);

    if (is_modbus_connected) {
        if (datc_interface_->getModbusRecvErr()) {
            ui_->lineEdit_monitor_mode->setText("Read error");
            ui_->lineEdit_monitor_warn->clear();
        } else {
            ui_->lineEdit_monitor_mode->setText(" " + QString::fromStdString(datc_status.status_str));
            // 경고는 둘째 줄. 칸보다 길면(경고 여러 개) 앞부분부터 보이게 하고, 전체는 툴팁으로 표시
            const QString warn_text = datc_status.warn_str.empty() ?
                                      QString() : " " + QString::fromStdString(datc_status.warn_str);
            ui_->lineEdit_monitor_warn->setText(warn_text);
            ui_->lineEdit_monitor_warn->setCursorPosition(0);
            ui_->lineEdit_monitor_warn->setToolTip(warn_text);
        }

        QString qstr_slave_addr = (datc_interface_->getSlaveAddr() == 0) ?
                                  "N/A" : QString::number(datc_interface_->getSlaveAddr());

        ui_->lineEdit_current_slave_addr->setText(qstr_slave_addr);
    } else {
        ui_->lineEdit_current_slave_addr->setText("N/A");
    }

#ifndef RCLCPP__RCLCPP_HPP_
    // Socket comm. status check
    const bool is_socket_connected = datc_interface_->isSocketConnected();

    tcp_widget_->ui_.pushButton_tcp_start->setEnabled(!is_socket_connected);
    tcp_widget_->ui_.pushButton_tcp_stop ->setEnabled(is_socket_connected);

    setButtonStyle(tcp_widget_->ui_.pushButton_tcp_start);
    setButtonStyle(tcp_widget_->ui_.pushButton_tcp_stop);

    datc_interface_->setTcpSendStatus(tcp_widget_->ui_.checkBox_tcp_send_status->isChecked());
#endif

    // Slider control
    static std::function syncSliderSpinboxFn(
            [] (int &slider_data_prev, double &spinbox_data_prev, QSlider *slider, QDoubleSpinBox *spinbox) {
        int slider_data     = slider->value();
        double spinbox_data = spinbox->value();

        if (slider_data != slider_data_prev) {
            spinbox->setValue((double) slider_data);
        } else if (fabs(spinbox_data - spinbox_data_prev) >= 0.09) {
            slider->setValue((int) spinbox_data);
        }

        slider_data_prev  = slider->value();
        spinbox_data_prev = spinbox->value();
    });

    // Finger position control side
    static int slider_finger_pos_prev  = datc_ctrl_widget_->ui_.horizontalSlider_finger_pos->value();
    static int slider_torque_prev      = datc_ctrl_widget_->ui_.verticalSlider_torque->value();
    static int slider_speed_prev       = datc_ctrl_widget_->ui_.verticalSlider_speed ->value();
    static int slider_finger_pos_prev2 = impedance_ctrl_widget_->ui_.horizontalSlider_finger_pos->value();

    static double finger_pos_prev  = datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->value();
    static double torque_prev      = datc_ctrl_widget_->ui_.doubleSpinBox_torque->value();
    static double speed_prev       = datc_ctrl_widget_->ui_.doubleSpinBox_speed->value();
    static double finger_pos_prev2 = impedance_ctrl_widget_->ui_.doubleSpinBox_finger_pos->value();

    syncSliderSpinboxFn(slider_finger_pos_prev, finger_pos_prev,
                        datc_ctrl_widget_->ui_.horizontalSlider_finger_pos,
                        datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos);
    syncSliderSpinboxFn(slider_torque_prev, torque_prev,
                        datc_ctrl_widget_->ui_.verticalSlider_torque,
                        datc_ctrl_widget_->ui_.doubleSpinBox_torque);
    syncSliderSpinboxFn(slider_speed_prev, speed_prev,
                        datc_ctrl_widget_->ui_.verticalSlider_speed,
                        datc_ctrl_widget_->ui_.doubleSpinBox_speed);
    syncSliderSpinboxFn(slider_finger_pos_prev2, finger_pos_prev2,
                        impedance_ctrl_widget_->ui_.horizontalSlider_finger_pos,
                        impedance_ctrl_widget_->ui_.doubleSpinBox_finger_pos);

    // Advanced control side — 100 %는 모델별 속도 상한(장치 보고값, 없으면 kVelMax)
    const int vel_max_rpm     = datc_interface_->getVelMaxRpm();
    const int vel_min_percent = (int) ((double) std::min<int>(kVelMin, vel_max_rpm) / (double) vel_max_rpm * 100);

    if (advanced_ctrl_widget_->ui_.horizontalSlider_motor_speed->value() < vel_min_percent) {
        advanced_ctrl_widget_->ui_.horizontalSlider_motor_speed->setValue(vel_min_percent);
    }

    static int slider_vel_prev = advanced_ctrl_widget_->ui_.horizontalSlider_motor_speed->value();
    static int slider_cur_prev = advanced_ctrl_widget_->ui_.horizontalSlider_motor_current->value();

    static double vel_prev = advanced_ctrl_widget_->ui_.doubleSpinBox_motor_speed->value();
    static double cur_prev = advanced_ctrl_widget_->ui_.doubleSpinBox_motor_current->value();

    syncSliderSpinboxFn(slider_vel_prev, vel_prev,
                        advanced_ctrl_widget_->ui_.horizontalSlider_motor_speed,
                        advanced_ctrl_widget_->ui_.doubleSpinBox_motor_speed);
    syncSliderSpinboxFn(slider_cur_prev, cur_prev,
                        advanced_ctrl_widget_->ui_.horizontalSlider_motor_current,
                        advanced_ctrl_widget_->ui_.doubleSpinBox_motor_current);

    // Repeat test 진행 (이 콜백 주기 100 ms마다 한 단계)
    repeatStep(datc_status);
}

// Enable Disable
void MainWindow::datcEnable() {
    datc_interface_->motorEnable();
}

void MainWindow::datcDisable() {
    datc_interface_->motorDisable();
}

// 구버전 펌웨어(레지스터 0~19)는 손가락 위치를 0~1000 ‰(명령 104), 파지력을 정격 토크의 %(명령 212)로 받는다.
// 연결된 장치에 맞춰 위치·파지력 칸의 단위와 범위, 파지력 제목을 바꾼다(바뀔 때만). V3로 돌아오면 기본 범위로
// 되돌리고, 실제 범위(stroke·정격 파지력)는 timerCallback의 갱신 코드가 다시 맞춘다.
void MainWindow::applyFingerUnits(bool legacy) {
    if (legacy == finger_units_legacy_) {
        return;
    }
    finger_units_legacy_ = legacy;

    for (QDoubleSpinBox *sb : {datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos,
                               impedance_ctrl_widget_->ui_.doubleSpinBox_finger_pos}) {
        sb->setSuffix(legacy ? " %" : " mm");
        sb->setRange(0.0, 100.0);
    }
    datc_ctrl_widget_->ui_.horizontalSlider_finger_pos->setRange(0, 100);
    impedance_ctrl_widget_->ui_.horizontalSlider_finger_pos->setRange(0, 100);

    const int torque_min = legacy ? kLegacyTorquePctMin : 5;
    const int torque_max = legacy ? kLegacyTorquePctMax : 10;
    datc_ctrl_widget_->ui_.label_51->setText(legacy ? "Grip Force (%)" : "Grip Force (N)");
    datc_ctrl_widget_->ui_.doubleSpinBox_torque->setSuffix(legacy ? " %" : " N");
    datc_ctrl_widget_->ui_.verticalSlider_torque->setRange(torque_min, torque_max);
    datc_ctrl_widget_->ui_.doubleSpinBox_torque->setRange((double) torque_min, (double) torque_max);
    if (legacy) {
        // 범위를 바꾸면 값이 하한으로 붙으므로, 펌웨어 기본값인 정격(100 %)에서 시작한다
        datc_ctrl_widget_->ui_.verticalSlider_torque->setValue(torque_max);
        datc_ctrl_widget_->ui_.doubleSpinBox_torque->setValue((double) torque_max);
    }
}

// 구버전 펌웨어(레지스터 0~19)에는 V3 전용 명령(isV3OnlyCommand)이 없고, 앱이 플래시 맨 앞부터 있어 부트로더도
// 없다. 연결된 장치에 맞춰 그 기능의 위젯과 펌웨어 업데이트 시작 버튼을 막고, 툴팁으로 이유를 보인다(바뀔 때만).
// 그룹을 막으면 안쪽 위젯의 자체 활성 상태는 그대로 남아 있다가 풀릴 때 되살아난다.
void MainWindow::applyLegacyFeatureLock(bool legacy) {
    if (legacy == legacy_feature_lock_) {
        return;
    }
    legacy_feature_lock_ = legacy;

    const QString tip = legacy ? "구버전 펌웨어에는 없는 기능입니다 (MATC V3 전용)" : QString();
    for (QWidget *w : v3_only_widgets_) {
        w->setEnabled(!legacy);
        w->setToolTip(tip);
    }
    QPushButton *fw_start = dev_tab_widget_->ui_.pushButton_fw_start;
    fw_start->setEnabled(!legacy && !fw_updater_->isRunning());
    fw_start->setToolTip(legacy ? "구버전 펌웨어는 부트로더가 없어 GCS로 업데이트할 수 없습니다 (ST-Link로 굽기)"
                                : QString());
}

// Datc control
// 위치 칸 값 → 명령 104 값. V3는 mm 그대로(0mm=닫힘, stroke mm=열림, 펌웨어가 stroke로 정규화),
// 구버전 펌웨어는 열림 % → ‰(0~1000).
uint16_t MainWindow::fingerPosCommand(double value) const {
    return (uint16_t) ((finger_units_legacy_ ? value * 10.0 : value) + 0.5);
}

void MainWindow::datcFingerPosCtrl() {
    datc_interface_->setFingerPos(fingerPosCommand(datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->value()));
}

void MainWindow::datcFingerPosCtrl2() {
    datc_interface_->setFingerPos(fingerPosCommand(impedance_ctrl_widget_->ui_.doubleSpinBox_finger_pos->value()));
}

void MainWindow::datcMotorVelCtrl() {
    // 슬라이더 % → rpm. 100 % = 모델별 속도 상한 (MATC-10 900, GL60 310, GL80 450 rpm)
    int16_t vel = advanced_ctrl_widget_->ui_.doubleSpinBox_motor_speed->value() * datc_interface_->getVelMaxRpm() / 100;
    vel *= (advanced_ctrl_widget_->ui_.checkBox_motor_speed_reverse->isChecked()) ? -1 : 1;
    datc_interface_->motorVelCtrl(vel);
}

void MainWindow::datcMotorCurCtrl() {
    // 슬라이더 % → mA. 100 % = 모델별 정격 전류 (MATC-10 1297, GL60 1350, GL80 2800 mA)
    int16_t cur = advanced_ctrl_widget_->ui_.doubleSpinBox_motor_current->value() * datc_interface_->getCurMaxMa() / 100;
    cur *= (advanced_ctrl_widget_->ui_.checkBox_motor_current_reverse->isChecked()) ? -1 : 1;
    datc_interface_->motorCurCtrl(cur);
}

void MainWindow::datcInit() {
    datc_interface_->grpInitialize();
}

void MainWindow::datcOpen() {
    datc_interface_->grpOpen();
}

void MainWindow::datcClose() {
    datc_interface_->grpClose();
}

void MainWindow::datcStop() {
    datc_interface_->motorStop();
}

void MainWindow::datcVacuumGrpOn() {
    datc_interface_->vacuumGrpOn();
}

void MainWindow::datcVacuumGrpOff() {
    datc_interface_->vacuumGrpOff();
}

void MainWindow::datcSetTorque() {
    // 스핀박스 값(N)을 그립 파지력으로 전송. 펌웨어가 그리퍼 종류별로 N→전류 환산.
    // 구버전 펌웨어는 같은 명령(212)을 정격 토크의 %로 받으므로 칸 값(50~100 %)을 그대로 보낸다.
    datc_interface_->setMotorTorque((uint16_t) (datc_ctrl_widget_->ui_.doubleSpinBox_torque->value() + 0.5));
}

void MainWindow::datcSetSpeed() {
    datc_interface_->setMotorSpeed((uint16_t) datc_ctrl_widget_->ui_.doubleSpinBox_speed->value());
}

// Impedance related functions
void MainWindow::datcImpedanceOn() {
    datc_interface_->impedanceOn();
    usleep(100000);
    datc_interface_->grpInitialize();
}

void MainWindow::datcImpedanceOff() {
    datc_interface_->impedanceOff();
    usleep(100000);
    datc_interface_->grpInitialize();
}

void MainWindow::datcSetImpedanceParams() {
    int16_t slave_num       = impedance_ctrl_widget_->ui_.spinBox_impedance_slave_num->value();
    int16_t stiffness_level = impedance_ctrl_widget_->ui_.spinBox_impedance_stiffness_level->value();

    datc_interface_->setImpedanceParams(slave_num, stiffness_level);
}

// Modbus RTU related
void MainWindow::initModbus() {
    COUT("--------------------------------------------");
    COUT("[INFO] Port: " + modbus_widget_->ui_.comboBox_serial_port->currentText().toStdString());
    COUT("[INFO] Slave address #" + modbus_widget_->ui_.spinBox_slave_addr->text().toStdString());
    COUT("--------------------------------------------");

    std::string port = modbus_widget_->ui_.comboBox_serial_port->currentText().toStdString();
    uint16_t slave_addr = modbus_widget_->ui_.spinBox_slave_addr->value();

    // Release 모드에서도 정상 동작하도록 QString 타입 명시
    QString baudrate_qstr = modbus_widget_->ui_.comboBox_baudrate->currentText();
    int baudrate = baudrate_qstr.toInt();

    if (datc_interface_->init(port.c_str(), slave_addr, baudrate)) {
        // Successed
    } else {
        ui_->lineEdit_monitor_mode->setText("Invalid port");
        ui_->lineEdit_monitor_warn->clear();
        COUT("[ERROR] Port name or slave address invlaid !");
    }
}

void MainWindow::releaseModbus() {
    datc_interface_->modbusRelease();
    ui_->lineEdit_monitor_mode->setText("");
    ui_->lineEdit_monitor_warn->clear();
}

void MainWindow::changeSlaveAddress() {
    uint16_t slave_addr = modbus_widget_->ui_.spinBox_slave_addr->value();

    if (datc_interface_->modbusSlaveChange(slave_addr)) {
        // Successed
    } else {
        COUT("[ERROR] Slave change failed !");
    }
}

void MainWindow::setSlaveAddr() {
    uint16_t slave_addr = modbus_widget_->ui_.spinBox_slave_addr_4set->value();

    if (datc_interface_->setModbusAddr(slave_addr)) {
        // Successed
    } else {
        COUT("[ERROR] Slave change failed !");
    }
}

// 지정 속도로 재연결하고 그리퍼가 실제로 응답하는지 확인한다.
// 속도 전환 직후엔 선로에 남은 바이트 때문에 첫 요청이 실패할 수 있어 몇 번 재시도한다.
bool MainWindow::reconnectAt(int baudrate) {
    // 폴링 스레드가 버스를 잡고 있으면 probe가 락을 얻지 못한다(datc_comm_interface.hpp 참고)
    PollingPause pause(datc_interface_);
    QComboBox *cb = modbus_widget_->ui_.comboBox_baudrate;
    int idx = cb->findText(QString::number(baudrate));
    if (idx < 0) {
        cb->addItem(QString::number(baudrate));
        idx = cb->count() - 1;
    }
    cb->setCurrentIndex(idx);

    releaseModbus();
    initModbus();
    if (!datc_interface_->getConnectionState()) {
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        if (datc_interface_->probe()) {
            return true;
        }
        // 응답이 없으면 시도마다 응답 시간 초과까지 기다리므로, 그사이 창을 다시 그려 '응답 없음'을 막는다
        // (사용자 입력은 처리하지 않아 검색 도중 다른 명령이 끼어들지 않는다)
        QApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
        QThread::msleep(100);
    }
    return false;
}

// 장치(펌웨어) Modbus 통신속도 변경 버튼: 확인 창 → changeDeviceBaudrate → 결과 창.
void MainWindow::setDeviceBaudrate() {
    if (!datc_interface_->getConnectionState()) {
        QMessageBox::warning(this, "장치 통신속도 변경", "먼저 현재 속도로 그리퍼에 연결하세요.");
        return;
    }
    if (baud_change_busy_) {
        return;
    }
    baud_change_busy_ = true;
    auto busy_reset = qScopeGuard([this] { baud_change_busy_ = false; });

    const int new_baud = combo_dev_baud_->currentText().toInt();
    const int old_baud = modbus_widget_->ui_.comboBox_baudrate->currentText().toInt();
    if (new_baud == old_baud) {
        QMessageBox::information(this, "장치 통신속도 변경",
                                 QString("이미 %1 bps로 연결되어 있습니다.").arg(new_baud));
        return;
    }
    const QString msg = QString(
        "그리퍼의 Modbus 통신속도를 %1 → %2 bps로 변경합니다.\n\n"
        "• 그리퍼 EEPROM에 저장되어 전원을 껐다 켜도 유지됩니다.\n"
        "• GCS는 새 속도로 자동 재연결합니다.\n"
        "• PLC 등 다른 마스터를 쓰면 그쪽 속도도 함께 바꿔야 합니다.\n"
        "• 그리퍼가 동작하지 않을 때 실행하세요(저장 중 약 0.4초간 그리퍼 제어 로직이 멈춥니다).\n\n"
        "계속할까요?").arg(old_baud).arg(new_baud);
    if (QMessageBox::question(this, "장치 통신속도 변경", msg) != QMessageBox::Yes) {
        return;
    }

    switch (changeDeviceBaudrate(new_baud, old_baud)) {
        case BaudChange::CHANGED:
            QMessageBox::information(this, "장치 통신속도 변경",
                QString("변경 완료 — %1 bps로 다시 연결했습니다.").arg(new_baud));
            break;
        case BaudChange::REVERTED:
            QMessageBox::warning(this, "장치 통신속도 변경",
                QString("그리퍼가 이전 속도(%1 bps)로 계속 응답합니다.\n"
                        "EEPROM 저장에 실패했을 수 있습니다(상태워드 bit15). 다시 시도해 주세요.").arg(old_baud));
            break;
        case BaudChange::NO_RESPONSE:
            QMessageBox::warning(this, "장치 통신속도 변경",
                "새 속도와 이전 속도 모두에서 응답이 없습니다.\n'속도 자동 검색'을 실행해 주세요.");
            break;
        case BaudChange::NOT_SENT:
            QMessageBox::warning(this, "장치 통신속도 변경", "변경 명령 전송에 실패했습니다.");
            break;
    }
}

// 장치 통신속도 변경 공통 절차 — '장치 속도 변경' 버튼(setDeviceBaudrate)과 원격 명령(remoteSetDeviceBaudrate: TCP 52,
// ROS2 set_modbus_baudrate)이 쓴다.
// 명령 전송 → 그리퍼가 EEPROM 저장 후 즉시 전환 → GCS가 새 속도로 재연결해 응답을 확인한다.
// 실패하면 이전 속도로 되돌아간다(REVERTED: 그리퍼가 이전 속도로 응답, NO_RESPONSE: 둘 다 응답 없음).
MainWindow::BaudChange MainWindow::changeDeviceBaudrate(int new_baud, int old_baud) {
    if (!datc_interface_->setModbusBaudrate(new_baud)) {
        return BaudChange::NOT_SENT;
    }
    COUT("[INFO] Device baudrate change: " + std::to_string(old_baud) + " -> " + std::to_string(new_baud));

    // 그리퍼가 EEPROM 저장(약 0.4 s)을 마치고 속도를 바꿀 시간을 준 뒤 새 속도로 재연결.
    // 그동안 폴링은 옛 속도로 시간 초과만 반복하므로 멈춰 둔다.
    PollingPause pause(datc_interface_);
    QApplication::setOverrideCursor(Qt::WaitCursor);
    QThread::msleep(1500);
    bool ok_new = reconnectAt(new_baud);
    if (!ok_new) {
        // EEPROM 재시도 등으로 저장이 늦어 아직 전환 전일 수 있다 — 조금 더 기다려 한 번 더 시도
        QThread::msleep(1000);
        ok_new = reconnectAt(new_baud);
    }
    const bool ok_old = ok_new ? false : reconnectAt(old_baud);
    QApplication::restoreOverrideCursor();

    if (ok_new) {
        return BaudChange::CHANGED;
    }
    if (ok_old) {
        if (combo_dev_baud_ != nullptr) {
            combo_dev_baud_->setCurrentText(QString::number(old_baud));
        }
        return BaudChange::REVERTED;
    }
    return BaudChange::NO_RESPONSE;
}

// 장치(펌웨어) 통신속도 자동 검색 — 속도를 잊었거나 다른 마스터가 바꿨을 때 복구용.
// 펌웨어는 저장값이 허용 속도가 아니면 115200으로 기동하므로, 그리퍼는 항상 아래 중 하나로 응답한다.
void MainWindow::autoDetectBaudrate() {
    if (baud_change_busy_) {
        return;
    }
    baud_change_busy_ = true;
    auto busy_reset = qScopeGuard([this] { baud_change_busy_ = false; });

    const int candidates[] = {115200, 57600, 38400, 19200, 9600};
    int found = 0;
    QApplication::setOverrideCursor(Qt::WaitCursor);
    {
        PollingPause pause(datc_interface_);   // 후보 사이에 폴링이 끼어들지 않게 검색 내내 멈춘다
        for (int b : candidates) {
            if (reconnectAt(b)) {
                found = b;
                break;
            }
        }
    }
    QApplication::restoreOverrideCursor();

    if (found != 0) {
        if (combo_dev_baud_ != nullptr) {
            combo_dev_baud_->setCurrentText(QString::number(found));
        }
        QMessageBox::information(this, "속도 자동 검색",
            QString("그리퍼가 %1 bps로 응답합니다 — 이 속도로 연결했습니다.").arg(found));
    } else {
        releaseModbus();
        QMessageBox::warning(this, "속도 자동 검색",
            "응답하는 속도를 찾지 못했습니다.\n포트, 슬레이브 주소, 배선과 전원을 확인해 주세요.");
    }
}

// Repeat test — 열기 → 열림 도달 → Open Delay → 닫기 → 닫힘 완료(물체 파지/빈 닫힘) → Close Delay → …
// 열림→닫힘 한 사이클마다 횟수 +1. 열림/닫힘 대기 중 위치가 3 s 동안 안 움직이거나(120 s 상한),
// 통신이 끊기거나, 모터 폴트가 나면 멈추고 Error Code에 원인을 표시한다.
// Stop은 시퀀스만 멈춘다(진행 중인 동작은 끝까지 감).
// Target Count(0 = 무제한)에 닿으면 그 사이클의 닫힘을 마친 상태에서 멈추고 "Done"을 표시한다.
void MainWindow::repeatStart() {
    if (!datc_interface_->getConnectionState()) {
        repeatSetStatus("Modbus not connected");
        return;
    }
    const uint32_t target = repeatTarget();
    if ((target > 0) && (repeat_count_ >= target)) {
        repeatSetStatus("Target reached - Reset");
        return;
    }
    repeatSetStatus(QString());
    repeat_open_done_  = false;
    repeat_close_done_ = false;
    repeat_running_    = true;
    repeat_timer_.start();

    // 이미 열려 있으면(98 % 이상) 닫기부터, 아니면 열기부터 시작
    const DatcStatus status = datc_interface_->getDatcStatus();
    repeat_state_ = (status.finger_pos < kRepeatOpenPosMin) ? RepeatState::CMD_OPEN : RepeatState::CMD_CLOSE;
    repeatLog(QString("[Repeat] Start (count %1, target %2)")
                  .arg(repeat_count_)
                  .arg(target > 0 ? QString::number(target) : QString("unlimited")));
}

void MainWindow::repeatStop() {
    const bool was_running = repeat_running_;
    repeat_running_ = false;
    repeat_state_   = RepeatState::IDLE;
    repeatSetStatus("Stopped");
    if (was_running) {
        repeatLog(QString("[Repeat] Stopped (count %1)").arg(repeat_count_));
    }
}

void MainWindow::repeatReset() {
    repeat_open_done_  = false;
    repeat_close_done_ = false;
    repeat_running_    = false;
    repeat_state_      = RepeatState::IDLE;
    repeat_count_      = 0;
    repeatUpdateCount();
    repeatSetStatus(QString());
}

uint32_t MainWindow::repeatTarget() const {
    return (uint32_t) qMax(0LL, qRound64(modbus_widget_->ui_.doubleSpinBox_repeat_target->value()));
}

void MainWindow::repeatUpdateCount() {
    const uint32_t target = repeatTarget();
    modbus_widget_->ui_.label_repeat_cnt->setText(
        (target > 0) ? QString("%1 / %2").arg(repeat_count_).arg(target) : QString::number(repeat_count_));
}

// Error Code 칸. 오류·정지는 폼 스타일(빨간 굵은 글씨), 목표 도달은 초록색.
void MainWindow::repeatSetStatus(const QString &text, bool done) {
    QLabel *label = modbus_widget_->ui_.label_repeat_error_code;
    label->setStyleSheet(done ? QString("color:#007A00;") : QString());
    label->setText(text);
}

void MainWindow::repeatStep(const DatcStatus &status) {
    if (!repeat_running_) {
        return;
    }
    if (!datc_interface_->getConnectionState()) {
        repeatHalt("Modbus disconnected", "[Error] Modbus disconnected");
        return;
    }
    if (status.fault) {
        const uint16_t code = (status.fault_now != 0) ? status.fault_now : status.fault_occurred;
        const QString code_str = "0x" + QString::number(code, 16).toUpper().rightJustified(4, '0');
        repeatHalt("Motor Fault " + code_str, "[Error] Motor fault " + code_str);
        return;
    }

    auto &ui = modbus_widget_->ui_;
    const qint64   elapsed_ms = repeat_timer_.elapsed();
    const uint16_t pos        = status.finger_pos;

    if ((repeat_state_ == RepeatState::WAIT_OPEN) || (repeat_state_ == RepeatState::WAIT_CLOSE)) {
        if (qAbs((int) pos - (int) repeat_progress_pos_) >= kRepeatProgressPosTol) {
            repeat_progress_pos_ = pos;
            repeat_progress_timer_.restart();
        }
        const bool stalled = (repeat_progress_timer_.elapsed() > kRepeatStallTimeoutMs);
        if (stalled || (elapsed_ms > kRepeatMoveTimeoutMs)) {
            const bool    opening = (repeat_state_ == RepeatState::WAIT_OPEN);
            const QString reason  = stalled
                ? QString("no movement for %1 s").arg(kRepeatStallTimeoutMs / 1000)
                : QString("not finished in %1 s").arg(kRepeatMoveTimeoutMs / 1000);
            repeatHalt(opening ? "Timeout (open)" : "Timeout (close)",
                       QString("[Error] Timeout: %1 stopped at %2 % (%3, %4 mA, gObj %5)")
                           .arg(opening ? "open" : "close")
                           .arg(pos / 10.0, 0, 'f', 1)
                           .arg(reason)
                           .arg(status.motor_cur)
                           .arg(status.gobj_result));
            return;
        }
    }

    switch (repeat_state_) {
    case RepeatState::CMD_OPEN:
        repeatLog("[Command] Open");
        datc_interface_->grpOpen();
        repeat_timer_.restart();
        repeat_progress_pos_ = pos;
        repeat_progress_timer_.start();
        repeat_state_ = RepeatState::WAIT_OPEN;
        repeatLog("[State] Waiting open position");
        break;

    case RepeatState::WAIT_OPEN:
        if (pos >= kRepeatOpenPosMin) {
            repeat_open_done_ = true;
            repeat_timer_.restart();
            repeat_state_ = RepeatState::OPEN_DELAY;
            repeatLog("[State] Open position reached");
            repeatLog(QString("[Delay] Open hold %1 s").arg(ui.doubleSpinBox_repeat_open_delay->value(), 0, 'f', 1));
        }
        break;

    case RepeatState::OPEN_DELAY:
        if (elapsed_ms >= qRound64(ui.doubleSpinBox_repeat_open_delay->value() * 1000.0)) {
            repeat_timer_.restart();
            repeat_state_ = RepeatState::CMD_CLOSE;
            repeatLog("[State] Open delay complete");
        }
        break;

    case RepeatState::CMD_CLOSE:
        repeatLog("[Command] Close");
        datc_interface_->grpClose();
        repeat_prev_pos_   = pos;
        repeat_close_start_pos_ = pos;
        repeat_stable_cnt_ = 0;
        repeat_timer_.restart();
        repeat_progress_pos_ = pos;
        repeat_progress_timer_.start();
        repeat_state_ = RepeatState::WAIT_CLOSE;
        repeatLog("[State] Waiting close");
        break;

    case RepeatState::WAIT_CLOSE: {
        // 물체 파지: 닫기 300 ms 뒤부터 위치·속도가 3틱 연속 멈춰 있고, 전류가 200 mA 이상이거나
        // 펌웨어가 gObj=1(닫는 중 물체 감지)을 보고. 빈 닫힘: 위치 3 % 이하.
        const bool still = (elapsed_ms >= kRepeatSettleMs) &&
                           (qAbs((int) pos - (int) repeat_prev_pos_) <= kRepeatStablePosTol) &&
                           (qAbs((int) status.motor_vel) <= kRepeatStableVelRpm);
        repeat_stable_cnt_ = still ? (repeat_stable_cnt_ + 1) : 0;
        repeat_prev_pos_   = pos;

        const bool grasped = (repeat_stable_cnt_ >= kRepeatStableTicks) &&
                             ((qAbs((int) status.motor_cur) >= kRepeatGraspCurMa) || (status.gobj_result == 1));
        if (!grasped && (pos > kRepeatClosePosMax)) {
            break;   // 아직 닫는 중
        }
        // 닫히지도 않았는데 파지로 보고 = 완전 열림 자세에서 닫기 출발 실패를 펌웨어가 물체로 오인.
        // 사이클로 세지 않고 멈춰 원인을 남긴다(펌웨어 2026-10-14 접근 전류 하한 수정 전 증상).
        const int closed_by = (int) repeat_close_start_pos_ - (int) pos;
        if (grasped && (closed_by < kRepeatGraspMinTravel)) {
            repeatHalt("Grasp without closing",
                       QString("[Error] Grasp reported without closing: %1 % (closed %2 %, %3 mA, gObj %4)")
                           .arg(pos / 10.0, 0, 'f', 1)
                           .arg(closed_by / 10.0, 0, 'f', 1)
                           .arg(status.motor_cur)
                           .arg(status.gobj_result));
            return;
        }
        repeat_close_done_ = true;
        repeatLog(grasped ? "[State] Close complete: object grasped" : "[State] Close complete");
        repeatLog("---------------------------");
        if (repeat_open_done_ && repeat_close_done_) {
            ++repeat_count_;
            repeatUpdateCount();
            repeatLog(QString("[Repeat] Count: %1").arg(repeat_count_));
            repeat_open_done_  = false;
            repeat_close_done_ = false;

            const uint32_t target = repeatTarget();
            if ((target > 0) && (repeat_count_ >= target)) {
                repeat_running_ = false;
                repeat_state_   = RepeatState::IDLE;
                repeatSetStatus("Done", true);
                repeatLog(QString("[Repeat] Target %1 reached - finished (closed)").arg(target));
                break;
            }
        }
        repeat_timer_.restart();
        repeat_state_ = RepeatState::CLOSE_DELAY;
        repeatLog(QString("[Delay] Close hold %1 s").arg(ui.doubleSpinBox_repeat_close_delay->value(), 0, 'f', 1));
        break;
    }

    case RepeatState::CLOSE_DELAY:
        if (elapsed_ms >= qRound64(ui.doubleSpinBox_repeat_close_delay->value() * 1000.0)) {
            repeat_timer_.restart();
            repeat_state_ = RepeatState::CMD_OPEN;
            repeatLog("[State] Close delay complete");
        }
        break;

    case RepeatState::IDLE:
    default:
        break;
    }
}

void MainWindow::repeatHalt(const QString &error_text, const QString &log_msg) {
    repeat_running_ = false;
    repeat_state_   = RepeatState::IDLE;
    repeatSetStatus(error_text);
    repeatLog(log_msg);
}

void MainWindow::repeatLog(const QString &msg) {
    QPlainTextEdit *log = modbus_widget_->ui_.plainTextEdit_repeat_command_status;
    log->appendPlainText(QTime::currentTime().toString("HH:mm:ss") + "  " + msg);
    log->verticalScrollBar()->setValue(log->verticalScrollBar()->maximum());
}

// Dev ui related functions
void MainWindow::dev_setGainP() {
    int p_p = dev_tab_widget_->ui_.spinBox_p_p->value();
    int p_i = dev_tab_widget_->ui_.spinBox_p_i->value();
    int p_d = dev_tab_widget_->ui_.spinBox_p_d->value();

    datc_interface_->setPositionPidGain(p_p, p_i, p_d);
}

void MainWindow::dev_setGainV() {
    int v_p = dev_tab_widget_->ui_.spinBox_v_p->value();
    int v_i = dev_tab_widget_->ui_.spinBox_v_i->value();

    datc_interface_->setVelocityPiGain(v_p, v_i);
}

void MainWindow::dev_setGainC() {
    int c_p = dev_tab_widget_->ui_.spinBox_c_p->value();
    int c_i = dev_tab_widget_->ui_.spinBox_c_i->value();

    datc_interface_->setCurrentPiGain(c_p, c_i);
}

void MainWindow::dev_customCmd() {
    int addr    = dev_tab_widget_->ui_.spinBox_custom_cmd->value();
    int value_1 = dev_tab_widget_->ui_.spinBox_custom_cmd_value_1->value();
    int value_2 = dev_tab_widget_->ui_.spinBox_custom_cmd_value_2->value();
    int value_3 = dev_tab_widget_->ui_.spinBox_custom_cmd_value_3->value();

    datc_interface_->customCmd(addr, value_1, value_2, value_3);
}

void MainWindow::dev_resetPos() {
    datc_interface_->resetPosition();
}

void MainWindow::dev_setElecAngle() {
    int16_t elec_angle_deg = (int16_t) dev_tab_widget_->ui_.spinBox_elec_angle->value();
    datc_interface_->changeElectricalAngle(elec_angle_deg);
}

// 그리퍼 매핑표 CSV 일괄 임포트. CSV 열: index,stroke_mm,force_N,payload_x10 (헤더/빈줄/'#' 무시).
// 각 행을 writeMapEntry(16비트)로 전송 후 saveGripperMap으로 EEPROM 영구 저장.
// ※ 매핑표의 스펙(stroke/force/payload)만 갱신 — 기구학(종류 분류/상수)은 펌웨어 영역(별도).
void MainWindow::dev_importGripperMap() {
    if (!datc_interface_->getConnectionState()) {
        printf("[MapImport] 연결되지 않음 — Modbus 연결 후 다시 시도.\n");
        QMessageBox::warning(this, "그리퍼 맵 임포트", "먼저 Modbus로 그리퍼에 연결하세요.");
        return;
    }
    QString path = QFileDialog::getOpenFileName(this, "그리퍼 맵 CSV 선택", "",
                                                "CSV files (*.csv);;All files (*.*)");
    if (path.isEmpty()) return;

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        printf("[MapImport] 파일 열기 실패: %s\n", path.toStdString().c_str());
        return;
    }

    QTextStream in(&file);
    int written = 0, skipped = 0, failed = 0;
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty() || line.startsWith("#")) continue;

        QStringList col = line.split(',');
        bool okIdx = false;
        int idx = col.value(0).trimmed().toInt(&okIdx);
        if (!okIdx) continue;                 // 헤더/비숫자 행 무시
        if (col.size() < 4) { skipped++; continue; }

        bool ok1, ok2, ok3;
        int stroke  = col.value(1).trimmed().toInt(&ok1);
        int force   = col.value(2).trimmed().toInt(&ok2);
        int payload = col.value(3).trimmed().toInt(&ok3);
        if (!ok1 || !ok2 || !ok3 || idx < 0 || idx > 127 ||
            stroke < 0 || force < 0 || payload < 0) { skipped++; continue; }

        // writeMapEntry는 동기 Modbus 쓰기(응답 대기)라 자연 페이싱됨. 즉시명령이라 RAM맵 갱신.
        if (datc_interface_->writeMapEntry((uint16_t) idx, (uint16_t) stroke,
                                           (uint16_t) force, (uint16_t) payload)) {
            written++;
        } else {
            failed++;
        }
    }
    file.close();

    // RAM맵 → EEPROM 영구 저장. 펌웨어는 저장 결과를 상태 워드 bit15(EEPROM Save Fail)로만 알려주므로,
    // 저장(약 0.4 s)이 끝나 상태가 갱신될 시간을 기다린 뒤 bit15를 확인한다 — 확인 없이 "완료"로 표시하면
    // 저장에 실패해 재부팅 후 맵이 되돌아가도 성공으로 오인한다.
    QApplication::setOverrideCursor(Qt::WaitCursor);
    const bool sent = datc_interface_->saveGripperMap();
    uint16_t status = 0;
    bool got_status = false;
    if (sent) {
        QThread::msleep(1500);
        got_status = datc_interface_->readStatusWord(status);
    }
    QApplication::restoreOverrideCursor();

    const QString counts = QString("%1개 기록, %2개 스킵, %3개 전송 실패").arg(written).arg(skipped).arg(failed);
    printf("[MapImport] %s (%s)\n", counts.toStdString().c_str(), path.toStdString().c_str());

    if (!sent) {
        QMessageBox::warning(this, "그리퍼 맵 임포트", counts + "\n\n저장 명령(218) 전송에 실패했습니다.");
    } else if (!got_status) {
        QMessageBox::warning(this, "그리퍼 맵 임포트",
                             counts + "\n\n저장 결과를 확인하지 못했습니다(상태 읽기 실패). 연결을 확인하세요.");
    } else if (status & (0x01 << 15)) {
        QMessageBox::warning(this, "그리퍼 맵 임포트",
                             counts + "\n\nEEPROM 저장에 실패했습니다(상태 워드 bit15). 재부팅하면 이전 맵으로 "
                             "돌아갑니다. Dev 탭 'Encoder / Map'에서 Error Clear 후 다시 시도하세요.");
    } else {
        QMessageBox::information(this, "그리퍼 맵 임포트", counts + "\n\nEEPROM 저장을 확인했습니다.");
    }
}

// Dev 탭 "Encoder / Map" 서브탭 — 코드로 추가(.ui 미변경).
//   - 엔코더 고조파 보정: 계수 입력(219)·EEPROM 저장(220)·조회(221). 측정은 tools/enc_harm_calib.py
//   - 그리퍼 맵 조회(217): 저장된 사양표의 한 항목을 reg 24~27 응답으로 확인
//   - Error Clear(9): 래치되는 상태 워드 bit14(통신 오버런)·bit15(EEPROM 저장 실패) 해제
void MainWindow::buildCalibrationTab() {
    QWidget *page = new QWidget(dev_tab_widget_);
    QVBoxLayout *vl = new QVBoxLayout(page);

    auto requireConn = [this]() {
        if (!datc_interface_->getConnectionState()) {
            QMessageBox::warning(this, "Modbus", "먼저 Modbus로 그리퍼에 연결하세요.");
            return false;
        }
        return true;
    };

    // --- 엔코더 고조파 자동 보정 (222/223, v3.02~) — 동작은 아래 계수 표(readAllHarm) 정의 뒤에서 연결 ---
    QGroupBox *grpAuto = new QGroupBox("엔코더 고조파 자동 보정 (명령 222 시작·취소 / 223 결과, 펌웨어 v3.02 이상)", page);
    QVBoxLayout *av = new QVBoxLayout(grpAuto);
    QLabel *autoNote = new QLabel(
        "툴(그리퍼)을 떼고 모터 축이 자유롭게 돌 수 있는 상태에서 실행합니다. 모터가 현재 위치에서 약 9.5회전 "
        "갔다가 되돌아오는 왕복을 반복하며(약 1분) 오차를 측정·계산하고, 오차가 줄었으면 결과를 EEPROM에 저장합니다. "
        "모터 Enable과 전기각 캘리브레이션이 필요하고 Initialize는 필요 없습니다. 실행 중 다른 명령을 보내거나 "
        "버튼을 누르면 취소되고 원래 계수로 돌아갑니다.", grpAuto);
    autoNote->setWordWrap(true);
    av->addWidget(autoNote);

    QHBoxLayout *autoRow = new QHBoxLayout();
    QPushButton  *btnAutoStart  = new QPushButton("자동 보정 시작 (222)", grpAuto);
    QPushButton  *btnAutoCancel = new QPushButton("취소", grpAuto);
    QProgressBar *autoProgress  = new QProgressBar(grpAuto);
    autoProgress->setRange(0, 100);
    autoProgress->setValue(0);
    btnAutoStart->setMinimumHeight(30);
    btnAutoCancel->setMinimumHeight(30);
    btnAutoCancel->setEnabled(false);
    autoRow->addWidget(btnAutoStart, 2);
    autoRow->addWidget(btnAutoCancel, 1);
    autoRow->addWidget(autoProgress, 3);
    av->addLayout(autoRow);
    QLabel *autoStatus = new QLabel("-", grpAuto);
    autoStatus->setWordWrap(true);
    av->addWidget(autoStatus);
    vl->addWidget(grpAuto);

    // --- 엔코더 고조파 보정 (219~221) ---
    QGroupBox *grpHarm = new QGroupBox("엔코더 고조파 보정 (명령 219 입력 / 220 저장 / 221 조회)", page);
    QVBoxLayout *hv = new QVBoxLayout(grpHarm);
    QLabel *harmNote = new QLabel(
        "측정각 오차 d(φ) = A·cos kφ + B·sin kφ (k = 1~4). 단위는 s16 기계각 카운트(65536 = 360°)이며 "
        "진폭 √(A²+B²)은 2048(11.25°) 이하여야 합니다. 계수는 유닛마다 다르므로 위의 자동 보정을 쓰거나, "
        "툴을 뗄 수 없으면 tools/enc_harm_calib.py로 측정한 값을 넣습니다. 모터 Disable 상태에서 입력하는 것을 권장하며, Initialize·이동 중에 보낸 입력은 "
        "동작이 끝난 뒤 적용됩니다. 입력은 RAM에만 반영되므로 'EEPROM 저장'을 눌러야 재부팅 후에도 유지됩니다.",
        grpHarm);
    harmNote->setWordWrap(true);
    hv->addWidget(harmNote);

    QHBoxLayout *setRow = new QHBoxLayout();
    QSpinBox *spinK = new QSpinBox(grpHarm);
    spinK->setRange(1, kEncHarmCount);
    spinK->setPrefix("k = ");
    QSpinBox *spinA = new QSpinBox(grpHarm);
    spinA->setRange(-kEncHarmLimit, kEncHarmLimit);
    spinA->setPrefix("A = ");
    QSpinBox *spinB = new QSpinBox(grpHarm);
    spinB->setRange(-kEncHarmLimit, kEncHarmLimit);
    spinB->setPrefix("B = ");
    QPushButton *btnHarmSet = new QPushButton("계수 입력 (219)", grpHarm);
    setRow->addWidget(spinK, 1);
    setRow->addWidget(spinA, 1);
    setRow->addWidget(spinB, 1);
    setRow->addWidget(btnHarmSet, 1);
    hv->addLayout(setRow);

    QHBoxLayout *opRow = new QHBoxLayout();
    QPushButton *btnHarmRead  = new QPushButton("전체 조회 (221)", grpHarm);
    QPushButton *btnHarmSave  = new QPushButton("EEPROM 저장 (220)", grpHarm);
    QPushButton *btnHarmClear = new QPushButton("전체 해제 (219, k = 0)", grpHarm);
    opRow->addWidget(btnHarmRead, 1);
    opRow->addWidget(btnHarmSave, 1);
    opRow->addWidget(btnHarmClear, 1);
    hv->addLayout(opRow);

    QPlainTextEdit *harmView = new QPlainTextEdit(grpHarm);
    harmView->setReadOnly(true);
    harmView->setMinimumHeight(90);
    harmView->setMaximumHeight(120);
    hv->addWidget(harmView);
    vl->addWidget(grpHarm);

    for (QPushButton *btn : {btnHarmSet, btnHarmRead, btnHarmSave, btnHarmClear}) {
        btn->setMinimumHeight(30);
    }

    // 차수 1~4를 조회해 표시한다. 응답이 하나라도 있으면 true.
    auto readAllHarm = [this, harmView]() {
        QStringList lines;
        bool any = false;
        EncHarmEntry last;
        for (uint16_t k = 1; k <= kEncHarmCount; ++k) {
            EncHarmEntry e;
            if (datc_interface_->queryEncoderHarmonic(k, e)) {
                any  = true;
                last = e;
                const double amp_deg = sqrt((double) e.a * e.a + (double) e.b * e.b) * 360.0 / 65536.0;
                lines << QString("k=%1   A=%2   B=%3   (진폭 %4°)").arg(k).arg(e.a).arg(e.b).arg(amp_deg, 0, 'f', 3);
            } else {
                lines << QString("k=%1   응답 없음").arg(k);
            }
        }
        if (any) {
            lines << QString("보정 %1 · 마지막 저장(이번 전원 투입 후): %2")
                         .arg(last.active ? "동작 중" : "꺼짐")
                         .arg(last.last_save_fail ? "실패" : (last.last_save_ok ? "성공" : "없음"));
        }
        harmView->setPlainText(lines.join("\n"));
        return any;
    };

    QObject::connect(btnHarmSet, &QPushButton::clicked, this, [this, requireConn, readAllHarm, spinK, spinA, spinB]() {
        if (!requireConn()) return;
        const int a = spinA->value();
        const int b = spinB->value();
        if (a * a + b * b > kEncHarmLimit * kEncHarmLimit) {
            QMessageBox::warning(this, "엔코더 보정",
                                 "진폭 √(A²+B²)이 2048을 넘습니다. 펌웨어가 무시하므로 보내지 않았습니다.");
            return;
        }
        QApplication::setOverrideCursor(Qt::WaitCursor);
        const bool ok = datc_interface_->setEncoderHarmonic((uint16_t) spinK->value(), (int16_t) a, (int16_t) b);
        QThread::msleep(50);   // 219는 펌웨어 큐 경로 — 대기 상태면 다음 주기에 적용된다
        readAllHarm();
        QApplication::restoreOverrideCursor();
        if (!ok) {
            QMessageBox::warning(this, "엔코더 보정", "계수 입력 명령 전송에 실패했습니다.");
        }
    });

    QObject::connect(btnHarmRead, &QPushButton::clicked, this, [this, requireConn, readAllHarm]() {
        if (!requireConn()) return;
        QApplication::setOverrideCursor(Qt::WaitCursor);
        const bool any = readAllHarm();
        QApplication::restoreOverrideCursor();
        if (!any) {
            QMessageBox::warning(this, "엔코더 보정",
                                 "조회 응답이 없습니다. 엔코더 보정(221)을 지원하지 않는 이전 펌웨어일 수 있습니다.");
        }
    });

    QObject::connect(btnHarmSave, &QPushButton::clicked, this, [this, requireConn, readAllHarm]() {
        if (!requireConn()) return;
        QApplication::setOverrideCursor(Qt::WaitCursor);
        const bool sent = datc_interface_->saveEncoderHarmonics();
        EncHarmEntry e;
        bool got = false;
        if (sent) {
            QThread::msleep(500);   // EEPROM 기록·재읽기 검증 대기
            got = datc_interface_->queryEncoderHarmonic(1, e);   // 상태 bit1/bit2 = 마지막 저장 결과
            readAllHarm();
        }
        QApplication::restoreOverrideCursor();
        if (!sent) {
            QMessageBox::warning(this, "엔코더 보정", "저장 명령(220) 전송에 실패했습니다.");
        } else if (!got) {
            QMessageBox::warning(this, "엔코더 보정", "저장 결과를 확인하지 못했습니다(조회 응답 없음).");
        } else if (e.last_save_fail || !e.last_save_ok) {
            QMessageBox::warning(this, "엔코더 보정",
                                 "EEPROM 저장에 실패했습니다. 재부팅하면 계수가 사라집니다. 다시 시도하세요.");
        } else {
            QMessageBox::information(this, "엔코더 보정", "EEPROM 저장을 확인했습니다.");
        }
    });

    QObject::connect(btnHarmClear, &QPushButton::clicked, this, [this, requireConn, readAllHarm]() {
        if (!requireConn()) return;
        if (QMessageBox::question(this, "엔코더 보정",
                "모든 계수를 지우고 보정을 끕니다(RAM).\nEEPROM에서도 지우려면 이어서 'EEPROM 저장'을 누르세요.\n\n계속할까요?")
                != QMessageBox::Yes) {
            return;
        }
        QApplication::setOverrideCursor(Qt::WaitCursor);
        datc_interface_->clearEncoderHarmonics();
        QThread::msleep(50);
        readAllHarm();
        QApplication::restoreOverrideCursor();
    });

    // 자동 보정 동작: 시작 후 reg 33을 0.5 s마다 읽어 진행률을 보이고, 끝나면 223으로 결과를 받는다.
    struct AutoCalRun {
        uint16_t      pre          = 0;       // 시작 직전 상태 워드 (이전 결과와 구분용)
        bool          seen_running = false;
        QElapsedTimer t;
    };
    auto autoRun   = std::make_shared<AutoCalRun>();
    QTimer *autoTimer = new QTimer(page);
    autoTimer->setInterval(500);

    QObject::connect(btnAutoStart, &QPushButton::clicked, this,
                     [this, requireConn, autoRun, autoTimer, btnAutoStart, btnAutoCancel, autoProgress, autoStatus]() {
        if (!requireConn()) return;
        if (!datc_interface_->getDatcStatus().enable) {
            QMessageBox::warning(this, "엔코더 자동 보정", "모터가 Disable 상태입니다. 먼저 Motor Enable을 하세요.");
            return;
        }
        DeviceInfo info;
        if (!datc_interface_->readDeviceInfoNow(info) || info.fw_version < kAutoCalMinFw) {
            QMessageBox::warning(this, "엔코더 자동 보정",
                                 QString("연결된 펌웨어(%1)는 자동 보정을 지원하지 않습니다. v3.02 이상으로 업데이트하세요.")
                                     .arg(info.valid ? QString::fromStdString(matcFwVersionStr(info.fw_version))
                                                     : QString("장치 정보 없음")));
            return;
        }
        if (QMessageBox::question(this, "엔코더 자동 보정",
                "툴(그리퍼)을 분리했고 모터 축이 자유롭게 돌 수 있는지 확인하세요.\n"
                "모터가 현재 위치에서 약 1분 동안 여러 바퀴 회전합니다.\n\n시작할까요?")
                != QMessageBox::Yes) {
            return;
        }
        uint16_t pre = 0;
        datc_interface_->readAutoEncHarmonicStatus(pre);
        if (!datc_interface_->startAutoEncHarmonic()) {
            QMessageBox::warning(this, "엔코더 자동 보정", "시작 명령(222) 전송에 실패했습니다.");
            return;
        }
        autoRun->pre          = pre;
        autoRun->seen_running = false;
        autoRun->t.start();
        btnAutoStart->setEnabled(false);
        btnAutoCancel->setEnabled(true);
        autoProgress->setValue(0);
        autoStatus->setText("시작 요청을 보냈습니다…");
        autoTimer->start();
    });

    QObject::connect(btnAutoCancel, &QPushButton::clicked, this, [this, autoStatus]() {
        datc_interface_->cancelAutoEncHarmonic();
        autoStatus->setText("취소 요청을 보냈습니다. 진행 중인 이동(최대 약 2초)을 마치고 멈춥니다…");
    });

    QObject::connect(autoTimer, &QTimer::timeout, this,
                     [this, autoRun, autoTimer, btnAutoStart, btnAutoCancel, autoProgress, autoStatus, readAllHarm]() {
        uint16_t st = 0;
        if (!datc_interface_->getConnectionState() || !datc_interface_->readAutoEncHarmonicStatus(st)) {
            if (!datc_interface_->getConnectionState()) {
                autoTimer->stop();
                btnAutoStart->setEnabled(true);
                btnAutoCancel->setEnabled(false);
                autoStatus->setText("Modbus 연결이 끊겨 진행 상황을 확인할 수 없습니다.");
            }
            return;
        }
        const AutoCalState s = autoCalState(st);
        if (s == AutoCalState::RUNNING) {
            autoRun->seen_running = true;
            autoProgress->setValue(autoCalLow(st));
            autoStatus->setText(QString("진행 중 — 측정 %1회차 (%2%)").arg(autoCalMeas(st)).arg(autoCalLow(st)));
            return;
        }
        // 실행 중을 본 적이 없으면, 상태가 바뀌었거나(시작 거부) 3 s가 지났을 때 결과로 본다
        if (!autoRun->seen_running && st == autoRun->pre && autoRun->t.elapsed() < 3000) {
            return;
        }
        autoTimer->stop();
        btnAutoStart->setEnabled(true);
        btnAutoCancel->setEnabled(false);

        if (s == AutoCalState::DONE) {
            autoProgress->setValue(100);
            AutoEncHarmResult r2;
            AutoEncHarmResult r4;
            const bool ok2 = datc_interface_->queryAutoEncHarmonicResult(2, r2);
            const bool ok4 = datc_interface_->queryAutoEncHarmonicResult(4, r4);
            readAllHarm();
            QString msg = (autoCalLow(st) == 1)
                ? QString("보정할 오차가 없어(잡음 수준) 계수 0으로 저장했습니다.")
                : QString("측정 %1회로 보정을 마치고 EEPROM에 저장했습니다.").arg(autoCalMeas(st));
            if (ok2 && ok4) {
                msg += QString("\n위치오차 2차 %1° → %2°,  4차 %3° → %4°")
                           .arg(r2.before_mdeg / 1000.0, 0, 'f', 3).arg(r2.after_mdeg / 1000.0, 0, 'f', 3)
                           .arg(r4.before_mdeg / 1000.0, 0, 'f', 3).arg(r4.after_mdeg / 1000.0, 0, 'f', 3);
            }
            autoStatus->setText(msg);
            QMessageBox::information(this, "엔코더 자동 보정", msg);
        } else if (s == AutoCalState::FAILED) {
            autoProgress->setValue(0);
            const QString msg = QString::fromStdString(autoCalErrorText(autoCalLow(st)));
            readAllHarm();
            autoStatus->setText("실패 — " + msg);
            QMessageBox::warning(this, "엔코더 자동 보정", msg);
        } else if (s == AutoCalState::CANCELLED) {
            autoProgress->setValue(0);
            readAllHarm();
            autoStatus->setText("취소되었습니다. 원래 계수로 되돌렸습니다.");
        } else {
            autoStatus->setText("장치가 자동 보정 상태를 보고하지 않습니다(펌웨어가 지원하지 않을 수 있습니다).");
        }
    });

    // --- 그리퍼 맵 조회 (217) ---
    QGroupBox *grpMap = new QGroupBox("그리퍼 맵 조회 (명령 217)", page);
    QHBoxLayout *mapRow = new QHBoxLayout(grpMap);
    QSpinBox *spinIdx = new QSpinBox(grpMap);
    spinIdx->setRange(0, 127);
    spinIdx->setPrefix("Index ");
    QPushButton *btnMapRead = new QPushButton("조회", grpMap);
    btnMapRead->setMinimumHeight(30);
    QLabel *mapResult = new QLabel("-", grpMap);
    mapRow->addWidget(spinIdx, 1);
    mapRow->addWidget(btnMapRead, 1);
    mapRow->addWidget(mapResult, 3);
    vl->addWidget(grpMap);

    QObject::connect(btnMapRead, &QPushButton::clicked, this, [this, requireConn, spinIdx, mapResult]() {
        if (!requireConn()) return;
        GripperMapEntry e;
        QApplication::setOverrideCursor(Qt::WaitCursor);
        const bool ok = datc_interface_->queryMapEntry((uint16_t) spinIdx->value(), e);
        QApplication::restoreOverrideCursor();
        mapResult->setText(ok ? QString("Index %1 — 스트로크 %2 mm, 파지력 %3 N, 가반하중 %4 kg")
                                    .arg(e.index).arg(e.stroke_mm).arg(e.force_n)
                                    .arg((double) e.payload_x10 / 10.0, 0, 'f', 1)
                              : QString("응답 없음"));
    });

    // --- 일체형 그리퍼 고정 ID (224, v3.04~) ---
    QGroupBox *grpFixed = new QGroupBox("일체형 그리퍼 고정 ID (명령 224, 펌웨어 v3.04 이상)", page);
    QVBoxLayout *fixedVl = new QVBoxLayout(grpFixed);
    QLabel *fixedNote = new QLabel(
        "포고핀이 없는 일체형 그리퍼는 ID 핀이 '없음'으로 읽혀 맵 사양(스트로크·파지력)과 기구 종류를 쓰지 못합니다. "
        "그리퍼 ID를 저장해 두면 ID 핀이 '없음'일 때 이 ID를 쓰고 연결 감지도 '연결됨'으로 보고합니다"
        "(ID 핀이 읽히면 핀이 우선). MATC(툴 체인저)에는 설정하지 마세요 — 툴을 빼도 그리퍼가 있는 것으로 보입니다.",
        grpFixed);
    fixedNote->setWordWrap(true);
    QHBoxLayout *fixedRow = new QHBoxLayout();
    QSpinBox *spinFixed = new QSpinBox(grpFixed);
    spinFixed->setRange(0, kFixedGripperIdMax);
    spinFixed->setPrefix("ID ");
    QPushButton *btnFixedSet   = new QPushButton("설정·저장 (224)", grpFixed);
    QPushButton *btnFixedClear = new QPushButton("해제", grpFixed);
    btnFixedSet->setMinimumHeight(30);
    btnFixedClear->setMinimumHeight(30);
    QLabel *fixedState = new QLabel("-", grpFixed);
    fixedRow->addWidget(spinFixed, 1);
    fixedRow->addWidget(btnFixedSet, 1);
    fixedRow->addWidget(btnFixedClear, 1);
    fixedRow->addWidget(fixedState, 2);
    fixedVl->addWidget(fixedNote);
    fixedVl->addLayout(fixedRow);
    vl->addWidget(grpFixed);

    // 저장된 값 표시 — 장치 정보(reg 34, 약 1 s마다 갱신)를 0.5 s마다 반영
    QTimer *fixedTimer = new QTimer(grpFixed);
    QObject::connect(fixedTimer, &QTimer::timeout, this, [this, fixedState]() {
        const DeviceInfo info = datc_interface_->getDeviceInfo();
        QString t;
        if (!datc_interface_->getConnectionState() || !info.valid) {
            t = "-";
        } else if (info.fw_version < kFixedGripperIdMinFw) {
            t = "펌웨어 v3.04 이상 필요";
        } else if (!info.hasFixedId()) {
            t = "저장된 고정 ID 없음";
        } else {
            t = QString("저장된 고정 ID %1 — %2").arg(info.fixed_gripper_id)
                    .arg(info.fixedIdInUse() ? "사용 중" : "ID 핀이 읽혀 사용 안 함");
        }
        if (fixedState->text() != t) {
            fixedState->setText(t);
        }
    });
    fixedTimer->start(500);

    // 연결과 펌웨어 버전(v3.04~)을 확인 창보다 먼저 본다
    auto requireFixedIdFw = [this, requireConn]() {
        if (!requireConn()) return false;
        const DeviceInfo info = datc_interface_->getDeviceInfo();
        if (!info.valid || info.fw_version < kFixedGripperIdMinFw) {
            QMessageBox::warning(this, "고정 ID", "펌웨어 v3.04 이상에서만 쓸 수 있습니다.");
            return false;
        }
        return true;
    };

    // 224 전송 후 EEPROM 저장 결과를 reg 34(장치 정보 폴링)와 상태 bit15로 확인한다
    auto sendFixedId = [this](uint16_t id, const QString &done_msg) {
        if (!datc_interface_->setFixedGripperId(id)) {
            QMessageBox::warning(this, "고정 ID", "명령(224) 전송에 실패했습니다.");
            return;
        }
        QTimer::singleShot(2000, this, [this, id, done_msg]() {
            const DeviceInfo now = datc_interface_->getDeviceInfo();
            if ((now.fixed_gripper_id == id) && !datc_interface_->getDatcStatus().eep_save_fail) {
                QMessageBox::information(this, "고정 ID", done_msg);
            } else {
                QMessageBox::warning(this, "고정 ID",
                                     "저장을 확인하지 못했습니다(EEPROM 저장 실패 또는 응답 없음). 다시 시도하세요.");
            }
        });
    };

    QObject::connect(btnFixedSet, &QPushButton::clicked, this, [this, requireFixedIdFw, spinFixed, sendFixedId]() {
        if (!requireFixedIdFw()) return;
        const uint16_t   id   = (uint16_t) spinFixed->value();
        const DeviceInfo info = datc_interface_->getDeviceInfo();
        const DatcStatus st   = datc_interface_->getDatcStatus();
        QString msg = QString("그리퍼 ID %1을(를) 일체형 그리퍼 고정 ID로 EEPROM에 저장합니다.\n"
                              "ID 핀이 '없음'일 때 이 ID의 맵 사양과 기구 종류를 쓰고, 연결 감지를 '연결됨'으로 보고합니다.\n\n")
                          .arg(id);
        if ((st.gripper_id != kFixedGripperIdNone) && !info.fixedIdInUse()) {
            msg += QString("주의: 지금 ID 핀으로 그리퍼 ID %1이(가) 읽히고 있습니다. MATC(툴 체인저)로 보입니다.\n"
                           "MATC에 고정 ID를 넣으면 툴을 빼도 그리퍼가 있는 것으로 보입니다.\n\n")
                       .arg(st.gripper_id);
        }
        msg += "계속할까요?";
        if (QMessageBox::question(this, "고정 ID", msg) != QMessageBox::Yes) return;
        sendFixedId(id, QString("고정 ID %1을(를) 저장했습니다.").arg(id));
    });

    QObject::connect(btnFixedClear, &QPushButton::clicked, this, [this, requireFixedIdFw, sendFixedId]() {
        if (!requireFixedIdFw()) return;
        if (QMessageBox::question(this, "고정 ID", "저장된 고정 ID를 지웁니다. 계속할까요?") != QMessageBox::Yes) return;
        sendFixedId(kFixedGripperIdNone, "고정 ID를 지웠습니다.");
    });

    // 구버전 펌웨어에 연결되면 V3 전용 그룹을 막는다(Error Clear는 구버전에도 있다)
    v3_only_widgets_ << grpAuto << grpHarm << grpMap << grpFixed;

    // --- Error Clear (9) ---
    QGroupBox *grpErr = new QGroupBox("경고·오류 해제 (명령 9 Error Clear)", page);
    QHBoxLayout *errRow = new QHBoxLayout(grpErr);
    QLabel *errNote = new QLabel(
        "상태 워드 bit14(통신 오버런)·bit15(EEPROM 저장 실패)는 한 번 켜지면 이 명령 전까지 유지됩니다. "
        "상단 Mode 아랫줄 경고에 'Comm Overrun', 'EEPROM Save Fail'로 나타납니다.", grpErr);
    errNote->setWordWrap(true);
    QPushButton *btnErrClear = new QPushButton("Error Clear (9)", grpErr);
    btnErrClear->setMinimumHeight(30);
    errRow->addWidget(errNote, 3);
    errRow->addWidget(btnErrClear, 1);
    vl->addWidget(grpErr);

    QObject::connect(btnErrClear, &QPushButton::clicked, this, [this, requireConn]() {
        if (!requireConn()) return;
        datc_interface_->errorClear();
    });

    vl->addStretch(1);
    // 내용이 창 높이보다 길어 설명·표가 눌리지 않도록 스크롤 영역에 넣는다
    QScrollArea *scroll = new QScrollArea(dev_tab_widget_);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setWidget(page);
    dev_tab_widget_->ui_.tabWidget_dev->addTab(scroll, "Encoder / Map");
}

// --- Firmware Update (Dev tab) ---------------------------------------------

void MainWindow::dev_fwBrowse() {
    QString last = dev_tab_widget_->ui_.lineEdit_fw_path->text();
    QString start_dir = last.isEmpty() ? QString() : QFileInfo(last).absolutePath();
    QString path = QFileDialog::getOpenFileName(
        this, "Select application .bin", start_dir,
        "Application binary (*.bin);;All files (*)");
    if (!path.isEmpty()) {
        dev_tab_widget_->ui_.lineEdit_fw_path->setText(path);
    }
}

void MainWindow::dev_fwStart() {
    if (fw_updater_->isRunning()) {
        dev_fwOnLog("Update already in progress.");
        return;
    }

    QString bin = dev_tab_widget_->ui_.lineEdit_fw_path->text().trimmed();
    if (bin.isEmpty() || !QFileInfo::exists(bin)) {
        dev_fwOnLog("Pick a valid .bin file first.");
        return;
    }

    QString port    = QString::fromStdString(modbus_widget_->ui_.comboBox_serial_port->currentText().toStdString());
    int     baud    = modbus_widget_->ui_.comboBox_baudrate->currentText().toInt();
    bool    auto_rst = dev_tab_widget_->ui_.checkBox_fw_auto_reset->isChecked();
    bool    jump    = dev_tab_widget_->ui_.checkBox_fw_jump->isChecked();

    if (port.isEmpty()) {
        dev_fwOnLog("No serial port selected (Modbus tab → Port).");
        return;
    }
    if (baud <= 0) baud = 115200;

    // Clear log + progress UI for new run.
    dev_tab_widget_->ui_.textEdit_fw_log->clear();
    dev_tab_widget_->ui_.progressBar_fw->setValue(0);
    dev_tab_widget_->ui_.pushButton_fw_start->setEnabled(false);
    dev_tab_widget_->ui_.pushButton_fw_cancel->setEnabled(true);

    dev_fwOnLog(QString("Target: %1 @ %2 8N1").arg(port).arg(baud));

    // Step 1: SYSTEM_RESET으로 부트로더에 진입시킨다(부트로더가 리셋 요청 표시를 보고 30초간 동기화 대기).
    // Modbus가 끊겨 있으면 Modbus 탭 설정(포트·속도·주소)으로 잠깐 연결해 리셋을 보낸다 — 연결을
    // 깜빡하면 리셋 없이 앱 상태로 남아 동기화가 항상 실패했다.
    const int kManualSyncSec = 30;   // 사람이 전원을 껐다 켤 시간
    fw_reset_sent_ = false;
    fw_app_baud_   = baud;
    if (auto_rst) {
        // 포트가 열려 있어도 장치가 실제로 응답하는지 확인한다 — "Read error" 상태에서 보낸 리셋 명령은
        // 장치에 전달되지 않아, 리셋한 줄 알고 기다리다 동기화가 실패했다.
        bool alive = datc_interface_->getConnectionState() && datc_interface_->probe();
        if (!alive) {
            dev_fwOnLog(datc_interface_->getConnectionState()
                ? "Modbus 포트는 열려 있지만 장치가 응답하지 않습니다(Read error) → 다시 연결해 봅니다..."
                : "Modbus 미연결 → Modbus 탭 설정으로 연결해 SYSTEM_RESET을 보냅니다...");
            alive = reconnectAt(baud);
        }
        // 리셋을 보내기 전에(장치에 아무것도 쓰기 전에) 업데이트 파일과 장치의 모델을 비교한다.
        if (!fwCheckModel(bin, alive)) {
            fwAbortStart("모델 확인 단계에서 중단했습니다. 장치에는 아무것도 쓰지 않았습니다.");
            return;
        }
        if (alive) {
            dev_fwOnLog("Sending SYSTEM_RESET via Modbus to enter bootloader...");
            datc_interface_->systemReset();
            // libmodbus holds the same COM port; close it so the updater can reopen it raw.
            datc_interface_->modbusRelease();
            fw_reset_sent_ = true;
            dev_fwOnLog("Modbus disconnected.");
        } else {
            releaseModbus();
            dev_fwOnLog(QString("장치가 Modbus에 응답하지 않아 리셋을 보내지 못했습니다(포트·Baudrate·Slave Address 확인, "
                                "통신속도를 바꿨다면 Modbus 탭 '속도 자동 검색'). 이전 업데이트가 중단돼 부트로더에 머물러 "
                                "있을 수도 있습니다. 동기화가 진행되는 %1초 안에 보드 전원을 껐다 켜세요.").arg(kManualSyncSec));
        }
    } else {
        // 수동 리셋: 지금 연결돼 응답하는 장치가 있으면 그 모델과 비교, 없으면 사용자 확인
        const bool alive = datc_interface_->getConnectionState() && datc_interface_->probe();
        if (!fwCheckModel(bin, alive)) {
            fwAbortStart("모델 확인 단계에서 중단했습니다. 장치에는 아무것도 쓰지 않았습니다.");
            return;
        }
        dev_fwOnLog(QString("Auto-reset off — 동기화가 진행되는 %1초 안에 보드 전원을 껐다 켜세요.").arg(kManualSyncSec));
        if (datc_interface_->getConnectionState()) {
            datc_interface_->modbusRelease();
            dev_fwOnLog("Modbus released to free the serial port.");
        }
    }

    // 부트로더는 항상 115200(펌웨어 MATC_BOOT_UART_BAUDRATE). SYSTEM_RESET은 위에서 현재(앱) 속도의
    // Modbus 연결로 이미 보냈으므로, 부트로더와는 115200으로 통신한다 — 장치 속도를 바꿔도 업데이트 가능.
    const int kBootloaderBaud = 115200;
    if (baud != kBootloaderBaud) {
        dev_fwOnLog(QString("App %1 bps -> bootloader %2 bps").arg(baud).arg(kBootloaderBaud));
    }
    fw_updater_->startUpdate(port, kBootloaderBaud, bin, jump, fw_reset_sent_ ? 10 : kManualSyncSec);
}

// 업데이트 파일(앱 이미지 정보 블록 "MATCINF1")과 연결된 장치(reg 28)의 모델을 비교한다 (v3.01~).
// 다른 모델 펌웨어를 올리면 극쌍수·전류 스케일이 맞지 않는다 — 특히 저장된 전기각 오프셋이 다른 모델 기준이
// 되어 토크 방향이 뒤집힐 수 있다(MATC-10과 MATC-20은 전기각 record 매직이 같다). 확실히 다르면 막고,
// 어느 한쪽이라도 확인할 수 없으면(이전 빌드·이전 펌웨어·무응답) 사용자에게 묻는다.
bool MainWindow::fwCheckModel(const QString &bin, bool device_alive) {
    const FirmwareUpdater::AppInfo img = FirmwareUpdater::readAppInfoFromFile(bin);
    DeviceInfo dev;
    const bool dev_ok = device_alive && datc_interface_->readDeviceInfoNow(dev);

    const QString img_str = img.found
        ? QString("%1 %2").arg(QString::fromStdString(matcModelName(img.model)))
                          .arg(QString::fromStdString(matcFwVersionStr(img.fw_version)))
        : QString("모델 정보 없음 (v3.01 이전 빌드)");
    const QString dev_str = dev_ok
        ? QString("%1 %2").arg(QString::fromStdString(matcModelName(dev.model)))
                          .arg(QString::fromStdString(matcFwVersionStr(dev.fw_version)))
        : (device_alive ? QString("모델 정보 없음 (v3.01 이전 펌웨어)") : QString("응답 없음 — 확인 불가"));
    dev_fwOnLog("업데이트 파일: " + img_str);
    dev_fwOnLog("연결된 장치: " + dev_str);

    if (img.found && dev_ok) {
        if (img.model != dev.model) {
            dev_fwOnLog("✗ 모델 불일치 — 업데이트하지 않습니다.");
            QMessageBox::critical(this, "펌웨어 업데이트",
                QString("업데이트 파일은 %1용인데 연결된 장치는 %2입니다.\n\n"
                        "다른 모델의 펌웨어를 올리면 모터 극쌍수·전류 스케일이 맞지 않아 위험합니다"
                        "(전기각이 어긋나 토크 방향이 뒤집힐 수 있음).\n"
                        "장치에 맞는 프로젝트에서 빌드한 MATC_V3.bin을 선택하세요.")
                    .arg(QString::fromStdString(matcModelName(img.model)))
                    .arg(QString::fromStdString(matcModelName(dev.model))));
            return false;
        }
        dev_fwOnLog("✓ 모델 일치");
        return true;
    }

    QString why;
    if (!img.found) {
        why += "• 업데이트 파일에 모델 정보가 없습니다 (v3.01 이전 빌드).\n";
    }
    if (!dev_ok) {
        why += device_alive ? "• 장치 펌웨어가 모델 정보를 제공하지 않습니다 (v3.01 이전).\n"
                            : "• 장치가 Modbus에 응답하지 않아 모델을 확인할 수 없습니다.\n";
    }
    const QString msg = QString(
        "업데이트 파일과 장치의 모델이 같은지 확인할 수 없습니다.\n\n%1\n"
        "업데이트 파일: %2\n연결된 장치: %3\n\n"
        "다른 모델(MATC-10 / MATC-20 / MATC-30)의 펌웨어를 올리면 위험합니다. "
        "장치와 같은 모델의 펌웨어가 맞는지 직접 확인했다면 계속하세요.\n\n계속할까요?")
        .arg(why).arg(img_str).arg(dev_str);
    if (QMessageBox::warning(this, "펌웨어 업데이트", msg, QMessageBox::Yes | QMessageBox::No, QMessageBox::No)
            != QMessageBox::Yes) {
        return false;
    }
    dev_fwOnLog("모델 확인 불가 — 사용자 확인 후 진행합니다.");
    return true;
}

void MainWindow::fwAbortStart(const QString &reason) {
    dev_fwOnLog(reason);
    dev_tab_widget_->ui_.pushButton_fw_start->setEnabled(!legacy_feature_lock_);
    dev_tab_widget_->ui_.pushButton_fw_cancel->setEnabled(false);
}

void MainWindow::dev_fwCancel() {
    if (fw_updater_->isRunning()) {
        fw_updater_->cancel();
        dev_fwOnLog("Cancel requested...");
    }
}

void MainWindow::dev_fwOnLog(QString msg) {
    dev_tab_widget_->ui_.textEdit_fw_log->append(msg);
}

void MainWindow::dev_fwOnProgress(int pct) {
    dev_tab_widget_->ui_.progressBar_fw->setValue(pct);
}

void MainWindow::dev_fwOnFinished(bool ok, QString msg) {
    dev_fwOnLog(ok ? QString("✓ ") + msg : QString("✗ ") + msg);
    dev_tab_widget_->ui_.pushButton_fw_start->setEnabled(!legacy_feature_lock_);
    dev_tab_widget_->ui_.pushButton_fw_cancel->setEnabled(false);

    // 업데이트 후 새 펌웨어가 실제로 기동해 Modbus에 응답하는지 확인한다(앱 기동 시간을 고려해
    // 2초 뒤부터 1초 간격 최대 3회). 앱 통신속도는 업데이트 전 연결에 쓰던 Baudrate(EEPROM 저장값).
    if (ok && dev_tab_widget_->ui_.checkBox_fw_jump->isChecked()) {
        const int app_baud = modbus_widget_->ui_.comboBox_baudrate->currentText().toInt();
        dev_fwOnLog(QString("새 펌웨어 기동 대기 후 %1 bps로 Modbus 재연결을 확인합니다...").arg(app_baud));
        QTimer::singleShot(2000, this, [this, app_baud]() { dev_fwVerifyApp(app_baud, 3); });
    }

    // SYSTEM_RESET을 보냈는데 동기화가 실패했다면, 앱이 곧바로 다시 응답하는지로 원인을 가른다.
    // (현재 부트로더는 리셋 요청을 받으면 30초간 부트로더에 머물러 그동안 Modbus 응답이 없다.)
    if (!ok && fw_reset_sent_ && fw_updater_->syncTimedOut()) {
        const int app_baud = fw_app_baud_;
        QTimer::singleShot(300, this, [this, app_baud]() {
            if (fw_updater_->isRunning()) {
                return;
            }
            if (reconnectAt(app_baud)) {
                dev_fwOnLog("→ 리셋 직후 앱이 바로 응답합니다: 보드에 부트로더가 없거나(디버그용 부트 스텁으로 앱 직행), "
                            "리셋 요청을 모르는 이전 부트로더입니다. ST-Link/CubeProgrammer로 현재 빌드의 "
                            "MATC_Bootloader.hex를 0x08000000에 구운 뒤 다시 시도하세요(앱은 지워지지 않음).");
            } else {
                releaseModbus();
                dev_fwOnLog("→ 리셋 후 Modbus 응답도 없습니다. 부트로더가 아직 대기 중(최대 30초)이거나 포트·배선 문제일 수 "
                            "있습니다. 30초 뒤 다시 시도하거나, Auto-reset을 끄고 동기화 중에 전원을 껐다 켜 보세요.");
            }
        });
    }
}

void MainWindow::dev_fwVerifyApp(int baud, int tries_left) {
    if (fw_updater_->isRunning()) {
        return;   // 그 사이 새 업데이트가 시작됨
    }
    if (reconnectAt(baud)) {
        dev_fwOnLog(QString("✓ 새 펌웨어가 Modbus에 응답합니다 (%1 bps) — 연결됨.").arg(baud));
        DeviceInfo info;
        if (datc_interface_->readDeviceInfoNow(info)) {
            dev_fwOnLog(QString("  장치: %1 %2")
                            .arg(QString::fromStdString(matcModelName(info.model)))
                            .arg(QString::fromStdString(matcFwVersionStr(info.fw_version))));
            if (!info.elangleCalibrated()) {
                dev_fwOnLog(info.elangleModelMismatch()
                    ? "  ⚠ 저장된 전기각이 다른 모델의 값이라 거부됐습니다 → Dev 탭에서 전기각 재설정(51) 후 사용하세요."
                    : "  ⚠ 전기각이 캘리브되지 않았습니다 → Dev 탭에서 전기각 설정(51) 후 사용하세요.");
            }
        }
        return;
    }
    if (tries_left > 1) {
        QTimer::singleShot(1000, this, [this, baud, tries_left]() { dev_fwVerifyApp(baud, tries_left - 1); });
        return;
    }
    releaseModbus();
    dev_fwOnLog("✗ 업데이트 후 Modbus 응답이 없습니다. 보드 전원을 껐다 켠 뒤 Modbus 탭에서 다시 연결해 보세요. "
                "전원을 다시 켠 뒤에는 응답한다면, 부트로더→앱 점프 직후의 통신 초기화 문제입니다.");
}

#ifndef RCLCPP__RCLCPP_HPP_
// TCP comm. related functions
void MainWindow::startTcpComm() {
    string addr          = tcp_widget_->ui_.lineEdit_tcp_addr->text().toStdString();
    uint16_t socket_port = tcp_widget_->ui_.lineEdit_tcp_port->text().toUInt();

    datc_interface_->initTcp(addr, socket_port);
}

void MainWindow::stopTcpComm() {
    datc_interface_->releaseTcp();
}
#endif

// 원격 장치 통신속도 변경(TCP 명령 52, ROS2 set_modbus_baudrate): '장치 속도 변경' 버튼과 같은 절차를 확인 창 없이
// 진행하고, 결과를 reportDeviceBaudrate(ok, baudrate = 처리 뒤 GCS 연결 속도, result)로 돌려준다 — TCP는 상태 JSON의
// last_query(command 52), ROS2는 서비스 응답. 그동안 TCP 명령·ROS2 서비스 요청은 기다린다.
// 연결 전이거나 다른 재연결·속도 변경·자동 검색 중이면 보내지 않는다(not_sent). 허용 속도가 아니거나 구버전 펌웨어면
// setModbusBaudrate가 거절한다(not_sent).
void MainWindow::remoteSetDeviceBaudrate(int baudrate) {
    const int old_baud = modbus_widget_->ui_.comboBox_baudrate->currentText().toInt();
    auto report = [&] (bool ok, int now_baud, const char *result) {
        COUT("[Command 52] " << result << " (" << old_baud << " -> " << baudrate << " bps, now " << now_baud << " bps)");
        datc_interface_->reportDeviceBaudrate(ok, now_baud, result);
    };

    if (!datc_interface_->getConnectionState() || baud_change_busy_ || datc_interface_->isPollingPaused()) {
        report(false, old_baud, "not_sent");
        return;
    }
    if (baudrate == old_baud) {
        report(true, old_baud, "unchanged");
        return;
    }
    baud_change_busy_ = true;
    auto busy_reset = qScopeGuard([this] { baud_change_busy_ = false; });

    switch (changeDeviceBaudrate(baudrate, old_baud)) {
        case BaudChange::CHANGED:
            if (combo_dev_baud_ != nullptr) {
                combo_dev_baud_->setCurrentText(QString::number(baudrate));
            }
            report(true, baudrate, "changed");
            break;
        case BaudChange::REVERTED:
            report(false, old_baud, "reverted");
            break;
        case BaudChange::NO_RESPONSE:
            report(false, old_baud, "no_response");
            break;
        case BaudChange::NOT_SENT:
            report(false, old_baud, "not_sent");
            break;
    }
}

void MainWindow::on_pushButton_select_modbus_clicked() {
    ui_->stackedWidget->setCurrentIndex((int) WidgetSeq::MODBUS_WIDGET);

    ui_->pushButton_select_modbus    ->setStyleSheet(menu_btn_active_str_);
    ui_->pushButton_select_datc_ctrl ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_adv       ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_tcp       ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_imped_ctrl->setStyleSheet(menu_btn_inactive_str_);
}

void MainWindow::on_pushButton_select_datc_ctrl_clicked() {
    ui_->stackedWidget->setCurrentIndex((int) WidgetSeq::DATC_CTRL_WIDGET);

    ui_->pushButton_select_modbus    ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_datc_ctrl ->setStyleSheet(menu_btn_active_str_);
    ui_->pushButton_select_adv       ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_tcp       ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_imped_ctrl->setStyleSheet(menu_btn_inactive_str_);
}

void MainWindow::on_pushButton_select_adv_clicked() {
    ui_->stackedWidget->setCurrentIndex((int) WidgetSeq::ADVANCED_CTRL_WIDGET);

    ui_->pushButton_select_modbus    ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_datc_ctrl ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_adv       ->setStyleSheet(menu_btn_active_str_);
    ui_->pushButton_select_tcp       ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_imped_ctrl->setStyleSheet(menu_btn_inactive_str_);
}

void MainWindow::on_pushButton_select_tcp_clicked() {
    ui_->stackedWidget->setCurrentIndex((int) WidgetSeq::TCP_WIDGET);

    ui_->pushButton_select_modbus    ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_datc_ctrl ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_adv       ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_tcp       ->setStyleSheet(menu_btn_active_str_);
    ui_->pushButton_select_imped_ctrl->setStyleSheet(menu_btn_inactive_str_);
}

void MainWindow::on_pushButton_select_imped_ctrl_clicked() {
    ui_->stackedWidget->setCurrentIndex((int) WidgetSeq::IMPEDANCE_CTRL_WIDGET);

    ui_->pushButton_select_modbus    ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_datc_ctrl ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_adv       ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_tcp       ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_imped_ctrl->setStyleSheet(menu_btn_active_str_);
}

void MainWindow::on_pushButton_modbus_refresh_clicked() {
    modbus_widget_->ui_.comboBox_serial_port->clear();

    std::vector<std::string> ser_port_str_vec = getSerialPortLists();

    for (auto i : ser_port_str_vec) {
        modbus_widget_->ui_.comboBox_serial_port->addItem(QString::fromStdString(i));
    }

    if (!ser_port_str_vec.empty()) {
        modbus_widget_->ui_.comboBox_serial_port->setCurrentIndex(ser_port_str_vec.size() - 1);
    }
}

void MainWindow::on_pushButton_logo_clicked() {
    static uint cnt = 0;

    if (dev_tab_accessibility_) {
        if (++cnt >= 3) {
            ui_->stackedWidget->setCurrentIndex((int) WidgetSeq::DEV_TAB_WIDGET);

            ui_->pushButton_select_modbus    ->setStyleSheet(menu_btn_inactive_str_);
            ui_->pushButton_select_datc_ctrl ->setStyleSheet(menu_btn_inactive_str_);
            ui_->pushButton_select_adv       ->setStyleSheet(menu_btn_inactive_str_);
            ui_->pushButton_select_tcp       ->setStyleSheet(menu_btn_inactive_str_);
            ui_->pushButton_select_imped_ctrl->setStyleSheet(menu_btn_inactive_str_);
        }
    }
}

void MainWindow::on_pushButton_hidden_clicked() {
    static uint cnt = 0;
    dev_tab_accessibility_ = (++cnt >= 5) ? true : false;
}

std::vector<std::string> MainWindow::getSerialPortLists() {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(WIN64) || defined(_WIN64) || defined(__WIN64__)
    std::vector<std::string> comPorts;

    // Define the GUID for the ports
    GUID guid = { 0x4d36e978, 0xe325, 0x11ce, { 0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18 } };

    // Get a handle to a device information set for all devices matching the specified class
    HDEVINFO deviceInfoSet = SetupDiGetClassDevs(&guid, 0, 0, DIGCF_PRESENT);

    if (deviceInfoSet == INVALID_HANDLE_VALUE) {
        std::cerr << "Error: SetupDiGetClassDevs failed." << std::endl;
        return comPorts;
    }

    // Enumerate through all devices in the device information set
    SP_DEVINFO_DATA deviceInfoData;
    deviceInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
    for (DWORD i = 0; SetupDiEnumDeviceInfo(deviceInfoSet, i, &deviceInfoData); ++i) {
        // Get the registry key name for the device
        HKEY hkey = SetupDiOpenDevRegKey(deviceInfoSet, &deviceInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if (hkey == INVALID_HANDLE_VALUE) {
            continue;
        }

        // Get the length of the registry key name
        DWORD size = 0;
        RegQueryValueExA(hkey, "PortName", 0, 0, 0, &size);

        // Get the registry key name
        char* name = new char[size];
        RegQueryValueExA(hkey, "PortName", 0, 0, (LPBYTE)name, &size);
        comPorts.push_back(name);

        // Clean up
        delete[] name;
        RegCloseKey(hkey);
    }

    // Clean up
    SetupDiDestroyDeviceInfoList(deviceInfoSet);

    return comPorts;
#else
    std::vector<std::string> serialPorts;

    // Open the /dev directory
    DIR *dir = opendir("/dev");

    if (!dir) {
        std::cerr << "Error opening /dev directory." << std::endl;
        return serialPorts;
    }

    // Read the contents of the directory
    struct dirent *entry;

    while ((entry = readdir(dir)) != nullptr) {
        // Check if the entry is a character device file and has "tty" in its name
        if (entry->d_type == DT_CHR && std::strstr(entry->d_name, "ttyUSB") != nullptr) {
            std::string portName = "/dev/" + std::string(entry->d_name);
            serialPorts.push_back(portName);
        }
    }

    // Close the directory
    closedir(dir);

    return serialPorts;
#endif
}

} // end of namespace
