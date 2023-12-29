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

using namespace Qt;

namespace gripper_ui {

MainWindow::MainWindow(int argc, char **argv, bool &success, QWidget *parent) : QMainWindow(parent) {
    ui_ = new Ui::MainWindow();

    modbus_widget_        = new ModbusWidget(this);
    datc_ctrl_widget_     = new DatcCtrlWidget(this);
    tcp_widget_           = new TcpWidget(this);
    advanced_ctrl_widget_ = new AdvancedCtrlWidget(this);

    ui_->setupUi(this);

    //setWindowIcon(QIcon(":/images/icon.png"));
    datc_interface_ = new DatcCommInterface(argc, argv);

    // Stacked widget
    ui_->stackedWidget->addWidget(modbus_widget_);
    ui_->stackedWidget->addWidget(datc_ctrl_widget_);
    ui_->stackedWidget->addWidget(advanced_ctrl_widget_);

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
    int torque_init_value = 100;
    int speed_init_value  = 75;

    datc_ctrl_widget_->ui_.horizontalSlider_finger_pos->setValue(finger_pos_init_value);
    datc_ctrl_widget_->ui_.verticalSlider_torque->setValue      (torque_init_value);
    datc_ctrl_widget_->ui_.verticalSlider_speed ->setValue      (speed_init_value);

    datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->setValue((double) finger_pos_init_value);
    datc_ctrl_widget_->ui_.doubleSpinBox_torque->setValue    ((double) torque_init_value);
    datc_ctrl_widget_->ui_.doubleSpinBox_speed->setValue     ((double) speed_init_value);

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
    modbus_widget_->ui_.comboBox_baudrate->setCurrentIndex(2);
    modbus_widget_->ui_.comboBox_baudrate->setDisabled(true);

    // Check box setting
    QString checkbox_qstr = "QCheckBox::indicator {width:25px; height: 25px;}";

    tcp_widget_->ui_.checkBox_tcp_send_status->setStyleSheet(checkbox_qstr);

    checkbox_qstr = "QCheckBox::indicator {width:20px; height: 20px;}";

    advanced_ctrl_widget_->ui_.checkBox_motor_speed_reverse  ->setStyleSheet(checkbox_qstr);
    advanced_ctrl_widget_->ui_.checkBox_motor_current_reverse->setStyleSheet(checkbox_qstr);

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

#ifndef RCLCPP__RCLCPP_HPP_
    // TCP socket commiunication related btn
    QObject::connect(tcp_widget_->ui_.pushButton_tcp_start, SIGNAL(clicked()), this, SLOT(startTcpComm()));
    QObject::connect(tcp_widget_->ui_.pushButton_tcp_stop , SIGNAL(clicked()), this, SLOT(stopTcpComm()));
#else
    ui_->pushButton_select_tcp->setHidden(true);
#endif

    timer_ = new QTimer(this);
    connect(timer_, SIGNAL(timeout()), this, SLOT(timerCallback()));
    timer_->start(100); // msec

    datc_interface_->start();
    success = true;
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

    // Display
    ui_->lineEdit_monitor_finger_position->setText(QString::number((double) datc_status.finger_pos / 10 , 'f', 1) + " %");
    ui_->lineEdit_monitor_current        ->setText(QString::number(datc_status.motor_cur) + " mA");

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
            ui_->lineEdit_monitor_mode->setText("Failed to read input register.");
        } else {
            ui_->lineEdit_monitor_mode->setText(" " + QString::fromStdString(datc_status.status_str));
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
    static int slider_finger_pos_prev = datc_ctrl_widget_->ui_.horizontalSlider_finger_pos->value();
    static int slider_torque_prev     = datc_ctrl_widget_->ui_.verticalSlider_torque->value();
    static int slider_speed_prev      = datc_ctrl_widget_->ui_.verticalSlider_speed ->value();

    static double finger_pos_prev = datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->value();
    static double torque_prev     = datc_ctrl_widget_->ui_.doubleSpinBox_torque->value();
    static double speed_prev      = datc_ctrl_widget_->ui_.doubleSpinBox_speed->value();

    syncSliderSpinboxFn(slider_finger_pos_prev, finger_pos_prev,
                        datc_ctrl_widget_->ui_.horizontalSlider_finger_pos,
                        datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos);
    syncSliderSpinboxFn(slider_torque_prev, torque_prev,
                        datc_ctrl_widget_->ui_.verticalSlider_torque,
                        datc_ctrl_widget_->ui_.doubleSpinBox_torque);
    syncSliderSpinboxFn(slider_speed_prev, speed_prev,
                        datc_ctrl_widget_->ui_.verticalSlider_speed,
                        datc_ctrl_widget_->ui_.doubleSpinBox_speed);

    // Advanced control side
    const int vel_min_percent = (int) ((double) kVelMin / (double) kVelMax * 100);

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
}

// Enable Disable
void MainWindow::datcEnable() {
    datc_interface_->motorEnable();
}

void MainWindow::datcDisable() {
    datc_interface_->motorDisable();
}

// Datc control
void MainWindow::datcFingerPosCtrl() {
    datc_interface_->setFingerPos(datc_ctrl_widget_->ui_.doubleSpinBox_finger_pos->value() * 10);
}

void MainWindow::datcMotorVelCtrl() {
    int16_t vel = advanced_ctrl_widget_->ui_.doubleSpinBox_motor_speed->value() * kVelMax / 100;
    vel *= (advanced_ctrl_widget_->ui_.checkBox_motor_speed_reverse->isChecked()) ? -1 : 1;
    datc_interface_->motorVelCtrl(vel);
}

void MainWindow::datcMotorCurCtrl() {
    int16_t cur = advanced_ctrl_widget_->ui_.doubleSpinBox_motor_current->value() * kCurMax / 100;
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
    datc_interface_->setMotorTorque((uint16_t) datc_ctrl_widget_->ui_.doubleSpinBox_torque->value());
}

void MainWindow::datcSetSpeed() {
    datc_interface_->setMotorSpeed((uint16_t) datc_ctrl_widget_->ui_.doubleSpinBox_speed->value());
}

// Modbus RTU related
void MainWindow::initModbus() {
    COUT("--------------------------------------------");
    COUT("[INFO] Port: " + modbus_widget_->ui_.comboBox_serial_port->currentText().toStdString());
    COUT("[INFO] Slave address #" + modbus_widget_->ui_.spinBox_slave_addr->text().toStdString());
    COUT("--------------------------------------------");

    const char* port = modbus_widget_->ui_.comboBox_serial_port->currentText().toStdString().c_str();
    uint16_t slave_addr = modbus_widget_->ui_.spinBox_slave_addr->value();

    if (datc_interface_->init(port, slave_addr)) {
        // Successed
    } else {
        ui_->lineEdit_monitor_mode->setText("Invalid port or permission.");
        COUT("[ERROR] Port name or slave address invlaid !");
    }
}

void MainWindow::releaseModbus() {
    datc_interface_->modbusRelease();
    ui_->lineEdit_monitor_mode->setText("");
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

void MainWindow::on_pushButton_select_modbus_clicked() {
    ui_->stackedWidget->setCurrentIndex((int) WidgetSeq::MODBUS_WIDGET);

    ui_->pushButton_select_modbus   ->setStyleSheet(menu_btn_active_str_);
    ui_->pushButton_select_datc_ctrl->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_adv      ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_tcp      ->setStyleSheet(menu_btn_inactive_str_);
}

void MainWindow::on_pushButton_select_datc_ctrl_clicked() {
    ui_->stackedWidget->setCurrentIndex((int) WidgetSeq::DATC_CTRL_WIDGET);

    ui_->pushButton_select_modbus   ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_datc_ctrl->setStyleSheet(menu_btn_active_str_);
    ui_->pushButton_select_adv      ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_tcp      ->setStyleSheet(menu_btn_inactive_str_);
}

void MainWindow::on_pushButton_select_adv_clicked() {
    ui_->stackedWidget->setCurrentIndex((int) WidgetSeq::ADVANCED_CTRL_WIDGET);

    ui_->pushButton_select_modbus   ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_datc_ctrl->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_adv      ->setStyleSheet(menu_btn_active_str_);
    ui_->pushButton_select_tcp      ->setStyleSheet(menu_btn_inactive_str_);
}

void MainWindow::on_pushButton_select_tcp_clicked() {
    ui_->stackedWidget->setCurrentIndex((int) WidgetSeq::TCP_WIDGET);

    ui_->pushButton_select_modbus   ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_datc_ctrl->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_adv      ->setStyleSheet(menu_btn_inactive_str_);
    ui_->pushButton_select_tcp      ->setStyleSheet(menu_btn_active_str_);
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
