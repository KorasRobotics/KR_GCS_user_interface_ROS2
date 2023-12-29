/**
 * @file custom_widget.hpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief
 * @version 1.0
 * @date 2023-11-14
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef CUSTOM_WIDGET_HPP
#define CUSTOM_WIDGET_HPP

#include "ui_modbus_form.h"
#include "ui_datc_control_form.h"
#include "ui_tcp_form.h"
#include "ui_advanced_control.h"

class ModbusWidget : public QWidget {
    Q_OBJECT

public:
    ModbusWidget(QWidget *parent = nullptr) : QWidget(parent) {
        ui_.setupUi(this);
    }

    Ui::ModbusForm ui_;
};

class DatcCtrlWidget : public QWidget {
    Q_OBJECT

public:
    DatcCtrlWidget(QWidget *parent = nullptr) : QWidget(parent) {
        ui_.setupUi(this);
    }

    Ui::DatcCtrlForm ui_;
};

class TcpWidget : public QWidget {
    Q_OBJECT

public:
    TcpWidget(QWidget *parent = nullptr) : QWidget(parent) {
        ui_.setupUi(this);
    }

    Ui::TcpForm ui_;
};

class AdvancedCtrlWidget : public QWidget {
    Q_OBJECT

public:
    AdvancedCtrlWidget(QWidget *parent = nullptr) : QWidget(parent) {
        ui_.setupUi(this);
    }

    Ui::AdvancedCtrlForm ui_;
};

#endif // CUSTOM_WIDGET_HPP
