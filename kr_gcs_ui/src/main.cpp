/**
 * @file main.cpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief Qt based gui.
 * @version 1.0
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "main_window.hpp"
#include <QApplication>
#include <QFontDatabase>
#include <QStyleFactory>
#include <QPalette>

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // ---------------------------------------------------------------------
    // Force a fixed Light theme regardless of the OS (Windows dark mode 등)
    // .ui 파일의 stylesheet들이 흰색/회색 배경을 가정해서 작성되어 있으므로,
    // 시스템 다크 모드 팔레트가 적용되면 글자/입력 위젯이 보이지 않게 된다.
    // 'Fusion' 스타일 + 고정 light palette 로 일관된 외관을 강제한다.
    // ---------------------------------------------------------------------
    QApplication::setStyle(QStyleFactory::create("Fusion"));

    QPalette light_palette;
    light_palette.setColor(QPalette::Window,          QColor(0xFF, 0xFF, 0xFF));
    light_palette.setColor(QPalette::WindowText,      QColor(0x00, 0x00, 0x00));
    light_palette.setColor(QPalette::Base,            QColor(0xFF, 0xFF, 0xFF));
    light_palette.setColor(QPalette::AlternateBase,   QColor(0xF0, 0xF0, 0xF0));
    light_palette.setColor(QPalette::ToolTipBase,     QColor(0xFF, 0xFF, 0xDC));
    light_palette.setColor(QPalette::ToolTipText,     QColor(0x00, 0x00, 0x00));
    light_palette.setColor(QPalette::Text,            QColor(0x00, 0x00, 0x00));
    light_palette.setColor(QPalette::Button,          QColor(0xE0, 0xE0, 0xE0));
    light_palette.setColor(QPalette::ButtonText,      QColor(0x00, 0x00, 0x00));
    light_palette.setColor(QPalette::BrightText,      QColor(0xFF, 0x00, 0x00));
    light_palette.setColor(QPalette::Highlight,       QColor(0x2A, 0x82, 0xDA));
    light_palette.setColor(QPalette::HighlightedText, QColor(0xFF, 0xFF, 0xFF));
    light_palette.setColor(QPalette::PlaceholderText, QColor(0x80, 0x80, 0x80));
    // Disabled state
    light_palette.setColor(QPalette::Disabled, QPalette::WindowText, QColor(0x80, 0x80, 0x80));
    light_palette.setColor(QPalette::Disabled, QPalette::Text,       QColor(0x80, 0x80, 0x80));
    light_palette.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(0x80, 0x80, 0x80));
    QApplication::setPalette(light_palette);

    // Font load
    QString font_path = ":/font/NotoSansKR-VariableFont_wght.ttf";

    QFontDatabase font_database;

    if (font_database.addApplicationFont(font_path) == -1) {
        COUT("Font not found: \"" + font_path.toStdString() +  "\"");
    }

    bool flag_init_success = false;

    gripper_ui::MainWindow w(argc, argv, flag_init_success);

    if (flag_init_success) {
        w.show();

        app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
        int result = app.exec();

        return result;
    } else {
        return -1;
    }
}
