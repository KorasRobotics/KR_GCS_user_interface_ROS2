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

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

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
