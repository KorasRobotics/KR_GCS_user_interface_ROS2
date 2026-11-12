/**
 * @file firmware_updater.hpp
 * @brief MATC USART3 bootloader native firmware updater.
 *
 * Implements the bootloader protocol described in
 * `STM/matc_firmware_v3/MATC_Bootloader/README.md`.
 *   - Sync:  host sends "MATCBOOT" until bootloader replies "MATCOK\n"
 *   - Frame: "MBL1" + cmd(u8) + reserved(u8) + length(u16 LE)
 *            + offset(u32 LE) + crc32(u32 LE) + payload[length]
 *   - ACK :  "MB\x06\n", NACK: "MB\x15\n"
 *
 * Runs the upload sequence on a worker thread and emits progress/log/finished
 * signals so the Qt main thread can update the UI without blocking.
 */
#ifndef FIRMWARE_UPDATER_HPP
#define FIRMWARE_UPDATER_HPP

#include <QObject>
#include <QString>
#include <QByteArray>

#include <atomic>
#include <thread>

class FirmwareUpdater : public QObject {
    Q_OBJECT
public:
    // 앱 이미지 정보 블록 (MATC v3.01~ 펌웨어, kr_matc_modbus.c g_matcAppInfo).
    // 매직 "MATCINF1" 뒤에 리틀엔디안 uint16: info_size, model, fw_version, max_speed_rpm,
    // cur_full_scale_ma, pole_pairs. 펌웨어 쪽 구조를 바꾸면 readAppInfo도 함께 바꿀 것.
    struct AppInfo {
        bool     found             = false;
        uint16_t model             = 0;     // 10 = MATC-10, 20 = MATC-20(GL60), 30 = MATC-30(GL80)
        uint16_t fw_version        = 0;     // 301 = v3.01
        uint16_t max_speed_rpm     = 0;
        uint16_t cur_full_scale_ma = 0;
        uint16_t pole_pairs        = 0;
    };
    static AppInfo readAppInfo(const QByteArray &image);
    static AppInfo readAppInfoFromFile(const QString &bin_path);

    explicit FirmwareUpdater(QObject *parent = nullptr);
    ~FirmwareUpdater() override;

    // True while a worker thread is running an upload.
    bool isRunning() const { return running_.load(); }
    // 마지막 실행이 부트로더 동기화 시간 초과로 끝났는지(진단용).
    bool syncTimedOut() const { return sync_timed_out_.load(); }

public slots:
    // Start a firmware update. Returns immediately; progress is delivered via signals.
    // port_name     e.g. "COM3" on Windows, "/dev/ttyUSB0" on Linux
    // baudrate      bootloader baudrate (always 115200 for MATC v3)
    // bin_path      raw application .bin linked at 0x08008000
    // jump_to_app   send BOOT after END so the freshly written image runs
    // sync_timeout_s 부트로더 동기화 대기(초). SYSTEM_RESET을 보냈으면 10, 전원 재투입을 기다리면 더 길게.
    void startUpdate(QString port_name, int baudrate, QString bin_path, bool jump_to_app,
                     int sync_timeout_s = 10);

    // Cooperative cancel; the worker checks the flag between frames.
    void cancel();

signals:
    void log(QString message);
    void progress(int percent);
    void finished(bool success, QString message);

private:
    void runUpload(QString port_name, int baudrate, QString bin_path, bool jump_to_app, int sync_timeout_s);
    static uint32_t crc32(const uint8_t *data, size_t len);

    std::atomic<bool> running_{false};
    std::atomic<bool> cancel_flag_{false};
    std::atomic<bool> sync_timed_out_{false};
    std::thread worker_;
};

#endif // FIRMWARE_UPDATER_HPP
