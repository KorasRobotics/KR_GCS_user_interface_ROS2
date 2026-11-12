/**
 * @file firmware_updater.cpp
 * @brief MATC USART3 bootloader uploader implementation.
 */
#include "firmware_updater.hpp"

#include <QFile>
#include <QFileInfo>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <vector>

namespace {

constexpr char     kMagic[4]     = {'M', 'B', 'L', '1'};
constexpr char     kSync[]       = "MATCBOOT";        // 8 bytes, no null
constexpr char     kSyncReply[]  = "MATCOK\n";        // 7 bytes
constexpr uint8_t  kAck[4]       = {'M', 'B', 0x06, '\n'};
constexpr uint8_t  kNack[4]      = {'M', 'B', 0x15, '\n'};

constexpr uint8_t kCmdBegin = 1;
constexpr uint8_t kCmdData  = 2;
constexpr uint8_t kCmdEnd   = 3;
constexpr uint8_t kCmdBoot  = 4;
constexpr uint8_t kCmdPing  = 5;

constexpr size_t kChunkSize = 1024;     // multiple of 8, matches Python tool default

// 장치 측 처리 시간에 맞춘 응답 대기 시간. BEGIN은 이미지 크기만큼 플래시 페이지를 지우고
// (2KB 페이지당 최대 ~40ms → 480KB면 ~10s), END는 이미지 전체 CRC를 두 번 계산한다(HSI 16MHz).
constexpr auto kBeginAckTimeout = std::chrono::milliseconds(15000);
constexpr auto kDataAckTimeout  = std::chrono::milliseconds(3000);
constexpr auto kEndAckTimeout   = std::chrono::milliseconds(15000);
constexpr auto kBootAckTimeout  = std::chrono::milliseconds(2000);

// 업데이트 이미지는 0x08008000(부트로더 32KB 뒤)부터의 앱 영역이어야 한다.
constexpr uint32_t kFlashBase    = 0x08000000u;
constexpr uint32_t kAppBase      = 0x08008000u;
constexpr uint32_t kFlashEnd     = 0x08080000u;          // 512KB
constexpr int      kAppOffset    = (int)(kAppBase - kFlashBase);   // 0x8000
constexpr uint32_t kRamBase      = 0x20000000u;
constexpr uint32_t kRamEnd       = 0x20020000u;          // 128KB

// Read up to `expected` bytes with a deadline; returns the bytes actually read.
// 동기 read_some은 Windows/Linux 모두 1바이트가 올 때까지 무기한 대기하므로(asio가 포트를
// "최소 1바이트 대기"로 설정) 타임아웃이 걸리지 않는다 — 장치가 "MATCOK\n" 7바이트만 보내고
// 침묵하면 32바이트를 기다리다 영원히 멈췄다. 비동기 읽기 + run_for 기한 + 취소로 구현.
std::vector<uint8_t> read_with_timeout(boost::asio::io_service &io,
                                       boost::asio::serial_port &port,
                                       size_t expected,
                                       std::chrono::milliseconds timeout) {
    using namespace std::chrono;
    std::vector<uint8_t> buf;
    buf.reserve(expected);

    const auto deadline = steady_clock::now() + timeout;
    uint8_t tmp[256];

    while (buf.size() < expected) {
        const auto now = steady_clock::now();
        if (now >= deadline) break;

        bool done = false;
        size_t got = 0;
        boost::system::error_code rec;
        const size_t want = std::min<size_t>(sizeof(tmp), expected - buf.size());
        port.async_read_some(boost::asio::buffer(tmp, want),
                             [&](const boost::system::error_code &e, size_t n) {
                                 rec = e;
                                 got = n;
                                 done = true;
                             });
        io.restart();
        io.run_for(duration_cast<milliseconds>(deadline - now));

        if (!done) {
            // 기한 내 수신 없음 → 대기 중인 읽기를 취소하고 완료 핸들러까지 소진한다.
            boost::system::error_code ignored;
            port.cancel(ignored);
            io.restart();
            io.run();
            if (done && !rec && got > 0) {
                buf.insert(buf.end(), tmp, tmp + got);   // 취소 직전에 도착한 바이트
            }
            break;
        }
        if (rec) {
            break;   // 포트 오류 — 호출부가 응답 부족으로 판단한다.
        }
        buf.insert(buf.end(), tmp, tmp + got);
    }
    return buf;
}

uint32_t read_le32(const QByteArray &b, int off) {
    return (uint32_t)(uint8_t)b[off] | ((uint32_t)(uint8_t)b[off + 1] << 8) |
           ((uint32_t)(uint8_t)b[off + 2] << 16) | ((uint32_t)(uint8_t)b[off + 3] << 24);
}

// off 위치의 벡터 테이블(초기 SP, Reset 핸들러)이 0x08008000 앱의 것인지 확인.
bool app_vector_ok(const QByteArray &b, int off) {
    if (b.size() < off + 8) return false;
    const uint32_t sp = read_le32(b, off);
    const uint32_t rv = read_le32(b, off + 4);
    return (sp > kRamBase) && (sp <= kRamEnd) && ((rv & 1u) != 0u) && (rv >= kAppBase) && (rv < kFlashEnd);
}

// [from, to) 구간이 모두 0x00 또는 모두 0xFF(objcopy가 채운 빈 공간)인지.
bool is_fill_region(const QByteArray &b, int from, int to) {
    if (to > b.size() || from >= to) return false;
    const char first = b[from];
    if (first != (char)0x00 && first != (char)0xFF) return false;
    for (int i = from; i < to; ++i) {
        if (b[i] != first) return false;
    }
    return true;
}

bool ack_matches(const std::vector<uint8_t> &response) {
    return response.size() >= 4 &&
           std::memcmp(response.data(), kAck, 4) == 0;
}

bool nack_matches(const std::vector<uint8_t> &response) {
    return response.size() >= 4 &&
           std::memcmp(response.data(), kNack, 4) == 0;
}

// Build a bootloader frame; if crc_override is set, use it instead of crc32(payload).
std::vector<uint8_t> make_frame(uint8_t cmd, uint32_t offset,
                                const uint8_t *payload, size_t length,
                                bool use_crc_override, uint32_t crc_override) {
    std::vector<uint8_t> frame;
    frame.reserve(16 + length);
    // magic
    frame.insert(frame.end(), kMagic, kMagic + 4);
    // cmd, reserved
    frame.push_back(cmd);
    frame.push_back(0);
    // length (u16 LE)
    frame.push_back((uint8_t)(length & 0xFF));
    frame.push_back((uint8_t)((length >> 8) & 0xFF));
    // offset (u32 LE)
    frame.push_back((uint8_t)(offset & 0xFF));
    frame.push_back((uint8_t)((offset >> 8) & 0xFF));
    frame.push_back((uint8_t)((offset >> 16) & 0xFF));
    frame.push_back((uint8_t)((offset >> 24) & 0xFF));
    // crc32 (u32 LE)
    uint32_t crc;
    if (use_crc_override) {
        crc = crc_override;
    } else {
        // Standard PKZIP / Ethernet CRC32, polynomial 0xEDB88320.
        crc = 0xFFFFFFFFu;
        for (size_t i = 0; i < length; ++i) {
            crc ^= payload[i];
            for (int b = 0; b < 8; ++b) {
                uint32_t mask = -(int32_t)(crc & 1);
                crc = (crc >> 1) ^ (0xEDB88320u & mask);
            }
        }
        crc ^= 0xFFFFFFFFu;
    }
    frame.push_back((uint8_t)(crc & 0xFF));
    frame.push_back((uint8_t)((crc >> 8) & 0xFF));
    frame.push_back((uint8_t)((crc >> 16) & 0xFF));
    frame.push_back((uint8_t)((crc >> 24) & 0xFF));
    // payload
    if (length > 0) frame.insert(frame.end(), payload, payload + length);
    return frame;
}

} // namespace

FirmwareUpdater::AppInfo FirmwareUpdater::readAppInfo(const QByteArray &image) {
    static const char kInfoMagic[8] = {'M', 'A', 'T', 'C', 'I', 'N', 'F', '1'};
    AppInfo info;
    const int pos = image.indexOf(QByteArray(kInfoMagic, 8));
    if (pos < 0 || image.size() < pos + 8 + 12) {
        return info;   // v3.01 이전 빌드 — 정보 블록 없음
    }
    auto u16 = [&image](int off) {
        return (uint16_t) ((uint8_t) image[off] | ((uint16_t) (uint8_t) image[off + 1] << 8));
    };
    const int base = pos + 8;
    if (u16(base) < 20) {   // info_size: magic(8) + 6 × uint16
        return info;
    }
    info.found             = true;
    info.model             = u16(base + 2);
    info.fw_version        = u16(base + 4);
    info.max_speed_rpm     = u16(base + 6);
    info.cur_full_scale_ma = u16(base + 8);
    info.pole_pairs        = u16(base + 10);
    return info;
}

FirmwareUpdater::AppInfo FirmwareUpdater::readAppInfoFromFile(const QString &bin_path) {
    QFile f(bin_path);
    if (!f.open(QIODevice::ReadOnly)) {
        return AppInfo();
    }
    return readAppInfo(f.readAll());
}

FirmwareUpdater::FirmwareUpdater(QObject *parent) : QObject(parent) {}

FirmwareUpdater::~FirmwareUpdater() {
    cancel_flag_ = true;
    if (worker_.joinable()) worker_.join();
}

void FirmwareUpdater::cancel() {
    cancel_flag_ = true;
}

void FirmwareUpdater::startUpdate(QString port_name, int baudrate,
                                  QString bin_path, bool jump_to_app, int sync_timeout_s) {
    if (running_.load()) {
        emit log("Update already in progress; ignoring new request.");
        return;
    }
    if (worker_.joinable()) worker_.join();

    cancel_flag_ = false;
    sync_timed_out_ = false;
    running_ = true;

    worker_ = std::thread([this, port_name, baudrate, bin_path, jump_to_app, sync_timeout_s]() {
        runUpload(port_name, baudrate, bin_path, jump_to_app, sync_timeout_s);
        running_ = false;
    });
}

uint32_t FirmwareUpdater::crc32(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            uint32_t mask = -(int32_t)(crc & 1);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

void FirmwareUpdater::runUpload(QString port_name, int baudrate,
                                QString bin_path, bool jump_to_app, int sync_timeout_s) {
    using boost::asio::serial_port;
    using boost::asio::serial_port_base;

    // ----- Load image -----
    QFile f(bin_path);
    if (!f.open(QIODevice::ReadOnly)) {
        emit finished(false, QString("Cannot open file: %1").arg(bin_path));
        return;
    }
    QByteArray image = f.readAll();
    f.close();
    if (image.isEmpty()) {
        emit finished(false, "Image is empty.");
        return;
    }

    // 2026-10-13 이후 빌드의 MATC_V3.bin은 부트 스텁이 빠져 0x08008000 앱 영역만 담으므로 그대로 보낸다.
    // 그 이전 빌드의 MATC_V3.bin은 0x08000000(디버그용 8바이트 부트 스텁)부터 시작하는 전체 이미지라
    // 앞 32KB가 스텁+빈 공간이다. 그대로 보내면 부트로더가 0x08008000부터 기록해 앱이 32KB 밀리고,
    // 벡터 검사는 통과해 "완료"로 끝난 뒤 앱이 기동하지 않는다 → 앱 영역(0x08008000~)만 잘라 보낸다.
    if (image.size() > kAppOffset && app_vector_ok(image, kAppOffset) &&
        is_fill_region(image, 8, kAppOffset)) {
        image = image.mid(kAppOffset);
        emit log(QString("0x08000000부터의 전체 이미지(부트 스텁 포함) → 0x08008000 앱 영역만 전송 (앞 %1 bytes 제외)")
                    .arg(kAppOffset));
    }
    if (!app_vector_ok(image, 0)) {
        emit finished(false,
            "선택한 파일이 0x08008000용 앱 이미지가 아닙니다(벡터 테이블 불일치). "
            "MATC_V3 빌드의 MATC_V3.bin을 선택하세요. 장치에는 아무것도 쓰지 않았습니다.");
        return;
    }

    uint32_t image_crc = crc32(reinterpret_cast<const uint8_t *>(image.constData()),
                               (size_t) image.size());
    emit log(QString("Image: %1 (%2 bytes, CRC32=0x%3)")
                .arg(QFileInfo(bin_path).fileName())
                .arg(image.size())
                .arg(image_crc, 8, 16, QChar('0')).toUpper());

    // ----- Open serial port -----
    boost::asio::io_service io;
    serial_port port(io);
    boost::system::error_code ec;

    // boost::asio expects "\\\\.\\COMn" form on Windows for ports > COM9; pass either form.
    std::string port_std = port_name.toStdString();
    port.open(port_std, ec);
    if (ec) {
        emit finished(false, QString("Cannot open serial port %1: %2")
                                .arg(port_name).arg(QString::fromStdString(ec.message())));
        return;
    }

    port.set_option(serial_port_base::baud_rate(baudrate));
    port.set_option(serial_port_base::character_size(8));
    port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    port.set_option(serial_port_base::parity(serial_port_base::parity::none));
    port.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));

    emit log(QString("Opened %1 @ %2 8N1").arg(port_name).arg(baudrate));

    // ----- Sync with bootloader -----
    if (sync_timeout_s < 1) sync_timeout_s = 10;
    emit log(QString("Waiting for bootloader sync (%1s)...").arg(sync_timeout_s));
    emit progress(0);

    bool synced = false;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(sync_timeout_s);
    std::vector<uint8_t> sync_buf;
    sync_buf.reserve(256);

    while (std::chrono::steady_clock::now() < deadline) {
        if (cancel_flag_.load()) {
            port.close();
            emit finished(false, "Cancelled before sync.");
            return;
        }
        boost::asio::write(port, boost::asio::buffer(kSync, 8), ec);
        if (ec) {
            port.close();
            emit finished(false, QString("Write failed during sync: %1")
                                    .arg(QString::fromStdString(ec.message())));
            return;
        }
        auto rx = read_with_timeout(io, port, 32, std::chrono::milliseconds(150));
        sync_buf.insert(sync_buf.end(), rx.begin(), rx.end());
        if (sync_buf.size() >= 7) {
            // search for "MATCOK\n"
            for (size_t i = 0; i + 7 <= sync_buf.size(); ++i) {
                if (std::memcmp(&sync_buf[i], kSyncReply, 7) == 0) {
                    synced = true;
                    break;
                }
            }
        }
        if (synced) break;
        if (sync_buf.size() > 128) {
            // keep tail to avoid unbounded growth
            sync_buf.erase(sync_buf.begin(), sync_buf.end() - 7);
        }
    }

    if (!synced) {
        // 이전 업데이트가 중단돼 부트로더가 이미 프레임 수신 단계(UpdateLoop)에 있으면 "MATCBOOT"에 응답하지 않고
        // Modbus도 안 된다(전원을 껐다 켜야 풀림). 부트로더의 16바이트 프레임 수신이 시간 초과(5s)로 정렬을
        // 되찾을 때까지 잠시 송신을 멈춘 뒤 PING 프레임으로 확인 — ACK면 그 상태에서 이어서 업데이트한다.
        emit log("MATCBOOT 무응답 → 이미 업데이트 모드인 부트로더인지 PING으로 확인합니다(약 6초)...");
        for (int i = 0; i < 53; ++i) {   // 5.3s, 취소 확인
            if (cancel_flag_.load()) {
                port.close();
                emit finished(false, "Cancelled before sync.");
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        read_with_timeout(io, port, 256, std::chrono::milliseconds(50));   // 남은 수신 바이트 비우기
        auto ping = make_frame(kCmdPing, 0, nullptr, 0, /*use_crc_override*/ true, 0);
        boost::asio::write(port, boost::asio::buffer(ping.data(), ping.size()), ec);
        if (!ec && ack_matches(read_with_timeout(io, port, 4, std::chrono::milliseconds(1500)))) {
            synced = true;
            emit log("부트로더가 이미 업데이트 모드입니다(이전 업데이트 중단) → 이어서 진행합니다.");
        }
    }

    if (!synced) {
        port.close();
        sync_timed_out_ = true;
        emit finished(false,
            "부트로더 동기화 시간 초과(응답 없음). 확인하세요: "
            "① 보드에 MATC 부트로더가 있는지 — ST-Link/CubeIDE로 앱(ELF 또는 MATC_V3.bin)을 0x08000000부터 "
            "구우면 디버그용 부트 스텁이 부트로더를 덮어써 GCS 업데이트가 불가능합니다. 이 경우 "
            "MATC_Bootloader.hex를 0x08000000에 한 번 구워야 합니다. "
            "② 자동 리셋을 껐거나 Modbus가 연결되지 않았다면, 동기화가 진행되는 동안 보드 전원을 껐다 켜세요.");
        return;
    }
    emit log("Bootloader synced.");

    // Drain any leftover bytes.
    read_with_timeout(io, port, 64, std::chrono::milliseconds(50));

    // ----- BEGIN -----
    {
        auto frame = make_frame(kCmdBegin, (uint32_t) image.size(),
                                nullptr, 0, /*use_crc_override*/ true, image_crc);
        boost::asio::write(port, boost::asio::buffer(frame.data(), frame.size()), ec);
        if (ec) {
            port.close();
            emit finished(false, QString("BEGIN write failed: %1")
                                    .arg(QString::fromStdString(ec.message())));
            return;
        }
        auto resp = read_with_timeout(io, port, 4, kBeginAckTimeout);
        if (!ack_matches(resp)) {
            port.close();
            emit finished(false, nack_matches(resp)
                ? "BEGIN NACK from bootloader (image size/CRC mismatch?)"
                : "BEGIN no ACK (timeout).");
            return;
        }
        emit log("BEGIN ACK.");
    }

    // ----- DATA chunks -----
    const int total = image.size();
    int offset = 0;
    while (offset < total) {
        if (cancel_flag_.load()) {
            port.close();
            emit finished(false, "Cancelled during data transfer.");
            return;
        }
        int len = std::min<int>(kChunkSize, total - offset);
        auto frame = make_frame(kCmdData, (uint32_t) offset,
                                reinterpret_cast<const uint8_t *>(image.constData() + offset),
                                (size_t) len,
                                /*use_crc_override*/ false, 0);
        boost::asio::write(port, boost::asio::buffer(frame.data(), frame.size()), ec);
        if (ec) {
            port.close();
            emit finished(false, QString("DATA write failed @0x%1: %2")
                                    .arg(offset, 8, 16, QChar('0'))
                                    .arg(QString::fromStdString(ec.message())));
            return;
        }
        auto resp = read_with_timeout(io, port, 4, kDataAckTimeout);
        if (!ack_matches(resp)) {
            port.close();
            emit finished(false,
                QString("DATA %1 @0x%2")
                    .arg(nack_matches(resp) ? "NACK" : "timeout")
                    .arg(offset, 8, 16, QChar('0')).toUpper());
            return;
        }
        offset += len;
        int pct = (int) ((int64_t) offset * 100 / total);
        emit progress(pct);
    }
    emit log("All DATA chunks ACKed.");

    // ----- END -----
    {
        auto frame = make_frame(kCmdEnd, 0, nullptr, 0, /*use_crc_override*/ true, 0);
        boost::asio::write(port, boost::asio::buffer(frame.data(), frame.size()), ec);
        if (ec) {
            port.close();
            emit finished(false, QString("END write failed: %1")
                                    .arg(QString::fromStdString(ec.message())));
            return;
        }
        auto resp = read_with_timeout(io, port, 4, kEndAckTimeout);
        if (!ack_matches(resp)) {
            port.close();
            emit finished(false, nack_matches(resp)
                ? "END NACK (image CRC mismatch on device)"
                : "END no ACK (timeout).");
            return;
        }
        emit log("END ACK (image verified on device).");
    }

    // ----- BOOT -----
    if (jump_to_app) {
        auto frame = make_frame(kCmdBoot, 0, nullptr, 0, /*use_crc_override*/ true, 0);
        boost::asio::write(port, boost::asio::buffer(frame.data(), frame.size()), ec);
        if (ec) {
            port.close();
            emit finished(false, QString("BOOT write failed: %1")
                                    .arg(QString::fromStdString(ec.message())));
            return;
        }
        auto resp = read_with_timeout(io, port, 4, kBootAckTimeout);
        if (!ack_matches(resp)) {
            // Some bootloaders jump before ACK lands; that's not a hard failure.
            emit log("BOOT ACK not received; device likely jumped to application.");
        } else {
            emit log("BOOT ACK; application started.");
        }
    }

    port.close();
    emit progress(100);
    emit finished(true,
        QString("Firmware update complete (%1 bytes, CRC32=0x%2).")
            .arg(image.size())
            .arg(image_crc, 8, 16, QChar('0')).toUpper());
}
