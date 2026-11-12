/**
 * @file datc_ctrl.hpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief
 * @version 1.0
 * @date 2023-11-03
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef DATC_CTRL_HPP
#define DATC_CTRL_HPP

#include "modbus_comm.hpp"
#include <cstdio>
#include <map>
#include <string>

#define CMD_ADDR 0

// Status registers (10–17): motor/gripper state feedback
const uint16_t kDatcStatusRegAddr  = 10;
const uint16_t kDatcStatusRegNum   = 8;

// Gripper info registers (20–23): ID and physical specs (MATC v3 only)
const uint16_t kGripperInfoRegAddr = 20;
const uint16_t kGripperInfoRegNum  = 4;

// Combined read range — MATC v3 펌웨어 호환
//   regs 10..23 = status(10-17) + reserved(18-19) + gripper info(20-23)
// MATC v3 펌웨어의 REG_HOLDING_NREGS=28 이고 reg 20-27은 그리퍼 ID/맵 응답용.
// (MATC v2 이하 펌웨어를 사용한다면 8로 변경)
const uint16_t kDatcFullReadNum    = 14;  // regs 10..23 (MATC v3)

// Query response registers (24–27): 조회 명령(217 맵 조회 / 221 엔코더 보정 계수 조회)의 응답 영역 (MATC v3)
//   reg 24에 요청한 번호(맵 index 또는 차수 k)가 되돌아오므로 이것으로 응답 갱신을 확인한다.
const uint16_t kQueryRespRegAddr   = 24;
const uint16_t kQueryRespRegNum    = 4;

// Device info registers (28–33): 모델·펌웨어 버전·제어 한계·장치 플래그 (MATC v3.01~)
//   28 모델(10/20/30)  29 FW 버전(301 = v3.01)  30 속도 제어 상한[rpm]  31 전류 제어 100%[mA]
//   32 플래그(bit0 전기각 캘리브됨, bit1 다른 모델의 전기각이라 거부됨, bit2 엔코더 보정 동작)  33 예약
// 이전 펌웨어(REG_HOLDING_NREGS=28)는 이 영역을 읽으면 예외 응답 → 미지원으로 보고 기본 한계를 쓴다.
const uint16_t kDeviceInfoRegAddr  = 28;
const uint16_t kDeviceInfoRegNum   = 6;

// Encoder harmonic compensation (219~221, MATC v3) — 펌웨어 KR_FLASH_ENC_HARM_COUNT / _LIMIT
const uint16_t kEncHarmCount       = 4;     // 차수 k = 1..4
const int32_t  kEncHarmLimit       = 2048;  // 계수 진폭 √(A²+B²) 상한 [s16 기계각 카운트] = 11.25도

// Encoder harmonic auto calibration (222/223, MATC v3.02~) — 진행 상태는 reg 33
const uint16_t kAutoCalRegAddr     = 33;
const uint16_t kAutoCalMinFw       = 302;   // 이 버전부터 지원

// 일체형 그리퍼 고정 ID (224, MATC v3.04~) — 포고핀이 없어 ID 핀이 '없음'일 때 펌웨어가 쓰는 저장 ID
const uint16_t kFixedGripperIdRegAddr = 34;    // 저장된 고정 ID (128 = 없음)
const uint16_t kFixedGripperIdMinFw   = 304;   // 이 버전부터 지원
const uint16_t kFixedGripperIdMax     = 126;   // 127은 ID 핀 '없음' 패턴이라 쓰지 않는다
const uint16_t kFixedGripperIdNone    = 128;

// reg 33 상태 워드 = 상태(15..12) | 측정 회차(11..8) | 진행률(진행 중) 또는 오류 코드(실패)(7..0)
enum class AutoCalState : uint8_t { NONE = 0, RUNNING = 1, DONE = 2, FAILED = 3, CANCELLED = 4 };
inline AutoCalState autoCalState(uint16_t st) { return (AutoCalState) (st >> 12); }
inline uint8_t      autoCalMeas(uint16_t st)  { return (uint8_t) ((st >> 8) & 0x0F); }
inline uint8_t      autoCalLow(uint16_t st)   { return (uint8_t) (st & 0xFF); }
string autoCalErrorText(uint8_t code);      // 실패 코드 설명 (펌웨어 kr_matc_enccal.h ENCCAL_ERR_*)

const uint16_t kModbusSlaveAddrMin = 1;
const uint16_t kModbusSlaveAddrMax = 247;

#define SEND_CMD_VECTOR(...) mbc_.sendData(CMD_ADDR, __VA_ARGS__)

// SEND_CMD: 단일 명령(TV 미사용)도 reg 0~3를 FC16(write multiple)로 한꺼번에 쓴다.
//   - FC06 단일 쓰기로 reg 0만 변경하면 reg 1~3에 이전 명령의 stale 값이 남아
//     펌웨어 enqueue 시점에 잘못된 TV가 함께 큐잉될 수 있다.
//   - reg 0~3를 [cmd, 0, 0, 0]으로 명시적으로 덮어써서 race condition 제거.
//   - 결과: MOTOR_ENABLE / CHANGE_ELECTRICAL_ANGLE 등 단일 명령도 위치/속도
//     제어와 동일하게 FC16으로 통일되어, 펌웨어 처리 일관성이 보장된다.
#define SEND_CMD(cmd) mbc_.sendData(CMD_ADDR, std::vector<uint16_t>({(uint16_t)(cmd), 0, 0, 0}))

using namespace std;

const uint16_t kDurationMin    = 10;
const uint16_t kDurationMax    = 10000;
const uint16_t kFingerPosMin   = 0;
const uint16_t kFingerPosMax   = 1000;
const uint16_t kGripForceMin   = 1;     // 그립 파지력 [N] 하한 (펌웨어 MB_SET_TORQUE >=1)
const uint16_t kGripForceMax   = 1000;  // 그립 파지력 [N] 상한 (펌웨어 MB_FORCE_MAX_N)
const uint16_t kSpeedRatioMin  = 1;     // 펌웨어 MB_SET_VELOCITY는 1~100 %만 수용 (0은 무시됨)
const uint16_t kSpeedRatioMax  = 100;

// 구버전 펌웨어(MATC V1/V2, 일체형 그리퍼 — 레지스터 0~19)의 명령 단위. DatcStatus::legacy_map 참고.
//   손가락 위치(104) = 0~1000 ‰(1000 = 완전 열림), 파지력(212) = 정격 토크의 %
//   펌웨어는 파지력 25~100 %를 받지만, 이전 GCS와 Gripper Control 탭 눈금(50~100 %)에 맞춰 50~100 %만 쓴다.
const uint16_t kLegacyTorquePctMin = 50;
const uint16_t kLegacyTorquePctMax = 100;

// 속도·전류 제어 한계. 장치가 한계를 보고하면(reg 30·31, v3.01~) 그 값을 쓰고, 아니면 아래 기본값(MATC-10 기준).
//   속도 상한 = min(kVelMax, 장치 상한) — MATC-10의 기존 900 rpm 여유는 유지, GL60 310 / GL80 450 rpm은 장치값.
//   전류 100% = 장치 정격(MATC-10 1297 / GL60 1350 / GL80 2800 mA). 펌웨어도 정격으로 다시 클램프한다.
const uint16_t kVelMin =  100;
const uint16_t kVelMax =  900;
const uint16_t kCurMax = 1200;

// 제어기 모델 코드(reg 28, 앱 이미지 정보 블록) → 표시 이름
inline std::string matcModelName(uint16_t model) {
    switch (model) {
        case 10: return "MATC-10";
        case 20: return "MATC-20 (GL60)";
        case 30: return "MATC-30 (GL80)";
        default: return "Unknown model " + std::to_string(model);
    }
}

// 펌웨어 버전 코드(301) → "v3.01"
inline std::string matcFwVersionStr(uint16_t version) {
    char buf[16];
    snprintf(buf, sizeof(buf), "v%u.%02u", (unsigned) (version / 100), (unsigned) (version % 100));
    return buf;
}

// 장치 정보 (regs 28–33, MATC v3.01~)
struct DeviceInfo {
    bool     valid             = false;   // regs 28–33을 읽었음 (v3.01 이상)
    bool     unsupported       = false;   // 연결된 펌웨어가 장치 정보를 제공하지 않음 (v3.01 이전)
    uint16_t model             = 0;
    uint16_t fw_version        = 0;
    uint16_t max_speed_rpm     = 0;
    uint16_t cur_full_scale_ma = 0;
    uint16_t flags             = 0;
    uint16_t auto_cal          = 0;       // reg 33: 엔코더 자동 보정 상태 (v3.02~, 이전 버전은 0)
    uint16_t fixed_gripper_id  = kFixedGripperIdNone;   // reg 34: 저장된 일체형 고정 ID (v3.04~, 128 = 없음)

    bool elangleCalibrated()    const { return (flags & 0x1) != 0; }
    bool elangleModelMismatch() const { return (flags & 0x2) != 0; }
    bool encHarmActive()        const { return (flags & 0x4) != 0; }
    bool fixedIdInUse()         const { return (flags & 0x8) != 0; }   // ID 핀 '없음' → 고정 ID 적용 중 (v3.04~)
    bool hasFixedId()           const { return fixed_gripper_id <= kFixedGripperIdMax; }
};

enum class DATC_COMMAND {
    // Motor basic
    MOTOR_ENABLE              = 1,
    MOTOR_STOP                = 2,   // MB_STOP_P: mode-aware stop
    MOTOR_EMERGENCY_STOP      = 3,   // MB_STOP_V: immediate force stop (MATC v3)
    MOTOR_DISABLE             = 4,
    MOTOR_POSITION_CONTROL    = 5,
    MOTOR_VELOCITY_CONTROL    = 6,
    MOTOR_CURRENT_CONTROL     = 7,
    RESET_POSITION            = 8,
    ERROR_CLEAR               = 9,
    SYSTEM_RESET              = 10,
    // System config
    CHANGE_MODBUS_ADDRESS     = 50,
    CHANGE_ELECTRICAL_ANGLE   = 51,
    CHANGE_MODBUS_BAUDRATE    = 52,   // value_1 = bps / 100. 펌웨어가 EEPROM 저장 후 즉시 전환
    // Gripper
    GRIPPER_SET_DIRECTION     = 100,
    GRIPPER_INITIALIZE        = 101,
    GRIPPER_OPEN              = 102,
    GRIPPER_CLOSE             = 103,
    SET_FINGER_POSITION       = 104,
    GRIPPER_INITIALIZE2       = 105,
    VACUUM_GRIPPER_ON         = 106,
    VACUUM_GRIPPER_OFF        = 107,
    // Impedance
    IMPEDANCE_ON              = 108,
    IMPEDANCE_OFF             = 109,
    SET_IMPEDANCE_PARAMS      = 110,
    SET_IMPEDANCE_MBK         = 111,
    // Parameters
    SET_MAX_VALUE             = 201,
    SET_POSITION_PID_GAIN     = 211,
    SET_MOTOR_TORQUE          = 212,
    SET_MOTOR_SPEED           = 213,
    SET_CURRENT_PI_GAIN       = 214,
    SET_VELOCITY_PI_GAIN      = 215,
    // Gripper mapping table (MATC v3)
    WRITE_MAP_ENTRY           = 216,
    READ_MAP_ENTRY            = 217,
    SAVE_MAP                  = 218,
    // Encoder harmonic compensation (MATC v3) — AS5048 자석 편심/기울기 각도오차 보정
    SET_ENC_HARM              = 219,   // value_1 = 차수 k(1~4, 0 = 전체 해제), value_2 = A, value_3 = B (int16)
    SAVE_ENC_HARM             = 220,   // 현재 계수를 EEPROM에 저장
    READ_ENC_HARM             = 221,   // value_1 = k(1~4) → 응답 reg 24~27 = (k, A, B, 상태)
    // Encoder harmonic auto calibration (MATC v3.02~) — 툴을 뗀 상태에서 모터가 스스로 왕복하며 측정·저장
    AUTO_ENC_HARM             = 222,   // value_1 = 1 시작, 0 취소. 진행 상태는 reg 33
    READ_AUTO_ENC_HARM        = 223,   // value_1 = k(1~4) → 응답 reg 24~27 = (k, 보정 전, 보정 후 [0.001도], 상태)
    // Integrated gripper fixed ID (MATC v3.04~) — 포고핀 없는 일체형 그리퍼의 그리퍼 ID를 EEPROM에 저장
    SET_FIXED_GRIPPER_ID      = 224,   // value_1 = 0~126 저장, 128 = 해제. 저장된 값은 reg 34
};

// 구버전 펌웨어(DatcStatus::legacy_map)에 없는 V3 전용 명령 — 통신속도 변경(52), 그리퍼 맵(216~218),
// 엔코더 고조파 보정·자동 보정(219~223), 고정 ID(224). 구버전에 연결돼 있으면 보내지 않는다.
// (214·215 PI 게인은 matc_firmware_v2에는 있고 KUGRP 계열에는 없어 막지 않는다.)
inline bool isV3OnlyCommand(uint16_t cmd) {
    return (cmd == (uint16_t) DATC_COMMAND::CHANGE_MODBUS_BAUDRATE) ||
           ((cmd >= (uint16_t) DATC_COMMAND::WRITE_MAP_ENTRY) && (cmd <= (uint16_t) DATC_COMMAND::SET_FIXED_GRIPPER_ID));
}

// 자동 보정 결과 조회(223) 응답 — reg 24~27
struct AutoEncHarmResult {
    uint16_t k           = 0;
    uint16_t before_mdeg = 0;   // 보정 전 k차 위치오차 진폭 [0.001도] (방향 평균)
    uint16_t after_mdeg  = 0;   // 확정 계수에서의 k차 위치오차 진폭 [0.001도]
    uint16_t status      = 0;   // reg 33과 같은 상태 워드
};

// 맵 조회(217) 응답 — reg 24~27
struct GripperMapEntry {
    uint16_t index       = 0;
    uint16_t stroke_mm   = 0;
    uint16_t force_n     = 0;
    uint16_t payload_x10 = 0;
};

// 엔코더 보정 계수 조회(221) 응답 — reg 24~27. A, B는 펌웨어 측정 기계각 기준, s16 카운트(65536 = 360도)
struct EncHarmEntry {
    uint16_t k              = 0;
    int16_t  a              = 0;
    int16_t  b              = 0;
    bool     active         = false;   // 상태 bit0: 보정 동작 중
    bool     last_save_ok   = false;   // 상태 bit1: 마지막 저장(220) 성공
    bool     last_save_fail = false;   // 상태 bit2: 마지막 저장(220) 실패
};

struct DatcStatus {
    string status_str;
    string warn_str;    // 경고(전기각 상태, 래치 경고 bit 14·15). " | "로 연결, 없으면 빈 문자열

    // Status bits (register 10)
    bool enable            = false;
    bool initialize        = false;
    bool motor_pos_ctrl    = false;
    bool motor_vel_ctrl    = false;
    bool motor_cur_ctrl    = false;
    bool grp_open          = false;
    bool grp_close         = false;
    bool gripper_direction = false;
    bool object_grasp      = false;
    bool fault             = false;
    bool grip_hold         = false;
    bool slip_detected     = false;

    // gObj 파지 결과 (register 10, bit 12-13) — MATC v3, Robotiq gOBJ식 (latched)
    // 0=이동중, 1=물체감지(닫는중), 2=물체감지(여는중), 3=목표위치 도달(물체없음)
    uint8_t gobj_result    = 0;

    // 래치 경고 (register 10, bit 14-15) — MATC v3. 에러 클리어(9) 전까지 유지
    bool comm_overrun      = false;   // bit 14: Modbus 수신 오버런(프레임 유실) 발생
    bool eep_save_fail     = false;   // bit 15: EEPROM 저장 실패(그리퍼 맵·통신속도·엔코더 보정)

    // 파지 중 계산 파지력 [N] (register 19) — MATC v3
    uint16_t grip_force_calc_n = 0;

    // Motor feedback (registers 10–17)
    int16_t  motor_pos      = 0;  // [deg]
    int16_t  motor_vel      = 0;  // [RPM]
    int16_t  motor_cur      = 0;  // [mA]
    uint16_t finger_pos     = 0;  // [0–1000 = 0–100 %]
    uint16_t voltage        = 0;  // [V]
    uint16_t states         = 0;
    uint16_t fault_now      = 0;
    uint16_t fault_occurred = 0;

    // Gripper connection (register 18) — MATC v3
    // CONNECT_CHECK(PG0) 하드웨어 신호. true = 그리퍼 물리적으로 연결됨
    bool gripper_connected = false;

    // Gripper physical info (registers 20–23) — MATC v3
    // gripper_id == 128 means no gripper connected
    uint16_t gripper_id          = 128;
    uint16_t gripper_stroke_mm   = 0;   // max stroke [mm]
    uint16_t gripper_force_n     = 0;   // max force  [N]
    uint16_t gripper_payload_x10 = 0;   // payload x10 (e.g. 40 => 4.0 kg)

    // 구버전 펌웨어(레지스터 0~19): reg 10~23 읽기가 Illegal Data Address로 거부된 장치, 또는 읽혀도 reg 20이
    // 그리퍼 ID 범위를 벗어난 장치(레지스터를 늘린 진단판, reg 20 = 0xD1A6).
    // 연결 감지(18)·파지력(19)·그리퍼 정보(20~23)·장치 정보(28~)가 없고, 상태 bit 12가 통신 오버런이며,
    // 위치·파지력 명령 단위가 V3와 다르다(kLegacyTorquePct*).
    bool legacy_map = false;
};

class DatcCtrl {
public:
    DatcCtrl();
    ~DatcCtrl();

    bool modbusInit(const char *port_name, uint16_t slave_address, int baudrate);
    bool modbusRelease();
    bool modbusSlaveChange(uint16_t slave_addr);

    bool motorEnable();
    bool motorStop();           // mode-aware stop (MB_STOP_P)
    bool motorEmergencyStop();  // immediate force stop (MB_STOP_V, MATC v3)
    bool motorDisable();
    bool resetPosition();
    bool errorClear();
    bool systemReset();

    bool setModbusAddr(uint16_t slave_addr);
    bool changeElectricalAngle(int16_t elec_angle_deg = 0);
    // 장치(펌웨어) Modbus 통신속도 변경 — 9600/19200/38400/57600/115200. EEPROM 저장 후 즉시 전환
    bool setModbusBaudrate(int baudrate);
    // 현재 연결로 장치가 실제 응답하는지 확인 (상태 레지스터 1개 읽기)
    bool probe();

    bool grpInitialize();
    bool grpInitialize2(int16_t offset_deg);
    bool grpOpen();
    bool grpClose();
    bool grpSetDirection(bool reverse_direction);

    // Datc control
    bool setFingerPos(uint16_t finger_pos);
    bool motorVelCtrl(int16_t vel);
    bool motorCurCtrl(int16_t cur);
    bool motorPosCtrl(int16_t pos_deg, uint16_t duration);

    bool vacuumGrpOn();
    bool vacuumGrpOff();

    bool setMotorTorque(uint16_t force_n);   // 그립 파지력 [N] (이전: 토크 % )
    bool setMotorSpeed (uint16_t speed_ratio);
    bool setMaxValue(int16_t max_vel, int16_t max_acc);
    bool setPositionPidGain(int16_t kp, int16_t ki, int16_t kd);
    bool setCurrentPiGain(int16_t kp, int16_t ki, int16_t kd = 0);
    bool setVelocityPiGain(int16_t kp, int16_t ki, int16_t kd = 0);

    bool readDatcData();
    DatcStatus getDatcStatus() {return status_;}
    bool getConnectionState() {return mbc_.getConnectionState();}
    bool getModbusRecvErr() {return flag_modbus_recv_err_;}

    uint16_t getSlaveAddr() {return mbc_.getSlaveAddr();}

    // Impedance related functions
    bool impedanceOn();
    bool impedanceOff();
    bool setImpedanceParams(int16_t slave_num, int16_t stiffness_level);
    bool setImpedanceMBK(int16_t mass, int16_t damping, int16_t stiffness);

    // 상태 워드(reg 10)를 즉시 한 번 읽는다 — 저장 결과(bit15) 확인 등 폴링 주기를 기다리지 않을 때
    bool readStatusWord(uint16_t &status);

    // 장치 정보 (regs 28–33, MATC v3.01~). 연결 직후 읽고 이후 약 1초마다 갱신(플래그).
    DeviceInfo getDeviceInfo();
    bool readDeviceInfoNow(DeviceInfo &info);   // 즉시 한 번 읽기 (펌웨어 업데이트 전 모델 확인 등)
    uint16_t getVelMaxRpm();                    // 속도 제어 상한 [rpm]
    uint16_t getCurMaxMa();                     // 전류 제어 100% [mA]

    // Gripper mapping table — MATC v3
    // writeMapEntry: reg1=index, reg2=stroke_mm, reg3=force_n, reg4=payload_x10 (각 16비트)
    bool writeMapEntry(uint16_t index, uint16_t stroke_mm, uint16_t force_n, uint16_t payload_x10);
    bool readMapEntry(uint16_t index);                              // 217 전송만 (응답은 reg 24~27)
    bool queryMapEntry(uint16_t index, GripperMapEntry &entry);     // 217 전송 후 응답(reg 24~27)까지 확인
    bool saveGripperMap();

    // Encoder harmonic compensation — MATC v3 (219~221)
    bool setEncoderHarmonic(uint16_t k, int16_t a, int16_t b);      // 219, 즉시 적용(RAM). 저장은 220
    bool clearEncoderHarmonics();                                   // 219 k=0, 전체 해제(보정 끔)
    bool saveEncoderHarmonics();                                    // 220
    bool queryEncoderHarmonic(uint16_t k, EncHarmEntry &entry);     // 221 전송 후 응답(reg 24~27)까지 확인

    // Encoder harmonic auto calibration — MATC v3.02~ (222/223, reg 33)
    bool startAutoEncHarmonic();                                    // 222 value_1 = 1
    bool cancelAutoEncHarmonic();                                   // 222 value_1 = 0
    bool readAutoEncHarmonicStatus(uint16_t &status);               // reg 33을 즉시 한 번 읽기
    bool queryAutoEncHarmonicResult(uint16_t k, AutoEncHarmResult &result);   // 223 전송 후 응답 확인
    bool setFixedGripperId(uint16_t id);                            // 224 (0~126 저장, 128 = 해제)

    // Dev ui related functions
    bool customCmd(uint16_t cmd, uint16_t value_1 = 0, uint16_t value_2 = 0, uint16_t value_3 = 0);

protected:
    bool checkDurationRange(string error_prefix, uint16_t &duration);
    bool command(DATC_COMMAND cmd, uint16_t value_1 = 0, uint16_t value_2 = 0, uint16_t value_3 = 0);
    bool rejectOnLegacy(uint16_t cmd);   // 구버전 펌웨어에 V3 전용 명령이면 true(보내지 않음)
    bool queryCommand(DATC_COMMAND cmd, uint16_t value_1, vector<uint16_t> &resp);
    void pollDeviceInfo();
    void resetDeviceInfo();   // 연결·슬레이브가 바뀌면 다른 장치일 수 있으므로 다시 읽게 한다

    ModbusComm mbc_;
    DatcStatus status_;

    bool flag_modbus_recv_err_ = false;

    // readDatcData 읽기 모드 — 폴링 스레드만 쓴다. resetDeviceInfo()가 read_mode_reset_을 세우면
    // 다음 주기에 처음부터 다시 판정한다(다른 장치일 수 있으므로).
    std::atomic<bool> read_mode_reset_{true};
    uint16_t read_reg_num_       = kDatcFullReadNum;  // 현재 읽기 개수 (14 또는 8)
    int      full_read_fail_cnt_ = 0;                 // 14개 읽기 연속 실패 횟수
    int      illegal_addr_cnt_   = 0;                 // 그중 Illegal Data Address로 거부된 연속 횟수
    int      fallback_retry_cnt_ = 0;                 // 8개 모드에서 재시도 카운터

    // 장치 정보 — 폴링 스레드가 쓰고 UI/TCP 스레드가 읽으므로 mutex_info_로 보호
    mutex      mutex_info_;
    DeviceInfo device_info_;
    int        info_fail_cnt_ = 0;
    int        info_poll_cnt_ = 0;
};

#endif // DATC_CTRL_HPP
