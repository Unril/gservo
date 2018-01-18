#pragma once

#include "parser.h"

#include <DynamixelMotor.h>
#include <EEPROM.h>
#include <Print.h>

namespace gservo {

class JoinPrint final : public Print {
public:
    JoinPrint(Print* s1, Print* s2) : s1_(s1), s2_(s2) {}

    size_t write(uint8_t uint8) override {
        const auto w1 = s1_->write(uint8);
        const auto w2 = s2_->write(uint8);
        return min(w1, w2);
    }

private:
    Print* s1_, * s2_;
};

class HC06 {
public:
    HC06(Print* s) : s_(s) {}

    bool setBaudRate(uint32_t baud) {
        const char c = letter(baud);
        if (c == '\0') {
            return false;
        }
        s_->print("AT+BAUD");
        s_->print(c);
        s_->print('\n');
        return true;
    }

    bool setName(const char* name) {
        s_->print("AT+NAME");
        s_->print(name);
        s_->print('\n');
        return true;
    }

private:
    char letter(uint32_t baud) {
        switch (baud) {
            case 1200u:return '1';
            case 2400u:return '2';
            case 4800u:return '3';
            case 9600u:return '4';
            case 19200u:return '5';
            case 38400u:return '6';
            case 57600u:return '7';
            case 115200u:return '8';
            case 230400u:return '9';
            case 460800u:return 'A';
            case 921600u:return 'B';
            case 1382400u:return 'C';
            default:s_->print("unsupported baud rate");
                return '\0';
        }
    }

    Print* s_;
};

inline const char* statusMsg(DynamixelStatus s) {
    if (s == DYN_STATUS_OK) {
        return nullptr;
    }
    if (s == DYN_STATUS_INTERNAL_ERROR) {
        return "Invalid command parameters";
    }
    if (s & DYN_STATUS_COM_ERROR) {
        if (s & DYN_STATUS_TIMEOUT) {
            return "communication error, timeout";
        } else if (s & DYN_STATUS_CHECKSUM_ERROR) {
            return "communication error, invalid response checksum";
        }
        return "communication error";
    } else {
        if (s & DYN_STATUS_INPUT_VOLTAGE_ERROR) {
            return "invalid input voltage";
        }
        if (s & DYN_STATUS_ANGLE_LIMIT_ERROR) {
            return "angle limit error";
        }
        if (s & DYN_STATUS_OVERHEATING_ERROR) {
            return "overheating";
        }
        if (s & DYN_STATUS_RANGE_ERROR) {
            return "out of range value";
        }
        if (s & DYN_STATUS_CHECKSUM_ERROR) {
            return "invalid command checksum";
        }
        if (s & DYN_STATUS_OVERLOAD_ERROR) {
            return "overload";
        }
        if (s & DYN_STATUS_INSTRUCTION_ERROR) {
            return "invalid instruction";
        }
    }
    return "unknown error";
}

namespace MotorsConst {
constexpr float unitRpm = 0.111f;
constexpr float unitDegPerSec = unitRpm * 360.f / 60.f;
constexpr float unitDegPerSecInv = 1.f / unitDegPerSec;

constexpr float unitDeg = 300.f / 1023.f;
constexpr float unitDegInv = 1.f / unitDeg;

constexpr int16_t maxPos = 1023;
constexpr int16_t maxSpeed = 1023;
}

class Motors {
public:
    using MVec = Vec<int16_t>;

    Motors(DynamixelInterface* di) : di_(di) {
        for (int i = 0; i < COORDS; ++i) {
            const auto id = static_cast<DynamixelID>(i + 1);
            motor_[i] = new DynamixelMotor(*di_, id);
        }
    }

    ~Motors() {
        for (auto& m : motor_) {
            delete m;
        }
    }

    void init() {
        for (auto m : motor_) {
            m->init();
            m->jointMode();
            m->enableTorque();
        }
    }

    void loop(float dtSec) {
        currPos_ = motorCurrentPos();
    }

    void move(const FVec& goal, const FVec& speed, const FVec& acc) {
        const auto mSpeed = clampEach(convSpeed(speed), MVec::c(0),
                                      MVec::c(MotorsConst::maxSpeed));
        goalPos_ = clampEach(convPos(goal), MVec::c(0),
                             MVec::c(MotorsConst::maxPos));
        setMotorSpeed(mSpeed);
        setMotorGoalPos(goalPos_);
    }

    bool isMoving() const { return goalPos_ != currPos_; }

    FVec currentPos() {
        return convPos(motorCurrentPos());
    }

    DynamixelStatus broadcastChangeId(DynamixelID id) {
        return di_->write(BROADCAST_ID, 3, id);
    }

    DynamixelStatus status() {
        for (auto m : motor_) {
            if (auto s = m->status()) {
                return s;
            }
        }
        return DYN_STATUS_OK;
    }

private:
    FVec convSpeed(const MVec& speed) {
        return speed.cast<float>() * MotorsConst::unitDegPerSec;
    }

    FVec convPos(const MVec& pos) {
        return pos.cast<float>() * MotorsConst::unitDeg;
    }

    MVec convSpeed(const FVec& speed) {
        return (speed * MotorsConst::unitDegPerSecInv).cast<int16_t>();
    }

    MVec convPos(const FVec& pos) {
        return (pos * MotorsConst::unitDegInv).cast<int16_t>();
    }

    void setMotorGoalPos(const MVec& pos) {
        for (int i = 0; i < COORDS; ++i) {
            motor_[i]->goalPosition(static_cast<uint16_t>(pos[i]));
        }
    }

    MVec motorCurrentPos() {
        MVec pos{};
        for (int i = 0; i < COORDS; ++i) {
            pos[i] = motor_[i]->currentPosition();
        }
        return pos;
    }

    void setMotorSpeed(const MVec& speed) {
        for (int i = 0; i < COORDS; ++i) {
            motor_[i]->speed(speed[i]);
        }
    }

    MVec motorCurrentSpeed() {
        MVec speed{};
        for (int i = 0; i < COORDS; ++i) {
            uint16_t s{};
            motor_[i]->read(0X26, s);
            if (s >= 1024) {
                speed[i] = -static_cast<int16_t>(s & ~1024);
            } else {
                speed[i] = s;
            }
        }
        return speed;
    }

    DynamixelInterface* di_{};
    DynamixelMotor* motor_[COORDS]{};
    MVec currPos_{};
    MVec goalPos_{};
};

class CallbacksImpl final : public Callbacks {
    struct Set {
        FVec zero_{};
        FVec speed_{};
        FVec accel_{};
    };

public:
    CallbacksImpl(Print* s, Motors* motors) : s_(s), motors_(motors) {}

    void begin() {
        EEPROM.get(0, set_);
        for (int i = 0; i < COORDS; ++i) {
            defSet_.speed_[i] = 1;
            defSet_.accel_[i] = 1;
            defSet_.zero_[i] = 0;
        }
        if (isnan(set_.zero_[0])) {
            set_ = defSet_;
        }
    }

    void loop(float dtSec) {
        bool wereMoving = motors_->isMoving();
        motors_->loop(dtSec);
        if (wereMoving) {
            if (!motors_->isMoving()) {
                stopped();
            }
        }
    }

    void stopped() {
        if (report_) {
            reportCurrentPos();
            report_ = false;
        }
    }

    void eol() override {}

    void homing() override { move(FVec::c(0), true); }

    void setMode(Mode g) override { fast_ = g == Mode::Fast; }

    void setSpeed(float val) override { speedOverride_ = val; }

    void move(const FVec& pos, bool report) override {
        report_ = report;
        auto goal = motors_->currentPos();
        for (int i = 0; i < COORDS; ++i) {
            if (pos.has(i)) {
                goal[i] = pos[i] + set_.zero_[i];
            }
        }
        auto speed = set_.speed_;
        if (!fast_ && speedOverride_ > 0) {
            speed = FVec::c(speedOverride_);
        }
        motors_->move(goal, speed, set_.accel_);
    }

    void reportCurrentPos() override {
        const auto pos = motors_->currentPos() - set_.zero_;
        s_->print("MPos:");
        for (int i = 0; i < COORDS; ++i) {
            s_->print(pos[i]);
            s_->print(",");
        }
        s_->print("0\n");
    }

    void setSetting(Setting s, float val, bool hasVal) override {
#define GSERVO_SET_SETTING(field, name)                                      \
    case Setting::name##X:                                                   \
    case Setting::name##Y:                                                   \
        set_.field[s - Setting::name##X]                                     \
            = hasVal ? val : defSet_.field[s - Setting::name##X];            \
        break;
        const auto old = set_;
        switch (s) {
            GSERVO_SET_SETTING(speed_, Speed);
            GSERVO_SET_SETTING(accel_, Accel);
            GSERVO_SET_SETTING(zero_, Zero);
            default:s_->print("unexpected setting\n");
                return;
        }
#undef GSERVO_SET_SETTING
        if (memcmp(&old, &set_, sizeof(Set)) != 0) {
            EEPROM.put(0, set_);
        }
        s_->print("Ok\n");
    }

    void showSettings() override {
#define GSERVO_PRINT_SETTING(field, name)                                    \
    for (int i = 0; i < COORDS; ++i) {                                       \
        printSetting(Setting::name##X + i, set_.field[i], #name);            \
    }
        GSERVO_PRINT_SETTING(speed_, Speed);
        GSERVO_PRINT_SETTING(accel_, Accel);
        GSERVO_PRINT_SETTING(zero_, Zero);
#undef GSERVO_PRINT_SETTING
    }

    void error(const char* msg) override {
        s_->print(msg);
        s_->print("; ");
    }

    void errorPos(char c, int i) override {
        s_->print(" character ");
        s_->print(c);
        s_->print(" at ");
        s_->print(i);
        s_->print("; ");
    }

    void help() override {
        const char* msg = R"( Application:
$H                       | homing to zero position
g0 x%.2f y%.2f           | generic movement
g1 x%.2f y%.2f f%.2f     | generic movement with given speed
g0 x%.2f M2              | x axis only movement and report position after move
x%.2f                    | x axis only movement
?                        | ask current position

$110=1                   | set speed deg/s x
$111=1                   | set convSpeed deg/s y
$120=1                   | set acceleration deg/s^2 x
$121=1                   | set acceleration deg/s^2 y
$140=0                   | set zero position deg x
$141=0                   | set zero position deg y
$$                       | show setting

%0 id                    | set servo id
%%                       | show help)";
        s_->print(msg);
    }

private:
    void printSetting(Setting s, float val, const char* name) {
        s_->print("$");
        s_->print((int) s);
        s_->print("=");
        s_->print(val);
        s_->print(" (");
        s_->print(name);
        s_->print(")\n");
    }

    Print* s_;
    Motors* motors_;
    Set set_{}, defSet_;
    bool report_{};
    float speedOverride_{};
    bool fast_{};
};

} // namespace gservo
