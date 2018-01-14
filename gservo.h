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

template<typename T>
inline T clamp(T val, T minV, T maxV) {
    return val > maxV ? maxV : val < minV ? minV : val;
}

namespace MotorsConst {
constexpr float unitRpm = 0.111f;
constexpr float unitDegPerSec = unitRpm * 360.f / 60.f;
constexpr float unitDegPerSecInv = 1.f / unitDegPerSec;

constexpr float unitDeg = 300.f / 1023.f;
constexpr float unitDegInv = 1.f / unitDeg;
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

    void loop(unsigned long dtMicros) {
        auto currPos = motorCurrentPos();

        currPos_ = currPos;
    }

    void move(const FVec& goal, const FVec& speed, const FVec& acc) {}

    FVec current() const { return {}; }

    DynamixelStatus broadcastChangeId(DynamixelID id) {
        return di_->write(BROADCAST_ID, 3, id);
    }

private:
    FVec speed(const MVec& speed) {
        return speed.cast<float>() * MotorsConst::unitDegPerSec;
    }

    FVec pos(const MVec& pos) {
        return pos.cast<float>() * MotorsConst::unitDeg;
    }

    MVec speed(const FVec& speed) {
        return (speed * MotorsConst::unitDegPerSecInv).cast<int16_t>();
    }

    MVec pos(const FVec& pos) {
        return (pos * MotorsConst::unitDegInv).cast<int16_t>();
    }

    DynamixelStatus checkMotorsStatus() {
        for (auto m : motor_) {
            if (auto s = m->status()) {
                return s;
            }
        }
        return DYN_STATUS_OK;
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
    MVec currSpeed_{};
    MVec currAcc_{};
    MVec goalPos_{};
    MVec maxSpeed_{};
    MVec maxAcc_{};
};

class CallbacksImpl final : public Callbacks {
    struct Set {
        float zero_[COORDS]{};
        float ratio_[COORDS]{};
        float speed_[COORDS]{};
        float accel_[COORDS]{};
        float retention_[COORDS]{};
    };

public:
    CallbacksImpl(Print* s, Motors* motors) : s_(s), motors_(motors) {}

    void begin() {
        EEPROM.get(0, set_);
        for (int i = 0; i < COORDS; ++i) {
            defSet_.ratio_[i] = 1;
            defSet_.speed_[i] = 1;
            defSet_.accel_[i] = 1;
            defSet_.retention_[i] = 0;
            defSet_.zero_[i] = 0;
        }
        if (isnan(set_.zero_[0])) {
            set_ = defSet_;
        }
        for (int i = 0; i < COORDS; ++i) {
            auto* m = motor_[i];
            m->jointMode();
            m->enableTorque(set_.retention_[i] > 0);
            constexpr uint8_t zero = 0;
            m->write(26, zero);
            m->write(27, zero);
            m->write(28, zero);
            m->write(29, zero);
            m->write(48, zero);
            m->write(49, zero);
        }
    }

    void loop() {
        const bool anyWereMoving = isAnyMoving();
        for (int i = 0; i < COORDS; ++i) {
            if (moving_[i]) {
                const auto currPos = motor_[i]->currentPosition();
                moving_[i] = currPos != goalPos_[i];
            }
        }
        if (anyWereMoving) {
            if (!isAnyMoving()) {
                stopped();
            }
        }
    }

    void stopped() {
        for (int i = 0; i < COORDS; ++i) {
            goalPos_[i] = motor_[i]->currentPosition();
        }
        if (report_) {
            reportCurrentPos();
            report_ = false;
        }
    }

    void eol() override {}

    void homing() override { move({{0, 0}, {true, true}, true}, false); }

    void setMode(Mode g) override { fast_ = g == Mode::Fast; }

    void setSpeed(float val) override { speedOverride_ = val; }

    void move(const Vec& pos, bool report) override {
        for (int i = 0; i < COORDS; ++i) {
            if (pos.has[i]) {
                auto coord = pos.coord[i];
                if (set_.ratio_[i] != 0) {
                    coord *= set_.ratio_[i];
                }
                coord += set_.zero_[i];
                goalPos_[i] = toDynPos(coord);
            }
        }
        report_ = pos.report;
        for (int i = 0; i < COORDS; ++i) {
            auto* m = motor_[i];
            if (!fast_ && speedOverride_ > 0) {
                m->speed(toDynSpeed(speedOverride_));
            } else {
                m->speed(toDynSpeed(set_.speed_[i]));
            }
            m->enableTorque(set_.retention_[i] > 0);
            m->goalPosition(goalPos_[i]);
            moving_[i] = true;
        }
        checkStatus();
    }

    void reportCurrentPos() override {
        for (int i = 0; i < COORDS; ++i) {
            const auto pos = motor_[i]->currentPosition();
            auto coord = fromDynPos(pos);
            coord -= set_.zero_[i];
            coord /= set_.ratio_[i];
            s_->print(coord);
            s_->print(" ");
        }
        s_->print("\n");
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
            GSERVO_SET_SETTING(ratio_, Ratio);
            GSERVO_SET_SETTING(speed_, Speed);
            GSERVO_SET_SETTING(accel_, Accel);
            GSERVO_SET_SETTING(zero_, Zero);
            GSERVO_SET_SETTING(retention_, Retention);
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
        GSERVO_PRINT_SETTING(ratio_, Ratio);
        GSERVO_PRINT_SETTING(speed_, Speed);
        GSERVO_PRINT_SETTING(accel_, Accel);
        GSERVO_PRINT_SETTING(zero_, Zero);
        GSERVO_PRINT_SETTING(retention_, Retention);
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

$100=1                   | set ratio x
$101=1                   | set ratio y
$110=1                   | set speed deg/s x
$111=1                   | set speed deg/s y
$120=1                   | set acceleration deg/s^2 x
$121=1                   | set acceleration deg/s^2 y
$140=0                   | set zero position deg x
$141=0                   | set zero position deg y
$150=0                   | set retention on/off x
$151=0                   | set retention on/off y
$$                       | show setting

%0 id                    | set servo id
%1 name                  | set bluetooth name
%%                       | show help)";
        s_->print(msg);
    }

private:
    bool isAnyMoving() const {
        bool anyMoving = false;
        for (bool i : moving_) {
            anyMoving = anyMoving || i;
        }
        return anyMoving;
    }

    void printSetting(Setting s, float val, const char* name) {
        s_->print("$");
        s_->print((int) s);
        s_->print("=");
        s_->print(val);
        s_->print(" (");
        s_->print(name);
        s_->print(")\n");
    }

    static constexpr float UNIT_PER_DEG = 1024.0f / 300.0f;
    static constexpr float DEG_PER_UNIT = 1.f / UNIT_PER_DEG;

    uint16_t toDynPos(float coordDeg) {
        if (coordDeg <= 0) {
            return 0;
        }
        if (coordDeg >= 300) {
            return 1023;
        }
        return static_cast<uint16_t>(coordDeg * UNIT_PER_DEG);
    }

    float fromDynPos(uint16_t pos) {
        if (pos == 0) {
            return 0;
        }
        if (pos == 1023) {
            return 300;
        }
        return pos * DEG_PER_UNIT;
    }

    uint16_t toDynSpeed(float speedNormalized) {
        if (speedNormalized <= 0) {
            return 0;
        }
        if (speedNormalized >= 1) {
            return 1023;
        }
        return static_cast<uint16_t>(speedNormalized * 1024.f);
    }

    Print* s_;
    Motors* motors_;
    Set set_{}, defSet_;
    uint16_t goalPos_[COORDS]{};
    bool moving_[COORDS]{};
    bool report_{};
    float speedOverride_{};
    bool fast_{};
};

} // namespace gservo
