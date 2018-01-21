#pragma once

#include "parser.h"

#include <DynamixelMotor.h>
#include <EEPROM.h>
#include <Print.h>

namespace gservo {

class JoinPrint final : public Print {
public:
    JoinPrint(Print* s1, Print* s2) : s1_(s1), s2_(s2) {}

    size_t write(uint8_t uint8) override
    {
        const auto w1 = s1_->write(uint8);
        const auto w2 = s2_->write(uint8);
        return min(w1, w2);
    }

private:
    Print *s1_, *s2_;
};

struct Set {
    float homingPullOff_;
    FVec speed_;
    FVec accel_;
    FVec zero_;
    FVec p_;
    FVec i_;
    FVec d_;
    FVec punch_;
    FVec torque_;
};

Set defSettings();

struct Reg {
    Reg(Set* set)
    {
        int it = 0;
        const auto add = [&](unsigned s, float* f) { items[it++] = Item{s, f}; };
        add(27, &set->homingPullOff_);
        for (unsigned i = 0; i < COORDS; ++i) {
            add(110 + i, &set->speed_[i]);
            add(120 + i, &set->accel_[i]);
            add(140 + i, &set->zero_[i]);
            add(200 + i, &set->p_[i]);
            add(210 + i, &set->i_[i]);
            add(220 + i, &set->d_[i]);
            add(230 + i, &set->punch_[i]);
            add(240 + i, &set->torque_[i]);
        }
        sort();
    }

    float get(unsigned s)
    {
        for (const auto& it : items) {
            if (it.snum == s) {
                return *it.f;
            }
        }
        return NAN;
    }

    void set(unsigned s, float val)
    {
        for (const auto& it : items) {
            if (it.snum == s) {
                *it.f = val;
            }
        }
    }

    bool anyNan()
    {
        for (const auto& it : items) {
            if (isnan(*it.f)) {
                return true;
            }
        }
        return false;
    }

    void print(Print& p)
    {
        sort();
        for (const auto& it : items) {
            if (it.snum == 0) {
                return;
            }
            printOne(p, it.snum, *it.f);
        }
    }

private:
    void printOne(Print& p, unsigned s, float val)
    {
        p.print('$');
        p.print(s);
        p.print('=');
        p.print(val);
        p.print('\n');
    }

    int itemsLen()
    {
        for (int i = 0; i < size; ++i) {
            if (items[i].snum == 0) {
                return i;
            }
        }
        return 0;
    }

    void sort()
    {
        const int len = itemsLen();
        for (int i = 1; i < len; i++) {
            int j = i;
            while (j > 0 && items[j - 1].snum > items[j].snum) {
                auto tmp = items[j];
                items[j] = items[j - 1];
                items[j - 1] = tmp;
                j--;
            }
        }
    }

    struct Item {
        unsigned snum;
        float* f;
    };

    static constexpr int size = 32;
    Item items[size]{};
};

namespace MotorsConstAx {
constexpr float unitRpm = 0.111f;
constexpr float unitDegPerSec = unitRpm * 360.f / 60.f;
constexpr float unitDegPerSecInv = 1.f / unitDegPerSec;

constexpr float unitDeg = 300.f / 1023.f;
constexpr float unitDegInv = 1.f / unitDeg;

constexpr float unitDegPerSec2 = 0;
constexpr float unitDegPerSec2Inv = 0;

constexpr int16_t maxPos = 1023;
constexpr int16_t maxSpeed = 1023;
constexpr int16_t maxAcc = 0;
} // namespace MotorsConstAx

namespace MotorsConstMx {
constexpr float unitRpm = 0.916f;
constexpr float unitDegPerMin = unitRpm * 360.f;
constexpr float unitDegPerMinInv = 1.f / unitDegPerMin;

constexpr float unitDeg = 0.088f;
constexpr float unitDegInv = 1.f / unitDeg;

constexpr float unitDegPerSec2 = 8.583f;
constexpr float unitDegPerSec2Inv = 1.f / unitDegPerSec2;

constexpr int16_t maxPos = 4095;
constexpr int16_t maxSpeed = 1023;
constexpr float maxSpeedDegPerSec = maxSpeed * unitDegPerMin;
constexpr int16_t maxAcc = 254;
constexpr float maxAccDegPerSec2 = maxAcc * unitDegPerSec2;
} // namespace MotorsConstMx

namespace MotorsConst = MotorsConstMx;

class Motors {
public:
    using MVec = Vec<int16_t>;

    Motors(DynamixelInterface* di) : di_(di)
    {
        for (int i = 0; i < COORDS; ++i) {
            const auto id = static_cast<DynamixelID>(i + 1);
            motor_[i] = new DynamixelMotor(*di_, id);
        }
    }

    ~Motors()
    {
        for (auto& m : motor_) {
            delete m;
        }
    }

    void init()
    {
        s_ = DYN_STATUS_OK;
        for (auto m : motor_) {
            s_ |= m->init();
            m->jointMode(0, MotorsConst::maxPos);
        }
    }

    void updateSettings(const Set& s)
    {
        s_ = DYN_STATUS_OK;
        const auto mAcc = clampEach((s.accel_ * MotorsConstMx::unitDegPerSec2Inv).round<uint8_t>(),
                                    0u,
                                    MotorsConst::maxAcc);
        const auto pGain = clampEach((s.p_ * 254.f).round<uint8_t>(), 0u, 254u);
        const auto iGain = clampEach((s.i_ * 254.f).round<uint8_t>(), 0u, 254u);
        const auto dGain = clampEach((s.d_ * 254.f).round<uint8_t>(), 0u, 254u);
        const auto punch = clampEach((s.punch_ * 1023.f).round<uint16_t>(), 0u, 1023u);
        const auto torque = clampEach((s.torque_ * 1023.f).round<uint16_t>(), 0u, 1023u);
        for (int i = 0; i < COORDS; ++i) {
            auto m = motor_[i];
            s_ |= m->write(0X20, uint16_t{0});
            s_ |= m->write(0X49, mAcc[i]);
            s_ |= m->write(0X1A, dGain[i]);
            s_ |= m->write(0X1B, iGain[i]);
            s_ |= m->write(0X1C, pGain[i]);
            s_ |= m->write(0X30, punch[i]);
            s_ |= m->write(0X22, torque[i]);
            s_ |= m->write(0X0E, torque[i]);
        }
    }

    void enable(bool b, int coord = -1)
    {
        if (coord < 0) {
            s_ = di_->write(BROADCAST_ID, DYN_ADDRESS_ENABLE_TORQUE, static_cast<uint8_t>(b));
        }
        else {
            motor_[coord]->enableTorque(b);
        }
    }

    void loop() { currPos_ = motorCurrentPos(); }

    void move(const FVec& goal, const FVec& speed)
    {
        const auto mSpeed = convSpeed(clampEach(speed, 0.f, MotorsConst::maxSpeedDegPerSec));
        for (int i = 0; i < COORDS; ++i) {
            motor_[i]->speed(mSpeed[i]);
        }
        goalPos_ = clampEach(convPos(goal), 0u, MotorsConst::maxPos);
        sendMoveToGoal();
    }

    void stop()
    {
        goalPos_ = currPos_;
        sendMoveToGoal();
    }

    bool isMoving() const { return goalPos_ != currPos_; }

    FVec currentPos() { return convPos(motorCurrentPos()); }

    void changeId(DynamixelID id, DynamixelID newId)
    {
        const auto bnewId = static_cast<uint8_t>(newId);
        s_ = di_->write(id, DYN_ADDRESS_ID, bnewId);
    }

    DynamixelID getId(DynamixelID id)
    {
        uint8_t bnewId{0xFF};
        s_ = di_->read(id, DYN_ADDRESS_ID, bnewId);
        return bnewId;
    }

    void led(bool on = false, DynamixelID id = BROADCAST_ID)
    {
        const auto bon = static_cast<uint8_t>(on);
        s_ = di_->write(id, 0X19, bon);
    }

    void changeBaud(bool fast = false, DynamixelID id = BROADCAST_ID)
    {
        const auto baud = static_cast<uint8_t>(fast ? 1u : 207u);
        s_ = di_->write(id, DYN_ADDRESS_BAUDRATE, baud);
    }

    uint16_t read(uint8_t addr, DynamixelID id)
    {
        uint16_t val{0xFFFF};
        s_ = di_->read(id, addr, val);
        return val;
    }

    GStr status()
    {
        return statusMsg(s_);
        s_ = DYN_STATUS_OK;
    }

private:
    void sendMoveToGoal()
    {
        for (int i = 0; i < COORDS; ++i) {
            motor_[i]->goalPosition(static_cast<uint16_t>(goalPos_[i]));
        }
        s_ = motorCurrentStatus();
    }

    inline GStr statusMsg(DynamixelStatus s)
    {
        if (s == DYN_STATUS_OK) {
            return nullptr;
        }
        if (s == DYN_STATUS_INTERNAL_ERROR) {
            return F("Invalid command parameters");
        }
        if (s & DYN_STATUS_COM_ERROR) {
            if (s & DYN_STATUS_TIMEOUT) {
                return F("communication error, timeout");
            }
            else if (s & DYN_STATUS_CHECKSUM_ERROR) {
                return F("communication error, invalid response checksum");
            }
            return F("communication error");
        }
        else {
            if (s & DYN_STATUS_INPUT_VOLTAGE_ERROR) {
                return F("invalid input voltage");
            }
            if (s & DYN_STATUS_ANGLE_LIMIT_ERROR) {
                return F("angle limit error");
            }
            if (s & DYN_STATUS_OVERHEATING_ERROR) {
                return F("overheating");
            }
            if (s & DYN_STATUS_RANGE_ERROR) {
                return F("out of range value");
            }
            if (s & DYN_STATUS_CHECKSUM_ERROR) {
                return F("invalid command checksum");
            }
            if (s & DYN_STATUS_OVERLOAD_ERROR) {
                return F("overload");
            }
            if (s & DYN_STATUS_INSTRUCTION_ERROR) {
                return F("invalid instruction");
            }
        }
        return F("unknown error");
    }

    FVec convSpeed(const MVec& speed) { return speed.cast<float>() * MotorsConst::unitDegPerMin; }

    FVec convPos(const MVec& pos) { return pos.cast<float>() * MotorsConst::unitDeg; }

    MVec convSpeed(const FVec& speed)
    {
        return (speed * MotorsConst::unitDegPerMinInv).round<int16_t>();
    }

    MVec convPos(const FVec& pos) { return (pos * MotorsConst::unitDegInv).round<int16_t>(); }

    MVec motorCurrentPos()
    {
        MVec pos{};
        for (int i = 0; i < COORDS; ++i) {
            pos[i] = motor_[i]->currentPosition();
        }
        return pos;
    }

    MVec motorCurrentSpeed()
    {
        MVec speed{};
        for (int i = 0; i < COORDS; ++i) {
            uint16_t s{};
            motor_[i]->read(0X26, s);
            if (s >= 1024) {
                speed[i] = -static_cast<int16_t>(s & ~1024);
            }
            else {
                speed[i] = s;
            }
        }
        return speed;
    }

    DynamixelStatus motorCurrentStatus()
    {
        for (auto m : motor_) {
            if (auto s = m->status()) {
                return s;
            }
        }
        return DYN_STATUS_OK;
    }

    DynamixelInterface* di_{};
    DynamixelMotor* motor_[COORDS]{};
    MVec currPos_{};
    MVec goalPos_{};
    DynamixelStatus s_{DYN_STATUS_OK};
};

class CallbacksImpl final : public Callbacks {
public:
    CallbacksImpl(Print* s, Motors* motors) : s_(s), motors_(motors) {}

    void begin()
    {
        motors_->init();
        eol();
        EEPROM.get(0, set_);
        if (Reg{&set_}.anyNan()) {
            set_ = defSettings();
        }
        motors_->updateSettings(set_);
    }

    void loop()
    {
        bool wereMoving = motors_->isMoving();
        motors_->loop();
        if (wereMoving) {
            if (!motors_->isMoving()) {
                stopped();
            }
        }
    }

    void stopped()
    {
        if (report_) {
            reportCurrentPos();
            report_ = false;
        }
    }

    void eol() override
    {
        if (auto s = motors_->status()) {
            s_->print(F("Error: "));
            s_->print(s);
            s_->print(F("\n"));
        }
    }

    void homing() override { move(FVec::ofConst(set_.homingPullOff_), true); }

    void setMode(Mode g) override { fast_ = g == Mode::Fast; }

    void setSpeed(float val) override { speedOverride_ = val; }

    void move(const FVec& pos, bool report) override
    {
        report_ = report;
        auto goal = motors_->currentPos();
        for (int i = 0; i < COORDS; ++i) {
            if (pos.has(i)) {
                goal[i] = pos[i] + set_.zero_[i];
            }
        }
        auto speed = set_.speed_;
        if (!fast_ && speedOverride_ > 0) {
            speed = FVec::ofConst(speedOverride_);
        }
        motors_->move(goal, speed);
    }

    void reportCurrentPos() override
    {
        const auto pos = motors_->currentPos() - set_.zero_;
        s_->print("MPos:");
        for (int i = 0; i < COORDS; ++i) {
            s_->print(pos[i]);
            s_->print(',');
        }
        s_->print("0\n");
    }

    void stop() override { motors_->stop(); }

    void setSetting(unsigned s, float val, bool hasVal) override
    {
        if (s == 1) {
            motors_->enable(val == 255.f);
        }
        else if (s > 250u && s < 250u + COORDS) {
            motors_->enable(hasVal && val > 0, s - 250);
        }
        else {
            const auto old = set_;
            if (!hasVal) {
                auto ds = defSettings();
                val = Reg{&ds}.get(s);
            }
            Reg{&set_}.set(s, val);
            motors_->updateSettings(set_);
            if (memcmp(&old, &set_, sizeof(Set)) != 0) {
                EEPROM.put(0, set_);
            }
        }
        s_->print(F("Ok\n"));
    }

    void showSettings() override { Reg{&set_}.print(*s_); }

    void error(GStr msg) override
    {
        s_->print(msg);
        s_->print(F("; "));
    }

    void errorPos(char c, int i) override
    {
        s_->print(F(" char "));
        s_->print(c);
        s_->print(F(" at "));
        s_->print(i);
        s_->print(F("; "));
    }

    void help() override
    {
        const auto msg = F(R"(
 Application:
$H                       | homing to zero position
g0 x%.2f y%.2f           | generic movement
g1 x%.2f y%.2f f%.2f     | generic movement with given speed
g0 x%.2f M2              | x axis only movement and report position after move
x%.2f                    | x axis only movement
?                        | ask current position

%0 id newId              | set servo id use id=254 to broadcast
%1 id bool               | turn servo led to 1=on, 0=off
%2 id val                | generic read
%%                       | show help

$$                       | show setting
$1=255                   | set enable both axis then set to 255
$27=0                    | homing pull off, deg
$110=0                   | set speed deg/min x, zero is full speed
$111=0                   | set speed deg/min y, zero is full speed
$120=0                   | set acceleration deg/s^2 x, zero is full acceleration
$121=0                   | set acceleration deg/s^2 y, zero is full acceleration
$140=0                   | set zero position deg x
$141=0                   | set zero position deg y
$200=0.1                 | set proportional gain x, from 0 to 1
$201=0.1                 | set proportional gain y, from 0 to 1
$210=0                   | set integral gain x, from 0 to 1
$211=0                   | set integral gain y, from 0 to 1
$220=0.05                | set derivative gain x, from 0 to 1
$221=0.05                | set derivative gain y, from 0 to 1
$230=0                   | set punch x, from 0 to 1
$231=0                   | set punch y, from 0 to 1
$240=1                   | set torque x, from 0 to 1
$241=1                   | set torque y, from 0 to 1
$250=1                   | set enable x, 1 or 0
$251=1                   | set enable y, 1 or 0
)");
        s_->print(msg);
    }

    void servoId(unsigned cmd, int id, int val) override
    {
        const auto id1 = static_cast<DynamixelID>(id >= 0 ? id : BROADCAST_ID);
        switch (cmd) {
        case 0:
            if (val >= 0) {
                motors_->changeId(id1, static_cast<DynamixelID>(val));
            }
            else {
                s_->print(motors_->getId(id1));
                s_->print('\n');
            }
            break;
        case 1:
            motors_->led(val > 0, id1);
            break;
        case 2:
            if (id >= 0 && val >= 0) {
                auto res = motors_->read(static_cast<uint8_t>(val), static_cast<DynamixelID>(id));
                s_->print(res);
                s_->print('\n');
            }
            break;
        default:
            s_->print(F("Wrong command "));
            s_->print(cmd);
            s_->print("\n");
            break;
        }
    }

private:
    Print* s_;
    Motors* motors_;
    Set set_{};
    bool report_{};
    float speedOverride_{};
    bool fast_{};
};

} // namespace gservo
