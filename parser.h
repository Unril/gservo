#pragma once

#include <inttypes.h>
#include <math.h>

namespace gservo {

/*
 Application:
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
%%                       | show help
*/

constexpr int COORDS = 2;

constexpr char coordNames[COORDS]{'x', 'y'};

template<typename T>
struct Vec {
    T coord[COORDS];

    T* begin() { return coord; }

    T* end() { return coord + COORDS; }

    T& operator[](int i) { return coord[i]; }

    const T* begin() const { return coord; }

    const T* end() const { return coord + COORDS; }

    T operator[](int i) const { return coord[i]; }

    static Vec c(T val) {
        Vec v{};
        for (auto& c : v.coord) {
            c = val;
        }
        return v;
    }

    static Vec nan() { return c(_nanf()); }

    template<typename M>
    Vec<M> cast() const {
        Vec<M> v{};
        for (int i = 0; i < COORDS; ++i) {
            v[i] = static_cast<M>(coord[i]);
        }
        return v;
    }

    void setNan(int i) { coord[i] = _nanf(); }

    bool has(int i) const { return !isnan(coord[i]); }

    bool any() const {
        for (auto& c : coord) {
            if (!isnan(c)) {
                return true;
            }
        }
        return false;
    }

    bool all() const {
        for (auto& c : coord) {
            if (isnan(c)) {
                return false;
            }
        }
        return true;
    }

    T min() const {
        T v = INFINITY;
        for (auto& c : coord) {
            v = fmin(v, c);
        }
        return v;
    }

    T max() const {
        T v = -INFINITY;
        for (auto& c : coord) {
            v = fmax(v, c);
        }
        return v;
    }

    Vec& operator+=(const Vec& v) {
        for (int i = 0; i < COORDS; ++i) {
            coord[i] += v.coord[i];
        }
        return *this;
    }

    inline friend Vec operator+(Vec a, const Vec& b) { return a += b; }

    inline friend Vec operator+(T a, const Vec& b) {
        return Vec::c(a) += b;
    }

    inline friend Vec operator+(Vec a, T b) { return a += Vec::c(b); }

    Vec& operator-=(const Vec& v) {
        for (int i = 0; i < COORDS; ++i) {
            coord[i] -= v.coord[i];
        }
        return *this;
    }

    inline friend Vec operator-(Vec a, const Vec& b) { return a -= b; }

    inline friend Vec operator-(T a, const Vec& b) {
        return Vec::c(a) -= b;
    }

    inline friend Vec operator-(Vec a, T b) { return a -= Vec::c(b); }

    Vec& operator*=(const Vec& v) {
        for (int i = 0; i < COORDS; ++i) {
            coord[i] *= v.coord[i];
        }
        return *this;
    }

    inline friend Vec operator*(Vec a, const Vec& b) { return a *= b; }

    inline friend Vec operator*(T a, const Vec& b) {
        return Vec::c(a) *= b;
    }

    inline friend Vec operator*(Vec a, T b) { return a *= Vec::c(b); }

    Vec& operator/=(const Vec& v) {
        for (int i = 0; i < COORDS; ++i) {
            coord[i] /= v.coord[i];
        }
        return *this;
    }

    inline friend Vec operator/(Vec a, const Vec& b) { return a /= b; }

    inline friend Vec operator/(T a, const Vec& b) {
        return Vec::c(a) /= b;
    }

    inline friend Vec operator/(Vec a, T b) { return a /= Vec::c(b); }
};

using FVec = Vec<float>;
using IVec = Vec<int>;

enum class Mode : unsigned {
    Fast = 0, Normal = 1,
};

enum class Setting : unsigned {
    RatioX = 100, RatioY = 101, SpeedX = 110, SpeedY = 111, AccelX = 120,
    AccelY = 121, ZeroX = 140, ZeroY = 141, RetentionX = 150, RetentionY = 151,
};

inline int operator-(Setting a, Setting b) { return (int) a - (int) b; }

inline Setting operator+(Setting a, int b) {
    return static_cast<Setting>((int) a + b);
}

class Callbacks {
public:
    virtual ~Callbacks() = default;

    virtual void eol() = 0;

    virtual void homing() = 0;

    virtual void setMode(Mode g) = 0;

    virtual void setSpeed(float val) = 0;

    virtual void move(const FVec& pos, bool report) = 0;

    virtual void reportCurrentPos() = 0;

    virtual void setSetting(Setting s, float val, bool hasVal) = 0;

    virtual void showSettings() = 0;

    virtual void servoId(unsigned i) = 0;

    virtual void help() = 0;

    virtual void error(const char* msg) = 0;

    virtual void errorPos(char c, int i) = 0;
};

class Parser {
public:
    Parser(Callbacks* cb) : cb_(cb) {}

    void parse(const char* str, int len) {
        c_ = str;
        pos_ = 0;
        len_ = len;
        while (pos_ < len_ && curr()) {
            if (!parseLine()) {
                cb_->errorPos(curr(), pos_);
                skipLine();
            }
        }
    }

private:
    bool parseLine() {
        if (consume('?')) {
            cb_->reportCurrentPos();
        } else if (checkCoord()) {
            if (!parseMove()) {
                cb_->error("expect move");
                return false;
            }
        } else if (consume('g')) {
            unsigned code = 0;
            if (!parseUnsigned(code)) {
                cb_->error("expect unsigned integer");
                return false;
            }
            cb_->setMode(static_cast<Mode>(code));
            if (!parseMove()) {
                cb_->error("expect move");
                return false;
            }
        } else if (consume('$')) {
            if (consume('$')) {
                cb_->showSettings();
            } else if (consume('h')) {
                cb_->homing();
            } else if (!parseSetSetting()) {
                cb_->error("expect set setting");
                return false;
            }
        } else if (consume('%')) {
            if (consume('%')) {
                cb_->help();
            } else {
                unsigned i{};
                if (!parseUnsigned(i)) {
                    cb_->error("expect unsigned number");
                    return false;
                }
                skip();
                switch (i) {
                    case 0:
                        if (!parseUnsigned(i)) {
                            cb_->error("expect unsigned number");
                            return false;
                        }
                        cb_->servoId(i);
                        break;
                    default:cb_->error("wrong command number");
                        break;
                }
            }
        }
        return requireEol();
    }

    bool parseMove() {
        float speed{};
        bool hasSpeed = false;
        if (checkSpeed()) {
            if (!parseSpeed(speed)) {
                cb_->error("expect speed");
                return false;
            }
            hasSpeed = true;
        }
        FVec pos = FVec::nan();
        if (!parsePos(pos)) {
            cb_->error("expect position");
            return false;
        }
        if (!hasSpeed && checkSpeed()) {
            if (!parseSpeed(speed)) {
                cb_->error("expect speed");
                return false;
            }
            hasSpeed = true;
        }
        bool report = false;
        if (pos.any() && consume('m')) {
            if (!consume('2', false)) {
                cb_->error("expect m2");
                return false;
            }
            report = true;
        }
        if (hasSpeed) {
            cb_->setSpeed(speed);
        }
        if (pos.any()) {
            cb_->move(pos, report);
        }
        return true;
    }

    bool parseSetSetting() {
        unsigned s = 0;
        if (!parseUnsigned(s)) {
            cb_->error("expect unsigned integer");
            return false;
        }
        if (!consume('=')) {
            cb_->error("expect =");
            return false;
        }
        float val{};
        bool hasVal = false;
        if (checkFloat()) {
            if (!parseFloat(val)) {
                cb_->error("expect floating point");
                return false;
            }
            hasVal = true;
        }
        cb_->setSetting(static_cast<Setting>(s), val, hasVal);
        return true;
    }

    bool checkSpeed() { return check('f'); }

    bool parseSpeed(float& val) {
        if (!consume('f')) {
            return false;
        }
        if (!parseFloat(val)) {
            cb_->error("expect floating point after f");
            return false;
        }
        return true;
    }

    bool checkCoord() const {
        int i{};
        return checkCoord(i);
    }

    bool checkCoord(int& i) const {
        for (i = 0; i < COORDS; ++i) {
            if (check(coordNames[i])) {
                return true;
            }
        }
        return false;
    }

    bool parsePos(FVec& pos) {
        int i{};
        while (checkCoord(i)) {
            if (!parseCoord(pos, i)) {
                return false;
            }
        }
        return true;
    }

    bool parseCoord(FVec& pos, int coord) {
        if (consume(coordNames[coord])) {
            if (!parseFloat(pos[coord])) {
                cb_->error("expect floating point");
                return false;
            }
            return true;
        }
        return false;
    }

    bool checkFloat() { return check('-') || check('.') || isDigit(); }

    bool parseFloat(float& val) {
        if (!checkFloat()) {
            cb_->error("expect digit or negation sign");
            return false;
        }
        bool isNegative = false;
        bool isFraction = false;
        long long value = 0;
        float fraction = 1.0;
        unsigned digit{};
        if (consume('-')) {
            isNegative = true;
        }
        do {
            if (consume('.', false)) {
                if (isFraction) {
                    cb_->error("unexpected dot");
                    return false;
                }
                isFraction = true;
            } else if (consumeDigit(digit)) {
                value = value * 10 + digit;
                if (isFraction) {
                    fraction *= 0.1f;
                }
            } else {
                cb_->error("expect digit or fraction separator");
                return false;
            }
        } while (isDigit() || (check('.') && !isFraction));

        if (isNegative) {
            value = -value;
        }
        if (isFraction) {
            val = value * fraction;
        } else {
            val = value;
        }
        skip();
        return true;
    }

    bool parseUnsigned(unsigned& i) {
        unsigned curr = 0;
        if (!consumeDigit(curr)) {
            cb_->error("expect digit");
            return false;
        }
        do {
            i = curr + i * 10;
        } while (consumeDigit(curr));
        skip();
        return true;
    }

    bool isDigit() const { return curr() >= '0' && curr() <= '9'; }

    bool consumeDigit(unsigned& digit) {
        if (isDigit()) {
            digit = static_cast<unsigned>(curr() - '0');
            next(false);
            return true;
        }
        return false;
    }

    void skipLine() {
        while (curr() && !check('\n')) {
            next(false);
        }
        consume('\n', false);
    }

    void skip() {
        while (check(' ')) {
            ++pos_;
        }
    }

    void next(bool skipAfter = true) {
        do {
            ++pos_;
        } while (skipAfter && check(' '));
    }

    char curr() const { return c_[pos_]; }

    bool check(char c) const { return curr() == c; }

    bool consume(char c, bool skipAfter = true) {
        if (check(c)) {
            next(skipAfter);
            return true;
        }
        return false;
    }

    bool requireEol() {
        if (!consume('\n')) {
            cb_->error("expect end of line");
            return false;
        };
        cb_->eol();
        return true;
    }

    Callbacks* cb_{};
    const char* c_{};
    int pos_{};
    int len_{};
};

} // namespace gservo
