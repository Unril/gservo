#pragma once

#include <inttypes.h>
#include <math.h>

namespace gservo {

#ifndef PROGMEM
#define __FlashStringHelper char
#define PSTR(s) s
#define F(s) s
#endif
using GStr = const __FlashStringHelper*;

constexpr int COORDS = 2;

constexpr char coordNames[COORDS]{'x', 'y'};

template <typename T, typename TMin, typename TMax>
inline T clamp(T val, TMin minV, TMax maxV)
{
    return val > maxV ? maxV : val < minV ? minV : val;
}

template <typename T>
struct Vec {
    T coord[COORDS];

    T* begin() { return coord; }

    T* end() { return coord + COORDS; }

    T& operator[](int i) { return coord[i]; }

    const T* begin() const { return coord; }

    const T* end() const { return coord + COORDS; }

    T operator[](int i) const { return coord[i]; }

    static Vec ofConst(T val)
    {
        Vec v{};
        for (auto& c : v.coord) {
            c = val;
        }
        return v;
    }

    static Vec ofNaN() { return ofConst(NAN); }

    template <typename M>
    Vec<M> cast() const
    {
        Vec<M> v{};
        for (int i = 0; i < COORDS; ++i) {
            v[i] = static_cast<M>(coord[i]);
        }
        return v;
    }

    template <typename M>
    Vec<M> round() const
    {
        Vec<M> v{};
        for (int i = 0; i < COORDS; ++i) {
            v[i] = static_cast<M>(lroundf(coord[i]));
        }
        return v;
    }

    bool has(int i) const { return !isnan(coord[i]); }

    bool any() const
    {
        for (auto& c : coord) {
            if (!isnan(c)) {
                return true;
            }
        }
        return false;
    }

    bool all() const
    {
        for (auto& c : coord) {
            if (isnan(c)) {
                return false;
            }
        }
        return true;
    }

    T minVal() const
    {
        T v = INFINITY;
        for (auto& c : coord) {
            v = fmin(v, c);
        }
        return v;
    }

    T maxVal() const
    {
        T v = -INFINITY;
        for (auto& c : coord) {
            v = fmax(v, c);
        }
        return v;
    }

    Vec& operator+=(const Vec& v)
    {
        for (int i = 0; i < COORDS; ++i) {
            coord[i] += v.coord[i];
        }
        return *this;
    }

    inline friend Vec operator+(Vec a, const Vec& b) { return a += b; }

    inline friend Vec operator+(T a, const Vec& b) { return Vec::ofConst(a) += b; }

    inline friend Vec operator+(Vec a, T b) { return a += Vec::ofConst(b); }

    Vec operator-() const
    {
        Vec v{};
        for (int i = 0; i < COORDS; ++i) {
            v[i] = -coord[i];
        }
        return v;
    }

    Vec& operator-=(const Vec& v)
    {
        for (int i = 0; i < COORDS; ++i) {
            coord[i] -= v.coord[i];
        }
        return *this;
    }

    inline friend Vec operator-(Vec a, const Vec& b) { return a -= b; }

    inline friend Vec operator-(T a, const Vec& b) { return Vec::ofConst(a) -= b; }

    inline friend Vec operator-(Vec a, T b) { return a -= Vec::ofConst(b); }

    Vec& operator*=(const Vec& v)
    {
        for (int i = 0; i < COORDS; ++i) {
            coord[i] *= v.coord[i];
        }
        return *this;
    }

    inline friend Vec operator*(Vec a, const Vec& b) { return a *= b; }

    inline friend Vec operator*(T a, const Vec& b) { return Vec::ofConst(a) *= b; }

    inline friend Vec operator*(Vec a, T b) { return a *= Vec::ofConst(b); }

    Vec& operator/=(const Vec& v)
    {
        for (int i = 0; i < COORDS; ++i) {
            coord[i] /= v.coord[i];
        }
        return *this;
    }

    inline friend Vec operator/(Vec a, const Vec& b) { return a /= b; }

    inline friend Vec operator/(T a, const Vec& b) { return Vec::ofConst(a) /= b; }

    inline friend Vec operator/(Vec a, T b) { return a /= Vec::ofConst(b); }

    friend bool operator==(const Vec& lhs, const Vec& rhs)
    {
        for (int i = 0; i < COORDS; ++i) {
            if (lhs[i] != rhs[i]) {
                return false;
            }
        }
        return true;
    }

    friend bool operator!=(const Vec& lhs, const Vec& rhs) { return !(rhs == lhs); }
};

template <typename T, typename TMin, typename TMax>
Vec<T> clampEach(Vec<T> val, TMin vMin, TMax vMax)
{
    for (int i = 0; i < COORDS; ++i) {
        val[i] = clamp(val[i], vMin, vMax);
    }
    return val;
}

using FVec = Vec<float>;
using IVec = Vec<int>;

enum class Mode : unsigned {
    Fast = 0,
    Normal = 1,
};

class Callbacks {
public:
    virtual ~Callbacks() = default;

    virtual void eol() = 0;

    virtual void homing() = 0;

    virtual void stop() = 0;

    virtual void setMode(Mode g) = 0;

    virtual void setSpeed(float val) = 0;

    virtual void move(const FVec& pos, bool report) = 0;

    virtual void reportCurrentPos() = 0;

    virtual void setSetting(unsigned s, float val, bool hasVal) = 0;

    virtual void showSetting(unsigned s) = 0;
	
    virtual void showSettings() = 0;

    virtual void servoId(unsigned command, int id, int val) = 0;

    virtual void help() = 0;

    virtual void error(GStr msg) = 0;

    virtual void errorPos(char c, int i) = 0;
};

class Parser {
public:
    Parser(Callbacks* cb) : cb_(cb) {}

    void parse(const char* str, int len)
    {
        c_ = str;
        pos_ = 0;
        len_ = len;
        while (pos_ < len_ && curr()) {
            if (!parseLine()) {
                cb_->errorPos(curr(), pos_);
				cb_->eol();
                skipLine();
            }
        }
    }

private:
    bool parseLine()
    {
        if (consume('?')) {
            cb_->reportCurrentPos();
        }else if (consume('!')) {
            cb_->stop();
        }
        else if (checkCoord()) {
            if (!parseMove()) {
                cb_->error(F("expect move"));
                return false;
            }
        }
        else if (consume('g')) {
            unsigned code = 0;
            if (!parseUnsigned(code)) {
                cb_->error(F("expect unsigned integer"));
                return false;
            }
            cb_->setMode(static_cast<Mode>(code));
            if (!parseMove()) {
                cb_->error(F("expect move"));
                return false;
            }
        }
        else if (consume('$')) {
            if (consume('$')) {
                cb_->showSettings();
            }
            else if (consume('h')) {
                cb_->homing();
            }
            else if (!parseSetSetting()) {
                cb_->error(F("expect set setting"));
                return false;
            }
        }
        else if (consume('%')) {
            if (consume('%')) {
                cb_->help();
            }
            else {
                unsigned cmd{};
                if (!parseUnsigned(cmd)) {
                    cb_->error(F("expect unsigned number"));
                    return false;
                }
                unsigned tmp{};
                int id = -1, val = -1;
                if (parseUnsigned(tmp)) {
                    id = tmp;
                    if (parseUnsigned(tmp)) {
                        val = tmp;
                    }
                }
                cb_->servoId(cmd, id, val);
            }
        }
        return requireEol();
    }

    bool parseMove()
    {
        float speed{};
        bool hasSpeed = false;
        if (checkSpeed()) {
            if (!parseSpeed(speed)) {
                cb_->error(F("expect convSpeed"));
                return false;
            }
            hasSpeed = true;
        }
        FVec pos = FVec::ofNaN();
        if (!parsePos(pos)) {
            cb_->error(F("expect position"));
            return false;
        }
        if (!hasSpeed && checkSpeed()) {
            if (!parseSpeed(speed)) {
                cb_->error(F("expect convSpeed"));
                return false;
            }
            hasSpeed = true;
        }
        bool report = false;
        if (pos.any() && consume('m')) {
            if (!consume('2', false)) {
                cb_->error(F("expect m2"));
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

    bool parseSetSetting()
    {
        unsigned s{};
        if (!parseUnsigned(s)) {
            cb_->error(F("expect setting number"));
            return false;
        }
        if (!consume('=')) {
            cb_->showSetting(s);
            return true;
        }
        float val{};
        bool hasVal = false;
        if (checkFloat()) {
            if (!parseFloat(val)) {
                cb_->error(F("expect floating point"));
                return false;
            }
            hasVal = true;
        }
        cb_->setSetting(s, val, hasVal);
        return true;
    }

    bool checkSpeed() { return check('f'); }

    bool parseSpeed(float& val)
    {
        if (!consume('f')) {
            return false;
        }
        if (!parseFloat(val)) {
            cb_->error(F("expect floating point after f"));
            return false;
        }
        return true;
    }

    bool checkCoord() const
    {
        int i{};
        return checkCoord(i);
    }

    bool checkCoord(int& i) const
    {
        for (i = 0; i < COORDS; ++i) {
            if (check(coordNames[i])) {
                return true;
            }
        }
        return false;
    }

    bool parsePos(FVec& pos)
    {
        int i{};
        while (checkCoord(i)) {
            if (!parseCoord(pos, i)) {
                return false;
            }
        }
        return true;
    }

    bool parseCoord(FVec& pos, int coord)
    {
        if (consume(coordNames[coord])) {
            if (!parseFloat(pos[coord])) {
                cb_->error(F("expect floating point"));
                return false;
            }
            return true;
        }
        return false;
    }

    bool checkFloat() { return check('-') || check('.') || isDigit(); }

    bool parseFloat(float& val)
    {
        if (!checkFloat()) {
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
                    cb_->error(F("unexpected dot"));
                    return false;
                }
                isFraction = true;
            }
            else if (consumeDigit(digit)) {
                value = value * 10 + digit;
                if (isFraction) {
                    fraction *= 0.1f;
                }
            }
            else {
                cb_->error(F("expect digit or fraction separator"));
                return false;
            }
        } while (isDigit() || (check('.') && !isFraction));

        if (isNegative) {
            value = -value;
        }
        if (isFraction) {
            val = value * fraction;
        }
        else {
            val = value;
        }
        skip();
        return true;
    }

    bool parseUnsigned(unsigned& i)
    {
        unsigned curr = 0;
        if (!consumeDigit(curr)) {
            return false;
        }
        i = 0;
        do {
            i = curr + i * 10;
        } while (consumeDigit(curr));
        skip();
        return true;
    }

    bool isDigit() const { return curr() >= '0' && curr() <= '9'; }

    bool consumeDigit(unsigned& digit)
    {
        if (isDigit()) {
            digit = static_cast<unsigned>(curr() - '0');
            next(false);
            return true;
        }
        return false;
    }

    void skipLine()
    {
        while (curr() && !(check('\n') || check('\r'))) {
            next(false);
        }
        consume('\r', false);
		consume('\n', false);
    }

    void skip()
    {
        while (check(' ')) {
            ++pos_;
        }
    }

    void next(bool skipAfter = true)
    {
        do {
            ++pos_;
        } while (skipAfter && check(' '));
    }

    char curr() const { return c_[pos_]; }

    bool check(char c) const { return curr() == c; }

    bool consume(char c, bool skipAfter = true)
    {
        if (check(c)) {
            next(skipAfter);
            return true;
        }
        return false;
    }

    bool requireEol()
    {
		consume('\r');
        if (!consume('\n')) {
            cb_->error(F("expect end of line"));
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
