#pragma once

#include <cstdint>

namespace gservo {

/*
 Application:
$H                       | homing to zero position
M2                       | report position after move ends
g0 x%.2f y%.2f M2        | generic movement
g1 x%.2f y%.2f f%.2f M2  | generic movement with given speed
g0 x%.2f M2              | x axis only movement
?                        | ask current position

$110=                    | set speed x
$111=                    | set speed y
$120=                    | set acceleration x
$121=                    | set acceleration y
$140=                    | set zero position x
$141=                    | set zero position y
$$                       | show setting
 * */

struct Pos {
    float x, y;
    bool hasX, hasY;
};

template <typename Src, typename Callbacks>
class Parser {
public:
    Parser(Src* src, Callbacks* cb) : src_(src), cb_(cb) {}

    void run()
    {
        next();
        while (true) {
            parse();
        }
    }

private:
    bool parse()
    {
        skip();
        if (maybe('?')) {
            reportCurrentPos();
            skip();
            return requireEol();
        }
        if (check('x') || check('y')) {
            skip();
            return parseMove(0);
        }
        if (maybe('g')) {
            skip();
            const int g = parseInt();
            skip();
            return parseMove(g);
        }
        if (maybe('$')) {
            if (maybe('$')) {
                showSettings();
                skip();
                return requireEol();
            }
            if (maybe('H')) {
                homing();
                skip();
                return requireEol();
            }
            skip();
            const int s = parseInt();
            skip();
            if (!require('=')) {
                return false;
            }
            skip();
            float val{};
            if (parseFloat(val)) {
                setSetting(s, val);
            }
            else {
                clearSetting(s);
            }
            skip();
            return requireEol();
        }
        return requireEol();
    }

    void setSetting(const int s, float val) {}

    void clearSetting(const int s) {}

    bool parseMove(int g) { return false; }

    int parseInt() { return 0; }

    bool parseFloat(float&) { return 0; }

    char next()
    {
        c_ = getChar();
        return c_;
    }

    bool check(char c) { return c_ == c; }

    bool maybe(char c)
    {
        if (c_ == c) {
            next();
            return true;
        }
        return false;
    }

    bool require(char c)
    {
        if (maybe(c)) {
            return true;
        }
        error("Unexpected character");
        return false;
    }

    bool requireEol() { return require('\n'); }

    void skip(char c = ' ')
    {
        while (c_ == c) {
            next();
        }
    }

    char getChar() { return src_->getChar(); }

    void error(const char* msg) { cb_->error(msg); }

    void eol() { cb_->eol(); }

    void homing() { cb_->homing(); }

    void fastMovement(const Pos& pos) { cb_->fastMovement(pos); }

    void movement(const Pos& pos) { cb_->movement(pos); }

    void reportCurrentPos() { cb_->reportCurrentPos(); }

    void setSpeed(const Pos& pos) { cb_->setSpeed(); }

    void setAcceleration(const Pos& pos) { cb_->setAcceleration(); }

    void showSettings() { cb_->showSettings(); }

    Src* src_{};
    Callbacks* cb_{};
    char c_{};
};

} // namespace gservo
