#include "../parser.h"

#include "catch.hpp"

#include <sstream>
#include <string>

namespace gservo {
namespace tests {
using namespace Catch;

class StrCb : public Callbacks {
public:
    StrCb() { ss_.setf(std::ios_base::boolalpha); }

    std::string str() const { return ss_.str(); }

    void error(const char* msg) override { ss_ << "err " << msg << ";"; }

    void errorPos(char c, int i) override
    {
        ss_ << " '" << c << "' at " << i << ";";
    }

    void eol() override { ss_ << "eol;"; }

    void homing() override { ss_ << "homing;"; }

    void setMode(Mode g) override
    {
        ss_ << "g " << static_cast<int>(g) << ";";
    }

    void setSpeed(float val) override { ss_ << "convSpeed " << val << ";"; }

    void move(const FVec& p, bool report) override
    {
        ss_ << "move ";
        for (int i = 0; i < COORDS; ++i) {
            ss_ << p[i] << ", " << p.has(i) << ", ";
        }
        ss_ << report << ";";
    }

    void reportCurrentPos() override { ss_ << "curr convPos;"; }

    void setSetting(Setting s, float val, bool hasVal) override
    {
        ss_ << "s " << static_cast<int>(s) << ", " << val << ", " << hasVal
            << ";";
    }

    void showSettings() override { ss_ << "s show;"; }

    void servoId(unsigned i) override { ss_ << "servoId " << i << ";"; }

    void help() override { ss_ << "help;"; }

private:
    std::stringstream ss_;
};

std::string parse(const std::string& str)
{
    StrCb cb;
    Parser p{&cb};
    p.parse(str.c_str(), static_cast<int>(str.length()));
    return cb.str();
}

TEST_CASE("Parser")
{
    CHECK_THAT(parse(""), Equals(""));
    CHECK_THAT(parse("%%\n"), Equals("help;eol;"));
    CHECK_THAT(parse("%0 123\n"), Equals("servoId 123;eol;"));
    CHECK_THAT(parse("\n"), Equals("eol;"));
    CHECK_THAT(parse("?\n"), Equals("curr convPos;eol;"));
    CHECK_THAT(parse("$$\n"), Equals("s show;eol;"));
    CHECK_THAT(parse("$h\n"), Equals("homing;eol;"));
    CHECK_THAT(parse("$110=\n"), Equals("s 110, 0, false;eol;"));
    CHECK_THAT(parse("$110=12.3\n"), Equals("s 110, 12.3, true;eol;"));
    CHECK_THAT(parse("$111=1.\n"), Equals("s 111, 1, true;eol;"));
    CHECK_THAT(parse("$140=.3\n"), Equals("s 140, 0.3, true;eol;"));
    CHECK_THAT(parse("$141=98\n"), Equals("s 141, 98, true;eol;"));
    CHECK_THAT(parse("x1y2\n"), Equals("move 1, true, 2, true, false;eol;"));
    CHECK_THAT(parse("y1.2x3.4\n"),
               Equals("move 3.4, true, 1.2, true, false;eol;"));
    CHECK_THAT(parse("x 10 y 20 m2\n"),
               Equals("move 10, true, 20, true, true;eol;"));
    CHECK_THAT(parse("x 10\n"),
               Equals("move 10, true, nan, false, false;eol;"));
    CHECK_THAT(parse("y 10\n"),
               Equals("move nan, false, 10, true, false;eol;"));
    CHECK_THAT(parse("y 10 m2\n"),
               Equals("move nan, false, 10, true, true;eol;"));
    CHECK_THAT(parse("g0\n"), Equals("g 0;eol;"));
    CHECK_THAT(parse("g1\n"), Equals("g 1;eol;"));
    CHECK_THAT(parse("g1 f10\n"), Equals("g 1;convSpeed 10;eol;"));
    CHECK_THAT(parse("g0 x0 y0 m2\n"),
               Equals("g 0;move 0, true, 0, true, true;eol;"));
    CHECK_THAT(parse("g1 x0 y0 f1000 m2\n"),
               Equals("g 1;convSpeed 1000;move 0, true, 0, true, true;eol;"));
    CHECK_THAT(parse("g1 x0 f1000\n"
                     "g1y20f10\n"
                     "g1 x 100 y 200 f 1 m2\n"),
               Equals("g 1;convSpeed 1000;move 0, true, nan, false, false;eol;"
                      "g 1;convSpeed 10;move nan, false, 20, true, false;eol;"
                      "g 1;convSpeed 1;move 100, true, 200, true, true;eol;"));
}
} // namespace tests
} // namespace gservo
