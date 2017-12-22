#pragma once

namespace gservo {

/*
 Application:
$H                       | homing
g0 x%.2f y%.2f M2        | generic movement
g1 x%.2f y%.2f f%.2f M2  | generic movement with given speed
g0 x%.2f M2              | x axis only movement
?                        | ask current position
$1=255                   | enable retention
$1=15                    | disable retention

%.2f is a number placeholder

Manual (through terminal only, in addition to commands above):
g0 y                     |
g0 x                     |
$27                      | set homing angle
$110=                    | speed
$111=                    | speed
$120=                    | acceleration
$121=                    | acceleration
$$                       | show setting
 * */

class Parser {
};

} // namespace gservo
