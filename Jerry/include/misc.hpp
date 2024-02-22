#ifndef MISC_HPP
#define MISC_HPP

struct Coordinate
{
    double x;
    double y;
    double z;
};
typedef struct Coordinate Coordinate;

struct PIDConstants
{
    double P;
    double I;
    double D;
};
typedef struct PIDConstants PIDConstants;

enum TeamColor
{
    Red,
    Blue
};

constexpr double PI = 3.141592653589793;
constexpr double DEG2RAD = PI / 180;

#endif