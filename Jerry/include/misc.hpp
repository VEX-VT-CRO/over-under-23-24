#ifndef MISC_HPP
#define MISC_HPP

struct Coordinate
{
    double x;
    double y;
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

#endif