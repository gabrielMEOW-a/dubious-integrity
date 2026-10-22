#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern ez::Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intake(10);
inline pros::Motor liftf(4);
inline pros::Motor liftb(3);
inline pros::Rotation arm(-2);
inline ez::Piston claw('A');

inline ez::PID cascadeLift{0.5,0.0,3,30,"Lift"};
inline ez::PID armSpin{3,0,10,0,"Spinny spinny"};
// inline pros::adi::DigitalIn limit_switch('A');
