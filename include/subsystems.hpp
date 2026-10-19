#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern ez::Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intake(10);
inline pros::Motor liftf(4);
inline pros::Motor liftb(3);
// inline pros::adi::DigitalIn limit_switch('A');
