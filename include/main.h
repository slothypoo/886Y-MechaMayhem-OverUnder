/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * Copyright (c) 2017-2022, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#define PROS_USE_SIMPLE_NAMES

#define PROS_USE_LITERALS

#include "api.h"
#include "chassis.hpp"

using namespace pros;
using namespace std;


#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers hereƒß
 */
//#include <iostream>
#endif

extern pros::Motor lF;
extern pros::Motor rF;
extern pros::Motor lB;
extern pros::Motor rB;
extern pros::Motor lM;
extern pros::Motor rM;
extern pros::Motor intake;
extern pros::Motor hang;

extern pros::Controller master;
extern pros::ADIDigitalOut frontWings;
extern pros::ADIDigitalOut backWings;
extern pros::ADIDigitalOut ratchet;



extern Imu imu;


#endif  // _PROS_MAIN_H_