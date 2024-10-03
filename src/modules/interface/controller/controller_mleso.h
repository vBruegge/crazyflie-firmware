/**
 *    ||          ____  _ __  ______
 * +------+      / __ )(_) /_/ ____/_________ _____  ___
 * | 0xBC |     / __  / / __/ /    / ___/ __ `/_  / / _	\
 * +------+    / /_/ / / /_/ /___ / /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\____//_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * mleso_controller.h: mleso controller
 */

#ifndef MLESO_CONTROLLER_H_
#define MLESO_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>


void mlesoControllerInit(const float updateDt);
bool mlesoControllerTest(void);

/**
 * Make the controller run an update of the mleso PID. The output is
 * the desired rate which should be fed into a rate controller. The
 * mleso controller can be run in a slower update rate then the rate
 * controller.
 */
void mlesoControllerCorrectAttitudePID(
       float eulerRollActual, float eulerPitchActual, float eulerYawActual,
       float eulerRollDesired, float eulerPitchDesired, float eulerYawDesired,
       float* rollRateDesired, float* pitchRateDesired, float* yawRateDesired);

/**
 * Make the controller run an update of the rate PID. The output is
 * the actuator force.
 */
void mlesoControllerCorrectRatePID(
       float rollRateActual, float pitchRateActual, float yawRateActual,
       float rollRateDesired, float pitchRateDesired, float yawRateDesired);

/**
 * Reset controller roll mleso PID
 */
void mlesoControllerResetRollAttitudePID(float rollActual);

/**
 * Reset controller pitch mleso PID
 */
void mlesoControllerResetPitchAttitudePID(float pitchActual);

/**
 * Reset controller roll, pitch and yaw PID's.
 */
void mlesoControllerResetAllPID(float rollActual, float pitchActual, float yawActual);

/**
 * Get the actuator output.
 */
void mlesoControllerGetActuatorOutput(int16_t* roll, int16_t* pitch, int16_t* yaw);

/**
 * Get yaw max delta
 */
float mlesoControllerGetYawMaxDelta(void);


#endif /* MLESO_CONTROLLER_H_ */
