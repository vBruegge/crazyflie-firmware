/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 * controller_mleso.h - MLESO Controller Interface
 */
#ifndef __CONTROLLER_MLESO_H__
#define __CONTROLLER_MLESO_H__

#include "stabilizer_types.h"
#include <Eigen/Core>

void controllerMLESOInit(void);
bool controllerMLESOTest(void);
void controllerMLESO(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep);
void getStateFeedback(Eigen::Vector4f& stateFeedback, const Eigen::VectorXf& state);
void convertFromMotorCmd2ActCmd(Eigen::Vector4f& act_cmd);
void convertFromActCmd2MotorCmd(Eigen::Vector4f& act_cmd);
Eigen::Vector4f mlesoModel(Eigen::Vector4f& act_cmd, Eigen::VectorXf stateP);

/**
 * Reset controller roll feedback PID
 */
void mlesoControllerResetRollFeedbackPID(void);

/**
 * Reset controller pitch Feedback PID
 */
void mlesoControllerResetPitchFeedbackPID(void);

/**
 * Reset controller roll, pitch and yaw PID's.
 */
void mlesoControllerResetAllPID(void);


#endif //__CONTROLLER_MLESO_H__
