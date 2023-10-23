/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
* Copyright (C) 2022 Bitcraze AB & Flapper Drones (https://flapper-drones.com)
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
 * grabber.c: Flapper Nimble+ PCB driver
 */

#define DEBUG_MODULE "GRABBER_DEBUG"

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "config.h"
#include "grabber.h"
#include "servo_utility.c"

static bool isInit;

void grabberInit(DeckInfo* info)
{
  if (isInit)
    return;

  xTaskCreate(grabberTask, GRABBER_TASK_NAME, GRABBER_TASK_STACKSIZE, NULL, GRABBER_TASK_PRI, NULL);
  MotorPeriDef servoMap = MOTORS_PB4_TIM3_CH1_BRUSHLESS_PP;
  servoInit(&servoMap);
  if(servoTest())
    return;

  isInit = true;
}

bool grabberTest(void)
{
  bool testStatus;
  testStatus = true;

  if (!isInit)
    return false;

  return testStatus;
}

//TODO: add Krispin's code
void grabberTask(void* arg)
{
  /*systemWaitStart();
  TickType_t xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, M2T(1));

    reading_last = analogReadVoltage(DECK_GPIO_TX2);
    current_last = reading_last*ampsPerVolt;

    // simple low pass filter
    current = filter_alpha*current + (1.0f - filter_alpha)*current_last;

    #ifdef CONFIG_DECK_FLAPPER_MEASURE_VBAT_ON_PA3
    vbat = filter_alpha*vbat + (1.0f - filter_alpha)*pmMeasureExtBatteryVoltage();
    #else
    vbat = pmGetBatteryVoltage();

    #endif
    power = vbat * current;
  }*/
}

static const DeckDriver grabber_deck = {
  .vid = 0,
  .pid = 0,
  .name = "grabber",

  //TODO: add GPIO pins
  .usedGpio = DECK_USING_,
  
  .init = grabberInit,
  .test = grabberTest,
};

DECK_DRIVER(grabber_deck);



PARAM_GROUP_START(deck)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, Grabber, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(grabber)
/*LOG_ADD(LOG_FLOAT, vbat, &vbat)
LOG_ADD(LOG_FLOAT, i_raw, &current_last)
LOG_ADD(LOG_FLOAT, current, &current)
LOG_ADD(LOG_FLOAT, power, &power)*/
LOG_GROUP_STOP(grabber)

/**
 *
 * Current sensor parameters
 */
PARAM_GROUP_START(grabber)
/**
 * @brief Current sensor constant (A/V)
 */
//PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, ampsPerVolt, &ampsPerVolt)
/**
 * @brief Current filter parameter <0; 1), set 0 to disable, 0.9999 for max effect 
 */
//PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, filtAlpha, &filter_alpha)

PARAM_GROUP_STOP(grabber)