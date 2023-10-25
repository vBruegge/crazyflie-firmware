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
#include "motors.h"
#include "servo_utility.h"
#include "crtp_commander.h"

static bool isInit;
static const MotorPerifDef** servoMap = &motorMapArbitraryServo;
const deckPin_t* mosfetPin = &DECK_GPIO_IO1;
const deckPin_t* activateGrabberPin = &DECK_GPIO_IO2;

void grabberInit(DeckInfo* info)
{
  if (isInit)
    return;

  xTaskCreate(grabberTask, GRABBER_TASK_NAME, GRABBER_TASK_STACKSIZE, NULL, GRABBER_TASK_PRI, NULL);
  servoInit(*servoMap);
  if(servoTest(*servoMap))
    return;
  pinMode(*activateGrabberPin, INPUT);
  pinMode(*mosfetPin, OUTPUT);

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

//TODO: add test if throttle != 0
//TODO: send zero throttle when landed?
void grabberTask(void* arg)
{
  systemWaitStart();
  int grabberState = RDY2LAND;
  bool disengageGrabber = false;
  digitalWrite(*mosfetPin, LOW);
  const int actuationTime = 200;
  TickType_t startServo =  xTaskGetTickCount();

  while (1) {
    if(digitalRead(*activateGrabberPin) && grabberState == RDY2LAND) {

      startServo = xTaskGetTickCount();
      digitalWrite(*mosfetPin, HIGH);
      servoSetRatio(0);
      grabberState = ACTIVATING_GRABBER;
    }
    else if((grabberState == ACTIVATING_GRABBER || grabberState == RELEASING_GRABBER)
                    && xTaskGetTickCount() > startServo + M2T(actuationTime)) {
                      
      digitalWrite(*mosfetPin, LOW);
      if(grabberState == ACTIVATING_GRABBER)
        grabberState = LANDED;
      else
        grabberState = RDY2LAND;
    }
    else if(disengageGrabber && grabberState == LANDED) {
      startServo = xTaskGetTickCount();
      digitalWrite(*mosfetPin, HIGH);

      servoSetRatio(70);
      grabberState = RELEASING_GRABBER;
    }
    disengageGrabber = getGrabberStatus();
  }
}

static const DeckDriver grabber_deck = {
  .vid = 0,
  .pid = 0,
  .name = "grabber",

  //TODO: add GPIO pins
  .usedGpio = DECK_USING_IO_2 | DECK_USING_IO_1,
  
  .init = grabberInit,
  .test = grabberTest,
};

DECK_DRIVER(grabber_deck);

/*

PARAM_GROUP_START(deck)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, Grabber, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(grabber)
LOG_ADD(LOG_FLOAT, vbat, &vbat)
LOG_ADD(LOG_FLOAT, i_raw, &current_last)
LOG_ADD(LOG_FLOAT, current, &current)
LOG_ADD(LOG_FLOAT, power, &power)*/
//LOG_GROUP_STOP(grabber)

/**
 *
 * Current sensor parameters
 
PARAM_GROUP_START(grabber)

 * @brief Current sensor constant (A/V)
 */
//PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, ampsPerVolt, &ampsPerVolt)
/**
 * @brief Current filter parameter <0; 1), set 0 to disable, 0.9999 for max effect 
 */
//PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, filtAlpha, &filter_alpha)

//PARAM_GROUP_STOP(grabber)