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
* grabber.c: grabber deck driver
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
#include "log.h"
#include "stabilizer.h"

static bool isInit;
static const MotorPerifDef** servoMap = &motorMapArbitraryServo;
const deckPin_t* mosfetPin = &DECK_GPIO_IO1;
const deckPin_t* activateGrabberPin = &DECK_GPIO_IO2;
static int grabberState = IDLE;
static bool disengageGrabber = false;

void grabberInit(DeckInfo* info)
{
  if (isInit)
    return;

  xTaskCreate(grabberTask, GRABBER_TASK_NAME, GRABBER_TASK_STACKSIZE, NULL, GRABBER_TASK_PRI, NULL);
  servoInit(*servoMap);
  if(!servoTest())
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
//TODO: add MOSFET
void grabberTask(void* arg)
{
  grabberState = IDLE;
  systemWaitStart();
  //wait for take-off
  while(getThrust() < 0.1f) {
    continue;
  }
  digitalWrite(*mosfetPin, HIGH);
  const int servoPos = 60;
  servoSetRatio(servoPos);

  const int actuationTime = 150; //TODO: define delay
  TickType_t startServo =  xTaskGetTickCount();
  while(xTaskGetTickCount() < startServo + M2T(actuationTime)) {
    continue;
  }
  DEBUG_PRINT("Servo at initial position\n");
  grabberState = RDY2LAND;
  digitalWrite(*mosfetPin, LOW);
  DEBUG_PRINT("Grabbber ready to land!\n");

  while (1) {
    if(digitalRead(*activateGrabberPin) && grabberState == RDY2LAND) {

      startServo = xTaskGetTickCount();
      digitalWrite(*mosfetPin, HIGH);
      servoSetRatio(0);
      grabberState = ACTIVATING_GRABBER;
      DEBUG_PRINT("Activated grabber!\n");
    }
    else if((grabberState == ACTIVATING_GRABBER || grabberState == RELEASING_GRABBER)
                    && xTaskGetTickCount() > startServo + M2T(actuationTime)) {
                      
      digitalWrite(*mosfetPin, LOW);
      if(grabberState == ACTIVATING_GRABBER) {
        grabberState = LANDED;
        resetThrust(); //stop flapping
        DEBUG_PRINT("Flapper landed!\n");
      }
      else {
        grabberState = RDY2LAND;
        DEBUG_PRINT("Grabbber ready to land!\n");
      }
    }
    else if(disengageGrabber && grabberState == LANDED && getThrust() > 0.1f) {
      startServo = xTaskGetTickCount();
      digitalWrite(*mosfetPin, HIGH);

      servoSetRatio(servoPos);
      grabberState = RELEASING_GRABBER;
      DEBUG_PRINT("Releasing grabber!\n");
    }
    disengageGrabber = getGrabberStatus();
  }
}

static const DeckDriver grabber_deck = {
  .vid = 0x00,
  .pid = 0x00,
  .name = "bcGrabber",

  //TODO: add GPIO pins
  .usedGpio = DECK_USING_IO_2 | DECK_USING_IO_1,
  .init = grabberInit,
  .test = grabberTest,
};

DECK_DRIVER(grabber_deck);


LOG_GROUP_START(grabber)
LOG_ADD(LOG_INT16, grabberState, &grabberState)
LOG_ADD(LOG_UINT8, disengageGrabber, &disengageGrabber)
LOG_GROUP_STOP(grabber)
