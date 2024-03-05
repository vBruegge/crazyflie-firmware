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

#define DEBUG_MODULE "GRABBER2_DEBUG"

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "config.h"
#include "grabber2.h"
#include "crtp_commander.h"
#include "log.h"
#include "stabilizer.h"

#define ACTIVATION_PERIOD_MS 60*1e3
#define REACTIVATION_TIME_MS 500
#define ACTIVATION_TIME_MS 800

static bool isInit;
const deckPin_t* engageGrabberPin = &DECK_GPIO_IO2;
const deckPin_t* disengageGrabberPin = &DECK_GPIO_IO1;
static int grabberState = IDLE;
static bool activateGrabber = false;

void grabber2Init(DeckInfo* info)
{
  if (isInit)
    return;

  xTaskCreate(grabber2Task, GRABBER2_TASK_NAME, GRABBER2_TASK_STACKSIZE, NULL, GRABBER2_TASK_PRI, NULL);
  
  pinMode(*engageGrabberPin, OUTPUT);
  pinMode(*disengageGrabberPin, OUTPUT);

  isInit = true;
}

bool grabber2Test(void)
{
  bool testStatus = true;

  if (!isInit)
    return false;

  return testStatus;
}

//TODO: add test if throttle != 0
//TODO: send zero throttle when landed?
void grabber2Task(void* arg)
{
    grabberState = IDLE;

    //turn of Mosfet
    digitalWrite(*engageGrabberPin, LOW);
    digitalWrite(*disengageGrabberPin, LOW);

    systemWaitStart();
    //wait for take-off
    while(getThrust() < 0.1f) {
      continue;
    }
    digitalWrite(*disengageGrabberPin, HIGH);
    TickType_t activation = xTaskGetTickCount();
    while(xTaskGetTickCount() < activation + M2T(ACTIVATION_TIME_MS)) {
      continue;
    }

    grabberState = RDY2LAND;
    DEBUG_PRINT("Grabbber ready to land!\n");

    while (1) {
      activateGrabber = getGrabberStatus();
      switch(grabberState) {
        case RDY2LAND:
          if(activateGrabber) {
            activation = xTaskGetTickCount();
            digitalWrite(*engageGrabberPin, HIGH);
            DEBUG_PRINT("Activating grabber!\n");
            grabberState = ACTIVATING_GRABBER;
          }
          break;
        case ACTIVATING_GRABBER:
          if(xTaskGetTickCount() > activation + M2T(ACTIVATION_TIME_MS)) {
            digitalWrite(*engageGrabberPin, LOW);
            DEBUG_PRINT("Flapper landed!\n");
            activation = xTaskGetTickCount();
            grabberState = LANDED;
          }
          break;
        case LANDED:
          if(activateGrabber && getThrust() > 0.1f) {
              digitalWrite(*engageGrabberPin, LOW);
              digitalWrite(*disengageGrabberPin, HIGH);
              activation = xTaskGetTickCount();
              DEBUG_PRINT("Releasing grabber!\n");
              grabberState = RELEASING_GRABBER;
          }
          else if(xTaskGetTickCount() > activation + M2T(ACTIVATION_PERIOD_MS)) {
            digitalWrite(*engageGrabberPin, HIGH);
            DEBUG_PRINT("Activating grabber!\n");
            grabberState = ACTIVATING_GRABBER_LANDED;
            activation = xTaskGetTickCount();
          }
          break;
        case RELEASING_GRABBER:
          if(xTaskGetTickCount() > activation + M2T(ACTIVATION_TIME_MS)) {
              grabberState = RDY2LAND;
              DEBUG_PRINT("Grabbber ready to land!\n");
              digitalWrite(*disengageGrabberPin, LOW);
          }
          break;
        case ACTIVATING_GRABBER_LANDED:
          if (xTaskGetTickCount() > activation + M2T(REACTIVATION_TIME_MS)) {
            digitalWrite(*engageGrabberPin, LOW);
            activation = xTaskGetTickCount();
            grabberState = LANDED;
          }
          break;
        default:
          break;
      }
    }
}

static const DeckDriver grabber2_deck = {
  .vid = 0x00,
  .pid = 0x00,
  .name = "bcGrabber2",

  //TODO: add GPIO pins
  .usedGpio = DECK_USING_IO_2 | DECK_USING_IO_1,

  .init = grabber2Init,
  .test = grabber2Test,
};

DECK_DRIVER(grabber2_deck);


LOG_GROUP_START(grabber2)
LOG_ADD(LOG_INT16, grabberState, &grabberState)
LOG_ADD(LOG_UINT8, activateGrabber, &activateGrabber)
LOG_GROUP_STOP(grabber2)
