#ifndef _SERVO_GRIPPER_H
#define _SERVO_GRIPPER_H

#include "deck_core.h"

#ifdef CONFIG_PLATFORM_SERVO_GRIPPER
enum gripperStates {
    IDLE = 0,
    RDY2LAND = 1,
    ACTIVATING_GRIPPER = 2,
    LANDED = 3,
    RELEASING_GRIPPER = 4,
};
#endif

void servoGripperInit();

bool servoGripperTest(void);
void servoGripperTask(void* arg);

#endif /* _SERVO_GRIPPER_H */