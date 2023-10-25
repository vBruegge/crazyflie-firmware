#ifndef _GRABBER_H_
#define _GRABBER_H_

#include "deck_core.h"

enum grabberStates {
    IDLE = 0,
    RDY2LAND = 1,
    ACTIVATING_GRABBER = 2,
    LANDED = 3,
    RELEASING_GRABBER = 4,
};

void grabberInit(DeckInfo* info);

bool grabberTest(void);
void grabberTask(void* arg);

#endif /* _GRABBER_H_ */