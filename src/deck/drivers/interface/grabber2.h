#ifndef _GRABBER2_H_
#define _GRABBER2_H_

#include "deck_core.h"

enum grabberStates {
    IDLE = 0,
    RDY2LAND = 1,
    ACTIVATING_GRABBER = 2,
    LANDED = 3,
    ACTIVATING_GRABBER_LANDED = 4,
    RELEASING_GRABBER = 5,
};

void grabber2Init(DeckInfo* info);

bool grabber2Test(void);
void grabber2Task(void* arg);

#endif /* _GRABBER2_H_ */