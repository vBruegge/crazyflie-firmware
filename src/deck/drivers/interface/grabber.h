#ifndef _GRABBER_H_
#define _GRABBER_H_

#include "deck_core.h"

enum grabberStates {
    IDLE = 0,
    RDY2LAND = 1,
    LANDED = 2,
};

void grabberInit(DeckInfo* info);

bool grabberTest(void);
void grabberTask(void* arg);

#endif /* _GRABBER_H_ */