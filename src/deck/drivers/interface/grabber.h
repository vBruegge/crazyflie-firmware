#ifndef _GRABBER_H_
#define _GRABBER_H_

#include "deck_core.h"

void grabberInit(DeckInfo* info);

bool grabberTest(void);
void grabberTask(void* arg);

#endif /* _GRABBER_H_ */