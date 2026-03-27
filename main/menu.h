#ifndef MENU_H
#define MENU_H

#include <stdbool.h>
#include "main.h"

void menuStart(void);
const synchronism* menuGetSelectedSynchronism(void);
int menuGetRPM(void);
bool menuIsGeneratingSignal(void);

#endif // MENU_H
