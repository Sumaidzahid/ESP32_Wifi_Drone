#ifndef RECEIVER_H
#define RECEIVER_H

#include "config.h"
#include "motors.h"

extern unsigned long lastCommandTime;

void parseUDP(char* data);
void goForward();
void goforward();
void goBack();
void goback();
void goLeft();
void goleft();
void goRight();
void goright();
void rotateRight();
void rotateLeft();
void rotateright();
void rotateleft();
void goUp();
void goDown();

#endif
