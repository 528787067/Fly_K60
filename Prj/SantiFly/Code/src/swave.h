#ifndef _SWAVE_H_
#define _SWAVE_H_

#include "include.h"
#include "control.h"

#define SWAVE_WORK	0		// �������ó������Ƿ���

#define TRIQ		PTB1
#define ECHO		PTB0

void swaveInit();
int getSwaveDistance();

#endif