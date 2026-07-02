/*
 * button.h
 *
 *  Created on: Jul 2, 2026
 *      Author: HWNOT
 */
#ifndef BUTTON_H
#define BUTTON_H

#include "main.h"

extern volatile int run;     // 자동 왕복 동작 중
extern volatile int pause;   // 일시 정지
extern volatile int estop;   // 비상 정지

void button_init(void);
void button_run(void);

#endif
