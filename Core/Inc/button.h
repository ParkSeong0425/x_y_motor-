#ifndef BUTTON_H
#define BUTTON_H

#include "main.h"

extern volatile int run;
extern volatile int pause;
extern volatile int estop;

void button_init(void);
void button_run(void);

int button_home_x(int *out);
int button_home_y(int *out);
int button_save_x(int n);
int button_save_y(int n);

#endif
