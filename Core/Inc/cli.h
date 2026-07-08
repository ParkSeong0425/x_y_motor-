#ifndef CLI_H
#define CLI_H

#include "main.h"

void cli_run(void);
int cli_go(int col, int dan, int xrpm, int yrpm);
int cli_go_home(void);
void cli_set_x(int n, int pos);
void cli_set_y(int n, int pos);
int cli_getc(uint8_t *c);
int cli_homed(void);

#endif
