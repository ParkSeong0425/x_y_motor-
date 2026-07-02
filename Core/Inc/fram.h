/*
 * fram.h
 *
 *  Created on: Jul 2, 2026
 *      Author: HWNOT
 */

#ifndef FRAM_H
#define FRAM_H

#include "main.h"

void fram_read(uint16_t addr, void *buf, uint16_t len);
void fram_write(uint16_t addr, void *buf, uint16_t len);

#endif
