/*
 * vcp.h
 *
 *  Created on: 1 févr. 2026
 *      Author: Jonathan Marois
 */

#ifndef MODULES_INC_VCP_H_
#define MODULES_INC_VCP_H_

#include <stdint.h>

uint8_t VCP_IsReady(void);
int8_t VCP_SendStringBlocking(const char *str); // 0=OK, 1=not ready(timeout), 2=tx err/timeout, 3=invalid


#endif /* MODULES_INC_VCP_H_ */
