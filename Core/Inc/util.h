/*
 * flash.h
 *
 *  Created on: Feb 23, 2023
 *      Author: bockn
 */

#ifndef INC_UTIL_H_
#define INC_UTIL_H_

#include <stdio.h>

uint8_t calc_checksum(const uint8_t *data, size_t len);

#endif /* INC_UTIL_H_ */
