/*
 * debug.h
 *
 *  Created on: 08.02.2017
 *      Author: sefo
 */

#ifndef DEBUG_H_
#define DEBUG_H_

//#define DEBUG

#ifdef DEBUG
#define debug(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define debug(fmt, ...) NULL
#endif

#endif /* DEBUG_H_ */
