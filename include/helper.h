/*
 * helper.h
 *
 *  Created on: 06.01.2017
 *      Author: sefo
 */

#ifndef INCLUDE_HELPER_H_
#define INCLUDE_HELPER_H_

#include <stddef.h>

#define container_of(ptr, type, member) ({			\
	const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
	(type *)( (char *)__mptr - offsetof(type,member) );})


#endif /* INCLUDE_HELPER_H_ */
