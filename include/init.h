/*
 * init.h
 *
 *  Created on: 06.01.2017
 *      Author: sefo
 */

#ifndef INCLUDE_INIT_H_
#define INCLUDE_INIT_H_


typedef int (*initcall_t)(void);

void do_initcalls(void);

#define early_initcall(_fn_) \
	static __attribute__((section(".initcall.1"), used)) initcall_t func_ptr_##_fn = _fn_

#define module_init(_fn_) \
	static __attribute__((section(".initcall.4"), used)) initcall_t func_ptr_##_fn = _fn_


#endif /* INCLUDE_INIT_H_ */
