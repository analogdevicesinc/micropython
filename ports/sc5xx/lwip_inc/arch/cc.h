/*
 * cc.h
 * Contains compiler-specific definitions and typedefs for lwIP on ADSP-SC5xx processors.
 */
#ifndef _ARCH_CC_H_
#define _ARCH_CC_H_

// lwIP requires memset
#include <string.h>

// Define platform endianness
#undef BYTE_ORDER
#define BYTE_ORDER (LITTLE_ENDIAN)

// Compiler hints for packing structures
// (Note: this packs the struct layouts but doesn't modify compiler
//        access to the members - we're on our own for avoiding
//        alignment exceptions.)
#define PACK_STRUCT_FIELD(x) x
#define PACK_STRUCT_STRUCT
#define PACK_STRUCT_BEGIN _Pragma("pack(1)")
#define PACK_STRUCT_END   _Pragma("pack()")

#define LWIP_RAND()   rand()

// prototypes for printf(), fflush() and abort()
#include <stdio.h>
#include <stdlib.h>

// Plaform specific diagnostic output
#ifdef LWIP_DEBUG
#define LWIP_PLATFORM_DIAG(x)	\
  do {printf x;} while(0)
#else
#define LWIP_PLATFORM_DIAG(x)
#endif

#ifndef LWIP_NOASSERT
#define LWIP_PLATFORM_ASSERT(x) \
  do {printf("Assertion \"%s\" failed at line %d in %s\n", \
             x, __LINE__, __FILE__); fflush(NULL); abort();} while(0)
#else
#define LWIP_PLATFORM_ASSERT(x)
#endif

#endif /* _ARCH_CC_H_ */
