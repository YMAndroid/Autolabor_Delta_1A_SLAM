
#ifndef __TYPEDEF_H__
#define __TYPEDEF_H__

#include <stdint.h>


typedef int8_t              s8;
typedef uint8_t             u8;

typedef int16_t             s16;
typedef uint16_t            u16;

typedef int32_t             s32;
typedef uint32_t            u32;

typedef int64_t             s64;
typedef uint64_t            u64;

enum TYPEBOOL
{
  FALSE = 0, TRUE  = !FALSE
};

#define HWREG(x)                            	 (*((volatile unsigned long *)(x)))
#define HWREGS(x)                            	 (*((volatile signed long *)(x)))
#define HSREG(x)								 (*((volatile unsigned short *)(x)))
#define HBREG(x)                            	 (*((volatile unsigned char *)(x)))

#define U8_MAX     								 ((u8)255)
#define S8_MAX     								 ((s8)127)
#define S8_MIN     								 ((s8)-128)
#define U16_MAX    								 ((u16)65535u)
#define S16_MAX    								 ((s16)32767)
#define S16_MIN    								 ((s16)-32768)
#define U32_MAX    								 ((u32)4294967295uL)
#define S32_MAX    								 ((s32)2147483647)
#define S32_MIN    								 ((s32)-2147483648)

#endif //__TYPEDEF_H__
