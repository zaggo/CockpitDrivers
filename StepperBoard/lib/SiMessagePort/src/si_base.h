#ifndef SI_BASE_H
#define SI_BASE_H

#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <float.h>
#include <limits.h>
#include <assert.h>

#ifdef WIN
#include <winsock2.h>
#include <Ws2tcpip.h>
#include <windows.h>
#include <iphlpapi.h>
#endif

#ifdef ARDUINO
#define SI_EMBEDDED
#endif

 /* Guard C code in headers, while including them from C++ */
#ifdef  __cplusplus
# define SI_BEGIN_DECLS  extern "C" {
# define SI_END_DECLS    }
#else
# define SI_BEGIN_DECLS
# define SI_END_DECLS
#endif

/* mark arguments explicitly as unused */
#define SI_IGNORE(x)    (void)(x)

SI_BEGIN_DECLS

typedef int SI_BOOL;

#define SI_TRUE    1
#define SI_FALSE   0

enum SiResult {
    SI_ERROR,
    SI_OK
};

#define SI_BASE_PI		 3.141592	// PI
#define SI_BASE_PI180	 0.017453	// PI / 180
#define SI_BASE_180PI	57.295780	// 180 / PI

#define SI_BASE_MALLOC(__object_type__) (__object_type__*)si_base_malloc(sizeof(__object_type__))

#define SI_BASE_ARRAY_SIZE(x)     (sizeof(x)/sizeof(x[0]))

// Convert degrees to radians and vice versa.
#define SI_BASE_DEGREES_TO_RADIANS(x) (x * SI_BASE_PI180)
#define SI_BASE_RADIANS_TO_DEGREES(x) (x * SI_BASE_180PI)

#define SI_BASE_DEGREES_NORMALIZE_FLOAT(__value__) (fmodf((fmodf(__value__, 360.0f) + 360.0f), 360.0f))
#define SI_BASE_DEGREES_NORMALIZE_DOUBLE(__value__) (fmod((fmod(__value__, 360.0) + 360.0), 360.0))

#define SI_BIT(B) (1  << (B))
#define SI_BIT_SET(R, B) ((R) |= (1  << (B)))
#define SI_BIT_CLR(R, B) ((R) &= ~(1  << (B)))
#define SI_BIT_TOGGLE(R, B) ((R) ^= (1  << (B)))
#define SI_BIT_PUT(R, B, V) ((V) ? SI_BIT_SET((R), (B)) : SI_BIT_CLR((R), (B)) )
#define SI_BIT_GET(R, B) ((R) & (1  << (B)) ? SI_TRUE : SI_FALSE)

#define SI_BIT_U32(B) (1UL  << (B))
#define SI_BIT_U32_SET(R, B) ((R) |= (1UL  << (B)))
#define SI_BIT_U32_CLR(R, B) ((R) &= ~(1UL  << (B)))
#define SI_BIT_U32_TOGGLE(R, B) ((R) ^= (1UL  << (B)))
#define SI_BIT_U32_PUT(R, B, V) ((V) ? SI_BIT_U32_SET((R), (B)) : SI_BIT_U32_CLR((R), (B)) )
#define SI_BIT_U32_GET(R, B) ((R) & (1UL  << (B)) ? SI_TRUE : SI_FALSE)

#define SI_BIT_U64(B) (1ULL  << (B))
#define SI_BIT_U64_SET(R, B) ((R) |= (1ULL  << (B)))
#define SI_BIT_U64_CLR(R, B) ((R) &= ~(1ULL  << (B)))
#define SI_BIT_U64_TOGGLE(R, B) ((R) ^= (1ULL  << (B)))
#define SI_BIT_U64_PUT(R, B, V) ((V) ? SI_BIT_U64_SET((R), (B)) : SI_BIT_U64_CLR((R), (B)) )
#define SI_BIT_U64_GET(R, B) ((R) & (1ULL  << (B)) ? SI_TRUE : SI_FALSE)

#define SI_MIN(a, b) (((a)<(b))?(a):(b))
#define SI_MAX(a, b) (((a)>(b))?(a):(b))
#define SI_CAP(val, min, max) (((val)<=(min))?(min):((val)>(max)?(max):(val)))

#ifdef __cplusplus
#ifdef WIN
#define SI_EXPORT extern "C" __declspec(dllexport)
#endif
#ifdef MAC
#define SI_EXPORT extern "C" __attribute__((visibility("default")))
#endif
#ifdef LIN
#define SI_EXPORT extern "C" __attribute__((visibility("default")))
#endif
#else
#ifdef WIN
#define SI_EXPORT __declspec(dllexport)
#endif
#ifdef MAC
#define SI_EXPORT __attribute__((visibility("default")))
#endif
#ifdef LIN
#define SI_EXPORT __attribute__((visibility("default")))
#endif
#endif

#ifdef __GNUC__
#define SI_PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

#ifdef _MSC_VER
#define SI_PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop))
#endif

void* si_base_malloc(size_t size);
void* si_base_calloc(size_t size);
void* si_base_realloc(void* block, size_t size);
void si_base_free(void* block);

SI_END_DECLS

#endif
