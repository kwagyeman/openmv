/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2024 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2024 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SIMD abstraction.
 */
#include <arm_math.h>
#include <cmsis_extension.h>

#if (__ARM_ARCH >= 8)
#define VECTOR_SIZE_BYTES   16
#else
#define VECTOR_SIZE_BYTES   4
#endif

#define INT8_VECTOR_SIZE    (VECTOR_SIZE_BYTES / sizeof(int8_t))
#define UINT8_VECTOR_SIZE   (VECTOR_SIZE_BYTES / sizeof(uint8_t))

#define INT16_VECTOR_SIZE   (VECTOR_SIZE_BYTES / sizeof(int16_t))
#define UINT16_VECTOR_SIZE  (VECTOR_SIZE_BYTES / sizeof(uint16_t))

#define INT32_VECTOR_SIZE   (VECTOR_SIZE_BYTES / sizeof(int32_t))
#define UINT32_VECTOR_SIZE  (VECTOR_SIZE_BYTES / sizeof(uint32_t))

#if (VECTOR_SIZE_BYTES >= 8)
#define INT64_VECTOR_SIZE   (VECTOR_SIZE_BYTES / sizeof(int64_t))
#define UINT64_VECTOR_SIZE  (VECTOR_SIZE_BYTES / sizeof(uint64_t))
#endif

#if (__ARM_ARCH >= 8)
typedef int8x16_t v128_s8_t;
typedef uint8x16_t v128_u8_t;

typedef int16x8_t v128_s16_t;
typedef uint16x8_t v128_u16_t;

typedef int32x4_t v128_s32_t;
typedef uint32x4_t v128_u32_t;

#if (VECTOR_SIZE_BYTES >= 8)
typedef int64x2_t v128_s64_t;
typedef uint64x2_t v128_u64_t;
#endif
#else
typedef int8_t v128_s8_t __attribute__ ((vector_size (VECTOR_SIZE_BYTES)));
typedef uint8_t v128_u8_t __attribute__ ((vector_size (VECTOR_SIZE_BYTES)));

typedef int16_t v128_s16_t __attribute__ ((vector_size (VECTOR_SIZE_BYTES)));
typedef uint16_t v128_u16_t __attribute__ ((vector_size (VECTOR_SIZE_BYTES)));

typedef int32_t v128_s32_t __attribute__ ((vector_size (VECTOR_SIZE_BYTES)));
typedef uint32_t v128_u32_t __attribute__ ((vector_size (VECTOR_SIZE_BYTES)));

#if (VECTOR_SIZE_BYTES >= 8)
typedef int64_t v128_s64_t __attribute__ ((vector_size (VECTOR_SIZE_BYTES)));
typedef uint64_t v128_u64_t __attribute__ ((vector_size (VECTOR_SIZE_BYTES)));
#endif
#endif

typedef union {
    v128_s8_t s8;
    v128_u8_t u8;
    v128_s16_t s16;
    v128_u16_t u16;
    v128_s32_t s32;
    v128_u32_t u32;
    #if (VECTOR_SIZE_BYTES >= 8)
    v128_s64_t s64;
    v128_u64_t u64;
    #endif
} v128_t;

static inline v128_t vhadd_u8(v128_t v0, v128_t v1) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vhaddq(v0.u8, v1.u8);
    #elif (__ARM_ARCH >= 7)
    return (v128_t) { .u32 = { __UHADD8(v0.u32[0], v1.u32[0]) } };
    #else
    return (v128_t) { .u8 = (v0.u8 + v1.u8) >> 1 };
    #endif
}

static inline v128_t vhadd_s8(v128_t v0, v128_t v1) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vhaddq(v0.s8, v1.s8);
    #elif (__ARM_ARCH >= 7)
    return (v128_t) { .s32 = { __SHADD8(v0.s32[0], v1.s32[0]) } };
    #else
    return (v128_t) { .s8 = (v0.s8 + v1.s8) >> 1 };
    #endif
}

static inline v128_t vhadd_u16(v128_t v0, v128_t v1) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vhaddq(v0.u16, v1.u16);
    #elif (__ARM_ARCH >= 7)
    return (v128_t) { .u32 = { __UHADD16(v0.u32[0], v1.u32[0]) } };
    #else
    return (v128_t) { .u16 = (v0.u16 + v1.u16) >> 1 };
    #endif
}

static inline v128_t vhadd_s16(v128_t v0, v128_t v1) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vhaddq(v0.s16, v1.s16);
    #elif (__ARM_ARCH >= 7)
    return (v128_t) { .s32 = { __SHADD16(v0.s32[0], v1.s32[0]) } };
    #else
    return (v128_t) { .s16 = (v0.s16 + v1.s16) >> 1 };
    #endif
}

static inline v128_t vuxtb16(v128_t v0) {
    #if (__ARM_ARCH >= 7)
    return (v128_t) { .u32 = { __UXTB16(v0.u32[0]) } };
    #else
    v128_t r;
    r.u8[0] = v0.u8[0];
    r.u8[1] = 0;
    r.u8[2] = v0.u8[2];
    r.u8[3] = 0;
    return r;
    #endif
}

static inline v128_t vuxtb16_8(v128_t v0) {
    #if (__ARM_ARCH >= 7)
    return (v128_t) { .u32 = { __UXTB16_RORn(v0.u32[0], 8) } };
    #else
    v128_t r;
    r.u8[0] = v0.u8[1];
    r.u8[1] = 0;
    r.u8[2] = v0.u8[3];
    r.u8[3] = 0;
    return r;
    #endif
}

static inline v128_t vpkhbt(v128_t v0, v128_t v1) {
    #if (__ARM_ARCH >= 7)
    return (v128_t) { .u32 = { __PKHBT(v0.u32[0], v1.u32[0], 16) } };
    #else
    v128_t r;
    r.u16[0] = v0.u16[0];
    r.u16[1] = v1.u16[0];
    return r;
    #endif
}

static inline v128_t vpkhbt_8(v128_t v0, v128_t v1) {
    #if (__ARM_ARCH >= 7)
    return (v128_t) { .u32 = { __PKHBT(v0.u32[0], v1.u32[0], 8) } };
    #else
    v128_t r;
    r.u16[0] = v0.u16[0];
    r.u16[1] = v1.u32[0] >> 8;
    return r;
    #endif
}

static inline v128_t vpkhtb(v128_t v0, v128_t v1) {
    #if (__ARM_ARCH >= 7)
    return (v128_t) { .u32 = { __PKHTB(v0.u32[0], v1.u32[0], 16) } };
    #else
    v128_t r;
    r.u16[0] = v1.u16[1];
    r.u16[1] = v0.u16[1];
    return r;
    #endif
}

static inline v128_t vpkhtb_8(v128_t v0, v128_t v1) {
    #if (__ARM_ARCH >= 7)
    return (v128_t) { .u32 = { __PKHTB(v0.u32[0], v1.u32[0], 8) } };
    #else
    v128_t r;
    r.u16[0] = v1.s32[0] >> 8;
    r.u16[1] = v0.u16[1];
    return r;
    #endif
}

static inline uint8_t vget_u8(v128_t v0, uint32_t n) {
    return v0.u8[n];
}

static inline int8_t vget_s8(v128_t v0, uint32_t n) {
    return v0.s8[n];
}

static inline uint16_t vget_u16(v128_t v0, uint32_t n) {
    return v0.u16[n];
}

static inline int16_t vget_s16(v128_t v0, uint32_t n) {
    return v0.s16[n];
}

static inline uint32_t vget_u32(v128_t v0, uint32_t n) {
    return v0.u32[n];
}

static inline int32_t vget_s32(v128_t v0, uint32_t n) {
    return v0.s32[n];
}

#if (VECTOR_SIZE_BYTES >= 8)
static inline uint64_t vget_u64(v128_t v0, uint32_t n) {
    return v0.u64[n];
}

static inline int64_t vget_s64(v128_t v0, uint32_t n) {
    return v0.s64[n];
}
#endif

static inline v128_t vset_u8(v128_t v0, uint32_t n, uint8_t x) {
    v0.u8[n] = x;
    return v0;
}

static inline v128_t vset_s8(v128_t v0, uint32_t n, int8_t x) {
    v0.s8[n] = x;
    return v0;
}

static inline v128_t vset_u16(v128_t v0, uint32_t n, uint16_t x) {
    v0.u16[n] = x;
    return v0;
}

static inline v128_t vset_s16(v128_t v0, uint32_t n, int16_t x) {
    v0.s16[n] = x;
    return v0;
}

static inline v128_t vset_u32(v128_t v0, uint32_t n, uint32_t x) {
    v0.u32[n] = x;
    return v0;
}

static inline v128_t vset_s32(v128_t v0, uint32_t n, int32_t x) {
    v0.s32[n] = x;
    return v0;
}

#if (VECTOR_SIZE_BYTES >= 8)
static inline v128_t vset_u64(v128_t v0, uint32_t n, uint64_t x) {
    v0.u64[n] = x;
    return v0;
}

static inline v128_t vset_s64(v128_t v0, uint32_t n, int64_t x) {
    v0.s64[n] = x;
    return v0;
}
#endif

// GCC does not vectorize assignment from a scalar to a vector.
static inline v128_t vdup_u8(uint8_t x) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vdupq_n_u8(x);
    #else
    return (v128_t) { .u32 = { x * 0x01010101 } };
    #endif
}

// GCC does not vectorize assignment from a scalar to a vector.
static inline v128_t vdup_s8(int8_t x) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vdupq_n_s8(x);
    #else
    return (v128_t) { .s32 = { (x & 0xFF) * 0x01010101 } };
    #endif
}

// GCC does not vectorize assignment from a scalar to a vector.
static inline v128_t vdup_u16(uint16_t x) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vdupq_n_u16(x);
    #else
    return (v128_t) { .u32 = { x * 0x00010001 } };
    #endif
}

// GCC does not vectorize assignment from a scalar to a vector.
static inline v128_t vdup_s16(int16_t x) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vdupq_n_s16(x);
    #else
    return (v128_t) { .s32 = { (x & 0xFFFF) * 0x00010001 } };
    #endif
}

static inline v128_t vdup_u32(uint32_t x) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vdupq_n_u32(x);
    #else
    return (v128_t) { .u32 = { x } };
    #endif
}

static inline v128_t vdup_s32(int32_t x) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vdupq_n_s32(x);
    #else
    return (v128_t) { .s32 = { x } };
    #endif
}

static inline v128_t vshlc(v128_t v0, uint32_t n) {
    #if (__ARM_ARCH >= 8)
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
    uint32_t c;
    return (v128_t) vshlcq(v0.u32, &c, n);
    #pragma GCC diagnostic pop
    #else
    return (v128_t) { .u32 = v0.u32 << n };
    #endif
}

static inline v128_t vadd_u32(v128_t v0, v128_t v1) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vaddq(v0.u32, v1.u32);
    #else
    return (v128_t) { .u32 = v0.u32 + v1.u32 };
    #endif
}

static inline v128_t vadd_s32(v128_t v0, v128_t v1) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vaddq(v0.s32, v1.s32);
    #else
    return (v128_t) { .s32 = v0.s32 + v1.s32 };
    #endif
}

static inline v128_t vasr_s32(v128_t v0, uint32_t n) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vshrq(v0.s32, n);
    #else
    return (v128_t) { .s32 = v0.s32 >> n };
    #endif
}

static inline v128_t vlsl_u32(v128_t v0, uint32_t n) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vshlq_n(v0.u32, n);
    #else
    return (v128_t) { .u32 = v0.u32 << n };
    #endif
}

static inline v128_t vlsl_s32(v128_t v0, uint32_t n) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vshlq_n(v0.s32, n);
    #else
    return (v128_t) { .s32 = v0.s32 << n };
    #endif
}

static inline v128_t vlsr_u32(v128_t v0, uint32_t n) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vshrq(v0.u32, n);
    #else
    return (v128_t) { .u32 = v0.u32 >> n };
    #endif
}

static inline v128_t vand_u32(v128_t v0, v128_t v1) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vandq(v0.u32, v1.u32);
    #else
    return (v128_t) { .u32 = v0.u32 & v1.u32 };
    #endif
}

static inline v128_t vand_s32(v128_t v0, v128_t v1) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vandq(v0.s32, v1.s32);
    #else
    return (v128_t) { .s32 = v0.s32 & v1.s32 };
    #endif
}

static inline v128_t vorr_u32(v128_t v0, v128_t v1) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vorrq(v0.u32, v1.u32);
    #else
    return (v128_t) { .u32 = v0.u32 | v1.u32 };
    #endif
}

static inline v128_t vorr_s32(v128_t v0, v128_t v1) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vorrq(v0.s32, v1.s32);
    #else
    return (v128_t) { .s32 = v0.s32 | v1.s32 };
    #endif
}

static inline v128_t vmul_u32(v128_t v0, v128_t v1) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vmulq(v0.u32, v1.u32);
    #else
    return (v128_t) { .u32 = v0.u32 * v1.u32 };
    #endif
}

static inline v128_t vmul_s32(v128_t v0, v128_t v1) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vmulq(v0.s32, v1.s32);
    #else
    return (v128_t) { .s32 = v0.s32 * v1.s32 };
    #endif
}

static inline v128_t vmla_u32(v128_t v0, v128_t v1, v128_t v2) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vaddq(vmulq(v0.u32, v1.u32), v2.u32);
    #else
    return (v128_t) { .u32 = (v0.u32 * v1.u32) + v2.u32 };
    #endif
}

static inline v128_t vmla_s32(v128_t v0, v128_t v1, v128_t v2) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vaddq(vmulq(v0.s32, v1.s32), v2.s32);
    #else
    return (v128_t) { .s32 = (v0.s32 * v1.s32) + v2.s32 };
    #endif
}

static inline v128_t vldr_u8(const uint8_t *p, uint32_t n) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vldrbq_z_u8(p, vctp8q(n));
    #else
    v128_t v0;

    if (n > 3) {
        v0.u32[0] = *((uint32_t *) p);
    } else if (n > 2) {
        v0.u32[0] = *((uint16_t *) p);
        v0.u8[2] = p[2];
    } else if (n > 1) {
        v0.u32[0] = *((uint16_t *) p);
    } else {
        v0.u32[0] = p[0];
    }

    return v0;
    #endif
}

static inline void vstr_u8(uint8_t *p, v128_t v0, uint32_t n) {
    #if (__ARM_ARCH >= 8)
    vstrbq_p_u8(p, v0.u8, vctp8q(n));
    #else
    if (n > 3) {
        *((uint32_t *) p) = v0.u32[0];
    } else if (n > 2) {
        *((uint16_t *) p) = v0.u16[0];
        p[2] = v0.u8[2];
    } else if (n > 1) {
        *((uint16_t *) p) = v0.u16[0];
    } else {
        p[0] = v0.u8[0];
    }
    #endif
}

static inline v128_t vldr_u16(uint16_t *p, uint32_t n) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vldrhq_z_u16(p, vctp16q(n));
    #else
    v128_t v0;

    if (n > 1) {
        v0.u32[0] = *((uint32_t *) p);
    } else {
        v0.u32[0] = p[0];
    }

    return v0;
    #endif
}

static inline void vstr_u16(uint16_t *p, v128_t v0, uint32_t n) {
    #if (__ARM_ARCH >= 8)
    vstrhq_p_u16(p, v0.u16, vctp16q(n));
    #else
    if (n > 1) {
        *((uint32_t *) p) = v0.u32[0];
    } else {
        p[0] = v0.u16[0];
    }
    #endif
}

static inline v128_t vldr_u8_widen_u16(uint8_t *p, uint32_t n) {
    #if (__ARM_ARCH >= 8)
    return (v128_t) vldrbq_z_u16(p, vctp16q(n));
    #else
    v128_t v0;

    if (n > 1) {
        v0.u32[0] = *((uint16_t *) p);
        v0.u8[2] = v0.u8[1];
        v0.u8[1] = 0;
    } else {
        v0.u32[0] = *p;
    }

    return v0;
    #endif
}

static inline void vstr_u16_narrow_u8(uint8_t *p, v128_t v0, uint32_t n) {
    #if (__ARM_ARCH >= 8)
    vstrbq_p_u16(p, v0.u16, vctp16q(n));
    #else
    if (n > 1) {
        *((uint16_t *) p) = (v0.u8[2] << 8) | v0.u8[0];
    } else {
        *p = v0.u8[0];
    }
    #endif
}
