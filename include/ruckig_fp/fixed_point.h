/**
 * @file fixed_point.h
 * @brief Fixed-point arithmetic types, macros, and inline helpers for ruckig_fp.
 *
 * This file defines the `fp_t` type (Q16.16 fixed-point by default) and a
 * complete set of arithmetic helpers suitable for use on the RP2040 Cortex-M0+,
 * which has no hardware floating-point unit.
 *
 * ## Representation
 *
 * A `fp_t` value stores a signed fixed-point number with `FP_FRAC_BITS` binary
 * fractional digits.  With the default of 16:
 *
 *   - Integer range:  -(2^15) = -32768  to  (2^15 - 1) = 32767
 *   - Resolution:      1 / 65536 ≈ 0.0000153
 *
 * Changing `FP_FRAC_BITS` to 24 gives higher precision but a narrower integer
 * range (±127).  Changing it to 8 gives a wider range (±8388607) but coarser
 * resolution.  Choose based on the physical units in your application.
 *
 * ## Usage
 *
 * ```c
 * fp_t a = FP_FROM_INT(5);          // 5.0 in fixed-point
 * fp_t b = FP_FROM_FLOAT(1.5f);     // 1.5 in fixed-point (compile-time)
 * fp_t c = fp_mul(a, b);            // 7.5 in fixed-point
 * float f = FP_TO_FLOAT(c);         // back to float for debugging
 * ```
 *
 * @note `FP_FROM_FLOAT` and `FP_TO_FLOAT` use floating-point casts and are
 *       intended only for compile-time constant initialisation or debug output.
 *       Do not call them in real-time code paths.
 */

#ifndef RUCKIG_FP_FIXED_POINT_H
#define RUCKIG_FP_FIXED_POINT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * Configurable precision
 * --------------------------------------------------------------------- */

/**
 * @brief Number of fractional bits in the Q(32-FP_FRAC_BITS).FP_FRAC_BITS
 *        fixed-point representation.
 *
 * Default: 16 (Q16.16). Override by defining this before including the header.
 */
#ifndef FP_FRAC_BITS
#define FP_FRAC_BITS 16
#endif

/** Scaling factor: 2^FP_FRAC_BITS */
#define FP_SCALE  ((int32_t)(1 << FP_FRAC_BITS))

/* -------------------------------------------------------------------------
 * Base type
 * --------------------------------------------------------------------- */

/**
 * @brief Signed Q(32-FP_FRAC_BITS).FP_FRAC_BITS fixed-point scalar.
 *
 * Stored as a 32-bit signed integer.  The value represented is:
 *   real_value = fp_value / 2^FP_FRAC_BITS
 */
typedef int32_t fp_t;

/* -------------------------------------------------------------------------
 * Conversion macros
 * --------------------------------------------------------------------- */

/**
 * @brief Convert a compile-time integer literal to fp_t.
 * @param x  Integer value (must fit in the integer portion).
 */
#define FP_FROM_INT(x)    ((fp_t)((x) << FP_FRAC_BITS))

/**
 * @brief Convert a compile-time float/double literal to fp_t.
 *
 * Implemented as a cast and is evaluated at compile time for constant
 * expressions.  Do **not** use in real-time code paths.
 *
 * @param x  Floating-point value.
 */
#define FP_FROM_FLOAT(x)  ((fp_t)((x) * (float)FP_SCALE))

/**
 * @brief Truncate a fp_t to its integer part (towards zero).
 * @param x  Fixed-point value.
 * @return   Integer part as int32_t.
 */
#define FP_TO_INT(x)      ((int32_t)((x) >> FP_FRAC_BITS))

/**
 * @brief Convert a fp_t back to float (for debugging only).
 * @param x  Fixed-point value.
 * @return   Floating-point representation.
 */
#define FP_TO_FLOAT(x)    ((float)(x) / (float)FP_SCALE)

/* -------------------------------------------------------------------------
 * Constant helpers
 * --------------------------------------------------------------------- */

/** Fixed-point representation of 0 */
#define FP_ZERO    ((fp_t)0)

/** Fixed-point representation of 1 */
#define FP_ONE     FP_FROM_INT(1)

/** Fixed-point representation of 2 */
#define FP_TWO     FP_FROM_INT(2)

/** Fixed-point representation of 6 */
#define FP_SIX     FP_FROM_INT(6)

/** Maximum representable fp_t value */
#define FP_MAX     ((fp_t)INT32_MAX)

/** Minimum representable fp_t value */
#define FP_MIN_VAL ((fp_t)INT32_MIN)

/* -------------------------------------------------------------------------
 * Arithmetic inline functions
 * --------------------------------------------------------------------- */

/**
 * @brief Multiply two fixed-point values.
 *
 * Widens both operands to 64-bit, multiplies, then shifts right by
 * FP_FRAC_BITS to restore the correct binary point.  This avoids the
 * intermediate overflow that would occur if the multiplication were done
 * in 32 bits.
 *
 * Precision: the result is rounded towards zero (truncated), introducing
 * at most one LSB of error per multiplication.
 *
 * @param a  First factor (fp_t).
 * @param b  Second factor (fp_t).
 * @return   a * b in fp_t, or saturated at FP_MAX / FP_MIN_VAL on overflow.
 */
static inline fp_t fp_mul(fp_t a, fp_t b)
{
    // Widen to 64-bit before multiplying to avoid 32-bit overflow.
    int64_t result = ((int64_t)a * (int64_t)b) >> FP_FRAC_BITS;
    // Saturate on overflow (result outside int32_t range).
    if (result > (int64_t)INT32_MAX) return FP_MAX;
    if (result < (int64_t)INT32_MIN) return FP_MIN_VAL;
    return (fp_t)result;
}

/**
 * @brief Divide two fixed-point values.
 *
 * Shifts the dividend left by FP_FRAC_BITS (using 64 bits) before the
 * integer division so that the result has the correct binary point.
 *
 * Precision: truncated towards zero.
 *
 * Caveat: undefined behaviour if `b == 0`; the caller must guard against
 * division by zero.
 *
 * @param a  Numerator (fp_t).
 * @param b  Denominator (fp_t, must not be zero).
 * @return   a / b in fp_t.
 */
static inline fp_t fp_div(fp_t a, fp_t b)
{
    // Shift a left by FP_FRAC_BITS in 64-bit space before dividing.
    int64_t numerator = (int64_t)a << FP_FRAC_BITS;
    return (fp_t)(numerator / (int64_t)b);
}

/**
 * @brief Absolute value of a fixed-point number.
 *
 * @param x  Input value.
 * @return   |x| in fp_t.  Note: FP_MIN_VAL has no positive counterpart
 *           in two's complement; the result is undefined for that input.
 */
static inline fp_t fp_abs(fp_t x)
{
    return (x < 0) ? -x : x;
}

/**
 * @brief Minimum of two fixed-point values.
 *
 * @param a  First value.
 * @param b  Second value.
 * @return   The smaller of a and b.
 */
static inline fp_t fp_min(fp_t a, fp_t b)
{
    return (a < b) ? a : b;
}

/**
 * @brief Maximum of two fixed-point values.
 *
 * @param a  First value.
 * @param b  Second value.
 * @return   The larger of a and b.
 */
static inline fp_t fp_max(fp_t a, fp_t b)
{
    return (a > b) ? a : b;
}

/**
 * @brief Signum of a fixed-point value.
 *
 * @param x  Input value.
 * @return   FP_FROM_INT(1) if x > 0, FP_FROM_INT(-1) if x < 0, FP_ZERO if x == 0.
 */
static inline fp_t fp_sign(fp_t x)
{
    if (x > 0) return FP_ONE;
    if (x < 0) return -FP_ONE;
    return FP_ZERO;
}

/* -------------------------------------------------------------------------
 * Square root (declared here, implemented in fixed_point.c)
 * --------------------------------------------------------------------- */

/**
 * @brief Integer Newton-Raphson square root for fixed-point values.
 *
 * Computes floor(sqrt(x)) in fixed-point arithmetic without using
 * `<math.h>` or any floating-point instruction.
 *
 * Algorithm: the Q(16.16) value x represents the real number x/65536.
 * We want sqrt(x/65536) = sqrt(x)/256 (for FP_FRAC_BITS == 16).
 * Internally we compute the 64-bit integer sqrt(x << FP_FRAC_BITS) and
 * return that as the fp_t result.
 *
 * Precision: result is within 1 LSB of the true value.
 *
 * Caveat: returns FP_ZERO for negative inputs (no NaN concept in integers).
 *
 * @param x  Non-negative fixed-point radicand.
 * @return   sqrt(x) in fp_t.
 */
fp_t fp_sqrt(fp_t x);

#ifdef __cplusplus
}
#endif

#endif /* RUCKIG_FP_FIXED_POINT_H */
