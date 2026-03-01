/**
 * @file fixed_point.c
 * @brief Fixed-point math utility functions for ruckig_fp.
 *
 * Implements the non-inline functions declared in fixed_point.h.
 * No <math.h> or floating-point arithmetic is used here.
 */

#include "ruckig_fp/fixed_point.h"

/* -------------------------------------------------------------------------
 * fp_sqrt — Newton-Raphson integer square root
 * --------------------------------------------------------------------- */

/**
 * @brief Compute the fixed-point square root of x using Newton-Raphson.
 *
 * Strategy for Q(32-F).F representation (F = FP_FRAC_BITS):
 *
 *   The stored integer value `x` represents the real number X = x / 2^F.
 *   We want Y = sqrt(X) = sqrt(x / 2^F) = sqrt(x) / 2^(F/2).
 *
 *   For even F (the normal case, F=16):
 *     Y_fixed = sqrt(x) / 2^(F/2)
 *             = sqrt(x << F) / 2^F      [multiply inside sqrt by 2^F,
 *                                         divide outside by 2^(F/2)]
 *
 *   So the algorithm is:
 *     1. Form the 64-bit integer  n = (int64_t)x << FP_FRAC_BITS
 *     2. Compute the 64-bit integer square root  r = isqrt64(n)
 *     3. Return r as fp_t (it is already in the correct Q format)
 *
 *   Newton-Raphson for integer sqrt:
 *     Start with a rough estimate, then iterate:
 *       r_new = (r_old + n / r_old) / 2
 *     Until convergence (|r_new - r_old| <= 1).
 *
 * Precision: The result satisfies  r^2 <= n < (r+1)^2, i.e. within 1 LSB.
 *
 * @param x  Non-negative fixed-point value.
 * @return   sqrt(x) as fp_t, or FP_ZERO if x <= 0.
 */
fp_t fp_sqrt(fp_t x)
{
    if (x <= 0) {
        return FP_ZERO;
    }

    // Step 1: scale the raw integer up so that the result will have the
    // correct binary point after taking the integer square root.
    // n represents X * 2^FP_FRAC_BITS in 64-bit space.
    uint64_t n = (uint64_t)(uint32_t)x << FP_FRAC_BITS;

    // Step 2: Newton-Raphson 64-bit integer square root.
    // Initial estimate: use the 32-bit sqrt of the upper 32 bits, shifted.
    // A safe starting point is to use the leading-bit position.
    uint64_t r;

    // Seed the estimate at roughly 2^((bits+1)/2).
    {
        // Count leading zeros to find the magnitude of n.
        // We want r such that r ≈ sqrt(n).
        uint64_t tmp = n;
        int bits = 0;
        while (tmp > 1ULL) {
            tmp >>= 1;
            bits++;
        }
        // r starts at 2^((bits+1)/2), a rough over-estimate.
        r = (uint64_t)1 << ((bits + 1) / 2);
    }

    // Newton-Raphson iterations converge quadratically; ~7 iterations
    // are sufficient for a 64-bit input.
    for (int i = 0; i < 16; i++) {
        uint64_t r_new = (r + n / r) >> 1;
        // Convergence: stop when the estimate no longer improves.
        if (r_new >= r) {
            break;
        }
        r = r_new;
    }

    // Step 3: ensure r is the floor (r^2 <= n), adjust by one if needed.
    // The Newton-Raphson result may be one too high due to integer rounding.
    while (r * r > n) {
        r--;
    }

    return (fp_t)r;
}
