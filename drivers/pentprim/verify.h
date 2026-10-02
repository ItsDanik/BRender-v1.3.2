#ifndef PENTPRIM_VERIFY_H
#define PENTPRIM_VERIFY_H

/*
 * Differential verification of the rewritten (fast*.c) rasteriser functions.
 *
 * With PENTPRIM_VERIFY=1 in the environment, every dispatched call is run
 * twice from the same starting state: first the original transliterated
 * code, then the rewrite. Colour buffer, depth buffer and the rasteriser
 * workspace must come out byte-identical; mismatches are reported on stderr
 * and summarised at exit. Without it only the rewrite runs.
 *
 * PENTPRIM_REFERENCE=1 runs only the original code (for A/B benchmarks).
 * PENTPRIM_TIMING=1 alternates original and rewrite call by call and
 * reports the average time per call of each at exit.
 */

extern int gPentprim_verify;
extern int gPentprim_reference;
extern int gPentprim_timing;

void PentprimVerify_Begin(void);
void PentprimVerify_Switch(void);
void PentprimVerify_End(const char* name);
int PentprimTiming_Begin(const char* name);
void PentprimTiming_End(void);

/* Set while a dispatched call runs during verification/timing so nested
 * dispatches (e.g. ZPT falling back to ZT) follow the outer choice:
 * 1 = original, 2 = rewrite */
extern int gPentprim_nested;

#define PENTPRIM_DISPATCH(name, ref_call, fast_call) \
    do {                                             \
        if (gPentprim_nested) {                      \
            if (gPentprim_nested == 1) {             \
                ref_call;                            \
            } else {                                 \
                fast_call;                           \
            }                                        \
        } else if (gPentprim_reference) {            \
            ref_call;                                \
        } else if (gPentprim_timing) {               \
            if (PentprimTiming_Begin(name)) {        \
                gPentprim_nested = 1;                \
                ref_call;                            \
            } else {                                 \
                gPentprim_nested = 2;                \
                fast_call;                           \
            }                                        \
            gPentprim_nested = 0;                    \
            PentprimTiming_End();                    \
        } else if (gPentprim_verify) {               \
            PentprimVerify_Begin();                  \
            gPentprim_nested = 1;                    \
            ref_call;                                \
            PentprimVerify_Switch();                 \
            gPentprim_nested = 2;                    \
            fast_call;                               \
            gPentprim_nested = 0;                    \
            PentprimVerify_End(name);                \
        } else {                                     \
            fast_call;                               \
        }                                            \
    } while (0)

#endif
