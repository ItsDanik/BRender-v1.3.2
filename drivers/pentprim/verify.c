/*
 * Differential verification of the rewritten rasteriser functions, see verify.h
 */
#include "verify.h"

#include "brender.h"
#include "fpwork.h"
#include "work.h"
#include "x86emu.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

int gPentprim_verify;
int gPentprim_reference;
int gPentprim_timing;
int gPentprim_nested;

typedef struct {
    struct prim_work work;
    struct workspace_t workspace;
    x86emu_state_t x86_state;
    x86_reg regs[7];
} tState;

static tState start_state;
static tState ref_state;
static br_uint_8 *start_colour, *start_depth, *ref_colour, *ref_depth;
static size_t colour_size, depth_size, buffers_size;

#define MAX_NAMES 32
static struct {
    const char* name;
    long calls;
    long pixel_fails;
    long state_fails;
} stats[MAX_NAMES];
static int reported;

// call by call alternating timing
static struct {
    const char* name;
    long calls[2];
    long long ns[2];
} timing[MAX_NAMES];
static int timing_slot, timing_ref;
static struct timespec timing_start;

static void timing_summary(void) {
    int i;
    fprintf(stderr, "pentprim timing (average per call, original vs rewrite):\n");
    for (i = 0; i < MAX_NAMES && timing[i].name; i++) {
        double ref = timing[i].calls[1] ? (double)timing[i].ns[1] / timing[i].calls[1] : 0;
        double fast = timing[i].calls[0] ? (double)timing[i].ns[0] / timing[i].calls[0] : 0;
        fprintf(stderr, "  %-32s %8ld calls  %9.0fns -> %9.0fns  (x%.2f)\n",
            timing[i].name, timing[i].calls[0] + timing[i].calls[1], ref, fast, fast > 0 ? ref / fast : 0);
    }
}

int PentprimTiming_Begin(const char* name) {
    int i;
    for (i = 0; i < MAX_NAMES - 1 && timing[i].name && strcmp(timing[i].name, name) != 0; i++) {
    }
    timing[i].name = name;
    timing_slot = i;
    timing_ref = (timing[i].calls[0] + timing[i].calls[1]) & 1;
    clock_gettime(CLOCK_MONOTONIC, &timing_start);
    return timing_ref;
}

void PentprimTiming_End(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    timing[timing_slot].ns[timing_ref] += (now.tv_sec - timing_start.tv_sec) * 1000000000LL + (now.tv_nsec - timing_start.tv_nsec);
    timing[timing_slot].calls[timing_ref]++;
}

static void save_state(tState* s) {
    s->work = work;
    s->workspace = workspace;
    s->x86_state = x86_state;
    s->regs[0] = eax;
    s->regs[1] = ebx;
    s->regs[2] = ecx;
    s->regs[3] = edx;
    s->regs[4] = esi;
    s->regs[5] = edi;
    s->regs[6] = ebp;
}

static void load_state(const tState* s) {
    work = s->work;
    workspace = s->workspace;
    x86_state = s->x86_state;
    eax = s->regs[0];
    ebx = s->regs[1];
    ecx = s->regs[2];
    edx = s->regs[3];
    esi = s->regs[4];
    edi = s->regs[5];
    ebp = s->regs[6];
}

static size_t buffer_size(struct render_buffer* b) {
    br_int_32 stride = b->stride_b < 0 ? -b->stride_b : b->stride_b;
    return b->base ? (size_t)stride * b->height : 0;
}

static void summary(void) {
    int i;
    fprintf(stderr, "pentprim verify summary:\n");
    for (i = 0; i < MAX_NAMES && stats[i].name; i++) {
        fprintf(stderr, "  %-32s %10ld calls, %ld pixel mismatches, %ld state mismatches\n",
            stats[i].name, stats[i].calls, stats[i].pixel_fails, stats[i].state_fails);
    }
}

__attribute__((constructor)) static void verify_init(void) {
    const char* env = getenv("PENTPRIM_VERIFY");
    gPentprim_verify = env != NULL && env[0] == '1';
    env = getenv("PENTPRIM_TIMING");
    gPentprim_timing = env != NULL && env[0] == '1';
    if (gPentprim_timing) {
        atexit(timing_summary);
    }
    env = getenv("PENTPRIM_REFERENCE");
    gPentprim_reference = env != NULL && env[0] == '1';
    if (gPentprim_reference) {
        fprintf(stderr, "pentprim: using the original rasteriser functions only\n");
    }
    if (gPentprim_verify) {
        fprintf(stderr, "pentprim: verifying rewritten rasteriser functions against the originals\n");
        atexit(summary);
    }
}

void PentprimVerify_Begin(void) {
    colour_size = buffer_size(&work.colour);
    depth_size = buffer_size(&work.depth);
    if (colour_size + depth_size > buffers_size) {
        buffers_size = colour_size + depth_size;
        start_colour = realloc(start_colour, buffers_size);
        start_depth = realloc(start_depth, buffers_size);
        ref_colour = realloc(ref_colour, buffers_size);
        ref_depth = realloc(ref_depth, buffers_size);
    }
    save_state(&start_state);
    memcpy(start_colour, work.colour.base, colour_size);
    if (depth_size) {
        memcpy(start_depth, work.depth.base, depth_size);
    }
}

void PentprimVerify_Switch(void) {
    save_state(&ref_state);
    memcpy(ref_colour, work.colour.base, colour_size);
    if (depth_size) {
        memcpy(ref_depth, work.depth.base, depth_size);
    }
    load_state(&start_state);
    memcpy(work.colour.base, start_colour, colour_size);
    if (depth_size) {
        memcpy(work.depth.base, start_depth, depth_size);
    }
}

static int first_diff(const br_uint_8* a, const br_uint_8* b, size_t n) {
    size_t i;
    for (i = 0; i < n; i++) {
        if (a[i] != b[i]) {
            return (int)i;
        }
    }
    return -1;
}

void PentprimVerify_End(const char* name) {
    int i, c, d, s;
    br_int_32 cs = work.colour.stride_b, ds = work.depth.stride_b;

    for (i = 0; i < MAX_NAMES && stats[i].name && strcmp(stats[i].name, name) != 0; i++) {
    }
    if (i == MAX_NAMES) {
        return;
    }
    stats[i].name = name;
    stats[i].calls++;

    c = first_diff(ref_colour, work.colour.base, colour_size);
    d = depth_size ? first_diff(ref_depth, work.depth.base, depth_size) : -1;
    s = first_diff((br_uint_8*)&ref_state.workspace, (br_uint_8*)&workspace, sizeof(workspace));
    if (s < 0) {
        // rasteriser state in work (scanline interpolants etc.), reported past the workspace bytes
        int w = first_diff((br_uint_8*)&ref_state.work, (br_uint_8*)&work, sizeof(work));
        if (w >= 0) {
            s = (int)sizeof(workspace) + w;
        }
    }
    if (c >= 0 || d >= 0) {
        stats[i].pixel_fails++;
    }
    if (s >= 0) {
        stats[i].state_fails++;
    }
    if ((c >= 0 || d >= 0 || s >= 0) && reported < 20) {
        reported++;
        fprintf(stderr, "pentprim verify: %s mismatch", name);
        if (c >= 0) {
            fprintf(stderr, " colour at (%d,%d) ref %d got %d", c % cs, c / cs, ref_colour[c], ((br_uint_8*)work.colour.base)[c]);
        }
        if (d >= 0) {
            fprintf(stderr, " depth at (%d,%d)", (d % ds) / 2, d / ds);
        }
        if (s >= 0) {
            if (s < (int)sizeof(workspace)) {
                fprintf(stderr, " workspace byte %d", s);
            } else {
                fprintf(stderr, " work byte %d", s - (int)sizeof(workspace));
            }
        }
        fprintf(stderr, "\n");
    }
    // continue from the reference result so later calls see the original state
    load_state(&ref_state);
    memcpy(work.colour.base, ref_colour, colour_size);
    if (depth_size) {
        memcpy(work.depth.base, ref_depth, depth_size);
    }
}
