/*
 * FPGA rasteriser offload: command emitter, buffer ownership and software
 * model, see fpgarast.h
 */
#include "fpgarast.h"

#include "brender.h"
#include "common.h"
#include "fpwork.h"
#include "work.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// NULL: the software model draws straight into host memory
static const tFpgaRast_backend* backend;

// model: an image of the rasteriser's memory to work on instead of host memory
static br_uint_8* model_image;

static void submit(const br_uint_32* w);

/*
 * Address space
 */

// buffer windows: host memory bound to a 1MB slot
static br_uint_8* window_host[FR_WINDOWS];
static int window_count;

/*
 * Regions: the parts of a window in use as colour or depth buffer.
 *
 * With a backend the FPGA has its own copy of every region, and the two
 * copies are brought in line only when needed: the host's is uploaded before
 * the FPGA draws into a region the host changed, the FPGA's is downloaded
 * when the host is about to look at a region the FPGA changed
 * (FpgaRast_Sync). Without a backend both are the same memory.
 */
typedef struct {
    int window;
    br_uint_32 lo, hi;             // offsets in the window
    br_uint_32 dirty_lo, dirty_hi; // changed by the FPGA since the last download
    br_uint_8 depth;
    br_uint_8 host_valid, fpga_valid;
} tRegion;

#define MAX_REGIONS 16
static tRegion regions[MAX_REGIONS];
static int region_count;

static struct {
    long scenes, uploads, upload_bytes, fills, downloads, download_bytes, waits, textures, texture_bytes;
} sync_stats;

// textures and shade tables: open addressing hash, host pointer -> address
#define HEAP_HASH_SIZE 1024
static struct heap_entry {
    const br_uint_8* host;
    br_uint_32 addr;
    br_uint_32 size;
    br_uint_32 checksum; // of what was uploaded
    int stale;
} heap_hash[HEAP_HASH_SIZE];
// the same entries in address order, for the model and the traces
static struct heap_entry heap_list[HEAP_HASH_SIZE];
static int heap_count, heap_hashed;
static br_uint_32 heap_top = FR_HEAP_BASE;

static br_uint_32 region_addr(const tRegion* r) {
    return FR_WINDOW_BASE + r->window * FR_WINDOW_SIZE;
}

static void fill_range(br_uint_32 addr, br_uint_32 row_bytes, br_uint_32 rows, br_uint_32 stride, br_uint_32 pattern) {
    br_uint_32 w[6];

    w[0] = FR_HEAD(FR_OP_FILL, 6, 0);
    w[1] = addr;
    w[2] = row_bytes;
    w[3] = rows;
    w[4] = stride;
    w[5] = pattern;
    submit(w);
    sync_stats.fills++;
}

// the host is about to use the region: bring its copy up to date
static void region_download(tRegion* r) {
    if (r->host_valid) {
        return;
    }
    if (backend != NULL) {
        backend->wait();
        sync_stats.waits++;
        if (r->dirty_hi > r->dirty_lo) {
            backend->download(region_addr(r) + r->dirty_lo, window_host[r->window] + r->dirty_lo, r->dirty_hi - r->dirty_lo);
            sync_stats.downloads++;
            sync_stats.download_bytes += r->dirty_hi - r->dirty_lo;
        }
    }
    r->host_valid = 1;
    r->dirty_lo = r->hi;
    r->dirty_hi = r->lo;
}

// the FPGA is about to draw into the region: bring its copy up to date
static void region_upload(tRegion* r) {
    const br_uint_8* host = window_host[r->window];
    br_uint_32 lo = r->lo & ~1u, size = r->hi - lo, i;

    if (r->fpga_valid || backend == NULL) {
        r->fpga_valid = 1;
        return;
    }
    // a cleared buffer needs no copy
    for (i = 2; i < size && host[lo + i] == host[lo + (i & 1)]; i++) {
    }
    if (i >= size && size >= 2) {
        const br_uint_32 pattern = host[lo] | (host[lo + 1] << 8);
        if (size >= FR_FILL_MAX) {
            fill_range(region_addr(r) + lo, FR_FILL_MAX, size / FR_FILL_MAX, FR_FILL_MAX, pattern);
        }
        if (size % FR_FILL_MAX) {
            fill_range(region_addr(r) + lo + size / FR_FILL_MAX * FR_FILL_MAX, size % FR_FILL_MAX, 1, FR_FILL_MAX, pattern);
        }
    } else {
        backend->upload(region_addr(r) + r->lo, host + r->lo, r->hi - r->lo);
        sync_stats.uploads++;
        sync_stats.upload_bytes += r->hi - r->lo;
    }
    r->fpga_valid = 1;
}

// the FPGA draws into [offset, offset + size) of the region
static void region_drawn(tRegion* r, br_uint_32 addr, br_uint_32 size) {
    br_uint_32 lo = addr - region_addr(r), hi = lo + size;

    if (lo < r->lo || lo > r->hi) {
        lo = r->lo;
    }
    if (hi > r->hi || hi < lo) {
        hi = r->hi;
    }
    if (lo < r->dirty_lo) {
        r->dirty_lo = lo;
    }
    if (hi > r->dirty_hi) {
        r->dirty_hi = hi;
    }
    r->host_valid = 0;
}

static tRegion* region_find(const br_uint_8* p, br_uint_32 size, int depth) {
    br_uint_32 lo, hi;
    tRegion* r;
    int w, i;

    for (w = 0; w < window_count; w++) {
        if (p >= window_host[w] && p + size <= window_host[w] + FR_WINDOW_SIZE) {
            break;
        }
    }
    if (w == window_count) {
        if (window_count == FR_WINDOWS || size > FR_WINDOW_SIZE - FR_WINDOW_ORIGIN) {
            fprintf(stderr, "fpgarast: out of buffer windows\n");
            abort();
        }
        window_host[w] = (br_uint_8*)p - FR_WINDOW_ORIGIN;
        window_count++;
    }
    lo = (br_uint_32)(p - window_host[w]);
    hi = lo + size;
    for (i = 0; i < region_count; i++) {
        r = &regions[i];
        if (r->window == w && lo < r->hi && hi > r->lo) {
            if (lo < r->lo || hi > r->hi) {
                // grows: start again from the host's copy
                region_download(r);
                r->fpga_valid = 0;
                r->lo = lo < r->lo ? lo : r->lo;
                r->hi = hi > r->hi ? hi : r->hi;
                r->dirty_lo = r->hi;
                r->dirty_hi = r->lo;
            }
            return r;
        }
    }
    if (region_count == MAX_REGIONS) {
        fprintf(stderr, "fpgarast: out of buffer regions\n");
        abort();
    }
    r = &regions[region_count++];
    r->window = w;
    r->lo = lo;
    r->hi = hi;
    r->dirty_lo = hi;
    r->dirty_hi = lo;
    r->depth = depth;
    r->host_valid = 1;
    r->fpga_valid = 0;
    return r;
}

// address of a render buffer's origin, with the FPGA's copy ready to draw into
static br_uint_32 buffer_addr(const struct render_buffer* buffer, int depth, tRegion** region) {
    const br_uint_8* p = buffer->base;
    tRegion* r = region_find(p, (br_uint_32)buffer->stride_b * buffer->height, depth);

    region_upload(r);
    *region = r;
    return region_addr(r) + (br_uint_32)(p - window_host[r->window]);
}

void FpgaRast_Sync(int flags) {
    int i;

    for (i = 0; i < region_count; i++) {
        tRegion* r = &regions[i];
        if (r->depth ? !(flags & (FR_SYNC_DEPTH_READ | FR_SYNC_DEPTH_WRITE)) : !(flags & FR_SYNC_COLOUR)) {
            continue;
        }
        region_download(r);
        if (!r->depth || (flags & FR_SYNC_DEPTH_WRITE)) {
            // the host may change it
            r->fpga_valid = 0;
        }
    }
    if (flags & FR_SYNC_SCENE_END) {
        sync_stats.scenes++;
    }
}

void FpgaRast_SyncDepthRead(const void* first, br_uint_32 bytes) {
    const br_uint_8* p = first;
    int i;

    for (i = 0; i < region_count; i++) {
        tRegion* r = &regions[i];
        const br_uint_8* host = window_host[r->window];
        br_uint_32 lo, hi;

        if (!r->depth || r->host_valid || backend == NULL || p + bytes <= host + r->dirty_lo || p >= host + r->dirty_hi) {
            continue;
        }
        // the FPGA's copy stays the one that counts, so nothing else changes
        lo = p > host + r->dirty_lo ? (br_uint_32)(p - host) : r->dirty_lo;
        hi = p + bytes < host + r->dirty_hi ? (br_uint_32)(p + bytes - host) : r->dirty_hi;
        backend->wait();
        backend->download(region_addr(r) + lo, window_host[r->window] + lo, hi - lo);
        sync_stats.waits++;
        sync_stats.downloads++;
        sync_stats.download_bytes += hi - lo;
    }
}

void FpgaRast_DepthWritten(const void* first, br_uint_32 bytes) {
    const br_uint_8* p = first;
    int i;

    for (i = 0; i < region_count; i++) {
        tRegion* r = &regions[i];
        const br_uint_8* host = window_host[r->window];

        if (!r->depth || backend == NULL || p + bytes <= host + r->lo || p >= host + r->hi) {
            continue;
        }
        if (r->host_valid) {
            // the host has all of it: the usual way
            r->fpga_valid = 0;
        } else {
            br_uint_32 lo = p > host + r->lo ? (br_uint_32)(p - host) : r->lo;
            br_uint_32 hi = p + bytes < host + r->hi ? (br_uint_32)(p + bytes - host) : r->hi;
            backend->wait();
            backend->upload(region_addr(r) + lo, host + lo, hi - lo);
            sync_stats.waits++;
            sync_stats.uploads++;
            sync_stats.upload_bytes += hi - lo;
        }
    }
}

static int colour_held;

void FpgaRast_SceneEnd(void) {
    if (!colour_held) {
        FpgaRast_Sync(FR_SYNC_COLOUR | FR_SYNC_SCENE_END);
    }
}

void FpgaRast_Hold(int hold) {
    if (!gPentprim_fpga) {
        return;
    }
    colour_held = hold;
    if (!hold) {
        FpgaRast_Sync(FR_SYNC_COLOUR | FR_SYNC_SCENE_END);
    }
}

int FpgaRast_Fill(void* pixels, br_uint_32 row_bytes, br_uint_32 rows, br_uint_32 stride, br_uint_32 pattern, int depth) {
    tRegion* r;

    if (!gPentprim_fpga || rows == 0 || row_bytes == 0 || row_bytes > FR_FILL_MAX || ((uintptr_t)pixels & 1)) {
        return 0;
    }
    r = region_find(pixels, (rows - 1) * stride + row_bytes, depth);
    if (backend == NULL) {
        // same memory: the model fills it
        fill_range(region_addr(r) + (br_uint_32)((br_uint_8*)pixels - window_host[r->window]), row_bytes, rows, stride, pattern);
        return 1;
    }
    if ((br_uint_32)((br_uint_8*)pixels - window_host[r->window]) == r->lo && (rows - 1) * stride + row_bytes == r->hi - r->lo && row_bytes == stride) {
        // all of it: whatever either side has does not matter any more
        r->fpga_valid = 1;
    } else {
        region_download(r);
        region_upload(r);
    }
    fill_range(region_addr(r) + (br_uint_32)((br_uint_8*)pixels - window_host[r->window]), row_bytes, rows, stride, pattern);
    region_drawn(r, region_addr(r) + (br_uint_32)((br_uint_8*)pixels - window_host[r->window]), (rows - 1) * stride + row_bytes);
    return 1;
}

static br_uint_32 heap_checksum(const br_uint_8* p, br_uint_32 size) {
    br_uint_32 sum = 2166136261u, i, v;

    for (i = 0; i + 4 <= size; i += 4) {
        memcpy(&v, p + i, 4);
        sum = (sum ^ v) * 16777619u;
        sum ^= sum >> 15;
    }
    return sum;
}

// lookups already done, by host pointer: triangles keep using the same few
// textures and tables
#define HEAP_RECENT 64
static struct {
    const void* host;
    br_uint_32 size, addr;
} heap_recent[HEAP_RECENT];

static br_uint_32 heap_lookup(const void* base, br_uint_32 size);

static inline br_uint_32 heap_addr(const void* base, br_uint_32 size) {
    const unsigned i = (unsigned)(((uintptr_t)base >> 6) ^ ((uintptr_t)base >> 12)) % HEAP_RECENT;

    if (heap_recent[i].host != base || heap_recent[i].size < size) {
        heap_recent[i].addr = heap_lookup(base, size);
        heap_recent[i].host = base;
        heap_recent[i].size = size;
    }
    return heap_recent[i].addr;
}

static br_uint_32 heap_lookup(const void* base, br_uint_32 size) {
    unsigned h = (unsigned)(((uintptr_t)base >> 4) * 2654435761u) % HEAP_HASH_SIZE;
    struct heap_entry* e;
    br_uint_32 checksum = 0;

    size = (size + 7) & ~7u;
    while (heap_hash[h].host != NULL && heap_hash[h].host != base) {
        h = (h + 1) % HEAP_HASH_SIZE;
    }
    e = &heap_hash[h];
    if (e->host != NULL && !e->stale && size <= e->size) {
        return e->addr;
    }
    if (e->host == NULL || size > e->size) {
        // new, or the memory now holds something bigger
        if (e->host == NULL && ++heap_hashed > HEAP_HASH_SIZE / 2) {
            fprintf(stderr, "fpgarast: too many textures\n");
            abort();
        }
        if (heap_count == HEAP_HASH_SIZE || heap_top + size > FR_HEAP_BASE + FR_HEAP_SIZE) {
            fprintf(stderr, "fpgarast: out of texture memory\n");
            abort();
        }
        e->host = base;
        e->addr = heap_top;
        e->size = size;
        heap_top += size;
        heap_list[heap_count++] = *e;
        if (backend != NULL) {
            checksum = heap_checksum(base, e->size);
        }
    } else if (backend != NULL) {
        // updated in place: most of the time nothing changed
        checksum = heap_checksum(base, e->size);
        if (checksum == e->checksum) {
            e->stale = 0;
            return e->addr;
        }
        // wait for what is still to be drawn from the old contents; the
        // rasteriser must forget its cached words
        {
            br_uint_32 w[2] = { FR_HEAD(FR_OP_FLUSH, 1, 0), FR_HEAD(FR_OP_NOP, 1, 0) };
            backend->wait();
            submit(w);
        }
    }
    e->stale = 0;
    e->checksum = checksum;
    if (backend != NULL) {
        backend->upload(e->addr, base, e->size);
        sync_stats.textures++;
        sync_stats.texture_bytes += e->size;
    }
    return e->addr;
}

void FpgaRast_Invalidate(const void* base) {
    unsigned h = (unsigned)(((uintptr_t)base >> 4) * 2654435761u) % HEAP_HASH_SIZE;

    if (!gPentprim_fpga || base == NULL) {
        return;
    }
    while (heap_hash[h].host != NULL && heap_hash[h].host != base) {
        h = (h + 1) % HEAP_HASH_SIZE;
    }
    if (heap_hash[h].host != NULL) {
        heap_hash[h].stale = 1;
    }
    memset(heap_recent, 0, sizeof(heap_recent));
}

// model: host memory behind an address. Buffer addresses stay valid across
// the whole window, heap addresses within their allocation.
static br_uint_8* host_ptr(br_uint_32 addr) {
    if (model_image != NULL) {
        return model_image + addr;
    }
    if (addr >= FR_HEAP_BASE) {
        int lo = 0, hi = heap_count - 1;
        while (lo < hi) {
            int mid = (lo + hi + 1) / 2;
            if (heap_list[mid].addr <= addr) {
                lo = mid;
            } else {
                hi = mid - 1;
            }
        }
        return (br_uint_8*)heap_list[lo].host + (addr - heap_list[lo].addr);
    }
    return window_host[(addr - FR_WINDOW_BASE) / FR_WINDOW_SIZE] + (addr - FR_WINDOW_BASE) % FR_WINDOW_SIZE;
}

/*
 * Emitter
 */

static br_uint_32 target_colour_stride, target_depth_stride;
static int target_valid;

/*
 * Development: traces for the RTL testbench (core/sim), without a backend.
 *
 * PENTPRIM_FPGA_TRACE=<dir> writes the memory image (mem.hex: buffers,
 * textures and tables as 64 bit words in $readmemh format) in front of
 * triangle number PENTPRIM_FPGA_TRACE_SKIP and the commands of the
 * PENTPRIM_FPGA_TRACE_COUNT triangles from there (cmds.hex), then exits.
 *
 * PENTPRIM_FPGA_REPLAY=<dir> does not start the game: it runs the software
 * model over that trace and writes the buffers it leaves to expect.hex.
 * PENTPRIM_FPGA_REPLAY_ONLY=<opcode> leaves out the other triangle types.
 */
static const char* trace_dir;
static long trace_skip, trace_count = 1000, trace_triangles;
static FILE* trace_cmds;
static FILE* trace_mem;
static int trace_heap_dumped;

static FILE* trace_open(const char* dir, const char* name, const char* mode) {
    char path[1024];
    FILE* f;

    snprintf(path, sizeof(path), "%s/%s", dir, name);
    f = fopen(path, mode);
    if (f == NULL) {
        fprintf(stderr, "fpgarast: cannot open %s\n", path);
        exit(1);
    }
    return f;
}

static void trace_dump_range(FILE* f, br_uint_32 addr, const br_uint_8* host, br_uint_32 size) {
    br_uint_32 i;
    int k;

    // whole 64 bit words: the bytes around an unaligned range come along
    host -= addr & 7;
    size = (size + (addr & 7) + 7) & ~7u;
    addr &= ~7u;
    fprintf(f, "@%x\n", addr >> 3);
    for (i = 0; i < size; i += 8) {
        for (k = 7; k >= 0; k--) {
            fprintf(f, "%02x", host[i + k]);
        }
        fputc('\n', f);
    }
}

static void trace_triangle(void) {
    if (trace_dir == NULL) {
        return;
    }
    if (trace_triangles == trace_skip + trace_count) {
        fclose(trace_cmds);
        fclose(trace_mem);
        fprintf(stderr, "fpgarast: trace of %ld triangles written to %s\n", trace_count, trace_dir);
        exit(0);
    }
    trace_triangles++;
}

static void submit(const br_uint_32* w) {
    if (trace_dir != NULL && trace_triangles > trace_skip) {
        int i;
        if (trace_cmds == NULL) {
            trace_mem = trace_open(trace_dir, "mem.hex", "w");
            for (i = 0; i < region_count; i++) {
                trace_dump_range(trace_mem, region_addr(&regions[i]) + regions[i].lo, window_host[regions[i].window] + regions[i].lo, regions[i].hi - regions[i].lo);
            }
            trace_cmds = trace_open(trace_dir, "cmds.hex", "w");
        }
        // textures and tables, including the ones first used inside the trace
        for (; trace_heap_dumped < heap_count; trace_heap_dumped++) {
            trace_dump_range(trace_mem, heap_list[trace_heap_dumped].addr, heap_list[trace_heap_dumped].host, heap_list[trace_heap_dumped].size);
        }
        for (i = 0; i < (int)FR_HEAD_WORDS(w[0]); i++) {
            fprintf(trace_cmds, "%08x\n", w[i]);
        }
    }
    if (backend != NULL) {
        backend->submit(w, FR_HEAD_WORDS(w[0]));
    } else {
        FpgaModel_Execute(w);
    }
}

static void emit_target(void) {
    br_uint_32 w[3];

    if (trace_dir != NULL && trace_triangles == trace_skip + 1 && trace_cmds == NULL) {
        // a trace starts with the target
        target_valid = 0;
    }
    if (target_valid && target_colour_stride == (br_uint_32)work.colour.stride_b && target_depth_stride == (br_uint_32)work.depth.stride_b) {
        return;
    }
    target_valid = 1;
    target_colour_stride = work.colour.stride_b;
    target_depth_stride = work.depth.stride_b;
    w[0] = FR_HEAD(FR_OP_TARGET, 3, 0);
    w[1] = target_colour_stride;
    w[2] = target_depth_stride;
    submit(w);
}

// A shade table has a row per intensity the material can have. The table
// itself may claim less (the game's car shadows point a one row table into
// the middle of a bigger one) and the rasteriser reads on regardless.
static br_uint_32 shade_table_addr(void) {
    br_uint_32 size = (br_uint_32)(work.index_base + work.index_range + 2) * 256;

    if (size < work.shade_table_size) {
        size = work.shade_table_size;
    }
    if (size > 0x10000) {
        size = 0x10000;
    }
    return heap_addr(work.shade_table, size);
}

// rows a triangle covers, one more than it draws: scan origins sit just in front of a row
static br_uint_32 triangle_rows(void) {
    return (workspace.topCount < 0 ? 0 : workspace.topCount + 1) + (workspace.bottomCount < 0 ? 0 : workspace.bottomCount + 1) + 1;
}

void FpgaRast_Tri(int flags, int pow2) {
    br_uint_32 w[FR_MAX_WORDS];
    tRegion *colour, *depth;
    int n = 16;

    trace_triangle();
    emit_target();
    w[1] = buffer_addr(&work.colour, 0, &colour) + workspace.scanAddress;
    w[2] = buffer_addr(&work.depth, 1, &depth) + workspace.depthAddress;
    w[3] = workspace.xm;
    w[4] = workspace.d_xm;
    if ((flags & (FR_TRI_I | FR_TRI_T)) == FR_TRI_I) {
        // the Gouraud setup keeps the major edge fraction elsewhere
        w[5] = work.main.f;
        w[6] = work.main.d_f;
    } else {
        w[5] = workspace.xm_f;
        w[6] = workspace.d_xm_f;
    }
    w[7] = workspace.x1;
    w[8] = workspace.d_x1;
    w[9] = workspace.x2;
    w[10] = workspace.d_x2;
    w[11] = (workspace.topCount & 0xffff) | ((br_uint_32)workspace.bottomCount << 16);
    w[12] = workspace.s_z;
    w[13] = workspace.d_z_x;
    w[14] = workspace.d_z_y_0;
    w[15] = workspace.d_z_y_1;
    if (!(flags & (FR_TRI_I | FR_TRI_T))) {
        w[n++] = workspace.colour;
    }
    if (flags & FR_TRI_I) {
        w[n++] = workspace.s_i;
        w[n++] = workspace.d_i_x;
        w[n++] = workspace.d_i_y_0;
        w[n++] = workspace.d_i_y_1;
    }
    if (flags & FR_TRI_T) {
        w[n++] = workspace.s_u;
        w[n++] = workspace.d_u_x;
        w[n++] = workspace.d_u_y_0;
        w[n++] = workspace.d_u_y_1;
        w[n++] = workspace.s_v;
        w[n++] = workspace.d_v_x;
        w[n++] = workspace.d_v_y_0;
        w[n++] = workspace.d_v_y_1;
        w[n++] = heap_addr(work.texture.base, 1u << (2 * pow2));
        if (flags & FR_TRI_I) {
            w[n++] = shade_table_addr();
        }
    }
    w[0] = FR_HEAD(FR_OP_TRI, n, flags | (pow2 << 4));
    submit(w);
    region_drawn(colour, w[1], triangle_rows() * work.colour.stride_b);
    region_drawn(depth, w[2], triangle_rows() * work.depth.stride_b);
}

void FpgaRast_ATri(int right_to_left) {
    br_uint_32 w[FR_MAX_WORDS];
    tRegion *colour, *depth;

    trace_triangle();
    emit_target();
    w[0] = FR_HEAD(FR_OP_ATRI, 35, right_to_left ? FR_TRI_RL : 0);
    w[1] = buffer_addr(&work.colour, 0, &colour) + workspace.scanAddress;
    w[2] = buffer_addr(&work.depth, 1, &depth) + workspace.depthAddress;
    w[3] = workspace.xm;
    w[4] = workspace.d_xm;
    w[5] = workspace.xm_f;
    w[6] = workspace.d_xm_f;
    w[7] = workspace.x1;
    w[8] = workspace.d_x1;
    w[9] = workspace.x2;
    w[10] = workspace.d_x2;
    w[11] = (workspace.topCount & 0xffff) | ((br_uint_32)workspace.bottomCount << 16);
    w[12] = workspace.s_z;
    w[13] = workspace.d_z_x;
    w[14] = workspace.d_z_y_0;
    w[15] = workspace.d_z_y_1;
    w[16] = workspaceA.su;
    w[17] = workspaceA.dux;
    w[18] = workspaceA.duy0;
    w[19] = workspaceA.duy1;
    w[20] = workspaceA.sv;
    w[21] = workspaceA.svf;
    w[22] = workspaceA.dvx;
    w[23] = workspaceA.dvxc;
    w[24] = workspaceA.dvxf;
    w[25] = workspaceA.dvy0;
    w[26] = workspaceA.dvy0c;
    w[27] = workspaceA.dvy1;
    w[28] = workspaceA.dvy1c;
    w[29] = workspaceA.dvy0f;
    w[30] = workspaceA.dvy1f;
    w[31] = workspaceA.uUpperBound;
    w[32] = workspaceA.vUpperBound;
    w[33] = work.texture.size;
    w[34] = heap_addr(work.texture.base, work.texture.size);
    submit(w);
    region_drawn(colour, w[1], triangle_rows() * work.colour.stride_b);
    region_drawn(depth, w[2], triangle_rows() * work.depth.stride_b);
}

int FpgaRast_Fog(void* colour_pixels, void* depth_pixels, int width, int rows, br_uint_32 colour_stride, br_uint_32 depth_stride,
    br_uint_32 start, br_uint_32 too_near, int shift, const void* table, br_uint_32 table_size) {
    br_uint_32 w[10];
    tRegion *colour, *depth;

    if (!gPentprim_fpga || width < 1 || width > 1024 || rows < 1 || ((uintptr_t)depth_pixels & 1) || table_size > 0x10000) {
        return 0;
    }
    colour = region_find(colour_pixels, (rows - 1) * colour_stride + width, 0);
    depth = region_find(depth_pixels, (rows - 1) * depth_stride + 2 * width, 1);
    region_upload(colour);
    region_upload(depth);
    w[0] = FR_HEAD(FR_OP_FOG, 9, 0);
    w[1] = region_addr(colour) + (br_uint_32)((br_uint_8*)colour_pixels - window_host[colour->window]);
    w[2] = region_addr(depth) + (br_uint_32)((br_uint_8*)depth_pixels - window_host[depth->window]);
    w[3] = width | (rows << 16);
    w[4] = colour_stride;
    w[5] = depth_stride;
    w[6] = (start & 0xffff) | (too_near << 16);
    w[7] = shift;
    w[8] = heap_addr(table, table_size);
    w[9] = FR_HEAD(FR_OP_NOP, 1, 0);
    submit(w);
    // the strides are no longer those of the target
    target_valid = 0;
    region_drawn(colour, w[1], (rows - 1) * colour_stride + width);
    return 1;
}

void FpgaRast_PTri(int backwards, int lit, int bits) {
    static int second;
    br_uint_32 w[FR_MAX_WORDS];
    tRegion *colour, *depth;
    int n = 26;

    // the triangle functions call once per trapezium; the command covers both
    if (second) {
        second = 0;
        return;
    }
    second = 1;

    trace_triangle();
    emit_target();
    w[1] = buffer_addr(&work.colour, 0, &colour) + workspace.scanAddress;
    w[2] = buffer_addr(&work.depth, 1, &depth) + workspace.depthAddress;
    w[3] = workspace.xm;
    w[4] = workspace.d_xm;
    w[5] = workspace.x1;
    w[6] = workspace.d_x1;
    w[7] = workspace.x2;
    w[8] = workspace.d_x2;
    w[9] = (workspace.topCount & 0xffff) | ((br_uint_32)workspace.bottomCount << 16);
    w[10] = workspace.s_z;
    w[11] = workspace.d_z_y_0;
    w[12] = workspace.d_z_y_1;
    w[13] = work.tsl.dz;
    w[14] = work.pq.current;
    w[15] = work.pq.grad_x;
    w[16] = work.pq.d_nocarry;
    w[17] = work.pq.d_carry;
    w[18] = work.pu.current;
    w[19] = work.pu.grad_x;
    w[20] = work.pu.d_nocarry;
    w[21] = work.pv.current;
    w[22] = work.pv.grad_x;
    w[23] = work.pv.d_nocarry;
    w[24] = work.tsl.source;
    w[25] = heap_addr(work.texture.base, 1u << (2 * bits));
    if (lit) {
        w[n++] = workspace.s_i;
        w[n++] = workspace.d_i_y_0;
        w[n++] = workspace.d_i_y_1;
        w[n++] = work.tsl.di;
        w[n++] = shade_table_addr();
    }
    w[0] = FR_HEAD(FR_OP_PTRI, n, (backwards ? FR_PTRI_B : 0) | (lit ? FR_PTRI_I : 0) | (bits << 4));
    submit(w);
    region_drawn(colour, w[1], triangle_rows() * work.colour.stride_b);
    region_drawn(depth, w[2], triangle_rows() * work.depth.stride_b);
}

/*
 * Software model
 *
 * Everything below works from command words only. The arithmetic follows
 * the loops in fastprim.c (which are verified against the original code),
 * restructured the way the hardware does it: one edge walker per triangle
 * family and one pixel datapath per mode.
 */

static br_uint_32 model_colour_stride, model_depth_stride;

static inline br_uint_32 ror16(br_uint_32 v) {
    return (v >> 16) | (v << 16);
}

typedef struct {
    br_uint_32 scan, zscan;
    br_uint_32 xm, d_xm, xm_f, d_xm_f;
    br_uint_32 s_z, d_z_x, d_z_y_0, d_z_y_1;
    br_uint_32 s_i, d_i_x, d_i_y_0, d_i_y_1;
    br_uint_32 s_u, d_u_x, d_u_y_0, d_u_y_1;
    br_uint_32 s_v, d_v_x, d_v_y_0, d_v_y_1;
    br_uint_8 colour;
    int flags, pow2;
    const br_uint_8* texture;
    const br_uint_8* shade;
} tTri;

// one scanline of n+1 pixels (n is negative or zero going right, positive
// or zero going left), ZT and ZTI: draw_zt_pow2 / draw_zti_pow2
static void tri_span_textured(const tTri* t, br_uint_8* line, br_uint_16* zline, br_int_32 n) {
    const int rl = t->flags & FR_TRI_RL;
    const br_uint_32 u_mask = (1u << t->pow2) - 1;
    const br_uint_32 v_mask = u_mask << t->pow2;
    const int v_shift = 16 - t->pow2;
    br_uint_32 z = ror16(t->s_z);
    br_uint_32 i = t->s_i;
    br_uint_32 u = t->s_u;
    br_uint_32 v = t->s_v;

    for (;;) {
        if ((br_uint_16)z <= zline[n]) {
            const br_uint_8 texel = t->texture[((v >> v_shift) & v_mask) | ((u >> 16) & u_mask)];
            if (texel != 0) {
                zline[n] = (br_uint_16)z;
                line[n] = (t->flags & FR_TRI_I) ? t->shade[(i & 0xff0000u) >> 8 | texel] : texel;
            }
        }
        if (!rl) {
            z += t->d_z_x;
            z += z < t->d_z_x;
            i += t->d_i_x;
            u += t->d_u_x;
            v += t->d_v_x;
            if (++n > 0) {
                break;
            }
        } else {
            const br_uint_32 borrow = z < t->d_z_x;
            z -= t->d_z_x + borrow;
            i -= t->d_i_x;
            u -= t->d_u_x;
            v -= t->d_v_x;
            if (--n < 0) {
                break;
            }
        }
    }
}

// flat (draw_z) and Gouraud (trapezium_zi): 32 bit depth compare against
// the previous pixel's upper half, z carry applied one pixel late
static void tri_span_plain(const tTri* t, br_uint_8* line, br_uint_16* zline, br_int_32 n) {
    const int rl = t->flags & FR_TRI_RL;
    const int gouraud = t->flags & FR_TRI_I;
    br_uint_32 z = ror16(t->s_z);
    br_uint_32 i = ror16(t->s_i);
    br_uint_32 carry = 0, tmp;

    if (gouraud) {
        // step the intensity back one pixel, its carry runs on into the loop
        if (!rl) {
            carry = i < t->d_i_x;
            i -= t->d_i_x;
            tmp = i;
            i = tmp - carry;
            carry = tmp < carry;
        } else {
            i += t->d_i_x;
            carry = i < t->d_i_x;
            tmp = i + carry;
            carry = tmp < i;
            i = tmp;
        }
    }
    for (;;) {
        const br_uint_32 prev = z;
        if (gouraud) {
            if (!rl) {
                z += carry;
                i += t->d_i_x;
                carry = i < t->d_i_x;
                i += carry;
            } else {
                z -= carry;
                carry = i < t->d_i_x;
                i -= t->d_i_x;
                i -= carry;
            }
        } else if (!rl) {
            z += carry;
        } else {
            z -= carry;
        }
        if (z <= ((prev & 0xffff0000u) | zline[n])) {
            zline[n] = (br_uint_16)z;
            line[n] = gouraud ? (br_uint_8)i : t->colour;
        }
        if (!rl) {
            z += t->d_z_x;
            carry = z < t->d_z_x;
            if (++n > 0) {
                break;
            }
        } else {
            carry = z < t->d_z_x;
            z -= t->d_z_x;
            if (--n < 0) {
                break;
            }
        }
    }
}

static void tri_half(tTri* t, br_uint_32 minor, const br_uint_32 d_minor, br_int_32 count) {
    const int rl = t->flags & FR_TRI_RL;
    const int textured = t->flags & FR_TRI_T;
    const int flat = !(t->flags & (FR_TRI_T | FR_TRI_I));

    if (count < 0) {
        return;
    }
    do {
        const br_uint_32 x = minor >> 16;
        const br_int_32 n = (br_int_32)((t->xm >> 16) - x);

        if (rl ? n >= 0 : n <= 0) {
            br_uint_8* const line = host_ptr(t->scan + x);
            br_uint_16* const zline = (br_uint_16*)host_ptr(t->zscan + 2 * x);
            if (textured) {
                tri_span_textured(t, line, zline, n);
            } else {
                tri_span_plain(t, line, zline, n);
            }
        }

        // per line: the major edge fraction carry selects the "carry" deltas
        t->xm_f += t->d_xm_f;
        if (t->xm_f < t->d_xm_f) {
            t->s_z += t->d_z_y_1;
            if (flat) {
                t->s_z += t->s_z < t->d_z_y_1;
            }
            t->s_i += t->d_i_y_1;
            t->s_u += t->d_u_y_1;
            t->s_v += t->d_v_y_1;
        } else {
            t->s_z += t->d_z_y_0;
            if (flat) {
                t->s_z += t->s_z < t->d_z_y_0;
            }
            t->s_i += t->d_i_y_0;
            t->s_u += t->d_u_y_0;
            t->s_v += t->d_v_y_0;
        }
        t->scan += model_colour_stride;
        t->zscan += model_depth_stride;
        minor += d_minor;
        t->xm += t->d_xm;
    } while (--count >= 0);
}

static void model_tri(const br_uint_32* w) {
    tTri t;
    int n = 16;

    memset(&t, 0, sizeof(t));
    t.flags = FR_HEAD_FLAGS(w[0]);
    t.pow2 = FR_FLAGS_POW2(t.flags);
    t.scan = w[1];
    t.zscan = w[2];
    t.xm = w[3];
    t.d_xm = w[4];
    t.xm_f = w[5];
    t.d_xm_f = w[6];
    t.s_z = w[12];
    t.d_z_x = w[13];
    t.d_z_y_0 = w[14];
    t.d_z_y_1 = w[15];
    if (!(t.flags & (FR_TRI_I | FR_TRI_T))) {
        t.colour = (br_uint_8)w[n++];
    }
    if (t.flags & FR_TRI_I) {
        t.s_i = w[n++];
        t.d_i_x = w[n++];
        t.d_i_y_0 = w[n++];
        t.d_i_y_1 = w[n++];
    }
    if (t.flags & FR_TRI_T) {
        t.s_u = w[n++];
        t.d_u_x = w[n++];
        t.d_u_y_0 = w[n++];
        t.d_u_y_1 = w[n++];
        t.s_v = w[n++];
        t.d_v_x = w[n++];
        t.d_v_y_0 = w[n++];
        t.d_v_y_1 = w[n++];
        t.texture = host_ptr(w[n++]);
        if (t.flags & FR_TRI_I) {
            t.shade = host_ptr(w[n++]);
        }
    }
    tri_half(&t, w[7], w[8], (br_int_16)(w[11] & 0xffff));
    tri_half(&t, w[9], w[10], (br_int_16)(w[11] >> 16));
}

/*
 * Perspective: zpt_trapezium / zpt_scanline in fastprim.c.
 *
 * u and v are numerators against the denominator q. At the start of every
 * scanline drawn they are brought into 0..q by stepping the texel (tu, tv),
 * which also adjusts their gradients, exactly as the original does. Along
 * the scanline the original keeps stepping an error term; the hardware
 * divides instead: the texel of a pixel is the texel of the scanline start
 * plus floor(u/q), floor(v/q), which is what the error term arrives at as
 * long as q is positive (it is, inside a triangle in front of the viewer).
 */
#define PTRI_MAX_STEPS 65536

// The quotient is a 16 bit signed number in the hardware and saturates
// (thousands of texels along one scanline do not happen)
static inline br_int_32 floor_div(br_int_64 a, br_int_32 q) {
    br_int_64 d;

    if (q <= 0) {
        return 0;
    }
    d = a >= 0 ? a / q : -((-a + q - 1) / q);
    return (br_int_32)(d < -32768 ? -32768 : d > 32767 ? 32767 : d);
}

// bring a numerator into 0..q, stepping the texel coordinate
static inline void ptri_normalise(br_uint_32* num, br_uint_32* grad, br_uint_32* nocarry, br_uint_32* texel,
    const br_uint_32 q, const br_uint_32 q_grad, const br_uint_32 q_nocarry) {
    int steps = PTRI_MAX_STEPS;

    if ((br_int_32)q <= 0) {
        return;
    }
    while ((br_int_32)*num >= (br_int_32)q && steps-- > 0) {
        (*texel)++;
        *grad -= q_grad;
        *nocarry -= q_nocarry;
        *num -= q;
    }
    while ((br_int_32)*num < 0 && steps-- > 0) {
        (*texel)--;
        *grad += q_grad;
        *nocarry += q_nocarry;
        *num += q;
    }
}

static void model_ptri(const br_uint_32* w) {
    const int flags = FR_HEAD_FLAGS(w[0]);
    const int backwards = flags & FR_PTRI_B;
    const int lit = flags & FR_PTRI_I;
    const int bits = FR_FLAGS_POW2(flags);
    const br_uint_32 mask = (1u << bits) - 1;
    br_uint_32 scan = w[1], zscan = w[2];
    br_uint_32 main_i = w[3];
    const br_uint_32 main_d = w[4];
    br_uint_32 s_z = w[10];
    const br_uint_32 d_z_y_0 = w[11], d_z_y_1 = w[12], dz = w[13];
    br_uint_32 q = w[14];
    const br_uint_32 q_grad = w[15], q_nocarry = w[16], q_carry = w[17];
    br_uint_32 u = w[18], u_grad = w[19], u_nocarry = w[20];
    br_uint_32 v = w[21], v_grad = w[22], v_nocarry = w[23];
    br_uint_32 tu = w[24] & mask, tv = (w[24] >> bits) & mask;
    const br_uint_8* const texture = host_ptr(w[25]);
    br_uint_32 s_i = lit ? w[26] : 0;
    const br_uint_32 d_i_y_0 = lit ? w[27] : 0, d_i_y_1 = lit ? w[28] : 0, di = lit ? w[29] : 0;
    const br_uint_8* const shade = lit ? host_ptr(w[30]) : NULL;
    int half;

    for (half = 0; half < 2; half++) {
        br_uint_32 top_i = half ? w[7] : w[5];
        const br_uint_32 top_d = half ? w[8] : w[6];
        br_int_32 count = (br_int_16)(half ? w[9] >> 16 : w[9] & 0xffff);

        if (count < 0) {
            continue;
        }
        do {
            const br_int_32 x_end = top_i >> 16;
            const br_int_32 x_start = main_i & 0xffff;

            if (backwards ? x_start >= x_end : x_start <= x_end) {
                br_uint_8* dest = host_ptr(scan + x_start);
                br_uint_16* zdest = (br_uint_16*)host_ptr(zscan + 2 * x_start);
                br_int_32 n = backwards ? x_start - x_end : x_end - x_start;
                br_uint_32 z = ror16(s_z);
                br_uint_32 i = s_i;
                br_uint_32 pq = q;
                br_int_64 pu, pv;

                ptri_normalise(&u, &u_grad, &u_nocarry, &tu, q, q_grad, q_nocarry);
                ptri_normalise(&v, &v_grad, &v_nocarry, &tv, q, q_grad, q_nocarry);
                pu = (br_int_32)u;
                pv = (br_int_32)v;
                for (;;) {
                    if ((br_uint_16)z <= *zdest) {
                        const br_uint_32 pu_texel = (tu + (br_uint_32)floor_div(pu, (br_int_32)pq)) & mask;
                        const br_uint_32 pv_texel = (tv + (br_uint_32)floor_div(pv, (br_int_32)pq)) & mask;
                        const br_uint_8 texel = texture[(pv_texel << bits) | pu_texel];
                        if (texel != 0) {
                            *zdest = (br_uint_16)z;
                            *dest = lit ? shade[((i >> 8) & 0xff00) | texel] : texel;
                        }
                    }
                    if (n-- == 0) {
                        break;
                    }
                    if (!backwards) {
                        dest++;
                        zdest++;
                        z += dz;
                        z += z < dz;
                        i += di;
                        pq += q_grad;
                        pu += (br_int_32)u_grad;
                        pv += (br_int_32)v_grad;
                    } else {
                        const br_uint_32 borrow = z < dz;
                        dest--;
                        zdest--;
                        z -= dz + borrow;
                        i -= di;
                        pq -= q_grad;
                        pu -= (br_int_32)u_grad;
                        pv -= (br_int_32)v_grad;
                    }
                }
            }

            // next scanline: the major edge fraction carry selects the deltas
            scan += model_colour_stride;
            zscan += model_depth_stride;
            {
                const br_uint_32 next = main_i + main_d;
                if (next < main_d) {
                    main_i = next + 1;
                    q += q_carry;
                    s_z += d_z_y_1;
                    s_i += d_i_y_1;
                    // u and v have no carry delta of their own: nocarry + one step along x
                    u += u_nocarry + u_grad;
                    v += v_nocarry + v_grad;
                } else {
                    main_i = next;
                    q += q_nocarry;
                    s_z += d_z_y_0;
                    s_i += d_i_y_0;
                    u += u_nocarry;
                    v += v_nocarry;
                }
            }
            top_i += top_d;
        } while (--count >= 0);
    }
}

/*
 * Arbitrary width texture: DRAW_ZT_I8 and PER_SCAN_ZT in zb8awtm.c.
 * The texel is texture[v + (u >> 16)], with v the byte offset of a texture
 * row. Both wrap by repeated correction, first up from below zero, then down
 * from the upper bound, in that order and without looking back.
 */
#define ATRI_MAX_WRAPS 65536

static inline br_uint_32 atri_wrap_v(br_uint_32 v, br_uint_32 bound, br_uint_32 size) {
    int n = ATRI_MAX_WRAPS;

    while ((br_int_32)v < 0 && n-- > 0) {
        v += size;
    }
    while (v >= bound && n-- > 0) {
        v -= size;
    }
    return v;
}

static inline br_uint_32 atri_wrap_u(br_uint_32 u, br_uint_32 bound) {
    int n = ATRI_MAX_WRAPS;

    while ((br_int_32)u < 0 && n-- > 0) {
        u += bound;
    }
    while ((br_int_32)u >= (br_int_32)bound && n-- > 0) {
        u -= bound;
    }
    return u;
}

static void model_atri(const br_uint_32* w) {
    const int rl = FR_HEAD_FLAGS(w[0]) & FR_TRI_RL;
    br_uint_32 scan = w[1], zscan = w[2];
    br_uint_32 xm = w[3], xm_f = w[5];
    const br_uint_32 d_xm = w[4], d_xm_f = w[6];
    br_uint_32 s_z = w[12];
    const br_uint_32 d_z_x = w[13], d_z_y_0 = w[14], d_z_y_1 = w[15];
    br_uint_32 su = w[16];
    const br_uint_32 dux = w[17], duy0 = w[18], duy1 = w[19];
    br_uint_32 sv = w[20], svf = w[21];
    const br_uint_32 dvx = w[22], dvxc = w[23], dvxf = w[24];
    const br_uint_32 dvy0 = w[25], dvy0c = w[26], dvy1 = w[27], dvy1c = w[28], dvy0f = w[29], dvy1f = w[30];
    const br_uint_32 u_bound = w[31], v_bound = w[32], size = w[33];
    const br_uint_8* const texture = host_ptr(w[34]);
    int half;

    for (half = 0; half < 2; half++) {
        br_uint_32 minor = half ? w[9] : w[7];
        const br_uint_32 d_minor = half ? w[10] : w[8];
        br_int_32 count = (br_int_16)(half ? w[11] >> 16 : w[11] & 0xffff);

        if (count < 0) {
            continue;
        }
        do {
            const br_uint_32 x = minor >> 16;
            br_int_32 n = (br_int_32)((xm >> 16) - x);

            if (rl ? n >= 0 : n <= 0) {
                br_uint_8* const line = host_ptr(scan + x);
                br_uint_16* const zline = (br_uint_16*)host_ptr(zscan + 2 * x);
                br_uint_32 v = sv, c_v = svf, c_u = su, c_z = s_z;
                br_uint_32 u = su >> 16;

                for (;;) {
                    const br_uint_16 z = (br_uint_16)(c_z >> 16);
                    if (z <= zline[n]) {
                        const br_uint_8 texel = texture[(br_uint_32)(v + u)];
                        if (texel != 0) {
                            zline[n] = z;
                            line[n] = texel;
                        }
                    }
                    if (!rl) {
                        const br_uint_32 t = c_v + dvxf;
                        v += t < c_v ? dvxc : dvx;
                        c_v = t;
                        c_u += dux;
                        c_z += d_z_x;
                    } else {
                        v -= c_v < dvxf ? dvxc : dvx;
                        c_v -= dvxf;
                        c_u -= dux;
                        c_z -= d_z_x;
                    }
                    v = atri_wrap_v(v, v_bound, size);
                    c_u = atri_wrap_u(c_u, u_bound);
                    u = (br_uint_32)((br_int_32)c_u >> 16);
                    if (!rl ? ++n > 0 : --n < 0) {
                        break;
                    }
                }
            }

            // per line: the major edge fraction carry selects the "carry"
            // deltas, the carry out of the v fraction a second set for v
            {
                const br_uint_32 f = xm_f + d_xm_f;
                const int carry = f < xm_f;
                const br_uint_32 vf = svf + (carry ? dvy1f : dvy0f);
                const int v_carry = vf < svf;

                xm_f = f;
                s_z += carry ? d_z_y_1 : d_z_y_0;
                su = atri_wrap_u(su + (carry ? duy1 : duy0), u_bound);
                svf = vf;
                sv = atri_wrap_v(sv + (carry ? (v_carry ? dvy1c : dvy1) : (v_carry ? dvy0c : dvy0)), v_bound, size);
            }
            scan += model_colour_stride;
            zscan += model_depth_stride;
            minor += d_minor;
            xm += d_xm;
        } while (--count >= 0);
    }
}

static void model_fog(const br_uint_32* w) {
    const br_uint_32 width = w[3] & 0xffff, rows = w[3] >> 16;
    const br_uint_16 start = (br_uint_16)w[6], too_near = (br_uint_16)(w[6] >> 16);
    const br_int_32 shift = (br_int_32)w[7];
    const br_uint_8* const table = host_ptr(w[8]);
    br_uint_32 row, x;

    model_colour_stride = w[4];
    model_depth_stride = w[5];
    for (row = 0; row < rows; row++) {
        br_uint_8* const colour = host_ptr(w[1] + row * w[4]);
        const br_uint_16* const depth = (br_uint_16*)host_ptr(w[2] + row * w[5]);

        for (x = 0; x < width; x++) {
            const br_uint_16 d = (br_uint_16)(depth[x] - start);
            if (depth[x] != 0xffff && d < too_near) {
                const br_uint_16 index = (br_uint_16)(shift > 0 ? d << shift : shift < 0 ? d >> -shift : d) & 0xff00;
                colour[x] = table[index | colour[x]];
            }
        }
    }
}

static void model_fill(const br_uint_32* w) {
    br_uint_32 addr = w[1], row, i;

    for (row = 0; row < w[3]; row++) {
        br_uint_8* p = host_ptr(addr);
        for (i = 0; i < w[2]; i++) {
            p[i] = (br_uint_8)(((addr + i) & 1) ? w[5] >> 8 : w[5]);
        }
        addr += w[4];
    }
}

void FpgaModel_Execute(const br_uint_32* w) {
    switch (FR_HEAD_OP(w[0])) {
    case FR_OP_TARGET:
        model_colour_stride = w[1];
        model_depth_stride = w[2];
        break;
    case FR_OP_TRI:
        model_tri(w);
        break;
    case FR_OP_PTRI:
        model_ptri(w);
        break;
    case FR_OP_FILL:
        model_fill(w);
        break;
    case FR_OP_ATRI:
        model_atri(w);
        break;
    case FR_OP_FOG:
        model_fog(w);
        break;
    }
}

/*
 * PENTPRIM_FPGA=2: the software model behind the backend interface, working
 * on an image of the rasteriser's memory like the hardware does. The game
 * then only sees what is downloaded, so a missing FpgaRast_Sync shows in the
 * picture.
 */
static void image_submit(const br_uint_32* w, int words) {
    (void)words;
    FpgaModel_Execute(w);
}

static void image_wait(void) {
}

static void image_upload(br_uint_32 addr, const void* host, br_uint_32 size) {
    memcpy(model_image + addr, host, size);
}

static void image_download(br_uint_32 addr, void* host, br_uint_32 size) {
    memcpy(host, model_image + addr, size);
}

static const tFpgaRast_backend image_backend = { image_submit, image_wait, image_upload, image_download };

static void sync_summary(void) {
    const double n = sync_stats.scenes ? (double)sync_stats.scenes : 1;

    fprintf(stderr, "fpgarast: %ld scenes; per scene %.2f uploads (%.0f bytes), %.2f fills, %.2f downloads (%.0f bytes), %.2f waits; %ld texture uploads (%ld bytes)\n",
        sync_stats.scenes, sync_stats.uploads / n, sync_stats.upload_bytes / n, sync_stats.fills / n,
        sync_stats.downloads / n, sync_stats.download_bytes / n, sync_stats.waits / n, sync_stats.textures, sync_stats.texture_bytes);
}

void FpgaRast_SetBackend(const tFpgaRast_backend* b) {
    int i;

    // whatever the FPGA has is taken over by the host first
    FpgaRast_Sync(FR_SYNC_COLOUR | FR_SYNC_DEPTH_WRITE);
    if (backend != NULL && b == NULL) {
        sync_summary();
    }
    backend = b;
    for (i = 0; i < region_count; i++) {
        regions[i].fpga_valid = 0;
    }
    for (i = 0; i < HEAP_HASH_SIZE; i++) {
        heap_hash[i].stale = 1;
    }
    memset(heap_recent, 0, sizeof(heap_recent));
    target_valid = 0;
}

static void replay(const char* dir) {
    static struct { br_uint_32 addr, size; } ranges[4096];
    int range_count = 0, i, n;
    br_uint_32 addr = 0, w[256];
    char line[64];
    FILE* f;

    model_image = calloc(1, FR_HEAP_BASE + FR_HEAP_SIZE);
    f = trace_open(dir, "mem.hex", "r");
    while (fgets(line, sizeof(line), f) != NULL) {
        if (line[0] == '@') {
            addr = (br_uint_32)strtoul(line + 1, NULL, 16) << 3;
            ranges[range_count].addr = addr;
            ranges[range_count++].size = 0;
        } else if (range_count > 0 && addr + 8 <= FR_HEAP_BASE + FR_HEAP_SIZE) {
            const unsigned long long v = strtoull(line, NULL, 16);
            memcpy(model_image + addr, &v, 8);
            addr += 8;
            ranges[range_count - 1].size += 8;
        }
    }
    fclose(f);

    f = trace_open(dir, "cmds.hex", "r");
    n = 0;
    while (fgets(line, sizeof(line), f) != NULL) {
        w[n++] = (br_uint_32)strtoul(line, NULL, 16);
        if (n == (int)FR_HEAD_WORDS(w[0])) {
            const char* only = getenv("PENTPRIM_FPGA_REPLAY_ONLY");
            if (only == NULL || FR_HEAD_OP(w[0]) == FR_OP_TARGET || (int)FR_HEAD_OP(w[0]) == atoi(only)) {
                FpgaModel_Execute(w);
            }
            n = 0;
        }
    }
    fclose(f);

    f = trace_open(dir, "expect.hex", "w");
    for (i = 0; i < range_count; i++) {
        if (ranges[i].addr < FR_HEAP_BASE) {
            trace_dump_range(f, ranges[i].addr, model_image + ranges[i].addr, ranges[i].size);
        }
    }
    fclose(f);
    fprintf(stderr, "fpgarast: replayed %s\n", dir);
}

__attribute__((constructor)) static void fpgarast_init(void) {
    const char* env = getenv("PENTPRIM_FPGA_REPLAY");

    if (env != NULL) {
        replay(env);
        exit(0);
    }
    env = getenv("PENTPRIM_FPGA");
    if (env != NULL && env[0] == '2') {
        model_image = calloc(1, FR_HEAP_BASE + FR_HEAP_SIZE);
        backend = &image_backend;
        atexit(sync_summary);
    }
    trace_dir = getenv("PENTPRIM_FPGA_TRACE");
    env = getenv("PENTPRIM_FPGA_TRACE_SKIP");
    if (env != NULL) {
        trace_skip = atol(env);
    }
    env = getenv("PENTPRIM_FPGA_TRACE_COUNT");
    if (env != NULL) {
        trace_count = atol(env);
    }
}
