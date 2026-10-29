/*
 * Copyright © 2021 Collabora Ltd.
 *
 * Derived from tu_device.c which is:
 * Copyright © 2016 Red Hat.
 * Copyright © 2016 Bas Nieuwenhuizen
 * Copyright © 2015 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * ---------------------------------------------------------------------
 * kbase JM (Job Manager) job submission -- self-contained, no shared
 * mesa/kbase headers
 * ---------------------------------------------------------------------
 *
 * Earlier versions of this file pulled the kbase ioctl definitions in
 * from shared headers (kbase_jm.h / kbase_kmod.h / mali_base_jm_kernel.h
 * / mali_base_csf_kernel.h, ...). In this tree those headers step on each
 * other in ways that are very sensitive to include order and to exactly
 * which other file in the same binary pulled which one in first (macro
 * and struct-tag collisions between the JM and CSF variants, a header
 * that silently produced no declarations for reasons we couldn't pin
 * down, etc). Chasing that across several rounds of CI cost more time
 * than the kbase ioctl surface used here is actually worth: this file
 * only needs 2 ioctls (VERSION_CHECK, JOB_SUBMIT) and reads one small,
 * fixed-layout event struct.
 *
 * So: everything this file needs from the kbase uAPI is declared right
 * here, under a "panvk_kbase_" prefix that cannot collide with anything
 * any other header in this translation unit defines. No external kbase
 * header is included.
 *
 * IMPORTANT -- layout and stride are two different things on this DDK
 * (r54p1, UAPI 11.46). The atom that the kernel parses is
 * `struct base_jd_atom_v2`, byte for byte (56 bytes, `jc` at offset 0,
 * `atom_number` at 40, `core_req` at 44) -- but KBASE_IOCTL_JOB_SUBMIT
 * insists on `stride == 64`. An earlier revision of this file passed the
 * 56-byte struct with stride 56, got EINVAL, and concluded that the
 * kernel wants the 64-byte "v3" layout with `seq_nr` first. It doesn't:
 * the fix for the EINVAL is the stride, not the layout. Submitting the v3
 * layout makes the ioctl succeed (stride 64) but the kernel then reads
 * every field from the wrong offset: `atom_number` comes out as 0 (it
 * reads our `nr_extres`), `core_req` comes out as 0 (it reads our
 * `pre_dep`), i.e. a dependency-only atom, which completes at once with
 * BASE_JD_EVENT_DONE without ever reaching the GPU. That is exactly the
 * "DONE, event atom_number 0, job header exception_status still 0"
 * symptom. Verified on the same GPU/DDK (Mali-G57 MC2, r54p1) by a raw
 * WRITE_VALUE harness: v2 layout + stride 64 executes, and
 * panvk-g99-jm's raw-jm-progress.md documents the same 56/64 finding.
 *
 * Because that shape is a property of the DDK and not something to hard-code,
 * the layout actually used is detected at run time: a NULL job is submitted
 * in the v2 shape and then in the v3 (seq_nr-first) shape, and the first one
 * the job manager executes wins (see panvk_kbase_atom_layout_get()).
 * PANVK_KBASE_ATOM_LAYOUT=v2|v3 forces one. The shape makes no difference to
 * performance: `seq_nr` is only a grouping label for tracing, and both shapes
 * go through the same JOB_SUBMIT path with the same single atom per ioctl.
 *
 * IMPORTANT #2 -- there is no per-atom job-slot override, don't invent one:
 * with the (wrongly v3-shaped, see above) atom, JOB_SUBMIT was *accepted*
 * but the submitted atom then sat forever with no completion event and no
 * fault event either, until our own watchdog timed it out ("device is
 * either stuck or its watchdog didn't fire"). An earlier revision of
 * this file "fixed" that by adding a `jobslot` byte to the atom (right
 * after `device_nr`, before `core_req`) plus a made-up
 * BASE_JD_REQ_JOB_SLOT core_req bit (1 << 17), on the theory that
 * automatic slot routing was unreliable for a combined vertex+tiler
 * (CS|T) atom on this device/kernel, and that reading per-slot
 * JS_FEATURES via KBASE_IOCTL_GET_GPUPROPS and pinning the atom to a
 * matching slot would fix it.
 *
 * That theory doesn't hold up. Cross-checked against several real
 * kbase UAPI trees (mali_base_kernel.h / mali_base_jm_kernel.h across
 * multiple driver vintages, plus mali_kbase_js.c), there is no
 * per-atom job-slot-override field or flag anywhere in the JM ABI.
 * The byte the earlier revision repurposed as `jobslot` is, in every
 * real copy of this struct, reserved padding between `device_nr` and
 * `core_req` that must stay zero; no core_req bit at position 17 (or
 * anywhere past BASE_JD_REQ_SKIP_CACHE_END at bit 16) is defined
 * either. Job-slot assignment (JS0 = fragment, JS1/JS2 = vertex/
 * tiler/compute) is done entirely inside the kernel's own job
 * scheduler from the FS/CS/T/ONLY_COMPUTE bits of `core_req` -- that's
 * the only lever userspace actually has, and it's the one every other
 * kbase-based driver relies on. So the "pin to a slot" write was a
 * no-op at best (the kernel ignores both the unknown core_req bit and
 * the reserved byte) and an ABI violation (nonzero reserved field) at
 * worst -- either way it could never have changed which slot the
 * kernel picked. That's exactly consistent with the identical
 * silent-timeout symptom coming back unchanged after this "fix" was
 * added: it never did anything.
 *
 * The actual fix is to submit only the real FS / (CS|T) requirement
 * bits, leave the reserved byte at 0, and let the kernel's scheduler
 * route the atom -- the same as every other kbase client does. If
 * jobs still don't complete after that, the cause is elsewhere (jc
 * pointing at unmapped/incoherent memory, a job chain the shader
 * cores genuinely fault on that this DDK/kernel combo fails to
 * report, a GPU power-domain issue, etc.) and needs fresh on-device
 * triage rather than another guess at slot routing.
 *
 * CORRECTION to the above (kept in place rather than deleted, since
 * the "don't invent an ABI field" instinct that prompted it was
 * right even though the specific conclusion wasn't): the claim that
 * there is no per-atom job-slot override anywhere in the kbase ABI is
 * wrong. The current upstream GPL kbase UAPI header
 * (mali_base_jm_kernel.h, ARM copyright 2019-2021, fetched and
 * checked directly against this file's struct) defines both a real
 * `jobslot` field at the exact offset this file used to call
 * `reserved0` (see the struct definition below) and a real
 * BASE_JD_REQ_JOB_SLOT core_req bit (1 << 17) that selects it. The
 * trees this was cross-checked against before (a b_r10p0-era one
 * among them) simply predate that field's addition to the ABI --
 * absent in a 2017 tree doesn't mean absent in this device's
 * R54P1-era one, which postdates the 2019-2021 header above. That
 * doesn't retroactively make the old per-atom "jobslot" hack this
 * comment originally describes correct -- it wrote to the right byte
 * for the wrong reason, with a made-up core_req bit (1 << 17
 * genuinely is BASE_JD_REQ_JOB_SLOT, but nothing else about that
 * revision's reasoning was checked against a real header either) --
 * and it's not what fixed the silent-timeout symptom described
 * above. What actually explained this file's most recent failure
 * (event_code 0x4003, decoded elsewhere in this file to
 * BASE_JD_EVENT_JOB_INVALID) was unrelated: a stale pre_dep on an
 * already-fully-completed atom, removed in
 * panvk_queue_jm_submit_atom() below. Automatic slot routing (leaving
 * PANVK_KBASE_JD_REQ_JOB_SLOT unset, as this file still does) remains
 * the default, equally-real kbase behaviour and hasn't been shown to
 * need overriding on this device.
 *
 *   - KBASE_IOCTL_VERSION_CHECK   = _IOWR(0x80, 0, {u16 major, u16 minor})
 *   - KBASE_IOCTL_JOB_SUBMIT      = _IOW (0x80, 2, {u64 addr, u32 nr_atoms, u32 stride})
 *   - struct base_jd_atom_v2 (56 bytes) padded to the 64-byte stride -- what
 *     we submit (layout below)
 *   - struct base_jd_event_v2 (24 bytes: u32 event_code, u8 atom_number,
 *     u8 pad[3], u64 udata[2])
 *   - BASE_JD_EVENT_DONE = 0x01
 */

#include <errno.h>
#include <inttypes.h>
#include <poll.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "util/log.h"
#include "util/os_misc.h"
#include "util/os_time.h"

#include "genxml/gen_macros.h"

#include "decode.h"

#include "panvk_buffer.h"
#include "panvk_cmd_buffer.h"
#include "panvk_device_memory.h"
#include "panvk_device.h"
#include "panvk_entrypoints.h"
#include "panvk_event.h"
#include "panvk_image.h"
#include "panvk_image_view.h"
#include "panvk_instance.h"
#include "panvk_physical_device.h"
#include "panvk_priv_bo.h"
#include "panvk_queue.h"

#include "vk_framebuffer.h"
#include "vk_sync.h"

#define PANVK_KBASE_IOCTL_TYPE 0x80

struct panvk_kbase_version_check {
   uint16_t major;
   uint16_t minor;
};
#define PANVK_KBASE_IOCTL_VERSION_CHECK \
   _IOWR(PANVK_KBASE_IOCTL_TYPE, 0, struct panvk_kbase_version_check)

struct panvk_kbase_job_submit {
   uint64_t addr;
   uint32_t nr_atoms;
   uint32_t stride;
};
#define PANVK_KBASE_IOCTL_JOB_SUBMIT \
   _IOW(PANVK_KBASE_IOCTL_TYPE, 2, struct panvk_kbase_job_submit)

struct panvk_kbase_dependency {
   uint8_t atom_id;
   uint8_t dependency_type;
};
#define PANVK_KBASE_DEP_TYPE_DATA 1

/* struct base_jd_atom_v2, byte for byte (56 bytes), padded to the 64-byte
 * stride this kernel's JOB_SUBMIT requires -- see the big comment at the
 * top of this file. Do NOT put anything in front of `jc`: the kernel reads
 * `jc` from offset 0. Every field sits on its natural alignment boundary,
 * so no __attribute__((packed)) is needed. */
struct panvk_kbase_atom {
   uint64_t jc;                              /* offset 0  */
   uint64_t udata[2];                        /* offset 8  (base_jd_udata) */
   uint64_t extres_list;                     /* offset 24 */
   uint16_t nr_extres;                       /* offset 32 */
   uint8_t jit_id[2];                        /* offset 34 */
   struct panvk_kbase_dependency pre_dep[2]; /* offset 36 */
   uint8_t atom_number;                      /* offset 40 */
   uint8_t prio;                             /* offset 41 */
   uint8_t device_nr;                        /* offset 42 */
   /* Job slot to use when PANVK_KBASE_JD_REQ_JOB_SLOT is set. Left at 0
    * (automatic routing): a vertex+tiler (CS|T) atom and a fragment (FS)
    * atom are routed by core_req alone. */
   uint8_t jobslot;                          /* offset 43 */
   uint32_t core_req;                        /* offset 44 */
   /* v2 ends here with padding[8] (56 bytes); the extra 8 bytes bring the
    * element to the 64-byte stride. All zero. */
   uint8_t padding[16];                      /* offset 48 */
};                                           /* size 64 */
_Static_assert(sizeof(struct panvk_kbase_atom) == 64,
               "JOB_SUBMIT stride on this kernel is 64");
_Static_assert(offsetof(struct panvk_kbase_atom, jc) == 0 &&
               offsetof(struct panvk_kbase_atom, atom_number) == 40 &&
               offsetof(struct panvk_kbase_atom, core_req) == 44,
               "must match struct base_jd_atom_v2");

/* Request the atom be executed on a specific job slot (the `jobslot`
 * field above), instead of letting the kernel's scheduler pick one
 * from core_req alone. Not currently set by this file -- see the
 * correction note on `jobslot` above for why -- kept defined for when
 * it's needed. */
#define PANVK_KBASE_JD_REQ_JOB_SLOT ((uint32_t)1 << 17)


/* --------------------------------------------------------------------- *
 * base_jd_core_req (JM) -- every bit this field defines, not just the
 * three panvk currently issues.
 *
 * This layout has been stable since the r5p0-era kbase UAPI (bits 5-8
 * were reserved BASE_MEM_HINT_* placeholders back then; they were
 * reused for EVENT_COALESCE/COHERENT_GROUP/PERMON/EXTERNAL_RESOURCES
 * once MEM_HINT was dropped) straight through every later JM release,
 * this "R54P1"-era one included: JM stopped gaining job-chain features
 * once ARM's active development moved to the CSF path for Valhall, so
 * these bit assignments are frozen, not just "current for this driver
 * version". Verified against ARM's own GPL kbase UAPI headers across
 * several driver vintages (a b_r10p0 tree, and JS/JD/JM sources from a
 * modern Android GKI tree that still ships the `#if !MALI_USE_CSF` JM
 * path alongside CSF) -- not against an r54p1 tree directly, since
 * ARM's kbase source isn't public past what OEM kernel drops mirror.
 *
 * core_req is a plain OR-able u32 bitmask; the *kernel* job scheduler
 * (never userspace) uses it to choose a job slot -- see IMPORTANT #2
 * above -- so nothing below ever selects a slot directly.
 */

/* --- HW job-type bits. Together with BASE_JD_REQ_SOFT_JOB, these are
 * mutually exclusive "what kind of atom is this" bits -- see
 * PANVK_KBASE_JD_REQ_ATOM_TYPE below. --- */
#define PANVK_KBASE_JD_REQ_DEP ((uint32_t)0)
   /* No requirement: dependency-only atom, runs nothing on the GPU.
    * Exists so a pure ordering edge can still be expressed as an atom. */
#define PANVK_KBASE_JD_REQ_FS ((uint32_t)1 << 0) /* fragment job */
#define PANVK_KBASE_JD_REQ_CS ((uint32_t)1 << 1)
   /* Vertex, geometry, *or* compute job -- "CS" here is a Midgard-era
    * job-type name and predates, and is unrelated to, Vulkan's
    * VK_QUEUE_COMPUTE_BIT. Compare PANVK_KBASE_JD_REQ_ONLY_COMPUTE
    * below, which narrows this to just the compute case. */
#define PANVK_KBASE_JD_REQ_T ((uint32_t)1 << 2) /* tiler job */
#define PANVK_KBASE_JD_REQ_CF ((uint32_t)1 << 3)
   /* HW cache-flush job -- a distinct job *type*, unrelated to the
    * SW SKIP_CACHE_START/END bits further down. Unused by panvk. */
#define PANVK_KBASE_JD_REQ_V ((uint32_t)1 << 4)
   /* HW value-writeback job. Unused by panvk. */

/* --- SW-only bits: interpreted by the kbase driver: the job-slot HW
 * itself never sees them. --- */
#define PANVK_KBASE_JD_REQ_EVENT_COALESCE ((uint32_t)1 << 5)
   /* Suppress this atom's own completion event; it's folded into
    * whichever later atom completes without this bit set. MUST stay
    * unset here: panvk_queue_jm_submit_atom() submits one atom and
    * blocks reading *that atom's own* event (see the poll/read loop
    * above) -- coalescing it away would make that read wait for some
    * later atom's event instead, which on a queue with only ever one
    * atom in flight may never come. Also mutually exclusive with
    * EXTERNAL_RESOURCES per the kbase UAPI. */
#define PANVK_KBASE_JD_REQ_COHERENT_GROUP ((uint32_t)1 << 6)
   /* "Needs some coherent core group, don't care which." A scheduler
    * fairness hint for compute atoms sharing the GPU with other
    * contexts (see kbase_js_defs.h's coherent-group-contention
    * tracking) -- irrelevant to correctness on this single-context,
    * one-atom-in-flight queue. Not set. */
#define PANVK_KBASE_JD_REQ_PERMON ((uint32_t)1 << 7)
   /* Enable HW performance counters only around this atom, to save
    * power otherwise. Not wired up: nothing reaches this file to say
    * whether a performance-counter capture is active (that lives in
    * panvk's own perf-counter path, not in panvk_gpu_queue). Left as
    * an extension point -- see panvk_kbase_core_req_build() below. */
#define PANVK_KBASE_JD_REQ_EXTERNAL_RESOURCES ((uint32_t)1 << 8)
   /* Atom's extres_list/nr_extres (see struct panvk_kbase_atom above)
    * carry external resources needing residency tracking. This is a
    * derived bit, not a free choice: required exactly when
    * nr_extres > 0, which this file currently never populates -- see
    * panvk_kbase_core_req_build() below. */
#define PANVK_KBASE_JD_REQ_SOFT_JOB ((uint32_t)1 << 9)
   /* Software-defined job: never touches the GPU, just tells kbase to
    * perform some bookkeeping action (fence signal/wait, JIT
    * alloc/free, event set/reset/wait, ...). panvk's own
    * PANVK_EVENT_OP_* handling is done entirely in userspace (see
    * panvk_event_is_set()/panvk_event_update()), not
    * through a kbase soft job, so none of the SOFT_* subtypes below
    * are ever issued by this file today; listed because the user asked
    * for every case core_req's value space defines, not just the ones
    * in current use. */
#define PANVK_KBASE_JD_REQ_SOFT_JOB_TYPE (PANVK_KBASE_JD_REQ_SOFT_JOB | 0x1f)
   /* Mask to recover a soft job's exact subtype: AND core_req against
    * this, then compare for equality against one of the SOFT_*
    * defines below (each already includes the SOFT_JOB bit itself). */
#define PANVK_KBASE_JD_REQ_SOFT_DUMP_CPU_GPU_TIME (PANVK_KBASE_JD_REQ_SOFT_JOB | 0x1)
#define PANVK_KBASE_JD_REQ_SOFT_FENCE_TRIGGER     (PANVK_KBASE_JD_REQ_SOFT_JOB | 0x2)
#define PANVK_KBASE_JD_REQ_SOFT_FENCE_WAIT        (PANVK_KBASE_JD_REQ_SOFT_JOB | 0x3)
#define PANVK_KBASE_JD_REQ_SOFT_REPLAY            (PANVK_KBASE_JD_REQ_SOFT_JOB | 0x4)
#define PANVK_KBASE_JD_REQ_SOFT_EVENT_WAIT        (PANVK_KBASE_JD_REQ_SOFT_JOB | 0x5)
#define PANVK_KBASE_JD_REQ_SOFT_EVENT_SET         (PANVK_KBASE_JD_REQ_SOFT_JOB | 0x6)
#define PANVK_KBASE_JD_REQ_SOFT_EVENT_RESET       (PANVK_KBASE_JD_REQ_SOFT_JOB | 0x7)
#define PANVK_KBASE_JD_REQ_SOFT_DEBUG_COPY        (PANVK_KBASE_JD_REQ_SOFT_JOB | 0x8)
#define PANVK_KBASE_JD_REQ_SOFT_JIT_ALLOC         (PANVK_KBASE_JD_REQ_SOFT_JOB | 0x9)
#define PANVK_KBASE_JD_REQ_SOFT_JIT_FREE          (PANVK_KBASE_JD_REQ_SOFT_JOB | 0xa)
#define PANVK_KBASE_JD_REQ_SOFT_EXT_RES_MAP       (PANVK_KBASE_JD_REQ_SOFT_JOB | 0xb)
#define PANVK_KBASE_JD_REQ_SOFT_EXT_RES_UNMAP     (PANVK_KBASE_JD_REQ_SOFT_JOB | 0xc)
#define PANVK_KBASE_JD_REQ_ONLY_COMPUTE ((uint32_t)1 << 10)
   /* HW: job chain is Compute-Shader jobs *only*, as opposed to the
    * broader CS bit above which also covers Vertex/Geometry. panvk
    * only ever issues combined vertex+tiler chains (CS|T, see
    * PANVK_KBASE_ATOM_VERTEX_TILER below) or fragment chains (FS),
    * never a compute-only chain, so this stays unset; kept as an
    * extension point for a future compute-only JM queue. */
#define PANVK_KBASE_JD_REQ_SPECIFIC_COHERENT_GROUP ((uint32_t)1 << 11)
   /* HW: pin to the core group named by the atom's device_nr, instead
    * of "any" (COHERENT_GROUP above); only guaranteed meaningful for
    * ONLY_COMPUTE atoms, and takes priority if both are set. Not
    * used; device_nr is left at its zero-initialized value. */
#define PANVK_KBASE_JD_REQ_EVENT_ONLY_ON_FAILURE ((uint32_t)1 << 12)
   /* SW: suppress the completion event on success, still send it on
    * failure. Not set, for the same reason as EVENT_COALESCE: this
    * driver's wait loop needs the event on every completion, success
    * or not, to know when to stop polling. */
#define PANVK_KBASE_JD_REQ_FS_AFBC ((uint32_t)1 << 13)
   /* SW: this fragment job's colour output targets an AFBC-encoded
    * surface. Only meaningful together with FS. This file has no
    * visibility into the batch's framebuffer image layout/modifier,
    * so it's derived from a caller-supplied flag rather than guessed
    * here -- see fb_targets_afbc in panvk_kbase_core_req_build(). */
#define PANVK_KBASE_JD_REQ_EVENT_NEVER ((uint32_t)1 << 14)
   /* SW: never send a completion event, success or failure. Never
    * set, for the same reason EVENT_COALESCE isn't: this driver's own
    * wait loop is the only consumer of that event. */
#define PANVK_KBASE_JD_REQ_SKIP_CACHE_START ((uint32_t)1 << 15)
#define PANVK_KBASE_JD_REQ_SKIP_CACHE_END   ((uint32_t)1 << 16)
   /* SW: skip the GPU-side cache clean/invalidate the job-manager HW
    * would otherwise do before/after this atom. Neither is set: the
    * fragment atom consumes the vertex/tiler atom's output (and BOs
    * are shared across batches on this queue), so every atom needs
    * the HW's normal inter-job cache maintenance to actually run.
    * (Separate mechanism from the CPU-side
    * pan_kmod_flush_bo_map_syncs() calls elsewhere in this file, which
    * manage CPU/GPU mapping coherency, not the GPU's own L2.) Bit 17
    * and above are unused/reserved in base_jd_core_req -- see
    * BASEP_JD_REQ_RESERVED in the real UAPI header -- which is also
    * why the old per-atom "jobslot" hack in IMPORTANT #2 above was
    * always a no-op: there was never a bit there to read. */

/* Mask of the bits that select the atom's *type*; dependency-only
 * atoms are still allowed to carry the rest of core_req around them. */
#define PANVK_KBASE_JD_REQ_ATOM_TYPE                                       \
   (PANVK_KBASE_JD_REQ_FS | PANVK_KBASE_JD_REQ_CS | PANVK_KBASE_JD_REQ_T | \
    PANVK_KBASE_JD_REQ_CF | PANVK_KBASE_JD_REQ_V |                        \
    PANVK_KBASE_JD_REQ_SOFT_JOB | PANVK_KBASE_JD_REQ_ONLY_COMPUTE)

#define PANVK_KBASE_JD_PRIO_MEDIUM 0

/* struct base_jd_event_v2, byte-for-byte: 24 bytes. */
struct panvk_kbase_event_v2 {
   uint32_t event_code;
   uint8_t atom_number;
   uint8_t padding[3];
   uint64_t udata[2];
};

#define PANVK_KBASE_JD_EVENT_DONE 0x01

/* --------------------------------------------------------------------- */

enum panvk_kbase_atom_kind {
   PANVK_KBASE_ATOM_VERTEX_TILER,
   PANVK_KBASE_ATOM_FRAGMENT,
};

/* The two atom shapes a kbase JOB_SUBMIT(stride = 64) may be asked to parse.
 * Which one a given kernel honours is detected at run time (see
 * panvk_kbase_atom_layout_get()), because getting it wrong is silent: the
 * kernel just sees a dependency-only atom and completes it with DONE.
 *   V2: base_jd_atom_v2 bytes (jc at offset 0), padded to 64 -- what r54p1
 *       was verified to execute.
 *   V3: base_jd_atom as shipped in older DDKs: `seq_nr` first, `jc` at 8. */
enum panvk_kbase_atom_layout {
   PANVK_KBASE_ATOM_LAYOUT_UNKNOWN = 0,
   PANVK_KBASE_ATOM_LAYOUT_V2,
   PANVK_KBASE_ATOM_LAYOUT_V3,
};

struct panvk_kbase_atom_v3 {
   uint64_t seq_nr;                          /* offset 0  */
   uint64_t jc;                              /* offset 8  */
   uint64_t udata[2];                        /* offset 16 */
   uint64_t extres_list;                     /* offset 32 */
   uint16_t nr_extres;                       /* offset 40 */
   uint8_t jit_id[2];                        /* offset 42 */
   struct panvk_kbase_dependency pre_dep[2]; /* offset 44 */
   uint8_t atom_number;                      /* offset 48 */
   uint8_t prio;                             /* offset 49 */
   uint8_t device_nr;                        /* offset 50 */
   uint8_t jobslot;                          /* offset 51 */
   uint32_t core_req;                        /* offset 52 */
   uint8_t renderpass_id;                    /* offset 56 */
   uint8_t padding[7];                       /* offset 57 */
};                                           /* size 64 */
_Static_assert(sizeof(struct panvk_kbase_atom_v3) == 64,
               "base_jd_atom (v3) is 64 bytes");

union panvk_kbase_atom_u {
   struct panvk_kbase_atom v2;
   struct panvk_kbase_atom_v3 v3;
};
_Static_assert(sizeof(union panvk_kbase_atom_u) == 64,
               "JOB_SUBMIT stride on this kernel is 64");

static void
panvk_kbase_atom_fill(union panvk_kbase_atom_u *out,
                      enum panvk_kbase_atom_layout layout, uint64_t jc,
                      uint64_t cookie, uint32_t core_req, uint8_t atom_number)
{
   memset(out, 0, sizeof(*out));

   /* pre_dep[] stays zeroed in both shapes (the queue is synchronous, no atom
    * ever depends on another), .jobslot stays 0 (automatic routing). */
   if (layout == PANVK_KBASE_ATOM_LAYOUT_V3) {
      out->v3.seq_nr = atom_number;
      out->v3.jc = jc;
      out->v3.udata[0] = cookie;
      out->v3.udata[1] = jc;
      out->v3.core_req = core_req;
      out->v3.atom_number = atom_number;
      out->v3.prio = PANVK_KBASE_JD_PRIO_MEDIUM;
   } else {
      out->v2.jc = jc;
      out->v2.udata[0] = cookie;
      out->v2.udata[1] = jc;
      out->v2.core_req = core_req;
      out->v2.atom_number = atom_number;
      out->v2.prio = PANVK_KBASE_JD_PRIO_MEDIUM;
   }
}

/* kbase hands the atom's `udata` back verbatim in the completion event, so it
 * is the one reliable way to tell *our* completion event from any other event
 * on this fd (the `atom_number` echo has been reported to be unreliable, see
 * the note in panvk_queue_jm_submit_atom()). High word: a tag; low word: a
 * per-process submission counter. */
#define PANVK_KBASE_EVT_COOKIE 0x504b564b00000000ull /* "PKVK" */

/* Job header word 0 is the exception status the job manager writes back when
 * a job finishes: 1 == DONE (the convention pandecode_abort_on_fault()
 * checks). The `next` pointer is in words 6/7 (see pan_jc.h). */
#define PANVK_KBASE_JOB_STATUS_DONE 1u
#define PANVK_KBASE_JOB_HDR_NEXT_OFFSET 24

struct panvk_kbase_ddk_version_req {
   uint64_t version_buffer;
   uint32_t size;
   uint32_t padding;
};
#define PANVK_KBASE_IOCTL_GET_DDK_VERSION \
   _IOW(PANVK_KBASE_IOCTL_TYPE, 13, struct panvk_kbase_ddk_version_req)

/* Where the job headers live, i.e. what has to be invalidated in the CPU
 * caches before the job manager's status write-back can be read. Both NULL
 * means "uncached mapping, nothing to invalidate". */
struct panvk_kbase_inval {
   struct panvk_pool *pool;
   struct panvk_priv_bo *bo;
};

static bool
panvk_kbase_env_flag(const char *name)
{
   const char *v = os_get_option(name);
   return v && v[0] && v[0] != '0';
}

/* Verify mode: don't take a DONE event at face value, also check the job
 * headers. On with PANVK_DEBUG=trace/sync or PANVK_KBASE_VERIFY=1. */
static bool
panvk_kbase_verify_enabled(void)
{
   return PANVK_DEBUG(TRACE) || PANVK_DEBUG(SYNC) ||
          panvk_kbase_env_flag("PANVK_KBASE_VERIFY");
}

/* Walk a job chain through its CPU mapping (GPU VA == CPU VA on kbase) and
 * report the first job whose exception status isn't DONE. */
static bool
panvk_kbase_chain_done(uint64_t jc, uint64_t *bad_jc, uint32_t *bad_status)
{
   uint64_t cur = jc;

   for (unsigned guard = 0; cur && guard < 4096; guard++) {
      const volatile uint8_t *hdr = (const volatile uint8_t *)(uintptr_t)cur;
      uint32_t status = *(const volatile uint32_t *)hdr;

      if (status != PANVK_KBASE_JOB_STATUS_DONE) {
         *bad_jc = cur;
         *bad_status = status;
         return false;
      }

      cur = *(const volatile uint64_t *)(hdr + PANVK_KBASE_JOB_HDR_NEXT_OFFSET);
   }

   return true;
}

/* The job headers normally live in *cached* CPU mappings, so the job
 * manager's status write-back is only visible after the CPU lines have been
 * invalidated. Re-invalidate on every poll round: a stale line makes a
 * finished job look unfinished. */
static bool
panvk_kbase_wait_chain_done(struct panvk_device *dev,
                            const struct panvk_kbase_inval *inval,
                            uint64_t jc, int wait_ms, uint64_t *bad_jc,
                            uint32_t *bad_status)
{
   int64_t end_ns = os_time_get_nano() + (int64_t)wait_ms * 1000000ll;

   for (;;) {
      if (inval && inval->pool)
         panvk_pool_invalidate_maps(inval->pool);
      if (inval && inval->bo)
         panvk_priv_bo_invalidate(inval->bo, 0, pan_kmod_bo_size(inval->bo->bo));
      pan_kmod_flush_bo_map_syncs(dev->kmod.dev);

      if (panvk_kbase_chain_done(jc, bad_jc, bad_status))
         return true;

      if (os_time_get_nano() >= end_ns)
         return false;

      usleep(1000);
   }
}

/* Upper bound on how long we'll wait for a single JM atom to complete
 * before giving up and declaring the device lost, instead of blocking
 * vkQueueSubmit()/vkQueueWaitIdle() forever if a completion event never
 * shows up. Tunable via PANVK_KBASE_ATOM_TIMEOUT_MS for bisecting a
 * specific hang. */
static int64_t
panvk_kbase_atom_timeout_ns(void)
{
   static int64_t cached_ns = -2; /* -2: not yet computed */

   if (cached_ns != -2)
      return cached_ns;

   int64_t timeout_ms = 10000;
   const char *env = os_get_option("PANVK_KBASE_ATOM_TIMEOUT_MS");
   if (env && env[0]) {
      char *end = NULL;
      long parsed = strtol(env, &end, 10);
      if (end && *end == '\0' && parsed > 0)
         timeout_ms = parsed;
      else
         mesa_logw("panvk: ignoring invalid PANVK_KBASE_ATOM_TIMEOUT_MS=%s",
                   env);
   }

   cached_ns = timeout_ms * 1000000ll;
   return cached_ns;
}

/* Human-readable name for a JM job exception code (base_jd_event_code). */
static const char *
panvk_kbase_exception_name(uint32_t code)
{
   switch (code) {
   case 0x40: return "JOB_CONFIG_FAULT (bad job descriptor/config)";
   case 0x41: return "JOB_POWER_FAULT";
   case 0x42: return "JOB_READ_FAULT (GPU read of an unmapped VA)";
   case 0x43: return "JOB_WRITE_FAULT (GPU write to an unmapped VA)";
   case 0x44: return "JOB_AFFINITY_FAULT";
   case 0x48: return "JOB_BUS_FAULT";
   case 0x50: return "INSTR_INVALID_PC";
   case 0x51: return "INSTR_INVALID_ENC";
   case 0x52: return "INSTR_TYPE_MISMATCH";
   case 0x53: return "INSTR_OPERAND_FAULT";
   case 0x54: return "INSTR_TLS_FAULT";
   case 0x55: return "INSTR_BARRIER_FAULT";
   case 0x56: return "INSTR_ALIGN_FAULT";
   case 0x58: return "DATA_INVALID_FAULT (invalid descriptor/vertex/index data)";
   case 0x59: return "TILE_RANGE_FAULT";
   case 0x5a: return "ADDR_RANGE_FAULT";
   case 0x60: return "OUT_OF_MEMORY (tiler heap exhausted)";
   case 0x7f: return "UNKNOWN";
   case 0x80: return "DELAYED_BUS_FAULT";
   case 0x88: return "SHAREABILITY_FAULT";
   default:   return "unrecognised";
   }
}

/* Submit a single job chain as one kbase JM atom and block until it has
 * completed or the wait times out. Called once per batch per atom kind
 * (vertex/tiler, then fragment), always waiting out the previous atom
 * fully before the next is even constructed -- see the correction note
 * inside this function for why that, not a pre_dep, is what actually
 * orders a batch's fragment atom after its vertex/tiler atom.
 *
 * See the big comment on panvk_gpu_queue::jm_last_atom for why this is
 * synchronous instead of returning a fence-like object: completion is
 * only observable by draining a shared poll()+read() base_jd_event_v2
 * stream on the device fd -- there's nothing DRM-syncobj-shaped to
 * export or wait on from outside this function.
 */
static bool
panvk_queue_jm_submit_atom(struct panvk_gpu_queue *queue,
                           const struct panvk_kbase_inval *inval,
                           enum panvk_kbase_atom_kind kind,
                           enum panvk_kbase_atom_layout layout, uint64_t jc,
                           bool force_verify, bool quiet, int64_t timeout_ns,
                           bool *status_ok)
{
   struct panvk_device *dev = to_panvk_device(queue->vk.base.device);
   int fd = dev->kmod.dev->fd;

   if (status_ok)
      *status_ok = false;

   /* Atom numbers are a u8 (BASE_JD_ATOM_COUNT == 256) and 0 is reserved
    * to mean "no dependency" in pre_dep, so cycle through 1..255. Since
    * this queue only ever has one atom in flight, there's no risk of
    * colliding with an atom that's still outstanding. */
   uint8_t atom_number = queue->jm_last_atom + 1;
   if (atom_number == 0)
      atom_number = 1;

   /* Which job slot this atom lands on (JS0 = fragment, JS1/JS2 =
    * vertex/tiler/compute) is left to the kernel job scheduler's
    * automatic routing, made from these two requirement bits alone --
    * this file doesn't request a specific slot (PANVK_KBASE_JD_REQ_JOB_SLOT
    * stays unset; see the correction note on the `jobslot` struct
    * field above for why that's a choice, not an ABI limitation). */
   uint32_t core_req = kind == PANVK_KBASE_ATOM_FRAGMENT
                           ? PANVK_KBASE_JD_REQ_FS
                           : (PANVK_KBASE_JD_REQ_CS | PANVK_KBASE_JD_REQ_T);

   static uint32_t submit_seq;
   const uint64_t cookie =
      PANVK_KBASE_EVT_COOKIE |
      (uint64_t)__atomic_add_fetch(&submit_seq, 1, __ATOMIC_RELAXED);

   union panvk_kbase_atom_u atom;
   panvk_kbase_atom_fill(&atom, layout, jc, cookie, core_req, atom_number);

   /* CORRECTION -- an earlier revision of this function set
    * atom.pre_dep[0] here, pointing at queue->jm_last_atom whenever
    * this wasn't the first atom submitted on the queue (e.g. the
    * fragment atom of a batch pointing at that same batch's
    * vertex/tiler atom). That is what was producing the
    * event_code 0x4003 failure this correction responds to:
    * decoded against the kbase job-event bit layout (bit 14 = SW
    * event, bits 12:11 = type, bits 10:0 = subtype -- see
    * mali_base_jm_kernel.h's base_jd_event_code), 0x4003 is exactly
    * BASE_JD_SW_EVENT | BASE_JD_SW_EVENT_JOB | 0x003, i.e.
    * BASE_JD_EVENT_JOB_INVALID ("raised for many reasons, including
    * invalid data in the atom") -- a kernel-side rejection of the
    * submitted atom's own fields, before the GPU ever saw it, not a
    * shader/hardware fault. That it only ever hit the second atom of
    * a batch (the one that carried a pre_dep) and never the first
    * (which never did) was the tell.
    *
    * Why a pre_dep here is invalid rather than merely redundant:
    * panvk_queue_jm_submit_atom() is fully synchronous -- by the time
    * this function is called for the fragment atom, the vertex/tiler
    * atom it would point at has already been submitted, awaited, and
    * had its completion event read out to completion, several
    * function calls earlier. There is nothing left in flight for a
    * pre_dep to order against, and nothing in the kbase UAPI docs
    * promises that an atom_number remains a valid pre_dep target
    * after userspace has already drained its completion event; this
    * DDK appears to reject that as malformed input instead of
    * silently ignoring it. The ordering pre_dep would have expressed
    * is already fully established by this function's own blocking
    * wait, host-side, before the next atom is even constructed -- so
    * dropping it costs nothing and removes the one field that
    * differed between an atom that succeeded and one that didn't. */

   struct panvk_kbase_job_submit submit = {
      .addr = (uintptr_t)&atom,
      .nr_atoms = 1,
      .stride = sizeof(atom), /* == 64, see the struct definition */
   };

   /* This queue is fully synchronous, so there must be no event waiting on
    * the fd at this point. If there is one, it belongs to nothing we are
    * about to submit: reading it as "our" completion would make every later
    * submission consume the previous atom's event. Drain and report it. */
   for (;;) {
      struct pollfd stale_pfd = {.fd = fd, .events = POLLIN};
      if (poll(&stale_pfd, 1, 0) <= 0 || !(stale_pfd.revents & POLLIN))
         break;

      struct panvk_kbase_event_v2 stale;
      if (read(fd, &stale, sizeof(stale)) != (ssize_t)sizeof(stale))
         break;

      mesa_logw("panvk: discarding stale kbase event before submit: "
                "code=0x%x atom=%u udata=0x%" PRIx64 ",0x%" PRIx64,
                stale.event_code, stale.atom_number, stale.udata[0],
                stale.udata[1]);
   }

   if (ioctl(fd, PANVK_KBASE_IOCTL_JOB_SUBMIT, &submit)) {
      mesa_loge("panvk: KBASE_IOCTL_JOB_SUBMIT failed: %s", strerror(errno));
      return false;
   }

   queue->jm_last_atom = atom_number;

   /* Only one atom is ever in flight at a time in this submission model,
    * so the next event for *this atom_number* is the one we're waiting
    * for; anything else read off the stream first (there shouldn't be
    * anything else, but the ABI doesn't promise it) is skipped. The whole
    * loop is bounded by an absolute deadline instead of giving poll() a
    * fresh full timeout every round, so a device that keeps handing us
    * events for other atom numbers can't turn this into an unbounded
    * wait -- this is what fixes vkQueueSubmit()/vkQueueWaitIdle() hanging
    * forever when a completion event never arrives. */
   int64_t deadline_ns =
      os_time_get_nano() +
      (timeout_ns > 0 ? timeout_ns : panvk_kbase_atom_timeout_ns());

   /* In verify mode a DONE event isn't taken at face value: the job headers
    * must also show the hardware's DONE status. An event that doesn't line up
    * with the headers is reported, and an unattributed one is skipped in
    * favour of the next. */
   const bool verify = force_verify || panvk_kbase_verify_enabled();
   const char *wait_env = os_get_option("PANVK_KBASE_STATUS_WAIT_MS");
   const int status_wait_ms = (wait_env && wait_env[0]) ? atoi(wait_env) : 200;
   unsigned done_events = 0;

   for (;;) {
      int64_t now_ns = os_time_get_nano();
      int64_t remaining_ns = deadline_ns - now_ns;
      if (remaining_ns <= 0) {
         if (done_events) {
            if (!quiet)
               mesa_logw("panvk: kbase atom %u: %u DONE event(s) seen but no job "
                      "header ever reported DONE; trusting the kernel (see "
                      "the warnings above)", atom_number, done_events);
            return true;
         }
         mesa_loge("panvk: timed out waiting for kbase JM atom %u (job "
                   "chain 0x%" PRIx64 ") to complete -- device is either "
                   "stuck or its watchdog didn't fire; treating this "
                   "queue as lost instead of hanging forever",
                   atom_number, jc);
         return false;
      }

      struct pollfd pfd = {.fd = fd, .events = POLLIN};
      int pret = poll(&pfd, 1, (int)(remaining_ns / 1000000));
      if (pret < 0) {
         if (errno == EINTR)
            continue;
         mesa_loge("panvk: poll() on kbase fd failed: %s", strerror(errno));
         return false;
      }
      if (pret == 0)
         continue; /* timed out this round; deadline check above catches
                     * the overall timeout on the next iteration */

      struct panvk_kbase_event_v2 evt;
      ssize_t n = read(fd, &evt, sizeof(evt));
      if (n < 0) {
         if (errno == EINTR || errno == EAGAIN)
            continue;
         mesa_loge("panvk: read() on kbase fd failed: %s", strerror(errno));
         return false;
      }
      if (n == 0 || (size_t)n < sizeof(evt))
         continue;

      /* Match the event to *this* atom by its udata cookie. An earlier
       * revision of this file concluded from an on-device test that the DDK
       * doesn't echo atom_number back (submitted 1, got 0) and accepted the
       * first event it read, whatever it was. kbase reports the index of the
       * atom slot, so that observation is suspect: an event for slot 0 means
       * the kernel put the atom in slot 0, i.e. it did not read our
       * atom_number -- and then it did not read core_req either, so it would
       * treat the atom as a pure dependency atom that completes at once with
       * DONE without ever touching the GPU. That matches "DONE, but the job
       * header status never changes". udata is echoed verbatim, so use it
       * to tell the cases apart; an all-zero udata is treated as
       * "unattributed" (a kernel that doesn't echo it) and is still
       * accepted unless the job headers contradict it. */
      const bool ours = evt.udata[0] == cookie;
      const bool unattributed = evt.udata[0] == 0 && evt.udata[1] == 0;

      if (verify && !quiet)
         mesa_logw("panvk: kbase event: code=0x%x atom=%u (submitted %u) "
                   "udata=0x%" PRIx64 ",0x%" PRIx64 " (sent 0x%" PRIx64
                   ") -> %s",
                   evt.event_code, evt.atom_number, atom_number,
                   evt.udata[0], evt.udata[1], cookie,
                   ours ? "ours" : unattributed ? "unattributed" : "foreign");

      if (!ours && !unattributed) {
         /* A probe (force_verify) that gets back a udata it never sent has
          * its answer: the kernel parsed the atom from other offsets than
          * the ones used, i.e. the wrong atom shape. Don't sit out the
          * timeout for an event that will never carry our cookie. */
         if (force_verify) {
            if (!quiet)
               mesa_logw("panvk: kbase atom came back with a foreign udata "
                         "(0x%" PRIx64 ",0x%" PRIx64 "): wrong atom shape",
                         evt.udata[0], evt.udata[1]);
            return true;
         }

         mesa_logw("panvk: ignoring kbase event that carries another cookie "
                   "(code=0x%x atom=%u)", evt.event_code, evt.atom_number);
         continue;
      }

      if (evt.event_code != PANVK_KBASE_JD_EVENT_DONE) {
         mesa_loge("panvk: kbase JM atom %u (job chain 0x%" PRIx64
                   ") reported failure (event_code 0x%x: %s)",
                   atom_number, jc, evt.event_code,
                   panvk_kbase_exception_name(evt.event_code));
         if (!dev->debug.decode_ctx)
            mesa_loge("panvk: re-run with PANVK_DEBUG=kbase_diag to dump "
                      "only the failing job chain");
         return false;
      }

      if (!verify)
         return true;

      done_events++;

      uint64_t bad_jc = 0;
      uint32_t bad_status = 0;
      if (panvk_kbase_wait_chain_done(dev, inval, jc, status_wait_ms,
                                      &bad_jc, &bad_status)) {
         if (status_ok)
            *status_ok = true;
         if (!quiet)
            mesa_logw("panvk: kbase atom %u: every job in chain 0x%" PRIx64
                      " reports DONE", atom_number, jc);
         return true;
      }

      if (!quiet)
         mesa_logw("panvk: kbase reported DONE for atom %u (%s event) but job "
                "0x%" PRIx64 " in chain 0x%" PRIx64 " still has "
                "exception_status=0x%x after %d ms (CPU caches "
                "invalidated). Either the atom never reached the GPU or "
                "the GPU's write-back isn't visible to the CPU.",
                atom_number, ours ? "matching" : "unattributed", bad_jc, jc,
                bad_status, status_wait_ms);

      if (ours)
         return true; /* The kernel says our atom completed: believe it. */

      /* Unattributed and contradicted by the headers: it may be a stray
       * event, so give the real one a short grace period. */
      int64_t grace_ns = os_time_get_nano() + 1000000000ll;
      if (grace_ns < deadline_ns)
         deadline_ns = grace_ns;
   }
}

/* ------------------------------------------------------------------------
 * Self-test: submit a NULL job through exactly the path real work takes and
 * check that the job manager marks it DONE. Splits the possible failures:
 *  - uncached buffer fails         -> the atom/submission path is broken
 *  - only cached buffers fail      -> CPU<->GPU cache maintenance is broken
 *  - everything passes             -> the draw job's own contents are suspect
 * Runs once, in verify mode (or PANVK_KBASE_SELFTEST=1); PANVK_KBASE_SELFTEST=0
 * disables it.
 * ---------------------------------------------------------------------- */

static void
panvk_kbase_log_ddk_version(int fd)
{
   char buf[160] = {0};
   struct panvk_kbase_ddk_version_req req = {
      .version_buffer = (uintptr_t)buf,
      .size = sizeof(buf) - 1,
   };

   if (ioctl(fd, PANVK_KBASE_IOCTL_GET_DDK_VERSION, &req) >= 0)
      mesa_logw("panvk: kbase DDK version: \"%s\"", buf);
   else
      mesa_logw("panvk: KBASE_IOCTL_GET_DDK_VERSION failed: %s",
                strerror(errno));
}

static bool
panvk_kbase_null_job_probe(struct panvk_gpu_queue *queue,
                           enum panvk_kbase_atom_layout layout, bool cached,
                           bool quiet, const char *what)
{
   struct panvk_device *dev = to_panvk_device(queue->vk.base.device);
   struct panvk_priv_bo *bo = NULL;

   VkResult res =
      panvk_priv_bo_create(dev, 4096, cached ? PAN_KMOD_BO_FLAG_WB_MMAP : 0,
                           VK_SYSTEM_ALLOCATION_SCOPE_DEVICE, &bo);
   if (res != VK_SUCCESS || !bo->addr.host) {
      if (!quiet)
         mesa_logw("panvk: kbase self-test (%s): can't allocate a test BO",
                   what);
      if (res == VK_SUCCESS)
         panvk_priv_bo_unref(bo);
      return false;
   }

   memset(bo->addr.host, 0, 64);
   pan_cast_and_pack(bo->addr.host, JOB_HEADER, header) {
      header.type = MALI_JOB_TYPE_NULL;
      header.index = 1;
   }

   if (cached) {
      panvk_priv_bo_flush(bo, 0, 64);
      pan_kmod_flush_bo_map_syncs(dev->kmod.dev);
   }

   const struct panvk_kbase_inval inval = {.bo = cached ? bo : NULL};
   bool status_ok = false;
   bool ok = panvk_queue_jm_submit_atom(queue, &inval,
                                        PANVK_KBASE_ATOM_VERTEX_TILER, layout,
                                        bo->addr.dev, true, quiet,
                                        2000000000ll, &status_ok);

   if (!quiet)
      mesa_logw("panvk: kbase self-test (%s): atom %s, job header status %s",
                what, ok ? "completed" : "FAILED",
                status_ok ? "DONE" : "NOT updated");

   panvk_priv_bo_unref(bo);
   return ok && status_ok;
}

static const char *
panvk_kbase_layout_name(enum panvk_kbase_atom_layout layout)
{
   switch (layout) {
   case PANVK_KBASE_ATOM_LAYOUT_V2:
      return "v2 (base_jd_atom_v2 bytes, jc at offset 0, stride 64)";
   case PANVK_KBASE_ATOM_LAYOUT_V3:
      return "v3 (seq_nr first, jc at offset 8, stride 64)";
   default:
      return "unknown";
   }
}

/* Find the atom shape this kernel actually executes: submit a NULL job in
 * each candidate shape (on an UNCACHED buffer, so no CPU cache maintenance
 * is involved) and keep the first one whose job header the job manager marks
 * DONE. A wrong shape is harmless: the kernel reads zeros where core_req
 * should be and completes the atom as a dependency-only one. */
static enum panvk_kbase_atom_layout
panvk_kbase_detect_atom_layout(struct panvk_gpu_queue *queue, bool quiet)
{
   static const enum panvk_kbase_atom_layout candidates[] = {
      PANVK_KBASE_ATOM_LAYOUT_V2,
      PANVK_KBASE_ATOM_LAYOUT_V3,
   };

   for (unsigned i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
      if (panvk_kbase_null_job_probe(queue, candidates[i], false, quiet,
                                     panvk_kbase_layout_name(candidates[i])))
         return candidates[i];
   }

   mesa_logw("panvk: kbase: neither atom shape executed a NULL job; assuming "
             "v2 (the shape verified on r54p1). Set PANVK_KBASE_ATOM_LAYOUT="
             "v2|v3 to force one.");
   return PANVK_KBASE_ATOM_LAYOUT_V2;
}

/* The atom shape used for every real submission. Resolved once per process:
 * PANVK_KBASE_ATOM_LAYOUT=v2|v3 forces one (v3 is accepted by JOB_SUBMIT too,
 * it just isn't executed by r54p1 -- see the top of this file), anything else
 * auto-detects. */
static enum panvk_kbase_atom_layout
panvk_kbase_atom_layout_get(struct panvk_gpu_queue *queue)
{
   static uint32_t resolved;
   uint32_t cur = __atomic_load_n(&resolved, __ATOMIC_ACQUIRE);
   if (cur != PANVK_KBASE_ATOM_LAYOUT_UNKNOWN)
      return (enum panvk_kbase_atom_layout)cur;

   const char *env = os_get_option("PANVK_KBASE_ATOM_LAYOUT");
   const bool verbose = panvk_kbase_verify_enabled();
   bool forced = true;
   enum panvk_kbase_atom_layout layout;

   if (env && !strcmp(env, "v2")) {
      layout = PANVK_KBASE_ATOM_LAYOUT_V2;
   } else if (env && !strcmp(env, "v3")) {
      layout = PANVK_KBASE_ATOM_LAYOUT_V3;
   } else {
      forced = false;
      layout = panvk_kbase_detect_atom_layout(queue, !verbose);
   }

   __atomic_store_n(&resolved, (uint32_t)layout, __ATOMIC_RELEASE);

   if (verbose || forced)
      mesa_logw("panvk: kbase atom layout: %s%s",
                panvk_kbase_layout_name(layout), forced ? " [forced]" : "");

   return layout;
}

static bool
panvk_kbase_selftest_run(struct panvk_gpu_queue *queue, bool cached,
                         const char *what)
{
   return panvk_kbase_null_job_probe(queue, panvk_kbase_atom_layout_get(queue),
                                     cached, false, what);
}

static void
panvk_kbase_selftest(struct panvk_gpu_queue *queue)
{
   static uint32_t once;
   const char *env = os_get_option("PANVK_KBASE_SELFTEST");

   if (env && env[0] == '0')
      return;
   if (!panvk_kbase_verify_enabled() && !panvk_kbase_env_flag("PANVK_KBASE_SELFTEST"))
      return;
   if (__atomic_exchange_n(&once, 1, __ATOMIC_ACQ_REL))
      return;

   struct panvk_device *dev = to_panvk_device(queue->vk.base.device);
   struct pan_kmod_dev *kdev = dev->kmod.dev;

   panvk_kbase_log_ddk_version(kdev->fd);

   if (!panvk_kbase_selftest_run(queue, false, "uncached CPU mapping")) {
      mesa_logw("panvk: kbase self-test: a NULL job on an UNCACHED buffer is "
                "not executed either, so the problem is the atom/submission "
                "path itself (atom layout or core_req as the kernel reads "
                "it, or kernel policy) -- not CPU caches and not the draw's "
                "descriptors. Compare the event's atom/udata fields above "
                "with what was submitted.");
      return;
   }

   const uint32_t saved_flags = kdev->flags;
   bool user_tested = false, user_ok = false;

   kdev->flags &= ~PAN_KMOD_DEV_FLAG_MMAP_SYNC_THROUGH_KERNEL;
   if (pan_kmod_can_sync_bo_map_from_userland(kdev)) {
      user_tested = true;
      user_ok = panvk_kbase_selftest_run(queue, true,
                                         "cached mapping + EL0 cache ops");
   }

   kdev->flags |= PAN_KMOD_DEV_FLAG_MMAP_SYNC_THROUGH_KERNEL;
   bool kernel_ok = panvk_kbase_selftest_run(
      queue, true, "cached mapping + KBASE_IOCTL_MEM_SYNC");

   kdev->flags = saved_flags;

   if (user_tested && user_ok) {
      mesa_logw("panvk: kbase self-test passed: atoms execute and CPU cache "
                "maintenance works. If the draw still doesn't complete, the "
                "problem is in the draw job's contents.");
   } else if (kernel_ok) {
      kdev->flags |= PAN_KMOD_DEV_FLAG_MMAP_SYNC_THROUGH_KERNEL;
      mesa_logw("panvk: kbase self-test: %s -- switching to kernel-side "
                "cache sync (KBASE_IOCTL_MEM_SYNC) for this process.",
                user_tested ? "EL0 cache ops don't make GPU writes visible"
                            : "no EL0 cache ops available");
   } else {
      mesa_logw("panvk: kbase self-test: cached mappings fail with both sync "
                "methods while uncached ones work. Run with "
                "PANVK_DEBUG=no_wb_mmap.");
   }
}

#if PAN_ARCH >= 9
/* Map `size` bytes of a buffer's memory for CPU reading, with the CPU caches
 * invalidated so that data the GPU wrote earlier is visible. Returns NULL on
 * failure. When *unmap_base is non-NULL on return, the caller has to
 * pan_kmod_bo_munmap() it once done. */
static const uint32_t *
panvk_jm_map_indirect_args(struct panvk_device *dev,
                           const struct panvk_buffer *buffer, uint64_t offset,
                           uint32_t size, void **unmap_base,
                           size_t *unmap_size)
{
   *unmap_base = NULL;
   *unmap_size = 0;

   struct panvk_device_memory *mem = buffer->mem;
   if (!mem || !mem->bo)
      return NULL;

   const uint64_t bo_size = pan_kmod_bo_size(mem->bo);
   const uint64_t bo_off = buffer->mem_offset + offset;
   if (bo_off + size > bo_size)
      return NULL;

   uint8_t *base = mem->addr.host;
   if (!base) {
      void *mapped =
         pan_kmod_bo_mmap(mem->bo, PROT_READ | PROT_WRITE, MAP_SHARED, NULL);
      if (mapped == MAP_FAILED)
         return NULL;

      base = mapped;
      *unmap_base = mapped;
      *unmap_size = bo_size;
   }

   uint8_t *ptr = base + bo_off;
   pan_kmod_queue_bo_map_sync(mem->bo, bo_off, ptr, size,
                              PAN_KMOD_BO_SYNC_CPU_CACHE_FLUSH_AND_INVALIDATE);
   pan_kmod_flush_bo_map_syncs(dev->kmod.dev);

   return (const uint32_t *)ptr;
}

/* Valhall/JM has no hardware (or GPU-kernel) path to read indirect draw /
 * dispatch arguments, so they are resolved here, on the CPU, right before the
 * batch is handed to the kernel. Submission is synchronous: every earlier
 * atom -- including the ones that produced the indirect buffer's contents --
 * has completed by now. See struct panvk_jm_indirect_fixup. */
static void
panvk_jm_resolve_indirect(struct panvk_device *dev, struct panvk_batch *batch)
{
   /* vkCmdDraw*IndirectCount records maxDrawCount fixups that all share the
    * same count word; read it once instead of once per recorded draw. */
   const struct panvk_buffer *cached_count_buffer = NULL;
   uint64_t cached_count_offset = 0;
   uint32_t cached_draw_count = 0;

   util_dynarray_foreach(&batch->indirect_fixups,
                         struct panvk_jm_indirect_fixup, f) {
      uint32_t args[5] = {0};
      uint32_t nwords;

      switch (f->kind) {
      case PANVK_JM_INDIRECT_DISPATCH:
         nwords = 3;
         break;
      case PANVK_JM_INDIRECT_DRAW:
         nwords = 4;
         break;
      default:
         nwords = 5;
         break;
      }

      void *unmap_base;
      size_t unmap_size;

      /* vkCmdDraw*IndirectCount: draws past the count stored in the count
       * buffer are skipped without even looking at their arguments (those
       * may legitimately lie outside the buffer). */
      bool past_count = false;
      if (f->count_buffer) {
         if (cached_count_buffer != f->count_buffer ||
             cached_count_offset != f->count_offset) {
            const uint32_t *csrc = panvk_jm_map_indirect_args(
               dev, f->count_buffer, f->count_offset, sizeof(uint32_t),
               &unmap_base, &unmap_size);

            cached_count_buffer = f->count_buffer;
            cached_count_offset = f->count_offset;
            cached_draw_count = 0;

            if (csrc) {
               cached_draw_count = *csrc;

               if (unmap_base) {
                  struct panvk_device_memory *cmem = f->count_buffer->mem;
                  pan_kmod_bo_munmap(cmem->bo, unmap_base, unmap_size);
               }
            } else {
               mesa_loge("panvk: couldn't read back the indirect draw count; "
                         "draws skipped");
            }
         }

         past_count = f->count_index >= cached_draw_count;
      }

      const uint32_t *src = NULL;
      if (!past_count)
         src = panvk_jm_map_indirect_args(dev, f->buffer, f->offset,
                                          nwords * sizeof(uint32_t),
                                          &unmap_base, &unmap_size);

      bool run = false;
      if (past_count) {
         /* Nothing to run. */
      } else if (src) {
         memcpy(args, src, nwords * sizeof(uint32_t));

         if (unmap_base) {
            struct panvk_device_memory *mem = f->buffer->mem;
            pan_kmod_bo_munmap(mem->bo, unmap_base, unmap_size);
         }

         run = f->kind == PANVK_JM_INDIRECT_DISPATCH
                  ? panvk_per_arch(jm_apply_indirect_dispatch)(f, args)
                  : panvk_per_arch(jm_apply_indirect_draw)(f, args);
      } else {
         mesa_loge("panvk: couldn't read back indirect command arguments; "
                   "command skipped");
      }

      /* Flip the header between NULL and the real job type. The job keeps
       * its state across re-submissions of the command buffer, so only touch
       * it when the answer changed. */
      if (run != f->enabled) {
         uint32_t *hdr = f->job;

         for (unsigned i = 0; i < ARRAY_SIZE(f->type_xor); i++)
            hdr[i] ^= f->type_xor[i];

         f->enabled = run;
      }
   }
}
#endif /* PAN_ARCH >= 9 */

static bool
panvk_queue_submit_batch(struct panvk_gpu_queue *queue,
                         struct panvk_cmd_buffer *cmdbuf,
                         struct panvk_batch *batch)
{
   struct panvk_device *dev = to_panvk_device(queue->vk.base.device);
   struct panvk_physical_device *phys_dev =
      to_panvk_physical_device(dev->vk.physical);

   /* Reset the batch if it's already been issued */
   if (batch->issued) {
      util_dynarray_foreach(&batch->jobs, void *, job)
         memset((*job), 0, 4 * 4);

      /* Reset the tiler before re-issuing the batch */
      if (batch->tiler.ctx_descs.cpu) {
         memcpy(batch->tiler.heap_desc.cpu, &batch->tiler.heap_templ,
                sizeof(batch->tiler.heap_templ));

         struct mali_tiler_context_packed *ctxs = batch->tiler.ctx_descs.cpu;

         for (uint32_t i = 0; i < batch->fb.layer_count; i++)
            memcpy(&ctxs[i], &batch->tiler.ctx_templ, sizeof(*ctxs));
      }

#if PAN_ARCH >= 9
      /* The hardware writes the vertex array pointer / strides of every
       * MALLOC_VERTEX_JOB while it runs; put back what the CPU wrote, or the
       * job faults (DATA_INVALID_FAULT) when it's run a second time. */
      util_dynarray_foreach(&batch->draw_fixups, struct panvk_jm_draw_fixup,
                            fixup)
         memcpy(fixup->words, fixup->orig, sizeof(fixup->orig));
#endif

      /* We don't keep track of BO <-> job relationship, so let's just flush the
       * whole desc pool for now. */
      panvk_pool_flush_maps(&cmdbuf->desc_pool);
   }

#if PAN_ARCH >= 9
   if (util_dynarray_num_elements(&batch->indirect_fixups,
                                  struct panvk_jm_indirect_fixup)) {
      panvk_jm_resolve_indirect(dev, batch);
      panvk_pool_flush_maps(&cmdbuf->desc_pool);
   }
#endif

   /* Flush pending synchronization requests before submitting the job, to
    * make sure things are GPU-visible. */
   pan_kmod_flush_bo_map_syncs(dev->kmod.dev);

   if (batch->vtc_jc.first_job) {
      const struct panvk_kbase_inval inval = {.pool = &cmdbuf->desc_pool};
      bool ok = panvk_queue_jm_submit_atom(
         queue, &inval, PANVK_KBASE_ATOM_VERTEX_TILER,
         panvk_kbase_atom_layout_get(queue), batch->vtc_jc.first_job, false,
         false, 0, NULL);

      /* Decode/abort-on-fault must NOT be skipped just because this atom
       * already failed -- that's exactly the case where the dump is
       * worth the most. Run it whenever a debug flag is on, OR the atom
       * itself failed. */
      if (PANVK_DEBUG(TRACE) || PANVK_DEBUG(SYNC) || !ok) {
         /* If we want to read the descriptors back, we need to invalidate the
          * whole desc pool, otherwise we might end up with stale data. */
         panvk_pool_invalidate_maps(&cmdbuf->desc_pool);
         pan_kmod_flush_bo_map_syncs(dev->kmod.dev);
      }

      /* dev->debug.decode_ctx only exists when some PANVK_DEBUG decode
       * flag was already on at device creation (that's where it's
       * allocated -- not in this file). The `!ok` half of every
       * condition above and below is unconditional: it fires on a real
       * job fault whether or not the person running this ever passed
       * PANVK_DEBUG=. Without this NULL check, that combination --
       * decode_ctx never allocated, atom genuinely fails -- calls
       * straight into pandecode_jc()/pandecode_dump_mappings()/
       * pandecode_abort_on_fault() with a NULL context, and none of the
       * three defend against that: it segfaults instead of reporting
       * the failure. Gating on decode_ctx here means a failure always
       * gets the mesa_loge() from panvk_queue_jm_submit_atom() and a
       * clean vk_queue_set_lost() either way; the extra pandecode dump
       * is a bonus when a context happens to exist, never a
       * requirement for reaching this point without crashing. */
      if ((PANVK_DEBUG(TRACE) || !ok) && dev->debug.decode_ctx)
         pandecode_jc(dev->debug.decode_ctx, batch->vtc_jc.first_job,
                      phys_dev->kmod.dev->props.gpu_id);

      if ((PANVK_DEBUG(DUMP) || !ok) && dev->debug.decode_ctx)
         pandecode_dump_mappings(dev->debug.decode_ctx);

      if (((PANVK_DEBUG(SYNC) &&
            !panvk_kbase_env_flag("PANVK_KBASE_NO_ABORT")) || !ok) &&
          dev->debug.decode_ctx)
         pandecode_abort_on_fault(dev->debug.decode_ctx,
                                  batch->vtc_jc.first_job,
                                  phys_dev->kmod.dev->props.gpu_id);

      if (!ok) {
         if (!dev->debug.decode_ctx)
            mesa_loge("panvk: re-run with PANVK_DEBUG=sync for a full "
                      "job-chain dump of this failure");
         return false;
      }
   }

   if (batch->frag_jc.first_job) {
      const struct panvk_kbase_inval inval = {.pool = &cmdbuf->desc_pool};
      bool ok = panvk_queue_jm_submit_atom(
         queue, &inval, PANVK_KBASE_ATOM_FRAGMENT,
         panvk_kbase_atom_layout_get(queue), batch->frag_jc.first_job, false,
         false, 0, NULL);

      if (PANVK_DEBUG(TRACE) || PANVK_DEBUG(SYNC) || !ok) {
         panvk_pool_invalidate_maps(&cmdbuf->desc_pool);
         pan_kmod_flush_bo_map_syncs(dev->kmod.dev);
      }

      if ((PANVK_DEBUG(TRACE) || !ok) && dev->debug.decode_ctx)
         pandecode_jc(dev->debug.decode_ctx, batch->frag_jc.first_job,
                      phys_dev->kmod.dev->props.gpu_id);

      if ((PANVK_DEBUG(DUMP) || !ok) && dev->debug.decode_ctx)
         pandecode_dump_mappings(dev->debug.decode_ctx);

      if (((PANVK_DEBUG(SYNC) &&
            !panvk_kbase_env_flag("PANVK_KBASE_NO_ABORT")) || !ok) &&
          dev->debug.decode_ctx)
         pandecode_abort_on_fault(dev->debug.decode_ctx,
                                  batch->frag_jc.first_job,
                                  phys_dev->kmod.dev->props.gpu_id);

      if (!ok) {
         if (!dev->debug.decode_ctx)
            mesa_loge("panvk: re-run with PANVK_DEBUG=sync for a full "
                      "job-chain dump of this failure");
         return false;
      }
   }

   if (PANVK_DEBUG(TRACE))
      pandecode_next_frame(dev->debug.decode_ctx);

   batch->issued = true;
   return true;
}

/* vkCmdWaitEvents2() operations recorded on this batch. There's no GPU-side
 * soft-event-wait atom we submit here, so -- same spirit as the
 * synchronous atom submission above -- we just block the CPU on the
 * host-visible event status until it's set. Because this queue only ever
 * has one batch in flight at a time, nothing downstream can race ahead of
 * this wait. */
static VkResult
panvk_queue_wait_events(struct panvk_gpu_queue *queue,
                        struct panvk_batch *batch)
{
   struct panvk_device *dev = to_panvk_device(queue->vk.base.device);

   util_dynarray_foreach(&batch->event_ops, struct panvk_cmd_event_op, op) {
      if (op->type != PANVK_EVENT_OP_WAIT)
         continue;

      /* Spin briefly (the event is usually set by a previous batch or another
       * thread a few microseconds from now), then back off so a long wait
       * doesn't peg a CPU core. */
      for (unsigned spins = 0; !panvk_event_is_set(op->event); spins++) {
         if (vk_device_is_lost(&dev->vk))
            return VK_ERROR_DEVICE_LOST;

         if (spins < 1000)
            thrd_yield();
         else
            os_time_sleep(100);
      }
   }

   return VK_SUCCESS;
}

static VkResult
panvk_queue_signal_events(struct panvk_gpu_queue *queue,
                          struct panvk_batch *batch)
{
   util_dynarray_foreach(&batch->event_ops, struct panvk_cmd_event_op, op) {
      switch (op->type) {
      case PANVK_EVENT_OP_SET:
         panvk_event_update(op->event, true);
         break;
      case PANVK_EVENT_OP_RESET:
         panvk_event_update(op->event, false);
         break;
      case PANVK_EVENT_OP_WAIT:
         /* Handled up-front in panvk_queue_wait_events(). */
         break;
      default:
         UNREACHABLE("bad panvk_cmd_event_op type\n");
      }
   }

   return VK_SUCCESS;
}

/* Submit every batch of a command buffer in order. A batch that executed
 * secondary command buffers (vkCmdExecuteCommands) submits them right after
 * its own work and events; secondaries can't nest, so the recursion is one
 * level deep. */
static VkResult
panvk_queue_submit_cmdbuf(struct panvk_gpu_queue *queue,
                          struct panvk_cmd_buffer *cmdbuf)
{
   VkResult result;

   list_for_each_entry(struct panvk_batch, batch, &cmdbuf->batches, node) {
      result = panvk_queue_wait_events(queue, batch);
      if (result != VK_SUCCESS)
         return result;

      if (!panvk_queue_submit_batch(queue, cmdbuf, batch))
         return vk_queue_set_lost(&queue->vk,
                                  "kbase JM atom submission failed");

      result = panvk_queue_signal_events(queue, batch);
      if (result != VK_SUCCESS)
         return result;

      util_dynarray_foreach(&batch->exec_cmdbufs, struct panvk_cmd_buffer *,
                            secondary) {
         result = panvk_queue_submit_cmdbuf(queue, *secondary);
         if (result != VK_SUCCESS)
            return result;
      }
   }

   return VK_SUCCESS;
}

VkResult
panvk_per_arch(gpu_queue_submit)(struct vk_queue *vk_queue,
                                 struct vk_queue_submit *submit)
{
   struct panvk_gpu_queue *queue =
      container_of(vk_queue, struct panvk_gpu_queue, vk);
   struct panvk_device *dev = to_panvk_device(queue->vk.base.device);
   VkResult result;

   panvk_kbase_selftest(queue);

   /* A kbase atom's pre_dep can only chain onto prior atoms *on this same
    * queue*; there's no way to hand it an external semaphore as a
    * GPU-side dependency. So wait semaphores are resolved on the CPU,
    * before we submit anything. This queue's own submissions are already
    * fully synchronous (see panvk_queue_jm_submit_atom()), so this doesn't
    * give up any pipelining we'd otherwise have had. */
   for (unsigned i = 0; i < submit->wait_count; i++) {
      result = vk_sync_wait(&dev->vk, submit->waits[i].sync,
                            submit->waits[i].wait_value,
                            VK_SYNC_WAIT_COMPLETE, UINT64_MAX);
      if (result != VK_SUCCESS)
         return result;
   }

   for (uint32_t j = 0; j < submit->command_buffer_count; ++j) {
      struct panvk_cmd_buffer *cmdbuf = container_of(
         submit->command_buffers[j], struct panvk_cmd_buffer, vk);

      result = panvk_queue_submit_cmdbuf(queue, cmdbuf);
      if (result != VK_SUCCESS)
         return result;
   }

   /* Every batch above already ran to completion by the time we get here,
    * so signalling is pure host-side bookkeeping. */
   for (unsigned i = 0; i < submit->signal_count; i++) {
      result = vk_sync_signal(&dev->vk, submit->signals[i].sync,
                              submit->signals[i].signal_value);
      if (result != VK_SUCCESS)
         return result;
   }

   return VK_SUCCESS;
}

VkResult
panvk_per_arch(create_gpu_queue)(struct panvk_device *device,
                                 const VkDeviceQueueCreateInfo *create_info,
                                 uint32_t queue_idx,
                                 struct vk_queue **out_queue)
{
   ASSERTED const VkDeviceQueueGlobalPriorityCreateInfoKHR *priority_info =
      vk_find_struct_const(create_info->pNext,
                           DEVICE_QUEUE_GLOBAL_PRIORITY_CREATE_INFO_KHR);
   ASSERTED const VkQueueGlobalPriorityKHR priority =
      priority_info ? priority_info->globalPriority
                    : VK_QUEUE_GLOBAL_PRIORITY_MEDIUM_KHR;

   /* XXX: struct base_jd_atom only carries a per-atom BASE_JD_PRIO_*
    * value, not a queue-wide priority negotiated at creation time, so we
    * don't plumb anything beyond MEDIUM through yet. */
   assert(priority == VK_QUEUE_GLOBAL_PRIORITY_MEDIUM_KHR);

   struct panvk_gpu_queue *queue =
      vk_zalloc(&device->vk.alloc, sizeof(*queue), 8,
               VK_SYSTEM_ALLOCATION_SCOPE_DEVICE);
   if (!queue)
      return panvk_error(device, VK_ERROR_OUT_OF_HOST_MEMORY);

   VkResult result =
      vk_queue_init(&queue->vk, &device->vk, create_info, queue_idx);
   if (result != VK_SUCCESS)
      goto err_free_queue;

   /* This MUST run on every successful path out of this function: it's
    * what wires panvk_per_arch(gpu_queue_submit)() into the generic
    * Vulkan-runtime queue-submission dispatcher. If it's ever skipped,
    * the very first vkQueueSubmit() calls through a NULL function
    * pointer and crashes before issuing a single ioctl. */
   queue->vk.driver_submit = panvk_per_arch(gpu_queue_submit);
   *out_queue = &queue->vk;
   return VK_SUCCESS;

err_free_queue:
   vk_free(&device->vk.alloc, queue);
   return result;
}

void
panvk_per_arch(destroy_gpu_queue)(struct vk_queue *vk_queue)
{
   struct panvk_gpu_queue *queue =
      container_of(vk_queue, struct panvk_gpu_queue, vk);
   struct panvk_device *dev = to_panvk_device(vk_queue->base.device);

   vk_queue_finish(&queue->vk);
   vk_free(&dev->vk.alloc, queue);
}

VkResult
panvk_per_arch(gpu_queue_check_status)(struct vk_queue *vk_queue)
{
   return VK_SUCCESS;
}

VKAPI_ATTR VkResult VKAPI_CALL
panvk_per_arch(QueueWaitIdle)(VkQueue _queue)
{
   VK_FROM_HANDLE(panvk_gpu_queue, queue, _queue);
   struct panvk_device *dev = to_panvk_device(queue->vk.base.device);

   /* we need to use vk_common_QueueWaitIdle if we ever go threaded */
   assert(queue->vk.submit.mode != VK_QUEUE_SUBMIT_MODE_THREADED);

   if (vk_device_is_lost(&dev->vk)) {
      /* Check printf buffer one more time before exiting */
      u_printf_with_ctx(stdout, &dev->printf.ctx);
      return VK_ERROR_DEVICE_LOST;
   }

   /* panvk_per_arch(gpu_queue_submit)() already blocks until every atom it
    * submits has completed (see panvk_queue_jm_submit_atom()), so there is
    * nothing left to wait for by the time control returns to this queue's
    * caller. */
   return VK_SUCCESS;
}
