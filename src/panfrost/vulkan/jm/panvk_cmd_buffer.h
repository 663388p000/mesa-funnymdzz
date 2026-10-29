/*
 * Copyright © 2021 Collabora Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef PANVK_CMD_BUFFER_H
#define PANVK_CMD_BUFFER_H

#ifndef PAN_ARCH
#error "PAN_ARCH must be defined"
#endif

#include <stdint.h>

#include "vulkan/runtime/vk_command_buffer.h"

#include "panvk_cmd_desc_state.h"
#include "panvk_cmd_dispatch.h"
#include "panvk_cmd_draw.h"
#include "panvk_cmd_push_constant.h"
#include "panvk_descriptor_set.h"
#include "panvk_descriptor_set_layout.h"
#include "panvk_device.h"
#include "panvk_macros.h"
#include "panvk_mempool.h"
#include "panvk_shader.h"

#include "pan_jc.h"

#include "util/list.h"

#include "genxml/gen_macros.h"

struct panvk_buffer;

/* Valhall (PAN_ARCH >= 9) MALLOC_VERTEX_JOB: the hardware writes the vertex
 * array pointer and the vertex strides into the job's DRAW section while the
 * job runs ("Written by hardware in MallocVertexShader job mode").  When a
 * command buffer is submitted a second time, the stale values make the job
 * fault immediately (DATA_INVALID_FAULT, exception 0x58), so we remember what
 * the CPU originally wrote and put it back before every re-submission. */
struct panvk_jm_draw_fixup {
   uint32_t *words; /* &DRAW.word[2] of the job */
   uint32_t orig[3];
};

enum panvk_jm_indirect_kind {
   PANVK_JM_INDIRECT_DISPATCH,
   PANVK_JM_INDIRECT_DRAW,
   PANVK_JM_INDIRECT_DRAW_INDEXED,
};

/* Valhall (PAN_ARCH >= 9) job-manager hardware has no way to read workgroup
 * counts / draw arguments from memory (the Bifrost patch kernels only know the
 * Bifrost job layouts).  Like the Gallium driver, we resolve indirect commands
 * on the CPU: the job is recorded as a NULL job, and when the batch is
 * submitted -- every earlier atom has already completed, submission is
 * synchronous -- the arguments are read back, the job is patched and its
 * type is switched from NULL to the real one. */
struct panvk_jm_indirect_fixup {
   enum panvk_jm_indirect_kind kind;

   struct panvk_buffer *buffer;
   uint64_t offset;

   /* vkCmdDraw*IndirectCount: maxDrawCount placeholder draws are recorded;
    * the one at count_index only runs if the 32-bit count stored in
    * count_buffer (read back at submit time) is greater than count_index. */
   struct panvk_buffer *count_buffer;
   uint64_t count_offset;
   uint32_t count_index;

   void *job;               /* job descriptor, CPU pointer */
   uint32_t type_xor[8];    /* flips the header job type NULL <-> real type */
   bool enabled;            /* header currently holds the real type */

   union {
      struct {
         uint32_t local_size[3];
         uint64_t res_table;
         uint64_t spd;
         uint64_t fau;
         uint64_t tsd;
         uint32_t fau_count;
         bool allow_merging;
         uint32_t *num_wg_sysval[3]; /* NULL when the shader doesn't use it */
      } dispatch;

      struct {
         unsigned draw_mode;
         bool point_size_fp16;
         unsigned index_size;
         bool primitive_restart;
         bool secondary_shader;
         uint64_t index_buffer_dev_addr; /* index buffer base address */
         uint32_t *first_vertex_sysval;  /* NULL when the shader doesn't use it */
         uint32_t *base_instance_sysval; /* NULL when the shader doesn't use it */
      } draw;
   };
};

struct panvk_batch {
   struct list_head node;
   struct util_dynarray jobs;
   struct util_dynarray event_ops;
   /* Secondary command buffers executed (vkCmdExecuteCommands) right after
    * this batch's own work, in order. The queue submits them recursively. */
   struct util_dynarray exec_cmdbufs; /* struct panvk_cmd_buffer * */
   struct util_dynarray draw_fixups;     /* struct panvk_jm_draw_fixup */
   struct util_dynarray indirect_fixups; /* struct panvk_jm_indirect_fixup */
   struct pan_jc vtc_jc;
   struct pan_jc frag_jc;
   struct {
      struct pan_ptr desc;
      uint32_t desc_stride;
      uint32_t bo_count;

      /* One slot per color, two more slots for the depth/stencil buffers. */
      struct pan_kmod_bo *bos[MAX_RTS + 2];
      uint32_t layer_count;
   } fb;
   struct {
      struct pan_kmod_bo *src, *dst;
   } blit;
   struct pan_ptr tls;
   struct {
      struct pan_tiler_context ctx;
      struct pan_ptr heap_desc;
      struct pan_ptr ctx_descs;
      struct mali_tiler_heap_packed heap_templ;
      struct mali_tiler_context_packed ctx_templ;
   } tiler;
   struct pan_tls_info tlsinfo;
   unsigned wls_total_size;
   bool issued;
};

enum panvk_cmd_event_op_type {
   PANVK_EVENT_OP_SET,
   PANVK_EVENT_OP_RESET,
   PANVK_EVENT_OP_WAIT,
};

struct panvk_cmd_event_op {
   enum panvk_cmd_event_op_type type;
   struct panvk_event *event;
};

struct panvk_cmd_buffer {
   struct vk_command_buffer vk;

   struct panvk_pool desc_pool;
   struct panvk_pool varying_pool;
   struct panvk_pool tls_pool;
   struct list_head batches;
   struct list_head push_sets;
   struct panvk_batch *cur_batch;

   /* VkCommandBufferBeginInfo::flags of the current recording. */
   VkCommandBufferUsageFlags flags;

   struct {
      struct panvk_cmd_graphics_state gfx;
      struct panvk_cmd_compute_state compute;
      struct panvk_push_constant_state push_constants;
   } state;
};

VK_DEFINE_HANDLE_CASTS(panvk_cmd_buffer, vk.base, VkCommandBuffer,
                       VK_OBJECT_TYPE_COMMAND_BUFFER)

#define panvk_cmd_buffer_obj_list_init(cmdbuf, list_name)                      \
   list_inithead(&(cmdbuf)->list_name)

#define panvk_cmd_buffer_obj_list_cleanup(cmdbuf, list_name)                   \
   do {                                                                        \
      struct panvk_cmd_pool *__pool =                                          \
         container_of(cmdbuf->vk.pool, struct panvk_cmd_pool, vk);             \
      list_splicetail(&(cmdbuf)->list_name, &__pool->list_name);               \
   } while (0)

#define panvk_cmd_buffer_obj_list_reset(cmdbuf, list_name)                     \
   do {                                                                        \
      struct panvk_cmd_pool *__pool =                                          \
         container_of(cmdbuf->vk.pool, struct panvk_cmd_pool, vk);             \
      list_splicetail(&(cmdbuf)->list_name, &__pool->list_name);               \
      list_inithead(&(cmdbuf)->list_name);                                     \
   } while (0)

static inline struct panvk_descriptor_state *
panvk_cmd_get_desc_state(struct panvk_cmd_buffer *cmdbuf,
                         VkPipelineBindPoint bindpoint)
{
   switch (bindpoint) {
   case VK_PIPELINE_BIND_POINT_GRAPHICS:
      return &cmdbuf->state.gfx.desc_state;

   case VK_PIPELINE_BIND_POINT_COMPUTE:
      return &cmdbuf->state.compute.desc_state;

   default:
      assert(!"Unsupported bind point");
      return NULL;
   }
}

extern const struct vk_command_buffer_ops panvk_per_arch(cmd_buffer_ops);

struct panvk_batch *
   panvk_per_arch(cmd_open_batch)(struct panvk_cmd_buffer *cmdbuf);

void panvk_per_arch(cmd_close_batch)(struct panvk_cmd_buffer *cmdbuf);

VkResult panvk_per_arch(cmd_alloc_fb_desc)(struct panvk_cmd_buffer *cmdbuf);

VkResult panvk_per_arch(cmd_alloc_tls_desc)(struct panvk_cmd_buffer *cmdbuf,
                                            bool gfx);

VkResult
   panvk_per_arch(cmd_prepare_tiler_context)(struct panvk_cmd_buffer *cmdbuf,
                                             uint32_t layer_idx);

void panvk_per_arch(cmd_bind_shaders)(struct vk_command_buffer *vk_cmd,
                                      uint32_t stage_count,
                                      const mesa_shader_stage *stages,
                                      struct vk_shader **const shaders);

#if PAN_ARCH >= 9 && PAN_ARCH < 10
/* Apply the arguments read back from an indirect buffer to a recorded
 * placeholder job. Return true when the job must actually run, false when the
 * command turns out to be empty (zero workgroups / zero vertices or
 * instances). */
bool panvk_per_arch(jm_apply_indirect_dispatch)(
   struct panvk_jm_indirect_fixup *fixup, const uint32_t *args);
bool panvk_per_arch(jm_apply_indirect_draw)(
   struct panvk_jm_indirect_fixup *fixup, const uint32_t *args);
#endif

#endif
