/*
 * Copyright © 2024 Collabora Ltd.
 *
 * Derived from tu_cmd_buffer.c which is:
 * Copyright © 2016 Red Hat.
 * Copyright © 2016 Bas Nieuwenhuizen
 * Copyright © 2015 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdint.h>
#include <string.h>
#include "genxml/gen_macros.h"

#include "panvk_buffer.h"
#include "panvk_cmd_alloc.h"
#include "panvk_cmd_buffer.h"
#include "panvk_cmd_desc_state.h"
#include "panvk_cmd_precomp.h"
#include "panvk_descriptor_set.h"
#include "panvk_device.h"
#include "panvk_entrypoints.h"
#include "panvk_macros.h"
#include "panvk_meta.h"
#include "panvk_physical_device.h"

#include "pan_desc.h"
#include "pan_encoder.h"
#include "pan_jc.h"
#include "pan_props.h"

#include "util/log.h"

#include <vulkan/vulkan_core.h>

uint64_t
panvk_per_arch(cmd_dispatch_prepare_tls)(
   struct panvk_cmd_buffer *cmdbuf, const struct panvk_shader_variant *cs,
   const struct pan_compute_dim *dim, bool indirect)
{
   struct panvk_batch *batch = cmdbuf->cur_batch;

   assert(batch);

   struct panvk_physical_device *phys_dev =
      to_panvk_physical_device(cmdbuf->vk.base.device->physical);

   panvk_per_arch(cmd_alloc_tls_desc)(cmdbuf, false);

   batch->tlsinfo.tls.size = MAX2(cs->info.tls_size, batch->tlsinfo.tls.size);
   batch->tlsinfo.wls.size = cs->info.wls_size;

   if (batch->tlsinfo.wls.size) {
      unsigned core_id_range =
         pan_query_core_id_range(&phys_dev->kmod.dev->props);

      batch->tlsinfo.wls.instances = pan_calc_wls_instances(
         &cs->cs.local_size, &phys_dev->kmod.dev->props, indirect ? NULL : dim);
      batch->wls_total_size = pan_calc_total_wls_size(
         batch->tlsinfo.wls.size, batch->tlsinfo.wls.instances, core_id_range);
   }

   return batch->tls.gpu;
}

#if PAN_ARCH < 9
static void
cmd_dispatch(struct panvk_cmd_buffer *cmdbuf, struct panvk_dispatch_info *info)
{
   const struct panvk_shader_variant *cs =
      panvk_shader_only_variant(cmdbuf->state.compute.shader);
   VkResult result;

   /* If there's no compute shader, we can skip the dispatch. */
   if (!panvk_priv_mem_check_alloc(cs->rsd))
      return;

   panvk_per_arch(cmd_close_batch)(cmdbuf);
   struct panvk_batch *batch = panvk_per_arch(cmd_open_batch)(cmdbuf);

   const struct panvk_shader_desc_info *cs_desc_info =
      &cmdbuf->state.compute.shader->desc_info;
   struct panvk_descriptor_state *desc_state =
      &cmdbuf->state.compute.desc_state;
   struct panvk_shader_desc_state *cs_desc_state =
      &cmdbuf->state.compute.cs.desc;

   struct pan_compute_dim wg_count = {
      info->direct.wg_count.x,
      info->direct.wg_count.y,
      info->direct.wg_count.z,
   };
   bool indirect = info->indirect.buffer_dev_addr != 0;
   uint64_t tsd = panvk_per_arch(cmd_dispatch_prepare_tls)(cmdbuf, cs,
                                                           &wg_count, indirect);

   result = panvk_per_arch(cmd_prepare_push_descs)(
      cmdbuf, desc_state, cs_desc_info->used_set_mask);
   if (result != VK_SUCCESS)
      return;

   struct pan_ptr copy_desc_job = {0};

   if (compute_state_dirty(cmdbuf, CS) ||
       compute_state_dirty(cmdbuf, DESC_STATE)) {
      result = panvk_per_arch(cmd_prepare_shader_desc_tables)(
         cmdbuf, desc_state, cs_desc_info, true, cs_desc_state);
      if (result != VK_SUCCESS)
         return;

      result = panvk_per_arch(cmd_prepare_dyn_ssbos)(
         cmdbuf, desc_state, cs_desc_info, cs_desc_state);
      if (result != VK_SUCCESS)
         return;

      result = panvk_per_arch(meta_get_copy_desc_job)(
         cmdbuf, cs_desc_info, &cmdbuf->state.compute.desc_state,
         cs_desc_state, 0, &copy_desc_job);
      if (result != VK_SUCCESS)
         return;

      if (copy_desc_job.cpu)
         util_dynarray_append(&batch->jobs, copy_desc_job.cpu);
   }

   panvk_per_arch(cmd_prepare_dispatch_sysvals)(cmdbuf, info);

   struct pan_ptr push_uniforms;
   result = panvk_per_arch(cmd_prepare_compute_push_uniforms)(
      cmdbuf, cs, &push_uniforms);
   if (result != VK_SUCCESS)
      return;
   cmdbuf->state.compute.push_uniforms = push_uniforms.gpu;

   struct pan_ptr job = panvk_cmd_alloc_desc(cmdbuf, COMPUTE_JOB);
   if (!job.gpu)
      return;

   util_dynarray_append(&batch->jobs, job.cpu);

   if (!indirect) {
      pan_pack_work_groups_compute(
         pan_section_ptr(job.cpu, COMPUTE_JOB, INVOCATION), wg_count.x,
         wg_count.y, wg_count.z, cs->cs.local_size.x,
         cs->cs.local_size.y, cs->cs.local_size.z, false, false);
   }

   pan_section_pack(job.cpu, COMPUTE_JOB, PARAMETERS, cfg) {
      cfg.job_task_split = util_logbase2_ceil(cs->cs.local_size.x + 1) +
                           util_logbase2_ceil(cs->cs.local_size.y + 1) +
                           util_logbase2_ceil(cs->cs.local_size.z + 1);
   }

   pan_section_pack(job.cpu, COMPUTE_JOB, DRAW, cfg) {
      cfg.state = panvk_priv_mem_dev_addr(cs->rsd);
      cfg.attributes = cs_desc_state->img_attrib_table;
      cfg.attribute_buffers =
         cs_desc_state->tables[PANVK_BIFROST_DESC_TABLE_IMG];
      cfg.thread_storage = tsd;
      cfg.uniform_buffers = cs_desc_state->tables[PANVK_BIFROST_DESC_TABLE_UBO];
      cfg.push_uniforms = cmdbuf->state.compute.push_uniforms;
      cfg.textures = cs_desc_state->tables[PANVK_BIFROST_DESC_TABLE_TEXTURE];
      cfg.samplers = cs_desc_state->tables[PANVK_BIFROST_DESC_TABLE_SAMPLER];
   }

   unsigned indirect_dep = 0;
   if (indirect) {
      /* We redirect write to memory sink for null pointers */
      uint64_t num_work_groups_x_sysval_addr = 0x8ull << 60;
      uint64_t num_work_groups_y_sysval_addr = 0x8ull << 60;
      uint64_t num_work_groups_z_sysval_addr = 0x8ull << 60;

      if (shader_uses_sysval(cs, compute, num_work_groups.x)) {
         num_work_groups_x_sysval_addr =
            cmdbuf->state.compute.push_uniforms +
            shader_remapped_sysval_offset(
               cs, sysval_offset(compute, num_work_groups.x));
      }

      if (shader_uses_sysval(cs, compute, num_work_groups.y)) {
         num_work_groups_y_sysval_addr =
            cmdbuf->state.compute.push_uniforms +
            shader_remapped_sysval_offset(
               cs, sysval_offset(compute, num_work_groups.y));
      }

      if (shader_uses_sysval(cs, compute, num_work_groups.z)) {
         num_work_groups_z_sysval_addr =
            cmdbuf->state.compute.push_uniforms +
            shader_remapped_sysval_offset(
               cs, sysval_offset(compute, num_work_groups.z));
      }

      struct panvk_precomp_ctx precomp_ctx = panvk_per_arch(precomp_cs)(cmdbuf);
      enum panlib_barrier precomp_barrier =
         copy_desc_job.gpu == 0 ? PANLIB_BARRIER_JM_SUPPRESS_PREFETCH
                                : PANLIB_BARRIER_NONE;

      panlib_indirect_dispatch(
         &precomp_ctx, panlib_1d(1), precomp_barrier,
         info->indirect.buffer_dev_addr, cs->cs.local_size.x,
         cs->cs.local_size.y, cs->cs.local_size.z, job.gpu,
         num_work_groups_x_sysval_addr, num_work_groups_y_sysval_addr,
         num_work_groups_z_sysval_addr);
      indirect_dep = batch->vtc_jc.job_index;
   }

   util_dynarray_append(&batch->jobs, job.cpu);

   unsigned copy_desc_dep =
      copy_desc_job.gpu
         ? pan_jc_add_job(&batch->vtc_jc, MALI_JOB_TYPE_COMPUTE, false,
                          indirect, 0, indirect_dep, &copy_desc_job, false)
         : indirect_dep;

   pan_jc_add_job(&batch->vtc_jc,
                  indirect ? MALI_JOB_TYPE_NOT_STARTED : MALI_JOB_TYPE_COMPUTE,
                  false, false, 0, copy_desc_dep, &job, false);

   panvk_per_arch(cmd_close_batch)(cmdbuf);
   clear_dirty_after_dispatch(cmdbuf);
}
#else /* PAN_ARCH >= 9 */
/* Valhall + JM compute dispatch.
 *
 * Valhall dropped the Bifrost RSD/attribute/UBO/texture/sampler tables: a
 * compute job carries one SHADER_ENVIRONMENT (resource table, SPD, TSD, FAU)
 * plus the workgroup size/count, exactly like the Gallium driver's
 * PAN_ARCH >= 9 launch_grid path.  The resource table and driver set are
 * built with the same CPU-only helpers the CSF backend (and this tree's v9
 * graphics path) already use.
 *
 * Without this, every compute-based meta operation was silently dropped:
 * vkCmdCopyBuffer, vkCmdCopyImageToBuffer, vkCmdFillBuffer, ... completed
 * "successfully" and wrote nothing. */
/* Everything that goes into a COMPUTE_JOB payload. Kept as a plain struct so
 * the payload can be packed again after an indirect dispatch's workgroup
 * counts have been read back (see jm_apply_indirect_dispatch()). */
struct jm_compute_params {
   uint32_t local_size[3];
   uint32_t count[3];
   uint64_t res_table;
   uint64_t spd;
   uint64_t fau;
   uint64_t tsd;
   uint32_t fau_count;
   bool allow_merging;
};

static void
jm_pack_compute_payload(void *job_cpu, const struct jm_compute_params *p)
{
   pan_section_pack(job_cpu, COMPUTE_JOB, PAYLOAD, cfg) {
      cfg.workgroup_size_x = p->local_size[0];
      cfg.workgroup_size_y = p->local_size[1];
      cfg.workgroup_size_z = p->local_size[2];

      cfg.workgroup_count_x = p->count[0];
      cfg.workgroup_count_y = p->count[1];
      cfg.workgroup_count_z = p->count[2];

      cfg.compute.resources = p->res_table;
      cfg.compute.shader = p->spd;
      cfg.compute.thread_storage = p->tsd;
      cfg.compute.fau = p->fau;
      cfg.compute.fau_count = p->fau_count;

      cfg.allow_merging_workgroups = p->allow_merging;

      /* One workgroup per task along Z: always valid, the hardware clamps
       * the increment to the job size on that axis. */
      cfg.task_increment = 1;
      cfg.task_axis = MALI_TASK_AXIS_Z;
   }
}

static void
jm_fill_compute_params(struct jm_compute_params *p,
                       const struct panvk_shader_variant *cs,
                       const struct pan_compute_dim *dim, uint64_t res_table,
                       uint64_t fau_ptr, uint32_t fau_count, uint64_t tsd)
{
   *p = (struct jm_compute_params){
      .local_size = {cs->cs.local_size.x, cs->cs.local_size.y,
                     cs->cs.local_size.z},
      .count = {dim->x, dim->y, dim->z},
      .res_table = res_table,
      .spd = panvk_priv_mem_dev_addr(cs->spd),
      .fau = fau_ptr,
      .tsd = tsd,
      .fau_count = fau_count,
      .allow_merging = cs->info.cs.allow_merging_workgroups,
   };
}

VkResult
panvk_per_arch(jm_emit_compute_job)(
   struct panvk_cmd_buffer *cmdbuf, const struct panvk_shader_variant *cs,
   const struct pan_compute_dim *dim, uint64_t res_table, uint64_t fau_ptr,
   uint32_t fau_count, uint64_t tsd, bool barrier, bool suppress_prefetch,
   unsigned local_dep, unsigned global_dep)
{
   struct panvk_batch *batch = cmdbuf->cur_batch;
   assert(batch);

   struct pan_ptr job = panvk_cmd_alloc_desc(cmdbuf, COMPUTE_JOB);
   if (!job.gpu)
      return VK_ERROR_OUT_OF_DEVICE_MEMORY;

   util_dynarray_append(&batch->jobs, job.cpu);

   struct jm_compute_params params;
   jm_fill_compute_params(&params, cs, dim, res_table, fau_ptr, fau_count,
                          tsd);
   jm_pack_compute_payload(job.cpu, &params);

   pan_jc_add_job(&batch->vtc_jc, MALI_JOB_TYPE_COMPUTE, barrier,
                  suppress_prefetch, local_dep, global_dep, &job, false);
   return VK_SUCCESS;
}

/* XOR mask that turns a job header of type `a` into one of type `b`
 * (everything else in the header is identical, so only the type bits are
 * set). Lets the CPU flip a recorded job between NULL and its real type
 * without knowing the header's bit layout. */
void
panvk_per_arch(jm_job_type_xor)(uint32_t out[8], unsigned a, unsigned b)
{
   _Static_assert(sizeof(struct mali_job_header_packed) <=
                     8 * sizeof(uint32_t),
                  "the job header no longer fits in the XOR mask");

   /* pan_pack() writes through a pointer to the *packed struct*, so pack into
    * real struct mali_job_header_packed objects, not raw word arrays. */
   struct mali_job_header_packed ha, hb;
   uint32_t wa[8] = {0}, wb[8] = {0};

   pan_pack(&ha, JOB_HEADER, h) {
      h.type = (enum mali_job_type)a;
   }
   pan_pack(&hb, JOB_HEADER, h) {
      h.type = (enum mali_job_type)b;
   }

   memcpy(wa, &ha, sizeof(ha));
   memcpy(wb, &hb, sizeof(hb));

   for (unsigned i = 0; i < 8; i++)
      out[i] = wa[i] ^ wb[i];
}

bool
panvk_per_arch(jm_apply_indirect_dispatch)(
   struct panvk_jm_indirect_fixup *f, const uint32_t *args)
{
   assert(f->kind == PANVK_JM_INDIRECT_DISPATCH);

   if (!args[0] || !args[1] || !args[2])
      return false;

   struct jm_compute_params p = {
      .local_size = {f->dispatch.local_size[0], f->dispatch.local_size[1],
                     f->dispatch.local_size[2]},
      .count = {args[0], args[1], args[2]},
      .res_table = f->dispatch.res_table,
      .spd = f->dispatch.spd,
      .fau = f->dispatch.fau,
      .tsd = f->dispatch.tsd,
      .fau_count = f->dispatch.fau_count,
      .allow_merging = f->dispatch.allow_merging,
   };

   jm_pack_compute_payload(f->job, &p);

   /* gl_NumWorkGroups lives in the push-uniform block (a fresh block is
    * allocated per dispatch, so patching it in place is safe). */
   for (unsigned i = 0; i < 3; i++) {
      if (f->dispatch.num_wg_sysval[i])
         *f->dispatch.num_wg_sysval[i] = args[i];
   }

   return true;
}

static VkResult
prepare_cs_driver_set(struct panvk_cmd_buffer *cmdbuf)
{
   const struct panvk_shader_desc_info *cs_desc_info =
      &cmdbuf->state.compute.shader->desc_info;
   const struct panvk_descriptor_state *desc_state =
      &cmdbuf->state.compute.desc_state;
   struct panvk_shader_desc_state *cs_desc_state =
      &cmdbuf->state.compute.cs.desc;
   uint32_t desc_count = cs_desc_info->dyn_bufs.count + 1;
   struct pan_ptr driver_set = panvk_cmd_alloc_dev_mem(
      cmdbuf, desc, desc_count * PANVK_DESCRIPTOR_SIZE, PANVK_DESCRIPTOR_SIZE);
   struct panvk_opaque_desc *descs = driver_set.cpu;

   if (!driver_set.gpu)
      return VK_ERROR_OUT_OF_DEVICE_MEMORY;

   /* Dummy sampler always comes first. */
   pan_cast_and_pack(&descs[0], SAMPLER, cfg) {
      cfg.clamp_integer_array_indices = false;
   }

   panvk_per_arch(cmd_fill_dyn_bufs)(desc_state, cs_desc_info,
                                     (struct mali_buffer_packed *)(&descs[1]));

   cs_desc_state->driver_set.dev_addr = driver_set.gpu;
   cs_desc_state->driver_set.size = desc_count * PANVK_DESCRIPTOR_SIZE;
   return VK_SUCCESS;
}

static void
cmd_dispatch(struct panvk_cmd_buffer *cmdbuf, struct panvk_dispatch_info *info)
{
   const struct panvk_shader_variant *cs =
      panvk_shader_only_variant(cmdbuf->state.compute.shader);
   VkResult result;

   /* If there's no compute shader, we can skip the dispatch. */
   if (!panvk_priv_mem_check_alloc(cs->spd))
      return;

   const bool indirect = info->indirect.buffer_dev_addr != 0;
   if (indirect && !info->indirect.buffer) {
      /* The args are read back from the CPU, so we need the VkBuffer, not just
       * its device address. */
      mesa_loge("panvk: indirect dispatch without a source buffer; skipped");
      return;
   }

   panvk_per_arch(cmd_close_batch)(cmdbuf);
   struct panvk_batch *batch = panvk_per_arch(cmd_open_batch)(cmdbuf);
   if (!batch)
      return;

   const struct panvk_shader_desc_info *cs_desc_info =
      &cmdbuf->state.compute.shader->desc_info;
   struct panvk_descriptor_state *desc_state =
      &cmdbuf->state.compute.desc_state;
   struct panvk_shader_desc_state *cs_desc_state =
      &cmdbuf->state.compute.cs.desc;

   struct pan_compute_dim dim = {
      info->direct.wg_count.x,
      info->direct.wg_count.y,
      info->direct.wg_count.z,
   };

   /* With indirect=true the WLS instance count is computed for the worst case
    * since the real workgroup count is only known at submit time. */
   uint64_t tsd =
      panvk_per_arch(cmd_dispatch_prepare_tls)(cmdbuf, cs, &dim, indirect);
   if (!tsd)
      return;

   /* The resource table lives in the *command buffer's* pools and a new
    * batch was just opened, so always rebuild it rather than trusting the
    * dirty bits left by a previous dispatch. */
   result = panvk_per_arch(cmd_prepare_push_descs)(
      cmdbuf, desc_state, cs_desc_info->used_set_mask);
   if (result != VK_SUCCESS)
      return;

   panvk_per_arch(cmd_prepare_dispatch_sysvals)(cmdbuf, info);

   result = prepare_cs_driver_set(cmdbuf);
   if (result != VK_SUCCESS)
      return;

   struct pan_ptr push_uniforms;
   result = panvk_per_arch(cmd_prepare_compute_push_uniforms)(
      cmdbuf, cs, &push_uniforms);
   if (result != VK_SUCCESS)
      return;
   cmdbuf->state.compute.push_uniforms = push_uniforms.gpu;

   result = panvk_per_arch(cmd_prepare_shader_res_table)(
      cmdbuf, desc_state, cs_desc_info, cs_desc_state, 1);
   if (result != VK_SUCCESS)
      return;

   if (!indirect) {
      result = panvk_per_arch(jm_emit_compute_job)(
         cmdbuf, cs, &dim, cs_desc_state->res_table,
         cmdbuf->state.compute.push_uniforms, cs->fau.total_count, tsd, false,
         false, 0, 0);
      if (result != VK_SUCCESS)
         return;
   } else {
      /* Record the job as a NULL job with placeholder counts and remember
       * everything needed to finish it when the batch is submitted. */
      struct pan_ptr job = panvk_cmd_alloc_desc(cmdbuf, COMPUTE_JOB);
      if (!job.gpu)
         return;

      util_dynarray_append(&batch->jobs, job.cpu);

      struct pan_compute_dim placeholder = {1, 1, 1};
      struct jm_compute_params params;
      jm_fill_compute_params(&params, cs, &placeholder,
                             cs_desc_state->res_table,
                             cmdbuf->state.compute.push_uniforms,
                             cs->fau.total_count, tsd);
      jm_pack_compute_payload(job.cpu, &params);

      pan_jc_add_job(&batch->vtc_jc, MALI_JOB_TYPE_NULL, false, false, 0, 0,
                     &job, false);

      struct panvk_jm_indirect_fixup fixup = {
         .kind = PANVK_JM_INDIRECT_DISPATCH,
         .buffer = info->indirect.buffer,
         .offset = info->indirect.offset,
         .job = job.cpu,
         .dispatch = {
            .local_size = {params.local_size[0], params.local_size[1],
                           params.local_size[2]},
            .res_table = params.res_table,
            .spd = params.spd,
            .fau = params.fau,
            .tsd = params.tsd,
            .fau_count = params.fau_count,
            .allow_merging = params.allow_merging,
         },
      };
      panvk_per_arch(jm_job_type_xor)(fixup.type_xor, MALI_JOB_TYPE_NULL,
                                      MALI_JOB_TYPE_COMPUTE);

      if (push_uniforms.cpu) {
         uint8_t *fau_cpu = push_uniforms.cpu;

         if (shader_uses_sysval(cs, compute, num_work_groups.x))
            fixup.dispatch.num_wg_sysval[0] = (uint32_t *)(fau_cpu +
               shader_remapped_sysval_offset(
                  cs, sysval_offset(compute, num_work_groups.x)));
         if (shader_uses_sysval(cs, compute, num_work_groups.y))
            fixup.dispatch.num_wg_sysval[1] = (uint32_t *)(fau_cpu +
               shader_remapped_sysval_offset(
                  cs, sysval_offset(compute, num_work_groups.y)));
         if (shader_uses_sysval(cs, compute, num_work_groups.z))
            fixup.dispatch.num_wg_sysval[2] = (uint32_t *)(fau_cpu +
               shader_remapped_sysval_offset(
                  cs, sysval_offset(compute, num_work_groups.z)));
      }

      util_dynarray_append(&batch->indirect_fixups, fixup);
   }

   panvk_per_arch(cmd_close_batch)(cmdbuf);
   clear_dirty_after_dispatch(cmdbuf);
}
#endif

VKAPI_ATTR void VKAPI_CALL
panvk_per_arch(CmdDispatchBase)(VkCommandBuffer commandBuffer,
                                uint32_t baseGroupX, uint32_t baseGroupY,
                                uint32_t baseGroupZ, uint32_t groupCountX,
                                uint32_t groupCountY, uint32_t groupCountZ)
{
   VK_FROM_HANDLE(panvk_cmd_buffer, cmdbuf, commandBuffer);

   if (groupCountX == 0 || groupCountY == 0 || groupCountZ == 0)
      return;

   struct panvk_dispatch_info info = {
      .wg_base = {baseGroupX, baseGroupY, baseGroupZ},
      .direct.wg_count = {groupCountX, groupCountY, groupCountZ},
   };
   cmd_dispatch(cmdbuf, &info);
}

VKAPI_ATTR void VKAPI_CALL
panvk_per_arch(CmdDispatchIndirect)(VkCommandBuffer commandBuffer,
                                    VkBuffer _buffer, VkDeviceSize offset)
{
   VK_FROM_HANDLE(panvk_cmd_buffer, cmdbuf, commandBuffer);
   VK_FROM_HANDLE(panvk_buffer, buffer, _buffer);
   uint64_t buffer_gpu = panvk_buffer_gpu_ptr(buffer, offset);
   struct panvk_dispatch_info info = {
      .indirect.buffer_dev_addr = buffer_gpu,
      .indirect.buffer = buffer,
      .indirect.offset = offset,
   };
   cmd_dispatch(cmdbuf, &info);
}
