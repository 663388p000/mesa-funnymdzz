/*
 * Copyright © 2021 Collabora Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef PANVK_BUFFER_H
#define PANVK_BUFFER_H

#include <stdint.h>

#include "vk_buffer.h"

struct panvk_priv_bo;

struct panvk_device_memory;

struct panvk_buffer {
   struct vk_buffer vk;

   /* Memory binding. Only the Job Manager backend needs it: on Valhall/JM
    * indirect draw/dispatch arguments are read back on the CPU when the batch
    * is submitted (see panvk_jm_resolve_indirect()). NULL for sparse or still
    * unbound buffers. */
   struct panvk_device_memory *mem;
   uint64_t mem_offset;
};

VK_DEFINE_NONDISP_HANDLE_CASTS(panvk_buffer, vk.base, VkBuffer,
                               VK_OBJECT_TYPE_BUFFER)

static inline uint64_t
panvk_buffer_gpu_ptr(const struct panvk_buffer *buffer, uint64_t offset)
{
   if (!buffer->vk.device_address)
      return 0;

   return vk_buffer_address(&buffer->vk, offset);
}

static inline uint64_t
panvk_buffer_range(const struct panvk_buffer *buffer, uint64_t offset,
                   uint64_t range)
{
   if (!buffer->vk.device_address)
      return 0;

   return vk_buffer_range(&buffer->vk, offset, range);
}

#endif
