/*
 * Copyright © 2021 Collabora Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef PANVK_QUEUE_H
#define PANVK_QUEUE_H

#ifndef PAN_ARCH
#error "PAN_ARCH must be defined"
#endif

#include <stdint.h>

#include "panvk_device.h"

#include "vk_queue.h"

struct panvk_gpu_queue {
   struct vk_queue vk;
   uint32_t sync;

   /* Last kbase JM atom_number submitted on this queue (0 = none yet).
    * atom_number only identifies one atom (1..255, wraps) and is NOT used
    * as a pre_dep target: submission is synchronous -- completion is only
    * observable by draining the shared poll()+read() base_jd_event_v2
    * stream on the device fd, so every atom is awaited right after it is
    * submitted and this queue only ever has one atom in flight. See
    * panvk_queue_jm_submit_atom() in panvk_vX_gpu_queue.c. */
   uint8_t jm_last_atom;

   /* Last base_jd_atom::seq_nr ("sequence number of logical grouping of
    * atoms") handed out on this queue (0 = none yet). One value per batch:
    * a batch's vertex/tiler atom opens a new group and its fragment atom
    * joins it. 64-bit and strictly increasing, so unlike atom_number it
    * never wraps or repeats. See panvk_kbase_seq_nr_assign() in
    * panvk_vX_gpu_queue.c. */
   uint64_t jm_seq_nr;
};

VK_DEFINE_HANDLE_CASTS(panvk_gpu_queue, vk.base, VkQueue, VK_OBJECT_TYPE_QUEUE)

VkResult panvk_per_arch(create_gpu_queue)(
   struct panvk_device *device, const VkDeviceQueueCreateInfo *create_info,
   uint32_t queue_idx, struct vk_queue **out_queue);
void panvk_per_arch(destroy_gpu_queue)(struct vk_queue *vk_queue);
VkResult panvk_per_arch(gpu_queue_submit)(struct vk_queue *vk_queue,
                                          struct vk_queue_submit *vk_submit);
VkResult panvk_per_arch(gpu_queue_check_status)(struct vk_queue *vk_queue);

#endif
