/*
 * Copyright © 2021 Collabora Ltd.
 * SPDX-License-Identifier: MIT
 */

#ifndef PANVK_EVENT_H
#define PANVK_EVENT_H

#include <stdbool.h>
#include <stdint.h>

#include "vk_object.h"

/* On kbase there are no DRM syncobjs (there is no DRM device at all), so a
 * VkEvent is a plain host-visible flag.
 *
 * vkSetEvent()/vkResetEvent()/vkGetEventStatus() access it directly.
 * vkCmdSetEvent2()/vkCmdResetEvent2() are recorded as batch event ops and
 * applied by the queue when the batch has completed; vkCmdWaitEvents2() makes
 * the queue wait for the flag before it submits the batch (the queue submits
 * synchronously, so waiting on the CPU is equivalent to a GPU-side wait). */
struct panvk_event {
   struct vk_object_base base;
   uint32_t signaled;
};

VK_DEFINE_NONDISP_HANDLE_CASTS(panvk_event, base, VkEvent, VK_OBJECT_TYPE_EVENT)

static inline bool
panvk_event_is_set(const struct panvk_event *event)
{
   return __atomic_load_n(&event->signaled, __ATOMIC_ACQUIRE) != 0;
}

static inline void
panvk_event_update(struct panvk_event *event, bool set)
{
   __atomic_store_n(&event->signaled, set ? 1u : 0u, __ATOMIC_RELEASE);
}

#endif
