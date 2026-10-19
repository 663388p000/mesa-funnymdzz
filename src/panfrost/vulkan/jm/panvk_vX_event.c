/*
 * Copyright © 2021 Collabora Ltd.
 * SPDX-License-Identifier: MIT
 */

#include "panvk_device.h"
#include "panvk_entrypoints.h"
#include "panvk_event.h"

#include "vk_log.h"

VKAPI_ATTR VkResult VKAPI_CALL
panvk_per_arch(CreateEvent)(VkDevice _device,
                            const VkEventCreateInfo *pCreateInfo,
                            const VkAllocationCallbacks *pAllocator,
                            VkEvent *pEvent)
{
   VK_FROM_HANDLE(panvk_device, device, _device);

   /* vk_object_zalloc() leaves the event in the "reset" state. */
   struct panvk_event *event = vk_object_zalloc(
      &device->vk, pAllocator, sizeof(*event), VK_OBJECT_TYPE_EVENT);
   if (!event)
      return panvk_error(device, VK_ERROR_OUT_OF_HOST_MEMORY);

   *pEvent = panvk_event_to_handle(event);

   return VK_SUCCESS;
}

VKAPI_ATTR void VKAPI_CALL
panvk_per_arch(DestroyEvent)(VkDevice _device, VkEvent _event,
                             const VkAllocationCallbacks *pAllocator)
{
   VK_FROM_HANDLE(panvk_device, device, _device);
   VK_FROM_HANDLE(panvk_event, event, _event);

   if (!event)
      return;

   vk_object_free(&device->vk, pAllocator, event);
}

VKAPI_ATTR VkResult VKAPI_CALL
panvk_per_arch(GetEventStatus)(VkDevice _device, VkEvent _event)
{
   VK_FROM_HANDLE(panvk_device, device, _device);
   VK_FROM_HANDLE(panvk_event, event, _event);

   if (vk_device_is_lost(&device->vk))
      return VK_ERROR_DEVICE_LOST;

   return panvk_event_is_set(event) ? VK_EVENT_SET : VK_EVENT_RESET;
}

VKAPI_ATTR VkResult VKAPI_CALL
panvk_per_arch(SetEvent)(VkDevice _device, VkEvent _event)
{
   VK_FROM_HANDLE(panvk_event, event, _event);

   /* The spec mandates that the event has been set before any vkCmdWaitEvents
    * that depends on it executes, so there is nothing to wake up: the queue
    * polls the flag. */
   panvk_event_update(event, true);

   return VK_SUCCESS;
}

VKAPI_ATTR VkResult VKAPI_CALL
panvk_per_arch(ResetEvent)(VkDevice _device, VkEvent _event)
{
   VK_FROM_HANDLE(panvk_event, event, _event);

   panvk_event_update(event, false);

   return VK_SUCCESS;
}
