/*
 * Copyright © 2021 Collabora Ltd.
 * Copyright © 2025 Arm Ltd.
 *
 * Derived from tu_wsi.c:
 * Copyright © 2016 Red Hat
 * Copyright © 2015 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 */

#include "panvk_wsi.h"
#include "panvk_instance.h"
#include "panvk_physical_device.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(HAVE_PAN_KMOD_KBASE)
#include "lib/kmod/kbase_kmod.h"
#endif

#include "vk_util.h"
#include "wsi_common.h"

static VKAPI_PTR PFN_vkVoidFunction
panvk_wsi_proc_addr(VkPhysicalDevice physicalDevice, const char *pName)
{
   VK_FROM_HANDLE(panvk_physical_device, pdevice, physicalDevice);
   struct panvk_instance *instance = to_panvk_instance(pdevice->vk.instance);

   return vk_instance_get_proc_addr_unchecked(&instance->vk, pName);
}

static bool
panvk_can_present_on_device(VkPhysicalDevice pdevice, int fd)
{
   /* BUG (was unconditional): this hook exists purely so a driver can
    * tell WSI "rendering and scanout are the same device, skip the
    * PRIME/foreign-device blit path" -- see Danylo Piliaiev's original
    * tu_wsi_can_present_on_device() (MR!11091), which this was copied
    * from. It assumes `fd` is a real DRM fd and asks drmGetDevice2()
    * whether its bus type is PLATFORM.
    *
    * Under kbase, `fd` is the kbase device node (e.g. /dev/mali0), not
    * a DRM fd at all -- there is no KMS/render node behind it, so
    * drmGetDevice2() can only ever fail on it, and this function
    * unconditionally returned false whenever kbase was in use. That
    * forces WSI down the "different device, blit via PRIME" path on
    * every present -- and on Termux/Android there is no second, real
    * display-controller device for that path to blit *to* in the first
    * place. Falling into that path with nothing on the other end is a
    * strong candidate for "submits and completes every frame, window
    * never updates": the app-visible submit/acquire/present calls can
    * all return VK_SUCCESS while the actual scanout step has nothing
    * valid to do.
    *
    * A kbase-driven Mali GPU is a platform-bus device by construction
    * (there's no PCI/USB Mali), which is exactly the case the DRM check
    * below was trying to detect -- so the correct, direct translation
    * for kbase is to skip the DRM query entirely and answer "yes".
    */
   VK_FROM_HANDLE(panvk_physical_device, pdev, pdevice);

   if (pdev->kbase_node_path[0] != '\0')
      return true;

   drmDevicePtr device;
   if (drmGetDevice2(fd, 0, &device) != 0)
      return false;
   /* Allow on-device presentation for all devices with bus type PLATFORM.
    * Other device types such as PCI or USB should use the PRIME blit path. */
   bool match = device->bustype == DRM_BUS_PLATFORM;

   drmFreeDevice(&device);

   return match;
}

/* ROOT CAUSE of "renders fine, window stays black" on Valhall/kbase:
 *
 * With sw_device=true, common WSI defaults to its "buffer blit" strategy:
 * the swapchain image is an OPTIMAL-tiled image (PanVK picks AFBC for it),
 * plus a separate linear host-visible VkBuffer.  On every present WSI
 * records vkCmdCopyImageToBuffer(image -> buffer) and the X11 SHM/PutImage
 * path then reads that buffer from the CPU.
 *
 * On PAN_ARCH >= 9 on the JM backend, compute dispatch used to be a no-op
 * (cmd_dispatch() in jm/panvk_vX_cmd_dispatch.c), and
 * CmdCopyImageToBuffer is a vk_meta *compute* copy.  So the copy was
 * silently dropped: the log showed only vertex+fragment atoms, never a
 * compute job, and the linear present buffer stayed all zeros.
 *
 * Compute dispatch now exists on v9/JM (jm_emit_compute_job), so the blit
 * path works too; linear stays the default because it is faster (no extra
 * copy per frame) and has no dependency on compute at all.
 *
 * Fix: ask WSI for a plain LINEAR, host-visible swapchain image that the
 * app renders into directly (no blit, no compute, no AFBC).  The
 * MESA_VK_WSI_DEBUG=linear switch is read inside wsi_device_init(), so it
 * must be in place before that call.  Set PANVK_KBASE_WSI_BLIT=1 to get the
 * old behaviour back (e.g. once v9 compute dispatch exists).
 */
static void
panvk_wsi_force_linear_swapchain(void)
{
   const char *keep_blit = getenv("PANVK_KBASE_WSI_BLIT");
   if (keep_blit && keep_blit[0] && keep_blit[0] != '0')
      return;

   const char *cur = getenv("MESA_VK_WSI_DEBUG");
   if (!cur || !cur[0]) {
      setenv("MESA_VK_WSI_DEBUG", "linear", 1);
      return;
   }

   /* Already asks for linear, or explicitly asks for the blit path. */
   if (strstr(cur, "linear") || strstr(cur, "buffer"))
      return;

   char merged[256];
   snprintf(merged, sizeof(merged), "%s,linear", cur);
   setenv("MESA_VK_WSI_DEBUG", merged, 1);
}

VkResult
panvk_wsi_init(struct panvk_physical_device *physical_device)
{
   struct panvk_instance *instance =
      to_panvk_instance(physical_device->vk.instance);
   const bool uses_kbase = physical_device->kbase_node_path[0] != '\0';
   const char *dri3_option = getenv("PANVK_KBASE_DRI3");
   /* The proprietary Termux:X11 layer uses WSI_X11_TERMUX=1 as its
    * process-wide switch.  Accept the same switch here when no explicit
    * PanVK override is present, so both ICDs exercise the same raw-FD DRI3
    * transport instead of silently falling back to the slower SHM path. */
   const char *termux_wsi = getenv("WSI_X11_TERMUX");
   const bool termux_raw_dri3 =
      uses_kbase && !dri3_option && termux_wsi && strcmp(termux_wsi, "0") != 0;
   const bool kbase_raw_dri3 =
      termux_raw_dri3 ||
      (uses_kbase && dri3_option &&
       (!strcmp(dri3_option, "raw") || !strcmp(dri3_option, "termux")));
   const bool kbase_dmabuf =
#if defined(HAVE_PAN_KMOD_KBASE)
      uses_kbase &&
      ((!dri3_option && termux_raw_dri3) ||
       (dri3_option && strcmp(dri3_option, "0") != 0)) &&
      kbase_kmod_supports_dmabuf(physical_device->kmod.dev);
#else
      false;
#endif
   VkResult result;

   /* The CPU/SHM presentation path (everything except raw/dmabuf DRI3)
    * must not depend on a GPU compute copy: see comment above. */
   if (uses_kbase && !kbase_dmabuf)
      panvk_wsi_force_linear_swapchain();

   result = wsi_device_init(&physical_device->wsi_device,
                            panvk_physical_device_to_handle(physical_device),
                            panvk_wsi_proc_addr, &instance->vk.alloc, -1,
                            &instance->drirc.options,
                            &(struct wsi_device_options){
                               .sw_device = uses_kbase && !kbase_dmabuf,
                               .wait_present_before_queue = kbase_dmabuf,
                               .x11_use_raw_fd_modifier =
                                  kbase_dmabuf && kbase_raw_dri3,
                            });
   if (result != VK_SUCCESS)
      return result;

   /* kbase syncs carry GPU seqno state in userspace and cannot be copied via
    * DRM syncobj fd payloads.  Keep even empty WSI submits on the real queue
    * so present fences are backed by the kbase submission timeline.
    */
   physical_device->wsi_device.disable_unordered_submits = uses_kbase;

   /* kbase is not a DRM fd.  The default CPU WSI path presents through
    * MIT-SHM.  Android dma-heaps can also provide shareable
    * allocations, but some Android X servers advertise DRI3 while rejecting
    * standard PixmapFromBuffer.  PANVK_KBASE_DRI3=raw (or termux) uses the
    * Termux:X11/Winlator private raw-FD modifier instead of DRM modifiers.
    */
   physical_device->wsi_device.supports_modifiers =
      !uses_kbase || (kbase_dmabuf && !kbase_raw_dri3);
   physical_device->wsi_device.can_present_on_device =
      panvk_can_present_on_device;

   physical_device->vk.wsi_device = &physical_device->wsi_device;

   return VK_SUCCESS;
}

void
panvk_wsi_finish(struct panvk_physical_device *physical_device)
{
   struct panvk_instance *instance =
      to_panvk_instance(physical_device->vk.instance);

   physical_device->vk.wsi_device = NULL;
   wsi_device_finish(&physical_device->wsi_device, &instance->vk.alloc);
}
