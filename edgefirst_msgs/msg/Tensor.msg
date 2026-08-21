# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

# Tensor — generic n-dimensional buffer reference.
#
# A 1:1 projection of the edgefirst-tensor HAL blob. Carries storage,
# geometry, optional image interpretation and optional quantization. This
# library CARRIES these fields; it never interprets them -- it does not parse
# "NV12" and does not know what any dtype code means.
#
# Unstamped. Use TensorStamped for standalone tensor topics, or CameraFrame
# for camera topics. Tensor MUST be the last field of any message that embeds
# it, and MUST begin at an 8-byte-aligned offset.
#
# dtype and storage_kind codes are defined by the EdgeFirst HAL Modular Tensor
# ABI specification, which is their single authority. They are NOT declared in
# this package -- cite the HAL table, never transcribe it.

# ── Fixed-size scalar prefix: 20 bytes, every field O(1) ────────────
uint32 storage_kind      # Backing store for the whole tensor; HAL code
uint32 pid               # Producer pid, for pidfd_open/pidfd_getfd on both shm
                         # and dma-buf handles; 0 when all planes are inlined
int32 fence_fd           # ACQUIRE fence: a DMA-fence sync_file fd the consumer
                         # MUST wait on before reading any plane. Per-Tensor,
                         # never per-plane -- it is a point on the producer's
                         # submission timeline, not a property of a plane's
                         # bytes. -1 = already signalled / none, and MUST be -1
                         # when every plane is inlined or the transport cannot
                         # pass fds, where an fd is as meaningless as pid and
                         # handle are off-host. There is no release fence: a
                         # pub/sub topic has no return path to carry one.
uint32 dtype             # Element type; HAL code
int32 quant_axis         # Per-channel quantization axis; -1 = per-tensor, -2 = none
# ── Variable-length tail ────────────────────────────────────────────
uint64[] shape           # Dimensions, outermost first. The addressing grid, NOT
                         # the byte layout: for subsampled formats
                         # product(shape) * dtype_size is not the buffer size.
                         # Extent lives in planes[].size.
int64[] strides          # BYTES per dimension, not elements. Empty = densely
                         # packed C-order -- only safe for an unpadded buffer;
                         # any pitch wider than shape[-1] * dtype_size MUST be
                         # explicit.
float32[] quant_scales   # Empty when quant_axis == -2; len 1 when == -1;
                         # len == shape[quant_axis] when >= 0
int32[] quant_zero_points  # Empty = symmetric; else same length as quant_scales
string format            # Format descriptor ("NV12", "rgb8", "h264"); "" = not
                         # an image. Non-empty => the colorimetry fields apply.
string color_space       # Color primaries: "bt709", "bt2020", "srgb", ""
string color_transfer    # Transfer function: "bt709", "srgb", "pq", "hlg", "linear", ""
string color_encoding    # YCbCr matrix: "bt601", "bt709", "bt2020", "" (N/A for RGB)
string color_range       # "full", "limited", ""
TensorPlane[] planes     # Typical len >= 1; len = 0 permitted for metadata-only
