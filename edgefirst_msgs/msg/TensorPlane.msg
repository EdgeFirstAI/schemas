# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

# TensorPlane — descriptor for a single plane of a Tensor.
#
# Two exclusive delivery modes:
#   handle >= 0 : plane bytes live behind the handle; consumer uses
#                 offset/size/used to map. `data` MUST be empty.
#   handle == -1: plane bytes are inlined in `data`; offset is ignored,
#                 size describes `data` length, and modifier MUST be 0.
#
# All planes of one Tensor share a single storage_kind, pid and fence_fd --
# multi-plane is not multi-storage. A field belongs here only if it describes
# this plane's bytes; one that describes the act which produced them -- the
# fence -- belongs on the Tensor. Only the interpretation of the bytes
# (modifier) varies per plane.
#
# The six fixed-size scalars below are contiguous and 8-byte aligned so an
# element carries no interior padding and every scalar is O(1) from the
# element start. Do not insert a variable-length field among them.

int64 handle             # DMA-BUF / shm fd, IOSurface id, or -1 when inline.
                         # 64-bit so it can carry any integer handle the HAL
                         # defines, not just an fd.
uint64 offset            # Byte offset of plane within the handle (ignored when inline)
uint64 stride            # Bytes per line of this plane
uint64 size              # Plane capacity (buffer span)
uint64 used              # Valid payload bytes (used <= size)
uint64 modifier          # DRM format modifier for this plane. 0 = DRM_FORMAT_MOD_LINEAR.
                         # AFBC on Mali, UBWC on Adreno. A consumer that reads a
                         # compressed buffer as linear renders garbage, so a non-zero
                         # value it does not recognise MUST be refused, not ignored.
                         # MUST be 0 on an inline plane.
uint8[] handle_bytes     # Opaque handle for backends whose handle is not an
                         # integer (e.g. cudaIpcMemHandle_t is 64 bytes). Empty
                         # for every storage_kind defined today. Present now
                         # because it cannot be added later: a TensorPlane is a
                         # CDR sequence element, so any change shifts every
                         # subsequent element.
uint8[] data             # Inlined plane bytes; non-empty only when handle == -1
