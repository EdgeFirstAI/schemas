# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

# CameraFrame — a camera frame as a stamped Tensor.
#
# Byte-identical in layout to TensorStamped. It exists as a separate type
# because the schema name is the contract on the topic: a subscriber or
# visualizer dispatches on edgefirst_msgs/msg/CameraFrame to know the payload
# is an image, without inspecting whether Tensor.format is non-empty.
# Reinterpreting one as the other is therefore free.
#
# Changed in 4.0.0: previously carried width/height/format/colorimetry/planes
# directly. Those now live in the embedded Tensor. Image dimensions come from
# Tensor.shape; the per-format convention is pinned in the HAL format table.
# Supersedes DmaBuffer (removed in 4.0.0) and the former CameraPlane.

std_msgs/Header header   # Timestamp and coordinate frame_id
uint64 seq               # Monotonic frame index for drop detection
                         # (V4L2 sequence / libcamera Request.sequence)
Tensor tensor            # The payload; MUST remain the last field
