# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

# TensorStamped — a Tensor with a timestamp and frame of reference.
#
# The standalone tensor IPC message. Byte-identical in layout to CameraFrame;
# the two are distinct types because in ROS the schema name is the contract on
# the topic.
#
# `seq` is load-bearing beyond drop detection: as a uint64 it aligns to 8 and
# occupies 8 bytes, which leaves `tensor` starting at an offset congruent to
# 0 mod 8 regardless of frame_id length. That is what makes the nested Tensor
# byte layout position-independent. Do not remove it.

std_msgs/Header header   # Timestamp and coordinate frame_id
uint64 seq               # Monotonic sample index for drop detection
Tensor tensor            # The payload; MUST remain the last field
