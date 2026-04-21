# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

# CameraFrame — multi-plane video frame reference.
#
# Carries references (DMA-BUF file descriptors) and/or inlined bytes to one or
# more image planes, plus frame-level metadata. Supersedes the single-plane
# DmaBuffer message (deprecated in 3.1.0, removed in 4.0.0).
#
# Scope: DMA-BUF-backed raw video frames, DMA-BUF-backed compressed bitstreams
# (NVENC/NVDEC output, MJPEG capture, etc.), and off-device bridges using
# inlined per-plane data[]. Host-memory compressed streams (IP camera RTSP)
# continue to use CompressedImage / CompressedVideo.

std_msgs/Header header       # Timestamp and coordinate frame_id
uint64 seq                   # Monotonic frame index for drop detection (V4L2 sequence / libcamera Request.sequence)
uint32 pid                   # Producer pid for pidfd_open; 0 when all planes use inlined data[]
uint32 width                 # Image width in pixels
uint32 height                # Image height in pixels
string format                # Format descriptor (e.g., "NV12", "rgb8", "rgb8_planar_nchw", "h264")
string color_space           # Color primaries: "bt709", "bt2020", "srgb", "smpte170m", "" = unknown
string color_transfer        # Transfer function: "bt709", "srgb", "pq", "hlg", "linear", ""
string color_encoding        # YCbCr matrix: "bt601", "bt709", "bt2020", "" (N/A for RGB)
string color_range           # "full", "limited", ""
int32 fence_fd               # DMA-fence sync_file fd; -1 = already signalled / no fence
CameraPlane[] planes         # Typical len>=1; len=0 permitted for metadata-only frames
