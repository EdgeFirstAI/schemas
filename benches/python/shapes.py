# SPDX-License-Identifier: Apache-2.0
# Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

"""Canonical message shapes for the Python CDR bench harness.

Mirror of `benches/cpp/common.hpp` so the Python bench fixtures match the
C++ bench fixtures one-for-one. Update both files together — the renderer
and the production sensor-output references in `benches/cpp/common.hpp`
are the source of truth; this module is the Python projection.

Each tuple's first element is the variant name used in the pytest test ID.
"""

from typing import NamedTuple


class ImageVariant(NamedTuple):
    name: str
    width: int
    height: int
    encoding: str
    # step_bpp: row-stride byte count (Y-plane only for planar formats)
    # payload_num/payload_den: total wire size = width*height * num/den
    # (NV12 = 3/2 bpp; RGB8 = 3/1; YUYV = 2/1).
    step_bpp: int
    payload_num: int
    payload_den: int

    @property
    def step(self) -> int:
        return self.width * self.step_bpp

    @property
    def payload_bytes(self) -> int:
        return self.width * self.height * self.payload_num // self.payload_den


IMAGE_VARIANTS = [
    ImageVariant("VGA_rgb8",  640,  480, "rgb8", 3, 3, 1),
    ImageVariant("VGA_yuyv",  640,  480, "yuyv", 2, 2, 1),
    ImageVariant("VGA_nv12",  640,  480, "nv12", 1, 3, 2),
    ImageVariant("HD_rgb8",  1280,  720, "rgb8", 3, 3, 1),
    ImageVariant("HD_yuyv",  1280,  720, "yuyv", 2, 2, 1),
    ImageVariant("HD_nv12",  1280,  720, "nv12", 1, 3, 2),
    ImageVariant("FHD_rgb8", 1920, 1080, "rgb8", 3, 3, 1),
    ImageVariant("FHD_yuyv", 1920, 1080, "yuyv", 2, 2, 1),
    ImageVariant("FHD_nv12", 1920, 1080, "nv12", 1, 3, 2),
]


class RadarCubeVariant(NamedTuple):
    name: str
    # [chirp_types, range_gates, rx_channels, doppler_bins×2_IQ]
    shape: tuple

    @property
    def cube_elements(self) -> int:
        return self.shape[0] * self.shape[1] * self.shape[2] * self.shape[3]


# DRVEGRD-169: 12 virtual RX channels (3TX × 4RX MIMO).
# DRVEGRD-171: 48 virtual RX channels (6TX × 8RX); only Extra-Long uses 256 doppler.
RADAR_CUBE_VARIANTS = [
    RadarCubeVariant("DRVEGRD169_ultra_short", (1,  64, 12, 128)),
    RadarCubeVariant("DRVEGRD169_short",       (2, 128, 12, 128)),
    RadarCubeVariant("DRVEGRD169_long",        (4, 256, 12, 128)),
    RadarCubeVariant("DRVEGRD171_short",       (2, 128, 48, 128)),
    RadarCubeVariant("DRVEGRD171_extra_long",  (4, 256, 48, 128)),
]


class MaskVariant(NamedTuple):
    name: str
    width: int
    height: int
    # num_classes is informational only; payload is always w×h bytes
    # (HAL decoder argmax-applies, one u8 per pixel regardless of class count).
    num_classes: int

    @property
    def payload_bytes(self) -> int:
        return self.width * self.height  # length=1 channel


MASK_VARIANTS = [
    MaskVariant("160x160_proto",   160,  160, 1),
    MaskVariant("320x320_8class",  320,  320, 8),
    MaskVariant("480x480_9class",  480,  480, 9),
    MaskVariant("640x640_8class",  640,  640, 1),
    MaskVariant("1280x720_hd",    1280,  720, 8),
    MaskVariant("1920x1080_fhd", 1920, 1080, 8),
]


class PointCloud2Variant(NamedTuple):
    name: str
    num_points: int
    point_step: int  # 13 = x/y/z f32 + reflect u8; 16 = + fusion_class/vision_class/instance_id

    @property
    def payload_bytes(self) -> int:
        return self.num_points * self.point_step

    def fields(self):
        """Return list of (name, offset, datatype, count) tuples for this variant.

        datatype matches sensor_msgs/PointField constants:
            2 = UINT8, 4 = UINT16, 7 = FLOAT32.
        """
        if self.point_step == 13:
            return [
                ("x",       0, 7, 1),
                ("y",       4, 7, 1),
                ("z",       8, 7, 1),
                ("reflect", 12, 2, 1),
            ]
        elif self.point_step == 16:
            return [
                ("x",            0, 7, 1),
                ("y",            4, 7, 1),
                ("z",            8, 7, 1),
                ("fusion_class", 12, 2, 1),
                ("vision_class", 13, 2, 1),
                ("instance_id",  14, 4, 1),
            ]
        else:
            raise ValueError(f"unknown point_step {self.point_step}")


POINT_CLOUD_VARIANTS = [
    PointCloud2Variant("robosense_e1r",          26000, 13),
    PointCloud2Variant("ouster_1024x10_128beam", 131072, 13),
    PointCloud2Variant("ouster_2048x10_128beam", 262144, 13),
    PointCloud2Variant("fusion_classes_ouster",  131072, 16),
]


class CompressedVideoVariant(NamedTuple):
    name: str
    payload_bytes: int


COMPRESSED_VIDEO_VARIANTS = [
    CompressedVideoVariant("10KB",   10 * 1024),
    CompressedVideoVariant("100KB", 100 * 1024),
    CompressedVideoVariant("500KB", 500 * 1024),
    CompressedVideoVariant("1MB",  1024 * 1024),
]


class DmaBufferVariant(NamedTuple):
    name: str
    width: int
    height: int
    stride: int
    fourcc: int
    length: int


# C++ uses a single "default" 1280×720 RGBA fixture (fourcc='RGB2' = 0x32424752,
# stride = 1280×4, length = 1280×720×4).
DMA_BUFFER_VARIANTS = [
    DmaBufferVariant("default", 1280, 720, 1280 * 4, 0x32424752, 1280 * 720 * 4),
]
