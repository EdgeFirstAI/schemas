// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Au-Zone Technologies. All Rights Reserved.
//
// Shared fixtures and naming conventions for the C++ codec benchmark suite.
// Every benchmark name across every backend follows the form
//
//     <Category>/<op>/<variant>
//
// where <op> is itself a slash-delimited path (e.g. "encode/new",
// "decode/decode", "access/one_field", "workflow/pub_loop_inplace") drawn
// from the bench::op namespace constants below. The full name therefore has
// four slash-separated segments at runtime — the renderer parses the leading
// Category and trailing variant and treats everything in between as the op
// path. Always compose names via bench_name() rather than building strings by
// hand so the convention stays consistent across backends.

#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace bench {

// Op constants. Use these (not string literals) to construct benchmark names.
namespace op {
    inline constexpr std::string_view encode_new          = "encode/new";
    inline constexpr std::string_view decode_decode       = "decode/decode";
    inline constexpr std::string_view access_one_field    = "access/one_field";
    inline constexpr std::string_view access_half_fields  = "access/half_fields";
    inline constexpr std::string_view access_all_fields   = "access/all_fields";
    inline constexpr std::string_view access_payload_iter = "access/payload_iter";
    inline constexpr std::string_view workflow_smp        = "workflow/sub_modify_pub";
    inline constexpr std::string_view workflow_pub_rebuild = "workflow/pub_loop_rebuild";
    inline constexpr std::string_view workflow_pub_inplace = "workflow/pub_loop_inplace";
    inline constexpr std::string_view workflow_sub        = "workflow/sub_loop";
}

// Compose canonical benchmark name "<Category>/<op>/<variant>" — note that
// <op> is itself a slash-delimited path (see bench::op constants), so the
// resulting name typically has four slash segments at runtime.
inline std::string bench_name(std::string_view category,
                              std::string_view op,
                              std::string_view variant) {
    std::string s;
    s.reserve(category.size() + op.size() + variant.size() + 2);
    s.append(category).push_back('/');
    s.append(op).push_back('/');
    s.append(variant);
    return s;
}

// Pseudo-random fixed-seed payload so all backends use identical bytes.
inline std::vector<std::uint8_t> make_payload(std::size_t bytes, std::uint32_t seed = 0xE49F) {
    std::vector<std::uint8_t> out(bytes);
    std::uint32_t s = seed;
    for (auto& b : out) {
        s = s * 1664525u + 1013904223u;  // LCG; deterministic across backends
        b = static_cast<std::uint8_t>(s >> 24);
    }
    return out;
}

namespace fixtures {

struct HeaderFixture {
    std::int32_t  stamp_sec   = 1234567890;
    std::uint32_t stamp_nanos = 123456789;
    std::string   frame_id    = "camera_frame";
    static constexpr std::string_view variant = "default";
};

struct TimeFixture {
    std::int32_t  sec   = 1234567890;
    std::uint32_t nanos = 123456789;
    static constexpr std::string_view variant = "default";
};

struct Vector3Fixture {
    double x = 1.5;
    double y = 2.5;
    double z = 3.5;
    static constexpr std::string_view variant = "default";
};

struct PoseFixture {
    // position
    double px = 10.0, py = 20.0, pz = 30.0;
    // orientation (identity quaternion)
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    static constexpr std::string_view variant = "default";
};

struct DmaBufferFixture {
    // header
    std::int32_t  stamp_sec   = 1234567890;
    std::uint32_t stamp_nanos = 123456789;
    std::string   frame_id    = "camera_dma";
    // body
    std::uint32_t pid    = 12345;
    std::int32_t  fd     = 7;
    std::uint32_t width  = 1280;
    std::uint32_t height = 720;
    std::uint32_t stride = 1280 * 4;   // RGBA
    std::uint32_t fourcc = 0x32424752; // 'RGB2'
    std::uint32_t length = 1280 * 720 * 4;
    static constexpr std::string_view variant = "default";
};

struct ImageVariant {
    std::string_view name;       // e.g. "HD_rgb8"
    std::uint32_t    width;
    std::uint32_t    height;
    std::string_view encoding;
    // step_bpp is the row-stride byte count (Y-plane only for planar formats);
    // it goes into Image::step. payload_bytes_num/payload_bytes_den together
    // express the total wire-data size as width*height * num/den, which lets
    // us model NV12 (Y full-res + half-res interleaved UV = 3/2 bpp total)
    // without losing precision to integer rounding.
    std::uint32_t    step_bpp;
    std::uint32_t    payload_bytes_num;
    std::uint32_t    payload_bytes_den;
};

// Compute total wire payload for an Image variant.
constexpr std::size_t image_payload_bytes(const ImageVariant& v) {
    return static_cast<std::size_t>(v.width) * v.height
         * v.payload_bytes_num / v.payload_bytes_den;
}

inline constexpr ImageVariant kImageVariants[] = {
    // RGB888: 3 bytes per pixel (no chroma subsampling). step=3, payload=3.
    {"VGA_rgb8",  640,  480, "rgb8", 3, 3, 1},
    // YUYV (4:2:2): 2 bytes per pixel interleaved Y/U/Y/V. step=2, payload=2.
    {"VGA_yuyv",  640,  480, "yuyv", 2, 2, 1},
    // NV12 (4:2:0): Y plane + interleaved UV at half resolution.
    // step=1 (row stride is Y bytes only), total payload=width*height*3/2.
    {"VGA_nv12",  640,  480, "nv12", 1, 3, 2},
    {"HD_rgb8",  1280,  720, "rgb8", 3, 3, 1},
    {"HD_yuyv",  1280,  720, "yuyv", 2, 2, 1},
    {"HD_nv12",  1280,  720, "nv12", 1, 3, 2},
    {"FHD_rgb8", 1920, 1080, "rgb8", 3, 3, 1},
    {"FHD_yuyv", 1920, 1080, "yuyv", 2, 2, 1},
    {"FHD_nv12", 1920, 1080, "nv12", 1, 3, 2},
};

// ---------------------------------------------------------------------------
// RadarCube variants — measured production shapes from radarpub.
//
// Layout order: [SEQUENCE=chirp_types, RANGE=range_gates, RXCHANNEL=rx_channels,
//                DOPPLER=doppler_bins] where the last dim is i16 PAIRS (I+Q),
// matching format_cube() in radarpub/src/radarpub.rs lines 629-635 which
// doubles shape[3] to account for complex data before publishing.
//
// DRVEGRD-169 uses 12 virtual RX channels (3TX × 4RX MIMO).
// Source: radarpub/ARCHITECTURE.md lines 447-450 (long/typical shape);
//         Short/UltraShort range axes halved from the architecture doc estimate.
//
//   long:        [4, 256, 12, 128]  → 786,432 i16  → 1,572,864 bytes (~1.5 MB)
//   short:       [2, 128, 12, 128]  → 393,216 i16  →   786,432 bytes (~768 KB)
//   ultra_short: [1,  64, 12, 128]  →  98,304 i16  →   196,608 bytes (~192 KB)
// ---------------------------------------------------------------------------
struct RadarCubeVariant {
    std::string_view name;
    std::uint16_t    shape[4];       // {chirp_types, range_gates, rx_channels, doppler_bins×2}
    std::size_t      cube_elements;  // total i16 count = shape[0]*shape[1]*shape[2]*shape[3]
};

inline constexpr RadarCubeVariant kRadarCubeVariants[] = {
    // Cube shape ordering is [chirp_types, range_gates, rx_channels, doppler_bins×2 IQ]
    // per radarpub's wire format (radarpub/src/eth.rs:662). The final dimension is
    // doubled because I and Q components are interleaved.
    //
    // DRVEGRD-169 (3TX × 4RX = 12 virtual channels per hardware ref):
    {"DRVEGRD169_ultra_short", {1,  64, 12, 128},   1u* 64u*12u*128u},
    {"DRVEGRD169_short",       {2, 128, 12, 128},   2u*128u*12u*128u},
    {"DRVEGRD169_long",        {4, 256, 12, 128},   4u*256u*12u*128u},
    // DRVEGRD-171 (6TX × 8RX = 48 virtual channels; only Extra-Long uses 256 doppler):
    {"DRVEGRD171_short",       {2, 128, 48, 128},   2u*128u*48u*128u},
    {"DRVEGRD171_extra_long",  {4, 256, 48, 128},   4u*256u*48u*128u},
};

// ---------------------------------------------------------------------------
// Mask variants: uint8 class-index per pixel (wire format confirmed at
// model/src/buildmsgs.rs lines 181-188: argmax applied by HAL decoder,
// one u8 per pixel regardless of class count).
//
// Resolutions match actual model service outputs:
//   160×160 — proto-resolution YOLO / minimum ModelPack VGA output
//             (model/src/model.rs line 402: H/W ≥ 160 threshold)
//   320×320 — ModelPack semantic output for HD camera input
//   480×480 — ModelPack semantic or segdet output for FHD camera input
//   640×640 — post-decode instance mask at VGA-like model input resolution
//   1280×720 — retina-scaled mask at HD camera native resolution
//   1920×1080 — retina-scaled mask at FHD camera native resolution
// ---------------------------------------------------------------------------
struct MaskVariant {
    std::string_view name;
    std::uint32_t    width;
    std::uint32_t    height;
    std::uint32_t    num_classes;  // informational only; payload is always w×h bytes
    // payload_size = width * height  (1 u8 per pixel)
};

inline constexpr MaskVariant kMaskVariants[] = {
    // Proto-resolution YOLO prototype output / minimum ModelPack VGA semantic mask
    {"160x160_proto",    160,  160, 1},
    // ModelPack semantic HD output — default production shape
    {"320x320_8class",   320,  320, 8},
    // ModelPack semantic / segdet FHD output
    {"480x480_9class",   480,  480, 9},
    // Post-decode instance mask at VGA-like model input (640-input YOLO);
    // name 640x640_8class kept for workflow benchmark compatibility
    {"640x640_8class",   640,  640, 1},
    // Retina-scaled semantic mask at HD camera native resolution
    {"1280x720_hd",     1280,  720, 8},
    // Retina-scaled semantic mask at FHD camera native resolution
    {"1920x1080_fhd",   1920, 1080, 8},
};

// ---------------------------------------------------------------------------
// CompressedVideo variants: literal payload sizes
// ---------------------------------------------------------------------------
struct CompressedVideoVariant {
    std::string_view name;
    std::size_t      payload_bytes;
};

inline constexpr CompressedVideoVariant kCompressedVideoVariants[] = {
    {"10KB",   10u*1024u},
    {"100KB", 100u*1024u},
    {"500KB", 500u*1024u},
    {"1MB",  1024u*1024u},
};

// ---------------------------------------------------------------------------
// PointCloud2 variants — measured sensor outputs from lidarpub services.
//
// point_step=13: x(f32)+y(f32)+z(f32)+reflect(u8) — lidarpub formats.rs:67-94
// point_step=16: x(f32)+y(f32)+z(f32)+fusion_class(u8)+vision_class(u8)+
//                instance_id(u16) — fusion/src/pcd.rs:285-314
//
// Sources:
//   robosense_e1r:           lidarpub/src/robosense.rs POINTS_PER_FRAME=30_000,
//                            ~26,000 actual points at 10 Hz
//   ouster_1024x10_128beam:  lidarpub.default:135 (default mode), ouster.rs:847
//                            128×1024 = 131,072 max; ~128,000 after crop
//   ouster_2048x10_128beam:  max Ouster mode, 128×2048 = 262,144 pts
//   fusion_classes_ouster:   rt/fusion/classes output, 131,072 pts × 16 bpp
//                            (fusion/src/pcd.rs:139, serialize_classes())
// ---------------------------------------------------------------------------
struct PointCloud2Variant {
    std::string_view name;
    std::uint32_t    num_points;
    std::uint32_t    point_step;  // bytes per point
};

inline constexpr PointCloud2Variant kPointCloud2Variants[] = {
    // Robosense E1R — smallest production lidar, single-return 13 bpp
    {"robosense_e1r",          26000u,   13u},
    // Ouster default (1024×10, 128-beam) — dominant production shape, 13 bpp
    {"ouster_1024x10_128beam",131072u,   13u},
    // Ouster max (2048×10, 128-beam) — largest unorganized cloud, 13 bpp
    {"ouster_2048x10_128beam",262144u,   13u},
    // Fusion classes output — post-fusion annotated cloud, 16 bpp
    {"fusion_classes_ouster",  131072u,  16u},
};

}  // namespace fixtures
}  // namespace bench
