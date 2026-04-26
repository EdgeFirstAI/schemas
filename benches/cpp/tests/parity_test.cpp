// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Au-Zone Technologies. All Rights Reserved.

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include <edgefirst/schemas.hpp>
#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>
#include <fastcdr/CdrSizeCalculator.hpp>

// Fast-CDR generated types
#include "std_msgs/Header.hpp"
#include "builtin_interfaces/TimeCdrAux.ipp"
#include "std_msgs/HeaderCdrAux.ipp"

#include "geometry_msgs/Vector3.hpp"
#include "geometry_msgs/Vector3CdrAux.ipp"
#include "geometry_msgs/Point.hpp"
#include "geometry_msgs/PointCdrAux.ipp"
#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/QuaternionCdrAux.ipp"
#include "geometry_msgs/Pose.hpp"
#include "geometry_msgs/PoseCdrAux.ipp"
#include "edgefirst_msgs/DmaBuffer.hpp"
#include "edgefirst_msgs/DmaBufferCdrAux.ipp"

// Large message types (Fast-CDR)
#include "sensor_msgs/Image.hpp"
#include "sensor_msgs/ImageCdrAux.ipp"
#include "sensor_msgs/PointField.hpp"
#include "sensor_msgs/PointFieldCdrAux.ipp"
#include "sensor_msgs/PointCloud2.hpp"
#include "sensor_msgs/PointCloud2CdrAux.ipp"
#include "edgefirst_msgs/Mask.hpp"
#include "edgefirst_msgs/MaskCdrAux.ipp"
#include "edgefirst_msgs/RadarCube.hpp"
#include "edgefirst_msgs/RadarCubeCdrAux.ipp"
#include "foxglove_msgs/CompressedVideo.hpp"
#include "foxglove_msgs/CompressedVideoCdrAux.ipp"

#include "common.hpp"

namespace ef = edgefirst::schemas;

// Declarations for Cyclone DDS parity encoders (defined in parity_cyclonedds.cpp,
// compiled with only the cyclonedds type headers in its include path so they don't
// collide with the Fast-CDR types included in this translation unit).
namespace parity_cdds {

std::vector<std::uint8_t> encode_header(const bench::fixtures::HeaderFixture& f);
std::vector<std::uint8_t> encode_time(const bench::fixtures::TimeFixture& f);
std::vector<std::uint8_t> encode_vector3(const bench::fixtures::Vector3Fixture& f);
std::vector<std::uint8_t> encode_pose(const bench::fixtures::PoseFixture& f);
std::vector<std::uint8_t> encode_dmabuffer(const bench::fixtures::DmaBufferFixture& f);
std::vector<std::uint8_t> encode_image(const bench::fixtures::ImageVariant& v,
                                        const std::vector<std::uint8_t>& payload);
std::vector<std::uint8_t> encode_radarcube(const bench::fixtures::RadarCubeVariant& v,
                                            const std::vector<std::int16_t>& cube);
std::vector<std::uint8_t> encode_mask(const bench::fixtures::MaskVariant& v,
                                       const std::vector<std::uint8_t>& payload);
std::vector<std::uint8_t> encode_compressedvideo(const std::vector<std::uint8_t>& payload);
std::vector<std::uint8_t> encode_pointcloud2(const bench::fixtures::PointCloud2Variant& v,
                                              const std::vector<std::uint8_t>& payload);

} // namespace parity_cdds

static std::vector<std::uint8_t> encode_with_edgefirst(const bench::fixtures::HeaderFixture& f) {
    // Copy the proven pattern from bench_edgefirst.cpp's header_wire_bytes().
    auto b = ef::HeaderBuilder::create();
    if (!b) { std::fprintf(stderr, "edgefirst HeaderBuilder::create failed\n"); std::abort(); }
    b->stamp(ef::Time{f.stamp_sec, f.stamp_nanos});
    auto fi = b->frame_id(f.frame_id.c_str());
    (void)fi;
    auto built = b->build();
    if (!built) { std::fprintf(stderr, "edgefirst HeaderBuilder::build failed\n"); std::abort(); }
    auto& r = *built;
    std::vector<std::uint8_t> out(r.data, r.data + r.size);
    ros_bytes_free(r.data, r.size);
    return out;
}

static std::vector<std::uint8_t> encode_with_fastcdr(const bench::fixtures::HeaderFixture& f) {
    // Copy the proven pattern from bench_fastcdr.cpp's encode_new.
    std_msgs::msg::Header msg;
    msg.stamp().sec(f.stamp_sec);
    msg.stamp().nanosec(f.stamp_nanos);
    msg.frame_id(f.frame_id);

    eprosima::fastcdr::CdrSizeCalculator calc(eprosima::fastcdr::CdrVersion::XCDRv1);
    size_t current_alignment = 0;
    size_t size = eprosima::fastcdr::calculate_serialized_size(calc, msg, current_alignment);

    std::vector<char> buf(size + 4);
    eprosima::fastcdr::FastBuffer fb(buf.data(), buf.size());
    eprosima::fastcdr::Cdr cdr(fb,
        eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
        eprosima::fastcdr::CdrVersion::XCDRv1);
    cdr.serialize_encapsulation();
    cdr << msg;

    auto bytes_written = cdr.get_serialized_data_length();
    auto* p = reinterpret_cast<std::uint8_t*>(buf.data());
    return std::vector<std::uint8_t>(p, p + bytes_written);
}

// ---------------------------------------------------------------------------
// Helpers shared by the new encode functions
// ---------------------------------------------------------------------------

template <typename T>
static std::vector<std::uint8_t> fastcdr_encode(const T& msg) {
    eprosima::fastcdr::CdrSizeCalculator calc(eprosima::fastcdr::CdrVersion::XCDRv1);
    size_t current_alignment = 0;
    size_t size = eprosima::fastcdr::calculate_serialized_size(calc, msg, current_alignment);
    std::vector<char> buf(size + 4);
    eprosima::fastcdr::FastBuffer fb(buf.data(), buf.size());
    eprosima::fastcdr::Cdr cdr(fb,
        eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
        eprosima::fastcdr::CdrVersion::XCDRv1);
    cdr.serialize_encapsulation();
    cdr << msg;
    auto bytes_written = cdr.get_serialized_data_length();
    auto* p = reinterpret_cast<std::uint8_t*>(buf.data());
    return std::vector<std::uint8_t>(p, p + bytes_written);
}

// ---------------------------------------------------------------------------
// Time parity
// ---------------------------------------------------------------------------

static std::vector<std::uint8_t> encode_time_with_edgefirst(const bench::fixtures::TimeFixture& f) {
    ef::Time t{f.sec, f.nanos};
    auto sz = t.encoded_size();
    std::vector<std::uint8_t> buf(*sz);
    (void)t.encode({buf.data(), buf.size()});
    return buf;
}

static std::vector<std::uint8_t> encode_time_with_fastcdr(const bench::fixtures::TimeFixture& f) {
    builtin_interfaces::msg::Time msg;
    msg.sec(f.sec);
    msg.nanosec(f.nanos);
    return fastcdr_encode(msg);
}

// ---------------------------------------------------------------------------
// Vector3 parity
// ---------------------------------------------------------------------------

static std::vector<std::uint8_t> encode_vector3_with_edgefirst(const bench::fixtures::Vector3Fixture& f) {
    ef::Vector3 v{f.x, f.y, f.z};
    auto sz = v.encoded_size();
    std::vector<std::uint8_t> buf(*sz);
    (void)v.encode({buf.data(), buf.size()});
    return buf;
}

static std::vector<std::uint8_t> encode_vector3_with_fastcdr(const bench::fixtures::Vector3Fixture& f) {
    geometry_msgs::msg::Vector3 msg;
    msg.x(f.x); msg.y(f.y); msg.z(f.z);
    return fastcdr_encode(msg);
}

// ---------------------------------------------------------------------------
// Pose parity
// ---------------------------------------------------------------------------

static std::vector<std::uint8_t> encode_pose_with_edgefirst(const bench::fixtures::PoseFixture& f) {
    ef::Pose p{f.px, f.py, f.pz, f.qx, f.qy, f.qz, f.qw};
    auto sz = p.encoded_size();
    std::vector<std::uint8_t> buf(*sz);
    (void)p.encode({buf.data(), buf.size()});
    return buf;
}

static std::vector<std::uint8_t> encode_pose_with_fastcdr(const bench::fixtures::PoseFixture& f) {
    geometry_msgs::msg::Pose msg;
    msg.position().x(f.px); msg.position().y(f.py); msg.position().z(f.pz);
    msg.orientation().x(f.qx); msg.orientation().y(f.qy);
    msg.orientation().z(f.qz); msg.orientation().w(f.qw);
    return fastcdr_encode(msg);
}

// ---------------------------------------------------------------------------
// DmaBuffer parity
// ---------------------------------------------------------------------------

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

static std::vector<std::uint8_t> encode_dmabuffer_with_edgefirst(const bench::fixtures::DmaBufferFixture& f) {
    auto r = ef::DmaBuffer::encode(
        ef::Time{f.stamp_sec, f.stamp_nanos},
        f.frame_id,
        f.pid, f.fd,
        f.width, f.height,
        f.stride, f.fourcc, f.length);
    if (!r) { std::fprintf(stderr, "edgefirst DmaBuffer::encode failed\n"); std::abort(); }
    auto sp = r->as_cdr();
    return std::vector<std::uint8_t>(sp.data(), sp.data() + sp.size());
}

#pragma GCC diagnostic pop

static std::vector<std::uint8_t> encode_dmabuffer_with_fastcdr(const bench::fixtures::DmaBufferFixture& f) {
    edgefirst_msgs::msg::DmaBuffer msg;
    msg.header().stamp().sec(f.stamp_sec);
    msg.header().stamp().nanosec(f.stamp_nanos);
    msg.header().frame_id(f.frame_id);
    msg.pid(f.pid); msg.fd(f.fd);
    msg.width(f.width); msg.height(f.height);
    msg.stride(f.stride); msg.fourcc(f.fourcc); msg.length(f.length);
    return fastcdr_encode(msg);
}

// ---------------------------------------------------------------------------
// Utilities
// ---------------------------------------------------------------------------

static void hex_dump(const char* label, const std::vector<std::uint8_t>& v) {
    std::fprintf(stderr, "%s (%zu bytes):", label, v.size());
    for (auto b : v) std::fprintf(stderr, " %02x", b);
    std::fprintf(stderr, "\n");
}

static bool check_parity(const char* name,
                          const std::vector<std::uint8_t>& a,
                          const std::vector<std::uint8_t>& b,
                          const char* label_a = "edgefirst",
                          const char* label_b = "fastcdr  ") {
    if (a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size()) == 0) {
        std::fprintf(stdout, "OK: %s parity/%s vs %s (%zu bytes)\n",
                     name, label_a, label_b, a.size());
        return true;
    }
    hex_dump((std::string(label_a) + "/" + name).c_str(), a);
    hex_dump((std::string(label_b) + "/" + name).c_str(), b);
    std::fprintf(stderr, "FAIL: %s CDR byte mismatch (%s vs %s)\n",
                 name, label_a, label_b);
    return false;
}

// ---------------------------------------------------------------------------
// Image parity  (representative: HD_rgb8)
// ---------------------------------------------------------------------------

static std::vector<std::uint8_t> encode_image_with_edgefirst(
        const bench::fixtures::ImageVariant& v,
        const std::vector<std::uint8_t>& payload) {
    std::uint32_t step = v.width * v.step_bpp;
    auto r = ef::Image::encode(
        ef::Time{1234567890, 123456789},
        "cam",
        v.height, v.width,
        std::string(v.encoding),
        false,
        step,
        {payload.data(), payload.size()});
    if (!r) { std::fprintf(stderr, "edgefirst Image::encode failed\n"); std::abort(); }
    auto sp = r->as_cdr();
    return std::vector<std::uint8_t>(sp.data(), sp.data() + sp.size());
}

static std::vector<std::uint8_t> encode_image_with_fastcdr(
        const bench::fixtures::ImageVariant& v,
        const std::vector<std::uint8_t>& payload) {
    sensor_msgs::msg::Image msg;
    msg.header().stamp().sec(1234567890);
    msg.header().stamp().nanosec(123456789);
    msg.header().frame_id("cam");
    msg.height(v.height);
    msg.width(v.width);
    msg.encoding(std::string(v.encoding));
    msg.is_bigendian(0);
    msg.step(v.width * v.step_bpp);
    msg.data(payload);
    return fastcdr_encode(msg);
}

// ---------------------------------------------------------------------------
// RadarCube parity  (representative: DRVEGRD169_short)
// ---------------------------------------------------------------------------

static const bench::fixtures::RadarCubeVariant& find_radar_variant(std::string_view name) {
    for (const auto& v : bench::fixtures::kRadarCubeVariants)
        if (v.name == name) return v;
    std::fprintf(stderr, "RadarCubeVariant '%s' not found\n", std::string(name).c_str());
    std::abort();
}

static std::vector<std::uint8_t> encode_radarcube_with_edgefirst(
        const bench::fixtures::RadarCubeVariant& v,
        const std::vector<std::int16_t>& cube) {
    std::uint16_t shape[4] = {v.shape[0], v.shape[1], v.shape[2], v.shape[3]};
    auto b = ef::RadarCubeBuilder::create();
    if (!b) { std::fprintf(stderr, "RadarCubeBuilder::create failed\n"); std::abort(); }
    b->stamp(ef::Time{1234567890, 0});
    (void)b->frame_id("radar");
    b->timestamp(0);
    (void)b->shape({shape, 4});
    (void)b->cube({cube.data(), cube.size()});
    b->is_complex(false);
    auto r = b->build();
    if (!r) { std::fprintf(stderr, "RadarCubeBuilder::build failed\n"); std::abort(); }
    std::vector<std::uint8_t> out(r->data, r->data + r->size);
    ros_bytes_free(r->data, r->size);
    return out;
}

static std::vector<std::uint8_t> encode_radarcube_with_fastcdr(
        const bench::fixtures::RadarCubeVariant& v,
        const std::vector<std::int16_t>& cube) {
    edgefirst_msgs::msg::RadarCube msg;
    msg.header().stamp().sec(1234567890);
    msg.header().stamp().nanosec(0);
    msg.header().frame_id("radar");
    msg.timestamp(0);
    msg.shape(std::vector<std::uint16_t>{v.shape[0], v.shape[1], v.shape[2], v.shape[3]});
    msg.cube(cube);
    msg.is_complex(false);
    return fastcdr_encode(msg);
}

// ---------------------------------------------------------------------------
// Mask parity  (representative: 320x320_8class)
// ---------------------------------------------------------------------------

static const bench::fixtures::MaskVariant& find_mask_variant(std::string_view name) {
    for (const auto& v : bench::fixtures::kMaskVariants)
        if (v.name == name) return v;
    std::fprintf(stderr, "MaskVariant '%s' not found\n", std::string(name).c_str());
    std::abort();
}

static std::vector<std::uint8_t> encode_mask_with_edgefirst(
        const bench::fixtures::MaskVariant& v,
        const std::vector<std::uint8_t>& payload) {
    auto r = ef::Mask::encode(
        v.height, v.width,
        static_cast<std::uint32_t>(payload.size()),
        "mono8",
        {payload.data(), payload.size()},
        false);
    if (!r) { std::fprintf(stderr, "edgefirst Mask::encode failed\n"); std::abort(); }
    auto sp = r->as_cdr();
    return std::vector<std::uint8_t>(sp.data(), sp.data() + sp.size());
}

static std::vector<std::uint8_t> encode_mask_with_fastcdr(
        const bench::fixtures::MaskVariant& v,
        const std::vector<std::uint8_t>& payload) {
    edgefirst_msgs::msg::Mask msg;
    msg.height(v.height);
    msg.width(v.width);
    msg.length(static_cast<std::uint32_t>(payload.size()));
    msg.encoding("mono8");
    msg.mask(payload);
    msg.boxed(false);
    return fastcdr_encode(msg);
}

// ---------------------------------------------------------------------------
// CompressedVideo parity  (representative: 10KB)
// ---------------------------------------------------------------------------

static const bench::fixtures::CompressedVideoVariant& find_cv_variant(std::string_view name) {
    for (const auto& v : bench::fixtures::kCompressedVideoVariants)
        if (v.name == name) return v;
    std::fprintf(stderr, "CompressedVideoVariant '%s' not found\n", std::string(name).c_str());
    std::abort();
}

static std::vector<std::uint8_t> encode_cv_with_edgefirst(
        const std::vector<std::uint8_t>& payload) {
    auto r = ef::CompressedVideo::encode(
        ef::Time{1234567890, 123456789},
        "cam",
        {payload.data(), payload.size()},
        "h264");
    if (!r) { std::fprintf(stderr, "edgefirst CompressedVideo::encode failed\n"); std::abort(); }
    auto sp = r->as_cdr();
    return std::vector<std::uint8_t>(sp.data(), sp.data() + sp.size());
}

static std::vector<std::uint8_t> encode_cv_with_fastcdr(
        const std::vector<std::uint8_t>& payload) {
    foxglove_msgs::msg::CompressedVideo msg;
    msg.timestamp().sec(1234567890);
    msg.timestamp().nanosec(123456789);
    msg.frame_id("cam");
    msg.data(payload);
    msg.format("h264");
    return fastcdr_encode(msg);
}

// ---------------------------------------------------------------------------
// PointCloud2 parity  (representative: sparse_1K)
// ---------------------------------------------------------------------------

static const bench::fixtures::PointCloud2Variant& find_pc2_variant(std::string_view name) {
    for (const auto& v : bench::fixtures::kPointCloud2Variants)
        if (v.name == name) return v;
    std::fprintf(stderr, "PointCloud2Variant '%s' not found\n", std::string(name).c_str());
    std::abort();
}

static std::vector<std::uint8_t> encode_pc2_with_edgefirst(
        const bench::fixtures::PointCloud2Variant& v,
        const std::vector<std::uint8_t>& payload) {
    // datatype 7 = FLOAT32
    static const ros_point_field_elem_t kFields[] = {
        {"x",         0,  7, 1},
        {"y",         4,  7, 1},
        {"z",         8,  7, 1},
        {"intensity", 12, 7, 1},
    };
    auto b = ef::PointCloud2Builder::create();
    if (!b) { std::fprintf(stderr, "PointCloud2Builder::create failed\n"); std::abort(); }
    b->stamp(ef::Time{1234567890, 123456789});
    (void)b->frame_id("lidar");
    b->height(1);
    b->width(v.num_points);
    (void)b->fields({kFields, 4});
    b->is_bigendian(false);
    b->point_step(v.point_step);
    b->row_step(v.num_points * v.point_step);
    (void)b->data({payload.data(), payload.size()});
    b->is_dense(true);
    auto r = b->build();
    if (!r) { std::fprintf(stderr, "PointCloud2Builder::build failed\n"); std::abort(); }
    std::vector<std::uint8_t> out(r->data, r->data + r->size);
    ros_bytes_free(r->data, r->size);
    return out;
}

static std::vector<std::uint8_t> encode_pc2_with_fastcdr(
        const bench::fixtures::PointCloud2Variant& v,
        const std::vector<std::uint8_t>& payload) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header().stamp().sec(1234567890);
    msg.header().stamp().nanosec(123456789);
    msg.header().frame_id("lidar");
    msg.height(1);
    msg.width(v.num_points);

    auto make_field = [](const char* name, uint32_t offset) {
        sensor_msgs::msg::PointField f;
        f.name(name); f.offset(offset); f.datatype(7); f.count(1);
        return f;
    };
    msg.fields({make_field("x", 0), make_field("y", 4),
                make_field("z", 8), make_field("intensity", 12)});
    msg.is_bigendian(false);
    msg.point_step(v.point_step);
    msg.row_step(v.num_points * v.point_step);
    msg.data(payload);
    msg.is_dense(true);
    return fastcdr_encode(msg);
}

int main() {
    bool ok = true;

    // -----------------------------------------------------------------------
    // edgefirst vs Fast-CDR parity
    // -----------------------------------------------------------------------
    {
        bench::fixtures::HeaderFixture f;
        ok &= check_parity("Header", encode_with_edgefirst(f), encode_with_fastcdr(f));
    }
    {
        bench::fixtures::TimeFixture f;
        ok &= check_parity("Time", encode_time_with_edgefirst(f), encode_time_with_fastcdr(f));
    }
    {
        bench::fixtures::Vector3Fixture f;
        ok &= check_parity("Vector3", encode_vector3_with_edgefirst(f), encode_vector3_with_fastcdr(f));
    }
    {
        bench::fixtures::PoseFixture f;
        ok &= check_parity("Pose", encode_pose_with_edgefirst(f), encode_pose_with_fastcdr(f));
    }
    {
        bench::fixtures::DmaBufferFixture f;
        ok &= check_parity("DmaBuffer", encode_dmabuffer_with_edgefirst(f), encode_dmabuffer_with_fastcdr(f));
    }

    // Large type parity — one representative variant each

    {
        // Image: HD_rgb8
        const char* vname = "HD_rgb8";
        const bench::fixtures::ImageVariant* img_v = nullptr;
        for (const auto& v : bench::fixtures::kImageVariants)
            if (v.name == vname) { img_v = &v; break; }
        if (!img_v) { std::fprintf(stderr, "ImageVariant %s not found\n", vname); return 1; }
        std::size_t ps = bench::fixtures::image_payload_bytes(*img_v);
        auto payload = bench::make_payload(ps);
        ok &= check_parity("Image/HD_rgb8",
            encode_image_with_edgefirst(*img_v, payload),
            encode_image_with_fastcdr(*img_v, payload));
    }
    {
        // RadarCube: DRVEGRD169_short
        const auto& rv = find_radar_variant("DRVEGRD169_short");
        std::vector<std::int16_t> cube(rv.cube_elements, 42);
        ok &= check_parity("RadarCube/DRVEGRD169_short",
            encode_radarcube_with_edgefirst(rv, cube),
            encode_radarcube_with_fastcdr(rv, cube));
    }
    {
        // Mask: 320x320_8class
        const auto& mv = find_mask_variant("320x320_8class");
        std::size_t ps = static_cast<std::size_t>(mv.width) *
                         static_cast<std::size_t>(mv.height);
        auto payload = bench::make_payload(ps);
        ok &= check_parity("Mask/320x320_8class",
            encode_mask_with_edgefirst(mv, payload),
            encode_mask_with_fastcdr(mv, payload));
    }
    {
        // CompressedVideo: 10KB
        const auto& cv = find_cv_variant("10KB");
        auto payload = bench::make_payload(cv.payload_bytes);
        ok &= check_parity("CompressedVideo/10KB",
            encode_cv_with_edgefirst(payload),
            encode_cv_with_fastcdr(payload));
    }
    {
        // PointCloud2: robosense_e1r (smallest production variant)
        const auto& pv = find_pc2_variant("robosense_e1r");
        std::size_t ps = static_cast<std::size_t>(pv.num_points) *
                         static_cast<std::size_t>(pv.point_step);
        auto payload = bench::make_payload(ps);
        ok &= check_parity("PointCloud2/robosense_e1r",
            encode_pc2_with_edgefirst(pv, payload),
            encode_pc2_with_fastcdr(pv, payload));
    }

    // -----------------------------------------------------------------------
    // edgefirst vs Cyclone DDS CDR parity
    // (Cyclone encoders live in parity_cyclonedds.cpp to avoid type-header
    //  collisions between Fast-CDR and Cyclone generated types.)
    // -----------------------------------------------------------------------
    {
        bench::fixtures::HeaderFixture f;
        ok &= check_parity("Header",
            encode_with_edgefirst(f), parity_cdds::encode_header(f),
            "edgefirst", "cyclonedds");
    }
    {
        bench::fixtures::TimeFixture f;
        ok &= check_parity("Time",
            encode_time_with_edgefirst(f), parity_cdds::encode_time(f),
            "edgefirst", "cyclonedds");
    }
    {
        bench::fixtures::Vector3Fixture f;
        ok &= check_parity("Vector3",
            encode_vector3_with_edgefirst(f), parity_cdds::encode_vector3(f),
            "edgefirst", "cyclonedds");
    }
    {
        bench::fixtures::PoseFixture f;
        ok &= check_parity("Pose",
            encode_pose_with_edgefirst(f), parity_cdds::encode_pose(f),
            "edgefirst", "cyclonedds");
    }
    {
        bench::fixtures::DmaBufferFixture f;
        ok &= check_parity("DmaBuffer",
            encode_dmabuffer_with_edgefirst(f), parity_cdds::encode_dmabuffer(f),
            "edgefirst", "cyclonedds");
    }
    {
        const char* vname = "HD_rgb8";
        const bench::fixtures::ImageVariant* img_v = nullptr;
        for (const auto& v : bench::fixtures::kImageVariants)
            if (v.name == vname) { img_v = &v; break; }
        if (!img_v) { std::fprintf(stderr, "ImageVariant %s not found\n", vname); return 1; }
        std::size_t ps = bench::fixtures::image_payload_bytes(*img_v);
        auto payload = bench::make_payload(ps);
        ok &= check_parity("Image/HD_rgb8",
            encode_image_with_edgefirst(*img_v, payload),
            parity_cdds::encode_image(*img_v, payload),
            "edgefirst", "cyclonedds");
    }
    {
        const auto& rv = find_radar_variant("DRVEGRD169_short");
        std::vector<std::int16_t> cube(rv.cube_elements, 42);
        ok &= check_parity("RadarCube/DRVEGRD169_short",
            encode_radarcube_with_edgefirst(rv, cube),
            parity_cdds::encode_radarcube(rv, cube),
            "edgefirst", "cyclonedds");
    }
    {
        const auto& mv = find_mask_variant("320x320_8class");
        std::size_t ps = static_cast<std::size_t>(mv.width) *
                         static_cast<std::size_t>(mv.height);
        auto payload = bench::make_payload(ps);
        ok &= check_parity("Mask/320x320_8class",
            encode_mask_with_edgefirst(mv, payload),
            parity_cdds::encode_mask(mv, payload),
            "edgefirst", "cyclonedds");
    }
    {
        const auto& cv = find_cv_variant("10KB");
        auto payload = bench::make_payload(cv.payload_bytes);
        ok &= check_parity("CompressedVideo/10KB",
            encode_cv_with_edgefirst(payload),
            parity_cdds::encode_compressedvideo(payload),
            "edgefirst", "cyclonedds");
    }
    {
        const auto& pv = find_pc2_variant("robosense_e1r");
        std::size_t ps = static_cast<std::size_t>(pv.num_points) *
                         static_cast<std::size_t>(pv.point_step);
        auto payload = bench::make_payload(ps);
        ok &= check_parity("PointCloud2/robosense_e1r",
            encode_pc2_with_edgefirst(pv, payload),
            parity_cdds::encode_pointcloud2(pv, payload),
            "edgefirst", "cyclonedds");
    }

    return ok ? 0 : 1;
}
