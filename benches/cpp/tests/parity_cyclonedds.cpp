// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Au-Zone Technologies. All Rights Reserved.
//
// Cyclone DDS CDR encoders for parity testing.
//
// This translation unit is compiled with ONLY the cyclonedds generated type
// headers in its include path (types/cyclonedds/), deliberately keeping the
// Fast-CDR generated types (types/fastcdr/idl/) out of scope.  Both sets
// of generated types live in the same C++ namespaces (e.g. std_msgs::msg)
// so they cannot be included in the same TU without collisions.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <org/eclipse/cyclonedds/core/cdr/extended_cdr_v1_ser.hpp>

// Cyclone-generated types
#include "builtin_interfaces/Time.hpp"
#include "std_msgs/Header.hpp"
#include "geometry_msgs/Vector3.hpp"
#include "geometry_msgs/Point.hpp"
#include "geometry_msgs/Quaternion.hpp"
#include "geometry_msgs/Pose.hpp"
#include "edgefirst_msgs/DmaBuffer.hpp"
#include "sensor_msgs/Image.hpp"
#include "sensor_msgs/PointField.hpp"
#include "sensor_msgs/PointCloud2.hpp"
#include "edgefirst_msgs/Mask.hpp"
#include "edgefirst_msgs/RadarCube.hpp"
#include "foxglove_msgs/CompressedVideo.hpp"

// Fixtures are header-only; safe to include here.
#include "common.hpp"

namespace parity_cdds {

using cdds_stream = org::eclipse::cyclonedds::core::cdr::xcdr_v1_stream;
using namespace org::eclipse::cyclonedds::core::cdr;

static constexpr uint8_t kEncap[4] = {0x00, 0x01, 0x00, 0x00};

template <typename T>
static std::vector<std::uint8_t> encode(const T& msg) {
    cdds_stream mstr;
    move(mstr, msg, false);
    size_t data_size = mstr.position();

    std::vector<char> buf(data_size + 4);
    buf[0] = static_cast<char>(kEncap[0]);
    buf[1] = static_cast<char>(kEncap[1]);
    buf[2] = static_cast<char>(kEncap[2]);
    buf[3] = static_cast<char>(kEncap[3]);

    cdds_stream wstr;
    wstr.set_buffer(buf.data() + 4, data_size);
    if (!write(wstr, msg, false)) {
        std::fprintf(stderr, "parity_cdds: write failed\n");
        std::abort();
    }
    buf.resize(wstr.position() + 4);
    return std::vector<std::uint8_t>(
        reinterpret_cast<const std::uint8_t*>(buf.data()),
        reinterpret_cast<const std::uint8_t*>(buf.data()) + buf.size());
}

// ---------------------------------------------------------------------------
// Public encoder functions
// ---------------------------------------------------------------------------

std::vector<std::uint8_t> encode_header(const bench::fixtures::HeaderFixture& f) {
    std_msgs::msg::Header msg;
    msg.stamp().sec(f.stamp_sec);
    msg.stamp().nanosec(f.stamp_nanos);
    msg.frame_id(f.frame_id);
    return encode(msg);
}

std::vector<std::uint8_t> encode_time(const bench::fixtures::TimeFixture& f) {
    builtin_interfaces::msg::Time msg;
    msg.sec(f.sec);
    msg.nanosec(f.nanos);
    return encode(msg);
}

std::vector<std::uint8_t> encode_vector3(const bench::fixtures::Vector3Fixture& f) {
    geometry_msgs::msg::Vector3 msg;
    msg.x(f.x); msg.y(f.y); msg.z(f.z);
    return encode(msg);
}

std::vector<std::uint8_t> encode_pose(const bench::fixtures::PoseFixture& f) {
    geometry_msgs::msg::Pose msg;
    msg.position().x(f.px); msg.position().y(f.py); msg.position().z(f.pz);
    msg.orientation().x(f.qx); msg.orientation().y(f.qy);
    msg.orientation().z(f.qz); msg.orientation().w(f.qw);
    return encode(msg);
}

std::vector<std::uint8_t> encode_dmabuffer(const bench::fixtures::DmaBufferFixture& f) {
    edgefirst_msgs::msg::DmaBuffer msg;
    msg.header().stamp().sec(f.stamp_sec);
    msg.header().stamp().nanosec(f.stamp_nanos);
    msg.header().frame_id(f.frame_id);
    msg.pid(f.pid); msg.fd(f.fd);
    msg.width(f.width); msg.height(f.height);
    msg.stride(f.stride); msg.fourcc(f.fourcc); msg.length(f.length);
    return encode(msg);
}

std::vector<std::uint8_t> encode_image(const bench::fixtures::ImageVariant& v,
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
    return encode(msg);
}

std::vector<std::uint8_t> encode_radarcube(const bench::fixtures::RadarCubeVariant& v,
                                            const std::vector<std::int16_t>& cube) {
    edgefirst_msgs::msg::RadarCube msg;
    msg.header().stamp().sec(1234567890);
    msg.header().stamp().nanosec(0);
    msg.header().frame_id("radar");
    msg.timestamp(0);
    msg.shape(std::vector<std::uint16_t>{v.shape[0], v.shape[1], v.shape[2], v.shape[3]});
    msg.cube(cube);
    msg.is_complex(false);
    return encode(msg);
}

std::vector<std::uint8_t> encode_mask(const bench::fixtures::MaskVariant& v,
                                       const std::vector<std::uint8_t>& payload) {
    edgefirst_msgs::msg::Mask msg;
    msg.height(v.height);
    msg.width(v.width);
    msg.length(static_cast<std::uint32_t>(payload.size()));
    msg.encoding("mono8");
    msg.mask(payload);
    msg.boxed(false);
    return encode(msg);
}

std::vector<std::uint8_t> encode_compressedvideo(const std::vector<std::uint8_t>& payload) {
    foxglove_msgs::msg::CompressedVideo msg;
    msg.timestamp().sec(1234567890);
    msg.timestamp().nanosec(123456789);
    msg.frame_id("cam");
    msg.data(payload);
    msg.format("h264");
    return encode(msg);
}

std::vector<std::uint8_t> encode_pointcloud2(const bench::fixtures::PointCloud2Variant& v,
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
    return encode(msg);
}

} // namespace parity_cdds
