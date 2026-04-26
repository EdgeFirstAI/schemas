// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Au-Zone Technologies. All Rights Reserved.

#include <benchmark/benchmark.h>
#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>
#include <fastcdr/CdrSizeCalculator.hpp>

#include <memory>

#include "std_msgs/Header.hpp"
// *CdrAux.ipp files contain the template specializations for
// calculate_serialized_size, serialize, and deserialize.
// Time must be included before Header since Header's specializations
// call the Time ones.
#include "builtin_interfaces/TimeCdrAux.ipp"
#include "std_msgs/HeaderCdrAux.ipp"

// Geometry and edgefirst types
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

// Large message types
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

// ===========================================================================
// Generic FastCDR helpers
// ===========================================================================

template <typename T>
static std::vector<char> fastcdr_encode_msg(const T& msg) {
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
    buf.resize(cdr.get_serialized_data_length());
    return buf;
}

template <typename T>
static T fastcdr_decode_msg(const std::vector<char>& bytes) {
    eprosima::fastcdr::FastBuffer fb(const_cast<char*>(bytes.data()), bytes.size());
    eprosima::fastcdr::Cdr cdr(fb,
        eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
        eprosima::fastcdr::CdrVersion::XCDRv1);
    cdr.read_encapsulation();
    T msg;
    cdr >> msg;
    return msg;
}

// Single-iteration encode for benchmarks (allocates buffer each time)
template <typename T>
static void fastcdr_encode_bench(benchmark::State& state, const T& msg) {
    for (auto _ : state) {
        eprosima::fastcdr::CdrSizeCalculator calc(eprosima::fastcdr::CdrVersion::XCDRv1);
        size_t current_alignment = 0;
        size_t size = eprosima::fastcdr::calculate_serialized_size(calc, msg, current_alignment);
        std::vector<char> buffer(size + 4);
        eprosima::fastcdr::FastBuffer fb(buffer.data(), buffer.size());
        eprosima::fastcdr::Cdr cdr(fb,
            eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
            eprosima::fastcdr::CdrVersion::XCDRv1);
        cdr.serialize_encapsulation();
        cdr << msg;
        benchmark::DoNotOptimize(buffer);
    }
}

// ===========================================================================
// Header benchmarks
// ===========================================================================

static void Header_encode_new(benchmark::State& state) {
    bench::fixtures::HeaderFixture f;
    std_msgs::msg::Header msg;
    msg.stamp().sec(f.stamp_sec);
    msg.stamp().nanosec(f.stamp_nanos);
    msg.frame_id(f.frame_id);
    fastcdr_encode_bench(state, msg);
}

static const std::vector<char>& header_wire_bytes_v() {
    static const std::vector<char> bytes = [] {
        bench::fixtures::HeaderFixture f;
        std_msgs::msg::Header msg;
        msg.stamp().sec(f.stamp_sec);
        msg.stamp().nanosec(f.stamp_nanos);
        msg.frame_id(f.frame_id);
        return fastcdr_encode_msg(msg);
    }();
    return bytes;
}

static void Header_decode_decode(benchmark::State& state) {
    auto& bytes = header_wire_bytes_v();
    for (auto _ : state) {
        auto msg = fastcdr_decode_msg<std_msgs::msg::Header>(bytes);
        benchmark::DoNotOptimize(msg);
    }
}

static void Header_access_one_field(benchmark::State& state) {
    auto& bytes = header_wire_bytes_v();
    for (auto _ : state) {
        auto msg = fastcdr_decode_msg<std_msgs::msg::Header>(bytes);
        auto sec = msg.stamp().sec();
        benchmark::DoNotOptimize(sec);
    }
}

static void Header_access_half_fields(benchmark::State& state) {
    auto& bytes = header_wire_bytes_v();
    for (auto _ : state) {
        auto msg   = fastcdr_decode_msg<std_msgs::msg::Header>(bytes);
        auto sec   = msg.stamp().sec();
        auto nanos = msg.stamp().nanosec();
        benchmark::DoNotOptimize(sec);
        benchmark::DoNotOptimize(nanos);
    }
}

static void Header_access_all_fields(benchmark::State& state) {
    auto& bytes = header_wire_bytes_v();
    for (auto _ : state) {
        auto msg   = fastcdr_decode_msg<std_msgs::msg::Header>(bytes);
        auto sec   = msg.stamp().sec();
        auto nanos = msg.stamp().nanosec();
        auto fid   = msg.frame_id();
        benchmark::DoNotOptimize(sec);
        benchmark::DoNotOptimize(nanos);
        benchmark::DoNotOptimize(fid);
    }
}

static void register_header_benchmarks() {
    benchmark::RegisterBenchmark(
        bench::bench_name("Header", bench::op::encode_new, "default").c_str(),
        Header_encode_new);
    benchmark::RegisterBenchmark(
        bench::bench_name("Header", bench::op::decode_decode, "default").c_str(),
        Header_decode_decode);
    benchmark::RegisterBenchmark(
        bench::bench_name("Header", bench::op::access_one_field, "default").c_str(),
        Header_access_one_field);
    benchmark::RegisterBenchmark(
        bench::bench_name("Header", bench::op::access_half_fields, "default").c_str(),
        Header_access_half_fields);
    benchmark::RegisterBenchmark(
        bench::bench_name("Header", bench::op::access_all_fields, "default").c_str(),
        Header_access_all_fields);
}

// ===========================================================================
// Small types benchmarks (Time, Vector3, Pose, DmaBuffer)
// ===========================================================================

static void register_small_type_benchmarks() {
    // ---- Time ----
    static const std::vector<char> time_bytes = [] {
        bench::fixtures::TimeFixture f;
        builtin_interfaces::msg::Time msg;
        msg.sec(f.sec); msg.nanosec(f.nanos);
        return fastcdr_encode_msg(msg);
    }();

    benchmark::RegisterBenchmark(
        bench::bench_name("Time", bench::op::encode_new, "default").c_str(),
        [](benchmark::State& state) {
            bench::fixtures::TimeFixture f;
            builtin_interfaces::msg::Time msg;
            msg.sec(f.sec); msg.nanosec(f.nanos);
            fastcdr_encode_bench(state, msg);
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Time", bench::op::decode_decode, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto msg = fastcdr_decode_msg<builtin_interfaces::msg::Time>(time_bytes);
                benchmark::DoNotOptimize(msg);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Time", bench::op::access_one_field, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto msg = fastcdr_decode_msg<builtin_interfaces::msg::Time>(time_bytes);
                auto sec = msg.sec();
                benchmark::DoNotOptimize(sec);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Time", bench::op::access_half_fields, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto msg = fastcdr_decode_msg<builtin_interfaces::msg::Time>(time_bytes);
                auto sec = msg.sec();
                benchmark::DoNotOptimize(sec);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Time", bench::op::access_all_fields, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto msg   = fastcdr_decode_msg<builtin_interfaces::msg::Time>(time_bytes);
                auto sec   = msg.sec();
                auto nanos = msg.nanosec();
                benchmark::DoNotOptimize(sec); benchmark::DoNotOptimize(nanos);
            }
        });

    // ---- Vector3 ----
    static const std::vector<char> v3_bytes = [] {
        bench::fixtures::Vector3Fixture f;
        geometry_msgs::msg::Vector3 msg;
        msg.x(f.x); msg.y(f.y); msg.z(f.z);
        return fastcdr_encode_msg(msg);
    }();

    benchmark::RegisterBenchmark(
        bench::bench_name("Vector3", bench::op::encode_new, "default").c_str(),
        [](benchmark::State& state) {
            bench::fixtures::Vector3Fixture f;
            geometry_msgs::msg::Vector3 msg;
            msg.x(f.x); msg.y(f.y); msg.z(f.z);
            fastcdr_encode_bench(state, msg);
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Vector3", bench::op::decode_decode, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto msg = fastcdr_decode_msg<geometry_msgs::msg::Vector3>(v3_bytes);
                benchmark::DoNotOptimize(msg);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Vector3", bench::op::access_one_field, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto msg = fastcdr_decode_msg<geometry_msgs::msg::Vector3>(v3_bytes);
                auto x = msg.x();
                benchmark::DoNotOptimize(x);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Vector3", bench::op::access_half_fields, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto msg = fastcdr_decode_msg<geometry_msgs::msg::Vector3>(v3_bytes);
                auto x = msg.x(); auto y = msg.y();
                benchmark::DoNotOptimize(x); benchmark::DoNotOptimize(y);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Vector3", bench::op::access_all_fields, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto msg = fastcdr_decode_msg<geometry_msgs::msg::Vector3>(v3_bytes);
                auto x = msg.x(); auto y = msg.y(); auto z = msg.z();
                benchmark::DoNotOptimize(x); benchmark::DoNotOptimize(y);
                benchmark::DoNotOptimize(z);
            }
        });

    // ---- Pose ----
    static const std::vector<char> pose_bytes = [] {
        bench::fixtures::PoseFixture f;
        geometry_msgs::msg::Pose msg;
        msg.position().x(f.px); msg.position().y(f.py); msg.position().z(f.pz);
        msg.orientation().x(f.qx); msg.orientation().y(f.qy);
        msg.orientation().z(f.qz); msg.orientation().w(f.qw);
        return fastcdr_encode_msg(msg);
    }();

    benchmark::RegisterBenchmark(
        bench::bench_name("Pose", bench::op::encode_new, "default").c_str(),
        [](benchmark::State& state) {
            bench::fixtures::PoseFixture f;
            geometry_msgs::msg::Pose msg;
            msg.position().x(f.px); msg.position().y(f.py); msg.position().z(f.pz);
            msg.orientation().x(f.qx); msg.orientation().y(f.qy);
            msg.orientation().z(f.qz); msg.orientation().w(f.qw);
            fastcdr_encode_bench(state, msg);
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Pose", bench::op::decode_decode, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto msg = fastcdr_decode_msg<geometry_msgs::msg::Pose>(pose_bytes);
                benchmark::DoNotOptimize(msg);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Pose", bench::op::access_one_field, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto msg = fastcdr_decode_msg<geometry_msgs::msg::Pose>(pose_bytes);
                auto px = msg.position().x();
                benchmark::DoNotOptimize(px);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Pose", bench::op::access_half_fields, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto msg = fastcdr_decode_msg<geometry_msgs::msg::Pose>(pose_bytes);
                auto px = msg.position().x(); auto py = msg.position().y();
                auto pz = msg.position().z();
                benchmark::DoNotOptimize(px); benchmark::DoNotOptimize(py);
                benchmark::DoNotOptimize(pz);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Pose", bench::op::access_all_fields, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto msg = fastcdr_decode_msg<geometry_msgs::msg::Pose>(pose_bytes);
                auto px = msg.position().x(); auto py = msg.position().y();
                auto pz = msg.position().z();
                auto ox = msg.orientation().x(); auto oy = msg.orientation().y();
                auto oz = msg.orientation().z(); auto ow = msg.orientation().w();
                benchmark::DoNotOptimize(px); benchmark::DoNotOptimize(py);
                benchmark::DoNotOptimize(pz); benchmark::DoNotOptimize(ox);
                benchmark::DoNotOptimize(oy); benchmark::DoNotOptimize(oz);
                benchmark::DoNotOptimize(ow);
            }
        });

    // ---- DmaBuffer ----
    static const std::vector<char> dma_bytes = [] {
        bench::fixtures::DmaBufferFixture f;
        edgefirst_msgs::msg::DmaBuffer msg;
        msg.header().stamp().sec(f.stamp_sec);
        msg.header().stamp().nanosec(f.stamp_nanos);
        msg.header().frame_id(f.frame_id);
        msg.pid(f.pid); msg.fd(f.fd);
        msg.width(f.width); msg.height(f.height);
        msg.stride(f.stride); msg.fourcc(f.fourcc); msg.length(f.length);
        return fastcdr_encode_msg(msg);
    }();

    benchmark::RegisterBenchmark(
        bench::bench_name("DmaBuffer", bench::op::encode_new, "default").c_str(),
        [](benchmark::State& state) {
            bench::fixtures::DmaBufferFixture f;
            edgefirst_msgs::msg::DmaBuffer msg;
            msg.header().stamp().sec(f.stamp_sec);
            msg.header().stamp().nanosec(f.stamp_nanos);
            msg.header().frame_id(f.frame_id);
            msg.pid(f.pid); msg.fd(f.fd);
            msg.width(f.width); msg.height(f.height);
            msg.stride(f.stride); msg.fourcc(f.fourcc); msg.length(f.length);
            fastcdr_encode_bench(state, msg);
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("DmaBuffer", bench::op::decode_decode, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto msg = fastcdr_decode_msg<edgefirst_msgs::msg::DmaBuffer>(dma_bytes);
                benchmark::DoNotOptimize(msg);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("DmaBuffer", bench::op::access_one_field, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto msg = fastcdr_decode_msg<edgefirst_msgs::msg::DmaBuffer>(dma_bytes);
                auto sec = msg.header().stamp().sec();
                benchmark::DoNotOptimize(sec);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("DmaBuffer", bench::op::access_half_fields, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto msg    = fastcdr_decode_msg<edgefirst_msgs::msg::DmaBuffer>(dma_bytes);
                auto sec    = msg.header().stamp().sec();
                auto width  = msg.width();
                auto height = msg.height();
                auto length = msg.length();
                benchmark::DoNotOptimize(sec);    benchmark::DoNotOptimize(width);
                benchmark::DoNotOptimize(height); benchmark::DoNotOptimize(length);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("DmaBuffer", bench::op::access_all_fields, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto msg    = fastcdr_decode_msg<edgefirst_msgs::msg::DmaBuffer>(dma_bytes);
                auto sec    = msg.header().stamp().sec();
                auto nanos  = msg.header().stamp().nanosec();
                auto fid    = msg.header().frame_id();
                auto pid    = msg.pid();
                auto fd     = msg.fd();
                auto width  = msg.width();
                auto height = msg.height();
                auto stride = msg.stride();
                auto fourcc = msg.fourcc();
                auto length = msg.length();
                benchmark::DoNotOptimize(sec);    benchmark::DoNotOptimize(nanos);
                benchmark::DoNotOptimize(fid);    benchmark::DoNotOptimize(pid);
                benchmark::DoNotOptimize(fd);     benchmark::DoNotOptimize(width);
                benchmark::DoNotOptimize(height); benchmark::DoNotOptimize(stride);
                benchmark::DoNotOptimize(fourcc); benchmark::DoNotOptimize(length);
            }
        });
}

// ===========================================================================
// Image benchmarks
// ===========================================================================

static void register_image_benchmarks() {
    for (const auto& v : bench::fixtures::kImageVariants) {
        std::size_t payload_size = bench::fixtures::image_payload_bytes(v);
        auto payload_ptr = std::make_shared<std::vector<std::uint8_t>>(
            bench::make_payload(payload_size));

        // Build the fastcdr message once
        auto msg_ptr = std::make_shared<sensor_msgs::msg::Image>([&]() {
            sensor_msgs::msg::Image m;
            m.header().stamp().sec(1234567890);
            m.header().stamp().nanosec(123456789);
            m.header().frame_id("cam");
            m.height(v.height);
            m.width(v.width);
            m.encoding(std::string(v.encoding));
            m.is_bigendian(0);
            m.step(v.width * v.step_bpp);
            m.data(*payload_ptr);
            return m;
        }());

        // Pre-encode wire bytes
        auto wire_ptr = std::make_shared<std::vector<char>>(fastcdr_encode_msg(*msg_ptr));

        // encode/new
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::encode_new, v.name).c_str(),
            [msg_ptr](benchmark::State& state) {
                fastcdr_encode_bench(state, *msg_ptr);
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(msg_ptr->data().size()));
            });

        // decode/decode
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::decode_decode, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<sensor_msgs::msg::Image>(*wire_ptr);
                    benchmark::DoNotOptimize(msg);
                }
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(wire_ptr->size()));
            });

        // access/one_field: stamp.sec
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::access_one_field, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<sensor_msgs::msg::Image>(*wire_ptr);
                    auto sec = msg.header().stamp().sec();
                    benchmark::DoNotOptimize(sec);
                }
            });

        // access/half_fields: stamp + height + width + encoding
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::access_half_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<sensor_msgs::msg::Image>(*wire_ptr);
                    auto sec = msg.header().stamp().sec();
                    auto h   = msg.height();
                    auto w   = msg.width();
                    auto enc = msg.encoding();
                    benchmark::DoNotOptimize(sec); benchmark::DoNotOptimize(h);
                    benchmark::DoNotOptimize(w);   benchmark::DoNotOptimize(enc);
                }
            });

        // access/all_fields
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::access_all_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg  = fastcdr_decode_msg<sensor_msgs::msg::Image>(*wire_ptr);
                    auto sec  = msg.header().stamp().sec();
                    auto ns   = msg.header().stamp().nanosec();
                    auto fid  = msg.header().frame_id();
                    auto h    = msg.height();
                    auto w    = msg.width();
                    auto enc  = msg.encoding();
                    auto be   = msg.is_bigendian();
                    auto stp  = msg.step();
                    auto dlen = msg.data().size();
                    benchmark::DoNotOptimize(sec); benchmark::DoNotOptimize(ns);
                    benchmark::DoNotOptimize(fid); benchmark::DoNotOptimize(h);
                    benchmark::DoNotOptimize(w);   benchmark::DoNotOptimize(enc);
                    benchmark::DoNotOptimize(be);  benchmark::DoNotOptimize(stp);
                    benchmark::DoNotOptimize(dlen);
                }
            });

        // access/payload_iter
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::access_payload_iter, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg  = fastcdr_decode_msg<sensor_msgs::msg::Image>(*wire_ptr);
                    auto& d   = msg.data();
                    std::uint64_t sum = 0;
                    for (auto b : d) sum += b;
                    benchmark::DoNotOptimize(sum);
                }
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(wire_ptr->size()));
            });
    }
}

// ===========================================================================
// RadarCube benchmarks
// ===========================================================================

static void register_radarcube_benchmarks() {
    for (const auto& v : bench::fixtures::kRadarCubeVariants) {
        auto cube_ptr = std::make_shared<std::vector<std::int16_t>>(
            v.cube_elements, 0);
        std::uint32_t s = 0xDEAD;
        for (auto& x : *cube_ptr) {
            s = s * 1664525u + 1013904223u;
            x = static_cast<std::int16_t>(s >> 16);
        }

        std::array<std::uint16_t, 4> shape_arr = {
            v.shape[0], v.shape[1], v.shape[2], v.shape[3]
        };

        // Build fastcdr message
        auto msg_ptr = std::make_shared<edgefirst_msgs::msg::RadarCube>([&]() {
            edgefirst_msgs::msg::RadarCube m;
            m.header().stamp().sec(1234567890);
            m.header().stamp().nanosec(0);
            m.header().frame_id("radar");
            m.timestamp(0);
            m.shape(std::vector<std::uint16_t>(shape_arr.begin(), shape_arr.end()));
            m.cube(*cube_ptr);
            m.is_complex(false);
            return m;
        }());

        auto wire_ptr = std::make_shared<std::vector<char>>(fastcdr_encode_msg(*msg_ptr));

        // encode/new
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::encode_new, v.name).c_str(),
            [msg_ptr](benchmark::State& state) {
                fastcdr_encode_bench(state, *msg_ptr);
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(msg_ptr->cube().size() * 2));
            });

        // decode/decode
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::decode_decode, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<edgefirst_msgs::msg::RadarCube>(*wire_ptr);
                    benchmark::DoNotOptimize(msg);
                }
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(wire_ptr->size()));
            });

        // access/one_field: stamp.sec
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::access_one_field, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<edgefirst_msgs::msg::RadarCube>(*wire_ptr);
                    auto sec = msg.header().stamp().sec();
                    benchmark::DoNotOptimize(sec);
                }
            });

        // access/half_fields: stamp + timestamp + cube size
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::access_half_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg  = fastcdr_decode_msg<edgefirst_msgs::msg::RadarCube>(*wire_ptr);
                    auto sec  = msg.header().stamp().sec();
                    auto ts   = msg.timestamp();
                    auto clen = msg.cube().size();
                    benchmark::DoNotOptimize(sec); benchmark::DoNotOptimize(ts);
                    benchmark::DoNotOptimize(clen);
                }
            });

        // access/all_fields
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::access_all_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg  = fastcdr_decode_msg<edgefirst_msgs::msg::RadarCube>(*wire_ptr);
                    auto sec  = msg.header().stamp().sec();
                    auto ns   = msg.header().stamp().nanosec();
                    auto fid  = msg.header().frame_id();
                    auto ts   = msg.timestamp();
                    auto lay  = msg.layout().size();
                    auto clen = msg.cube().size();
                    auto cplx = msg.is_complex();
                    benchmark::DoNotOptimize(sec); benchmark::DoNotOptimize(ns);
                    benchmark::DoNotOptimize(fid); benchmark::DoNotOptimize(ts);
                    benchmark::DoNotOptimize(lay); benchmark::DoNotOptimize(clen);
                    benchmark::DoNotOptimize(cplx);
                }
            });

        // access/payload_iter
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::access_payload_iter, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<edgefirst_msgs::msg::RadarCube>(*wire_ptr);
                    auto& c  = msg.cube();
                    std::int64_t sum = 0;
                    for (auto x : c) sum += x;
                    benchmark::DoNotOptimize(sum);
                }
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(wire_ptr->size()));
            });
    }
}

// ===========================================================================
// Mask benchmarks
// ===========================================================================

static void register_mask_benchmarks() {
    for (const auto& v : bench::fixtures::kMaskVariants) {
        std::size_t payload_size = static_cast<std::size_t>(v.width) *
                                   static_cast<std::size_t>(v.height);
        auto payload_ptr = std::make_shared<std::vector<std::uint8_t>>(
            bench::make_payload(payload_size));

        auto msg_ptr = std::make_shared<edgefirst_msgs::msg::Mask>([&]() {
            edgefirst_msgs::msg::Mask m;
            m.height(v.height);
            m.width(v.width);
            m.length(static_cast<std::uint32_t>(payload_size));
            m.encoding("mono8");
            m.mask(*payload_ptr);
            m.boxed(false);
            return m;
        }());

        auto wire_ptr = std::make_shared<std::vector<char>>(fastcdr_encode_msg(*msg_ptr));

        // encode/new
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::encode_new, v.name).c_str(),
            [msg_ptr](benchmark::State& state) {
                fastcdr_encode_bench(state, *msg_ptr);
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(msg_ptr->mask().size()));
            });

        // decode/decode
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::decode_decode, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<edgefirst_msgs::msg::Mask>(*wire_ptr);
                    benchmark::DoNotOptimize(msg);
                }
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(wire_ptr->size()));
            });

        // access/one_field: height
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::access_one_field, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<edgefirst_msgs::msg::Mask>(*wire_ptr);
                    auto h   = msg.height();
                    benchmark::DoNotOptimize(h);
                }
            });

        // access/half_fields: height + width + length + encoding
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::access_half_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<edgefirst_msgs::msg::Mask>(*wire_ptr);
                    auto h   = msg.height();
                    auto w   = msg.width();
                    auto len = msg.length();
                    auto enc = msg.encoding();
                    benchmark::DoNotOptimize(h);   benchmark::DoNotOptimize(w);
                    benchmark::DoNotOptimize(len); benchmark::DoNotOptimize(enc);
                }
            });

        // access/all_fields
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::access_all_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg  = fastcdr_decode_msg<edgefirst_msgs::msg::Mask>(*wire_ptr);
                    auto h    = msg.height();
                    auto w    = msg.width();
                    auto len  = msg.length();
                    auto enc  = msg.encoding();
                    auto boxd = msg.boxed();
                    auto dlen = msg.mask().size();
                    benchmark::DoNotOptimize(h);    benchmark::DoNotOptimize(w);
                    benchmark::DoNotOptimize(len);  benchmark::DoNotOptimize(enc);
                    benchmark::DoNotOptimize(boxd); benchmark::DoNotOptimize(dlen);
                }
            });

        // access/payload_iter
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::access_payload_iter, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<edgefirst_msgs::msg::Mask>(*wire_ptr);
                    auto& d  = msg.mask();
                    std::uint64_t sum = 0;
                    for (auto b : d) sum += b;
                    benchmark::DoNotOptimize(sum);
                }
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(wire_ptr->size()));
            });
    }
}

// ===========================================================================
// CompressedVideo benchmarks
// ===========================================================================

static void register_compressedvideo_benchmarks() {
    for (const auto& v : bench::fixtures::kCompressedVideoVariants) {
        auto payload_ptr = std::make_shared<std::vector<std::uint8_t>>(
            bench::make_payload(v.payload_bytes));

        auto msg_ptr = std::make_shared<foxglove_msgs::msg::CompressedVideo>([&]() {
            foxglove_msgs::msg::CompressedVideo m;
            m.timestamp().sec(1234567890);
            m.timestamp().nanosec(123456789);
            m.frame_id("cam");
            m.data(*payload_ptr);
            m.format("h264");
            return m;
        }());

        auto wire_ptr = std::make_shared<std::vector<char>>(fastcdr_encode_msg(*msg_ptr));

        // encode/new
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::encode_new, v.name).c_str(),
            [msg_ptr](benchmark::State& state) {
                fastcdr_encode_bench(state, *msg_ptr);
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(msg_ptr->data().size()));
            });

        // decode/decode
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::decode_decode, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<foxglove_msgs::msg::CompressedVideo>(*wire_ptr);
                    benchmark::DoNotOptimize(msg);
                }
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(wire_ptr->size()));
            });

        // access/one_field: timestamp.sec
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::access_one_field, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<foxglove_msgs::msg::CompressedVideo>(*wire_ptr);
                    auto sec = msg.timestamp().sec();
                    benchmark::DoNotOptimize(sec);
                }
            });

        // access/half_fields: timestamp + frame_id + format
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::access_half_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<foxglove_msgs::msg::CompressedVideo>(*wire_ptr);
                    auto sec = msg.timestamp().sec();
                    auto fid = msg.frame_id();
                    auto fmt = msg.format();
                    benchmark::DoNotOptimize(sec); benchmark::DoNotOptimize(fid);
                    benchmark::DoNotOptimize(fmt);
                }
            });

        // access/all_fields
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::access_all_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg  = fastcdr_decode_msg<foxglove_msgs::msg::CompressedVideo>(*wire_ptr);
                    auto sec  = msg.timestamp().sec();
                    auto ns   = msg.timestamp().nanosec();
                    auto fid  = msg.frame_id();
                    auto fmt  = msg.format();
                    auto dlen = msg.data().size();
                    benchmark::DoNotOptimize(sec); benchmark::DoNotOptimize(ns);
                    benchmark::DoNotOptimize(fid); benchmark::DoNotOptimize(fmt);
                    benchmark::DoNotOptimize(dlen);
                }
            });

        // access/payload_iter
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::access_payload_iter, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<foxglove_msgs::msg::CompressedVideo>(*wire_ptr);
                    auto& d  = msg.data();
                    std::uint64_t sum = 0;
                    for (auto b : d) sum += b;
                    benchmark::DoNotOptimize(sum);
                }
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(wire_ptr->size()));
            });
    }
}

// ===========================================================================
// PointCloud2 benchmarks
// ===========================================================================

static void register_pointcloud2_benchmarks() {
    for (const auto& v : bench::fixtures::kPointCloud2Variants) {
        std::size_t payload_size = static_cast<std::size_t>(v.num_points) *
                                   static_cast<std::size_t>(v.point_step);
        auto payload_ptr = std::make_shared<std::vector<std::uint8_t>>(
            bench::make_payload(payload_size));

        // Build PointField descriptors
        auto make_fields = []() {
            sensor_msgs::msg::PointField fx, fy, fz, fi;
            fx.name("x");  fx.offset(0);  fx.datatype(7); fx.count(1);
            fy.name("y");  fy.offset(4);  fy.datatype(7); fy.count(1);
            fz.name("z");  fz.offset(8);  fz.datatype(7); fz.count(1);
            fi.name("intensity"); fi.offset(12); fi.datatype(7); fi.count(1);
            return std::vector<sensor_msgs::msg::PointField>{fx, fy, fz, fi};
        };

        auto msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>([&]() {
            sensor_msgs::msg::PointCloud2 m;
            m.header().stamp().sec(1234567890);
            m.header().stamp().nanosec(123456789);
            m.header().frame_id("lidar");
            m.height(1);
            m.width(v.num_points);
            m.fields(make_fields());
            m.is_bigendian(false);
            m.point_step(v.point_step);
            m.row_step(v.num_points * v.point_step);
            m.data(*payload_ptr);
            m.is_dense(true);
            return m;
        }());

        auto wire_ptr = std::make_shared<std::vector<char>>(fastcdr_encode_msg(*msg_ptr));

        // encode/new
        benchmark::RegisterBenchmark(
            bench::bench_name("PointCloud2", bench::op::encode_new, v.name).c_str(),
            [msg_ptr](benchmark::State& state) {
                fastcdr_encode_bench(state, *msg_ptr);
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(msg_ptr->data().size()));
            });

        // decode/decode
        benchmark::RegisterBenchmark(
            bench::bench_name("PointCloud2", bench::op::decode_decode, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<sensor_msgs::msg::PointCloud2>(*wire_ptr);
                    benchmark::DoNotOptimize(msg);
                }
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(wire_ptr->size()));
            });

        // access/one_field: stamp.sec
        benchmark::RegisterBenchmark(
            bench::bench_name("PointCloud2", bench::op::access_one_field, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<sensor_msgs::msg::PointCloud2>(*wire_ptr);
                    auto sec = msg.header().stamp().sec();
                    benchmark::DoNotOptimize(sec);
                }
            });

        // access/half_fields: stamp + height + width + point_step + fields.size()
        benchmark::RegisterBenchmark(
            bench::bench_name("PointCloud2", bench::op::access_half_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<sensor_msgs::msg::PointCloud2>(*wire_ptr);
                    auto sec = msg.header().stamp().sec();
                    auto h   = msg.height();
                    auto w   = msg.width();
                    auto ps  = msg.point_step();
                    auto fl  = msg.fields().size();
                    benchmark::DoNotOptimize(sec); benchmark::DoNotOptimize(h);
                    benchmark::DoNotOptimize(w);   benchmark::DoNotOptimize(ps);
                    benchmark::DoNotOptimize(fl);
                }
            });

        // access/all_fields
        benchmark::RegisterBenchmark(
            bench::bench_name("PointCloud2", bench::op::access_all_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg  = fastcdr_decode_msg<sensor_msgs::msg::PointCloud2>(*wire_ptr);
                    auto sec  = msg.header().stamp().sec();
                    auto ns   = msg.header().stamp().nanosec();
                    auto fid  = msg.header().frame_id();
                    auto h    = msg.height();
                    auto w    = msg.width();
                    auto ps   = msg.point_step();
                    auto rs   = msg.row_step();
                    auto fl   = msg.fields().size();
                    auto be   = msg.is_bigendian();
                    auto isd  = msg.is_dense();
                    auto dlen = msg.data().size();
                    benchmark::DoNotOptimize(sec);  benchmark::DoNotOptimize(ns);
                    benchmark::DoNotOptimize(fid);  benchmark::DoNotOptimize(h);
                    benchmark::DoNotOptimize(w);    benchmark::DoNotOptimize(ps);
                    benchmark::DoNotOptimize(rs);   benchmark::DoNotOptimize(fl);
                    benchmark::DoNotOptimize(be);   benchmark::DoNotOptimize(isd);
                    benchmark::DoNotOptimize(dlen);
                }
            });

        // access/payload_iter
        benchmark::RegisterBenchmark(
            bench::bench_name("PointCloud2", bench::op::access_payload_iter, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<sensor_msgs::msg::PointCloud2>(*wire_ptr);
                    auto& d  = msg.data();
                    std::uint64_t sum = 0;
                    for (auto b : d) sum += b;
                    benchmark::DoNotOptimize(sum);
                }
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(wire_ptr->size()));
            });
    }
}

// ===========================================================================
// Workflow benchmarks
// ===========================================================================
//
// Representative variants: Header/default, Image/HD_rgb8, RadarCube/DRVEGRD169_short,
// Mask/640x640_8class, CompressedVideo/100KB, PointCloud2/medium_10K
//
// Fast-CDR has no in-place mutation mode: pub_loop_inplace uses the same
// full-rebuild approach as pub_loop_rebuild. This is intentional — it shows
// the structural asymmetry vs the edgefirst backend where ros_*_set_stamp
// allows a single in-place write instead of a full re-serialize.

static void register_workflow_benchmarks() {
    // Helper lambda for single-iteration inline encode (allocates buffer each time)
    auto inline_encode = [](auto& msg) -> std::vector<char> {
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
        buf.resize(cdr.get_serialized_data_length());
        return buf;
    };
    (void)inline_encode; // suppress "unused" if compiler is aggressive

    // ========================================================
    // Header / default
    // ========================================================
    {
        static const std::vector<char> hdr_wire = [] {
            bench::fixtures::HeaderFixture f;
            std_msgs::msg::Header msg;
            msg.stamp().sec(f.stamp_sec);
            msg.stamp().nanosec(f.stamp_nanos);
            msg.frame_id(f.frame_id);
            return fastcdr_encode_msg(msg);
        }();

        // workflow/sub_modify_pub
        benchmark::RegisterBenchmark(
            bench::bench_name("Header", bench::op::workflow_smp, "default").c_str(),
            [](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<std_msgs::msg::Header>(hdr_wire);
                    msg.stamp().sec(1234567891);             // mutate timestamp
                    auto buf = fastcdr_encode_msg(msg);
                    benchmark::DoNotOptimize(buf);
                }
            });

        // workflow/pub_loop_rebuild
        benchmark::RegisterBenchmark(
            bench::bench_name("Header", bench::op::workflow_pub_rebuild, "default").c_str(),
            [](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        std_msgs::msg::Header msg;
                        msg.stamp().sec(1234567890 + i);
                        msg.stamp().nanosec(0);
                        msg.frame_id("cam");
                        auto buf = fastcdr_encode_msg(msg);
                        benchmark::DoNotOptimize(buf);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/pub_loop_inplace: Fast-CDR has no in-place mode; full rebuild
        benchmark::RegisterBenchmark(
            bench::bench_name("Header", bench::op::workflow_pub_inplace, "default").c_str(),
            [](benchmark::State& state) {
                // NOTE: Fast-CDR has no in-place setter; rebuilds every iteration.
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        std_msgs::msg::Header msg;
                        msg.stamp().sec(1234567890 + i);
                        msg.stamp().nanosec(0);
                        msg.frame_id("cam");
                        auto buf = fastcdr_encode_msg(msg);
                        benchmark::DoNotOptimize(buf);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/sub_loop: decode + read stamp.sec + frame_id
        benchmark::RegisterBenchmark(
            bench::bench_name("Header", bench::op::workflow_sub, "default").c_str(),
            [](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto msg = fastcdr_decode_msg<std_msgs::msg::Header>(hdr_wire);
                        auto sec = msg.stamp().sec();
                        auto fid = msg.frame_id();
                        benchmark::DoNotOptimize(sec);
                        benchmark::DoNotOptimize(fid);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });
    }

    // ========================================================
    // Image / HD_rgb8
    // ========================================================
    {
        const bench::fixtures::ImageVariant* vp = nullptr;
        for (const auto& v : bench::fixtures::kImageVariants) {
            if (v.name == "HD_rgb8") { vp = &v; break; }
        }
        const auto& iv = *vp;

        std::size_t payload_size = bench::fixtures::image_payload_bytes(iv);
        auto payload_ptr = std::make_shared<std::vector<std::uint8_t>>(
            bench::make_payload(payload_size));

        auto msg_ptr = std::make_shared<sensor_msgs::msg::Image>([&]() {
            sensor_msgs::msg::Image m;
            m.header().stamp().sec(1234567890);
            m.header().stamp().nanosec(123456789);
            m.header().frame_id("cam");
            m.height(iv.height);
            m.width(iv.width);
            m.encoding(std::string(iv.encoding));
            m.is_bigendian(0);
            m.step(iv.width * iv.step_bpp);
            m.data(*payload_ptr);
            return m;
        }());
        auto wire_ptr = std::make_shared<std::vector<char>>(fastcdr_encode_msg(*msg_ptr));

        // workflow/sub_modify_pub
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::workflow_smp, "HD_rgb8").c_str(),
            [wire_ptr, msg_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<sensor_msgs::msg::Image>(*wire_ptr);
                    msg.header().stamp().sec(1234567891);    // mutate timestamp
                    auto buf = fastcdr_encode_msg(msg);
                    benchmark::DoNotOptimize(buf);
                }
            });

        // workflow/pub_loop_rebuild
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::workflow_pub_rebuild, "HD_rgb8").c_str(),
            [msg_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        msg_ptr->header().stamp().sec(1234567890 + i);
                        auto buf = fastcdr_encode_msg(*msg_ptr);
                        benchmark::DoNotOptimize(buf);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/pub_loop_inplace: Fast-CDR has no in-place mode; full re-serialize
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::workflow_pub_inplace, "HD_rgb8").c_str(),
            [msg_ptr](benchmark::State& state) {
                // NOTE: Fast-CDR has no in-place setter; re-serializes every iteration.
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        msg_ptr->header().stamp().sec(1234567890 + i);
                        auto buf = fastcdr_encode_msg(*msg_ptr);
                        benchmark::DoNotOptimize(buf);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/sub_loop: decode + read stamp.sec + height + width
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::workflow_sub, "HD_rgb8").c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto msg = fastcdr_decode_msg<sensor_msgs::msg::Image>(*wire_ptr);
                        auto sec = msg.header().stamp().sec();
                        auto h   = msg.height();
                        auto w   = msg.width();
                        benchmark::DoNotOptimize(sec);
                        benchmark::DoNotOptimize(h);
                        benchmark::DoNotOptimize(w);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });
    }

    // ========================================================
    // RadarCube / DRVEGRD169_short
    // ========================================================
    {
        const bench::fixtures::RadarCubeVariant* rp = nullptr;
        for (const auto& v : bench::fixtures::kRadarCubeVariants) {
            if (v.name == "DRVEGRD169_short") { rp = &v; break; }
        }
        const auto& rv = *rp;

        auto cube_ptr = std::make_shared<std::vector<std::int16_t>>(rv.cube_elements, 0);
        {
            std::uint32_t s = 0xDEAD;
            for (auto& x : *cube_ptr) {
                s = s * 1664525u + 1013904223u;
                x = static_cast<std::int16_t>(s >> 16);
            }
        }
        std::array<std::uint16_t, 4> shape_arr = {rv.shape[0], rv.shape[1], rv.shape[2], rv.shape[3]};

        auto msg_ptr = std::make_shared<edgefirst_msgs::msg::RadarCube>([&]() {
            edgefirst_msgs::msg::RadarCube m;
            m.header().stamp().sec(1234567890);
            m.header().stamp().nanosec(0);
            m.header().frame_id("radar");
            m.timestamp(0);
            m.shape(std::vector<std::uint16_t>(shape_arr.begin(), shape_arr.end()));
            m.cube(*cube_ptr);
            m.is_complex(false);
            return m;
        }());
        auto wire_ptr = std::make_shared<std::vector<char>>(fastcdr_encode_msg(*msg_ptr));

        // workflow/sub_modify_pub
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::workflow_smp, "DRVEGRD169_short").c_str(),
            [wire_ptr, msg_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<edgefirst_msgs::msg::RadarCube>(*wire_ptr);
                    msg.header().stamp().sec(1234567891);
                    msg.timestamp(msg.timestamp() + 1);
                    auto buf = fastcdr_encode_msg(msg);
                    benchmark::DoNotOptimize(buf);
                }
            });

        // workflow/pub_loop_rebuild
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::workflow_pub_rebuild, "DRVEGRD169_short").c_str(),
            [msg_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        msg_ptr->header().stamp().sec(1234567890 + i);
                        msg_ptr->timestamp(static_cast<uint64_t>(i));
                        auto buf = fastcdr_encode_msg(*msg_ptr);
                        benchmark::DoNotOptimize(buf);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/pub_loop_inplace: Fast-CDR has no in-place mode; full re-serialize
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::workflow_pub_inplace, "DRVEGRD169_short").c_str(),
            [msg_ptr](benchmark::State& state) {
                // NOTE: Fast-CDR has no in-place setter; re-serializes every iteration.
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        msg_ptr->header().stamp().sec(1234567890 + i);
                        msg_ptr->timestamp(static_cast<uint64_t>(i));
                        auto buf = fastcdr_encode_msg(*msg_ptr);
                        benchmark::DoNotOptimize(buf);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/sub_loop
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::workflow_sub, "DRVEGRD169_short").c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto msg  = fastcdr_decode_msg<edgefirst_msgs::msg::RadarCube>(*wire_ptr);
                        auto sec  = msg.header().stamp().sec();
                        auto ts   = msg.timestamp();
                        auto clen = msg.cube().size();
                        benchmark::DoNotOptimize(sec);
                        benchmark::DoNotOptimize(ts);
                        benchmark::DoNotOptimize(clen);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });
    }

    // ========================================================
    // Mask / 640x640_8class
    // ========================================================
    {
        const bench::fixtures::MaskVariant* mp = nullptr;
        for (const auto& v : bench::fixtures::kMaskVariants) {
            if (v.name == "640x640_8class") { mp = &v; break; }
        }
        const auto& mv = *mp;

        std::size_t payload_size = static_cast<std::size_t>(mv.width) *
                                   static_cast<std::size_t>(mv.height);
        auto payload_ptr = std::make_shared<std::vector<std::uint8_t>>(
            bench::make_payload(payload_size));

        auto msg_ptr = std::make_shared<edgefirst_msgs::msg::Mask>([&]() {
            edgefirst_msgs::msg::Mask m;
            m.height(mv.height);
            m.width(mv.width);
            m.length(static_cast<std::uint32_t>(payload_size));
            m.encoding("mono8");
            m.mask(*payload_ptr);
            m.boxed(false);
            return m;
        }());
        auto wire_ptr = std::make_shared<std::vector<char>>(fastcdr_encode_msg(*msg_ptr));

        // workflow/sub_modify_pub
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::workflow_smp, "640x640_8class").c_str(),
            [wire_ptr, msg_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<edgefirst_msgs::msg::Mask>(*wire_ptr);
                    // Mask has no stamp; re-encode with same dims (simulates re-pub)
                    auto buf = fastcdr_encode_msg(msg);
                    benchmark::DoNotOptimize(buf);
                }
            });

        // workflow/pub_loop_rebuild
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::workflow_pub_rebuild, "640x640_8class").c_str(),
            [msg_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto buf = fastcdr_encode_msg(*msg_ptr);
                        benchmark::DoNotOptimize(buf);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/pub_loop_inplace: Fast-CDR has no in-place mode; full re-serialize
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::workflow_pub_inplace, "640x640_8class").c_str(),
            [msg_ptr](benchmark::State& state) {
                // NOTE: Fast-CDR has no in-place setter; re-serializes every iteration.
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto buf = fastcdr_encode_msg(*msg_ptr);
                        benchmark::DoNotOptimize(buf);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/sub_loop: decode + read height + width + encoding
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::workflow_sub, "640x640_8class").c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto msg = fastcdr_decode_msg<edgefirst_msgs::msg::Mask>(*wire_ptr);
                        auto h   = msg.height();
                        auto w   = msg.width();
                        auto enc = msg.encoding();
                        benchmark::DoNotOptimize(h);
                        benchmark::DoNotOptimize(w);
                        benchmark::DoNotOptimize(enc);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });
    }

    // ========================================================
    // CompressedVideo / 100KB
    // ========================================================
    {
        const bench::fixtures::CompressedVideoVariant* cvp = nullptr;
        for (const auto& v : bench::fixtures::kCompressedVideoVariants) {
            if (v.name == "100KB") { cvp = &v; break; }
        }
        const auto& cvv = *cvp;

        auto payload_ptr = std::make_shared<std::vector<std::uint8_t>>(
            bench::make_payload(cvv.payload_bytes));

        auto msg_ptr = std::make_shared<foxglove_msgs::msg::CompressedVideo>([&]() {
            foxglove_msgs::msg::CompressedVideo m;
            m.timestamp().sec(1234567890);
            m.timestamp().nanosec(123456789);
            m.frame_id("cam");
            m.data(*payload_ptr);
            m.format("h264");
            return m;
        }());
        auto wire_ptr = std::make_shared<std::vector<char>>(fastcdr_encode_msg(*msg_ptr));

        // workflow/sub_modify_pub
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::workflow_smp, "100KB").c_str(),
            [wire_ptr, msg_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<foxglove_msgs::msg::CompressedVideo>(*wire_ptr);
                    msg.timestamp().sec(1234567891);
                    auto buf = fastcdr_encode_msg(msg);
                    benchmark::DoNotOptimize(buf);
                }
            });

        // workflow/pub_loop_rebuild
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::workflow_pub_rebuild, "100KB").c_str(),
            [msg_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        msg_ptr->timestamp().sec(1234567890 + i);
                        auto buf = fastcdr_encode_msg(*msg_ptr);
                        benchmark::DoNotOptimize(buf);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/pub_loop_inplace: Fast-CDR has no in-place mode; full re-serialize
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::workflow_pub_inplace, "100KB").c_str(),
            [msg_ptr](benchmark::State& state) {
                // NOTE: Fast-CDR has no in-place setter; re-serializes every iteration.
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        msg_ptr->timestamp().sec(1234567890 + i);
                        auto buf = fastcdr_encode_msg(*msg_ptr);
                        benchmark::DoNotOptimize(buf);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/sub_loop: decode + read timestamp.sec + format
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::workflow_sub, "100KB").c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto msg = fastcdr_decode_msg<foxglove_msgs::msg::CompressedVideo>(*wire_ptr);
                        auto sec = msg.timestamp().sec();
                        auto fmt = msg.format();
                        benchmark::DoNotOptimize(sec);
                        benchmark::DoNotOptimize(fmt);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });
    }

    // ========================================================
    // PointCloud2 / medium_10K
    // ========================================================
    {
        const bench::fixtures::PointCloud2Variant* pcp = nullptr;
        for (const auto& v : bench::fixtures::kPointCloud2Variants) {
            if (v.name == "robosense_e1r") { pcp = &v; break; }
        }
        const auto& pcv = *pcp;

        std::size_t payload_size = static_cast<std::size_t>(pcv.num_points) *
                                   static_cast<std::size_t>(pcv.point_step);
        auto payload_ptr = std::make_shared<std::vector<std::uint8_t>>(
            bench::make_payload(payload_size));

        auto make_fields = []() {
            sensor_msgs::msg::PointField fx, fy, fz, fi;
            fx.name("x");  fx.offset(0);  fx.datatype(7); fx.count(1);
            fy.name("y");  fy.offset(4);  fy.datatype(7); fy.count(1);
            fz.name("z");  fz.offset(8);  fz.datatype(7); fz.count(1);
            fi.name("intensity"); fi.offset(12); fi.datatype(7); fi.count(1);
            return std::vector<sensor_msgs::msg::PointField>{fx, fy, fz, fi};
        };

        auto msg_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>([&]() {
            sensor_msgs::msg::PointCloud2 m;
            m.header().stamp().sec(1234567890);
            m.header().stamp().nanosec(123456789);
            m.header().frame_id("lidar");
            m.height(1);
            m.width(pcv.num_points);
            m.fields(make_fields());
            m.is_bigendian(false);
            m.point_step(pcv.point_step);
            m.row_step(pcv.num_points * pcv.point_step);
            m.data(*payload_ptr);
            m.is_dense(true);
            return m;
        }());
        auto wire_ptr = std::make_shared<std::vector<char>>(fastcdr_encode_msg(*msg_ptr));

        // workflow/sub_modify_pub
        benchmark::RegisterBenchmark(
            bench::bench_name("PointCloud2", bench::op::workflow_smp, "robosense_e1r").c_str(),
            [wire_ptr, msg_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto msg = fastcdr_decode_msg<sensor_msgs::msg::PointCloud2>(*wire_ptr);
                    msg.header().stamp().sec(1234567891);
                    auto buf = fastcdr_encode_msg(msg);
                    benchmark::DoNotOptimize(buf);
                }
            });

        // workflow/pub_loop_rebuild
        benchmark::RegisterBenchmark(
            bench::bench_name("PointCloud2", bench::op::workflow_pub_rebuild, "robosense_e1r").c_str(),
            [msg_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        msg_ptr->header().stamp().sec(1234567890 + i);
                        auto buf = fastcdr_encode_msg(*msg_ptr);
                        benchmark::DoNotOptimize(buf);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/pub_loop_inplace: Fast-CDR has no in-place mode; full re-serialize
        benchmark::RegisterBenchmark(
            bench::bench_name("PointCloud2", bench::op::workflow_pub_inplace, "robosense_e1r").c_str(),
            [msg_ptr](benchmark::State& state) {
                // NOTE: Fast-CDR has no in-place setter; re-serializes every iteration.
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        msg_ptr->header().stamp().sec(1234567890 + i);
                        auto buf = fastcdr_encode_msg(*msg_ptr);
                        benchmark::DoNotOptimize(buf);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/sub_loop: decode + read stamp.sec + width + point_step
        benchmark::RegisterBenchmark(
            bench::bench_name("PointCloud2", bench::op::workflow_sub, "robosense_e1r").c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto msg = fastcdr_decode_msg<sensor_msgs::msg::PointCloud2>(*wire_ptr);
                        auto sec = msg.header().stamp().sec();
                        auto w   = msg.width();
                        auto ps  = msg.point_step();
                        benchmark::DoNotOptimize(sec);
                        benchmark::DoNotOptimize(w);
                        benchmark::DoNotOptimize(ps);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });
    }
}

// ===========================================================================
// main
// ===========================================================================

int main(int argc, char** argv) {
    register_header_benchmarks();
    register_small_type_benchmarks();
    register_image_benchmarks();
    register_radarcube_benchmarks();
    register_mask_benchmarks();
    register_compressedvideo_benchmarks();
    register_pointcloud2_benchmarks();
    register_workflow_benchmarks();
    ::benchmark::Initialize(&argc, argv);
    ::benchmark::RunSpecifiedBenchmarks();
    ::benchmark::Shutdown();
    return 0;
}
