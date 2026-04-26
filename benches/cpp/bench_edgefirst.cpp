// SPDX-License-Identifier: Apache-2.0
// Copyright (c) 2026 Au-Zone Technologies. All Rights Reserved.

#include <benchmark/benchmark.h>
#include <edgefirst/schemas.hpp>
#include "common.hpp"

#include <memory>

namespace ef = edgefirst::schemas;

// ===========================================================================
// Helpers
// ===========================================================================

// Encode a message via a builder and return wire bytes.
// Returns empty vector on failure.
template <typename BuildFn>
static std::vector<std::uint8_t> ef_build_to_bytes(BuildFn&& build_fn) {
    auto result = build_fn();
    if (!result) return {};
    auto& r = *result;
    std::vector<std::uint8_t> v(r.data, r.data + r.size);
    ros_bytes_free(r.data, r.size);
    return v;
}

// Free the bytes held by a builder's `build()` result. `Released` is a POD
// without a destructor (see schemas.hpp ownership notes), so dropping it
// leaks the encoded buffer. Call after `DoNotOptimize` in benchmark loops
// where the encoded bytes are immediately discarded — leaving it out causes
// RSS to grow per iteration on large messages.
template <typename ReleasedExpected>
static inline void ef_release(const ReleasedExpected& r) {
    if (r) ros_bytes_free(r->data, r->size);
}

// ===========================================================================
// Header benchmarks
// ===========================================================================

static void Header_encode_new(benchmark::State& state) {
    bench::fixtures::HeaderFixture f;
    for (auto _ : state) {
        auto b = ef::HeaderBuilder::create();
        if (!b) { state.SkipWithError("HeaderBuilder::create failed"); break; }
        b->stamp(ef::Time{f.stamp_sec, f.stamp_nanos});
        auto fi = b->frame_id(f.frame_id.c_str());
        (void)fi;
        auto result = b->build();
        benchmark::DoNotOptimize(result);
    }
}

// Lazily initialised once (Meyers singleton) and reused across all
// decode/access benchmarks so we measure only the operation under test.
static const std::vector<std::uint8_t>& header_wire_bytes() {
    static const std::vector<std::uint8_t> bytes = [] {
        bench::fixtures::HeaderFixture f;
        auto b = ef::HeaderBuilder::create();
        b->stamp(ef::Time{f.stamp_sec, f.stamp_nanos});
        (void)b->frame_id(f.frame_id.c_str());
        auto r = b->build();
        std::vector<std::uint8_t> v(r->data, r->data + r->size);
        ros_bytes_free(r->data, r->size);
        return v;
    }();
    return bytes;
}

static void Header_decode_decode(benchmark::State& state) {
    auto& bytes = header_wire_bytes();
    for (auto _ : state) {
        auto v = ef::HeaderView::from_cdr({bytes.data(), bytes.size()});
        benchmark::DoNotOptimize(v);
    }
}

static void Header_access_one_field(benchmark::State& state) {
    auto& bytes = header_wire_bytes();
    for (auto _ : state) {
        auto v = ef::HeaderView::from_cdr({bytes.data(), bytes.size()});
        auto sec = v->stamp().sec;
        benchmark::DoNotOptimize(sec);
    }
}

static void Header_access_half_fields(benchmark::State& state) {
    auto& bytes = header_wire_bytes();
    for (auto _ : state) {
        auto v = ef::HeaderView::from_cdr({bytes.data(), bytes.size()});
        auto t = v->stamp();
        auto sec   = t.sec;
        auto nanos = t.nanosec;
        benchmark::DoNotOptimize(sec);
        benchmark::DoNotOptimize(nanos);
    }
}

static void Header_access_all_fields(benchmark::State& state) {
    auto& bytes = header_wire_bytes();
    for (auto _ : state) {
        auto v = ef::HeaderView::from_cdr({bytes.data(), bytes.size()});
        auto t     = v->stamp();
        auto sec   = t.sec;
        auto nanos = t.nanosec;
        auto fid   = v->frame_id();
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
// Time benchmarks
// ===========================================================================

static const std::vector<std::uint8_t>& time_wire_bytes() {
    static const std::vector<std::uint8_t> bytes = [] {
        bench::fixtures::TimeFixture f;
        ef::Time t{f.sec, f.nanos};
        auto sz = t.encoded_size();
        std::vector<std::uint8_t> buf(*sz);
        (void)t.encode({buf.data(), buf.size()});
        return buf;
    }();
    return bytes;
}

static void Time_encode_new(benchmark::State& state) {
    bench::fixtures::TimeFixture f;
    for (auto _ : state) {
        ef::Time t{f.sec, f.nanos};
        auto sz = t.encoded_size();
        std::vector<std::uint8_t> buf(*sz);
        auto result = t.encode({buf.data(), buf.size()});
        benchmark::DoNotOptimize(result);
        benchmark::DoNotOptimize(buf);
    }
}

static void Time_decode_decode(benchmark::State& state) {
    auto& bytes = time_wire_bytes();
    for (auto _ : state) {
        auto v = ef::Time::decode({bytes.data(), bytes.size()});
        benchmark::DoNotOptimize(v);
    }
}

static void Time_access_one_field(benchmark::State& state) {
    auto& bytes = time_wire_bytes();
    for (auto _ : state) {
        auto v = ef::Time::decode({bytes.data(), bytes.size()});
        auto sec = v->sec;
        benchmark::DoNotOptimize(sec);
    }
}

static void Time_access_half_fields(benchmark::State& state) {
    auto& bytes = time_wire_bytes();
    for (auto _ : state) {
        auto v = ef::Time::decode({bytes.data(), bytes.size()});
        auto sec = v->sec;
        benchmark::DoNotOptimize(sec);
    }
}

static void Time_access_all_fields(benchmark::State& state) {
    auto& bytes = time_wire_bytes();
    for (auto _ : state) {
        auto v = ef::Time::decode({bytes.data(), bytes.size()});
        auto sec   = v->sec;
        auto nanos = v->nanosec;
        benchmark::DoNotOptimize(sec);
        benchmark::DoNotOptimize(nanos);
    }
}

static void register_small_type_benchmarks() {
    benchmark::RegisterBenchmark(
        bench::bench_name("Time", bench::op::encode_new, "default").c_str(),
        Time_encode_new);
    benchmark::RegisterBenchmark(
        bench::bench_name("Time", bench::op::decode_decode, "default").c_str(),
        Time_decode_decode);
    benchmark::RegisterBenchmark(
        bench::bench_name("Time", bench::op::access_one_field, "default").c_str(),
        Time_access_one_field);
    benchmark::RegisterBenchmark(
        bench::bench_name("Time", bench::op::access_half_fields, "default").c_str(),
        Time_access_half_fields);
    benchmark::RegisterBenchmark(
        bench::bench_name("Time", bench::op::access_all_fields, "default").c_str(),
        Time_access_all_fields);

    // ---- Vector3 ----
    static const std::vector<std::uint8_t> v3_bytes = [] {
        bench::fixtures::Vector3Fixture f;
        ef::Vector3 v{f.x, f.y, f.z};
        auto sz = v.encoded_size();
        std::vector<std::uint8_t> buf(*sz);
        (void)v.encode({buf.data(), buf.size()});
        return buf;
    }();

    benchmark::RegisterBenchmark(
        bench::bench_name("Vector3", bench::op::encode_new, "default").c_str(),
        [](benchmark::State& state) {
            bench::fixtures::Vector3Fixture f;
            for (auto _ : state) {
                ef::Vector3 v{f.x, f.y, f.z};
                auto sz = v.encoded_size();
                std::vector<std::uint8_t> buf(*sz);
                auto result = v.encode({buf.data(), buf.size()});
                benchmark::DoNotOptimize(result);
                benchmark::DoNotOptimize(buf);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Vector3", bench::op::decode_decode, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto v = ef::Vector3::decode({v3_bytes.data(), v3_bytes.size()});
                benchmark::DoNotOptimize(v);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Vector3", bench::op::access_one_field, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto v = ef::Vector3::decode({v3_bytes.data(), v3_bytes.size()});
                auto x = v->x;
                benchmark::DoNotOptimize(x);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Vector3", bench::op::access_half_fields, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto v = ef::Vector3::decode({v3_bytes.data(), v3_bytes.size()});
                auto x = v->x; auto y = v->y;
                benchmark::DoNotOptimize(x); benchmark::DoNotOptimize(y);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Vector3", bench::op::access_all_fields, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto v = ef::Vector3::decode({v3_bytes.data(), v3_bytes.size()});
                auto x = v->x; auto y = v->y; auto z = v->z;
                benchmark::DoNotOptimize(x); benchmark::DoNotOptimize(y);
                benchmark::DoNotOptimize(z);
            }
        });

    // ---- Pose ----
    static const std::vector<std::uint8_t> pose_bytes = [] {
        bench::fixtures::PoseFixture f;
        ef::Pose p{f.px, f.py, f.pz, f.qx, f.qy, f.qz, f.qw};
        auto sz = p.encoded_size();
        std::vector<std::uint8_t> buf(*sz);
        (void)p.encode({buf.data(), buf.size()});
        return buf;
    }();

    benchmark::RegisterBenchmark(
        bench::bench_name("Pose", bench::op::encode_new, "default").c_str(),
        [](benchmark::State& state) {
            bench::fixtures::PoseFixture f;
            for (auto _ : state) {
                ef::Pose p{f.px, f.py, f.pz, f.qx, f.qy, f.qz, f.qw};
                auto sz = p.encoded_size();
                std::vector<std::uint8_t> buf(*sz);
                auto result = p.encode({buf.data(), buf.size()});
                benchmark::DoNotOptimize(result);
                benchmark::DoNotOptimize(buf);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Pose", bench::op::decode_decode, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto v = ef::Pose::decode({pose_bytes.data(), pose_bytes.size()});
                benchmark::DoNotOptimize(v);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Pose", bench::op::access_one_field, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto v = ef::Pose::decode({pose_bytes.data(), pose_bytes.size()});
                auto px = v->px;
                benchmark::DoNotOptimize(px);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Pose", bench::op::access_half_fields, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto v = ef::Pose::decode({pose_bytes.data(), pose_bytes.size()});
                auto px = v->px; auto py = v->py; auto pz = v->pz;
                benchmark::DoNotOptimize(px); benchmark::DoNotOptimize(py);
                benchmark::DoNotOptimize(pz);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("Pose", bench::op::access_all_fields, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto v = ef::Pose::decode({pose_bytes.data(), pose_bytes.size()});
                auto px = v->px; auto py = v->py; auto pz = v->pz;
                auto ox = v->ox; auto oy = v->oy; auto oz = v->oz; auto ow = v->ow;
                benchmark::DoNotOptimize(px); benchmark::DoNotOptimize(py);
                benchmark::DoNotOptimize(pz); benchmark::DoNotOptimize(ox);
                benchmark::DoNotOptimize(oy); benchmark::DoNotOptimize(oz);
                benchmark::DoNotOptimize(ow);
            }
        });

    // ---- DmaBuffer ----
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    static const std::vector<std::uint8_t> dma_bytes = [] {
        bench::fixtures::DmaBufferFixture f;
        auto r = ef::DmaBuffer::encode(
            ef::Time{f.stamp_sec, f.stamp_nanos},
            f.frame_id,
            f.pid, f.fd,
            f.width, f.height,
            f.stride, f.fourcc, f.length);
        auto sp = r->as_cdr();
        std::vector<std::uint8_t> v(sp.data(), sp.data() + sp.size());
        return v;
    }();
#pragma GCC diagnostic pop

    benchmark::RegisterBenchmark(
        bench::bench_name("DmaBuffer", bench::op::encode_new, "default").c_str(),
        [](benchmark::State& state) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            bench::fixtures::DmaBufferFixture f;
            for (auto _ : state) {
                auto result = ef::DmaBuffer::encode(
                    ef::Time{f.stamp_sec, f.stamp_nanos},
                    f.frame_id,
                    f.pid, f.fd,
                    f.width, f.height,
                    f.stride, f.fourcc, f.length);
                benchmark::DoNotOptimize(result);
            }
#pragma GCC diagnostic pop
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("DmaBuffer", bench::op::decode_decode, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto v = ef::DmaBufferView::from_cdr({dma_bytes.data(), dma_bytes.size()});
                benchmark::DoNotOptimize(v);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("DmaBuffer", bench::op::access_one_field, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto v = ef::DmaBufferView::from_cdr({dma_bytes.data(), dma_bytes.size()});
                auto sec = v->stamp().sec;
                benchmark::DoNotOptimize(sec);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("DmaBuffer", bench::op::access_half_fields, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto v   = ef::DmaBufferView::from_cdr({dma_bytes.data(), dma_bytes.size()});
                auto sec = v->stamp().sec;
                auto w   = v->width();
                auto h   = v->height();
                auto len = v->length();
                benchmark::DoNotOptimize(sec); benchmark::DoNotOptimize(w);
                benchmark::DoNotOptimize(h);   benchmark::DoNotOptimize(len);
            }
        });
    benchmark::RegisterBenchmark(
        bench::bench_name("DmaBuffer", bench::op::access_all_fields, "default").c_str(),
        [](benchmark::State& state) {
            for (auto _ : state) {
                auto v      = ef::DmaBufferView::from_cdr({dma_bytes.data(), dma_bytes.size()});
                auto t      = v->stamp();
                auto sec    = t.sec;
                auto nanos  = t.nanosec;
                auto fid    = v->frame_id();
                auto pid    = v->pid();
                auto fd     = v->fd();
                auto width  = v->width();
                auto height = v->height();
                auto stride = v->stride();
                auto fourcc = v->fourcc();
                auto length = v->length();
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
        // Build wire bytes for this variant (shared across decode/access ops)
        std::size_t payload_size = bench::fixtures::image_payload_bytes(v);
        auto payload_ptr = std::make_shared<std::vector<std::uint8_t>>(
            bench::make_payload(payload_size));

        // Pre-encode wire bytes for decode/access ops
        auto wire_ptr = std::make_shared<std::vector<std::uint8_t>>([&]() {
            std::uint32_t step = v.width * v.step_bpp;
            auto r = ef::Image::encode(
                ef::Time{1234567890, 123456789},
                "cam",
                v.height, v.width,
                std::string(v.encoding),
                false,
                step,
                {payload_ptr->data(), payload_ptr->size()});
            if (!r) return std::vector<std::uint8_t>{};
            auto sp = r->as_cdr();
            return std::vector<std::uint8_t>(sp.data(), sp.data() + sp.size());
        }());

        // encode/new
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::encode_new, v.name).c_str(),
            [v, payload_ptr](benchmark::State& state) {
                std::uint32_t step = v.width * v.step_bpp;
                for (auto _ : state) {
                    auto r = ef::Image::encode(
                        ef::Time{1234567890, 123456789},
                        "cam",
                        v.height, v.width,
                        std::string(v.encoding),
                        false,
                        step,
                        {payload_ptr->data(), payload_ptr->size()});
                    benchmark::DoNotOptimize(r);
                }
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(payload_ptr->size()));
            });

        // decode/decode
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::decode_decode, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw = ef::ImageView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    benchmark::DoNotOptimize(vw);
                }
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(wire_ptr->size()));
            });

        // access/one_field
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::access_one_field, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw  = ef::ImageView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto sec = vw->stamp().sec;
                    benchmark::DoNotOptimize(sec);
                }
            });

        // access/half_fields: stamp + height + width + encoding
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::access_half_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw  = ef::ImageView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto sec = vw->stamp().sec;
                    auto h   = vw->height();
                    auto w   = vw->width();
                    auto enc = vw->encoding();
                    benchmark::DoNotOptimize(sec); benchmark::DoNotOptimize(h);
                    benchmark::DoNotOptimize(w);   benchmark::DoNotOptimize(enc);
                }
            });

        // access/all_fields
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::access_all_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw  = ef::ImageView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto t   = vw->stamp();
                    auto sec = t.sec; auto ns = t.nanosec;
                    auto fid = vw->frame_id();
                    auto h   = vw->height();
                    auto w   = vw->width();
                    auto enc = vw->encoding();
                    auto be  = vw->is_bigendian();
                    auto stp = vw->step();
                    auto dlen = vw->data().size();
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
                    auto vw   = ef::ImageView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto data = vw->data();
                    std::uint64_t sum = 0;
                    for (auto b : data) sum += b;
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
        // Fill with deterministic pseudo-random data
        std::uint32_t s = 0xDEAD;
        for (auto& x : *cube_ptr) {
            s = s * 1664525u + 1013904223u;
            x = static_cast<std::int16_t>(s >> 16);
        }

        std::array<std::uint16_t, 4> shape_arr = {
            v.shape[0], v.shape[1], v.shape[2], v.shape[3]
        };
        auto shape_ptr = std::make_shared<std::array<std::uint16_t, 4>>(shape_arr);

        // Pre-encode wire bytes
        auto wire_ptr = std::make_shared<std::vector<std::uint8_t>>([&]() {
            auto b = ef::RadarCubeBuilder::create();
            if (!b) return std::vector<std::uint8_t>{};
            b->stamp(ef::Time{1234567890, 0});
            (void)b->frame_id("radar");
            b->timestamp(0);
            (void)b->shape({shape_arr.data(), shape_arr.size()});
            (void)b->cube({cube_ptr->data(), cube_ptr->size()});
            b->is_complex(false);
            auto r = b->build();
            if (!r) return std::vector<std::uint8_t>{};
            std::vector<std::uint8_t> out(r->data, r->data + r->size);
            ros_bytes_free(r->data, r->size);
            return out;
        }());

        // encode/new
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::encode_new, v.name).c_str(),
            [shape_ptr, cube_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto b = ef::RadarCubeBuilder::create();
                    if (!b) { state.SkipWithError("create failed"); break; }
                    b->stamp(ef::Time{1234567890, 0});
                    (void)b->frame_id("radar");
                    b->timestamp(0);
                    (void)b->shape({shape_ptr->data(), shape_ptr->size()});
                    (void)b->cube({cube_ptr->data(), cube_ptr->size()});
                    b->is_complex(false);
                    auto r = b->build();
                    benchmark::DoNotOptimize(r);
                    ef_release(r);
                }
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(cube_ptr->size() * 2));
            });

        // decode/decode
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::decode_decode, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw = ef::RadarCubeView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    benchmark::DoNotOptimize(vw);
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
                    auto vw  = ef::RadarCubeView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto sec = vw->stamp().sec;
                    benchmark::DoNotOptimize(sec);
                }
            });

        // access/half_fields: stamp + timestamp + cube_len
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::access_half_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw  = ef::RadarCubeView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto sec = vw->stamp().sec;
                    auto ts  = vw->timestamp();
                    auto clen = vw->cube_len();
                    benchmark::DoNotOptimize(sec); benchmark::DoNotOptimize(ts);
                    benchmark::DoNotOptimize(clen);
                }
            });

        // access/all_fields
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::access_all_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw   = ef::RadarCubeView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto t    = vw->stamp();
                    auto sec  = t.sec; auto ns = t.nanosec;
                    auto fid  = vw->frame_id();
                    auto ts   = vw->timestamp();
                    auto lay  = vw->layout();
                    auto clen = vw->cube_len();
                    auto cplx = vw->is_complex();
                    benchmark::DoNotOptimize(sec); benchmark::DoNotOptimize(ns);
                    benchmark::DoNotOptimize(fid); benchmark::DoNotOptimize(ts);
                    benchmark::DoNotOptimize(lay); benchmark::DoNotOptimize(clen);
                    benchmark::DoNotOptimize(cplx);
                }
            });

        // access/payload_iter: iterate raw cube bytes
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::access_payload_iter, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw   = ef::RadarCubeView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto data = vw->cube_raw();
                    std::uint64_t sum = 0;
                    for (auto b : data) sum += b;
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

        // Pre-encode wire bytes
        auto wire_ptr = std::make_shared<std::vector<std::uint8_t>>([&]() {
            auto r = ef::Mask::encode(
                v.height, v.width,
                static_cast<std::uint32_t>(payload_size),
                "mono8",
                {payload_ptr->data(), payload_ptr->size()},
                false);
            if (!r) return std::vector<std::uint8_t>{};
            auto sp = r->as_cdr();
            return std::vector<std::uint8_t>(sp.data(), sp.data() + sp.size());
        }());

        // encode/new
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::encode_new, v.name).c_str(),
            [v, payload_ptr](benchmark::State& state) {
                std::size_t ps = static_cast<std::size_t>(v.width) *
                                 static_cast<std::size_t>(v.height);
                for (auto _ : state) {
                    auto r = ef::Mask::encode(
                        v.height, v.width,
                        static_cast<std::uint32_t>(ps),
                        "mono8",
                        {payload_ptr->data(), payload_ptr->size()},
                        false);
                    benchmark::DoNotOptimize(r);
                }
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(payload_ptr->size()));
            });

        // decode/decode
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::decode_decode, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw = ef::MaskView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    benchmark::DoNotOptimize(vw);
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
                    auto vw = ef::MaskView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto h  = vw->height();
                    benchmark::DoNotOptimize(h);
                }
            });

        // access/half_fields: height + width + length + encoding
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::access_half_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw  = ef::MaskView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto h   = vw->height();
                    auto w   = vw->width();
                    auto len = vw->length();
                    auto enc = vw->encoding();
                    benchmark::DoNotOptimize(h);   benchmark::DoNotOptimize(w);
                    benchmark::DoNotOptimize(len); benchmark::DoNotOptimize(enc);
                }
            });

        // access/all_fields
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::access_all_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw   = ef::MaskView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto h    = vw->height();
                    auto w    = vw->width();
                    auto len  = vw->length();
                    auto enc  = vw->encoding();
                    auto boxd = vw->boxed();
                    auto dlen = vw->data().size();
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
                    auto vw   = ef::MaskView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto data = vw->data();
                    std::uint64_t sum = 0;
                    for (auto b : data) sum += b;
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

        // Pre-encode wire bytes
        auto wire_ptr = std::make_shared<std::vector<std::uint8_t>>([&]() {
            auto r = ef::CompressedVideo::encode(
                ef::Time{1234567890, 123456789},
                "cam",
                {payload_ptr->data(), payload_ptr->size()},
                "h264");
            if (!r) return std::vector<std::uint8_t>{};
            auto sp = r->as_cdr();
            return std::vector<std::uint8_t>(sp.data(), sp.data() + sp.size());
        }());

        // encode/new
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::encode_new, v.name).c_str(),
            [payload_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto r = ef::CompressedVideo::encode(
                        ef::Time{1234567890, 123456789},
                        "cam",
                        {payload_ptr->data(), payload_ptr->size()},
                        "h264");
                    benchmark::DoNotOptimize(r);
                }
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(payload_ptr->size()));
            });

        // decode/decode
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::decode_decode, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw = ef::CompressedVideoView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    benchmark::DoNotOptimize(vw);
                }
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(wire_ptr->size()));
            });

        // access/one_field: stamp.sec
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::access_one_field, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw  = ef::CompressedVideoView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto sec = vw->stamp().sec;
                    benchmark::DoNotOptimize(sec);
                }
            });

        // access/half_fields: stamp + frame_id + format
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::access_half_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw  = ef::CompressedVideoView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto sec = vw->stamp().sec;
                    auto fid = vw->frame_id();
                    auto fmt = vw->format();
                    benchmark::DoNotOptimize(sec); benchmark::DoNotOptimize(fid);
                    benchmark::DoNotOptimize(fmt);
                }
            });

        // access/all_fields
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::access_all_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw   = ef::CompressedVideoView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto t    = vw->stamp();
                    auto sec  = t.sec; auto ns = t.nanosec;
                    auto fid  = vw->frame_id();
                    auto fmt  = vw->format();
                    auto dlen = vw->data().size();
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
                    auto vw   = ef::CompressedVideoView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto data = vw->data();
                    std::uint64_t sum = 0;
                    for (auto b : data) sum += b;
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
    // 4 fields: x, y, z, intensity (each float32 = 4 bytes, offset 0/4/8/12)
    // datatype 7 = FLOAT32 per ros_point_field_builder_set_datatype doc
    static const ros_point_field_elem_t kFields[] = {
        {"x",         0,  7, 1},
        {"y",         4,  7, 1},
        {"z",         8,  7, 1},
        {"intensity", 12, 7, 1},
    };
    constexpr std::size_t kFieldCount = 4;

    for (const auto& v : bench::fixtures::kPointCloud2Variants) {
        std::size_t payload_size = static_cast<std::size_t>(v.num_points) *
                                   static_cast<std::size_t>(v.point_step);
        auto payload_ptr = std::make_shared<std::vector<std::uint8_t>>(
            bench::make_payload(payload_size));

        // Pre-encode wire bytes
        auto wire_ptr = std::make_shared<std::vector<std::uint8_t>>([&]() {
            auto b = ef::PointCloud2Builder::create();
            if (!b) return std::vector<std::uint8_t>{};
            b->stamp(ef::Time{1234567890, 123456789});
            (void)b->frame_id("lidar");
            b->height(1);
            b->width(v.num_points);
            (void)b->fields({kFields, kFieldCount});
            b->is_bigendian(false);
            b->point_step(v.point_step);
            b->row_step(v.num_points * v.point_step);
            (void)b->data({payload_ptr->data(), payload_ptr->size()});
            b->is_dense(true);
            auto r = b->build();
            if (!r) return std::vector<std::uint8_t>{};
            std::vector<std::uint8_t> out(r->data, r->data + r->size);
            ros_bytes_free(r->data, r->size);
            return out;
        }());

        // encode/new
        benchmark::RegisterBenchmark(
            bench::bench_name("PointCloud2", bench::op::encode_new, v.name).c_str(),
            [v, payload_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto b = ef::PointCloud2Builder::create();
                    if (!b) { state.SkipWithError("create failed"); break; }
                    b->stamp(ef::Time{1234567890, 123456789});
                    (void)b->frame_id("lidar");
                    b->height(1);
                    b->width(v.num_points);
                    (void)b->fields({kFields, kFieldCount});
                    b->is_bigendian(false);
                    b->point_step(v.point_step);
                    b->row_step(v.num_points * v.point_step);
                    (void)b->data({payload_ptr->data(), payload_ptr->size()});
                    b->is_dense(true);
                    auto r = b->build();
                    benchmark::DoNotOptimize(r);
                    ef_release(r);
                }
                state.SetBytesProcessed(
                    static_cast<std::int64_t>(state.iterations()) *
                    static_cast<std::int64_t>(payload_ptr->size()));
            });

        // decode/decode
        benchmark::RegisterBenchmark(
            bench::bench_name("PointCloud2", bench::op::decode_decode, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw = ef::PointCloud2View::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    benchmark::DoNotOptimize(vw);
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
                    auto vw  = ef::PointCloud2View::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto sec = vw->stamp().sec;
                    benchmark::DoNotOptimize(sec);
                }
            });

        // access/half_fields: stamp + height + width + point_step + fields_len
        benchmark::RegisterBenchmark(
            bench::bench_name("PointCloud2", bench::op::access_half_fields, v.name).c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw  = ef::PointCloud2View::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto sec = vw->stamp().sec;
                    auto h   = vw->height();
                    auto w   = vw->width();
                    auto ps  = vw->point_step();
                    auto fl  = vw->fields_len();
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
                    auto vw  = ef::PointCloud2View::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto t   = vw->stamp();
                    auto sec = t.sec; auto ns = t.nanosec;
                    auto fid = vw->frame_id();
                    auto h   = vw->height();
                    auto w   = vw->width();
                    auto ps  = vw->point_step();
                    auto rs  = vw->row_step();
                    auto fl  = vw->fields_len();
                    auto be  = vw->is_bigendian();
                    auto isd = vw->is_dense();
                    auto dlen = vw->data().size();
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
                    auto vw   = ef::PointCloud2View::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto data = vw->data();
                    std::uint64_t sum = 0;
                    for (auto b : data) sum += b;
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
// In-place setter availability:
//   Header           : YES  - ros_header_set_stamp
//   Image            : YES  - ros_image_set_stamp
//   RadarCube        : YES  - ros_radar_cube_set_stamp
//   CompressedVideo  : YES  - ros_foxglove_compressed_video_set_stamp
//   PointCloud2      : YES  - ros_point_cloud2_set_stamp
//   Mask             : YES  - no stamp on Mask itself, but ros_mask_set_height/
//                             width/length/boxed update fixed-size fields in place

static void register_workflow_benchmarks() {
    // ========================================================
    // Header / default
    // ========================================================
    {
        // Pre-encoded wire bytes shared across sub/pub ops
        static const std::vector<std::uint8_t> hdr_wire = [] {
            bench::fixtures::HeaderFixture f;
            auto b = ef::HeaderBuilder::create();
            b->stamp(ef::Time{f.stamp_sec, f.stamp_nanos});
            (void)b->frame_id(f.frame_id.c_str());
            auto r = b->build();
            std::vector<std::uint8_t> v(r->data, r->data + r->size);
            ros_bytes_free(r->data, r->size);
            return v;
        }();

        // workflow/sub_modify_pub: borrow → read frame_id → rebuild with new stamp
        benchmark::RegisterBenchmark(
            bench::bench_name("Header", bench::op::workflow_smp, "default").c_str(),
            [](benchmark::State& state) {
                for (auto _ : state) {
                    auto v = ef::HeaderView::from_cdr({hdr_wire.data(), hdr_wire.size()});
                    auto fid = v->frame_id();               // caller-side metadata
                    auto b = ef::HeaderBuilder::create();
                    b->stamp(ef::Time{1234567891, 0});       // updated timestamp
                    (void)b->frame_id(fid.data());
                    auto r = b->build();
                    benchmark::DoNotOptimize(r);
                    ef_release(r);
                }
            });

        // workflow/pub_loop_rebuild: 1000-iter publisher, fresh build each time
        benchmark::RegisterBenchmark(
            bench::bench_name("Header", bench::op::workflow_pub_rebuild, "default").c_str(),
            [](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto b = ef::HeaderBuilder::create();
                        b->stamp(ef::Time{1234567890 + i, 0});
                        (void)b->frame_id("cam");
                        auto r = b->build();
                        benchmark::DoNotOptimize(r);
                        ef_release(r);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/pub_loop_inplace: build once, reuse buffer with in-place stamp setter
        benchmark::RegisterBenchmark(
            bench::bench_name("Header", bench::op::workflow_pub_inplace, "default").c_str(),
            [](benchmark::State& state) {
                // Build the initial buffer once outside the timed loop
                auto b = ef::HeaderBuilder::create();
                b->stamp(ef::Time{1234567890, 0});
                (void)b->frame_id("cam");
                auto r0 = b->build();
                std::vector<std::uint8_t> buf(r0->data, r0->data + r0->size);
                ros_bytes_free(r0->data, r0->size);
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        // In-place update of stamp; buf layout is stable
                        ros_header_set_stamp(buf.data(), buf.size(),
                                             1234567890 + i, static_cast<uint32_t>(i));
                        benchmark::DoNotOptimize(buf.data());
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/sub_loop: 1000-iter subscriber, read stamp.sec + frame_id only
        benchmark::RegisterBenchmark(
            bench::bench_name("Header", bench::op::workflow_sub, "default").c_str(),
            [](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto v   = ef::HeaderView::from_cdr({hdr_wire.data(), hdr_wire.size()});
                        auto sec = v->stamp().sec;
                        auto fid = v->frame_id();
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
        // Find the HD_rgb8 variant
        const bench::fixtures::ImageVariant* vp = nullptr;
        for (const auto& v : bench::fixtures::kImageVariants) {
            if (v.name == "HD_rgb8") { vp = &v; break; }
        }
        const auto& iv = *vp;

        std::size_t payload_size = bench::fixtures::image_payload_bytes(iv);
        auto payload_ptr = std::make_shared<std::vector<std::uint8_t>>(
            bench::make_payload(payload_size));

        auto wire_ptr = std::make_shared<std::vector<std::uint8_t>>([&]() {
            std::uint32_t step = iv.width * iv.step_bpp;
            auto r = ef::Image::encode(
                ef::Time{1234567890, 123456789}, "cam",
                iv.height, iv.width, std::string(iv.encoding), false, step,
                {payload_ptr->data(), payload_ptr->size()});
            if (!r) return std::vector<std::uint8_t>{};
            auto sp = r->as_cdr();
            return std::vector<std::uint8_t>(sp.data(), sp.data() + sp.size());
        }());

        // workflow/sub_modify_pub
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::workflow_smp, "HD_rgb8").c_str(),
            [wire_ptr, payload_ptr, iv](benchmark::State& state) {
                std::uint32_t step = iv.width * iv.step_bpp;
                for (auto _ : state) {
                    auto vw   = ef::ImageView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto enc  = vw->encoding();
                    auto r    = ef::Image::encode(
                        ef::Time{1234567891, 0}, "cam",
                        vw->height(), vw->width(),
                        std::string(enc), false, step,
                        {payload_ptr->data(), payload_ptr->size()});
                    benchmark::DoNotOptimize(r);
                }
            });

        // workflow/pub_loop_rebuild
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::workflow_pub_rebuild, "HD_rgb8").c_str(),
            [payload_ptr, iv](benchmark::State& state) {
                std::uint32_t step = iv.width * iv.step_bpp;
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto r = ef::Image::encode(
                            ef::Time{1234567890 + i, 0}, "cam",
                            iv.height, iv.width, std::string(iv.encoding), false, step,
                            {payload_ptr->data(), payload_ptr->size()});
                        benchmark::DoNotOptimize(r);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/pub_loop_inplace: ros_image_set_stamp updates stamp in-place
        benchmark::RegisterBenchmark(
            bench::bench_name("Image", bench::op::workflow_pub_inplace, "HD_rgb8").c_str(),
            [payload_ptr, iv, wire_ptr](benchmark::State& state) {
                // Start from a copy of the pre-encoded wire bytes
                std::vector<std::uint8_t> buf(*wire_ptr);
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        ros_image_set_stamp(buf.data(), buf.size(),
                                            1234567890 + i, static_cast<uint32_t>(i));
                        benchmark::DoNotOptimize(buf.data());
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
                        auto vw  = ef::ImageView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                        auto sec = vw->stamp().sec;
                        auto h   = vw->height();
                        auto w   = vw->width();
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
        auto shape_ptr = std::make_shared<std::array<std::uint16_t, 4>>(shape_arr);

        auto wire_ptr = std::make_shared<std::vector<std::uint8_t>>([&]() {
            auto b = ef::RadarCubeBuilder::create();
            if (!b) return std::vector<std::uint8_t>{};
            b->stamp(ef::Time{1234567890, 0});
            (void)b->frame_id("radar");
            b->timestamp(0);
            (void)b->shape({shape_arr.data(), shape_arr.size()});
            (void)b->cube({cube_ptr->data(), cube_ptr->size()});
            b->is_complex(false);
            auto r = b->build();
            if (!r) return std::vector<std::uint8_t>{};
            std::vector<std::uint8_t> out(r->data, r->data + r->size);
            ros_bytes_free(r->data, r->size);
            return out;
        }());

        // workflow/sub_modify_pub
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::workflow_smp, "DRVEGRD169_short").c_str(),
            [wire_ptr, cube_ptr, shape_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw = ef::RadarCubeView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto ts = vw->timestamp();
                    auto b  = ef::RadarCubeBuilder::create();
                    if (!b) { state.SkipWithError("create failed"); break; }
                    b->stamp(ef::Time{1234567891, 0});
                    (void)b->frame_id("radar");
                    b->timestamp(ts + 1);
                    (void)b->shape({shape_ptr->data(), shape_ptr->size()});
                    (void)b->cube({cube_ptr->data(), cube_ptr->size()});
                    b->is_complex(false);
                    auto r = b->build();
                    benchmark::DoNotOptimize(r);
                    ef_release(r);
                }
            });

        // workflow/pub_loop_rebuild
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::workflow_pub_rebuild, "DRVEGRD169_short").c_str(),
            [cube_ptr, shape_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto b = ef::RadarCubeBuilder::create();
                        if (!b) { state.SkipWithError("create failed"); break; }
                        b->stamp(ef::Time{1234567890 + i, 0});
                        (void)b->frame_id("radar");
                        b->timestamp(static_cast<uint64_t>(i));
                        (void)b->shape({shape_ptr->data(), shape_ptr->size()});
                        (void)b->cube({cube_ptr->data(), cube_ptr->size()});
                        b->is_complex(false);
                        auto r = b->build();
                        benchmark::DoNotOptimize(r);
                        ef_release(r);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/pub_loop_inplace: ros_radar_cube_set_stamp updates stamp in-place
        benchmark::RegisterBenchmark(
            bench::bench_name("RadarCube", bench::op::workflow_pub_inplace, "DRVEGRD169_short").c_str(),
            [wire_ptr](benchmark::State& state) {
                std::vector<std::uint8_t> buf(*wire_ptr);
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        ros_radar_cube_set_stamp(buf.data(), buf.size(),
                                                 1234567890 + i, static_cast<uint32_t>(i));
                        benchmark::DoNotOptimize(buf.data());
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
                        auto vw  = ef::RadarCubeView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                        auto sec = vw->stamp().sec;
                        auto ts  = vw->timestamp();
                        auto clen = vw->cube_len();
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
    // NOTE: Mask has no header/stamp field. pub_loop_inplace exercises the
    //       fixed-size in-place setters (ros_mask_set_length / set_boxed).
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

        auto wire_ptr = std::make_shared<std::vector<std::uint8_t>>([&]() {
            auto r = ef::Mask::encode(
                mv.height, mv.width,
                static_cast<std::uint32_t>(payload_size),
                "mono8",
                {payload_ptr->data(), payload_ptr->size()},
                false);
            if (!r) return std::vector<std::uint8_t>{};
            auto sp = r->as_cdr();
            return std::vector<std::uint8_t>(sp.data(), sp.data() + sp.size());
        }());

        // workflow/sub_modify_pub: decode → read dims → re-encode
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::workflow_smp, "640x640_8class").c_str(),
            [wire_ptr, payload_ptr, mv](benchmark::State& state) {
                std::size_t ps = static_cast<std::size_t>(mv.width) *
                                 static_cast<std::size_t>(mv.height);
                for (auto _ : state) {
                    auto vw  = ef::MaskView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto h   = vw->height();
                    auto w   = vw->width();
                    auto r   = ef::Mask::encode(
                        h, w,
                        static_cast<std::uint32_t>(ps),
                        "mono8",
                        {payload_ptr->data(), payload_ptr->size()},
                        false);
                    benchmark::DoNotOptimize(r);
                }
            });

        // workflow/pub_loop_rebuild
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::workflow_pub_rebuild, "640x640_8class").c_str(),
            [payload_ptr, mv](benchmark::State& state) {
                std::size_t ps = static_cast<std::size_t>(mv.width) *
                                 static_cast<std::size_t>(mv.height);
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto r = ef::Mask::encode(
                            mv.height, mv.width,
                            static_cast<std::uint32_t>(ps),
                            "mono8",
                            {payload_ptr->data(), payload_ptr->size()},
                            false);
                        benchmark::DoNotOptimize(r);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/pub_loop_inplace: ros_mask_set_length / set_boxed update fields
        // in-place on a single shared buffer (no stamp on Mask, but length/boxed
        // are the natural per-iteration mutables).
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::workflow_pub_inplace, "640x640_8class").c_str(),
            [mv, wire_ptr](benchmark::State& state) {
                // Start from a copy of the pre-encoded wire bytes
                std::vector<std::uint8_t> buf(*wire_ptr);
                std::uint32_t base_length = mv.width * mv.height;
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        ros_mask_set_length(buf.data(), buf.size(),
                                            base_length + static_cast<std::uint32_t>(i));
                        ros_mask_set_boxed(buf.data(), buf.size(),
                                           static_cast<std::uint8_t>(i & 1));
                        benchmark::DoNotOptimize(buf.data());
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/sub_loop
        benchmark::RegisterBenchmark(
            bench::bench_name("Mask", bench::op::workflow_sub, "640x640_8class").c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto vw  = ef::MaskView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                        auto h   = vw->height();
                        auto w   = vw->width();
                        auto enc = vw->encoding();
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

        auto wire_ptr = std::make_shared<std::vector<std::uint8_t>>([&]() {
            auto r = ef::CompressedVideo::encode(
                ef::Time{1234567890, 123456789}, "cam",
                {payload_ptr->data(), payload_ptr->size()}, "h264");
            if (!r) return std::vector<std::uint8_t>{};
            auto sp = r->as_cdr();
            return std::vector<std::uint8_t>(sp.data(), sp.data() + sp.size());
        }());

        // workflow/sub_modify_pub
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::workflow_smp, "100KB").c_str(),
            [wire_ptr, payload_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw  = ef::CompressedVideoView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto fmt = vw->format();
                    auto r   = ef::CompressedVideo::encode(
                        ef::Time{1234567891, 0}, "cam",
                        {payload_ptr->data(), payload_ptr->size()},
                        std::string(fmt));
                    benchmark::DoNotOptimize(r);
                }
            });

        // workflow/pub_loop_rebuild
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::workflow_pub_rebuild, "100KB").c_str(),
            [payload_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto r = ef::CompressedVideo::encode(
                            ef::Time{1234567890 + i, 0}, "cam",
                            {payload_ptr->data(), payload_ptr->size()}, "h264");
                        benchmark::DoNotOptimize(r);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/pub_loop_inplace: ros_foxglove_compressed_video_set_stamp
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::workflow_pub_inplace, "100KB").c_str(),
            [wire_ptr](benchmark::State& state) {
                std::vector<std::uint8_t> buf(*wire_ptr);
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        ros_foxglove_compressed_video_set_stamp(
                            buf.data(), buf.size(),
                            1234567890 + i, static_cast<uint32_t>(i));
                        benchmark::DoNotOptimize(buf.data());
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/sub_loop
        benchmark::RegisterBenchmark(
            bench::bench_name("CompressedVideo", bench::op::workflow_sub, "100KB").c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto vw  = ef::CompressedVideoView::from_cdr({wire_ptr->data(), wire_ptr->size()});
                        auto sec = vw->stamp().sec;
                        auto fmt = vw->format();
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

        static const ros_point_field_elem_t kFields[] = {
            {"x",         0,  7, 1},
            {"y",         4,  7, 1},
            {"z",         8,  7, 1},
            {"intensity", 12, 7, 1},
        };
        constexpr std::size_t kFieldCount = 4;

        std::size_t payload_size = static_cast<std::size_t>(pcv.num_points) *
                                   static_cast<std::size_t>(pcv.point_step);
        auto payload_ptr = std::make_shared<std::vector<std::uint8_t>>(
            bench::make_payload(payload_size));

        auto wire_ptr = std::make_shared<std::vector<std::uint8_t>>([&]() {
            auto b = ef::PointCloud2Builder::create();
            if (!b) return std::vector<std::uint8_t>{};
            b->stamp(ef::Time{1234567890, 123456789});
            (void)b->frame_id("lidar");
            b->height(1);
            b->width(pcv.num_points);
            (void)b->fields({kFields, kFieldCount});
            b->is_bigendian(false);
            b->point_step(pcv.point_step);
            b->row_step(pcv.num_points * pcv.point_step);
            (void)b->data({payload_ptr->data(), payload_ptr->size()});
            b->is_dense(true);
            auto r = b->build();
            if (!r) return std::vector<std::uint8_t>{};
            std::vector<std::uint8_t> out(r->data, r->data + r->size);
            ros_bytes_free(r->data, r->size);
            return out;
        }());

        // workflow/sub_modify_pub
        benchmark::RegisterBenchmark(
            bench::bench_name("PointCloud2", bench::op::workflow_smp, "robosense_e1r").c_str(),
            [wire_ptr, payload_ptr, pcv](benchmark::State& state) {
                for (auto _ : state) {
                    auto vw  = ef::PointCloud2View::from_cdr({wire_ptr->data(), wire_ptr->size()});
                    auto w   = vw->width();
                    auto b   = ef::PointCloud2Builder::create();
                    if (!b) { state.SkipWithError("create failed"); break; }
                    b->stamp(ef::Time{1234567891, 0});
                    (void)b->frame_id("lidar");
                    b->height(1);
                    b->width(w);
                    (void)b->fields({kFields, kFieldCount});
                    b->is_bigendian(false);
                    b->point_step(pcv.point_step);
                    b->row_step(w * pcv.point_step);
                    (void)b->data({payload_ptr->data(), payload_ptr->size()});
                    b->is_dense(true);
                    auto r = b->build();
                    benchmark::DoNotOptimize(r);
                    ef_release(r);
                }
            });

        // workflow/pub_loop_rebuild
        benchmark::RegisterBenchmark(
            bench::bench_name("PointCloud2", bench::op::workflow_pub_rebuild, "robosense_e1r").c_str(),
            [payload_ptr, pcv](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto b = ef::PointCloud2Builder::create();
                        if (!b) { state.SkipWithError("create failed"); break; }
                        b->stamp(ef::Time{1234567890 + i, 0});
                        (void)b->frame_id("lidar");
                        b->height(1);
                        b->width(pcv.num_points);
                        (void)b->fields({kFields, kFieldCount});
                        b->is_bigendian(false);
                        b->point_step(pcv.point_step);
                        b->row_step(pcv.num_points * pcv.point_step);
                        (void)b->data({payload_ptr->data(), payload_ptr->size()});
                        b->is_dense(true);
                        auto r = b->build();
                        benchmark::DoNotOptimize(r);
                        ef_release(r);
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/pub_loop_inplace: ros_point_cloud2_set_stamp
        benchmark::RegisterBenchmark(
            bench::bench_name("PointCloud2", bench::op::workflow_pub_inplace, "robosense_e1r").c_str(),
            [wire_ptr](benchmark::State& state) {
                std::vector<std::uint8_t> buf(*wire_ptr);
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        ros_point_cloud2_set_stamp(buf.data(), buf.size(),
                                                   1234567890 + i, static_cast<uint32_t>(i));
                        benchmark::DoNotOptimize(buf.data());
                    }
                }
                state.SetItemsProcessed(state.iterations() * 1000);
            });

        // workflow/sub_loop
        benchmark::RegisterBenchmark(
            bench::bench_name("PointCloud2", bench::op::workflow_sub, "robosense_e1r").c_str(),
            [wire_ptr](benchmark::State& state) {
                for (auto _ : state) {
                    for (int i = 0; i < 1000; ++i) {
                        auto vw  = ef::PointCloud2View::from_cdr({wire_ptr->data(), wire_ptr->size()});
                        auto sec = vw->stamp().sec;
                        auto w   = vw->width();
                        auto ps  = vw->point_step();
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
