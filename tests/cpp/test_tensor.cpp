/**
 * @file test_tensor.cpp
 * @brief Tests for the C++ tensor message family wrappers.
 *
 * Covers TensorView, BorrowedTensorView, BorrowedTensorPlaneView, the two
 * byte-identical wrappers (TensorStampedView / CameraFrameView), and the
 * three builders.
 *
 * The contract under test is composition: a wrapper is a header plus `seq`
 * plus an embedded Tensor, and the embedded Tensor's bytes are
 * position-independent. Several cases assert that directly rather than only
 * round-tripping fields.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Au-Zone Technologies. All Rights Reserved.
 */

#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <edgefirst/schemas.hpp>

#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace ef = edgefirst::schemas;

// ============================================================================
// Helpers
// ============================================================================

static void free_released(ef::Released& r) {
    if (r.data) {
        ros_bytes_free(r.data, r.size);
        r.data = nullptr;
        r.size = 0;
    }
}

/// RAII wrapper so tests can early-return without leaking.
struct Bytes {
    ef::Released r{nullptr, 0};
    explicit Bytes(ef::Released rel) : r(rel) {}
    Bytes(const Bytes&)            = delete;
    Bytes& operator=(const Bytes&) = delete;
    ~Bytes() { free_released(r); }
    [[nodiscard]] ef::span<const std::uint8_t> span() const { return {r.data, r.size}; }
};

static const std::array<std::uint64_t, 2> SHAPE_NV12{480, 640};
static const std::array<std::int64_t, 2>  STRIDES_NV12{640, 1};
static const std::array<std::uint8_t, 4>  HANDLE_BYTES{0xDE, 0xAD, 0xBE, 0xEF};

/// Two referenced planes, as a single-allocation NV12 dma-buf frame carries.
static std::array<ros_tensor_plane_elem_t, 2> nv12_planes() {
    std::array<ros_tensor_plane_elem_t, 2> p{};
    p[0].handle           = 7;
    p[0].offset           = 0;
    p[0].stride           = 640;
    p[0].size             = 640 * 480;
    p[0].used             = 640 * 480;
    p[0].handle_bytes     = HANDLE_BYTES.data();
    p[0].handle_bytes_len = HANDLE_BYTES.size();
    p[1].handle = 7;
    p[1].offset = 640 * 480;
    p[1].stride = 640;
    p[1].size   = 640 * 480 / 2;
    p[1].used   = 640 * 480 / 2;
    return p;
}

/// Configure a TensorBuilder as an unquantized NV12 camera frame.
static void configure_nv12(ef::TensorBuilder& tb,
                           const std::array<ros_tensor_plane_elem_t, 2>& planes) {
    tb.storage_kind(2).pid(4242).fence_fd(-1).dtype(1).quant_axis(-2);
    REQUIRE(tb.shape({SHAPE_NV12.data(), SHAPE_NV12.size()}).has_value());
    REQUIRE(tb.strides({STRIDES_NV12.data(), STRIDES_NV12.size()}).has_value());
    REQUIRE(tb.format("NV12").has_value());
    REQUIRE(tb.color_space("bt709").has_value());
    REQUIRE(tb.color_transfer("bt709").has_value());
    REQUIRE(tb.color_encoding("bt709").has_value());
    REQUIRE(tb.color_range("limited").has_value());
    REQUIRE(tb.planes({planes.data(), planes.size()}).has_value());
}

// ============================================================================
// Tensor — standalone
// ============================================================================

TEST_CASE("TensorBuilder round-trips every field", "[tensor]") {
    auto tb = ef::TensorBuilder::create();
    REQUIRE(tb.has_value());
    auto planes = nv12_planes();
    configure_nv12(*tb, planes);

    auto built = tb->build();
    REQUIRE(built.has_value());
    Bytes bytes{*built};

    auto t = ef::TensorView::from_cdr(bytes.span());
    REQUIRE(t.has_value());

    CHECK(t->storage_kind() == 2);
    CHECK(t->pid() == 4242);
    CHECK(t->fence_fd() == -1);
    CHECK(t->dtype() == 1);
    CHECK(t->quant_axis() == -2);
    CHECK(t->format() == "NV12");
    CHECK(t->color_space() == "bt709");
    CHECK(t->color_range() == "limited");
    CHECK(t->num_planes() == 2);

    CHECK(t->shape_len() == 2);
    REQUIRE(t->shape() == std::vector<std::uint64_t>{480, 640});
    REQUIRE(t->strides() == std::vector<std::int64_t>{640, 1});

    auto d0 = t->shape_at(0);
    REQUIRE(d0.has_value());
    CHECK(*d0 == 480);

    // Out of range is an error, not a silent zero.
    CHECK_FALSE(t->shape_at(2).has_value());

    // Unquantized carries no scales.
    CHECK(t->quant_scales().empty());
}

TEST_CASE("Plane iteration borrows and yields correct fields", "[tensor][planes]") {
    auto tb = ef::TensorBuilder::create();
    REQUIRE(tb.has_value());
    auto planes = nv12_planes();
    configure_nv12(*tb, planes);

    auto built = tb->build();
    REQUIRE(built.has_value());
    Bytes bytes{*built};
    auto t = ef::TensorView::from_cdr(bytes.span());
    REQUIRE(t.has_value());

    auto range = t->planes();
    REQUIRE(range.size() == 2);
    REQUIRE_FALSE(range.empty());

    std::vector<std::uint64_t> offsets;
    for (auto p : range) {
        offsets.push_back(p.offset());
        CHECK_FALSE(p.is_inline());
        CHECK(p.handle() == 7);
        CHECK(p.stride() == 640);
        CHECK(p.used() <= p.size());
        // A referenced plane carries no inline bytes.
        CHECK(p.data().empty());
    }
    REQUIRE(offsets == std::vector<std::uint64_t>{0, 640 * 480});

    // handle_bytes are borrowed from the CDR buffer, not copied.
    auto first = *range.begin();
    auto hb = first.handle_bytes();
    REQUIRE(hb.size() == HANDLE_BYTES.size());
    CHECK(std::memcmp(hb.data(), HANDLE_BYTES.data(), hb.size()) == 0);
    const auto* buf_begin = bytes.span().data();
    const auto* buf_end   = buf_begin + bytes.span().size();
    CHECK(hb.data() >= buf_begin);
    CHECK(hb.data() < buf_end);
}

TEST_CASE("Inline plane round-trips its payload", "[tensor][planes]") {
    const std::array<std::uint8_t, 8> payload{1, 2, 3, 4, 5, 6, 7, 8};
    ros_tensor_plane_elem_t plane{};
    plane.handle   = -1;
    plane.size     = payload.size();
    plane.used     = payload.size();
    plane.data     = payload.data();
    plane.data_len = payload.size();

    auto tb = ef::TensorBuilder::create();
    REQUIRE(tb.has_value());
    tb->dtype(1);
    REQUIRE(tb->planes({&plane, 1}).has_value());

    auto built = tb->build();
    REQUIRE(built.has_value());
    Bytes bytes{*built};
    auto t = ef::TensorView::from_cdr(bytes.span());
    REQUIRE(t.has_value());

    auto p = *t->planes().begin();
    CHECK(p.is_inline());
    CHECK(p.handle() == -1);
    auto d = p.data();
    REQUIRE(d.size() == payload.size());
    CHECK(std::memcmp(d.data(), payload.data(), d.size()) == 0);
}

// ============================================================================
// Validation
// ============================================================================

TEST_CASE("Builder rejects incoherent plane sets", "[tensor][validation]") {
    SECTION("inline plane whose size disagrees with its data") {
        const std::array<std::uint8_t, 8> payload{};
        ros_tensor_plane_elem_t plane{};
        plane.handle   = -1;
        plane.size     = 99;  // lies
        plane.data     = payload.data();
        plane.data_len = payload.size();

        auto tb = ef::TensorBuilder::create();
        REQUIRE(tb.has_value());
        REQUIRE(tb->planes({&plane, 1}).has_value());
        CHECK_FALSE(tb->build().has_value());
    }

    SECTION("mixed transport modes") {
        const std::array<std::uint8_t, 4> payload{};
        std::array<ros_tensor_plane_elem_t, 2> planes{};
        planes[0].handle = 3;  // referenced
        planes[0].size   = 16;
        planes[1].handle = -1;  // inline — incoherent with plane 0
        planes[1].size     = payload.size();
        planes[1].data     = payload.data();
        planes[1].data_len = payload.size();

        auto tb = ef::TensorBuilder::create();
        REQUIRE(tb.has_value());
        REQUIRE(tb->planes({planes.data(), planes.size()}).has_value());
        CHECK_FALSE(tb->build().has_value());
    }

    SECTION("used exceeding size") {
        ros_tensor_plane_elem_t plane{};
        plane.handle = 3;
        plane.size   = 16;
        plane.used   = 17;

        auto tb = ef::TensorBuilder::create();
        REQUIRE(tb.has_value());
        REQUIRE(tb->planes({&plane, 1}).has_value());
        CHECK_FALSE(tb->build().has_value());
    }
}

TEST_CASE("quant_axis selects the required scale shape", "[tensor][validation]") {
    SECTION("per-tensor: exactly one scale") {
        const std::array<std::uint64_t, 1> shape{4};
        const std::array<float, 1> scale{0.125f};
        const std::array<std::int32_t, 1> zero{7};

        auto tb = ef::TensorBuilder::create();
        REQUIRE(tb.has_value());
        tb->dtype(3).quant_axis(-1);
        REQUIRE(tb->shape({shape.data(), shape.size()}).has_value());
        REQUIRE(tb->quant_scales({scale.data(), scale.size()}).has_value());
        REQUIRE(tb->quant_zero_points({zero.data(), zero.size()}).has_value());

        auto built = tb->build();
        REQUIRE(built.has_value());
        Bytes bytes{*built};
        auto t = ef::TensorView::from_cdr(bytes.span());
        REQUIRE(t.has_value());
        CHECK(t->quant_axis() == -1);
        REQUIRE(t->quant_scales().size() == 1);
        CHECK(t->quant_scales()[0] == 0.125f);
        REQUIRE(t->quant_zero_points().size() == 1);
        CHECK(t->quant_zero_points()[0] == 7);
    }

    SECTION("per-axis: exactly shape[axis] scales") {
        const std::array<std::uint64_t, 2> shape{3, 8};
        const std::array<float, 3> scales{0.5f, 0.25f, 0.125f};

        auto tb = ef::TensorBuilder::create();
        REQUIRE(tb.has_value());
        tb->quant_axis(0);
        REQUIRE(tb->shape({shape.data(), shape.size()}).has_value());
        REQUIRE(tb->quant_scales({scales.data(), scales.size()}).has_value());
        CHECK(tb->build().has_value());
    }

    SECTION("per-axis with the wrong count is rejected") {
        const std::array<std::uint64_t, 2> shape{3, 8};
        const std::array<float, 2> scales{0.5f, 0.25f};  // should be 3

        auto tb = ef::TensorBuilder::create();
        REQUIRE(tb.has_value());
        tb->quant_axis(0);
        REQUIRE(tb->shape({shape.data(), shape.size()}).has_value());
        REQUIRE(tb->quant_scales({scales.data(), scales.size()}).has_value());
        CHECK_FALSE(tb->build().has_value());
    }

    SECTION("unquantized with scales is rejected") {
        const std::array<float, 1> scale{0.5f};
        auto tb = ef::TensorBuilder::create();
        REQUIRE(tb.has_value());
        tb->quant_axis(-2);
        REQUIRE(tb->quant_scales({scale.data(), scale.size()}).has_value());
        CHECK_FALSE(tb->build().has_value());
    }
}

TEST_CASE("strides rank must match shape rank", "[tensor][validation]") {
    const std::array<std::uint64_t, 2> shape{4, 4};
    const std::array<std::int64_t, 1> strides{4};

    auto tb = ef::TensorBuilder::create();
    REQUIRE(tb.has_value());
    REQUIRE(tb->shape({shape.data(), shape.size()}).has_value());
    REQUIRE(tb->strides({strides.data(), strides.size()}).has_value());
    CHECK_FALSE(tb->build().has_value());
}

// ============================================================================
// Wrappers
// ============================================================================

TEST_CASE("CameraFrame round-trips header and payload", "[tensor][wrapper]") {
    auto tb = ef::TensorBuilder::create();
    REQUIRE(tb.has_value());
    auto planes = nv12_planes();
    configure_nv12(*tb, planes);

    auto fb = ef::CameraFrameBuilder::create();
    REQUIRE(fb.has_value());
    fb->stamp({1234567890, 123456789}).seq(99);
    REQUIRE(fb->frame_id("camera_0").has_value());
    REQUIRE(fb->tensor(*tb).has_value());

    auto built = fb->build();
    REQUIRE(built.has_value());
    Bytes bytes{*built};

    auto f = ef::CameraFrameView::from_cdr(bytes.span());
    REQUIRE(f.has_value());
    CHECK(f->stamp().sec == 1234567890);
    CHECK(f->stamp().nanosec == 123456789);
    CHECK(f->frame_id() == "camera_0");
    CHECK(f->seq() == 99);

    auto t = f->tensor();
    CHECK(t.format() == "NV12");
    CHECK(t.num_planes() == 2);
    CHECK(t.pid() == 4242);
    CHECK((*t.planes().begin()).handle() == 7);

    // as_cdr() exposes the whole message and is the buffer we passed in.
    CHECK(f->as_cdr().size() == bytes.span().size());
}

TEST_CASE("A wrapper builder with no tensor payload fails", "[tensor][wrapper]") {
    auto fb = ef::CameraFrameBuilder::create();
    REQUIRE(fb.has_value());
    fb->seq(1);
    CHECK_FALSE(fb->build().has_value());
}

/*
 * Position independence: the SAME tensor, in either wrapper, with frame_id
 * strings of different length, must yield byte-identical tensor bytes. This
 * is what `seq` (a uint64) buys — it forces the embedded tensor to an
 * 8-aligned offset no matter how long frame_id is.
 */
TEST_CASE("Tensor bytes are identical across wrappers and frame_ids",
          "[tensor][wrapper][position-independence]") {
    auto tb = ef::TensorBuilder::create();
    REQUIRE(tb.has_value());
    auto planes = nv12_planes();
    configure_nv12(*tb, planes);

    const std::array<const char*, 3> ids{"a", "camera_0", "a_very_long_frame_identifier_x"};
    std::vector<std::uint8_t> reference;

    for (const char* id : ids) {
        auto fb = ef::CameraFrameBuilder::create();
        REQUIRE(fb.has_value());
        REQUIRE(fb->frame_id(id).has_value());
        fb->seq(5);
        REQUIRE(fb->tensor(*tb).has_value());
        auto fbuilt = fb->build();
        REQUIRE(fbuilt.has_value());
        Bytes fbytes{*fbuilt};
        auto f = ef::CameraFrameView::from_cdr(fbytes.span());
        REQUIRE(f.has_value());
        auto ft = f->tensor().tensor_bytes();

        auto sb = ef::TensorStampedBuilder::create();
        REQUIRE(sb.has_value());
        REQUIRE(sb->frame_id(id).has_value());
        sb->seq(5);
        REQUIRE(sb->tensor(*tb).has_value());
        auto sbuilt = sb->build();
        REQUIRE(sbuilt.has_value());
        Bytes sbytes{*sbuilt};
        auto s = ef::TensorStampedView::from_cdr(sbytes.span());
        REQUIRE(s.has_value());
        auto st = s->tensor().tensor_bytes();

        // Wrapper to wrapper...
        REQUIRE(ft.size() == st.size());
        CHECK(std::memcmp(ft.data(), st.data(), ft.size()) == 0);

        // ...and across every frame_id length.
        if (reference.empty()) {
            reference.assign(ft.data(), ft.data() + ft.size());
        } else {
            REQUIRE(ft.size() == reference.size());
            CHECK(std::memcmp(ft.data(), reference.data(), ft.size()) == 0);
        }
    }
}

TEST_CASE("An embedded tensor re-parses standalone", "[tensor][wrapper]") {
    auto tb = ef::TensorBuilder::create();
    REQUIRE(tb.has_value());
    auto planes = nv12_planes();
    configure_nv12(*tb, planes);

    auto fb = ef::CameraFrameBuilder::create();
    REQUIRE(fb.has_value());
    REQUIRE(fb->frame_id("camera_0").has_value());
    REQUIRE(fb->tensor(*tb).has_value());
    auto built = fb->build();
    REQUIRE(built.has_value());
    Bytes bytes{*built};

    auto f = ef::CameraFrameView::from_cdr(bytes.span());
    REQUIRE(f.has_value());

    auto standalone = f->tensor().to_standalone_cdr();
    REQUIRE(standalone.has_value());
    Bytes sa{*standalone};

    auto t = ef::TensorView::from_cdr(sa.span());
    REQUIRE(t.has_value());
    CHECK(t->format() == "NV12");
    CHECK(t->num_planes() == 2);
    CHECK(t->pid() == 4242);

    // Byte-identical to encoding the same tensor from scratch.
    auto direct = tb->build();
    REQUIRE(direct.has_value());
    Bytes d{*direct};
    REQUIRE(sa.span().size() == d.span().size());
    CHECK(std::memcmp(sa.span().data(), d.span().data(), sa.span().size()) == 0);
}

// ============================================================================
// Encoding into a caller buffer
// ============================================================================

TEST_CASE("encode_into reports short buffers and matches build()", "[tensor]") {
    auto tb = ef::TensorBuilder::create();
    REQUIRE(tb.has_value());
    auto planes = nv12_planes();
    configure_nv12(*tb, planes);

    std::array<std::uint8_t, 8> small{};
    CHECK_FALSE(tb->encode_into({small.data(), small.size()}).has_value());

    auto built = tb->build();
    REQUIRE(built.has_value());
    Bytes bytes{*built};

    std::vector<std::uint8_t> big(bytes.span().size());
    auto n = tb->encode_into({big.data(), big.size()});
    REQUIRE(n.has_value());
    CHECK(*n == bytes.span().size());
    CHECK(std::memcmp(big.data(), bytes.span().data(), *n) == 0);
}

// ============================================================================
// Handle lifetime
// ============================================================================

TEST_CASE("Views are move-only and release exactly once", "[tensor][lifetime]") {
    static_assert(!std::is_copy_constructible_v<ef::TensorView>);
    static_assert(!std::is_copy_assignable_v<ef::TensorView>);
    static_assert(std::is_move_constructible_v<ef::TensorView>);
    static_assert(!std::is_copy_constructible_v<ef::CameraFrameView>);
    static_assert(std::is_move_constructible_v<ef::CameraFrameView>);

    // Borrowed children must not be storable past their parent.
    static_assert(!std::is_default_constructible_v<ef::BorrowedTensorView>);
    static_assert(!std::is_copy_assignable_v<ef::BorrowedTensorView>);
    static_assert(!std::is_default_constructible_v<ef::BorrowedTensorPlaneView>);
    static_assert(!std::is_copy_assignable_v<ef::BorrowedTensorPlaneView>);

    auto tb = ef::TensorBuilder::create();
    REQUIRE(tb.has_value());
    auto planes = nv12_planes();
    configure_nv12(*tb, planes);
    auto built = tb->build();
    REQUIRE(built.has_value());
    Bytes bytes{*built};

    auto t = ef::TensorView::from_cdr(bytes.span());
    REQUIRE(t.has_value());
    auto moved = std::move(*t);
    CHECK(moved.num_planes() == 2);
    // The moved-from view is inert; its destructor must not double-free.
}

TEST_CASE("from_cdr rejects malformed input", "[tensor]") {
    const std::array<std::uint8_t, 4> bad{0xDE, 0xAD, 0xBE, 0xEF};
    CHECK_FALSE(ef::TensorView::from_cdr({bad.data(), bad.size()}).has_value());
    CHECK_FALSE(ef::CameraFrameView::from_cdr({bad.data(), bad.size()}).has_value());
    CHECK_FALSE(ef::TensorStampedView::from_cdr({bad.data(), bad.size()}).has_value());
}
