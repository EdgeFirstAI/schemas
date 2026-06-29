/**
 * @file test_de2781_schemas.cpp
 * @brief C++ wrapper tests for the DE-2781 nav_msgs / sensor_msgs additions:
 *        MapMetaData, RelativeHumidity, TimeReference, GridCells,
 *        OccupancyGrid, and Path (with its forward iterator).
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Au-Zone Technologies. All Rights Reserved.
 *
 * Coverage:
 *   - Golden-decode + field checks for all six types (fixtures loaded at
 *     runtime from testdata/cdr/, mirroring the C suite).
 *   - Builder bytes == golden bytes for the construct-capable types.
 *   - Zero-copy pointer identity: OccupancyGrid::data() and the Path
 *     iterator's frame_id both borrow into the view's as_cdr() buffer.
 *   - The Path C++ iterator yields the same poses as pose(index) over the
 *     whole sequence (and is a single O(n) pass).
 */

#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <edgefirst/schemas.hpp>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace ef = edgefirst::schemas;

// All golden fixtures share this header stamp / frame_id (see
// scripts/generate_cdr_testdata.py).
static constexpr ef::Time kStamp{1234567890, 123456789};
static constexpr const char* kFrame = "test_frame";

static std::vector<std::uint8_t> load_fixture(const std::string& relpath) {
    std::FILE* f = std::fopen(relpath.c_str(), "rb");
    REQUIRE(f != nullptr);
    REQUIRE(std::fseek(f, 0, SEEK_END) == 0);
    long sz = std::ftell(f);
    REQUIRE(sz >= 0);
    REQUIRE(std::fseek(f, 0, SEEK_SET) == 0);
    std::vector<std::uint8_t> buf(static_cast<std::size_t>(sz));
    std::size_t got = std::fread(buf.data(), 1, buf.size(), f);
    std::fclose(f);
    REQUIRE(got == buf.size());
    return buf;
}

// Run a builder's allocating build() and return an owned copy of the bytes,
// releasing the C allocation.
template <typename BuildResult>
static std::vector<std::uint8_t> take_bytes(BuildResult&& r) {
    REQUIRE(r.has_value());
    std::vector<std::uint8_t> out(r->data, r->data + r->size);
    ros_bytes_free(r->data, r->size);
    return out;
}

// ── nav_msgs/MapMetaData (CdrFixed value class) ───────────────────────────

TEST_CASE("MapMetaData encode/decode roundtrip", "[de2781]") {
    ef::MapMetaData m{ef::Time{0, 0}, 0.05f, 200, 200,
                      ef::Pose{-5.0, -5.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
    auto sz = m.encoded_size();
    REQUIRE(sz.has_value());
    std::vector<std::uint8_t> buf(*sz);
    auto w = m.encode(ef::span<std::uint8_t>{buf.data(), buf.size()});
    REQUIRE(w.has_value());
    auto dec = ef::MapMetaData::decode(
        ef::span<const std::uint8_t>{buf.data(), *w});
    REQUIRE(dec.has_value());
    CHECK(dec->resolution == Approx(0.05f));
    CHECK(dec->width == 200u);
    CHECK(dec->height == 200u);
    CHECK(dec->origin.px == Approx(-5.0));
    CHECK(dec->origin.py == Approx(-5.0));
    CHECK(dec->origin.ow == Approx(1.0));
}

TEST_CASE("MapMetaData decodes golden fixture", "[de2781]") {
    auto bytes = load_fixture("testdata/cdr/nav_msgs/MapMetaData.cdr");
    auto m = ef::MapMetaData::decode(
        ef::span<const std::uint8_t>{bytes.data(), bytes.size()});
    REQUIRE(m.has_value());
    CHECK(m->map_load_time.sec == kStamp.sec);
    CHECK(m->map_load_time.nanosec == kStamp.nanosec);
    CHECK(m->resolution == Approx(0.05f));
    CHECK(m->width == 200u);
    CHECK(m->height == 200u);
    CHECK(m->origin.px == Approx(-5.0));
    CHECK(m->origin.py == Approx(-5.0));
    CHECK(m->origin.ow == Approx(1.0));
}

TEST_CASE("MapMetaData encode matches golden bytes", "[de2781]") {
    auto golden = load_fixture("testdata/cdr/nav_msgs/MapMetaData.cdr");
    ef::MapMetaData m{kStamp, 0.05f, 200, 200,
                      ef::Pose{-5.0, -5.0, 0.0, 0.0, 0.0, 0.0, 1.0}};
    std::vector<std::uint8_t> buf(golden.size());
    auto w = m.encode(ef::span<std::uint8_t>{buf.data(), buf.size()});
    REQUIRE(w.has_value());
    CHECK(*w == golden.size());
    CHECK(buf == golden);
}

// ── sensor_msgs/RelativeHumidity (view + builder) ─────────────────────────

TEST_CASE("RelativeHumidityView decodes golden fixture", "[de2781]") {
    auto bytes = load_fixture("testdata/cdr/sensor_msgs/RelativeHumidity.cdr");
    auto v = ef::RelativeHumidityView::from_cdr(
        ef::span<const std::uint8_t>{bytes.data(), bytes.size()});
    REQUIRE(v.has_value());
    CHECK(v->frame_id() == kFrame);
    CHECK(v->stamp().sec == kStamp.sec);
    CHECK(v->relative_humidity() == Approx(0.65));
    CHECK(v->variance() == Approx(0.001));
}

TEST_CASE("RelativeHumidityBuilder bytes match golden", "[de2781]") {
    auto golden = load_fixture("testdata/cdr/sensor_msgs/RelativeHumidity.cdr");
    auto b = ef::RelativeHumidityBuilder::create();
    REQUIRE(b.has_value());
    b->stamp(kStamp);
    REQUIRE(b->frame_id(kFrame).has_value());
    b->relative_humidity(0.65).variance(0.001);
    auto built = take_bytes(b->build());
    CHECK(built == golden);
}

// ── sensor_msgs/TimeReference (view + builder) ────────────────────────────

TEST_CASE("TimeReferenceView decodes golden fixture", "[de2781]") {
    auto bytes = load_fixture("testdata/cdr/sensor_msgs/TimeReference.cdr");
    auto v = ef::TimeReferenceView::from_cdr(
        ef::span<const std::uint8_t>{bytes.data(), bytes.size()});
    REQUIRE(v.has_value());
    CHECK(v->frame_id() == kFrame);
    CHECK(v->time_ref().sec == 1234567890);
    CHECK(v->time_ref().nanosec == 987654321u);
    CHECK(v->source() == "GPS_UTC");
}

TEST_CASE("TimeReferenceBuilder bytes match golden", "[de2781]") {
    auto golden = load_fixture("testdata/cdr/sensor_msgs/TimeReference.cdr");
    auto b = ef::TimeReferenceBuilder::create();
    REQUIRE(b.has_value());
    b->stamp(kStamp).time_ref(ef::Time{1234567890, 987654321});
    REQUIRE(b->frame_id(kFrame).has_value());
    REQUIRE(b->source("GPS_UTC").has_value());
    auto built = take_bytes(b->build());
    CHECK(built == golden);
}

// ── nav_msgs/GridCells (view + indexed cell accessor + builder) ───────────

TEST_CASE("GridCellsView decodes golden fixture", "[de2781]") {
    auto bytes = load_fixture("testdata/cdr/nav_msgs/GridCells.cdr");
    auto v = ef::GridCellsView::from_cdr(
        ef::span<const std::uint8_t>{bytes.data(), bytes.size()});
    REQUIRE(v.has_value());
    CHECK(v->frame_id() == kFrame);
    CHECK(v->cell_width() == Approx(0.5f));
    CHECK(v->cell_height() == Approx(0.5f));
    CHECK(v->size() == 3u);

    auto c0 = v->cell(0);
    REQUIRE(c0.has_value());
    CHECK(c0->x == Approx(1.0));
    CHECK(c0->y == Approx(2.0));

    auto c2 = v->cell(2);
    REQUIRE(c2.has_value());
    CHECK(c2->x == Approx(5.0));
    CHECK(c2->y == Approx(6.0));

    // Out-of-range yields an error rather than UB.
    auto bad = v->cell(99);
    CHECK_FALSE(bad.has_value());
}

TEST_CASE("GridCellsBuilder bytes match golden", "[de2781]") {
    auto golden = load_fixture("testdata/cdr/nav_msgs/GridCells.cdr");
    auto b = ef::GridCellsBuilder::create();
    REQUIRE(b.has_value());
    b->stamp(kStamp).cell_width(0.5f).cell_height(0.5f);
    REQUIRE(b->frame_id(kFrame).has_value());
    const double xyz[] = {1.0, 2.0, 0.0, 3.0, 4.0, 0.0, 5.0, 6.0, 0.0};
    REQUIRE(b->cells(ef::span<const double>{xyz, 9}).has_value());
    auto built = take_bytes(b->build());
    CHECK(built == golden);
}

// ── nav_msgs/OccupancyGrid (view + info + zero-copy data + builder) ───────

TEST_CASE("OccupancyGridView decodes golden fixture", "[de2781]") {
    auto bytes = load_fixture("testdata/cdr/nav_msgs/OccupancyGrid.cdr");
    auto v = ef::OccupancyGridView::from_cdr(
        ef::span<const std::uint8_t>{bytes.data(), bytes.size()});
    REQUIRE(v.has_value());
    CHECK(v->frame_id() == kFrame);

    auto info = v->info();
    CHECK(info.resolution == Approx(0.1f));
    CHECK(info.width == 4u);
    CHECK(info.height == 2u);

    auto data = v->data();
    REQUIRE(data.size() == 8u);
    CHECK(data[0] == 0);
    CHECK(data[1] == 50);
    CHECK(data[2] == 100);
    CHECK(data[3] == -1);
}

TEST_CASE("OccupancyGridView::data() is zero-copy into as_cdr()", "[de2781][zero_copy]") {
    auto bytes = load_fixture("testdata/cdr/nav_msgs/OccupancyGrid.cdr");
    auto v = ef::OccupancyGridView::from_cdr(
        ef::span<const std::uint8_t>{bytes.data(), bytes.size()});
    REQUIRE(v.has_value());

    auto cdr = v->as_cdr();
    auto data = v->data();
    REQUIRE(data.size() > 0);

    const auto* base = reinterpret_cast<const std::uint8_t*>(data.data());
    const auto* lo = cdr.data();
    const auto* hi = cdr.data() + cdr.size();
    // The occupancy bytes must lie inside the borrowed CDR buffer.
    CHECK(base >= lo);
    CHECK(base + data.size() <= hi);
}

TEST_CASE("OccupancyGridBuilder bytes match golden", "[de2781]") {
    auto golden = load_fixture("testdata/cdr/nav_msgs/OccupancyGrid.cdr");
    auto b = ef::OccupancyGridBuilder::create();
    REQUIRE(b.has_value());
    b->stamp(kStamp).info(ef::MapMetaData{
        kStamp, 0.1f, 4, 2, ef::Pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}});
    REQUIRE(b->frame_id(kFrame).has_value());
    const std::int8_t data[] = {0, 50, 100, -1, 25, 75, 0, 0};
    REQUIRE(b->data(ef::span<const std::int8_t>{data, 8}).has_value());
    auto built = take_bytes(b->build());
    CHECK(built == golden);
}

// ── nav_msgs/Path (view + iterator + builder) ─────────────────────────────

TEST_CASE("PathView decodes golden fixture", "[de2781]") {
    auto bytes = load_fixture("testdata/cdr/nav_msgs/Path.cdr");
    auto v = ef::PathView::from_cdr(
        ef::span<const std::uint8_t>{bytes.data(), bytes.size()});
    REQUIRE(v.has_value());
    CHECK(v->frame_id() == kFrame);
    CHECK(v->size() == 3u);

    auto p0 = v->pose(0);
    REQUIRE(p0.has_value());
    CHECK(p0->stamp.sec == 1);
    CHECK(p0->frame_id == "map");
    CHECK(p0->pose.px == Approx(1.0));
    CHECK(p0->pose.py == Approx(0.0));

    auto p2 = v->pose(2);
    REQUIRE(p2.has_value());
    CHECK(p2->stamp.sec == 3);
    CHECK(p2->pose.px == Approx(3.0));
    CHECK(p2->pose.oz == Approx(1.0));

    CHECK_FALSE(v->pose(99).has_value());
}

TEST_CASE("PathView iterator yields the same poses as pose(index)", "[de2781]") {
    auto bytes = load_fixture("testdata/cdr/nav_msgs/Path.cdr");
    auto v = ef::PathView::from_cdr(
        ef::span<const std::uint8_t>{bytes.data(), bytes.size()});
    REQUIRE(v.has_value());

    std::size_t idx = 0;
    for (const auto& ps : *v) {
        auto ref = v->pose(idx);
        REQUIRE(ref.has_value());
        CHECK(ps.stamp.sec == ref->stamp.sec);
        CHECK(ps.stamp.nanosec == ref->stamp.nanosec);
        CHECK(ps.frame_id == ref->frame_id);
        CHECK(ps.pose.px == Approx(ref->pose.px));
        CHECK(ps.pose.py == Approx(ref->pose.py));
        CHECK(ps.pose.pz == Approx(ref->pose.pz));
        CHECK(ps.pose.oz == Approx(ref->pose.oz));
        CHECK(ps.pose.ow == Approx(ref->pose.ow));
        ++idx;
    }
    CHECK(idx == v->size());
}

TEST_CASE("PathView iterator frame_id is zero-copy into as_cdr()", "[de2781][zero_copy]") {
    auto bytes = load_fixture("testdata/cdr/nav_msgs/Path.cdr");
    auto v = ef::PathView::from_cdr(
        ef::span<const std::uint8_t>{bytes.data(), bytes.size()});
    REQUIRE(v.has_value());

    auto cdr = v->as_cdr();
    const auto* lo = cdr.data();
    const auto* hi = cdr.data() + cdr.size();

    std::size_t count = 0;
    for (const auto& ps : *v) {
        REQUIRE_FALSE(ps.frame_id.empty());
        const auto* base = reinterpret_cast<const std::uint8_t*>(ps.frame_id.data());
        CHECK(base >= lo);
        CHECK(base + ps.frame_id.size() <= hi);
        ++count;
    }
    CHECK(count == 3u);
}

TEST_CASE("PathBuilder bytes match golden", "[de2781]") {
    auto golden = load_fixture("testdata/cdr/nav_msgs/Path.cdr");
    auto b = ef::PathBuilder::create();
    REQUIRE(b.has_value());
    b->stamp(kStamp);
    REQUIRE(b->frame_id(kFrame).has_value());
    REQUIRE(b->add_pose(ef::Time{1, 0}, "map",
                        ef::Pose{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}).has_value());
    REQUIRE(b->add_pose(ef::Time{2, 0}, "map",
                        ef::Pose{2.0, 1.0, 0.0, 0.0, 0.0, 0.707, 0.707}).has_value());
    REQUIRE(b->add_pose(ef::Time{3, 0}, "map",
                        ef::Pose{3.0, 2.0, 0.0, 0.0, 0.0, 1.0, 0.0}).has_value());
    auto built = take_bytes(b->build());
    CHECK(built == golden);
}

// ── Type-property checks ──────────────────────────────────────────────────

TEST_CASE("new view types are move-only", "[de2781][lifetime]") {
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<ef::RelativeHumidityView>);
    STATIC_REQUIRE(std::is_move_constructible_v<ef::RelativeHumidityView>);
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<ef::TimeReferenceView>);
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<ef::GridCellsView>);
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<ef::OccupancyGridView>);
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<ef::PathView>);
    STATIC_REQUIRE(std::is_move_constructible_v<ef::PathView>);
}

TEST_CASE("new builders are move-only", "[de2781][lifetime]") {
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<ef::RelativeHumidityBuilder>);
    STATIC_REQUIRE(std::is_move_constructible_v<ef::GridCellsBuilder>);
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<ef::PathBuilder>);
}

TEST_CASE("Path iterator is move-only input iterator", "[de2781][lifetime]") {
    using It = ef::PathView::iterator;
    STATIC_REQUIRE(std::is_same_v<It::iterator_category, std::input_iterator_tag>);
    STATIC_REQUIRE_FALSE(std::is_copy_constructible_v<It>);
    STATIC_REQUIRE(std::is_move_constructible_v<It>);
}

// MapMetaData is a plain copyable value type (CdrFixed).
TEST_CASE("MapMetaData is a copyable value type", "[de2781]") {
    STATIC_REQUIRE(std::is_copy_constructible_v<ef::MapMetaData>);
    STATIC_REQUIRE(std::is_trivially_copyable_v<ef::MapMetaData>);
}
