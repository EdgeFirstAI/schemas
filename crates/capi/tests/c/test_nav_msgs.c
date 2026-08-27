/**
 * @file test_nav_msgs.c
 * @brief Criterion tests for nav_msgs types (TOP2-770)
 *
 * Buffer-backed: Odometry
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "edgefirst/schemas.h"

static uint8_t *load_fixture_nm(const char *relpath, size_t *out_len) {
    FILE *f = fopen(relpath, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0) { fclose(f); return NULL; }
    uint8_t *buf = (uint8_t *) malloc((size_t) sz);
    size_t got = fread(buf, 1, (size_t) sz, f);
    fclose(f);
    if (got != (size_t) sz) { free(buf); return NULL; }
    *out_len = got;
    return buf;
}

Test(nav_msgs, odometry_from_golden_fixture) {
    size_t len = 0;
    uint8_t *b = load_fixture_nm("testdata/cdr/nav_msgs/Odometry.cdr", &len);
    cr_assert_not_null(b, "failed to load Odometry fixture");
    nav_msgs_odometry_t *v = nav_msgs_odometry_from_cdr(b, len);
    cr_assert_not_null(v);

    cr_assert_str_eq(nav_msgs_odometry_get_frame_id(v), "test_frame");
    cr_assert_str_eq(nav_msgs_odometry_get_child_frame_id(v), "base_link");

    double px = 0, py = 0, pz = 0, ox = 0, oy = 0, oz = 0, ow = 0;
    nav_msgs_odometry_get_pose(v, &px, &py, &pz, &ox, &oy, &oz, &ow);
    cr_assert_float_eq(px, 1.5, 1e-12);
    cr_assert_float_eq(pz, 3.0, 1e-12);
    cr_assert_float_eq(ow, 1.0, 1e-12);

    double pose_cov[36] = {0};
    nav_msgs_odometry_get_pose_covariance(v, pose_cov);
    cr_assert_float_eq(pose_cov[0], 0.1, 1e-12);
    cr_assert_float_eq(pose_cov[1], 0.01, 1e-12);

    double lx = 0, ly = 0, lz = 0, ax = 0, ay = 0, az = 0;
    nav_msgs_odometry_get_twist(v, &lx, &ly, &lz, &ax, &ay, &az);
    cr_assert_float_eq(lx, 1.0, 1e-12);
    cr_assert_float_eq(az, 0.3, 1e-12);

    double twist_cov[36] = {0};
    nav_msgs_odometry_get_twist_covariance(v, twist_cov);
    cr_assert_float_eq(twist_cov[0], 0.02, 1e-12);
    cr_assert_float_eq(twist_cov[7], 0.001, 1e-12);

    nav_msgs_odometry_free(v);
    free(b);
}

Test(nav_msgs, odometry_from_cdr_null) {
    errno = 0;
    cr_assert_null(nav_msgs_odometry_from_cdr(NULL, 100));
    cr_assert_eq(errno, EINVAL);
}

Test(nav_msgs, odometry_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    cr_assert_null(nav_msgs_odometry_from_cdr(bad, sizeof(bad)));
    cr_assert_eq(errno, EBADMSG);
}

Test(nav_msgs, odometry_free_null) {
    nav_msgs_odometry_free(NULL);
}

Test(nav_msgs, odometry_getters_null) {
    cr_assert_eq(nav_msgs_odometry_get_stamp_sec(NULL), 0);
    cr_assert_eq(nav_msgs_odometry_get_stamp_nanosec(NULL), 0);
    cr_assert_null(nav_msgs_odometry_get_frame_id(NULL));
    cr_assert_null(nav_msgs_odometry_get_child_frame_id(NULL));
    /* Pointer-out getters must be safe on NULL view. */
    nav_msgs_odometry_get_pose(NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    nav_msgs_odometry_get_pose_covariance(NULL, NULL);
    nav_msgs_odometry_get_twist(NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    nav_msgs_odometry_get_twist_covariance(NULL, NULL);
}

// ============================================================================
// MapMetaData Tests (CdrFixed)
// ============================================================================

Test(nav_msgs, map_meta_data_encode_decode_roundtrip) {
    uint8_t buf[256];
    size_t written = 0;
    int ret = nav_msgs_map_meta_data_encode(buf, sizeof(buf), &written,
        0, 0,        /* map_load_time */
        0.05f,       /* resolution */
        200, 200,    /* width, height */
        -5.0, -5.0, 0.0,  /* origin position */
        0.0, 0.0, 0.0, 1.0 /* origin orientation */
    );
    cr_assert_eq(ret, 0);
    cr_assert_gt(written, 0);

    float res = 0.0f;
    uint32_t w = 0, h = 0;
    double px = 0, py = 0, ow = 0;
    ret = nav_msgs_map_meta_data_decode(buf, written,
        NULL, NULL, &res, &w, &h, &px, &py, NULL, NULL, NULL, NULL, &ow);
    cr_assert_eq(ret, 0);
    cr_assert_float_eq(res, 0.05f, 1e-6f);
    cr_assert_eq(w, 200u);
    cr_assert_eq(h, 200u);
    cr_assert_float_eq(px, -5.0, 1e-12);
    cr_assert_float_eq(py, -5.0, 1e-12);
    cr_assert_float_eq(ow, 1.0, 1e-12);
}

Test(nav_msgs, map_meta_data_encode_size_query) {
    size_t written = 0;
    int ret = nav_msgs_map_meta_data_encode(NULL, 0, &written,
        0, 0, 0.0f, 0, 0, 0, 0, 0, 0, 0, 0, 1.0);
    cr_assert_eq(ret, 0);
    cr_assert_gt(written, 0);
}

Test(nav_msgs, map_meta_data_decode_null_data) {
    float res;
    errno = 0;
    int ret = nav_msgs_map_meta_data_decode(NULL, 100,
        NULL, NULL, &res, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

Test(nav_msgs, map_meta_data_from_golden_fixture) {
    size_t len = 0;
    uint8_t *b = load_fixture_nm("testdata/cdr/nav_msgs/MapMetaData.cdr", &len);
    cr_assert_not_null(b, "failed to load MapMetaData fixture");

    float res = 0;
    uint32_t w = 0, h = 0;
    double px = 0, py = 0, ow = 0;
    int ret = nav_msgs_map_meta_data_decode(b, len,
        NULL, NULL, &res, &w, &h, &px, &py, NULL, NULL, NULL, NULL, &ow);
    cr_assert_eq(ret, 0);
    cr_assert_float_eq(res, 0.05f, 1e-6f);
    cr_assert_eq(w, 200u);
    cr_assert_eq(h, 200u);
    cr_assert_float_eq(px, -5.0, 1e-12);
    cr_assert_float_eq(py, -5.0, 1e-12);
    cr_assert_float_eq(ow, 1.0, 1e-12);
    free(b);
}

// ============================================================================
// GridCells Tests (buffer-backed)
// ============================================================================

Test(nav_msgs, grid_cells_from_golden_fixture) {
    size_t len = 0;
    uint8_t *b = load_fixture_nm("testdata/cdr/nav_msgs/GridCells.cdr", &len);
    cr_assert_not_null(b, "failed to load GridCells fixture");
    nav_msgs_grid_cells_t *v = nav_msgs_grid_cells_from_cdr(b, len);
    cr_assert_not_null(v);

    cr_assert_str_eq(nav_msgs_grid_cells_get_frame_id(v), "test_frame");
    cr_assert_float_eq(nav_msgs_grid_cells_get_cell_width(v), 0.5f, 1e-6f);
    cr_assert_float_eq(nav_msgs_grid_cells_get_cell_height(v), 0.5f, 1e-6f);
    cr_assert_eq(nav_msgs_grid_cells_get_len(v), 3u);

    double x = 0, y = 0, z = 0;
    cr_assert_eq(nav_msgs_grid_cells_get_cell(v, 0, &x, &y, &z), 0);
    cr_assert_float_eq(x, 1.0, 1e-12);
    cr_assert_float_eq(y, 2.0, 1e-12);

    cr_assert_eq(nav_msgs_grid_cells_get_cell(v, 2, &x, &y, &z), 0);
    cr_assert_float_eq(x, 5.0, 1e-12);
    cr_assert_float_eq(y, 6.0, 1e-12);

    nav_msgs_grid_cells_free(v);
    free(b);
}

Test(nav_msgs, grid_cells_from_cdr_null) {
    errno = 0;
    cr_assert_null(nav_msgs_grid_cells_from_cdr(NULL, 100));
    cr_assert_eq(errno, EINVAL);
}

Test(nav_msgs, grid_cells_from_cdr_invalid) {
    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    cr_assert_null(nav_msgs_grid_cells_from_cdr(bad, sizeof(bad)));
    cr_assert_eq(errno, EBADMSG);
}

Test(nav_msgs, grid_cells_free_null) {
    nav_msgs_grid_cells_free(NULL);
}

Test(nav_msgs, grid_cells_getters_null) {
    cr_assert_eq(nav_msgs_grid_cells_get_stamp_sec(NULL), 0);
    cr_assert_eq(nav_msgs_grid_cells_get_stamp_nanosec(NULL), 0);
    cr_assert_null(nav_msgs_grid_cells_get_frame_id(NULL));
    cr_assert_float_eq(nav_msgs_grid_cells_get_cell_width(NULL), 0.0f, 1e-6f);
    cr_assert_eq(nav_msgs_grid_cells_get_len(NULL), 0u);
    cr_assert_eq(nav_msgs_grid_cells_get_cell(NULL, 0, NULL, NULL, NULL), -1);
}

Test(nav_msgs, grid_cells_cell_out_of_range) {
    size_t len = 0;
    uint8_t *b = load_fixture_nm("testdata/cdr/nav_msgs/GridCells.cdr", &len);
    cr_assert_not_null(b);
    nav_msgs_grid_cells_t *v = nav_msgs_grid_cells_from_cdr(b, len);
    cr_assert_not_null(v);
    errno = 0;
    cr_assert_eq(nav_msgs_grid_cells_get_cell(v, 99, NULL, NULL, NULL), -1);
    cr_assert_eq(errno, EINVAL);
    nav_msgs_grid_cells_free(v);
    free(b);
}

// ============================================================================
// OccupancyGrid Tests (buffer-backed)
// ============================================================================

Test(nav_msgs, occupancy_grid_from_golden_fixture) {
    size_t len = 0;
    uint8_t *b = load_fixture_nm("testdata/cdr/nav_msgs/OccupancyGrid.cdr", &len);
    cr_assert_not_null(b, "failed to load OccupancyGrid fixture");
    nav_msgs_occupancy_grid_t *v = nav_msgs_occupancy_grid_from_cdr(b, len);
    cr_assert_not_null(v);

    cr_assert_str_eq(nav_msgs_occupancy_grid_get_frame_id(v), "test_frame");

    float res = 0;
    uint32_t w = 0, h = 0;
    nav_msgs_occupancy_grid_get_info(v, NULL, NULL, &res, &w, &h,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL);
    cr_assert_float_eq(res, 0.1f, 1e-6f);
    cr_assert_eq(w, 4u);
    cr_assert_eq(h, 2u);

    cr_assert_eq(nav_msgs_occupancy_grid_get_data_len(v), 8u);
    const int8_t *data = nav_msgs_occupancy_grid_get_data(v);
    cr_assert_not_null(data);
    cr_assert_eq(data[0], 0);
    cr_assert_eq(data[1], 50);
    cr_assert_eq(data[2], 100);
    cr_assert_eq(data[3], -1);

    nav_msgs_occupancy_grid_free(v);
    free(b);
}

Test(nav_msgs, occupancy_grid_from_cdr_null) {
    errno = 0;
    cr_assert_null(nav_msgs_occupancy_grid_from_cdr(NULL, 100));
    cr_assert_eq(errno, EINVAL);
}

Test(nav_msgs, occupancy_grid_free_null) {
    nav_msgs_occupancy_grid_free(NULL);
}

Test(nav_msgs, occupancy_grid_getters_null) {
    cr_assert_eq(nav_msgs_occupancy_grid_get_stamp_sec(NULL), 0);
    cr_assert_null(nav_msgs_occupancy_grid_get_frame_id(NULL));
    cr_assert_eq(nav_msgs_occupancy_grid_get_data_len(NULL), 0u);
    cr_assert_null(nav_msgs_occupancy_grid_get_data(NULL));
    nav_msgs_occupancy_grid_get_info(NULL, NULL, NULL, NULL, NULL, NULL,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL);
}

// ============================================================================
// Path Tests (buffer-backed)
// ============================================================================

Test(nav_msgs, path_from_golden_fixture) {
    size_t len = 0;
    uint8_t *b = load_fixture_nm("testdata/cdr/nav_msgs/Path.cdr", &len);
    cr_assert_not_null(b, "failed to load Path fixture");
    nav_msgs_path_t *v = nav_msgs_path_from_cdr(b, len);
    cr_assert_not_null(v);

    cr_assert_str_eq(nav_msgs_path_get_frame_id(v), "test_frame");
    cr_assert_eq(nav_msgs_path_get_len(v), 3u);

    int32_t ssec = 0;
    double px = 0, py = 0, pz = 0;
    const char *pose_fid = NULL;
    cr_assert_eq(nav_msgs_path_get_pose(v, 0, &ssec, NULL, &pose_fid, &px, &py, &pz,
                                    NULL, NULL, NULL, NULL), 0);
    cr_assert_eq(ssec, 1);
    cr_assert_str_eq(pose_fid, "map");
    cr_assert_float_eq(px, 1.0, 1e-12);
    cr_assert_float_eq(py, 0.0, 1e-12);

    cr_assert_eq(nav_msgs_path_get_pose(v, 2, &ssec, NULL, NULL, NULL, NULL, NULL,
                                    NULL, NULL, NULL, NULL), 0);
    cr_assert_eq(ssec, 3);

    nav_msgs_path_free(v);
    free(b);
}

Test(nav_msgs, path_from_cdr_null) {
    errno = 0;
    cr_assert_null(nav_msgs_path_from_cdr(NULL, 100));
    cr_assert_eq(errno, EINVAL);
}

Test(nav_msgs, path_free_null) {
    nav_msgs_path_free(NULL);
}

Test(nav_msgs, path_getters_null) {
    cr_assert_eq(nav_msgs_path_get_stamp_sec(NULL), 0);
    cr_assert_null(nav_msgs_path_get_frame_id(NULL));
    cr_assert_eq(nav_msgs_path_get_len(NULL), 0u);
    cr_assert_eq(nav_msgs_path_get_pose(NULL, 0, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL), -1);
}

Test(nav_msgs, path_pose_out_of_range) {
    size_t len = 0;
    uint8_t *b = load_fixture_nm("testdata/cdr/nav_msgs/Path.cdr", &len);
    cr_assert_not_null(b);
    nav_msgs_path_t *v = nav_msgs_path_from_cdr(b, len);
    cr_assert_not_null(v);
    errno = 0;
    cr_assert_eq(nav_msgs_path_get_pose(v, 99, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL), -1);
    cr_assert_eq(errno, EINVAL);
    nav_msgs_path_free(v);
    free(b);
}

/* ── byte-equality: builder output must be identical to the golden fixture ── */

Test(nav_msgs, grid_cells_builder_matches_golden) {
    size_t golden_len = 0;
    uint8_t *golden = load_fixture_nm("testdata/cdr/nav_msgs/GridCells.cdr", &golden_len);
    cr_assert_not_null(golden, "failed to load GridCells fixture");

    nav_msgs_grid_cells_builder_t *b = nav_msgs_grid_cells_builder_new();
    cr_assert_not_null(b);
    nav_msgs_grid_cells_builder_set_stamp(b, 1234567890, 123456789u);
    cr_assert_eq(nav_msgs_grid_cells_builder_set_frame_id(b, "test_frame"), 0);
    nav_msgs_grid_cells_builder_set_cell_width(b, 0.5f);
    nav_msgs_grid_cells_builder_set_cell_height(b, 0.5f);
    double xyz[9] = {1.0, 2.0, 0.0,  3.0, 4.0, 0.0,  5.0, 6.0, 0.0};
    cr_assert_eq(nav_msgs_grid_cells_builder_set_cells(b, xyz, 3), 0);

    uint8_t *out = NULL;
    size_t out_len = 0;
    cr_assert_eq(nav_msgs_grid_cells_builder_build(b, &out, &out_len), 0);
    cr_assert_eq(out_len, golden_len);
    cr_assert_eq(memcmp(out, golden, golden_len), 0,
                 "GridCells builder output differs from golden fixture");

    edgefirst_schemas_bytes_free(out, out_len);
    nav_msgs_grid_cells_builder_free(b);
    free(golden);
}

Test(nav_msgs, occupancy_grid_builder_matches_golden) {
    size_t golden_len = 0;
    uint8_t *golden = load_fixture_nm("testdata/cdr/nav_msgs/OccupancyGrid.cdr", &golden_len);
    cr_assert_not_null(golden, "failed to load OccupancyGrid fixture");

    nav_msgs_occupancy_grid_builder_t *b = nav_msgs_occupancy_grid_builder_new();
    cr_assert_not_null(b);
    nav_msgs_occupancy_grid_builder_set_stamp(b, 1234567890, 123456789u);
    cr_assert_eq(nav_msgs_occupancy_grid_builder_set_frame_id(b, "test_frame"), 0);
    nav_msgs_occupancy_grid_builder_set_info(b,
        1234567890, 123456789u,   /* map_load_time */
        0.1f, 4u, 2u,             /* resolution, width, height */
        0.0, 0.0, 0.0,            /* origin position */
        0.0, 0.0, 0.0, 1.0);     /* origin orientation */
    const int8_t data[8] = {0, 50, 100, -1, 25, 75, 0, 0};
    cr_assert_eq(nav_msgs_occupancy_grid_builder_set_data(b, data, 8), 0);

    uint8_t *out = NULL;
    size_t out_len = 0;
    cr_assert_eq(nav_msgs_occupancy_grid_builder_build(b, &out, &out_len), 0);
    cr_assert_eq(out_len, golden_len);
    cr_assert_eq(memcmp(out, golden, golden_len), 0,
                 "OccupancyGrid builder output differs from golden fixture");

    edgefirst_schemas_bytes_free(out, out_len);
    nav_msgs_occupancy_grid_builder_free(b);
    free(golden);
}

Test(nav_msgs, path_builder_matches_golden) {
    size_t golden_len = 0;
    uint8_t *golden = load_fixture_nm("testdata/cdr/nav_msgs/Path.cdr", &golden_len);
    cr_assert_not_null(golden, "failed to load Path fixture");

    nav_msgs_path_builder_t *b = nav_msgs_path_builder_new();
    cr_assert_not_null(b);
    nav_msgs_path_builder_set_stamp(b, 1234567890, 123456789u);
    cr_assert_eq(nav_msgs_path_builder_set_frame_id(b, "test_frame"), 0);
    cr_assert_eq(nav_msgs_path_builder_add_pose(b, 1, 0u, "map",
        1.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0), 0);
    cr_assert_eq(nav_msgs_path_builder_add_pose(b, 2, 0u, "map",
        2.0, 1.0, 0.0,  0.0, 0.0, 0.707, 0.707), 0);
    cr_assert_eq(nav_msgs_path_builder_add_pose(b, 3, 0u, "map",
        3.0, 2.0, 0.0,  0.0, 0.0, 1.0, 0.0), 0);

    uint8_t *out = NULL;
    size_t out_len = 0;
    cr_assert_eq(nav_msgs_path_builder_build(b, &out, &out_len), 0);
    cr_assert_eq(out_len, golden_len);
    cr_assert_eq(memcmp(out, golden, golden_len), 0,
                 "Path builder output differs from golden fixture");

    edgefirst_schemas_bytes_free(out, out_len);
    nav_msgs_path_builder_free(b);
    free(golden);
}

Test(nav_msgs, odometry_builder_null) {
    errno = 0;
    cr_assert_eq(nav_msgs_odometry_builder_set_frame_id(NULL, "x"), -1);
    cr_assert_eq(errno, EINVAL);
}

Test(nav_msgs, odometry_builder_matches_golden) {
    size_t golden_len = 0;
    uint8_t *golden = load_fixture_nm("testdata/cdr/nav_msgs/Odometry.cdr", &golden_len);
    cr_assert_not_null(golden);

    double pose_cov[36] = {0};
    double twist_cov[36] = {0};
    for (int i = 0; i < 6; i++) {
        pose_cov[i * 6 + i] = 0.1 * (i + 1);
        twist_cov[i * 6 + i] = 0.02 * (i + 1);
    }
    pose_cov[1] = 0.01;
    pose_cov[6] = 0.01;
    twist_cov[7] = 0.001;

    nav_msgs_odometry_builder_t *b = nav_msgs_odometry_builder_new();
    cr_assert_not_null(b);
    nav_msgs_odometry_builder_set_stamp(b, 1234567890, 123456789u);
    cr_assert_eq(nav_msgs_odometry_builder_set_frame_id(b, "test_frame"), 0);
    cr_assert_eq(nav_msgs_odometry_builder_set_child_frame_id(b, "base_link"), 0);
    nav_msgs_odometry_builder_set_pose(b, 1.5, -2.5, 3.0, 0.0, 0.0, 0.0, 1.0);
    cr_assert_eq(nav_msgs_odometry_builder_set_pose_covariance(b, pose_cov), 0);
    nav_msgs_odometry_builder_set_twist(b, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3);
    cr_assert_eq(nav_msgs_odometry_builder_set_twist_covariance(b, twist_cov), 0);

    uint8_t *out = NULL;
    size_t out_len = 0;
    cr_assert_eq(nav_msgs_odometry_builder_build(b, &out, &out_len), 0);
    cr_assert_eq(out_len, golden_len);
    cr_assert_eq(memcmp(out, golden, golden_len), 0);

    edgefirst_schemas_bytes_free(out, out_len);
    nav_msgs_odometry_builder_free(b);
    free(golden);
}
