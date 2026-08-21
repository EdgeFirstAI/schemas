/**
 * @file test_tensor.c
 * @brief Criterion tests for the composed tensor message family.
 *
 * Covers Tensor (payload), TensorPlane, and the two byte-identical wrappers
 * TensorStamped and CameraFrame.
 *
 * The contract these tests defend is composition: a wrapper is a header plus
 * `seq` plus an embedded Tensor, and the embedded Tensor's bytes are
 * position-independent — identical no matter which wrapper carries it, and
 * re-parseable standalone. Several tests below assert that directly rather
 * than merely round-tripping fields.
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "edgefirst/schemas.h"

/* ------------------------------------------------------------------ */
/* Fixtures                                                            */
/* ------------------------------------------------------------------ */

static const uint64_t SHAPE_NV12[2] = {480, 640};
static const int64_t STRIDES_NV12[2] = {640, 1};

/* Two referenced planes, as a real NV12 dma-buf frame would carry:
 * one allocation, Y at offset 0 and UV at offset w*h. */
static ros_tensor_plane_elem_t nv12_planes[2];
static const uint8_t HANDLE_BYTES[4] = {0xDE, 0xAD, 0xBE, 0xEF};

static void init_nv12_planes(void) {
    memset(nv12_planes, 0, sizeof(nv12_planes));
    nv12_planes[0].handle = 7;
    nv12_planes[0].offset = 0;
    nv12_planes[0].stride = 640;
    nv12_planes[0].size = 640 * 480;
    nv12_planes[0].used = 640 * 480;
    nv12_planes[0].modifier = 0;
    nv12_planes[0].handle_bytes = HANDLE_BYTES;
    nv12_planes[0].handle_bytes_len = sizeof(HANDLE_BYTES);

    nv12_planes[1].handle = 7;
    nv12_planes[1].offset = 640 * 480;
    nv12_planes[1].stride = 640;
    nv12_planes[1].size = 640 * 480 / 2;
    nv12_planes[1].used = 640 * 480 / 2;
    nv12_planes[1].modifier = 0;
}

/* Populate a tensor builder with the standard NV12 fixture. */
static ros_tensor_builder_t *make_nv12_builder(void) {
    init_nv12_planes();
    ros_tensor_builder_t *tb = ros_tensor_builder_new();
    ros_tensor_builder_set_storage_kind(tb, 2);
    ros_tensor_builder_set_pid(tb, 4242);
    ros_tensor_builder_set_fence_fd(tb, -1);
    ros_tensor_builder_set_dtype(tb, 1);
    ros_tensor_builder_set_quant_axis(tb, -2);
    ros_tensor_builder_set_shape(tb, SHAPE_NV12, 2);
    ros_tensor_builder_set_strides(tb, STRIDES_NV12, 2);
    ros_tensor_builder_set_format(tb, "NV12");
    ros_tensor_builder_set_color_space(tb, "bt709");
    ros_tensor_builder_set_color_transfer(tb, "bt709");
    ros_tensor_builder_set_color_encoding(tb, "bt709");
    ros_tensor_builder_set_color_range(tb, "limited");
    ros_tensor_builder_set_planes(tb, nv12_planes, 2);
    return tb;
}

/* ------------------------------------------------------------------ */
/* Tensor — standalone round trip                                      */
/* ------------------------------------------------------------------ */

Test(tensor, builder_round_trip_all_fields) {
    ros_tensor_builder_t *tb = make_nv12_builder();

    uint8_t *bytes = NULL;
    size_t len = 0;
    cr_assert_eq(ros_tensor_builder_build(tb, &bytes, &len), 0);
    cr_assert_not_null(bytes);
    cr_assert_gt(len, 0);

    ros_tensor_t *t = ros_tensor_from_cdr(bytes, len);
    cr_assert_not_null(t);

    cr_assert_eq(ros_tensor_get_storage_kind(t), 2);
    cr_assert_eq(ros_tensor_get_pid(t), 4242);
    cr_assert_eq(ros_tensor_get_fence_fd(t), -1);
    cr_assert_eq(ros_tensor_get_dtype(t), 1);
    cr_assert_eq(ros_tensor_get_quant_axis(t), -2);
    cr_assert_str_eq(ros_tensor_get_format(t), "NV12");
    cr_assert_str_eq(ros_tensor_get_color_space(t), "bt709");
    cr_assert_str_eq(ros_tensor_get_color_transfer(t), "bt709");
    cr_assert_str_eq(ros_tensor_get_color_encoding(t), "bt709");
    cr_assert_str_eq(ros_tensor_get_color_range(t), "limited");

    cr_assert_eq(ros_tensor_get_shape_len(t), 2);
    uint64_t d = 0;
    cr_assert_eq(ros_tensor_get_shape_at(t, 0, &d), 0);
    cr_assert_eq(d, 480);
    cr_assert_eq(ros_tensor_get_shape_at(t, 1, &d), 0);
    cr_assert_eq(d, 640);

    cr_assert_eq(ros_tensor_get_strides_len(t), 2);
    int64_t s = 0;
    cr_assert_eq(ros_tensor_get_strides_at(t, 0, &s), 0);
    cr_assert_eq(s, 640);

    /* An unquantized tensor carries no scales at all. */
    size_t n = 99;
    cr_assert_null(ros_tensor_get_quant_scales(t, &n));
    cr_assert_eq(n, 0);

    cr_assert_eq(ros_tensor_get_num_planes(t), 2);

    ros_tensor_free(t);
    ros_bytes_free(bytes, len);
    ros_tensor_builder_free(tb);
}

Test(tensor, plane_fields_round_trip) {
    ros_tensor_builder_t *tb = make_nv12_builder();
    uint8_t *bytes = NULL;
    size_t len = 0;
    cr_assert_eq(ros_tensor_builder_build(tb, &bytes, &len), 0);

    ros_tensor_t *t = ros_tensor_from_cdr(bytes, len);
    cr_assert_not_null(t);

    const ros_tensor_plane_t *p0 = ros_tensor_get_plane(t, 0);
    cr_assert_not_null(p0);
    cr_assert_eq(ros_tensor_plane_get_handle(p0), 7);
    cr_assert_eq(ros_tensor_plane_get_offset(p0), 0);
    cr_assert_eq(ros_tensor_plane_get_stride(p0), 640);
    cr_assert_eq(ros_tensor_plane_get_size(p0), 640u * 480u);
    cr_assert_eq(ros_tensor_plane_get_used(p0), 640u * 480u);
    cr_assert_eq(ros_tensor_plane_get_modifier(p0), 0);
    cr_assert(!ros_tensor_plane_is_inline(p0));

    size_t hb_len = 0;
    const uint8_t *hb = ros_tensor_plane_get_handle_bytes(p0, &hb_len);
    cr_assert_eq(hb_len, 4);
    cr_assert_not_null(hb);
    cr_assert_eq(memcmp(hb, HANDLE_BYTES, 4), 0);

    /* A referenced plane carries no inline bytes. */
    size_t d_len = 1;
    const uint8_t *d = ros_tensor_plane_get_data(p0, &d_len);
    cr_assert_eq(d_len, 0);
    cr_assert_null(d);

    const ros_tensor_plane_t *p1 = ros_tensor_get_plane(t, 1);
    cr_assert_not_null(p1);
    cr_assert_eq(ros_tensor_plane_get_offset(p1), 640u * 480u);

    /* Out of range. */
    errno = 0;
    cr_assert_null(ros_tensor_get_plane(t, 2));
    cr_assert_eq(errno, EINVAL);

    ros_tensor_free(t);
    ros_bytes_free(bytes, len);
    ros_tensor_builder_free(tb);
}

Test(tensor, inline_plane_round_trip) {
    const uint8_t payload[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    ros_tensor_plane_elem_t plane;
    memset(&plane, 0, sizeof(plane));
    plane.handle = -1;
    plane.size = sizeof(payload);
    plane.used = sizeof(payload);
    plane.data = payload;
    plane.data_len = sizeof(payload);

    ros_tensor_builder_t *tb = ros_tensor_builder_new();
    ros_tensor_builder_set_dtype(tb, 1);
    ros_tensor_builder_set_planes(tb, &plane, 1);

    uint8_t *bytes = NULL;
    size_t len = 0;
    cr_assert_eq(ros_tensor_builder_build(tb, &bytes, &len), 0);

    ros_tensor_t *t = ros_tensor_from_cdr(bytes, len);
    cr_assert_not_null(t);
    const ros_tensor_plane_t *p = ros_tensor_get_plane(t, 0);
    cr_assert_not_null(p);
    cr_assert(ros_tensor_plane_is_inline(p));
    cr_assert_eq(ros_tensor_plane_get_handle(p), -1);

    size_t n = 0;
    const uint8_t *got = ros_tensor_plane_get_data(p, &n);
    cr_assert_eq(n, sizeof(payload));
    cr_assert_eq(memcmp(got, payload, sizeof(payload)), 0);

    ros_tensor_free(t);
    ros_bytes_free(bytes, len);
    ros_tensor_builder_free(tb);
}

/* ------------------------------------------------------------------ */
/* Validation — the builder must reject incoherent plane sets          */
/* ------------------------------------------------------------------ */

Test(tensor, inline_plane_with_wrong_size_is_rejected) {
    const uint8_t payload[8] = {0};
    ros_tensor_plane_elem_t plane;
    memset(&plane, 0, sizeof(plane));
    plane.handle = -1;
    plane.size = 99; /* lies about the inline length */
    plane.data = payload;
    plane.data_len = sizeof(payload);

    ros_tensor_builder_t *tb = ros_tensor_builder_new();
    ros_tensor_builder_set_planes(tb, &plane, 1);

    uint8_t *bytes = NULL;
    size_t len = 0;
    errno = 0;
    cr_assert_eq(ros_tensor_builder_build(tb, &bytes, &len), -1);
    cr_assert_eq(errno, EBADMSG);
    ros_tensor_builder_free(tb);
}

Test(tensor, mixed_transport_modes_are_rejected) {
    const uint8_t payload[4] = {0};
    ros_tensor_plane_elem_t planes[2];
    memset(planes, 0, sizeof(planes));
    planes[0].handle = 3; /* referenced */
    planes[0].size = 16;
    planes[1].handle = -1; /* inline — incoherent with plane 0 */
    planes[1].size = sizeof(payload);
    planes[1].data = payload;
    planes[1].data_len = sizeof(payload);

    ros_tensor_builder_t *tb = ros_tensor_builder_new();
    ros_tensor_builder_set_planes(tb, planes, 2);

    uint8_t *bytes = NULL;
    size_t len = 0;
    errno = 0;
    cr_assert_eq(ros_tensor_builder_build(tb, &bytes, &len), -1);
    cr_assert_eq(errno, EBADMSG);
    ros_tensor_builder_free(tb);
}

Test(tensor, used_greater_than_size_is_rejected) {
    ros_tensor_plane_elem_t plane;
    memset(&plane, 0, sizeof(plane));
    plane.handle = 3;
    plane.size = 16;
    plane.used = 17; /* more used than allocated */

    ros_tensor_builder_t *tb = ros_tensor_builder_new();
    ros_tensor_builder_set_planes(tb, &plane, 1);

    uint8_t *bytes = NULL;
    size_t len = 0;
    errno = 0;
    cr_assert_eq(ros_tensor_builder_build(tb, &bytes, &len), -1);
    cr_assert_eq(errno, EBADMSG);
    ros_tensor_builder_free(tb);
}

/* ------------------------------------------------------------------ */
/* Bulk shape/strides copy                                             */
/* ------------------------------------------------------------------ */

Test(tensor, copy_shape_and_strides) {
    ros_tensor_builder_t *tb = make_nv12_builder();
    uint8_t *bytes = NULL;
    size_t len = 0;
    cr_assert_eq(ros_tensor_builder_build(tb, &bytes, &len), 0);
    ros_tensor_t *t = ros_tensor_from_cdr(bytes, len);
    cr_assert_not_null(t);

    uint64_t shape[2] = {0, 0};
    cr_assert_eq(ros_tensor_copy_shape(t, shape, 2), 2);
    cr_assert_eq(shape[0], 480);
    cr_assert_eq(shape[1], 640);

    int64_t strides[2] = {0, 0};
    cr_assert_eq(ros_tensor_copy_strides(t, strides, 2), 2);
    cr_assert_eq(strides[0], 640);
    cr_assert_eq(strides[1], 1);

    /* Too small a destination is reported, not truncated silently. */
    errno = 0;
    cr_assert_eq(ros_tensor_copy_shape(t, shape, 1), -1);
    cr_assert_eq(errno, ENOBUFS);

    ros_tensor_free(t);
    ros_bytes_free(bytes, len);
    ros_tensor_builder_free(tb);
}

/* ------------------------------------------------------------------ */
/* Wrappers                                                            */
/* ------------------------------------------------------------------ */

Test(tensor, camera_frame_round_trip) {
    ros_tensor_builder_t *tb = make_nv12_builder();
    ros_camera_frame_builder_t *fb = ros_camera_frame_builder_new();
    ros_camera_frame_builder_set_stamp(fb, 1234567890, 123456789);
    ros_camera_frame_builder_set_frame_id(fb, "camera_0");
    ros_camera_frame_builder_set_seq(fb, 99);
    cr_assert_eq(ros_camera_frame_builder_set_tensor(fb, tb), 0);

    uint8_t *bytes = NULL;
    size_t len = 0;
    cr_assert_eq(ros_camera_frame_builder_build(fb, &bytes, &len), 0);

    ros_camera_frame_t *f = ros_camera_frame_from_cdr(bytes, len);
    cr_assert_not_null(f);
    cr_assert_eq(ros_camera_frame_get_stamp_sec(f), 1234567890);
    cr_assert_eq(ros_camera_frame_get_stamp_nanosec(f), 123456789);
    cr_assert_str_eq(ros_camera_frame_get_frame_id(f), "camera_0");
    cr_assert_eq(ros_camera_frame_get_seq(f), 99);

    const ros_tensor_t *t = ros_camera_frame_get_tensor(f);
    cr_assert_not_null(t);
    cr_assert_str_eq(ros_tensor_get_format(t), "NV12");
    cr_assert_eq(ros_tensor_get_num_planes(t), 2);
    cr_assert_eq(ros_tensor_plane_get_handle(ros_tensor_get_plane(t, 0)), 7);

    ros_camera_frame_free(f);
    ros_bytes_free(bytes, len);
    ros_camera_frame_builder_free(fb);
    ros_tensor_builder_free(tb);
}

Test(tensor, builder_without_tensor_is_rejected) {
    ros_camera_frame_builder_t *fb = ros_camera_frame_builder_new();
    ros_camera_frame_builder_set_seq(fb, 1);
    uint8_t *bytes = NULL;
    size_t len = 0;
    errno = 0;
    cr_assert_eq(ros_camera_frame_builder_build(fb, &bytes, &len), -1);
    cr_assert_eq(errno, EINVAL);
    ros_camera_frame_builder_free(fb);
}

/*
 * Position independence: the SAME tensor embedded in either wrapper, with
 * frame_id strings of different length, must produce byte-identical tensor
 * bytes. This is what `seq` (a uint64) buys — it forces the embedded tensor
 * to an 8-aligned offset no matter how long frame_id is.
 */
Test(tensor, tensor_bytes_are_identical_across_wrappers_and_frame_ids) {
    ros_tensor_builder_t *tb = make_nv12_builder();

    const char *ids[3] = {"a", "camera_0", "a_very_long_frame_identifier_x"};
    uint8_t *ref = NULL;
    size_t ref_len = 0;

    for (int i = 0; i < 3; i++) {
        /* CameraFrame */
        ros_camera_frame_builder_t *fb = ros_camera_frame_builder_new();
        ros_camera_frame_builder_set_frame_id(fb, ids[i]);
        ros_camera_frame_builder_set_seq(fb, 5);
        ros_camera_frame_builder_set_tensor(fb, tb);
        uint8_t *fbytes = NULL;
        size_t flen = 0;
        cr_assert_eq(ros_camera_frame_builder_build(fb, &fbytes, &flen), 0);
        ros_camera_frame_t *f = ros_camera_frame_from_cdr(fbytes, flen);
        cr_assert_not_null(f);
        size_t tlen = 0;
        const uint8_t *tb_bytes =
            ros_tensor_get_tensor_bytes(ros_camera_frame_get_tensor(f), &tlen);

        /* TensorStamped, same inputs */
        ros_tensor_stamped_builder_t *sb = ros_tensor_stamped_builder_new();
        ros_tensor_stamped_builder_set_frame_id(sb, ids[i]);
        ros_tensor_stamped_builder_set_seq(sb, 5);
        ros_tensor_stamped_builder_set_tensor(sb, tb);
        uint8_t *sbytes = NULL;
        size_t slen = 0;
        cr_assert_eq(ros_tensor_stamped_builder_build(sb, &sbytes, &slen), 0);
        ros_tensor_stamped_t *s = ros_tensor_stamped_from_cdr(sbytes, slen);
        cr_assert_not_null(s);
        size_t s_tlen = 0;
        const uint8_t *s_tbytes =
            ros_tensor_get_tensor_bytes(ros_tensor_stamped_get_tensor(s), &s_tlen);

        /* Same wrapper-to-wrapper... */
        cr_assert_eq(tlen, s_tlen);
        cr_assert_eq(memcmp(tb_bytes, s_tbytes, tlen), 0);

        /* ...and same across every frame_id length. */
        if (ref == NULL) {
            ref = (uint8_t *) malloc(tlen);
            memcpy(ref, tb_bytes, tlen);
            ref_len = tlen;
        } else {
            cr_assert_eq(tlen, ref_len);
            cr_assert_eq(memcmp(tb_bytes, ref, tlen), 0);
        }

        ros_camera_frame_free(f);
        ros_bytes_free(fbytes, flen);
        ros_camera_frame_builder_free(fb);
        ros_tensor_stamped_free(s);
        ros_bytes_free(sbytes, slen);
        ros_tensor_stamped_builder_free(sb);
    }
    free(ref);
    ros_tensor_builder_free(tb);
}

/* An embedded tensor, re-headed, must parse as a standalone Tensor. */
Test(tensor, embedded_tensor_reparses_standalone) {
    ros_tensor_builder_t *tb = make_nv12_builder();
    ros_camera_frame_builder_t *fb = ros_camera_frame_builder_new();
    ros_camera_frame_builder_set_frame_id(fb, "camera_0");
    ros_camera_frame_builder_set_tensor(fb, tb);
    uint8_t *bytes = NULL;
    size_t len = 0;
    cr_assert_eq(ros_camera_frame_builder_build(fb, &bytes, &len), 0);
    ros_camera_frame_t *f = ros_camera_frame_from_cdr(bytes, len);
    cr_assert_not_null(f);

    uint8_t *standalone = NULL;
    size_t sa_len = 0;
    cr_assert_eq(
        ros_tensor_to_standalone_cdr(ros_camera_frame_get_tensor(f), &standalone, &sa_len), 0);

    ros_tensor_t *t = ros_tensor_from_cdr(standalone, sa_len);
    cr_assert_not_null(t);
    cr_assert_str_eq(ros_tensor_get_format(t), "NV12");
    cr_assert_eq(ros_tensor_get_num_planes(t), 2);
    cr_assert_eq(ros_tensor_get_pid(t), 4242);

    /* And it is byte-identical to encoding the same tensor from scratch. */
    uint8_t *direct = NULL;
    size_t direct_len = 0;
    cr_assert_eq(ros_tensor_builder_build(tb, &direct, &direct_len), 0);
    cr_assert_eq(sa_len, direct_len);
    cr_assert_eq(memcmp(standalone, direct, sa_len), 0);

    ros_tensor_free(t);
    ros_bytes_free(standalone, sa_len);
    ros_bytes_free(direct, direct_len);
    ros_camera_frame_free(f);
    ros_bytes_free(bytes, len);
    ros_camera_frame_builder_free(fb);
    ros_tensor_builder_free(tb);
}

/* ------------------------------------------------------------------ */
/* Ownership                                                           */
/* ------------------------------------------------------------------ */

/*
 * Freeing a parent-borrowed child must be a no-op with errno=EINVAL, not a
 * double free. Both the tensor handed out by a wrapper and the planes handed
 * out by a tensor are borrowed.
 */
Test(tensor, freeing_borrowed_children_is_a_safe_noop) {
    ros_tensor_builder_t *tb = make_nv12_builder();
    ros_camera_frame_builder_t *fb = ros_camera_frame_builder_new();
    ros_camera_frame_builder_set_tensor(fb, tb);
    uint8_t *bytes = NULL;
    size_t len = 0;
    cr_assert_eq(ros_camera_frame_builder_build(fb, &bytes, &len), 0);
    ros_camera_frame_t *f = ros_camera_frame_from_cdr(bytes, len);
    cr_assert_not_null(f);

    const ros_tensor_t *t = ros_camera_frame_get_tensor(f);
    cr_assert_not_null(t);
    errno = 0;
    ros_tensor_free((ros_tensor_t *) t);
    cr_assert_eq(errno, EINVAL);

    /* Still usable — the no-op really was a no-op. */
    cr_assert_eq(ros_tensor_get_num_planes(t), 2);

    const ros_tensor_plane_t *p = ros_tensor_get_plane(t, 0);
    errno = 0;
    ros_tensor_plane_free((ros_tensor_plane_t *) p);
    cr_assert_eq(errno, EINVAL);
    cr_assert_eq(ros_tensor_plane_get_handle(p), 7);

    ros_camera_frame_free(f);
    ros_bytes_free(bytes, len);
    ros_camera_frame_builder_free(fb);
    ros_tensor_builder_free(tb);
}

/* A standalone tensor handle IS owned, and frees without complaint. */
Test(tensor, standalone_tensor_free_is_owned) {
    ros_tensor_builder_t *tb = make_nv12_builder();
    uint8_t *bytes = NULL;
    size_t len = 0;
    cr_assert_eq(ros_tensor_builder_build(tb, &bytes, &len), 0);
    ros_tensor_t *t = ros_tensor_from_cdr(bytes, len);
    cr_assert_not_null(t);
    errno = 0;
    ros_tensor_free(t);
    cr_assert_eq(errno, 0);
    ros_bytes_free(bytes, len);
    ros_tensor_builder_free(tb);
}

/* ------------------------------------------------------------------ */
/* In-place setters                                                    */
/* ------------------------------------------------------------------ */

Test(tensor, in_place_stamp_and_seq) {
    ros_tensor_builder_t *tb = make_nv12_builder();
    ros_camera_frame_builder_t *fb = ros_camera_frame_builder_new();
    ros_camera_frame_builder_set_frame_id(fb, "camera_0");
    ros_camera_frame_builder_set_tensor(fb, tb);
    uint8_t *bytes = NULL;
    size_t len = 0;
    cr_assert_eq(ros_camera_frame_builder_build(fb, &bytes, &len), 0);

    cr_assert_eq(ros_camera_frame_set_stamp(bytes, len, 42, 7), 0);
    cr_assert_eq(ros_camera_frame_set_seq(bytes, len, 12345), 0);

    ros_camera_frame_t *f = ros_camera_frame_from_cdr(bytes, len);
    cr_assert_not_null(f);
    cr_assert_eq(ros_camera_frame_get_stamp_sec(f), 42);
    cr_assert_eq(ros_camera_frame_get_stamp_nanosec(f), 7);
    cr_assert_eq(ros_camera_frame_get_seq(f), 12345);

    /* The payload must be untouched by a header-only edit. */
    cr_assert_eq(ros_tensor_get_num_planes(ros_camera_frame_get_tensor(f)), 2);
    cr_assert_str_eq(ros_tensor_get_format(ros_camera_frame_get_tensor(f)), "NV12");

    ros_camera_frame_free(f);
    ros_bytes_free(bytes, len);
    ros_camera_frame_builder_free(fb);
    ros_tensor_builder_free(tb);
}

/* ------------------------------------------------------------------ */
/* NULL and error handling                                             */
/* ------------------------------------------------------------------ */

Test(tensor, from_cdr_null_and_invalid) {
    errno = 0;
    cr_assert_null(ros_tensor_from_cdr(NULL, 10));
    cr_assert_eq(errno, EINVAL);

    uint8_t bad[] = {0xDE, 0xAD, 0xBE, 0xEF};
    errno = 0;
    cr_assert_null(ros_tensor_from_cdr(bad, sizeof(bad)));
    cr_assert_eq(errno, EBADMSG);

    errno = 0;
    cr_assert_null(ros_camera_frame_from_cdr(NULL, 10));
    cr_assert_eq(errno, EINVAL);

    errno = 0;
    cr_assert_null(ros_tensor_stamped_from_cdr(bad, sizeof(bad)));
    cr_assert_eq(errno, EBADMSG);
}

Test(tensor, null_getters_return_defaults) {
    /* Defaults are the schema's "absent" values, not blanket zeroes. */
    cr_assert_eq(ros_tensor_get_fence_fd(NULL), -1);
    cr_assert_eq(ros_tensor_get_quant_axis(NULL), -2);
    cr_assert_eq(ros_tensor_get_storage_kind(NULL), 0);
    cr_assert_eq(ros_tensor_get_num_planes(NULL), 0);
    cr_assert_null(ros_tensor_get_format(NULL));
    cr_assert_eq(ros_tensor_plane_get_handle(NULL), -1);
    cr_assert_eq(ros_tensor_plane_get_size(NULL), 0);
    cr_assert(!ros_tensor_plane_is_inline(NULL));

    size_t n = 99;
    cr_assert_null(ros_tensor_get_quant_scales(NULL, &n));
    cr_assert_eq(n, 0);

    uint64_t d = 0;
    errno = 0;
    cr_assert_eq(ros_tensor_get_shape_at(NULL, 0, &d), -1);
    cr_assert_eq(errno, EINVAL);
}

Test(tensor, free_null_is_safe) {
    ros_tensor_free(NULL);
    ros_tensor_plane_free(NULL);
    ros_camera_frame_free(NULL);
    ros_tensor_stamped_free(NULL);
    ros_tensor_builder_free(NULL);
    ros_camera_frame_builder_free(NULL);
    ros_tensor_stamped_builder_free(NULL);
}

Test(tensor, builder_string_setters_reject_null) {
    ros_tensor_builder_t *tb = ros_tensor_builder_new();
    errno = 0;
    cr_assert_eq(ros_tensor_builder_set_format(tb, NULL), -1);
    cr_assert_eq(errno, EINVAL);
    ros_tensor_builder_free(tb);
}

Test(tensor, encode_into_reports_short_buffer) {
    ros_tensor_builder_t *tb = make_nv12_builder();
    uint8_t small[8];
    size_t written = 0;
    errno = 0;
    cr_assert_eq(ros_tensor_builder_encode_into(tb, small, sizeof(small), &written), -1);
    cr_assert_eq(errno, ENOBUFS);

    /* A right-sized buffer succeeds and reports the same length as build(). */
    uint8_t *bytes = NULL;
    size_t len = 0;
    cr_assert_eq(ros_tensor_builder_build(tb, &bytes, &len), 0);
    uint8_t *big = (uint8_t *) malloc(len);
    cr_assert_eq(ros_tensor_builder_encode_into(tb, big, len, &written), 0);
    cr_assert_eq(written, len);
    cr_assert_eq(memcmp(big, bytes, len), 0);

    free(big);
    ros_bytes_free(bytes, len);
    ros_tensor_builder_free(tb);
}

/* ------------------------------------------------------------------ */
/* Quantization                                                        */
/* ------------------------------------------------------------------ */
/*
 * `quant_axis` selects which of three mutually exclusive shapes the
 * quantization parameters take, and the encoder enforces the match:
 *   -2  unquantized      -> quant_scales MUST be empty
 *   -1  per-tensor       -> exactly one scale
 *   >=0 per-axis         -> exactly shape[axis] scales
 * Each arm is exercised here, in both its accepted and rejected form, so a
 * regression that drops the check fails rather than silently widening.
 */

Test(tensor, per_tensor_quantization_round_trip) {
    static const uint64_t shape[1] = {4};
    static const float scale[1] = {0.125f};
    static const int32_t zero[1] = {7};

    ros_tensor_builder_t *tb = ros_tensor_builder_new();
    ros_tensor_builder_set_dtype(tb, 3);
    ros_tensor_builder_set_shape(tb, shape, 1);
    ros_tensor_builder_set_quant_axis(tb, -1);
    ros_tensor_builder_set_quant_scales(tb, scale, 1);
    ros_tensor_builder_set_quant_zero_points(tb, zero, 1);

    uint8_t *bytes = NULL;
    size_t len = 0;
    cr_assert_eq(ros_tensor_builder_build(tb, &bytes, &len), 0);

    ros_tensor_t *t = ros_tensor_from_cdr(bytes, len);
    cr_assert_not_null(t);
    cr_assert_eq(ros_tensor_get_quant_axis(t), -1);

    size_t n = 0;
    const float *sc = ros_tensor_get_quant_scales(t, &n);
    cr_assert_eq(n, 1);
    cr_assert_float_eq(sc[0], 0.125f, 1e-6);
    const int32_t *zp = ros_tensor_get_quant_zero_points(t, &n);
    cr_assert_eq(n, 1);
    cr_assert_eq(zp[0], 7);

    ros_tensor_free(t);
    ros_bytes_free(bytes, len);
    ros_tensor_builder_free(tb);
}

Test(tensor, per_axis_quantization_round_trip) {
    static const uint64_t shape[2] = {3, 8};
    static const float scales[3] = {0.5f, 0.25f, 0.125f};

    ros_tensor_builder_t *tb = ros_tensor_builder_new();
    ros_tensor_builder_set_dtype(tb, 3);
    ros_tensor_builder_set_shape(tb, shape, 2);
    ros_tensor_builder_set_quant_axis(tb, 0);
    ros_tensor_builder_set_quant_scales(tb, scales, 3); /* == shape[0] */

    uint8_t *bytes = NULL;
    size_t len = 0;
    cr_assert_eq(ros_tensor_builder_build(tb, &bytes, &len), 0);

    ros_tensor_t *t = ros_tensor_from_cdr(bytes, len);
    cr_assert_not_null(t);
    cr_assert_eq(ros_tensor_get_quant_axis(t), 0);
    size_t n = 0;
    cr_assert_not_null(ros_tensor_get_quant_scales(t, &n));
    cr_assert_eq(n, 3);

    ros_tensor_free(t);
    ros_bytes_free(bytes, len);
    ros_tensor_builder_free(tb);
}

Test(tensor, unquantized_with_scales_is_rejected) {
    static const float scale[1] = {0.5f};
    ros_tensor_builder_t *tb = ros_tensor_builder_new();
    ros_tensor_builder_set_quant_axis(tb, -2); /* unquantized... */
    ros_tensor_builder_set_quant_scales(tb, scale, 1); /* ...but scales given */

    uint8_t *bytes = NULL;
    size_t len = 0;
    errno = 0;
    cr_assert_eq(ros_tensor_builder_build(tb, &bytes, &len), -1);
    cr_assert_eq(errno, EBADMSG);
    ros_tensor_builder_free(tb);
}

Test(tensor, per_axis_scale_count_must_match_that_axis) {
    static const uint64_t shape[2] = {3, 8};
    static const float scales[2] = {0.5f, 0.25f}; /* should be 3 */

    ros_tensor_builder_t *tb = ros_tensor_builder_new();
    ros_tensor_builder_set_shape(tb, shape, 2);
    ros_tensor_builder_set_quant_axis(tb, 0);
    ros_tensor_builder_set_quant_scales(tb, scales, 2);

    uint8_t *bytes = NULL;
    size_t len = 0;
    errno = 0;
    cr_assert_eq(ros_tensor_builder_build(tb, &bytes, &len), -1);
    cr_assert_eq(errno, EBADMSG);
    ros_tensor_builder_free(tb);
}

Test(tensor, zero_points_must_match_scales_length) {
    static const uint64_t shape[1] = {4};
    static const float scale[1] = {0.5f};
    static const int32_t zeros[2] = {1, 2}; /* one too many */

    ros_tensor_builder_t *tb = ros_tensor_builder_new();
    ros_tensor_builder_set_shape(tb, shape, 1);
    ros_tensor_builder_set_quant_axis(tb, -1);
    ros_tensor_builder_set_quant_scales(tb, scale, 1);
    ros_tensor_builder_set_quant_zero_points(tb, zeros, 2);

    uint8_t *bytes = NULL;
    size_t len = 0;
    errno = 0;
    cr_assert_eq(ros_tensor_builder_build(tb, &bytes, &len), -1);
    cr_assert_eq(errno, EBADMSG);
    ros_tensor_builder_free(tb);
}

Test(tensor, strides_must_match_shape_rank) {
    static const uint64_t shape[2] = {4, 4};
    static const int64_t strides[1] = {4}; /* rank mismatch */

    ros_tensor_builder_t *tb = ros_tensor_builder_new();
    ros_tensor_builder_set_shape(tb, shape, 2);
    ros_tensor_builder_set_strides(tb, strides, 1);

    uint8_t *bytes = NULL;
    size_t len = 0;
    errno = 0;
    cr_assert_eq(ros_tensor_builder_build(tb, &bytes, &len), -1);
    cr_assert_eq(errno, EBADMSG);
    ros_tensor_builder_free(tb);
}

/*
 * Colorimetry without a format is meaningless — the format names the encoding
 * those attributes qualify. An empty format therefore requires every
 * colorimetry field to be empty too.
 */
Test(tensor, colorimetry_without_format_is_rejected) {
    ros_tensor_builder_t *tb = ros_tensor_builder_new();
    ros_tensor_builder_set_color_space(tb, "bt709"); /* no format set */

    uint8_t *bytes = NULL;
    size_t len = 0;
    errno = 0;
    cr_assert_eq(ros_tensor_builder_build(tb, &bytes, &len), -1);
    cr_assert_eq(errno, EBADMSG);
    ros_tensor_builder_free(tb);
}

/* ------------------------------------------------------------------ */
/* Golden fixtures                                                     */
/* ------------------------------------------------------------------ */
/*
 * The .cdr files are produced by the pycdr2 dataclasses in
 * benches/python/legacy/ — an implementation of the same .msg contract that
 * is independent of this library. Decoding them here checks the C surface
 * against that second encoder rather than against itself.
 */

static uint8_t *load_fixture(const char *relpath, size_t *out_len) {
    FILE *f = fopen(relpath, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0) { fclose(f); return NULL; }
    uint8_t *buf = (uint8_t *) malloc((size_t) sz);
    if (!buf) { fclose(f); return NULL; }
    size_t got = fread(buf, 1, (size_t) sz, f);
    fclose(f);
    if (got != (size_t) sz) { free(buf); return NULL; }
    *out_len = got;
    return buf;
}

Test(tensor_golden, tensor_decodes) {
    size_t len = 0;
    uint8_t *buf = load_fixture("testdata/cdr/edgefirst_msgs/Tensor.cdr", &len);
    cr_assert_not_null(buf, "fixture missing — run scripts/generate_cdr_testdata.py");

    ros_tensor_t *t = ros_tensor_from_cdr(buf, len);
    cr_assert_not_null(t);
    cr_assert_eq(ros_tensor_get_storage_kind(t), 2);
    cr_assert_eq(ros_tensor_get_pid(t), 4242);
    cr_assert_eq(ros_tensor_get_dtype(t), 1);
    cr_assert_eq(ros_tensor_get_quant_axis(t), -2);
    cr_assert_str_eq(ros_tensor_get_format(t), "NV12");
    cr_assert_str_eq(ros_tensor_get_color_range(t), "limited");
    cr_assert_eq(ros_tensor_get_num_planes(t), 2);

    uint64_t shape[2] = {0, 0};
    cr_assert_eq(ros_tensor_copy_shape(t, shape, 2), 2);
    cr_assert_eq(shape[0], 480);
    cr_assert_eq(shape[1], 640);

    const ros_tensor_plane_t *p = ros_tensor_get_plane(t, 1);
    cr_assert_not_null(p);
    cr_assert_eq(ros_tensor_plane_get_offset(p), 640u * 480u);

    ros_tensor_free(t);
    free(buf);
}

Test(tensor_golden, inline_tensor_decodes) {
    size_t len = 0;
    uint8_t *buf = load_fixture("testdata/cdr/edgefirst_msgs/Tensor_inline.cdr", &len);
    cr_assert_not_null(buf);

    ros_tensor_t *t = ros_tensor_from_cdr(buf, len);
    cr_assert_not_null(t);
    cr_assert_str_eq(ros_tensor_get_format(t), "mono8");
    const ros_tensor_plane_t *p = ros_tensor_get_plane(t, 0);
    cr_assert(ros_tensor_plane_is_inline(p));

    size_t n = 0;
    const uint8_t *d = ros_tensor_plane_get_data(p, &n);
    cr_assert_eq(n, 8);
    for (size_t i = 0; i < n; i++) cr_assert_eq(d[i], (uint8_t) i);

    ros_tensor_free(t);
    free(buf);
}

Test(tensor_golden, quantized_tensor_decodes) {
    size_t len = 0;
    uint8_t *buf = load_fixture("testdata/cdr/edgefirst_msgs/Tensor_quantized.cdr", &len);
    cr_assert_not_null(buf);

    ros_tensor_t *t = ros_tensor_from_cdr(buf, len);
    cr_assert_not_null(t);
    cr_assert_eq(ros_tensor_get_quant_axis(t), 0);

    size_t n = 0;
    const float *sc = ros_tensor_get_quant_scales(t, &n);
    cr_assert_eq(n, 3); /* == shape[0] */
    cr_assert_float_eq(sc[0], 0.5f, 1e-6);
    cr_assert_float_eq(sc[2], 0.125f, 1e-6);

    const int32_t *zp = ros_tensor_get_quant_zero_points(t, &n);
    cr_assert_eq(n, 3);
    cr_assert_eq(zp[2], -128);

    ros_tensor_free(t);
    free(buf);
}

/* Both wrappers decode from the same golden bytes — that is what
 * byte-identical means, checked against files pycdr2 generated separately. */
Test(tensor_golden, both_wrappers_decode_either_fixture) {
    const char *paths[2] = {
        "testdata/cdr/edgefirst_msgs/TensorStamped.cdr",
        "testdata/cdr/edgefirst_msgs/CameraFrame.cdr",
    };
    for (int i = 0; i < 2; i++) {
        size_t len = 0;
        uint8_t *buf = load_fixture(paths[i], &len);
        cr_assert_not_null(buf, "fixture missing: %s", paths[i]);

        ros_camera_frame_t *f = ros_camera_frame_from_cdr(buf, len);
        cr_assert_not_null(f);
        cr_assert_eq(ros_camera_frame_get_seq(f), 99);
        cr_assert_str_eq(ros_camera_frame_get_frame_id(f), "test_frame");
        cr_assert_str_eq(ros_tensor_get_format(ros_camera_frame_get_tensor(f)), "NV12");

        ros_tensor_stamped_t *s = ros_tensor_stamped_from_cdr(buf, len);
        cr_assert_not_null(s);
        cr_assert_eq(ros_tensor_stamped_get_seq(s), 99);
        cr_assert_str_eq(ros_tensor_stamped_get_frame_id(s), "test_frame");

        ros_camera_frame_free(f);
        ros_tensor_stamped_free(s);
        free(buf);
    }
}

/*
 * Position independence against goldens: the long-frame_id fixture has a
 * different header size, so the tensor sits at a different offset — but
 * re-heading it must reproduce the standalone Tensor golden byte for byte.
 */
Test(tensor_golden, long_frame_id_reheads_to_the_standalone_golden) {
    size_t flen = 0, slen = 0;
    uint8_t *fbuf = load_fixture(
        "testdata/cdr/edgefirst_msgs/CameraFrame_long_frame_id.cdr", &flen);
    uint8_t *sbuf = load_fixture("testdata/cdr/edgefirst_msgs/Tensor.cdr", &slen);
    cr_assert_not_null(fbuf);
    cr_assert_not_null(sbuf);

    ros_camera_frame_t *f = ros_camera_frame_from_cdr(fbuf, flen);
    cr_assert_not_null(f);
    cr_assert_str_eq(ros_camera_frame_get_frame_id(f), "a_very_long_frame_identifier_x");

    uint8_t *rehead = NULL;
    size_t rlen = 0;
    cr_assert_eq(
        ros_tensor_to_standalone_cdr(ros_camera_frame_get_tensor(f), &rehead, &rlen), 0);
    cr_assert_eq(rlen, slen);
    cr_assert_eq(memcmp(rehead, sbuf, rlen), 0);

    ros_bytes_free(rehead, rlen);
    ros_camera_frame_free(f);
    free(fbuf);
    free(sbuf);
}
