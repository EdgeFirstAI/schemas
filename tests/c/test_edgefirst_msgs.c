/**
 * @file test_edgefirst_msgs.c
 * @brief Criterion tests for EdgeFirst custom messages (Detect, Mask, etc.)
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <string.h>
#include "edgefirst/schemas.h"

// ============================================================================
// DetectTrack Tests
// ============================================================================

Test(edgefirst_msgs, detect_track_create_and_destroy) {
    DetectTrack *track = edgefirst_detect_track_create(42, 10);
    cr_assert_not_null(track);
    
    cr_assert_eq(edgefirst_detect_track_get_id(track), 42);
    cr_assert_eq(edgefirst_detect_track_get_age(track), 10);
    
    edgefirst_detect_track_destroy(track);
}

Test(edgefirst_msgs, detect_track_set_values) {
    DetectTrack *track = edgefirst_detect_track_create(0, 0);
    cr_assert_not_null(track);
    
    edgefirst_detect_track_set_id(track, 999);
    edgefirst_detect_track_set_age(track, 5);
    
    cr_assert_eq(edgefirst_detect_track_get_id(track), 999);
    cr_assert_eq(edgefirst_detect_track_get_age(track), 5);
    
    edgefirst_detect_track_destroy(track);
}

Test(edgefirst_msgs, detect_track_serialize_deserialize) {
    DetectTrack *original = edgefirst_detect_track_create(123, 45);
    cr_assert_not_null(original);
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_detect_track_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    cr_assert_not_null(buffer);
    cr_assert_gt(len, 0);
    
    DetectTrack *deserialized = edgefirst_detect_track_deserialize(buffer, len);
    cr_assert_not_null(deserialized);
    
    cr_assert_eq(edgefirst_detect_track_get_id(deserialized), 123);
    cr_assert_eq(edgefirst_detect_track_get_age(deserialized), 45);
    
    edgefirst_detect_track_destroy(original);
    edgefirst_detect_track_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}

// ============================================================================
// DetectBox2D Tests
// ============================================================================

Test(edgefirst_msgs, detect_box2d_create_and_destroy) {
    DetectBox2D *box = edgefirst_detect_box2d_create(0.5, 0.5, 0.2, 0.3);
    cr_assert_not_null(box);
    
    cr_assert_float_eq(edgefirst_detect_box2d_get_center_x(box), 0.5, 0.0001);
    cr_assert_float_eq(edgefirst_detect_box2d_get_center_y(box), 0.5, 0.0001);
    cr_assert_float_eq(edgefirst_detect_box2d_get_width(box), 0.2, 0.0001);
    cr_assert_float_eq(edgefirst_detect_box2d_get_height(box), 0.3, 0.0001);
    
    edgefirst_detect_box2d_destroy(box);
}

Test(edgefirst_msgs, detect_box2d_normalized_coords) {
    // Test that normalized coordinates (0.0-1.0) work correctly
    DetectBox2D *box = edgefirst_detect_box2d_create(0.25, 0.75, 0.1, 0.15);
    cr_assert_not_null(box);
    
    cr_assert_float_eq(edgefirst_detect_box2d_get_center_x(box), 0.25, 0.0001);
    cr_assert_float_eq(edgefirst_detect_box2d_get_center_y(box), 0.75, 0.0001);
    
    edgefirst_detect_box2d_destroy(box);
}

Test(edgefirst_msgs, detect_box2d_set_values) {
    DetectBox2D *box = edgefirst_detect_box2d_create(0.0, 0.0, 0.0, 0.0);
    cr_assert_not_null(box);
    
    edgefirst_detect_box2d_set_center_x(box, 0.6);
    edgefirst_detect_box2d_set_center_y(box, 0.4);
    edgefirst_detect_box2d_set_width(box, 0.3);
    edgefirst_detect_box2d_set_height(box, 0.2);
    
    cr_assert_float_eq(edgefirst_detect_box2d_get_center_x(box), 0.6, 0.0001);
    cr_assert_float_eq(edgefirst_detect_box2d_get_center_y(box), 0.4, 0.0001);
    cr_assert_float_eq(edgefirst_detect_box2d_get_width(box), 0.3, 0.0001);
    cr_assert_float_eq(edgefirst_detect_box2d_get_height(box), 0.2, 0.0001);
    
    edgefirst_detect_box2d_destroy(box);
}

Test(edgefirst_msgs, detect_box2d_serialize_deserialize) {
    DetectBox2D *original = edgefirst_detect_box2d_create(0.3, 0.7, 0.15, 0.25);
    cr_assert_not_null(original);
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_detect_box2d_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    
    DetectBox2D *deserialized = edgefirst_detect_box2d_deserialize(buffer, len);
    cr_assert_not_null(deserialized);
    
    cr_assert_float_eq(edgefirst_detect_box2d_get_center_x(deserialized), 0.3, 0.0001);
    cr_assert_float_eq(edgefirst_detect_box2d_get_center_y(deserialized), 0.7, 0.0001);
    cr_assert_float_eq(edgefirst_detect_box2d_get_width(deserialized), 0.15, 0.0001);
    cr_assert_float_eq(edgefirst_detect_box2d_get_height(deserialized), 0.25, 0.0001);
    
    edgefirst_detect_box2d_destroy(original);
    edgefirst_detect_box2d_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}

// ============================================================================
// Detect Tests
// ============================================================================

Test(edgefirst_msgs, detect_create_and_destroy) {
    Detect *detect = edgefirst_detect_create();
    cr_assert_not_null(detect);
    
    Header *header = edgefirst_detect_get_header(detect);
    cr_assert_not_null(header);
    
    size_t box_count = 0;
    const DetectBox2D *const *boxes = edgefirst_detect_get_boxes(detect, &box_count);
    cr_assert_eq(box_count, 0, "Default should have no boxes");
    
    edgefirst_detect_destroy(detect);
}

Test(edgefirst_msgs, detect_set_label) {
    Detect *detect = edgefirst_detect_create();
    cr_assert_not_null(detect);
    
    int ret = edgefirst_detect_set_label(detect, "person");
    cr_assert_eq(ret, 0);
    
    const char *label = edgefirst_detect_get_label(detect);
    cr_assert_str_eq(label, "person");
    
    edgefirst_detect_destroy(detect);
}

Test(edgefirst_msgs, detect_set_score) {
    Detect *detect = edgefirst_detect_create();
    cr_assert_not_null(detect);
    
    edgefirst_detect_set_score(detect, 0.95);
    
    cr_assert_float_eq(edgefirst_detect_get_score(detect), 0.95, 0.0001);
    
    edgefirst_detect_destroy(detect);
}

Test(edgefirst_msgs, detect_serialize_deserialize_no_boxes) {
    Detect *original = edgefirst_detect_create();
    cr_assert_not_null(original);
    
    edgefirst_detect_set_label(original, "car");
    edgefirst_detect_set_score(original, 0.88);
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_detect_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    
    Detect *deserialized = edgefirst_detect_deserialize(buffer, len);
    cr_assert_not_null(deserialized);
    
    const char *label = edgefirst_detect_get_label(deserialized);
    cr_assert_str_eq(label, "car");
    
    cr_assert_float_eq(edgefirst_detect_get_score(deserialized), 0.88, 0.0001);
    
    edgefirst_detect_destroy(original);
    edgefirst_detect_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}

// ============================================================================
// Mask Tests
// ============================================================================

Test(edgefirst_msgs, mask_create_and_destroy) {
    Mask *mask = edgefirst_mask_create();
    cr_assert_not_null(mask);
    
    Header *header = edgefirst_mask_get_header(mask);
    cr_assert_not_null(header);
    
    cr_assert_eq(edgefirst_mask_get_height(mask), 0);
    cr_assert_eq(edgefirst_mask_get_width(mask), 0);
    cr_assert_eq(edgefirst_mask_get_length(mask), 0);
    
    edgefirst_mask_destroy(mask);
}

Test(edgefirst_msgs, mask_set_dimensions) {
    Mask *mask = edgefirst_mask_create();
    cr_assert_not_null(mask);
    
    edgefirst_mask_set_height(mask, 480);
    edgefirst_mask_set_width(mask, 640);
    edgefirst_mask_set_length(mask, 100);
    
    cr_assert_eq(edgefirst_mask_get_height(mask), 480);
    cr_assert_eq(edgefirst_mask_get_width(mask), 640);
    cr_assert_eq(edgefirst_mask_get_length(mask), 100);
    
    edgefirst_mask_destroy(mask);
}

Test(edgefirst_msgs, mask_set_encoding) {
    Mask *mask = edgefirst_mask_create();
    cr_assert_not_null(mask);
    
    int ret = edgefirst_mask_set_encoding(mask, "zstd");
    cr_assert_eq(ret, 0);
    
    const char *encoding = edgefirst_mask_get_encoding(mask);
    cr_assert_str_eq(encoding, "zstd");
    
    edgefirst_mask_destroy(mask);
}

Test(edgefirst_msgs, mask_set_encoding_empty) {
    Mask *mask = edgefirst_mask_create();
    cr_assert_not_null(mask);
    
    int ret = edgefirst_mask_set_encoding(mask, "");
    cr_assert_eq(ret, 0);
    
    const char *encoding = edgefirst_mask_get_encoding(mask);
    cr_assert_str_eq(encoding, "", "Empty encoding means raw data");
    
    edgefirst_mask_destroy(mask);
}

Test(edgefirst_msgs, mask_set_data) {
    Mask *mask = edgefirst_mask_create();
    cr_assert_not_null(mask);
    
    uint8_t test_data[] = {1, 2, 3, 4, 5};
    int ret = edgefirst_mask_set_data(mask, test_data, 5);
    cr_assert_eq(ret, 0);
    
    size_t data_len = 0;
    const uint8_t *data = edgefirst_mask_get_data(mask, &data_len);
    cr_assert_eq(data_len, 5);
    cr_assert_neq(data, NULL);
    
    for (size_t i = 0; i < 5; i++) {
        cr_assert_eq(data[i], test_data[i]);
    }
    
    edgefirst_mask_destroy(mask);
}

Test(edgefirst_msgs, mask_serialize_deserialize_no_data) {
    Mask *original = edgefirst_mask_create();
    cr_assert_not_null(original);
    
    edgefirst_mask_set_height(original, 100);
    edgefirst_mask_set_width(original, 200);
    edgefirst_mask_set_encoding(original, "");
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_mask_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    
    Mask *deserialized = edgefirst_mask_deserialize(buffer, len);
    cr_assert_not_null(deserialized);
    
    cr_assert_eq(edgefirst_mask_get_height(deserialized), 100);
    cr_assert_eq(edgefirst_mask_get_width(deserialized), 200);
    
    const char *encoding = edgefirst_mask_get_encoding(deserialized);
    cr_assert_str_eq(encoding, "");
    
    edgefirst_mask_destroy(original);
    edgefirst_mask_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}

Test(edgefirst_msgs, mask_serialize_deserialize_with_data) {
    Mask *original = edgefirst_mask_create();
    cr_assert_not_null(original);
    
    edgefirst_mask_set_height(original, 10);
    edgefirst_mask_set_width(original, 10);
    
    uint8_t test_data[100];
    for (int i = 0; i < 100; i++) {
        test_data[i] = (uint8_t)(i % 256);
    }
    edgefirst_mask_set_data(original, test_data, 100);
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_mask_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    
    Mask *deserialized = edgefirst_mask_deserialize(buffer, len);
    cr_assert_not_null(deserialized);
    
    size_t data_len = 0;
    const uint8_t *data = edgefirst_mask_get_data(deserialized, &data_len);
    cr_assert_eq(data_len, 100);
    
    for (size_t i = 0; i < 100; i++) {
        cr_assert_eq(data[i], test_data[i]);
    }
    
    edgefirst_mask_destroy(original);
    edgefirst_mask_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}

// ============================================================================
// DmaBuf Tests
// ============================================================================

Test(edgefirst_msgs, dmabuf_create_and_destroy) {
    DmaBuf *dmabuf = edgefirst_dmabuf_create();
    cr_assert_not_null(dmabuf);
    
    Header *header = edgefirst_dmabuf_get_header(dmabuf);
    cr_assert_not_null(header);
    
    edgefirst_dmabuf_destroy(dmabuf);
}

Test(edgefirst_msgs, dmabuf_set_dimensions) {
    DmaBuf *dmabuf = edgefirst_dmabuf_create();
    cr_assert_not_null(dmabuf);
    
    edgefirst_dmabuf_set_height(dmabuf, 1080);
    edgefirst_dmabuf_set_width(dmabuf, 1920);
    
    cr_assert_eq(edgefirst_dmabuf_get_height(dmabuf), 1080);
    cr_assert_eq(edgefirst_dmabuf_get_width(dmabuf), 1920);
    
    edgefirst_dmabuf_destroy(dmabuf);
}

Test(edgefirst_msgs, dmabuf_set_fourcc) {
    DmaBuf *dmabuf = edgefirst_dmabuf_create();
    cr_assert_not_null(dmabuf);
    
    // Test with YUYV fourcc code (0x56595559)
    edgefirst_dmabuf_set_fourcc(dmabuf, 0x56595559);
    
    cr_assert_eq(edgefirst_dmabuf_get_fourcc(dmabuf), 0x56595559);
    
    edgefirst_dmabuf_destroy(dmabuf);
}

Test(edgefirst_msgs, dmabuf_serialize_deserialize) {
    DmaBuf *original = edgefirst_dmabuf_create();
    cr_assert_not_null(original);
    
    edgefirst_dmabuf_set_height(original, 720);
    edgefirst_dmabuf_set_width(original, 1280);
    edgefirst_dmabuf_set_fourcc(original, 0x34325247); // 'RG24'
    edgefirst_dmabuf_set_fd(original, 42);
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_dmabuf_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    
    DmaBuf *deserialized = edgefirst_dmabuf_deserialize(buffer, len);
    cr_assert_not_null(deserialized);
    
    cr_assert_eq(edgefirst_dmabuf_get_height(deserialized), 720);
    cr_assert_eq(edgefirst_dmabuf_get_width(deserialized), 1280);
    cr_assert_eq(edgefirst_dmabuf_get_fourcc(deserialized), 0x34325247);
    cr_assert_eq(edgefirst_dmabuf_get_fd(deserialized), 42);
    
    edgefirst_dmabuf_destroy(original);
    edgefirst_dmabuf_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}
