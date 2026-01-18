/**
 * @file test_std_msgs.c
 * @brief Criterion tests for std_msgs (Header, ColorRGBA)
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <string.h>
#include "edgefirst/schemas.h"

// ============================================================================
// Header Tests
// ============================================================================

Test(std_msgs, header_create_and_destroy) {
    Header *header = edgefirst_header_create();
    cr_assert_not_null(header, "Header creation should succeed");
    
    // Default values
    Time *stamp = edgefirst_header_get_stamp(header);
    cr_assert_not_null(stamp);
    
    const char *frame_id = edgefirst_header_get_frame_id(header);
    cr_assert_not_null(frame_id);
    cr_assert_str_eq(frame_id, "", "Default frame_id should be empty");
    
    edgefirst_header_destroy(header);
}

Test(std_msgs, header_set_frame_id) {
    Header *header = edgefirst_header_create();
    cr_assert_not_null(header);
    
    int ret = edgefirst_header_set_frame_id(header, "camera_frame");
    cr_assert_eq(ret, 0, "Setting frame_id should succeed");
    
    const char *frame_id = edgefirst_header_get_frame_id(header);
    cr_assert_str_eq(frame_id, "camera_frame");
    
    edgefirst_header_destroy(header);
}

Test(std_msgs, header_set_frame_id_long) {
    Header *header = edgefirst_header_create();
    cr_assert_not_null(header);
    
    char long_string[1001];
    memset(long_string, 'A', 1000);
    long_string[1000] = '\0';
    
    int ret = edgefirst_header_set_frame_id(header, long_string);
    cr_assert_eq(ret, 0);
    
    const char *frame_id = edgefirst_header_get_frame_id(header);
    cr_assert_str_eq(frame_id, long_string);
    
    edgefirst_header_destroy(header);
}

Test(std_msgs, header_set_frame_id_special_chars) {
    Header *header = edgefirst_header_create();
    cr_assert_not_null(header);
    
    const char *special = "frame/with-special_chars.123";
    int ret = edgefirst_header_set_frame_id(header, special);
    cr_assert_eq(ret, 0);
    
    const char *frame_id = edgefirst_header_get_frame_id(header);
    cr_assert_str_eq(frame_id, special);
    
    edgefirst_header_destroy(header);
}

Test(std_msgs, header_set_stamp) {
    Header *header = edgefirst_header_create();
    cr_assert_not_null(header);
    
    Time *new_stamp = edgefirst_time_create(100, 200);
    edgefirst_header_set_stamp(header, new_stamp);
    
    Time *stamp = edgefirst_header_get_stamp(header);
    cr_assert_eq(edgefirst_time_get_sec(stamp), 100);
    cr_assert_eq(edgefirst_time_get_nanosec(stamp), 200);
    
    edgefirst_time_destroy(new_stamp);
    edgefirst_header_destroy(header);
}

Test(std_msgs, header_serialize_deserialize) {
    Header *original = edgefirst_header_create();
    cr_assert_not_null(original);
    
    Time *stamp = edgefirst_time_create(42, 999);
    edgefirst_header_set_stamp(original, stamp);
    edgefirst_header_set_frame_id(original, "test_frame");
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_header_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    cr_assert_not_null(buffer);
    cr_assert_gt(len, 0);
    
    Header *deserialized = edgefirst_header_deserialize(buffer, len);
    cr_assert_not_null(deserialized);
    
    Time *deser_stamp = edgefirst_header_get_stamp(deserialized);
    cr_assert_eq(edgefirst_time_get_sec(deser_stamp), 42);
    cr_assert_eq(edgefirst_time_get_nanosec(deser_stamp), 999);
    
    const char *frame_id = edgefirst_header_get_frame_id(deserialized);
    cr_assert_str_eq(frame_id, "test_frame");
    
    edgefirst_time_destroy(stamp);
    edgefirst_header_destroy(original);
    edgefirst_header_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}

Test(std_msgs, header_serialize_null) {
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    errno = 0;
    int ret = edgefirst_header_serialize(NULL, &buffer, &len);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

Test(std_msgs, header_deserialize_null) {
    errno = 0;
    Header *header = edgefirst_header_deserialize(NULL, 100);
    cr_assert_null(header);
    cr_assert_eq(errno, EINVAL);
}

// ============================================================================
// ColorRGBA Tests
// ============================================================================

Test(std_msgs, colorrgba_create_and_destroy) {
    ColorRGBA *color = edgefirst_colorrgba_create(1.0f, 0.5f, 0.25f, 0.75f);
    cr_assert_not_null(color);
    
    cr_assert_float_eq(edgefirst_colorrgba_get_r(color), 1.0f, 0.0001f);
    cr_assert_float_eq(edgefirst_colorrgba_get_g(color), 0.5f, 0.0001f);
    cr_assert_float_eq(edgefirst_colorrgba_get_b(color), 0.25f, 0.0001f);
    cr_assert_float_eq(edgefirst_colorrgba_get_a(color), 0.75f, 0.0001f);
    
    edgefirst_colorrgba_destroy(color);
}

Test(std_msgs, colorrgba_create_zero) {
    ColorRGBA *color = edgefirst_colorrgba_create(0.0f, 0.0f, 0.0f, 0.0f);
    cr_assert_not_null(color);
    
    cr_assert_float_eq(edgefirst_colorrgba_get_r(color), 0.0f, 0.0001f);
    cr_assert_float_eq(edgefirst_colorrgba_get_a(color), 0.0f, 0.0001f);
    
    edgefirst_colorrgba_destroy(color);
}

Test(std_msgs, colorrgba_set_values) {
    ColorRGBA *color = edgefirst_colorrgba_create(0.0f, 0.0f, 0.0f, 0.0f);
    cr_assert_not_null(color);
    
    edgefirst_colorrgba_set_r(color, 0.2f);
    edgefirst_colorrgba_set_g(color, 0.4f);
    edgefirst_colorrgba_set_b(color, 0.6f);
    edgefirst_colorrgba_set_a(color, 0.8f);
    
    cr_assert_float_eq(edgefirst_colorrgba_get_r(color), 0.2f, 0.0001f);
    cr_assert_float_eq(edgefirst_colorrgba_get_g(color), 0.4f, 0.0001f);
    cr_assert_float_eq(edgefirst_colorrgba_get_b(color), 0.6f, 0.0001f);
    cr_assert_float_eq(edgefirst_colorrgba_get_a(color), 0.8f, 0.0001f);
    
    edgefirst_colorrgba_destroy(color);
}

Test(std_msgs, colorrgba_serialize_deserialize) {
    ColorRGBA *original = edgefirst_colorrgba_create(0.1f, 0.2f, 0.3f, 0.4f);
    cr_assert_not_null(original);
    
    uint8_t *buffer = NULL;
    size_t len = 0;
    
    int ret = edgefirst_colorrgba_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    cr_assert_not_null(buffer);
    cr_assert_gt(len, 0);
    
    ColorRGBA *deserialized = edgefirst_colorrgba_deserialize(buffer, len);
    cr_assert_not_null(deserialized);
    
    cr_assert_float_eq(edgefirst_colorrgba_get_r(deserialized), 0.1f, 0.0001f);
    cr_assert_float_eq(edgefirst_colorrgba_get_g(deserialized), 0.2f, 0.0001f);
    cr_assert_float_eq(edgefirst_colorrgba_get_b(deserialized), 0.3f, 0.0001f);
    cr_assert_float_eq(edgefirst_colorrgba_get_a(deserialized), 0.4f, 0.0001f);
    
    edgefirst_colorrgba_destroy(original);
    edgefirst_colorrgba_destroy(deserialized);
    edgefirst_buffer_destroy(buffer);
}
