/**
 * @file test_std_msgs.c
 * @brief Criterion tests for std_msgs (Header, ColorRGBA)
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include "edgefirst/schemas.h"

// ============================================================================
// Header Tests
// ============================================================================

Test(std_msgs, header_create_and_destroy) {
    RosHeader *header = ros_header_new();
    cr_assert_not_null(header, "Header creation should succeed");

    // Default stamp
    const RosTime *stamp = ros_header_get_stamp(header);
    cr_assert_not_null(stamp);

    // Default frame_id (empty)
    char *frame_id = ros_header_get_frame_id(header);
    cr_assert_not_null(frame_id);
    cr_assert_str_eq(frame_id, "", "Default frame_id should be empty");
    free(frame_id);

    ros_header_free(header);
}

Test(std_msgs, header_set_frame_id) {
    RosHeader *header = ros_header_new();
    cr_assert_not_null(header);

    int ret = ros_header_set_frame_id(header, "camera_frame");
    cr_assert_eq(ret, 0, "Setting frame_id should succeed");

    char *frame_id = ros_header_get_frame_id(header);
    cr_assert_str_eq(frame_id, "camera_frame");
    free(frame_id);

    ros_header_free(header);
}

Test(std_msgs, header_set_frame_id_long) {
    RosHeader *header = ros_header_new();
    cr_assert_not_null(header);

    char long_string[1001];
    memset(long_string, 'A', 1000);
    long_string[1000] = '\0';

    int ret = ros_header_set_frame_id(header, long_string);
    cr_assert_eq(ret, 0);

    char *frame_id = ros_header_get_frame_id(header);
    cr_assert_str_eq(frame_id, long_string);
    free(frame_id);

    ros_header_free(header);
}

Test(std_msgs, header_set_frame_id_special_chars) {
    RosHeader *header = ros_header_new();
    cr_assert_not_null(header);

    const char *special = "frame/with-special_chars.123";
    int ret = ros_header_set_frame_id(header, special);
    cr_assert_eq(ret, 0);

    char *frame_id = ros_header_get_frame_id(header);
    cr_assert_str_eq(frame_id, special);
    free(frame_id);

    ros_header_free(header);
}

Test(std_msgs, header_set_stamp) {
    RosHeader *header = ros_header_new();
    cr_assert_not_null(header);

    // Get mutable stamp and set values
    RosTime *stamp = ros_header_get_stamp_mut(header);
    ros_time_set_sec(stamp, 100);
    ros_time_set_nanosec(stamp, 200);

    // Verify via const getter
    const RosTime *const_stamp = ros_header_get_stamp(header);
    cr_assert_eq(ros_time_get_sec(const_stamp), 100);
    cr_assert_eq(ros_time_get_nanosec(const_stamp), 200);

    ros_header_free(header);
}

Test(std_msgs, header_serialize_deserialize) {
    RosHeader *original = ros_header_new();
    cr_assert_not_null(original);

    RosTime *stamp = ros_header_get_stamp_mut(original);
    ros_time_set_sec(stamp, 42);
    ros_time_set_nanosec(stamp, 999);
    ros_header_set_frame_id(original, "test_frame");

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_header_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    cr_assert_not_null(buffer);
    cr_assert_gt(len, 0);

    RosHeader *deserialized = ros_header_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    const RosTime *deser_stamp = ros_header_get_stamp(deserialized);
    cr_assert_eq(ros_time_get_sec(deser_stamp), 42);
    cr_assert_eq(ros_time_get_nanosec(deser_stamp), 999);

    char *frame_id = ros_header_get_frame_id(deserialized);
    cr_assert_str_eq(frame_id, "test_frame");
    free(frame_id);

    ros_header_free(original);
    ros_header_free(deserialized);
    free(buffer);
}

Test(std_msgs, header_serialize_null) {
    uint8_t *buffer = NULL;
    size_t len = 0;

    errno = 0;
    int ret = ros_header_serialize(NULL, &buffer, &len);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}

Test(std_msgs, header_deserialize_null) {
    errno = 0;
    RosHeader *header = ros_header_deserialize(NULL, 100);
    cr_assert_null(header);
    cr_assert_eq(errno, EINVAL);
}

Test(std_msgs, header_free_null) {
    // Should not crash when freeing NULL
    ros_header_free(NULL);
}

// ============================================================================
// ColorRGBA Tests
// ============================================================================

Test(std_msgs, colorrgba_create_and_destroy) {
    RosColorRGBA *color = ros_color_rgba_new();
    cr_assert_not_null(color);

    ros_color_rgba_set_r(color, 1.0f);
    ros_color_rgba_set_g(color, 0.5f);
    ros_color_rgba_set_b(color, 0.25f);
    ros_color_rgba_set_a(color, 0.75f);

    cr_assert_float_eq(ros_color_rgba_get_r(color), 1.0f, 0.0001f);
    cr_assert_float_eq(ros_color_rgba_get_g(color), 0.5f, 0.0001f);
    cr_assert_float_eq(ros_color_rgba_get_b(color), 0.25f, 0.0001f);
    cr_assert_float_eq(ros_color_rgba_get_a(color), 0.75f, 0.0001f);

    ros_color_rgba_free(color);
}

Test(std_msgs, colorrgba_create_zero) {
    RosColorRGBA *color = ros_color_rgba_new();
    cr_assert_not_null(color);

    // Default: RGB = 0.0 (black), Alpha = 1.0 (fully opaque)
    cr_assert_float_eq(ros_color_rgba_get_r(color), 0.0f, 0.0001f);
    cr_assert_float_eq(ros_color_rgba_get_g(color), 0.0f, 0.0001f);
    cr_assert_float_eq(ros_color_rgba_get_b(color), 0.0f, 0.0001f);
    cr_assert_float_eq(ros_color_rgba_get_a(color), 1.0f, 0.0001f);

    ros_color_rgba_free(color);
}

Test(std_msgs, colorrgba_set_values) {
    RosColorRGBA *color = ros_color_rgba_new();
    cr_assert_not_null(color);

    ros_color_rgba_set_r(color, 0.2f);
    ros_color_rgba_set_g(color, 0.4f);
    ros_color_rgba_set_b(color, 0.6f);
    ros_color_rgba_set_a(color, 0.8f);

    cr_assert_float_eq(ros_color_rgba_get_r(color), 0.2f, 0.0001f);
    cr_assert_float_eq(ros_color_rgba_get_g(color), 0.4f, 0.0001f);
    cr_assert_float_eq(ros_color_rgba_get_b(color), 0.6f, 0.0001f);
    cr_assert_float_eq(ros_color_rgba_get_a(color), 0.8f, 0.0001f);

    ros_color_rgba_free(color);
}

Test(std_msgs, colorrgba_serialize_deserialize) {
    RosColorRGBA *original = ros_color_rgba_new();
    cr_assert_not_null(original);

    ros_color_rgba_set_r(original, 0.1f);
    ros_color_rgba_set_g(original, 0.2f);
    ros_color_rgba_set_b(original, 0.3f);
    ros_color_rgba_set_a(original, 0.4f);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_color_rgba_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    cr_assert_not_null(buffer);
    cr_assert_gt(len, 0);

    RosColorRGBA *deserialized = ros_color_rgba_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_float_eq(ros_color_rgba_get_r(deserialized), 0.1f, 0.0001f);
    cr_assert_float_eq(ros_color_rgba_get_g(deserialized), 0.2f, 0.0001f);
    cr_assert_float_eq(ros_color_rgba_get_b(deserialized), 0.3f, 0.0001f);
    cr_assert_float_eq(ros_color_rgba_get_a(deserialized), 0.4f, 0.0001f);

    ros_color_rgba_free(original);
    ros_color_rgba_free(deserialized);
    free(buffer);
}

Test(std_msgs, colorrgba_free_null) {
    // Should not crash when freeing NULL
    ros_color_rgba_free(NULL);
}
