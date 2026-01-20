/**
 * @file test_service.c
 * @brief Criterion tests for service types
 */

#include <criterion/criterion.h>
#include <errno.h>
#include <stdlib.h>
#include "edgefirst/schemas.h"

// ============================================================================
// ServiceHeader Tests
// ============================================================================

Test(service, service_header_create_and_destroy) {
    RosServiceHeader *header = ros_service_header_new();
    cr_assert_not_null(header);

    ros_service_header_set_guid(header, 0x1234567890ABCDEF);
    ros_service_header_set_seq(header, 42);

    cr_assert_eq(ros_service_header_get_guid(header), 0x1234567890ABCDEF);
    cr_assert_eq(ros_service_header_get_seq(header), 42);

    ros_service_header_free(header);
}

Test(service, service_header_default_zero) {
    RosServiceHeader *header = ros_service_header_new();
    cr_assert_not_null(header);

    cr_assert_eq(ros_service_header_get_guid(header), 0);
    cr_assert_eq(ros_service_header_get_seq(header), 0);

    ros_service_header_free(header);
}

Test(service, service_header_negative_guid) {
    RosServiceHeader *header = ros_service_header_new();
    cr_assert_not_null(header);

    ros_service_header_set_guid(header, -1);
    cr_assert_eq(ros_service_header_get_guid(header), -1);

    ros_service_header_set_guid(header, -9223372036854775807LL);
    cr_assert_eq(ros_service_header_get_guid(header), -9223372036854775807LL);

    ros_service_header_free(header);
}

Test(service, service_header_large_seq) {
    RosServiceHeader *header = ros_service_header_new();
    cr_assert_not_null(header);

    ros_service_header_set_seq(header, 18446744073709551615ULL);
    cr_assert_eq(ros_service_header_get_seq(header), 18446744073709551615ULL);

    ros_service_header_free(header);
}

Test(service, service_header_serialize_deserialize) {
    RosServiceHeader *original = ros_service_header_new();
    cr_assert_not_null(original);

    ros_service_header_set_guid(original, 987654321);
    ros_service_header_set_seq(original, 100);

    uint8_t *buffer = NULL;
    size_t len = 0;

    int ret = ros_service_header_serialize(original, &buffer, &len);
    cr_assert_eq(ret, 0);
    cr_assert_not_null(buffer);
    cr_assert_gt(len, 0);

    RosServiceHeader *deserialized = ros_service_header_deserialize(buffer, len);
    cr_assert_not_null(deserialized);

    cr_assert_eq(ros_service_header_get_guid(deserialized), 987654321);
    cr_assert_eq(ros_service_header_get_seq(deserialized), 100);

    ros_service_header_free(original);
    ros_service_header_free(deserialized);
    free(buffer);
}

Test(service, service_header_free_null) {
    ros_service_header_free(NULL);
}

Test(service, service_header_get_guid_null) {
    cr_assert_eq(ros_service_header_get_guid(NULL), 0);
}

Test(service, service_header_get_seq_null) {
    cr_assert_eq(ros_service_header_get_seq(NULL), 0);
}

Test(service, service_header_deserialize_null) {
    errno = 0;
    RosServiceHeader *header = ros_service_header_deserialize(NULL, 10);
    cr_assert_null(header);
    cr_assert_eq(errno, EINVAL);
}

Test(service, service_header_deserialize_empty) {
    uint8_t buffer[1] = {0};
    errno = 0;
    RosServiceHeader *header = ros_service_header_deserialize(buffer, 0);
    cr_assert_null(header);
    cr_assert_eq(errno, EINVAL);
}

Test(service, service_header_serialize_null) {
    uint8_t *buffer = NULL;
    size_t len = 0;

    errno = 0;
    int ret = ros_service_header_serialize(NULL, &buffer, &len);
    cr_assert_eq(ret, -1);
    cr_assert_eq(errno, EINVAL);
}
