/**
 * @file schemas.hpp
 * @brief EdgeFirst Schemas — modern C++17 header-only wrapper.
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2025 Au-Zone Technologies. All Rights Reserved.
 *
 * Header-only C++ wrapper around `<edgefirst/schemas.h>`, providing RAII
 * lifecycle management, move-only semantics, `expected<T, Error>` error
 * handling, zero-copy field access, and range-based iteration for array
 * children — all with zero runtime overhead versus the underlying C API.
 *
 * The wrapper links against the same `libedgefirst_schemas` shared library
 * as the C API; no additional build targets are required.
 *
 * @section ownership Ownership model
 *
 * The wrapper exposes three type categories per buffer-backed message:
 *
 * - **View types** (`HeaderView`, `ImageView`, `DetectView`, …) — non-owning,
 *   move-only references into a CDR byte buffer supplied by the caller. They
 *   are constructed via static `from_cdr(span<const std::uint8_t>)` factories
 *   returning `expected<ViewT, Error>`. The caller's backing buffer must
 *   outlive the view and every `std::string_view` / `span<const std::uint8_t>`
 *   accessor return value.
 *
 * - **Owning types** (`Header`, `Image`, `Mask`, `CompressedImage`,
 *   `CompressedVideo`, `DmaBuffer`) — move-only handles that encode a fresh
 *   CDR buffer at construction time via a static `encode(...)` factory.
 *   They own both the encoded bytes and a view handle over them; their
 *   accessors are safe to use as long as the owning instance is alive.
 *
 * - **Builder types** (`HeaderBuilder`, `ImuBuilder`, `DetectBuilder`, …) —
 *   move-only RAII wrappers around the C builder API. Created via a static
 *   `create()` factory returning `expected<Builder, Error>`. Fluent
 *   setter methods allow field-by-field message construction; `build()`
 *   allocates and encodes a CDR buffer (returned as `Released`), while
 *   `encode_into(span<uint8_t>)` writes into a caller-provided buffer
 *   for zero-allocation publishing. Builders are available for all message
 *   types that have a C builder API (27 types).
 *
 * A few types have only a view and no owning `encode()` factory. These
 * can still be constructed via their corresponding builder. The only
 * truly read-only type is `OdometryView`, which has neither an owning
 * type nor a builder in the C API.
 *
 * @section errors Error handling
 *
 * All fallible operations return `expected<T, Error>`. On C++23+ this is an
 * alias for `std::expected`; on older standards it aliases to the vendored
 * `edgefirst::stdlib::expected` backport (a renamed copy of `tl::expected`).
 * The Error struct carries the POSIX errno code and a static
 * `std::string_view` naming the underlying C function that failed.
 * Exceptions are never thrown by the wrapper; the C library's errno/NULL
 * conventions are mapped uniformly to `expected`.
 *
 * @section zerocopy Zero-copy guarantees
 *
 * Field accessors returning `std::string_view` or `span<const std::uint8_t>`
 * borrow directly into the backing CDR buffer. No bytes are copied. The
 * lifetime of these borrowed references is tied to the view or owning
 * instance, which is in turn tied to the backing buffer. Violating this
 * chain (e.g., destroying the buffer while a view still holds a
 * `std::string_view` from it) is undefined behavior.
 *
 * Array-valued children of parent messages (`DetectView::boxes()`,
 * `ModelView::boxes()`, `ModelView::masks()`) are exposed via a range
 * adaptor that yields lightweight borrowed handles. Each yielded handle is
 * valid only while the parent view is alive.
 *
 * @section threading Thread safety
 *
 * Distinct wrapper instances are safe to use concurrently. A single instance
 * should not be accessed from multiple threads without external
 * synchronization. `errno` is thread-local, so error reporting is
 * thread-safe under the usual POSIX conventions.
 *
 * @section allocation Allocation characteristics
 *
 * View types (`from_cdr`) are allocation-free aside from a single small
 * heap box for the opaque C handle — field accessors cost zero additional
 * allocations.
 *
 * Owning types allocate exactly **one** fresh CDR byte buffer per
 * `encode(...)` call, sized to the serialised message. The buffer is
 * owned by the wrapper and freed by its destructor via `ros_bytes_free`.
 * At high publish rates (for example a 30 Hz compressed-image stream),
 * this produces allocator churn proportional to `rate × message_size`.
 * Applications on allocator-sensitive paths should be aware of this and
 * either reuse encoded instances when possible or transfer ownership out
 * of the wrapper with `release()` to hand the buffer to a downstream
 * sink without an intermediate copy (see the publishing example below).
 *
 * Builder types offer an `encode_into(span<uint8_t>)` method that encodes
 * directly into a caller-provided buffer, eliminating per-message
 * allocation entirely. This is the recommended path for latency-sensitive
 * or allocation-sensitive code.
 *
 * @section interop Interop with zenoh-cpp and raw buffers
 *
 * `ViewT::from_cdr(span<const std::uint8_t>)` accepts any
 * `edgefirst::schemas::span` (aliased to `std::span` on C++20+). Zenoh-cpp
 * payloads can be passed directly:
 *
 * @code{.cpp}
 * void on_image(std::span<const std::uint8_t> payload) {  // from zenoh
 *     namespace ef = edgefirst::schemas;
 *     auto img = ef::ImageView::from_cdr({payload.data(), payload.size()});
 *     if (!img) return;
 *     std::string_view enc = img->encoding();  // borrows zenoh buffer
 *     // ... process ...
 * }
 * @endcode
 *
 * For **publishing** a freshly-encoded message, use `release()` on the
 * owning wrapper to transfer the encoded buffer into a `zenoh::Bytes`
 * (or any sink that takes a `(ptr, size, deleter)` tuple). This avoids
 * copying the encoded bytes and lets zenoh's own reference count drive
 * the buffer lifetime:
 *
 * @code{.cpp}
 * namespace ef = edgefirst::schemas;
 * auto img = ef::CompressedImage::encode(stamp, "cam", "jpeg", jpeg_bytes);
 * if (!img) return;
 * auto owned = std::move(*img).release();   // transfer (ptr, size) out
 * zenoh::Bytes payload{
 *     owned.data, owned.size,
 *     [size = owned.size](std::uint8_t* p) { ros_bytes_free(p, size); }
 * };
 * publisher.put(std::move(payload));
 * @endcode
 *
 * **Builder-based publishing** (zero allocation with `encode_into`):
 *
 * @code{.cpp}
 * namespace ef = edgefirst::schemas;
 * auto b = ef::ImuBuilder::create();
 * if (!b) return;
 * b->stamp({now_s, now_ns}).orientation({qx, qy, qz, qw})
 *   .angular_velocity({gx, gy, gz}).linear_acceleration({ax, ay, az});
 * b->frame_id("imu_link");
 *
 * std::array<std::uint8_t, 512> buf;
 * auto len = b->encode_into({buf.data(), buf.size()});
 * if (!len) return;
 * publisher.put({buf.data(), *len});
 * @endcode
 */

#ifndef EDGEFIRST_SCHEMAS_HPP
#define EDGEFIRST_SCHEMAS_HPP

#include <array>
#include <cstdint>
#include <cstddef>
#include <cerrno>
#include <iterator>
#include <string_view>

#include <edgefirst/schemas.h>
#include <edgefirst/stdlib/expected.hpp>
#include <edgefirst/stdlib/span.hpp>

namespace edgefirst::schemas {

// Re-export the stdlib polyfills as first-class edgefirst::schemas types.
// On C++23+ these alias to std::expected / std::span from the standard
// library; on older standards they alias to the vendored implementations
// in include/edgefirst/stdlib/. See include/edgefirst/stdlib/expected.hpp
// and include/edgefirst/stdlib/span.hpp for the conditional selection logic.

/// @brief `std::expected`-like sum type used for all fallible wrapper
///        operations.
///
/// Aliased to `std::expected<T, E>` on C++23+ or to the vendored
/// `edgefirst::stdlib::expected<T, E>` (a renamed `tl::expected`) on
/// earlier standards. Wrapper APIs never throw: errors are surfaced as
/// `unexpected<Error>` values.
template <class T, class E>
using expected = ::edgefirst::stdlib::expected<T, E>;

/// @brief Error wrapper returned by fallible wrapper operations on
///        failure.
///
/// Aliased to `std::unexpected` on C++23+ or to the vendored
/// `edgefirst::stdlib::unexpected` fallback on earlier standards.
template <class E>
using unexpected = ::edgefirst::stdlib::unexpected<E>;

/// @brief Non-owning view over a contiguous sequence of `T`.
///
/// Aliased to `std::span<T>` on C++20+ or to the vendored
/// `edgefirst::stdlib::span<T>` fallback on C++17. Used throughout the
/// wrapper for byte-buffer inputs and blob/array accessors.
template <class T>
using span = ::edgefirst::stdlib::span<T>;

/**
 * @brief Error information returned by fallible wrapper operations.
 *
 * Matches the POSIX errno conventions of the underlying C API. The
 * `where` field is a static `std::string_view` naming the C function
 * that failed, suitable for logging without allocation.
 */
struct Error {
    /// POSIX errno code — typically EINVAL, ENOBUFS, or EBADMSG.
    int code;
    /// Static string naming the C function that reported the error
    /// (e.g., "ros_image_from_cdr"). Always has static storage duration.
    std::string_view where;

    /// @brief Human-readable category label for `code`.
    /// @return A static string like "invalid argument", "buffer too
    ///         small", "bad message", or "unknown error".
    [[nodiscard]] std::string_view category() const noexcept {
        switch (code) {
        case EINVAL:  return "invalid argument";
        case ENOBUFS: return "buffer too small";
        case EBADMSG: return "bad message";
        default:      return "unknown error";
        }
    }

    /// @brief Construct an Error from the thread-local `errno` captured
    ///        immediately after a failed C call.
    /// @param w Static string literal naming the C function that failed.
    /// @return `Error{errno, w}`.
    static Error from_errno(std::string_view w) noexcept {
        return {errno, w};
    }
};

/**
 * @brief Raw ownership of an encoded CDR byte buffer transferred out of
 *        an owning wrapper via `release()`.
 *
 * The buffer was originally allocated by the C library's encode path
 * (`ros_<type>_encode`) and MUST be freed via `ros_bytes_free(data, size)`
 * OR handed to a sink that takes ownership (for example
 * `zenoh::Bytes` with a deleter lambda). Dropping a `Released` without
 * freeing `data` leaks the buffer — this is a POD by design so that
 * ownership transfer to C-style APIs is a trivial value copy.
 *
 * Typical use:
 * @code{.cpp}
 * auto img = ef::CompressedImage::encode(stamp, "cam", "jpeg", jpeg_span);
 * if (!img) return;
 * auto owned = std::move(*img).release();   // transfer ownership out
 * zenoh::Bytes payload{
 *     owned.data, owned.size,
 *     [size = owned.size](std::uint8_t* p) { ros_bytes_free(p, size); }
 * };
 * publisher.put(std::move(payload));
 * @endcode
 *
 * After `release()` the source owning wrapper is empty; its destructor
 * is a safe no-op and further accessor calls are undefined behavior
 * (it is moved-from).
 */
struct Released {
    /// Raw pointer to the encoded CDR bytes. `nullptr` when the source
    /// wrapper was moved-from or already released.
    std::uint8_t* data{nullptr};
    /// Size of the buffer in bytes.
    std::size_t   size{0};
};

/**
 * @brief ROS 2 `builtin_interfaces::Time` — trivially-copyable value type.
 *
 * A CDR1-little-endian fixed-size message representing a moment in time as
 * seconds since the Unix epoch plus a nanosecond component. This class is a
 * plain value type: copy-constructible, assignable, and `constexpr`-capable.
 *
 * @code{.cpp}
 * namespace ef = edgefirst::schemas;
 * ef::Time t{1234567890, 500000};  // sec, nanosec
 * std::uint8_t buf[16];
 * auto written = t.encode({buf, sizeof(buf)});
 * if (written) std::cout << "wrote " << *written << " bytes\n";
 * @endcode
 */
class Time {
public:
    /// Seconds since the Unix epoch (signed; allows dates before 1970).
    std::int32_t sec{0};
    /// Nanosecond component within the current second, in [0, 1e9).
    std::uint32_t nanosec{0};

    /// @brief Default-construct a zero Time (sec == 0, nanosec == 0).
    constexpr Time() noexcept = default;
    /// @brief Construct a Time from explicit seconds and nanoseconds.
    /// @param s Seconds component.
    /// @param n Nanosecond component within the second.
    constexpr Time(std::int32_t s, std::uint32_t n) noexcept : sec(s), nanosec(n) {}

    /// @brief Decode a CDR-encoded Time from a byte buffer.
    /// @param data CDR1-LE bytes; at least 12 bytes required.
    /// @return The decoded Time on success, or an Error with
    ///         `code == EBADMSG` if the buffer is too short or malformed.
    [[nodiscard]] static expected<Time, Error>
    decode(span<const std::uint8_t> data) noexcept {
        Time t;
        if (ros_time_decode(data.data(), data.size(), &t.sec, &t.nanosec) != 0)
            return unexpected<Error>(Error::from_errno("ros_time_decode"));
        return t;
    }

    /// @brief Encode this Time to CDR into the caller's buffer.
    /// @param out Destination buffer; must have capacity ≥ encoded_size().
    /// @return Number of bytes written on success, or an Error with
    ///         `code == ENOBUFS` if the buffer is too small.
    [[nodiscard]] expected<std::size_t, Error>
    encode(span<std::uint8_t> out) const noexcept {
        std::size_t written = 0;
        if (ros_time_encode(out.data(), out.size(), &written, sec, nanosec) != 0)
            return unexpected<Error>(Error::from_errno("ros_time_encode"));
        return written;
    }

    /// @brief Query the number of bytes required to encode this Time.
    /// @return The encoded CDR size (constant for fixed-size messages).
    /// @note Implemented by calling the underlying C encoder with
    ///       `nullptr` and reading back the size-query result.
    [[nodiscard]] expected<std::size_t, Error>
    encoded_size() const noexcept {
        std::size_t written = 0;
        if (ros_time_encode(nullptr, 0, &written, sec, nanosec) != 0)
            return unexpected<Error>(Error::from_errno("ros_time_encode"));
        return written;
    }
};

/// @brief ROS 2 `builtin_interfaces::Duration` — signed seconds plus
///        nanoseconds offset, representing a span of time.
class Duration {
public:
    /// Signed seconds component of the duration.
    std::int32_t sec{0};
    /// Nanosecond component within the second, in [0, 1e9).
    std::uint32_t nanosec{0};

    /// @brief Default-construct a zero Duration.
    constexpr Duration() noexcept = default;
    /// @brief Construct a Duration from explicit seconds and nanoseconds.
    /// @param s Seconds component (may be negative).
    /// @param n Nanosecond component within the second.
    constexpr Duration(std::int32_t s, std::uint32_t n) noexcept : sec(s), nanosec(n) {}

    /// @brief Decode a CDR-encoded Duration from a byte buffer.
    /// @param data CDR1-LE bytes; at least 12 bytes required.
    /// @return The decoded Duration on success, or an Error on failure.
    [[nodiscard]] static expected<Duration, Error>
    decode(span<const std::uint8_t> data) noexcept {
        Duration t;
        if (ros_duration_decode(data.data(), data.size(), &t.sec, &t.nanosec) != 0)
            return unexpected<Error>(Error::from_errno("ros_duration_decode"));
        return t;
    }

    /// @brief Encode this Duration to CDR into the caller's buffer.
    /// @param out Destination buffer; must have capacity ≥ encoded_size().
    /// @return Number of bytes written on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encode(span<std::uint8_t> out) const noexcept {
        std::size_t written = 0;
        if (ros_duration_encode(out.data(), out.size(), &written, sec, nanosec) != 0)
            return unexpected<Error>(Error::from_errno("ros_duration_encode"));
        return written;
    }

    /// @brief Query the number of bytes required to encode this Duration.
    /// @return The encoded CDR size on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encoded_size() const noexcept {
        std::size_t written = 0;
        if (ros_duration_encode(nullptr, 0, &written, sec, nanosec) != 0)
            return unexpected<Error>(Error::from_errno("ros_duration_encode"));
        return written;
    }
};

/// @brief ROS 2 `geometry_msgs::Vector3` — three double-precision components
///        representing a direction/velocity vector (no positional semantics).
class Vector3 {
public:
    /// X component (meters or m/s depending on usage context).
    double x{0};
    /// Y component.
    double y{0};
    /// Z component.
    double z{0};

    /// @brief Default-construct a zero Vector3.
    constexpr Vector3() noexcept = default;
    /// @brief Construct a Vector3 from explicit components.
    /// @param x_ X component.
    /// @param y_ Y component.
    /// @param z_ Z component.
    constexpr Vector3(double x_, double y_, double z_) noexcept : x(x_), y(y_), z(z_) {}

    /// @brief Decode a CDR-encoded Vector3 from a byte buffer.
    /// @param data CDR1-LE bytes; at least 24 bytes required.
    /// @return The decoded Vector3 on success, or an Error on failure.
    [[nodiscard]] static expected<Vector3, Error>
    decode(span<const std::uint8_t> data) noexcept {
        Vector3 t;
        if (ros_vector3_decode(data.data(), data.size(), &t.x, &t.y, &t.z) != 0)
            return unexpected<Error>(Error::from_errno("ros_vector3_decode"));
        return t;
    }

    /// @brief Encode this Vector3 to CDR into the caller's buffer.
    /// @param out Destination buffer; must have capacity ≥ encoded_size().
    /// @return Number of bytes written on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encode(span<std::uint8_t> out) const noexcept {
        std::size_t written = 0;
        if (ros_vector3_encode(out.data(), out.size(), &written, x, y, z) != 0)
            return unexpected<Error>(Error::from_errno("ros_vector3_encode"));
        return written;
    }

    /// @brief Query the number of bytes required to encode this Vector3.
    /// @return The encoded CDR size on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encoded_size() const noexcept {
        std::size_t written = 0;
        if (ros_vector3_encode(nullptr, 0, &written, x, y, z) != 0)
            return unexpected<Error>(Error::from_errno("ros_vector3_encode"));
        return written;
    }
};

/// @brief ROS 2 `geometry_msgs::Point` — three double-precision position
///        coordinates in meters, referenced to a parent frame.
class Point {
public:
    /// X coordinate in meters.
    double x{0};
    /// Y coordinate in meters.
    double y{0};
    /// Z coordinate in meters.
    double z{0};

    /// @brief Default-construct a zero Point (origin).
    constexpr Point() noexcept = default;
    /// @brief Construct a Point from explicit coordinates.
    /// @param x_ X coordinate in meters.
    /// @param y_ Y coordinate in meters.
    /// @param z_ Z coordinate in meters.
    constexpr Point(double x_, double y_, double z_) noexcept : x(x_), y(y_), z(z_) {}

    /// @brief Decode a CDR-encoded Point from a byte buffer.
    /// @param data CDR1-LE bytes; at least 24 bytes required.
    /// @return The decoded Point on success, or an Error on failure.
    [[nodiscard]] static expected<Point, Error>
    decode(span<const std::uint8_t> data) noexcept {
        Point t;
        if (ros_point_decode(data.data(), data.size(), &t.x, &t.y, &t.z) != 0)
            return unexpected<Error>(Error::from_errno("ros_point_decode"));
        return t;
    }

    /// @brief Encode this Point to CDR into the caller's buffer.
    /// @param out Destination buffer; must have capacity ≥ encoded_size().
    /// @return Number of bytes written on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encode(span<std::uint8_t> out) const noexcept {
        std::size_t written = 0;
        if (ros_point_encode(out.data(), out.size(), &written, x, y, z) != 0)
            return unexpected<Error>(Error::from_errno("ros_point_encode"));
        return written;
    }

    /// @brief Query the number of bytes required to encode this Point.
    /// @return The encoded CDR size on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encoded_size() const noexcept {
        std::size_t written = 0;
        if (ros_point_encode(nullptr, 0, &written, x, y, z) != 0)
            return unexpected<Error>(Error::from_errno("ros_point_encode"));
        return written;
    }
};

/// @brief ROS 2 `geometry_msgs::Quaternion` — unit quaternion representing
///        a 3D rotation in `(x, y, z, w)` order.
class Quaternion {
public:
    /// Imaginary i component.
    double x{0};
    /// Imaginary j component.
    double y{0};
    /// Imaginary k component.
    double z{0};
    /// Scalar (real) component.
    double w{0};

    /// @brief Default-construct a zero Quaternion (invalid identity; set
    ///        `w = 1` for the identity rotation).
    constexpr Quaternion() noexcept = default;
    /// @brief Construct a Quaternion from explicit components.
    /// @param x_ Imaginary i component.
    /// @param y_ Imaginary j component.
    /// @param z_ Imaginary k component.
    /// @param w_ Scalar (real) component.
    constexpr Quaternion(double x_, double y_, double z_, double w_) noexcept
        : x(x_), y(y_), z(z_), w(w_) {}

    /// @brief Decode a CDR-encoded Quaternion from a byte buffer.
    /// @param data CDR1-LE bytes; at least 32 bytes required.
    /// @return The decoded Quaternion on success, or an Error on failure.
    [[nodiscard]] static expected<Quaternion, Error>
    decode(span<const std::uint8_t> data) noexcept {
        Quaternion t;
        if (ros_quaternion_decode(data.data(), data.size(), &t.x, &t.y, &t.z, &t.w) != 0)
            return unexpected<Error>(Error::from_errno("ros_quaternion_decode"));
        return t;
    }

    /// @brief Encode this Quaternion to CDR into the caller's buffer.
    /// @param out Destination buffer; must have capacity ≥ encoded_size().
    /// @return Number of bytes written on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encode(span<std::uint8_t> out) const noexcept {
        std::size_t written = 0;
        if (ros_quaternion_encode(out.data(), out.size(), &written, x, y, z, w) != 0)
            return unexpected<Error>(Error::from_errno("ros_quaternion_encode"));
        return written;
    }

    /// @brief Query the number of bytes required to encode this
    ///        Quaternion.
    /// @return The encoded CDR size on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encoded_size() const noexcept {
        std::size_t written = 0;
        if (ros_quaternion_encode(nullptr, 0, &written, x, y, z, w) != 0)
            return unexpected<Error>(Error::from_errno("ros_quaternion_encode"));
        return written;
    }
};

/// @brief ROS 2 `geometry_msgs::Pose` — position + orientation in a single
///        flattened 7-double value type.
class Pose {
public:
    /// Position X coordinate in meters.
    double px{0};
    /// Position Y coordinate in meters.
    double py{0};
    /// Position Z coordinate in meters.
    double pz{0};
    /// Orientation quaternion imaginary i component.
    double ox{0};
    /// Orientation quaternion imaginary j component.
    double oy{0};
    /// Orientation quaternion imaginary k component.
    double oz{0};
    /// Orientation quaternion real component.
    double ow{0};

    /// @brief Default-construct a zero Pose.
    constexpr Pose() noexcept = default;
    /// @brief Construct a Pose from explicit position and orientation.
    /// @param px_ Position X coordinate in meters.
    /// @param py_ Position Y coordinate in meters.
    /// @param pz_ Position Z coordinate in meters.
    /// @param ox_ Orientation quaternion i component.
    /// @param oy_ Orientation quaternion j component.
    /// @param oz_ Orientation quaternion k component.
    /// @param ow_ Orientation quaternion real component.
    constexpr Pose(double px_, double py_, double pz_,
                   double ox_, double oy_, double oz_, double ow_) noexcept
        : px(px_), py(py_), pz(pz_), ox(ox_), oy(oy_), oz(oz_), ow(ow_) {}

    /// @brief Decode a CDR-encoded Pose from a byte buffer.
    /// @param data CDR1-LE bytes; at least 56 bytes required.
    /// @return The decoded Pose on success, or an Error on failure.
    [[nodiscard]] static expected<Pose, Error>
    decode(span<const std::uint8_t> data) noexcept {
        Pose t;
        if (ros_pose_decode(data.data(), data.size(),
                            &t.px, &t.py, &t.pz,
                            &t.ox, &t.oy, &t.oz, &t.ow) != 0)
            return unexpected<Error>(Error::from_errno("ros_pose_decode"));
        return t;
    }

    /// @brief Encode this Pose to CDR into the caller's buffer.
    /// @param out Destination buffer; must have capacity ≥ encoded_size().
    /// @return Number of bytes written on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encode(span<std::uint8_t> out) const noexcept {
        std::size_t written = 0;
        if (ros_pose_encode(out.data(), out.size(), &written,
                            px, py, pz, ox, oy, oz, ow) != 0)
            return unexpected<Error>(Error::from_errno("ros_pose_encode"));
        return written;
    }

    /// @brief Query the number of bytes required to encode this Pose.
    /// @return The encoded CDR size on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encoded_size() const noexcept {
        std::size_t written = 0;
        if (ros_pose_encode(nullptr, 0, &written, px, py, pz, ox, oy, oz, ow) != 0)
            return unexpected<Error>(Error::from_errno("ros_pose_encode"));
        return written;
    }
};

/// @brief ROS 2 `geometry_msgs::Transform` — translation plus rotation
///        quaternion, identical in layout to Pose but semantically a
///        rigid-body transform between two coordinate frames.
class Transform {
public:
    /// Translation X in meters.
    double tx{0};
    /// Translation Y in meters.
    double ty{0};
    /// Translation Z in meters.
    double tz{0};
    /// Rotation quaternion imaginary i component.
    double rx{0};
    /// Rotation quaternion imaginary j component.
    double ry{0};
    /// Rotation quaternion imaginary k component.
    double rz{0};
    /// Rotation quaternion real component.
    double rw{0};

    /// @brief Default-construct a zero Transform.
    constexpr Transform() noexcept = default;
    /// @brief Construct a Transform from explicit translation and rotation.
    /// @param tx_ Translation X in meters.
    /// @param ty_ Translation Y in meters.
    /// @param tz_ Translation Z in meters.
    /// @param rx_ Rotation quaternion i component.
    /// @param ry_ Rotation quaternion j component.
    /// @param rz_ Rotation quaternion k component.
    /// @param rw_ Rotation quaternion real component.
    constexpr Transform(double tx_, double ty_, double tz_,
                        double rx_, double ry_, double rz_, double rw_) noexcept
        : tx(tx_), ty(ty_), tz(tz_), rx(rx_), ry(ry_), rz(rz_), rw(rw_) {}

    /// @brief Decode a CDR-encoded Transform from a byte buffer.
    /// @param data CDR1-LE bytes; at least 56 bytes required.
    /// @return The decoded Transform on success, or an Error on failure.
    [[nodiscard]] static expected<Transform, Error>
    decode(span<const std::uint8_t> data) noexcept {
        Transform t;
        if (ros_transform_decode(data.data(), data.size(),
                                 &t.tx, &t.ty, &t.tz,
                                 &t.rx, &t.ry, &t.rz, &t.rw) != 0)
            return unexpected<Error>(Error::from_errno("ros_transform_decode"));
        return t;
    }

    /// @brief Encode this Transform to CDR into the caller's buffer.
    /// @param out Destination buffer; must have capacity ≥ encoded_size().
    /// @return Number of bytes written on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encode(span<std::uint8_t> out) const noexcept {
        std::size_t written = 0;
        if (ros_transform_encode(out.data(), out.size(), &written,
                                 tx, ty, tz, rx, ry, rz, rw) != 0)
            return unexpected<Error>(Error::from_errno("ros_transform_encode"));
        return written;
    }

    /// @brief Query the number of bytes required to encode this Transform.
    /// @return The encoded CDR size on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encoded_size() const noexcept {
        std::size_t written = 0;
        if (ros_transform_encode(nullptr, 0, &written, tx, ty, tz, rx, ry, rz, rw) != 0)
            return unexpected<Error>(Error::from_errno("ros_transform_encode"));
        return written;
    }
};

/// @brief ROS 2 `geometry_msgs::Twist` — linear and angular velocity of a
///        rigid body in free space (m/s and rad/s).
class Twist {
public:
    /// Linear velocity X component in m/s.
    double lx{0};
    /// Linear velocity Y component in m/s.
    double ly{0};
    /// Linear velocity Z component in m/s.
    double lz{0};
    /// Angular velocity X component in rad/s.
    double ax{0};
    /// Angular velocity Y component in rad/s.
    double ay{0};
    /// Angular velocity Z component in rad/s.
    double az{0};

    /// @brief Default-construct a zero Twist.
    constexpr Twist() noexcept = default;
    /// @brief Construct a Twist from explicit linear and angular
    ///        velocities.
    /// @param lx_ Linear X velocity in m/s.
    /// @param ly_ Linear Y velocity in m/s.
    /// @param lz_ Linear Z velocity in m/s.
    /// @param ax_ Angular X velocity in rad/s.
    /// @param ay_ Angular Y velocity in rad/s.
    /// @param az_ Angular Z velocity in rad/s.
    constexpr Twist(double lx_, double ly_, double lz_,
                    double ax_, double ay_, double az_) noexcept
        : lx(lx_), ly(ly_), lz(lz_), ax(ax_), ay(ay_), az(az_) {}

    /// @brief Decode a CDR-encoded Twist from a byte buffer.
    /// @param data CDR1-LE bytes; at least 48 bytes required.
    /// @return The decoded Twist on success, or an Error on failure.
    [[nodiscard]] static expected<Twist, Error>
    decode(span<const std::uint8_t> data) noexcept {
        Twist t;
        if (ros_twist_decode(data.data(), data.size(),
                             &t.lx, &t.ly, &t.lz,
                             &t.ax, &t.ay, &t.az) != 0)
            return unexpected<Error>(Error::from_errno("ros_twist_decode"));
        return t;
    }

    /// @brief Encode this Twist to CDR into the caller's buffer.
    /// @param out Destination buffer; must have capacity ≥ encoded_size().
    /// @return Number of bytes written on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encode(span<std::uint8_t> out) const noexcept {
        std::size_t written = 0;
        if (ros_twist_encode(out.data(), out.size(), &written,
                             lx, ly, lz, ax, ay, az) != 0)
            return unexpected<Error>(Error::from_errno("ros_twist_encode"));
        return written;
    }

    /// @brief Query the number of bytes required to encode this Twist.
    /// @return The encoded CDR size on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encoded_size() const noexcept {
        std::size_t written = 0;
        if (ros_twist_encode(nullptr, 0, &written, lx, ly, lz, ax, ay, az) != 0)
            return unexpected<Error>(Error::from_errno("ros_twist_encode"));
        return written;
    }
};

/// @brief ROS 2 `geometry_msgs::PoseWithCovariance` — Pose plus a 6×6
///        row-major covariance matrix over (x, y, z, rotX, rotY, rotZ).
class PoseWithCovariance {
public:
    Pose pose{};
    std::array<double, 36> covariance{};

    constexpr PoseWithCovariance() noexcept = default;
    PoseWithCovariance(Pose p, std::array<double, 36> cov) noexcept
        : pose(p), covariance(cov) {}

    [[nodiscard]] static expected<PoseWithCovariance, Error>
    decode(span<const std::uint8_t> data) noexcept {
        PoseWithCovariance t;
        if (ros_pose_with_covariance_decode(data.data(), data.size(),
                                            &t.pose.px, &t.pose.py, &t.pose.pz,
                                            &t.pose.ox, &t.pose.oy, &t.pose.oz, &t.pose.ow,
                                            t.covariance.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_pose_with_covariance_decode"));
        return t;
    }

    [[nodiscard]] expected<std::size_t, Error>
    encode(span<std::uint8_t> out) const noexcept {
        std::size_t written = 0;
        if (ros_pose_with_covariance_encode(out.data(), out.size(), &written,
                                            pose.px, pose.py, pose.pz,
                                            pose.ox, pose.oy, pose.oz, pose.ow,
                                            covariance.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_pose_with_covariance_encode"));
        return written;
    }

    [[nodiscard]] expected<std::size_t, Error>
    encoded_size() const noexcept {
        std::size_t written = 0;
        if (ros_pose_with_covariance_encode(nullptr, 0, &written,
                                            pose.px, pose.py, pose.pz,
                                            pose.ox, pose.oy, pose.oz, pose.ow,
                                            covariance.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_pose_with_covariance_encode"));
        return written;
    }
};

/// @brief ROS 2 `geometry_msgs::TwistWithCovariance` — Twist plus a 6×6
///        row-major covariance matrix over (x, y, z, rotX, rotY, rotZ).
class TwistWithCovariance {
public:
    Twist twist{};
    std::array<double, 36> covariance{};

    constexpr TwistWithCovariance() noexcept = default;
    TwistWithCovariance(Twist t, std::array<double, 36> cov) noexcept
        : twist(t), covariance(cov) {}

    [[nodiscard]] static expected<TwistWithCovariance, Error>
    decode(span<const std::uint8_t> data) noexcept {
        TwistWithCovariance t;
        if (ros_twist_with_covariance_decode(data.data(), data.size(),
                                             &t.twist.lx, &t.twist.ly, &t.twist.lz,
                                             &t.twist.ax, &t.twist.ay, &t.twist.az,
                                             t.covariance.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_twist_with_covariance_decode"));
        return t;
    }

    [[nodiscard]] expected<std::size_t, Error>
    encode(span<std::uint8_t> out) const noexcept {
        std::size_t written = 0;
        if (ros_twist_with_covariance_encode(out.data(), out.size(), &written,
                                             twist.lx, twist.ly, twist.lz,
                                             twist.ax, twist.ay, twist.az,
                                             covariance.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_twist_with_covariance_encode"));
        return written;
    }

    [[nodiscard]] expected<std::size_t, Error>
    encoded_size() const noexcept {
        std::size_t written = 0;
        if (ros_twist_with_covariance_encode(nullptr, 0, &written,
                                             twist.lx, twist.ly, twist.lz,
                                             twist.ax, twist.ay, twist.az,
                                             covariance.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_twist_with_covariance_encode"));
        return written;
    }
};

/// @brief ROS 2 `geometry_msgs::Accel` — linear and angular acceleration of
///        a rigid body in free space (m/s² and rad/s²).
class Accel {
public:
    /// Linear acceleration X component in m/s².
    double lx{0};
    /// Linear acceleration Y component in m/s².
    double ly{0};
    /// Linear acceleration Z component in m/s².
    double lz{0};
    /// Angular acceleration X component in rad/s².
    double ax{0};
    /// Angular acceleration Y component in rad/s².
    double ay{0};
    /// Angular acceleration Z component in rad/s².
    double az{0};

    /// @brief Default-construct a zero Accel.
    constexpr Accel() noexcept = default;
    /// @brief Construct an Accel from explicit linear and angular
    ///        accelerations.
    /// @param lx_ Linear X acceleration in m/s².
    /// @param ly_ Linear Y acceleration in m/s².
    /// @param lz_ Linear Z acceleration in m/s².
    /// @param ax_ Angular X acceleration in rad/s².
    /// @param ay_ Angular Y acceleration in rad/s².
    /// @param az_ Angular Z acceleration in rad/s².
    constexpr Accel(double lx_, double ly_, double lz_,
                    double ax_, double ay_, double az_) noexcept
        : lx(lx_), ly(ly_), lz(lz_), ax(ax_), ay(ay_), az(az_) {}

    /// @brief Decode a CDR-encoded Accel from a byte buffer.
    /// @param data CDR1-LE bytes; at least 48 bytes required.
    /// @return The decoded Accel on success, or an Error on failure.
    [[nodiscard]] static expected<Accel, Error>
    decode(span<const std::uint8_t> data) noexcept {
        Accel t;
        if (ros_accel_decode(data.data(), data.size(),
                             &t.lx, &t.ly, &t.lz,
                             &t.ax, &t.ay, &t.az) != 0)
            return unexpected<Error>(Error::from_errno("ros_accel_decode"));
        return t;
    }

    /// @brief Encode this Accel to CDR into the caller's buffer.
    /// @param out Destination buffer; must have capacity ≥ encoded_size().
    /// @return Number of bytes written on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encode(span<std::uint8_t> out) const noexcept {
        std::size_t written = 0;
        if (ros_accel_encode(out.data(), out.size(), &written,
                             lx, ly, lz, ax, ay, az) != 0)
            return unexpected<Error>(Error::from_errno("ros_accel_encode"));
        return written;
    }

    /// @brief Query the number of bytes required to encode this Accel.
    /// @return The encoded CDR size on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encoded_size() const noexcept {
        std::size_t written = 0;
        if (ros_accel_encode(nullptr, 0, &written, lx, ly, lz, ax, ay, az) != 0)
            return unexpected<Error>(Error::from_errno("ros_accel_encode"));
        return written;
    }
};

/// @brief ROS 2 `geometry_msgs::Wrench` — force and torque vectors applied
///        to a rigid body, each as a 3D vector (N and N·m).
class Wrench {
public:
    /// Force X component in Newtons.
    double fx{0};
    /// Force Y component in Newtons.
    double fy{0};
    /// Force Z component in Newtons.
    double fz{0};
    /// Torque X component in N·m.
    double tx{0};
    /// Torque Y component in N·m.
    double ty{0};
    /// Torque Z component in N·m.
    double tz{0};

    /// @brief Default-construct a zero Wrench.
    constexpr Wrench() noexcept = default;
    /// @brief Construct a Wrench from explicit force and torque components.
    constexpr Wrench(double fx_, double fy_, double fz_,
                     double tx_, double ty_, double tz_) noexcept
        : fx(fx_), fy(fy_), fz(fz_), tx(tx_), ty(ty_), tz(tz_) {}

    /// @brief Decode a CDR-encoded Wrench from a byte buffer.
    /// @param data CDR1-LE bytes; at least 48 bytes required.
    /// @return The decoded Wrench on success, or an Error on failure.
    [[nodiscard]] static expected<Wrench, Error>
    decode(span<const std::uint8_t> data) noexcept {
        Wrench w;
        if (ros_wrench_decode(data.data(), data.size(),
                              &w.fx, &w.fy, &w.fz,
                              &w.tx, &w.ty, &w.tz) != 0)
            return unexpected<Error>(Error::from_errno("ros_wrench_decode"));
        return w;
    }

    /// @brief Encode this Wrench to CDR into the caller's buffer.
    /// @param out Destination buffer; must have capacity ≥ encoded_size().
    /// @return Number of bytes written on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encode(span<std::uint8_t> out) const noexcept {
        std::size_t written = 0;
        if (ros_wrench_encode(out.data(), out.size(), &written,
                              fx, fy, fz, tx, ty, tz) != 0)
            return unexpected<Error>(Error::from_errno("ros_wrench_encode"));
        return written;
    }

    /// @brief Query the number of bytes required to encode this Wrench.
    /// @return The encoded CDR size on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encoded_size() const noexcept {
        std::size_t written = 0;
        if (ros_wrench_encode(nullptr, 0, &written, fx, fy, fz, tx, ty, tz) != 0)
            return unexpected<Error>(Error::from_errno("ros_wrench_encode"));
        return written;
    }
};

/// @brief ROS 2 `geometry_msgs::AccelWithCovariance` — acceleration with a
///        6×6 row-major covariance matrix (linear + angular).
class AccelWithCovariance {
public:
    Accel accel{};
    std::array<double, 36> covariance{};

    constexpr AccelWithCovariance() noexcept = default;
    AccelWithCovariance(Accel a, std::array<double, 36> cov) noexcept
        : accel(a), covariance(cov) {}

    [[nodiscard]] static expected<AccelWithCovariance, Error>
    decode(span<const std::uint8_t> data) noexcept {
        AccelWithCovariance a;
        if (ros_accel_with_covariance_decode(data.data(), data.size(),
                                             &a.accel.lx, &a.accel.ly, &a.accel.lz,
                                             &a.accel.ax, &a.accel.ay, &a.accel.az,
                                             a.covariance.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_accel_with_covariance_decode"));
        return a;
    }

    [[nodiscard]] expected<std::size_t, Error>
    encode(span<std::uint8_t> out) const noexcept {
        std::size_t written = 0;
        if (ros_accel_with_covariance_encode(out.data(), out.size(), &written,
                                             accel.lx, accel.ly, accel.lz,
                                             accel.ax, accel.ay, accel.az,
                                             covariance.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_accel_with_covariance_encode"));
        return written;
    }

    [[nodiscard]] expected<std::size_t, Error>
    encoded_size() const noexcept {
        std::size_t written = 0;
        if (ros_accel_with_covariance_encode(nullptr, 0, &written,
                                             accel.lx, accel.ly, accel.lz,
                                             accel.ax, accel.ay, accel.az,
                                             covariance.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_accel_with_covariance_encode"));
        return written;
    }
};

/// @brief ROS 2 `sensor_msgs::NavSatStatus` — GNSS fix status and service
///        bitmask as defined by `sensor_msgs/NavSatStatus.msg`.
class NavSatStatus {
public:
    /// Fix status (e.g. STATUS_NO_FIX, STATUS_FIX, STATUS_SBAS_FIX,
    /// STATUS_GBAS_FIX). See the ROS message definition for the full set.
    std::int8_t status{0};
    /// Bitmask of constellations used for the fix (GPS, GLONASS,
    /// COMPASS, GALILEO).
    std::uint16_t service{0};

    /// @brief Default-construct a NavSatStatus (no fix, no service).
    constexpr NavSatStatus() noexcept = default;
    /// @brief Construct a NavSatStatus from explicit fields.
    /// @param s Fix status code.
    /// @param svc Service constellation bitmask.
    constexpr NavSatStatus(std::int8_t s, std::uint16_t svc) noexcept
        : status(s), service(svc) {}

    /// @brief Decode a CDR-encoded NavSatStatus from a byte buffer.
    /// @param data CDR1-LE bytes.
    /// @return The decoded NavSatStatus on success, or an Error.
    [[nodiscard]] static expected<NavSatStatus, Error>
    decode(span<const std::uint8_t> data) noexcept {
        NavSatStatus t;
        if (ros_nav_sat_status_decode(data.data(), data.size(), &t.status, &t.service) != 0)
            return unexpected<Error>(Error::from_errno("ros_nav_sat_status_decode"));
        return t;
    }

    /// @brief Encode this NavSatStatus to CDR into the caller's buffer.
    /// @param out Destination buffer; must have capacity ≥ encoded_size().
    /// @return Number of bytes written on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encode(span<std::uint8_t> out) const noexcept {
        std::size_t written = 0;
        if (ros_nav_sat_status_encode(out.data(), out.size(), &written, status, service) != 0)
            return unexpected<Error>(Error::from_errno("ros_nav_sat_status_encode"));
        return written;
    }

    /// @brief Query the number of bytes required to encode this
    ///        NavSatStatus.
    /// @return The encoded CDR size on success, or an Error.
    [[nodiscard]] expected<std::size_t, Error>
    encoded_size() const noexcept {
        std::size_t written = 0;
        if (ros_nav_sat_status_encode(nullptr, 0, &written, status, service) != 0)
            return unexpected<Error>(Error::from_errno("ros_nav_sat_status_encode"));
        return written;
    }
};

// ============================================================================
// Buffer-backed types — template base classes (no macros)
// ============================================================================

namespace detail {

/**
 * @internal
 * @brief CRTP base for non-owning, move-only handle wrappers.
 *
 * Implementation helper shared by every `*View` class in the public API.
 * Holds a single opaque C handle pointer and releases it in the destructor
 * via the traits-provided `free` function. Derived classes expose
 * field accessors that read through `handle()`.
 *
 * The Traits struct must provide:
 *   - `using handle_type = <opaque C type>;`
 *   - `static constexpr auto from_cdr = <fn ptr>;`
 *   - `static constexpr auto free     = <fn ptr>;`
 *   - `static constexpr auto as_cdr   = <fn ptr>;`
 *   - `static constexpr std::string_view name = "...";`
 *
 * For types that lack `as_cdr` in the C API, omit the `as_cdr` field in
 * the traits and use ViewBaseNoCdr instead.
 */
template <typename Derived, typename Traits>
class ViewBase {
public:
    using handle_type = typename Traits::handle_type;

    ViewBase(const ViewBase&)            = delete;
    ViewBase& operator=(const ViewBase&) = delete;

    ViewBase(ViewBase&& o) noexcept : handle_(o.handle_) { o.handle_ = nullptr; }
    ViewBase& operator=(ViewBase&& o) noexcept {
        if (this != &o) { reset(); handle_ = o.handle_; o.handle_ = nullptr; }
        return *this;
    }
    ~ViewBase() { reset(); }

    [[nodiscard]] static expected<Derived, Error>
    from_cdr(span<const std::uint8_t> data) noexcept {
        auto* h = Traits::from_cdr(data.data(), data.size());
        if (!h) return unexpected<Error>(Error::from_errno(Traits::name));
        return Derived{h};
    }

    [[nodiscard]] span<const std::uint8_t> as_cdr() const noexcept {
        std::size_t n = 0;
        auto* p = Traits::as_cdr(handle_, &n);
        return {p, n};
    }

protected:
    explicit ViewBase(handle_type* h) noexcept : handle_(h) {}
    [[nodiscard]] const handle_type* handle() const noexcept { return handle_; }

private:
    void reset() noexcept {
        if (handle_) { Traits::free(handle_); handle_ = nullptr; }
    }
    handle_type* handle_{nullptr};
};

/**
 * @internal
 * @brief CRTP base for non-owning view wrappers that lack an `as_cdr`
 *        accessor in the underlying C API.
 *
 * Same shape as ViewBase but omits the `as_cdr()` member. Used for
 * `MaskView` and `BoxView`, whose C handles do not expose
 * `ros_<prefix>_as_cdr` after the Task 4 refactor.
 */
template <typename Derived, typename Traits>
class ViewBaseNoCdr {
public:
    using handle_type = typename Traits::handle_type;

    ViewBaseNoCdr(const ViewBaseNoCdr&)            = delete;
    ViewBaseNoCdr& operator=(const ViewBaseNoCdr&) = delete;

    ViewBaseNoCdr(ViewBaseNoCdr&& o) noexcept : handle_(o.handle_) { o.handle_ = nullptr; }
    ViewBaseNoCdr& operator=(ViewBaseNoCdr&& o) noexcept {
        if (this != &o) { reset(); handle_ = o.handle_; o.handle_ = nullptr; }
        return *this;
    }
    ~ViewBaseNoCdr() { reset(); }

    [[nodiscard]] static expected<Derived, Error>
    from_cdr(span<const std::uint8_t> data) noexcept {
        auto* h = Traits::from_cdr(data.data(), data.size());
        if (!h) return unexpected<Error>(Error::from_errno(Traits::name));
        return Derived{h};
    }

protected:
    explicit ViewBaseNoCdr(handle_type* h) noexcept : handle_(h) {}
    [[nodiscard]] const handle_type* handle() const noexcept { return handle_; }

private:
    void reset() noexcept {
        if (handle_) { Traits::free(handle_); handle_ = nullptr; }
    }
    handle_type* handle_{nullptr};
};

/**
 * @internal
 * @brief CRTP base for move-only owning wrappers.
 *
 * Holds both the encoded CDR bytes (allocated by
 * `ros_<prefix>_encode`) and the opaque view handle over them. The
 * destructor frees the handle via the traits and then releases the
 * encoded bytes via `ros_bytes_free()`. `as_cdr()` delegates to
 * `Traits::as_cdr` on the handle. The Traits struct has the same
 * shape as for ViewBase.
 */
template <typename Derived, typename Traits>
class OwnedBase {
public:
    using handle_type = typename Traits::handle_type;

    OwnedBase(const OwnedBase&)            = delete;
    OwnedBase& operator=(const OwnedBase&) = delete;

    OwnedBase(OwnedBase&& o) noexcept
        : bytes_(o.bytes_), bytes_len_(o.bytes_len_), handle_(o.handle_) {
        o.bytes_ = nullptr; o.bytes_len_ = 0; o.handle_ = nullptr;
    }
    OwnedBase& operator=(OwnedBase&& o) noexcept {
        if (this != &o) {
            reset();
            bytes_ = o.bytes_; bytes_len_ = o.bytes_len_; handle_ = o.handle_;
            o.bytes_ = nullptr; o.bytes_len_ = 0; o.handle_ = nullptr;
        }
        return *this;
    }
    ~OwnedBase() { reset(); }

    [[nodiscard]] span<const std::uint8_t> as_cdr() const noexcept {
        std::size_t n = 0;
        auto* p = Traits::as_cdr(handle_, &n);
        return {p, n};
    }

    /// @brief Transfer ownership of the encoded CDR byte buffer out of
    ///        this wrapper and return it as a raw `(data, size)` pair.
    ///
    /// After this call the wrapper is empty: its internal C handle has
    /// been freed, its byte pointer has been nulled, and the destructor
    /// becomes a safe no-op. The caller is responsible for freeing the
    /// returned buffer via `ros_bytes_free(r.data, r.size)` or handing
    /// ownership to a sink that frees it (for example a
    /// `zenoh::Bytes` constructed with a matching deleter lambda).
    ///
    /// Rvalue-qualified to make the ownership-transfer intent explicit
    /// at the call site; call as `std::move(owner).release()`.
    ///
    /// @return A `Released` POD with the transferred buffer pointer and
    ///         size. Returns `{nullptr, 0}` if the wrapper was already
    ///         empty (moved-from or previously released).
    [[nodiscard]] Released release() && noexcept {
        if (handle_) { Traits::free(handle_); handle_ = nullptr; }
        Released r{bytes_, bytes_len_};
        bytes_ = nullptr;
        bytes_len_ = 0;
        return r;
    }

protected:
    OwnedBase() noexcept = default;
    [[nodiscard]] const handle_type* handle() const noexcept { return handle_; }

    void take_ownership(std::uint8_t* bytes, std::size_t len,
                        handle_type* h) noexcept {
        bytes_ = bytes; bytes_len_ = len; handle_ = h;
    }

    // Convenience factory: parse freshly-encoded bytes into handle, take ownership.
    [[nodiscard]] static expected<Derived, Error>
    make_(std::uint8_t* out, std::size_t len) noexcept {
        auto* h = Traits::from_cdr(out, len);
        if (!h) {
            auto err = Error::from_errno(Traits::name);
            ros_bytes_free(out, len);
            return unexpected<Error>(err);
        }
        Derived result;
        result.take_ownership(out, len, h);
        return result;
    }

private:
    void reset() noexcept {
        if (handle_) { Traits::free(handle_); handle_ = nullptr; }
        if (bytes_)  { ros_bytes_free(bytes_, bytes_len_);
                       bytes_ = nullptr; bytes_len_ = 0; }
    }
    std::uint8_t* bytes_{nullptr};
    std::size_t   bytes_len_{0};
    handle_type*  handle_{nullptr};
};

/**
 * @internal
 * @brief CRTP base for move-only owning wrappers whose C handle lacks an
 *        `as_cdr` accessor.
 *
 * Same as OwnedBase except `as_cdr()` returns the stored encoded bytes
 * directly rather than calling `Traits::as_cdr`. Used by the owning
 * `Mask` class, since `ros_mask_as_cdr` does not exist in the C API.
 */
template <typename Derived, typename Traits>
class OwnedBaseNoCdr {
public:
    using handle_type = typename Traits::handle_type;

    OwnedBaseNoCdr(const OwnedBaseNoCdr&)            = delete;
    OwnedBaseNoCdr& operator=(const OwnedBaseNoCdr&) = delete;

    OwnedBaseNoCdr(OwnedBaseNoCdr&& o) noexcept
        : bytes_(o.bytes_), bytes_len_(o.bytes_len_), handle_(o.handle_) {
        o.bytes_ = nullptr; o.bytes_len_ = 0; o.handle_ = nullptr;
    }
    OwnedBaseNoCdr& operator=(OwnedBaseNoCdr&& o) noexcept {
        if (this != &o) {
            reset();
            bytes_ = o.bytes_; bytes_len_ = o.bytes_len_; handle_ = o.handle_;
            o.bytes_ = nullptr; o.bytes_len_ = 0; o.handle_ = nullptr;
        }
        return *this;
    }
    ~OwnedBaseNoCdr() { reset(); }

    [[nodiscard]] span<const std::uint8_t> as_cdr() const noexcept {
        return {bytes_, bytes_len_};
    }

    /// @brief Transfer ownership of the encoded CDR byte buffer out of
    ///        this wrapper and return it as a raw `(data, size)` pair.
    ///
    /// See `OwnedBase::release()` for the full contract. Applies to
    /// owning types whose C handle lacks an `as_cdr` accessor (today,
    /// just `Mask`).
    [[nodiscard]] Released release() && noexcept {
        if (handle_) { Traits::free(handle_); handle_ = nullptr; }
        Released r{bytes_, bytes_len_};
        bytes_ = nullptr;
        bytes_len_ = 0;
        return r;
    }

protected:
    OwnedBaseNoCdr() noexcept = default;
    [[nodiscard]] const handle_type* handle() const noexcept { return handle_; }

    void take_ownership(std::uint8_t* bytes, std::size_t len,
                        handle_type* h) noexcept {
        bytes_ = bytes; bytes_len_ = len; handle_ = h;
    }

    [[nodiscard]] static expected<Derived, Error>
    make_(std::uint8_t* out, std::size_t len) noexcept {
        auto* h = Traits::from_cdr(out, len);
        if (!h) {
            auto err = Error::from_errno(Traits::name);
            ros_bytes_free(out, len);
            return unexpected<Error>(err);
        }
        Derived result;
        result.take_ownership(out, len, h);
        return result;
    }

private:
    void reset() noexcept {
        if (handle_) { Traits::free(handle_); handle_ = nullptr; }
        if (bytes_)  { ros_bytes_free(bytes_, bytes_len_);
                       bytes_ = nullptr; bytes_len_ = 0; }
    }
    std::uint8_t* bytes_{nullptr};
    std::size_t   bytes_len_{0};
    handle_type*  handle_{nullptr};
};

/**
 * @internal
 * @brief CRTP base for move-only builder wrappers.
 *
 * Wraps the lifecycle of an opaque C builder handle (`*_builder_new`,
 * `*_builder_free`) and the two build paths: allocating (`build()` →
 * `Released`) and caller-buffer (`encode_into()` → size).
 *
 * Derived builder classes inherit this and add per-message fluent setters.
 * `BuilderTraits` must provide:
 *   - `using builder_type = <opaque C builder type>;`
 *   - `static constexpr auto new_fn  = <fn ptr>;`  // () → builder*
 *   - `static constexpr auto free_fn = <fn ptr>;`  // (builder*) → void
 *   - `static constexpr auto build_fn = <fn ptr>;`  // (builder*, uint8_t**, size_t*) → int
 *   - `static constexpr auto encode_into_fn = <fn ptr>;`  // (builder*, uint8_t*, size_t, size_t*) → int
 *   - `static constexpr std::string_view name = "...";`
 *   - `static constexpr std::string_view build_name = "...";`
 *   - `static constexpr std::string_view encode_into_name = "...";`
 *
 * The `name` field identifies the builder constructor for errors during
 * wrapper creation. `build_name` and `encode_into_name` identify the
 * underlying C entry points used by `build()` and `encode_into()`,
 * respectively, so error reports name the failing operation precisely.
 */
template <typename Derived, typename BuilderTraits>
class BuilderBase {
public:
    using builder_type = typename BuilderTraits::builder_type;

    BuilderBase(const BuilderBase&)            = delete;
    BuilderBase& operator=(const BuilderBase&) = delete;

    BuilderBase(BuilderBase&& o) noexcept
        : b_(o.b_) { o.b_ = nullptr; }
    BuilderBase& operator=(BuilderBase&& o) noexcept {
        if (this != &o) {
            if (b_) BuilderTraits::free_fn(b_);
            b_ = o.b_;
            o.b_ = nullptr;
        }
        return *this;
    }
    ~BuilderBase() { if (b_) BuilderTraits::free_fn(b_); }

    /// @brief Create a new builder instance.
    /// @return The builder on success, or an Error if allocation fails.
    [[nodiscard]] static expected<Derived, Error> create() noexcept {
        auto* raw = BuilderTraits::new_fn();
        if (!raw)
            return unexpected<Error>(Error::from_errno(BuilderTraits::name));
        Derived d{raw};
        return d;
    }

    /// @brief Allocate a fresh CDR buffer and encode the message.
    /// @return A `Released` POD on success (caller frees via
    ///         `ros_bytes_free`), or an Error.
    [[nodiscard]] expected<Released, Error> build() noexcept {
        std::uint8_t* out = nullptr;
        std::size_t len = 0;
        if (BuilderTraits::build_fn(b_, &out, &len) != 0)
            return unexpected<Error>(Error::from_errno(BuilderTraits::build_name));
        return Released{out, len};
    }

    /// @brief Encode the message into a caller-provided buffer.
    /// @param buf Destination buffer.
    /// @return Number of bytes written on success, or an Error (ENOBUFS
    ///         if buffer too small).
    [[nodiscard]] expected<std::size_t, Error>
    encode_into(span<std::uint8_t> buf) noexcept {
        std::size_t len = 0;
        if (BuilderTraits::encode_into_fn(b_, buf.data(), buf.size(), &len) != 0)
            return unexpected<Error>(Error::from_errno(BuilderTraits::encode_into_name));
        return len;
    }

protected:
    explicit BuilderBase(builder_type* raw) noexcept : b_(raw) {}
    [[nodiscard]] builder_type* ptr() const noexcept { return b_; }

private:
    builder_type* b_{nullptr};
};

// ---------------------------------------------------------------------------
// Traits structs — one per buffer-backed C handle type
// ---------------------------------------------------------------------------

struct HeaderTraits {
    using handle_type = ros_header_t;
    static constexpr auto from_cdr = ros_header_from_cdr;
    static constexpr auto free     = ros_header_free;
    static constexpr auto as_cdr   = ros_header_as_cdr;
    static constexpr std::string_view name = "ros_header";
};

struct CompressedImageTraits {
    using handle_type = ros_compressed_image_t;
    static constexpr auto from_cdr = ros_compressed_image_from_cdr;
    static constexpr auto free     = ros_compressed_image_free;
    static constexpr auto as_cdr   = ros_compressed_image_as_cdr;
    static constexpr std::string_view name = "ros_compressed_image";
};

struct ImuTraits {
    using handle_type = ros_imu_t;
    static constexpr auto from_cdr = ros_imu_from_cdr;
    static constexpr auto free     = ros_imu_free;
    static constexpr auto as_cdr   = ros_imu_as_cdr;
    static constexpr std::string_view name = "ros_imu";
};

struct NavSatFixTraits {
    using handle_type = ros_nav_sat_fix_t;
    static constexpr auto from_cdr = ros_nav_sat_fix_from_cdr;
    static constexpr auto free     = ros_nav_sat_fix_free;
    static constexpr auto as_cdr   = ros_nav_sat_fix_as_cdr;
    static constexpr std::string_view name = "ros_nav_sat_fix";
};

struct CameraInfoTraits {
    using handle_type = ros_camera_info_t;
    static constexpr auto from_cdr = ros_camera_info_from_cdr;
    static constexpr auto free     = ros_camera_info_free;
    static constexpr auto as_cdr   = ros_camera_info_as_cdr;
    static constexpr std::string_view name = "ros_camera_info";
};

struct TransformStampedTraits {
    using handle_type = ros_transform_stamped_t;
    static constexpr auto from_cdr = ros_transform_stamped_from_cdr;
    static constexpr auto free     = ros_transform_stamped_free;
    static constexpr auto as_cdr   = ros_transform_stamped_as_cdr;
    static constexpr std::string_view name = "ros_transform_stamped";
};

struct TwistStampedTraits {
    using handle_type = ros_twist_stamped_t;
    static constexpr auto from_cdr = ros_twist_stamped_from_cdr;
    static constexpr auto free     = ros_twist_stamped_free;
    static constexpr auto as_cdr   = ros_twist_stamped_as_cdr;
    static constexpr std::string_view name = "ros_twist_stamped";
};

struct AccelStampedTraits {
    using handle_type = ros_accel_stamped_t;
    static constexpr auto from_cdr = ros_accel_stamped_from_cdr;
    static constexpr auto free     = ros_accel_stamped_free;
    static constexpr auto as_cdr   = ros_accel_stamped_as_cdr;
    static constexpr std::string_view name = "ros_accel_stamped";
};

struct PointStampedTraits {
    using handle_type = ros_point_stamped_t;
    static constexpr auto from_cdr = ros_point_stamped_from_cdr;
    static constexpr auto free     = ros_point_stamped_free;
    static constexpr auto as_cdr   = ros_point_stamped_as_cdr;
    static constexpr std::string_view name = "ros_point_stamped";
};

struct InertiaStampedTraits {
    using handle_type = ros_inertia_stamped_t;
    static constexpr auto from_cdr = ros_inertia_stamped_from_cdr;
    static constexpr auto free     = ros_inertia_stamped_free;
    static constexpr auto as_cdr   = ros_inertia_stamped_as_cdr;
    static constexpr std::string_view name = "ros_inertia_stamped";
};

struct Vector3StampedTraits {
    using handle_type = ros_vector3_stamped_t;
    static constexpr auto from_cdr = ros_vector3_stamped_from_cdr;
    static constexpr auto free     = ros_vector3_stamped_free;
    static constexpr auto as_cdr   = ros_vector3_stamped_as_cdr;
    static constexpr std::string_view name = "ros_vector3_stamped";
};

struct PoseStampedTraits {
    using handle_type = ros_pose_stamped_t;
    static constexpr auto from_cdr = ros_pose_stamped_from_cdr;
    static constexpr auto free     = ros_pose_stamped_free;
    static constexpr auto as_cdr   = ros_pose_stamped_as_cdr;
    static constexpr std::string_view name = "ros_pose_stamped";
};

struct QuaternionStampedTraits {
    using handle_type = ros_quaternion_stamped_t;
    static constexpr auto from_cdr = ros_quaternion_stamped_from_cdr;
    static constexpr auto free     = ros_quaternion_stamped_free;
    static constexpr auto as_cdr   = ros_quaternion_stamped_as_cdr;
    static constexpr std::string_view name = "ros_quaternion_stamped";
};

struct WrenchStampedTraits {
    using handle_type = ros_wrench_stamped_t;
    static constexpr auto from_cdr = ros_wrench_stamped_from_cdr;
    static constexpr auto free     = ros_wrench_stamped_free;
    static constexpr auto as_cdr   = ros_wrench_stamped_as_cdr;
    static constexpr std::string_view name = "ros_wrench_stamped";
};

struct PoseWithCovarianceStampedTraits {
    using handle_type = ros_pose_with_covariance_stamped_t;
    static constexpr auto from_cdr = ros_pose_with_covariance_stamped_from_cdr;
    static constexpr auto free     = ros_pose_with_covariance_stamped_free;
    static constexpr auto as_cdr   = ros_pose_with_covariance_stamped_as_cdr;
    static constexpr std::string_view name = "ros_pose_with_covariance_stamped";
};

struct TwistWithCovarianceStampedTraits {
    using handle_type = ros_twist_with_covariance_stamped_t;
    static constexpr auto from_cdr = ros_twist_with_covariance_stamped_from_cdr;
    static constexpr auto free     = ros_twist_with_covariance_stamped_free;
    static constexpr auto as_cdr   = ros_twist_with_covariance_stamped_as_cdr;
    static constexpr std::string_view name = "ros_twist_with_covariance_stamped";
};

struct AccelWithCovarianceStampedTraits {
    using handle_type = ros_accel_with_covariance_stamped_t;
    static constexpr auto from_cdr = ros_accel_with_covariance_stamped_from_cdr;
    static constexpr auto free     = ros_accel_with_covariance_stamped_free;
    static constexpr auto as_cdr   = ros_accel_with_covariance_stamped_as_cdr;
    static constexpr std::string_view name = "ros_accel_with_covariance_stamped";
};

struct PolygonTraits {
    using handle_type = ros_polygon_t;
    static constexpr auto from_cdr = ros_polygon_from_cdr;
    static constexpr auto free     = ros_polygon_free;
    static constexpr auto as_cdr   = ros_polygon_as_cdr;
    static constexpr std::string_view name = "ros_polygon";
};

struct PolygonStampedTraits {
    using handle_type = ros_polygon_stamped_t;
    static constexpr auto from_cdr = ros_polygon_stamped_from_cdr;
    static constexpr auto free     = ros_polygon_stamped_free;
    static constexpr auto as_cdr   = ros_polygon_stamped_as_cdr;
    static constexpr std::string_view name = "ros_polygon_stamped";
};

struct PoseArrayTraits {
    using handle_type = ros_pose_array_t;
    static constexpr auto from_cdr = ros_pose_array_from_cdr;
    static constexpr auto free     = ros_pose_array_free;
    static constexpr auto as_cdr   = ros_pose_array_as_cdr;
    static constexpr std::string_view name = "ros_pose_array";
};

struct CompressedVideoTraits {
    using handle_type = ros_compressed_video_t;
    static constexpr auto from_cdr = ros_compressed_video_from_cdr;
    static constexpr auto free     = ros_compressed_video_free;
    static constexpr auto as_cdr   = ros_compressed_video_as_cdr;
    static constexpr std::string_view name = "ros_compressed_video";
};

// MaskTraits: ros_mask_as_cdr does not exist in the C API (removed in the
// Task 4 refactor).  The as_cdr field is intentionally absent; MaskView uses
// ViewBaseNoCdr and Mask uses OwnedBaseNoCdr to avoid calling it.
struct MaskTraits {
    using handle_type = ros_mask_t;
    static constexpr auto from_cdr = ros_mask_from_cdr;
    static constexpr auto free     = ros_mask_free;
    static constexpr std::string_view name = "ros_mask";
};

// DmaBufferTraits wraps deprecated C entry points. The DmaBufferView /
// DmaBuffer class templates themselves carry [[deprecated]] at the class
// level, so end users still get a warning; suppress the transitive
// C-level warning here to keep the header clean on -Werror builds.
#if defined(__GNUC__) || defined(__clang__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
struct DmaBufferTraits {
    using handle_type = ros_dmabuffer_t;
    static constexpr auto from_cdr = ros_dmabuffer_from_cdr;
    static constexpr auto free     = ros_dmabuffer_free;
    static constexpr auto as_cdr   = ros_dmabuffer_as_cdr;
    static constexpr std::string_view name = "ros_dmabuffer";
};
#if defined(__GNUC__) || defined(__clang__)
#  pragma GCC diagnostic pop
#endif

struct CameraFrameTraits {
    using handle_type = ros_camera_frame_t;
    static constexpr auto from_cdr = ros_camera_frame_from_cdr;
    static constexpr auto free     = ros_camera_frame_free;
    static constexpr std::string_view name = "ros_camera_frame";
};

struct LocalTimeTraits {
    using handle_type = ros_local_time_t;
    static constexpr auto from_cdr = ros_local_time_from_cdr;
    static constexpr auto free     = ros_local_time_free;
    static constexpr auto as_cdr   = ros_local_time_as_cdr;
    static constexpr std::string_view name = "ros_local_time";
};

struct TrackTraits {
    using handle_type = ros_track_t;
    static constexpr auto from_cdr = ros_track_from_cdr;
    static constexpr auto free     = ros_track_free;
    static constexpr auto as_cdr   = ros_track_as_cdr;
    static constexpr std::string_view name = "ros_track";
};

struct ImageTraits {
    using handle_type = ros_image_t;
    static constexpr auto from_cdr = ros_image_from_cdr;
    static constexpr auto free     = ros_image_free;
    static constexpr auto as_cdr   = ros_image_as_cdr;
    static constexpr std::string_view name = "ros_image";
};

struct PointCloud2Traits {
    using handle_type = ros_point_cloud2_t;
    static constexpr auto from_cdr = ros_point_cloud2_from_cdr;
    static constexpr auto free     = ros_point_cloud2_free;
    static constexpr auto as_cdr   = ros_point_cloud2_as_cdr;
    static constexpr std::string_view name = "ros_point_cloud2";
};

struct RadarCubeTraits {
    using handle_type = ros_radar_cube_t;
    static constexpr auto from_cdr = ros_radar_cube_from_cdr;
    static constexpr auto free     = ros_radar_cube_free;
    static constexpr auto as_cdr   = ros_radar_cube_as_cdr;
    static constexpr std::string_view name = "ros_radar_cube";
};

struct RadarInfoTraits {
    using handle_type = ros_radar_info_t;
    static constexpr auto from_cdr = ros_radar_info_from_cdr;
    static constexpr auto free     = ros_radar_info_free;
    static constexpr auto as_cdr   = ros_radar_info_as_cdr;
    static constexpr std::string_view name = "ros_radar_info";
};

// BoxTraits: ros_box_as_cdr was removed in the Task 4 refactor.
// BoxView uses ViewBaseNoCdr; Box (owning) is not provided as there is no
// ros_box_encode in the C API.
struct BoxTraits {
    using handle_type = ros_box_t;
    static constexpr auto from_cdr = ros_box_from_cdr;
    static constexpr auto free     = ros_box_free;
    static constexpr std::string_view name = "ros_box";
};

struct DetectTraits {
    using handle_type = ros_detect_t;
    static constexpr auto from_cdr = ros_detect_from_cdr;
    static constexpr auto free     = ros_detect_free;
    static constexpr auto as_cdr   = ros_detect_as_cdr;
    static constexpr std::string_view name = "ros_detect";
};

struct ModelTraits {
    using handle_type = ros_model_t;
    static constexpr auto from_cdr = ros_model_from_cdr;
    static constexpr auto free     = ros_model_free;
    static constexpr auto as_cdr   = ros_model_as_cdr;
    static constexpr std::string_view name = "ros_model";
};

struct ModelInfoTraits {
    using handle_type = ros_model_info_t;
    static constexpr auto from_cdr = ros_model_info_from_cdr;
    static constexpr auto free     = ros_model_info_free;
    static constexpr auto as_cdr   = ros_model_info_as_cdr;
    static constexpr std::string_view name = "ros_model_info";
};

struct MagneticFieldTraits {
    using handle_type = ros_magnetic_field_t;
    static constexpr auto from_cdr = ros_magnetic_field_from_cdr;
    static constexpr auto free     = ros_magnetic_field_free;
    static constexpr auto as_cdr   = ros_magnetic_field_as_cdr;
    static constexpr std::string_view name = "ros_magnetic_field";
};

struct FluidPressureTraits {
    using handle_type = ros_fluid_pressure_t;
    static constexpr auto from_cdr = ros_fluid_pressure_from_cdr;
    static constexpr auto free     = ros_fluid_pressure_free;
    static constexpr auto as_cdr   = ros_fluid_pressure_as_cdr;
    static constexpr std::string_view name = "ros_fluid_pressure";
};

struct TemperatureTraits {
    using handle_type = ros_temperature_t;
    static constexpr auto from_cdr = ros_temperature_from_cdr;
    static constexpr auto free     = ros_temperature_free;
    static constexpr auto as_cdr   = ros_temperature_as_cdr;
    static constexpr std::string_view name = "ros_temperature";
};

struct BatteryStateTraits {
    using handle_type = ros_battery_state_t;
    static constexpr auto from_cdr = ros_battery_state_from_cdr;
    static constexpr auto free     = ros_battery_state_free;
    static constexpr auto as_cdr   = ros_battery_state_as_cdr;
    static constexpr std::string_view name = "ros_battery_state";
};

struct OdometryTraits {
    using handle_type = ros_odometry_t;
    static constexpr auto from_cdr = ros_odometry_from_cdr;
    static constexpr auto free     = ros_odometry_free;
    static constexpr auto as_cdr   = ros_odometry_as_cdr;
    static constexpr std::string_view name = "ros_odometry";
};

struct VibrationTraits {
    using handle_type = ros_vibration_t;
    static constexpr auto from_cdr = ros_vibration_from_cdr;
    static constexpr auto free     = ros_vibration_free;
    static constexpr auto as_cdr   = ros_vibration_as_cdr;
    static constexpr std::string_view name = "ros_vibration";
};

struct MavrosAltitudeTraits {
    using handle_type = ros_mavros_altitude_t;
    static constexpr auto from_cdr = ros_mavros_altitude_from_cdr;
    static constexpr auto free     = ros_mavros_altitude_free;
    static constexpr auto as_cdr   = ros_mavros_altitude_as_cdr;
    static constexpr std::string_view name = "ros_mavros_altitude";
};

struct MavrosVfrHudTraits {
    using handle_type = ros_mavros_vfrhud_t;
    static constexpr auto from_cdr = ros_mavros_vfrhud_from_cdr;
    static constexpr auto free     = ros_mavros_vfrhud_free;
    static constexpr auto as_cdr   = ros_mavros_vfrhud_as_cdr;
    static constexpr std::string_view name = "ros_mavros_vfrhud";
};

struct MavrosEstimatorStatusTraits {
    using handle_type = ros_mavros_estimator_status_t;
    static constexpr auto from_cdr = ros_mavros_estimator_status_from_cdr;
    static constexpr auto free     = ros_mavros_estimator_status_free;
    static constexpr auto as_cdr   = ros_mavros_estimator_status_as_cdr;
    static constexpr std::string_view name = "ros_mavros_estimator_status";
};

struct MavrosExtendedStateTraits {
    using handle_type = ros_mavros_extended_state_t;
    static constexpr auto from_cdr = ros_mavros_extended_state_from_cdr;
    static constexpr auto free     = ros_mavros_extended_state_free;
    static constexpr auto as_cdr   = ros_mavros_extended_state_as_cdr;
    static constexpr std::string_view name = "ros_mavros_extended_state";
};

struct MavrosSysStatusTraits {
    using handle_type = ros_mavros_sys_status_t;
    static constexpr auto from_cdr = ros_mavros_sys_status_from_cdr;
    static constexpr auto free     = ros_mavros_sys_status_free;
    static constexpr auto as_cdr   = ros_mavros_sys_status_as_cdr;
    static constexpr std::string_view name = "ros_mavros_sys_status";
};

struct MavrosStateTraits {
    using handle_type = ros_mavros_state_t;
    static constexpr auto from_cdr = ros_mavros_state_from_cdr;
    static constexpr auto free     = ros_mavros_state_free;
    static constexpr auto as_cdr   = ros_mavros_state_as_cdr;
    static constexpr std::string_view name = "ros_mavros_state";
};

struct MavrosStatusTextTraits {
    using handle_type = ros_mavros_status_text_t;
    static constexpr auto from_cdr = ros_mavros_status_text_from_cdr;
    static constexpr auto free     = ros_mavros_status_text_free;
    static constexpr auto as_cdr   = ros_mavros_status_text_as_cdr;
    static constexpr std::string_view name = "ros_mavros_status_text";
};

struct MavrosGpsRawTraits {
    using handle_type = ros_mavros_gps_raw_t;
    static constexpr auto from_cdr = ros_mavros_gps_raw_from_cdr;
    static constexpr auto free     = ros_mavros_gps_raw_free;
    static constexpr auto as_cdr   = ros_mavros_gps_raw_as_cdr;
    static constexpr std::string_view name = "ros_mavros_gps_raw";
};

struct MavrosTimesyncStatusTraits {
    using handle_type = ros_mavros_timesync_status_t;
    static constexpr auto from_cdr = ros_mavros_timesync_status_from_cdr;
    static constexpr auto free     = ros_mavros_timesync_status_free;
    static constexpr auto as_cdr   = ros_mavros_timesync_status_as_cdr;
    static constexpr std::string_view name = "ros_mavros_timesync_status";
};

// ---------------------------------------------------------------------------
// Builder traits structs — one per C builder type
// ---------------------------------------------------------------------------

struct HeaderBuilderTraits {
    using builder_type = ros_header_builder_t;
    static constexpr auto new_fn = ros_header_builder_new;
    static constexpr auto free_fn = ros_header_builder_free;
    static constexpr auto build_fn = ros_header_builder_build;
    static constexpr auto encode_into_fn = ros_header_builder_encode_into;
    static constexpr std::string_view name = "ros_header_builder";
    static constexpr std::string_view build_name = "ros_header_builder_build";
    static constexpr std::string_view encode_into_name = "ros_header_builder_encode_into";
};

struct ImageBuilderTraits {
    using builder_type = ros_image_builder_t;
    static constexpr auto new_fn = ros_image_builder_new;
    static constexpr auto free_fn = ros_image_builder_free;
    static constexpr auto build_fn = ros_image_builder_build;
    static constexpr auto encode_into_fn = ros_image_builder_encode_into;
    static constexpr std::string_view name = "ros_image_builder";
    static constexpr std::string_view build_name = "ros_image_builder_build";
    static constexpr std::string_view encode_into_name = "ros_image_builder_encode_into";
};

struct CompressedImageBuilderTraits {
    using builder_type = ros_compressed_image_builder_t;
    static constexpr auto new_fn = ros_compressed_image_builder_new;
    static constexpr auto free_fn = ros_compressed_image_builder_free;
    static constexpr auto build_fn = ros_compressed_image_builder_build;
    static constexpr auto encode_into_fn = ros_compressed_image_builder_encode_into;
    static constexpr std::string_view name = "ros_compressed_image_builder";
    static constexpr std::string_view build_name = "ros_compressed_image_builder_build";
    static constexpr std::string_view encode_into_name = "ros_compressed_image_builder_encode_into";
};

struct ImuBuilderTraits {
    using builder_type = ros_imu_builder_t;
    static constexpr auto new_fn = ros_imu_builder_new;
    static constexpr auto free_fn = ros_imu_builder_free;
    static constexpr auto build_fn = ros_imu_builder_build;
    static constexpr auto encode_into_fn = ros_imu_builder_encode_into;
    static constexpr std::string_view name = "ros_imu_builder";
    static constexpr std::string_view build_name = "ros_imu_builder_build";
    static constexpr std::string_view encode_into_name = "ros_imu_builder_encode_into";
};

struct NavSatFixBuilderTraits {
    using builder_type = ros_nav_sat_fix_builder_t;
    static constexpr auto new_fn = ros_nav_sat_fix_builder_new;
    static constexpr auto free_fn = ros_nav_sat_fix_builder_free;
    static constexpr auto build_fn = ros_nav_sat_fix_builder_build;
    static constexpr auto encode_into_fn = ros_nav_sat_fix_builder_encode_into;
    static constexpr std::string_view name = "ros_nav_sat_fix_builder";
    static constexpr std::string_view build_name = "ros_nav_sat_fix_builder_build";
    static constexpr std::string_view encode_into_name = "ros_nav_sat_fix_builder_encode_into";
};

struct PointFieldBuilderTraits {
    using builder_type = ros_point_field_builder_t;
    static constexpr auto new_fn = ros_point_field_builder_new;
    static constexpr auto free_fn = ros_point_field_builder_free;
    static constexpr auto build_fn = ros_point_field_builder_build;
    static constexpr auto encode_into_fn = ros_point_field_builder_encode_into;
    static constexpr std::string_view name = "ros_point_field_builder";
    static constexpr std::string_view build_name = "ros_point_field_builder_build";
    static constexpr std::string_view encode_into_name = "ros_point_field_builder_encode_into";
};

struct PointCloud2BuilderTraits {
    using builder_type = ros_point_cloud2_builder_t;
    static constexpr auto new_fn = ros_point_cloud2_builder_new;
    static constexpr auto free_fn = ros_point_cloud2_builder_free;
    static constexpr auto build_fn = ros_point_cloud2_builder_build;
    static constexpr auto encode_into_fn = ros_point_cloud2_builder_encode_into;
    static constexpr std::string_view name = "ros_point_cloud2_builder";
    static constexpr std::string_view build_name = "ros_point_cloud2_builder_build";
    static constexpr std::string_view encode_into_name = "ros_point_cloud2_builder_encode_into";
};

struct CameraInfoBuilderTraits {
    using builder_type = ros_camera_info_builder_t;
    static constexpr auto new_fn = ros_camera_info_builder_new;
    static constexpr auto free_fn = ros_camera_info_builder_free;
    static constexpr auto build_fn = ros_camera_info_builder_build;
    static constexpr auto encode_into_fn = ros_camera_info_builder_encode_into;
    static constexpr std::string_view name = "ros_camera_info_builder";
    static constexpr std::string_view build_name = "ros_camera_info_builder_build";
    static constexpr std::string_view encode_into_name = "ros_camera_info_builder_encode_into";
};

struct MagneticFieldBuilderTraits {
    using builder_type = ros_magnetic_field_builder_t;
    static constexpr auto new_fn = ros_magnetic_field_builder_new;
    static constexpr auto free_fn = ros_magnetic_field_builder_free;
    static constexpr auto build_fn = ros_magnetic_field_builder_build;
    static constexpr auto encode_into_fn = ros_magnetic_field_builder_encode_into;
    static constexpr std::string_view name = "ros_magnetic_field_builder";
    static constexpr std::string_view build_name = "ros_magnetic_field_builder_build";
    static constexpr std::string_view encode_into_name = "ros_magnetic_field_builder_encode_into";
};

struct FluidPressureBuilderTraits {
    using builder_type = ros_fluid_pressure_builder_t;
    static constexpr auto new_fn = ros_fluid_pressure_builder_new;
    static constexpr auto free_fn = ros_fluid_pressure_builder_free;
    static constexpr auto build_fn = ros_fluid_pressure_builder_build;
    static constexpr auto encode_into_fn = ros_fluid_pressure_builder_encode_into;
    static constexpr std::string_view name = "ros_fluid_pressure_builder";
    static constexpr std::string_view build_name = "ros_fluid_pressure_builder_build";
    static constexpr std::string_view encode_into_name = "ros_fluid_pressure_builder_encode_into";
};

struct TemperatureBuilderTraits {
    using builder_type = ros_temperature_builder_t;
    static constexpr auto new_fn = ros_temperature_builder_new;
    static constexpr auto free_fn = ros_temperature_builder_free;
    static constexpr auto build_fn = ros_temperature_builder_build;
    static constexpr auto encode_into_fn = ros_temperature_builder_encode_into;
    static constexpr std::string_view name = "ros_temperature_builder";
    static constexpr std::string_view build_name = "ros_temperature_builder_build";
    static constexpr std::string_view encode_into_name = "ros_temperature_builder_encode_into";
};

struct BatteryStateBuilderTraits {
    using builder_type = ros_battery_state_builder_t;
    static constexpr auto new_fn = ros_battery_state_builder_new;
    static constexpr auto free_fn = ros_battery_state_builder_free;
    static constexpr auto build_fn = ros_battery_state_builder_build;
    static constexpr auto encode_into_fn = ros_battery_state_builder_encode_into;
    static constexpr std::string_view name = "ros_battery_state_builder";
    static constexpr std::string_view build_name = "ros_battery_state_builder_build";
    static constexpr std::string_view encode_into_name = "ros_battery_state_builder_encode_into";
};

struct MaskBuilderTraits {
    using builder_type = ros_mask_builder_t;
    static constexpr auto new_fn = ros_mask_builder_new;
    static constexpr auto free_fn = ros_mask_builder_free;
    static constexpr auto build_fn = ros_mask_builder_build;
    static constexpr auto encode_into_fn = ros_mask_builder_encode_into;
    static constexpr std::string_view name = "ros_mask_builder";
    static constexpr std::string_view build_name = "ros_mask_builder_build";
    static constexpr std::string_view encode_into_name = "ros_mask_builder_encode_into";
};

struct LocalTimeBuilderTraits {
    using builder_type = ros_local_time_builder_t;
    static constexpr auto new_fn = ros_local_time_builder_new;
    static constexpr auto free_fn = ros_local_time_builder_free;
    static constexpr auto build_fn = ros_local_time_builder_build;
    static constexpr auto encode_into_fn = ros_local_time_builder_encode_into;
    static constexpr std::string_view name = "ros_local_time_builder";
    static constexpr std::string_view build_name = "ros_local_time_builder_build";
    static constexpr std::string_view encode_into_name = "ros_local_time_builder_encode_into";
};

struct RadarCubeBuilderTraits {
    using builder_type = ros_radar_cube_builder_t;
    static constexpr auto new_fn = ros_radar_cube_builder_new;
    static constexpr auto free_fn = ros_radar_cube_builder_free;
    static constexpr auto build_fn = ros_radar_cube_builder_build;
    static constexpr auto encode_into_fn = ros_radar_cube_builder_encode_into;
    static constexpr std::string_view name = "ros_radar_cube_builder";
    static constexpr std::string_view build_name = "ros_radar_cube_builder_build";
    static constexpr std::string_view encode_into_name = "ros_radar_cube_builder_encode_into";
};

struct RadarInfoBuilderTraits {
    using builder_type = ros_radar_info_builder_t;
    static constexpr auto new_fn = ros_radar_info_builder_new;
    static constexpr auto free_fn = ros_radar_info_builder_free;
    static constexpr auto build_fn = ros_radar_info_builder_build;
    static constexpr auto encode_into_fn = ros_radar_info_builder_encode_into;
    static constexpr std::string_view name = "ros_radar_info_builder";
    static constexpr std::string_view build_name = "ros_radar_info_builder_build";
    static constexpr std::string_view encode_into_name = "ros_radar_info_builder_encode_into";
};

struct TrackBuilderTraits {
    using builder_type = ros_track_builder_t;
    static constexpr auto new_fn = ros_track_builder_new;
    static constexpr auto free_fn = ros_track_builder_free;
    static constexpr auto build_fn = ros_track_builder_build;
    static constexpr auto encode_into_fn = ros_track_builder_encode_into;
    static constexpr std::string_view name = "ros_track_builder";
    static constexpr std::string_view build_name = "ros_track_builder_build";
    static constexpr std::string_view encode_into_name = "ros_track_builder_encode_into";
};

struct DetectBoxBuilderTraits {
    using builder_type = ros_detect_box_builder_t;
    static constexpr auto new_fn = ros_detect_box_builder_new;
    static constexpr auto free_fn = ros_detect_box_builder_free;
    static constexpr auto build_fn = ros_detect_box_builder_build;
    static constexpr auto encode_into_fn = ros_detect_box_builder_encode_into;
    static constexpr std::string_view name = "ros_detect_box_builder";
    static constexpr std::string_view build_name = "ros_detect_box_builder_build";
    static constexpr std::string_view encode_into_name = "ros_detect_box_builder_encode_into";
};

struct DetectBuilderTraits {
    using builder_type = ros_detect_builder_t;
    static constexpr auto new_fn = ros_detect_builder_new;
    static constexpr auto free_fn = ros_detect_builder_free;
    static constexpr auto build_fn = ros_detect_builder_build;
    static constexpr auto encode_into_fn = ros_detect_builder_encode_into;
    static constexpr std::string_view name = "ros_detect_builder";
    static constexpr std::string_view build_name = "ros_detect_builder_build";
    static constexpr std::string_view encode_into_name = "ros_detect_builder_encode_into";
};

struct CameraFrameBuilderTraits {
    using builder_type = ros_camera_frame_builder_t;
    static constexpr auto new_fn = ros_camera_frame_builder_new;
    static constexpr auto free_fn = ros_camera_frame_builder_free;
    static constexpr auto build_fn = ros_camera_frame_builder_build;
    static constexpr auto encode_into_fn = ros_camera_frame_builder_encode_into;
    static constexpr std::string_view name = "ros_camera_frame_builder";
    static constexpr std::string_view build_name = "ros_camera_frame_builder_build";
    static constexpr std::string_view encode_into_name = "ros_camera_frame_builder_encode_into";
};

struct ModelBuilderTraits {
    using builder_type = ros_model_builder_t;
    static constexpr auto new_fn = ros_model_builder_new;
    static constexpr auto free_fn = ros_model_builder_free;
    static constexpr auto build_fn = ros_model_builder_build;
    static constexpr auto encode_into_fn = ros_model_builder_encode_into;
    static constexpr std::string_view name = "ros_model_builder";
    static constexpr std::string_view build_name = "ros_model_builder_build";
    static constexpr std::string_view encode_into_name = "ros_model_builder_encode_into";
};

struct ModelInfoBuilderTraits {
    using builder_type = ros_model_info_builder_t;
    static constexpr auto new_fn = ros_model_info_builder_new;
    static constexpr auto free_fn = ros_model_info_builder_free;
    static constexpr auto build_fn = ros_model_info_builder_build;
    static constexpr auto encode_into_fn = ros_model_info_builder_encode_into;
    static constexpr std::string_view name = "ros_model_info_builder";
    static constexpr std::string_view build_name = "ros_model_info_builder_build";
    static constexpr std::string_view encode_into_name = "ros_model_info_builder_encode_into";
};

struct VibrationBuilderTraits {
    using builder_type = ros_vibration_builder_t;
    static constexpr auto new_fn = ros_vibration_builder_new;
    static constexpr auto free_fn = ros_vibration_builder_free;
    static constexpr auto build_fn = ros_vibration_builder_build;
    static constexpr auto encode_into_fn = ros_vibration_builder_encode_into;
    static constexpr std::string_view name = "ros_vibration_builder";
    static constexpr std::string_view build_name = "ros_vibration_builder_build";
    static constexpr std::string_view encode_into_name = "ros_vibration_builder_encode_into";
};

struct FoxgloveCompressedVideoBuilderTraits {
    using builder_type = ros_foxglove_compressed_video_builder_t;
    static constexpr auto new_fn = ros_foxglove_compressed_video_builder_new;
    static constexpr auto free_fn = ros_foxglove_compressed_video_builder_free;
    static constexpr auto build_fn = ros_foxglove_compressed_video_builder_build;
    static constexpr auto encode_into_fn = ros_foxglove_compressed_video_builder_encode_into;
    static constexpr std::string_view name = "ros_foxglove_compressed_video_builder";
    static constexpr std::string_view build_name = "ros_foxglove_compressed_video_builder_build";
    static constexpr std::string_view encode_into_name = "ros_foxglove_compressed_video_builder_encode_into";
};

struct FoxgloveTextAnnotationBuilderTraits {
    using builder_type = ros_foxglove_text_annotation_builder_t;
    static constexpr auto new_fn = ros_foxglove_text_annotation_builder_new;
    static constexpr auto free_fn = ros_foxglove_text_annotation_builder_free;
    static constexpr auto build_fn = ros_foxglove_text_annotation_builder_build;
    static constexpr auto encode_into_fn = ros_foxglove_text_annotation_builder_encode_into;
    static constexpr std::string_view name = "ros_foxglove_text_annotation_builder";
    static constexpr std::string_view build_name = "ros_foxglove_text_annotation_builder_build";
    static constexpr std::string_view encode_into_name = "ros_foxglove_text_annotation_builder_encode_into";
};

struct FoxglovePointAnnotationBuilderTraits {
    using builder_type = ros_foxglove_point_annotation_builder_t;
    static constexpr auto new_fn = ros_foxglove_point_annotation_builder_new;
    static constexpr auto free_fn = ros_foxglove_point_annotation_builder_free;
    static constexpr auto build_fn = ros_foxglove_point_annotation_builder_build;
    static constexpr auto encode_into_fn = ros_foxglove_point_annotation_builder_encode_into;
    static constexpr std::string_view name = "ros_foxglove_point_annotation_builder";
    static constexpr std::string_view build_name = "ros_foxglove_point_annotation_builder_build";
    static constexpr std::string_view encode_into_name = "ros_foxglove_point_annotation_builder_encode_into";
};

struct FoxgloveImageAnnotationBuilderTraits {
    using builder_type = ros_foxglove_image_annotation_builder_t;
    static constexpr auto new_fn = ros_foxglove_image_annotation_builder_new;
    static constexpr auto free_fn = ros_foxglove_image_annotation_builder_free;
    static constexpr auto build_fn = ros_foxglove_image_annotation_builder_build;
    static constexpr auto encode_into_fn = ros_foxglove_image_annotation_builder_encode_into;
    static constexpr std::string_view name = "ros_foxglove_image_annotation_builder";
    static constexpr std::string_view build_name = "ros_foxglove_image_annotation_builder_build";
    static constexpr std::string_view encode_into_name = "ros_foxglove_image_annotation_builder_encode_into";
};

/**
 * @internal
 * @brief Lightweight range adaptor over a parent's indexed child
 *        accessor.
 *
 * Yields `ChildView` values (either `BorrowedBoxView` or
 * `BorrowedMaskView`) when iterated. Each yielded child is non-owning and
 * borrows directly from the parent's CDR buffer — destroying the parent
 * invalidates all previously-yielded children.
 *
 * Template parameters:
 *   - `ParentHandle` : the owning handle type (e.g. `ros_detect_t`).
 *   - `ChildView`    : the element type yielded by the iterator.
 *   - `GetFn`        : the C accessor type, e.g.
 *                      `decltype(&ros_detect_get_box)`.
 *
 * Length is passed directly as `std::uint32_t`; there is no `LenFn`
 * parameter.
 *
 * @warning The parent view must outlive every yielded child and any
 *          references borrowed from them.
 */
template <typename ParentHandle, typename ChildView, typename GetFn>
class ChildRange {
public:
    /// @internal
    /// @brief Forward input iterator yielding `ChildView` by value.
    class iterator {
    public:
        using iterator_category = std::input_iterator_tag;
        using value_type        = ChildView;
        using difference_type   = std::ptrdiff_t;
        using pointer           = void;       // proxy iterator — dereference returns by value
        using reference         = ChildView;  // proxy reference

        constexpr iterator() noexcept = default;
        iterator(const ParentHandle* parent, std::uint32_t idx, GetFn get) noexcept
            : parent_(parent), idx_(idx), get_(get) {}

        [[nodiscard]] ChildView operator*() const noexcept {
            return ChildView{get_(parent_, idx_)};
        }

        iterator& operator++() noexcept { ++idx_; return *this; }
        iterator  operator++(int) noexcept { auto tmp = *this; ++idx_; return tmp; }

        [[nodiscard]] bool operator==(const iterator& o) const noexcept { return idx_ == o.idx_; }
        [[nodiscard]] bool operator!=(const iterator& o) const noexcept { return idx_ != o.idx_; }

    private:
        const ParentHandle* parent_{nullptr};
        std::uint32_t       idx_{0};
        GetFn               get_{};
    };

    /// @brief Construct a ChildRange bound to a parent handle.
    /// @param parent The owning parent handle; must outlive this range.
    /// @param len The number of children to expose.
    /// @param get The C accessor function pointer used to fetch each
    ///            child by index.
    ChildRange(const ParentHandle* parent, std::uint32_t len, GetFn get) noexcept
        : parent_(parent), len_(len), get_(get) {}

    /// @brief Iterator to the first child in the range.
    /// @return An iterator positioned at index 0.
    [[nodiscard]] iterator    begin()  const noexcept { return {parent_, 0,    get_}; }
    /// @brief Past-the-end iterator.
    /// @return An iterator positioned at index `size()`.
    [[nodiscard]] iterator    end()    const noexcept { return {parent_, len_, get_}; }
    /// @brief Number of children in the range.
    /// @return The length captured at construction time.
    [[nodiscard]] std::uint32_t size() const noexcept { return len_; }
    /// @brief Check whether the range is empty.
    /// @return `true` if `size() == 0`.
    [[nodiscard]] bool         empty() const noexcept { return len_ == 0; }

private:
    const ParentHandle* parent_;
    std::uint32_t       len_;
    GetFn               get_;
};

/**
 * @internal
 * @brief Non-owning accessor over a parent-borrowed `ros_box_t*`.
 *
 * Yielded by `DetectView::boxes()` / `ModelView::boxes()` iteration. The
 * handle is owned by the parent view and becomes invalid when the parent
 * is destroyed. Must NOT call `ros_box_free`; lifetime is tied to the
 * parent handle.
 *
 * Non-default-constructible and non-assignable to prevent dangling
 * uses. Copy/move construction is permitted so range-based for loops
 * work.
 *
 * @warning Do not store a BorrowedBoxView past the lifetime of its parent
 *          view. The parent view's CDR buffer must also remain alive for
 *          any `std::string_view` returned by `label()` / `track_id()` to
 *          remain valid.
 */
class BorrowedBoxView {
public:
    BorrowedBoxView() = delete;                                   ///< @brief Deleted: requires a valid handle.
    BorrowedBoxView(const BorrowedBoxView&) = default;            ///< @brief Trivial copy constructor.
    BorrowedBoxView& operator=(const BorrowedBoxView&) = delete;  ///< @brief Deleted: prevents storing past parent lifetime.
    BorrowedBoxView(BorrowedBoxView&&) = default;                 ///< @brief Trivial move constructor.
    BorrowedBoxView& operator=(BorrowedBoxView&&) = delete;       ///< @brief Deleted: prevents reassignment.

    /// @brief Wrap a borrowed child handle.
    /// @param h Non-null borrowed `ros_box_t*` owned by the parent view.
    explicit BorrowedBoxView(const ros_box_t* h) noexcept : handle_(h) {}

    /// @brief Box center X coordinate (image-space or normalized).
    /// @return The center X value from the underlying handle.
    [[nodiscard]] float            center_x()       const noexcept { return ros_box_get_center_x(handle_); }
    /// @brief Box center Y coordinate.
    /// @return The center Y value from the underlying handle.
    [[nodiscard]] float            center_y()       const noexcept { return ros_box_get_center_y(handle_); }
    /// @brief Box width.
    /// @return The width value from the underlying handle.
    [[nodiscard]] float            width()          const noexcept { return ros_box_get_width(handle_); }
    /// @brief Box height.
    /// @return The height value from the underlying handle.
    [[nodiscard]] float            height()         const noexcept { return ros_box_get_height(handle_); }
    /// @brief Class label of the detected object.
    /// @return A `std::string_view` borrowed from the parent CDR buffer.
    /// @warning Valid only while the parent view and its backing buffer
    ///          are alive.
    [[nodiscard]] std::string_view label()          const noexcept { return ros_box_get_label(handle_); }
    /// @brief Detection confidence score in [0, 1].
    /// @return The score value from the underlying handle.
    [[nodiscard]] float            score()          const noexcept { return ros_box_get_score(handle_); }
    /// @brief Estimated distance to the detected object (meters).
    /// @return The distance value from the underlying handle.
    [[nodiscard]] float            distance()       const noexcept { return ros_box_get_distance(handle_); }
    /// @brief Estimated object speed (m/s).
    /// @return The speed value from the underlying handle.
    [[nodiscard]] float            speed()          const noexcept { return ros_box_get_speed(handle_); }
    /// @brief Optional tracker identifier string.
    /// @return A `std::string_view` borrowed from the parent CDR buffer,
    ///         empty if untracked.
    /// @warning Valid only while the parent view and its backing buffer
    ///          are alive.
    [[nodiscard]] std::string_view track_id()       const noexcept { return ros_box_get_track_id(handle_); }
    /// @brief Number of frames this track has been alive.
    /// @return The track lifetime value.
    [[nodiscard]] std::int32_t     track_lifetime() const noexcept { return ros_box_get_track_lifetime(handle_); }
    /// @brief Time at which the track was first created.
    /// @return A Time value assembled from the handle's seconds and
    ///         nanoseconds fields.
    [[nodiscard]] Time             track_created()  const noexcept {
        return Time{ros_box_get_track_created_sec(handle_),
                    ros_box_get_track_created_nanosec(handle_)};
    }

private:
    const ros_box_t* handle_;
};

/**
 * @internal
 * @brief Non-owning accessor over a parent-borrowed `ros_mask_t*`.
 *
 * Yielded by `ModelView::masks()` iteration. The handle is owned by the
 * parent view and becomes invalid when the parent is destroyed. Must NOT
 * call `ros_mask_free`; lifetime is tied to the parent handle.
 *
 * @warning Do not store a BorrowedMaskView past the lifetime of its
 *          parent view. All references returned by `encoding()` and
 *          `data()` borrow directly into the parent's CDR buffer.
 */
class BorrowedMaskView {
public:
    BorrowedMaskView() = delete;                                    ///< @brief Deleted: requires a valid handle.
    BorrowedMaskView(const BorrowedMaskView&) = default;            ///< @brief Trivial copy constructor.
    BorrowedMaskView& operator=(const BorrowedMaskView&) = delete;  ///< @brief Deleted: prevents storing past parent lifetime.
    BorrowedMaskView(BorrowedMaskView&&) = default;                 ///< @brief Trivial move constructor.
    BorrowedMaskView& operator=(BorrowedMaskView&&) = delete;       ///< @brief Deleted: prevents reassignment.

    /// @brief Wrap a borrowed child handle.
    /// @param h Non-null borrowed `ros_mask_t*` owned by the parent view.
    explicit BorrowedMaskView(const ros_mask_t* h) noexcept : handle_(h) {}

    /// @brief Mask height in pixels.
    /// @return The height value from the underlying handle.
    [[nodiscard]] std::uint32_t height()   const noexcept { return ros_mask_get_height(handle_); }
    /// @brief Mask width in pixels.
    /// @return The width value from the underlying handle.
    [[nodiscard]] std::uint32_t width()    const noexcept { return ros_mask_get_width(handle_); }
    /// @brief Raw byte length of the mask payload.
    /// @return The length value from the underlying handle.
    [[nodiscard]] std::uint32_t length()   const noexcept { return ros_mask_get_length(handle_); }
    /// @brief Mask pixel encoding identifier (e.g. "mono8", "rle").
    /// @return A `std::string_view` borrowed from the parent CDR buffer.
    /// @warning Valid only while the parent view and its backing buffer
    ///          are alive.
    [[nodiscard]] std::string_view encoding() const noexcept { return ros_mask_get_encoding(handle_); }
    /// @brief Raw mask payload bytes.
    /// @return A `span<const std::uint8_t>` borrowing into the parent CDR
    ///         buffer.
    /// @warning Valid only while the parent view and its backing buffer
    ///          are alive.
    [[nodiscard]] span<const std::uint8_t> data() const noexcept {
        std::size_t n = 0;
        auto* p = ros_mask_get_data(handle_, &n);
        return {p, n};
    }
    /// @brief Whether the mask is constrained to a bounding box.
    /// @return `true` if the mask is stored as box-relative pixels.
    [[nodiscard]] bool boxed() const noexcept { return ros_mask_get_boxed(handle_); }

private:
    const ros_mask_t* handle_;
};

/**
 * @internal
 * @brief Non-owning accessor over a parent-borrowed
 *        `ros_camera_plane_t*`.
 *
 * Yielded by `CameraFrameView::planes()` iteration. The handle is owned by
 * the parent view and becomes invalid when the parent is destroyed. Must
 * NOT call `ros_camera_plane_free`; lifetime is tied to the parent.
 *
 * @warning Do not store past the lifetime of the parent. `data()` borrows
 *          directly into the parent's CDR buffer.
 */
class BorrowedCameraPlaneView {
public:
    BorrowedCameraPlaneView() = delete;
    BorrowedCameraPlaneView(const BorrowedCameraPlaneView&) = default;
    BorrowedCameraPlaneView& operator=(const BorrowedCameraPlaneView&) = delete;
    BorrowedCameraPlaneView(BorrowedCameraPlaneView&&) = default;
    BorrowedCameraPlaneView& operator=(BorrowedCameraPlaneView&&) = delete;

    explicit BorrowedCameraPlaneView(const ros_camera_plane_t* h) noexcept
        : handle_(h) {}

    /// @brief DMA-BUF file descriptor, or -1 if the plane bytes are inlined.
    [[nodiscard]] std::int32_t  fd()     const noexcept { return ros_camera_plane_get_fd(handle_); }
    /// @brief Byte offset of the plane within the fd (0 when fd == -1).
    [[nodiscard]] std::uint32_t offset() const noexcept { return ros_camera_plane_get_offset(handle_); }
    /// @brief Bytes per line of the plane.
    [[nodiscard]] std::uint32_t stride() const noexcept { return ros_camera_plane_get_stride(handle_); }
    /// @brief Plane capacity in bytes (buffer span).
    [[nodiscard]] std::uint32_t size()   const noexcept { return ros_camera_plane_get_size(handle_); }
    /// @brief Valid payload bytes (<= size; strictly less for compressed
    ///        bitstreams).
    [[nodiscard]] std::uint32_t used()   const noexcept { return ros_camera_plane_get_used(handle_); }

    /// @brief Inlined plane bytes (only non-empty when fd == -1).
    /// @return span borrowing into the parent's CDR buffer; empty for
    ///         fd-backed planes.
    [[nodiscard]] span<const std::uint8_t> data() const noexcept {
        std::size_t n = 0;
        auto* p = ros_camera_plane_get_data(handle_, &n);
        return {p, n};
    }

private:
    const ros_camera_plane_t* handle_;
};

} // namespace detail

// ============================================================================
// Concrete buffer-backed types
// ============================================================================

// ---------------------------------------------------------------------------
// std_msgs - Header
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `std_msgs::Header` message.
 *
 * Constructed via `from_cdr(span)` over a caller-supplied CDR buffer. All
 * field accessors return values or non-owning references.
 *
 * @warning The caller's byte buffer passed to `from_cdr()` must outlive
 *          this view and every `std::string_view` returned by
 *          `frame_id()`. Using a borrowed reference after the buffer is
 *          freed is undefined behavior.
 *
 * @note Move-only. Copy construction and copy assignment are deleted to
 *       prevent double-free of the underlying C handle.
 *
 * @see Header for the owning counterpart.
 */
class HeaderView : public detail::ViewBase<HeaderView, detail::HeaderTraits> {
    using Base = detail::ViewBase<HeaderView, detail::HeaderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Header timestamp.
    /// @return A Time value assembled from the handle's stamp fields.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_header_get_stamp_sec(handle()),
                ros_header_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_header_get_frame_id(handle());
    }
};

/**
 * @brief Owning, move-only `std_msgs::Header` instance.
 *
 * Holds both the CDR-encoded byte buffer (allocated by `encode()`) and a
 * view handle over that buffer. Move-only; the destructor frees the
 * encoded bytes via `ros_bytes_free()` and releases the C handle.
 *
 * Use `encode()` to construct. Accessors forward to an internal view over
 * this instance's own buffer, so they are always valid as long as the
 * instance is alive.
 *
 * @see HeaderView for the non-owning counterpart over a caller-supplied
 *      buffer.
 */
class Header : public detail::OwnedBase<Header, detail::HeaderTraits> {
    using Base = detail::OwnedBase<Header, detail::HeaderTraits>;
    friend Base;
public:
    /// @brief Encode a fresh Header into a newly-allocated CDR buffer.
    /// @param stamp Message timestamp.
    /// @param frame_id Coordinate frame identifier (non-null UTF-8).
    /// @return A new Header instance on success, or an Error on allocator
    ///         failure or invalid UTF-8.
    [[nodiscard]] static expected<Header, Error>
    encode(Time stamp, std::string_view frame_id) noexcept {
        std::uint8_t* out = nullptr; std::size_t len = 0;
        if (ros_header_encode(&out, &len, stamp.sec, stamp.nanosec,
                              frame_id.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_header_encode"));
        return make_(out, len);
    }
    /// @brief Header timestamp.
    /// @return A Time value assembled from the handle's stamp fields.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_header_get_stamp_sec(handle()),
                ros_header_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from this Header's internal
    ///         CDR buffer. Valid for the lifetime of this instance.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_header_get_frame_id(handle());
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - CompressedImage
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `sensor_msgs::CompressedImage`.
 *
 * Wraps a caller-supplied CDR buffer produced by a compressed image
 * publisher (e.g. JPEG or PNG). Field accessors borrow into the buffer
 * without copying.
 *
 * @warning The caller's byte buffer passed to `from_cdr()` must outlive
 *          this view and every borrowed reference (`frame_id()`,
 *          `format()`, `data()`).
 * @note Move-only.
 * @see CompressedImage for the owning counterpart.
 */
class CompressedImageView
    : public detail::ViewBase<CompressedImageView, detail::CompressedImageTraits> {
    using Base = detail::ViewBase<CompressedImageView, detail::CompressedImageTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    /// @return The header stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_compressed_image_get_stamp_sec(handle()),
                ros_compressed_image_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_compressed_image_get_frame_id(handle());
    }
    /// @brief Compression format label (e.g. "jpeg", "png").
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view format() const noexcept {
        return ros_compressed_image_get_format(handle());
    }
    /// @brief Compressed image payload.
    /// @return A `span<const std::uint8_t>` borrowed from the CDR buffer.
    /// @warning Invalidated when this view or its backing buffer is
    ///          destroyed.
    [[nodiscard]] span<const std::uint8_t> data() const noexcept {
        std::size_t n = 0;
        auto* p = ros_compressed_image_get_data(handle(), &n);
        return {p, n};
    }
};

/**
 * @brief Owning, move-only `sensor_msgs::CompressedImage` instance.
 *
 * Holds both the CDR-encoded byte buffer and a view handle over it.
 *
 * @see CompressedImageView for the non-owning counterpart.
 */
class CompressedImage
    : public detail::OwnedBase<CompressedImage, detail::CompressedImageTraits> {
    using Base = detail::OwnedBase<CompressedImage, detail::CompressedImageTraits>;
    friend Base;
public:
    /// @brief Encode a fresh CompressedImage into a newly-allocated CDR
    ///        buffer.
    /// @param stamp Message timestamp.
    /// @param frame_id Coordinate frame identifier (non-null UTF-8).
    /// @param format Compression format label (e.g. "jpeg", "png").
    /// @param data Compressed image bytes; copied into the encoded
    ///        buffer by the underlying C encoder.
    /// @return A new CompressedImage on success, or an Error on allocator
    ///         failure or invalid UTF-8.
    [[nodiscard]] static expected<CompressedImage, Error>
    encode(Time stamp, std::string_view frame_id, std::string_view format,
           span<const std::uint8_t> data) noexcept {
        std::uint8_t* out = nullptr; std::size_t len = 0;
        if (ros_compressed_image_encode(&out, &len,
                stamp.sec, stamp.nanosec,
                frame_id.data(), format.data(),
                data.data(), data.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_compressed_image_encode"));
        return make_(out, len);
    }
    /// @brief Message timestamp.
    /// @return The header stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_compressed_image_get_stamp_sec(handle()),
                ros_compressed_image_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from this instance's buffer.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_compressed_image_get_frame_id(handle());
    }
    /// @brief Compression format label.
    /// @return A `std::string_view` borrowed from this instance's buffer.
    [[nodiscard]] std::string_view format() const noexcept {
        return ros_compressed_image_get_format(handle());
    }
    /// @brief Compressed image payload.
    /// @return A `span<const std::uint8_t>` borrowed from this instance's
    ///         buffer.
    [[nodiscard]] span<const std::uint8_t> data() const noexcept {
        std::size_t n = 0;
        auto* p = ros_compressed_image_get_data(handle(), &n);
        return {p, n};
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - Imu (view-only: no ros_imu_encode in the C API)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `sensor_msgs::Imu` message.
 *
 * View-only: the C API does not provide `ros_imu_encode`, so there is no
 * owning `Imu` counterpart.
 *
 * @warning The caller's CDR buffer passed to `from_cdr()` must outlive
 *          this view and every borrowed reference.
 * @note Move-only.
 */
class ImuView : public detail::ViewBase<ImuView, detail::ImuTraits> {
    using Base = detail::ViewBase<ImuView, detail::ImuTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    /// @return The header stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_imu_get_stamp_sec(handle()), ros_imu_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_imu_get_frame_id(handle());
    }
    /// @brief Orientation quaternion.
    /// @return A Quaternion value copied from the handle.
    [[nodiscard]] Quaternion orientation() const noexcept {
        Quaternion q;
        ros_imu_get_orientation(handle(), &q.x, &q.y, &q.z, &q.w);
        return q;
    }
    /// @brief 3x3 orientation covariance matrix (row-major, 9 doubles).
    /// @return The covariance values copied into an owning
    ///         `std::array<double, 9>`.
    [[nodiscard]] std::array<double, 9> orientation_covariance() const noexcept {
        std::array<double, 9> cov{};
        ros_imu_get_orientation_covariance(handle(), cov.data());
        return cov;
    }
    /// @brief Angular velocity (rad/s).
    /// @return A Vector3 value copied from the handle.
    [[nodiscard]] Vector3 angular_velocity() const noexcept {
        Vector3 v;
        ros_imu_get_angular_velocity(handle(), &v.x, &v.y, &v.z);
        return v;
    }
    /// @brief 3x3 angular velocity covariance matrix (row-major).
    /// @return The covariance values copied into an owning
    ///         `std::array<double, 9>`.
    [[nodiscard]] std::array<double, 9> angular_velocity_covariance() const noexcept {
        std::array<double, 9> cov{};
        ros_imu_get_angular_velocity_covariance(handle(), cov.data());
        return cov;
    }
    /// @brief Linear acceleration (m/s²).
    /// @return A Vector3 value copied from the handle.
    [[nodiscard]] Vector3 linear_acceleration() const noexcept {
        Vector3 v;
        ros_imu_get_linear_acceleration(handle(), &v.x, &v.y, &v.z);
        return v;
    }
    /// @brief 3x3 linear acceleration covariance matrix (row-major).
    /// @return The covariance values copied into an owning
    ///         `std::array<double, 9>`.
    [[nodiscard]] std::array<double, 9> linear_acceleration_covariance() const noexcept {
        std::array<double, 9> cov{};
        ros_imu_get_linear_acceleration_covariance(handle(), cov.data());
        return cov;
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - NavSatFix (view-only: no ros_nav_sat_fix_encode in the C API)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `sensor_msgs::NavSatFix`.
 *
 * View-only: the C API does not provide `ros_nav_sat_fix_encode`.
 *
 * @warning Backing CDR buffer must outlive this view and any borrowed
 *          `frame_id()` string.
 * @note Move-only.
 */
class NavSatFixView : public detail::ViewBase<NavSatFixView, detail::NavSatFixTraits> {
    using Base = detail::ViewBase<NavSatFixView, detail::NavSatFixTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    /// @return The header stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_nav_sat_fix_get_stamp_sec(handle()),
                ros_nav_sat_fix_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_nav_sat_fix_get_frame_id(handle());
    }
    /// @brief Latitude in degrees (WGS84).
    /// @return Latitude value from the handle.
    [[nodiscard]] double latitude() const noexcept {
        return ros_nav_sat_fix_get_latitude(handle());
    }
    /// @brief Longitude in degrees (WGS84).
    /// @return Longitude value from the handle.
    [[nodiscard]] double longitude() const noexcept {
        return ros_nav_sat_fix_get_longitude(handle());
    }
    /// @brief Altitude in meters above the WGS84 ellipsoid.
    /// @return Altitude value from the handle.
    [[nodiscard]] double altitude() const noexcept {
        return ros_nav_sat_fix_get_altitude(handle());
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - CameraInfo (view-only: no ros_camera_info_encode in the C API)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `sensor_msgs::CameraInfo`.
 *
 * Exposes intrinsic parameters and distortion model labels. View-only
 * (no encode function in the C API).
 *
 * @warning Backing CDR buffer must outlive this view and any borrowed
 *          string references.
 * @note Move-only.
 */
class CameraInfoView : public detail::ViewBase<CameraInfoView, detail::CameraInfoTraits> {
    using Base = detail::ViewBase<CameraInfoView, detail::CameraInfoTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    /// @return The header stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_camera_info_get_stamp_sec(handle()),
                ros_camera_info_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_camera_info_get_frame_id(handle());
    }
    /// @brief Image height in pixels.
    /// @return The height value from the handle.
    [[nodiscard]] std::uint32_t height() const noexcept {
        return ros_camera_info_get_height(handle());
    }
    /// @brief Image width in pixels.
    /// @return The width value from the handle.
    [[nodiscard]] std::uint32_t width() const noexcept {
        return ros_camera_info_get_width(handle());
    }
    /// @brief Distortion model name (e.g. "plumb_bob", "rational_polynomial").
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view distortion_model() const noexcept {
        return ros_camera_info_get_distortion_model(handle());
    }
    /// @brief Horizontal binning factor (1 = unbinned).
    /// @return The binning_x value from the handle.
    [[nodiscard]] std::uint32_t binning_x() const noexcept {
        return ros_camera_info_get_binning_x(handle());
    }
    /// @brief Vertical binning factor (1 = unbinned).
    /// @return The binning_y value from the handle.
    [[nodiscard]] std::uint32_t binning_y() const noexcept {
        return ros_camera_info_get_binning_y(handle());
    }
};

// ---------------------------------------------------------------------------
// geometry_msgs - TransformStamped (view-only: no encode in the C API)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a
 *        `geometry_msgs::TransformStamped`.
 *
 * Represents a rigid-body transform between two coordinate frames with a
 * timestamp. View-only (no encode function in the C API).
 *
 * @warning Backing CDR buffer must outlive this view and any borrowed
 *          string references.
 * @note Move-only.
 */
class TransformStampedView
    : public detail::ViewBase<TransformStampedView, detail::TransformStampedTraits> {
    using Base = detail::ViewBase<TransformStampedView, detail::TransformStampedTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    /// @return The header stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_transform_stamped_get_stamp_sec(handle()),
                ros_transform_stamped_get_stamp_nanosec(handle())};
    }
    /// @brief Parent coordinate frame identifier.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_transform_stamped_get_frame_id(handle());
    }
    /// @brief Child coordinate frame identifier.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view child_frame_id() const noexcept {
        return ros_transform_stamped_get_child_frame_id(handle());
    }
};

// ---------------------------------------------------------------------------
// geometry_msgs - TwistStamped (view-only)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `geometry_msgs::TwistStamped`.
 *
 * @warning Backing CDR buffer must outlive this view.
 * @note Move-only.
 */
class TwistStampedView
    : public detail::ViewBase<TwistStampedView, detail::TwistStampedTraits> {
    using Base = detail::ViewBase<TwistStampedView, detail::TwistStampedTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_twist_stamped_get_stamp_sec(handle()),
                ros_twist_stamped_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_twist_stamped_get_frame_id(handle());
    }
};

// ---------------------------------------------------------------------------
// geometry_msgs - AccelStamped (view-only)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `geometry_msgs::AccelStamped`.
 *
 * @warning Backing CDR buffer must outlive this view.
 * @note Move-only.
 */
class AccelStampedView
    : public detail::ViewBase<AccelStampedView, detail::AccelStampedTraits> {
    using Base = detail::ViewBase<AccelStampedView, detail::AccelStampedTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_accel_stamped_get_stamp_sec(handle()),
                ros_accel_stamped_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_accel_stamped_get_frame_id(handle());
    }
};

// ---------------------------------------------------------------------------
// geometry_msgs - PointStamped (view-only)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `geometry_msgs::PointStamped`.
 *
 * @warning Backing CDR buffer must outlive this view.
 * @note Move-only.
 */
class PointStampedView
    : public detail::ViewBase<PointStampedView, detail::PointStampedTraits> {
    using Base = detail::ViewBase<PointStampedView, detail::PointStampedTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_point_stamped_get_stamp_sec(handle()),
                ros_point_stamped_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_point_stamped_get_frame_id(handle());
    }
};

// ---------------------------------------------------------------------------
// geometry_msgs - InertiaStamped (view-only)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `geometry_msgs::InertiaStamped`.
 *
 * @warning Backing CDR buffer must outlive this view.
 * @note Move-only.
 */
class InertiaStampedView
    : public detail::ViewBase<InertiaStampedView, detail::InertiaStampedTraits> {
    using Base = detail::ViewBase<InertiaStampedView, detail::InertiaStampedTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_inertia_stamped_get_stamp_sec(handle()),
                ros_inertia_stamped_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_inertia_stamped_get_frame_id(handle());
    }
};

// ---------------------------------------------------------------------------
// geometry_msgs - Vector3Stamped (view-only)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `geometry_msgs::Vector3Stamped`.
 *
 * @warning Backing CDR buffer must outlive this view.
 * @note Move-only.
 */
class Vector3StampedView
    : public detail::ViewBase<Vector3StampedView, detail::Vector3StampedTraits> {
    using Base = detail::ViewBase<Vector3StampedView, detail::Vector3StampedTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_vector3_stamped_get_stamp_sec(handle()),
                ros_vector3_stamped_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_vector3_stamped_get_frame_id(handle());
    }
};

// ---------------------------------------------------------------------------
// geometry_msgs - PoseStamped (view-only)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `geometry_msgs::PoseStamped`.
 *
 * @warning Backing CDR buffer must outlive this view.
 * @note Move-only.
 */
class PoseStampedView
    : public detail::ViewBase<PoseStampedView, detail::PoseStampedTraits> {
    using Base = detail::ViewBase<PoseStampedView, detail::PoseStampedTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_pose_stamped_get_stamp_sec(handle()),
                ros_pose_stamped_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_pose_stamped_get_frame_id(handle());
    }
};

// ---------------------------------------------------------------------------
// geometry_msgs - QuaternionStamped (view-only)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `geometry_msgs::QuaternionStamped`.
 *
 * @warning Backing CDR buffer must outlive this view.
 * @note Move-only.
 */
class QuaternionStampedView
    : public detail::ViewBase<QuaternionStampedView, detail::QuaternionStampedTraits> {
    using Base = detail::ViewBase<QuaternionStampedView, detail::QuaternionStampedTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_quaternion_stamped_get_stamp_sec(handle()),
                ros_quaternion_stamped_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_quaternion_stamped_get_frame_id(handle());
    }
};

// ---------------------------------------------------------------------------
// geometry_msgs - WrenchStamped (view-only)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `geometry_msgs::WrenchStamped`.
 *
 * @warning Backing CDR buffer must outlive this view.
 * @note Move-only.
 */
class WrenchStampedView
    : public detail::ViewBase<WrenchStampedView, detail::WrenchStampedTraits> {
    using Base = detail::ViewBase<WrenchStampedView, detail::WrenchStampedTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_wrench_stamped_get_stamp_sec(handle()),
                ros_wrench_stamped_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_wrench_stamped_get_frame_id(handle());
    }
};

// ---------------------------------------------------------------------------
// geometry_msgs - PoseWithCovarianceStamped (view-only)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `geometry_msgs::PoseWithCovarianceStamped`.
 *
 * @warning Backing CDR buffer must outlive this view.
 * @note Move-only.
 */
class PoseWithCovarianceStampedView
    : public detail::ViewBase<PoseWithCovarianceStampedView, detail::PoseWithCovarianceStampedTraits> {
    using Base = detail::ViewBase<PoseWithCovarianceStampedView, detail::PoseWithCovarianceStampedTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_pose_with_covariance_stamped_get_stamp_sec(handle()),
                ros_pose_with_covariance_stamped_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_pose_with_covariance_stamped_get_frame_id(handle());
    }
};

// ---------------------------------------------------------------------------
// geometry_msgs - TwistWithCovarianceStamped (view-only)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `geometry_msgs::TwistWithCovarianceStamped`.
 *
 * @warning Backing CDR buffer must outlive this view.
 * @note Move-only.
 */
class TwistWithCovarianceStampedView
    : public detail::ViewBase<TwistWithCovarianceStampedView, detail::TwistWithCovarianceStampedTraits> {
    using Base = detail::ViewBase<TwistWithCovarianceStampedView, detail::TwistWithCovarianceStampedTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_twist_with_covariance_stamped_get_stamp_sec(handle()),
                ros_twist_with_covariance_stamped_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_twist_with_covariance_stamped_get_frame_id(handle());
    }
};

// ---------------------------------------------------------------------------
// geometry_msgs - AccelWithCovarianceStamped (view-only)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `geometry_msgs::AccelWithCovarianceStamped`.
 *
 * @warning Backing CDR buffer must outlive this view.
 * @note Move-only.
 */
class AccelWithCovarianceStampedView
    : public detail::ViewBase<AccelWithCovarianceStampedView, detail::AccelWithCovarianceStampedTraits> {
    using Base = detail::ViewBase<AccelWithCovarianceStampedView, detail::AccelWithCovarianceStampedTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_accel_with_covariance_stamped_get_stamp_sec(handle()),
                ros_accel_with_covariance_stamped_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_accel_with_covariance_stamped_get_frame_id(handle());
    }
};

// ---------------------------------------------------------------------------
// geometry_msgs - Polygon (view-only, sequence type)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `geometry_msgs::Polygon`.
 *
 * @warning Backing CDR buffer must outlive this view.
 * @note Move-only.
 */
class PolygonView
    : public detail::ViewBase<PolygonView, detail::PolygonTraits> {
    using Base = detail::ViewBase<PolygonView, detail::PolygonTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Number of points in the polygon.
    [[nodiscard]] std::size_t size() const noexcept {
        return ros_polygon_get_len(handle());
    }
};

// ---------------------------------------------------------------------------
// geometry_msgs - PolygonStamped (view-only)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `geometry_msgs::PolygonStamped`.
 *
 * @warning Backing CDR buffer must outlive this view.
 * @note Move-only.
 */
class PolygonStampedView
    : public detail::ViewBase<PolygonStampedView, detail::PolygonStampedTraits> {
    using Base = detail::ViewBase<PolygonStampedView, detail::PolygonStampedTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_polygon_stamped_get_stamp_sec(handle()),
                ros_polygon_stamped_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_polygon_stamped_get_frame_id(handle());
    }
    /// @brief Number of points in the polygon.
    [[nodiscard]] std::size_t size() const noexcept {
        return ros_polygon_stamped_get_len(handle());
    }
};

// ---------------------------------------------------------------------------
// geometry_msgs - PoseArray (view-only)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `geometry_msgs::PoseArray`.
 *
 * @warning Backing CDR buffer must outlive this view.
 * @note Move-only.
 */
class PoseArrayView
    : public detail::ViewBase<PoseArrayView, detail::PoseArrayTraits> {
    using Base = detail::ViewBase<PoseArrayView, detail::PoseArrayTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_pose_array_get_stamp_sec(handle()),
                ros_pose_array_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_pose_array_get_frame_id(handle());
    }
    /// @brief Number of poses in the array.
    [[nodiscard]] std::size_t size() const noexcept {
        return ros_pose_array_get_len(handle());
    }
};

// ---------------------------------------------------------------------------
// foxglove_msgs - CompressedVideo
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `foxglove_msgs::CompressedVideo`.
 *
 * Wraps a CDR payload carrying a compressed video frame (H.264, H.265,
 * VP9, AV1, …) together with its metadata.
 *
 * @warning Backing CDR buffer must outlive this view and every borrowed
 *          reference returned by `frame_id()`, `format()`, `data()`.
 * @note Move-only.
 * @see CompressedVideo for the owning counterpart.
 */
class CompressedVideoView
    : public detail::ViewBase<CompressedVideoView, detail::CompressedVideoTraits> {
    using Base = detail::ViewBase<CompressedVideoView, detail::CompressedVideoTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    /// @return The header stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_compressed_video_get_stamp_sec(handle()),
                ros_compressed_video_get_stamp_nanosec(handle())};
    }
    /// Alias for stamp(); matches the Foxglove schema field name.
    [[nodiscard]] Time timestamp() const noexcept { return stamp(); }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_compressed_video_get_frame_id(handle());
    }
    /// @brief Video codec identifier (e.g. "h264", "h265", "av1").
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view format() const noexcept {
        return ros_compressed_video_get_format(handle());
    }
    /// @brief Compressed video payload bytes.
    /// @return A `span<const std::uint8_t>` borrowed from the CDR buffer.
    /// @warning Invalidated when this view or its backing buffer is
    ///          destroyed.
    [[nodiscard]] span<const std::uint8_t> data() const noexcept {
        std::size_t n = 0;
        auto* p = ros_compressed_video_get_data(handle(), &n);
        return {p, n};
    }
};

/**
 * @brief Owning, move-only `foxglove_msgs::CompressedVideo` instance.
 *
 * Holds both the CDR-encoded byte buffer and a view handle over it.
 *
 * @see CompressedVideoView for the non-owning counterpart.
 */
class CompressedVideo
    : public detail::OwnedBase<CompressedVideo, detail::CompressedVideoTraits> {
    using Base = detail::OwnedBase<CompressedVideo, detail::CompressedVideoTraits>;
    friend Base;
public:
    /// @brief Encode a fresh CompressedVideo into a newly-allocated CDR
    ///        buffer.
    /// @param stamp Message timestamp.
    /// @param frame_id Coordinate frame identifier (non-null UTF-8).
    /// @param data Compressed video payload; copied into the encoded
    ///        buffer by the underlying C encoder.
    /// @param format Codec identifier (e.g. "h264", "h265").
    /// @return A new CompressedVideo on success, or an Error on allocator
    ///         failure or invalid UTF-8.
    [[nodiscard]] static expected<CompressedVideo, Error>
    encode(Time stamp, std::string_view frame_id,
           span<const std::uint8_t> data, std::string_view format) noexcept {
        std::uint8_t* out = nullptr; std::size_t len = 0;
        if (ros_compressed_video_encode(&out, &len,
                stamp.sec, stamp.nanosec,
                frame_id.data(),
                data.data(), data.size(),
                format.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_compressed_video_encode"));
        return make_(out, len);
    }
    /// @brief Message timestamp.
    /// @return The header stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_compressed_video_get_stamp_sec(handle()),
                ros_compressed_video_get_stamp_nanosec(handle())};
    }
    /// Alias for stamp(); matches the Foxglove schema field name.
    [[nodiscard]] Time timestamp() const noexcept { return stamp(); }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from this instance's buffer.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_compressed_video_get_frame_id(handle());
    }
    /// @brief Codec identifier.
    /// @return A `std::string_view` borrowed from this instance's buffer.
    [[nodiscard]] std::string_view format() const noexcept {
        return ros_compressed_video_get_format(handle());
    }
    /// @brief Compressed video payload bytes.
    /// @return A `span<const std::uint8_t>` borrowed from this instance's
    ///         buffer.
    [[nodiscard]] span<const std::uint8_t> data() const noexcept {
        std::size_t n = 0;
        auto* p = ros_compressed_video_get_data(handle(), &n);
        return {p, n};
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - Mask
//
// Note: ros_mask_as_cdr does not exist in the C API (removed in the Task 4
// refactor).  MaskView uses ViewBaseNoCdr (no as_cdr accessor).  Mask (owning)
// uses OwnedBaseNoCdr whose as_cdr() returns the stored encoded bytes directly.
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over an `edgefirst_msgs::Mask` message.
 *
 * Represents a pixel mask (full-image or bounding-box relative) published
 * by segmentation or instance-masking models. Unlike most views, MaskView
 * does not expose `as_cdr()` because the underlying C API does not
 * provide `ros_mask_as_cdr`.
 *
 * @warning Backing CDR buffer must outlive this view and any references
 *          returned by `encoding()` / `data()`.
 * @note Move-only.
 * @see Mask for the owning counterpart.
 */
class MaskView : public detail::ViewBaseNoCdr<MaskView, detail::MaskTraits> {
    using Base = detail::ViewBaseNoCdr<MaskView, detail::MaskTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;

    /// @brief Mask height in pixels.
    /// @return The height value from the handle.
    [[nodiscard]] std::uint32_t height() const noexcept {
        return ros_mask_get_height(handle());
    }
    /// @brief Mask width in pixels.
    /// @return The width value from the handle.
    [[nodiscard]] std::uint32_t width() const noexcept {
        return ros_mask_get_width(handle());
    }
    /// @brief Raw byte length of the mask payload.
    /// @return The length value from the handle.
    [[nodiscard]] std::uint32_t length() const noexcept {
        return ros_mask_get_length(handle());
    }
    /// @brief Mask pixel encoding identifier.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view encoding() const noexcept {
        return ros_mask_get_encoding(handle());
    }
    /// @brief Raw mask payload bytes.
    /// @return A `span<const std::uint8_t>` borrowed from the CDR buffer.
    /// @warning Invalidated when this view or its backing buffer is
    ///          destroyed.
    [[nodiscard]] span<const std::uint8_t> data() const noexcept {
        std::size_t n = 0;
        auto* p = ros_mask_get_data(handle(), &n);
        return {p, n};
    }
    /// @brief Whether the mask is stored as box-relative pixels.
    /// @return `true` if the mask is clipped to a bounding box.
    [[nodiscard]] bool boxed() const noexcept {
        return ros_mask_get_boxed(handle());
    }
};

/**
 * @brief Owning, move-only `edgefirst_msgs::Mask` instance.
 *
 * Holds both the CDR-encoded byte buffer and a view handle over it. Uses
 * `OwnedBaseNoCdr` because the C API does not expose `ros_mask_as_cdr`;
 * `as_cdr()` returns the stored encoded bytes directly.
 *
 * @see MaskView for the non-owning counterpart.
 */
class Mask : public detail::OwnedBaseNoCdr<Mask, detail::MaskTraits> {
    using Base = detail::OwnedBaseNoCdr<Mask, detail::MaskTraits>;
    friend Base;
public:
    /// @brief Encode a fresh Mask into a newly-allocated CDR buffer.
    /// @param height Mask height in pixels.
    /// @param width Mask width in pixels.
    /// @param length Raw byte length of the mask payload.
    /// @param encoding Encoding identifier (e.g. "mono8", "rle").
    /// @param data Mask payload bytes; copied into the encoded buffer.
    /// @param boxed Whether the mask is stored as box-relative pixels.
    /// @return A new Mask on success, or an Error on allocator failure or
    ///         invalid UTF-8 in `encoding`.
    [[nodiscard]] static expected<Mask, Error>
    encode(std::uint32_t height, std::uint32_t width, std::uint32_t length,
           std::string_view encoding, span<const std::uint8_t> data,
           bool boxed) noexcept {
        std::uint8_t* out = nullptr; std::size_t len = 0;
        if (ros_mask_encode(&out, &len,
                height, width, length,
                encoding.data(),
                data.data(), data.size(),
                boxed) != 0)
            return unexpected<Error>(Error::from_errno("ros_mask_encode"));
        return make_(out, len);
    }
    /// @brief Mask height in pixels.
    /// @return The height value from the handle.
    [[nodiscard]] std::uint32_t height() const noexcept {
        return ros_mask_get_height(handle());
    }
    /// @brief Mask width in pixels.
    /// @return The width value from the handle.
    [[nodiscard]] std::uint32_t width() const noexcept {
        return ros_mask_get_width(handle());
    }
    /// @brief Raw byte length of the mask payload.
    /// @return The length value from the handle.
    [[nodiscard]] std::uint32_t length() const noexcept {
        return ros_mask_get_length(handle());
    }
    /// @brief Mask pixel encoding identifier.
    /// @return A `std::string_view` borrowed from this instance's buffer.
    [[nodiscard]] std::string_view encoding() const noexcept {
        return ros_mask_get_encoding(handle());
    }
    /// @brief Raw mask payload bytes.
    /// @return A `span<const std::uint8_t>` borrowed from this instance's
    ///         buffer.
    [[nodiscard]] span<const std::uint8_t> data() const noexcept {
        std::size_t n = 0;
        auto* p = ros_mask_get_data(handle(), &n);
        return {p, n};
    }
    /// @brief Whether the mask is stored as box-relative pixels.
    /// @return `true` if the mask is clipped to a bounding box.
    [[nodiscard]] bool boxed() const noexcept {
        return ros_mask_get_boxed(handle());
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - DmaBuffer
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over an `edgefirst_msgs::DmaBuffer`
 *        zero-copy handle descriptor.
 *
 * Carries references (not pixel data) to a Linux DMA-BUF allocated by an
 * external process. Used for zero-copy inter-process image transfer.
 *
 * @warning Backing CDR buffer must outlive this view and any borrowed
 *          `frame_id()` string.
 * @note Move-only.
 * @see DmaBuffer for the owning counterpart.
 */
#if defined(__GNUC__) || defined(__clang__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
class [[deprecated("Use CameraFrameView instead; DmaBufferView will be removed in 4.0.0")]]
DmaBufferView : public detail::ViewBase<DmaBufferView, detail::DmaBufferTraits> {
    using Base = detail::ViewBase<DmaBufferView, detail::DmaBufferTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    /// @return The header stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_dmabuffer_get_stamp_sec(handle()),
                ros_dmabuffer_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_dmabuffer_get_frame_id(handle());
    }
    /// @brief PID of the process that owns the DMA-BUF file descriptor.
    /// @return The owning PID.
    [[nodiscard]] std::uint32_t pid() const noexcept {
        return ros_dmabuffer_get_pid(handle());
    }
    /// @brief The DMA-BUF file descriptor (valid in the owning process).
    /// @return The file descriptor number.
    [[nodiscard]] std::int32_t fd() const noexcept {
        return ros_dmabuffer_get_fd(handle());
    }
    /// @brief Image width in pixels.
    /// @return The width value from the handle.
    [[nodiscard]] std::uint32_t width() const noexcept {
        return ros_dmabuffer_get_width(handle());
    }
    /// @brief Image height in pixels.
    /// @return The height value from the handle.
    [[nodiscard]] std::uint32_t height() const noexcept {
        return ros_dmabuffer_get_height(handle());
    }
    /// @brief Row stride in bytes.
    /// @return The stride value from the handle.
    [[nodiscard]] std::uint32_t stride() const noexcept {
        return ros_dmabuffer_get_stride(handle());
    }
    /// @brief 4-character code identifying the pixel format
    ///        (e.g. V4L2 / DRM fourcc).
    /// @return The fourcc value from the handle.
    [[nodiscard]] std::uint32_t fourcc() const noexcept {
        return ros_dmabuffer_get_fourcc(handle());
    }
    /// @brief Total length of the DMA-BUF in bytes.
    /// @return The length value from the handle.
    [[nodiscard]] std::uint32_t length() const noexcept {
        return ros_dmabuffer_get_length(handle());
    }
};

/**
 * @brief Owning, move-only `edgefirst_msgs::DmaBuffer` instance.
 *
 * Holds both the CDR-encoded byte buffer and a view handle over it.
 *
 * @see DmaBufferView for the non-owning counterpart.
 */
class [[deprecated("Use CameraFrameView + inlined-data planes instead; DmaBuffer will be removed in 4.0.0")]]
DmaBuffer : public detail::OwnedBase<DmaBuffer, detail::DmaBufferTraits> {
    using Base = detail::OwnedBase<DmaBuffer, detail::DmaBufferTraits>;
    friend Base;
public:
    /// @brief Encode a fresh DmaBuffer descriptor into a newly-allocated
    ///        CDR buffer.
    /// @param stamp Message timestamp.
    /// @param frame_id Coordinate frame identifier (non-null UTF-8).
    /// @param pid Owning process PID.
    /// @param fd DMA-BUF file descriptor number.
    /// @param width Image width in pixels.
    /// @param height Image height in pixels.
    /// @param stride Row stride in bytes.
    /// @param fourcc 4-character code identifying the pixel format.
    /// @param length Total DMA-BUF length in bytes.
    /// @return A new DmaBuffer on success, or an Error on allocator
    ///         failure or invalid UTF-8 in `frame_id`.
    [[nodiscard]] static expected<DmaBuffer, Error>
    encode(Time stamp, std::string_view frame_id,
           std::uint32_t pid, std::int32_t fd,
           std::uint32_t width, std::uint32_t height,
           std::uint32_t stride, std::uint32_t fourcc,
           std::uint32_t length) noexcept {
        std::uint8_t* out = nullptr; std::size_t len = 0;
        if (ros_dmabuffer_encode(&out, &len,
                stamp.sec, stamp.nanosec,
                frame_id.data(),
                pid, fd,
                width, height,
                stride, fourcc, length) != 0)
            return unexpected<Error>(Error::from_errno("ros_dmabuffer_encode"));
        return make_(out, len);
    }
    /// @brief Message timestamp.
    /// @return The header stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_dmabuffer_get_stamp_sec(handle()),
                ros_dmabuffer_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from this instance's buffer.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_dmabuffer_get_frame_id(handle());
    }
    /// @brief Owning process PID.
    /// @return The PID stored in the handle.
    [[nodiscard]] std::uint32_t pid() const noexcept {
        return ros_dmabuffer_get_pid(handle());
    }
    /// @brief DMA-BUF file descriptor.
    /// @return The file descriptor number.
    [[nodiscard]] std::int32_t fd() const noexcept {
        return ros_dmabuffer_get_fd(handle());
    }
    /// @brief Image width in pixels.
    /// @return The width value from the handle.
    [[nodiscard]] std::uint32_t width() const noexcept {
        return ros_dmabuffer_get_width(handle());
    }
    /// @brief Image height in pixels.
    /// @return The height value from the handle.
    [[nodiscard]] std::uint32_t height() const noexcept {
        return ros_dmabuffer_get_height(handle());
    }
    /// @brief Row stride in bytes.
    /// @return The stride value from the handle.
    [[nodiscard]] std::uint32_t stride() const noexcept {
        return ros_dmabuffer_get_stride(handle());
    }
    /// @brief Pixel format fourcc.
    /// @return The fourcc value from the handle.
    [[nodiscard]] std::uint32_t fourcc() const noexcept {
        return ros_dmabuffer_get_fourcc(handle());
    }
    /// @brief Total DMA-BUF length in bytes.
    /// @return The length value from the handle.
    [[nodiscard]] std::uint32_t length() const noexcept {
        return ros_dmabuffer_get_length(handle());
    }
};
#if defined(__GNUC__) || defined(__clang__)
#  pragma GCC diagnostic pop
#endif

// ---------------------------------------------------------------------------
// edgefirst_msgs - CameraFrame (view-only, multi-plane)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over an `edgefirst_msgs::CameraFrame`.
 *
 * Multi-plane video frame reference: DMA-BUF fds (one per plane) and/or
 * inlined per-plane bytes (for off-device bridges), plus frame-level
 * metadata (sequence counter, colorimetry, GPU fence fd). Supersedes
 * `DmaBufferView` for all new code.
 *
 * View-only (no `ros_camera_frame_encode` in the C API).
 *
 * @warning Backing CDR buffer must outlive this view and every
 *          `BorrowedCameraPlaneView` yielded by `planes()`.
 * @note Move-only.
 *
 * @code{.cpp}
 * namespace ef = edgefirst::schemas;
 * auto cf = ef::CameraFrameView::from_cdr(payload);
 * if (!cf) return;
 * for (auto p : cf->planes()) {
 *     if (p.fd() >= 0) mmap_plane(p.fd(), p.offset(), p.size(), p.used());
 *     else             consume_inlined(p.data());
 * }
 * @endcode
 */
class CameraFrameView
    : public detail::ViewBaseNoCdr<CameraFrameView, detail::CameraFrameTraits> {
    using Base = detail::ViewBaseNoCdr<CameraFrameView, detail::CameraFrameTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;

    /// @brief Message timestamp.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_camera_frame_get_stamp_sec(handle()),
                ros_camera_frame_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier (borrowed from CDR buffer).
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_camera_frame_get_frame_id(handle());
    }
    /// @brief Monotonic frame index (from V4L2 sequence / libcamera).
    [[nodiscard]] std::uint64_t seq() const noexcept {
        return ros_camera_frame_get_seq(handle());
    }
    /// @brief Producer process id (0 when all planes inlined).
    [[nodiscard]] std::uint32_t pid() const noexcept {
        return ros_camera_frame_get_pid(handle());
    }
    /// @brief Image width in pixels.
    [[nodiscard]] std::uint32_t width()  const noexcept {
        return ros_camera_frame_get_width(handle());
    }
    /// @brief Image height in pixels.
    [[nodiscard]] std::uint32_t height() const noexcept {
        return ros_camera_frame_get_height(handle());
    }
    /// @brief DMA-fence sync_file fd (-1 = already signalled / no fence).
    [[nodiscard]] std::int32_t fence_fd() const noexcept {
        return ros_camera_frame_get_fence_fd(handle());
    }

    /// @brief Format descriptor (e.g. "NV12", "rgb8_planar_nchw", "h264").
    [[nodiscard]] std::string_view format() const noexcept {
        return ros_camera_frame_get_format(handle());
    }
    /// @brief Color primaries (e.g. "bt709", "srgb", "bt2020", "").
    [[nodiscard]] std::string_view color_space() const noexcept {
        return ros_camera_frame_get_color_space(handle());
    }
    /// @brief Transfer function (e.g. "bt709", "srgb", "pq", "hlg", "").
    [[nodiscard]] std::string_view color_transfer() const noexcept {
        return ros_camera_frame_get_color_transfer(handle());
    }
    /// @brief YCbCr encoding matrix (e.g. "bt601", "bt709", "bt2020", "").
    [[nodiscard]] std::string_view color_encoding() const noexcept {
        return ros_camera_frame_get_color_encoding(handle());
    }
    /// @brief Sample range ("full", "limited", or "" for unknown).
    [[nodiscard]] std::string_view color_range() const noexcept {
        return ros_camera_frame_get_color_range(handle());
    }

    /// @brief Number of planes.
    [[nodiscard]] std::uint32_t planes_len() const noexcept {
        return ros_camera_frame_get_planes_len(handle());
    }

    /// @brief Range adaptor over the planes.
    /// @return A `detail::ChildRange` yielding `detail::BorrowedCameraPlaneView`
    ///         elements; each borrows into this view's CDR buffer.
    [[nodiscard]] auto planes() const noexcept {
        return detail::ChildRange<ros_camera_frame_t,
                                   detail::BorrowedCameraPlaneView,
                                   decltype(&ros_camera_frame_get_plane)>{
            handle(),
            ros_camera_frame_get_planes_len(handle()),
            ros_camera_frame_get_plane
        };
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - LocalTime (view-only: no ros_local_time_encode in C API)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over an `edgefirst_msgs::LocalTime`
 *        message (wall-clock timestamp + timezone offset).
 *
 * View-only (no encode function in the C API).
 *
 * @warning Backing CDR buffer must outlive this view and any borrowed
 *          `frame_id()` string.
 * @note Move-only.
 */
class LocalTimeView : public detail::ViewBase<LocalTimeView, detail::LocalTimeTraits> {
    using Base = detail::ViewBase<LocalTimeView, detail::LocalTimeTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    /// @return The header stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_local_time_get_stamp_sec(handle()),
                ros_local_time_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_local_time_get_frame_id(handle());
    }
    /// @brief Timezone offset from UTC in minutes.
    /// @return The offset value from the handle.
    [[nodiscard]] std::int16_t timezone() const noexcept {
        return ros_local_time_get_timezone(handle());
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - Track (view-only: no ros_track_encode in C API)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over an `edgefirst_msgs::Track`
 *        standalone tracker message.
 *
 * View-only (no encode function in the C API).
 *
 * @warning Backing CDR buffer must outlive this view and any borrowed
 *          `id()` string.
 * @note Move-only.
 */
class TrackView : public detail::ViewBase<TrackView, detail::TrackTraits> {
    using Base = detail::ViewBase<TrackView, detail::TrackTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Track identifier string.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view id() const noexcept {
        return ros_track_get_id(handle());
    }
    /// @brief Track lifetime (number of frames this track has existed).
    /// @return The lifetime count from the handle.
    [[nodiscard]] std::int32_t lifetime() const noexcept {
        return ros_track_get_lifetime(handle());
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - Image
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `sensor_msgs::Image` message.
 *
 * Constructed via `from_cdr(span)` over a caller-supplied CDR buffer. All
 * field accessors return values or non-owning references
 * (`std::string_view` for strings, `span<const std::uint8_t>` for the
 * pixel data blob). No bytes are copied on construction or access — the
 * entire wrapper is a pointer into the caller's buffer plus an offset
 * table.
 *
 * @warning The caller's byte buffer passed to `from_cdr()` must outlive
 *          this view AND every borrowed reference returned by its
 *          accessors. Using `frame_id()`, `encoding()`, or `data()` after
 *          the buffer is freed is undefined behavior.
 *
 * @note Move-only. Copy construction and copy assignment are deleted to
 *       prevent double-free of the underlying C handle.
 *
 * @code{.cpp}
 * namespace ef = edgefirst::schemas;
 * std::vector<std::uint8_t> cdr_bytes = receive_from_wire();
 * auto img = ef::ImageView::from_cdr({cdr_bytes.data(), cdr_bytes.size()});
 * if (!img) { log_error(img.error()); return; }
 * std::cout << img->width() << "x" << img->height()
 *           << " " << img->encoding() << "\n";
 * @endcode
 *
 * @see Image for the owning counterpart that encodes bytes at
 *      construction.
 */
class ImageView : public detail::ViewBase<ImageView, detail::ImageTraits> {
    using Base = detail::ViewBase<ImageView, detail::ImageTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp (inherited from the `std_msgs/Header`
    ///        field).
    /// @return The stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_image_get_stamp_sec(handle()),
                ros_image_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame name identifying the camera.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this ImageView and its backing CDR buffer
    ///       remain alive.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_image_get_frame_id(handle());
    }
    /// @brief Image height in pixels (number of rows).
    /// @return The height value from the handle.
    [[nodiscard]] std::uint32_t height() const noexcept {
        return ros_image_get_height(handle());
    }
    /// @brief Image width in pixels (number of columns).
    /// @return The width value from the handle.
    [[nodiscard]] std::uint32_t width() const noexcept {
        return ros_image_get_width(handle());
    }
    /// @brief Pixel format encoding string — e.g. "rgb8", "bgr8",
    ///        "mono8".
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view encoding() const noexcept {
        return ros_image_get_encoding(handle());
    }
    /// @brief Whether the pixel data is stored in big-endian byte order.
    /// @return Non-zero if big-endian, zero otherwise.
    [[nodiscard]] std::uint8_t is_bigendian() const noexcept {
        return ros_image_get_is_bigendian(handle());
    }
    /// @brief Row stride in bytes (≥ `width() * bytes_per_pixel`).
    /// @return The step value from the handle.
    [[nodiscard]] std::uint32_t step() const noexcept {
        return ros_image_get_step(handle());
    }
    /// @brief Raw pixel data payload.
    /// @return A `span<const std::uint8_t>` borrowing directly into the
    ///         CDR buffer. Length is `step() * height()`.
    /// @warning Invalidated when this ImageView or its backing CDR buffer
    ///          is destroyed.
    [[nodiscard]] span<const std::uint8_t> data() const noexcept {
        std::size_t n = 0;
        auto* p = ros_image_get_data(handle(), &n);
        return {p, n};
    }
};

/**
 * @brief Owning, move-only `sensor_msgs::Image` instance.
 *
 * Holds both the CDR-encoded byte buffer (allocated by `encode()`) and a
 * view handle over that buffer. Move-only; the destructor frees the
 * encoded bytes via `ros_bytes_free()` and releases the C handle.
 *
 * Use `encode()` to construct. The resulting instance exposes all the
 * same accessors as `ImageView` — those accessors forward to an internal
 * view over this instance's own buffer, so they are always valid as long
 * as the instance is alive.
 *
 * @see ImageView for the non-owning counterpart over a caller-supplied
 *      buffer.
 */
class Image : public detail::OwnedBase<Image, detail::ImageTraits> {
    using Base = detail::OwnedBase<Image, detail::ImageTraits>;
    friend Base;
public:
    /**
     * @brief Encode a fresh Image into a newly-allocated CDR buffer.
     *
     * @param stamp Message timestamp.
     * @param frame_id Coordinate frame name (non-null UTF-8).
     * @param height Image height in pixels.
     * @param width Image width in pixels.
     * @param encoding Pixel format (e.g. "rgb8", "bgr8", "mono8").
     * @param is_bigendian Big-endian pixel storage flag.
     * @param step Row stride in bytes.
     * @param data Raw pixel data; the wrapper copies this into the
     *        encoded buffer (this is the single legitimate copy of pixel
     *        data, performed by the C encode function).
     * @return A new Image instance on success, or an Error on allocator
     *         failure or invalid UTF-8.
     * @note The only allocation in this function is the internal
     *       `ros_image_encode` buffer allocation by the Rust runtime.
     */
    [[nodiscard]] static expected<Image, Error>
    encode(Time stamp, std::string_view frame_id,
           std::uint32_t height, std::uint32_t width,
           std::string_view encoding, bool is_bigendian,
           std::uint32_t step, span<const std::uint8_t> data) noexcept {
        std::uint8_t* out = nullptr; std::size_t len = 0;
        if (ros_image_encode(&out, &len,
                stamp.sec, stamp.nanosec,
                frame_id.data(),
                height, width,
                encoding.data(),
                static_cast<std::uint8_t>(is_bigendian),
                step,
                data.data(), data.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_image_encode"));
        return make_(out, len);
    }
    /// @brief Timestamp of the encoded message.
    /// @return The stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_image_get_stamp_sec(handle()),
                ros_image_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame name.
    /// @return A `std::string_view` borrowed from this instance's buffer.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_image_get_frame_id(handle());
    }
    /// @brief Height in pixels.
    /// @return The height value from the handle.
    [[nodiscard]] std::uint32_t height() const noexcept {
        return ros_image_get_height(handle());
    }
    /// @brief Width in pixels.
    /// @return The width value from the handle.
    [[nodiscard]] std::uint32_t width() const noexcept {
        return ros_image_get_width(handle());
    }
    /// @brief Pixel encoding string.
    /// @return A `std::string_view` borrowed from this instance's buffer.
    [[nodiscard]] std::string_view encoding() const noexcept {
        return ros_image_get_encoding(handle());
    }
    /// @brief Big-endian flag.
    /// @return Non-zero if big-endian, zero otherwise.
    [[nodiscard]] std::uint8_t is_bigendian() const noexcept {
        return ros_image_get_is_bigendian(handle());
    }
    /// @brief Row stride in bytes.
    /// @return The step value from the handle.
    [[nodiscard]] std::uint32_t step() const noexcept {
        return ros_image_get_step(handle());
    }
    /// @brief Pixel data payload.
    /// @return A `span<const std::uint8_t>` borrowed from this instance's
    ///         buffer.
    [[nodiscard]] span<const std::uint8_t> data() const noexcept {
        std::size_t n = 0;
        auto* p = ros_image_get_data(handle(), &n);
        return {p, n};
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - PointCloud2 (view-only: no ros_point_cloud2_encode in C API)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a `sensor_msgs::PointCloud2`.
 *
 * Wraps a CDR payload describing a 2D-organized or unorganized point
 * cloud. View-only (no encode function in the C API).
 *
 * @warning Backing CDR buffer must outlive this view and any borrowed
 *          references returned by `frame_id()` / `data()`.
 * @note Move-only.
 */
class PointCloud2View
    : public detail::ViewBase<PointCloud2View, detail::PointCloud2Traits> {
    using Base = detail::ViewBase<PointCloud2View, detail::PointCloud2Traits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    /// @return The header stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_point_cloud2_get_stamp_sec(handle()),
                ros_point_cloud2_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_point_cloud2_get_frame_id(handle());
    }
    /// @brief Cloud height in points (rows). For unorganized clouds this
    ///        is 1.
    /// @return The height value from the handle.
    [[nodiscard]] std::uint32_t height() const noexcept {
        return ros_point_cloud2_get_height(handle());
    }
    /// @brief Cloud width in points (columns) — total points for
    ///        unorganized clouds.
    /// @return The width value from the handle.
    [[nodiscard]] std::uint32_t width() const noexcept {
        return ros_point_cloud2_get_width(handle());
    }
    /// @brief Length of a single point in bytes (sum of field sizes +
    ///        padding).
    /// @return The point_step value from the handle.
    [[nodiscard]] std::uint32_t point_step() const noexcept {
        return ros_point_cloud2_get_point_step(handle());
    }
    /// @brief Length of a single row in bytes
    ///        (`point_step() * width()`).
    /// @return The row_step value from the handle.
    [[nodiscard]] std::uint32_t row_step() const noexcept {
        return ros_point_cloud2_get_row_step(handle());
    }
    /// @brief Raw point cloud payload.
    /// @return A `span<const std::uint8_t>` borrowed from the CDR buffer.
    /// @warning Invalidated when this view or its backing buffer is
    ///          destroyed.
    [[nodiscard]] span<const std::uint8_t> data() const noexcept {
        std::size_t n = 0;
        auto* p = ros_point_cloud2_get_data(handle(), &n);
        return {p, n};
    }
    /// @brief Whether the cloud contains no invalid / NaN points.
    /// @return `true` if dense.
    [[nodiscard]] bool is_dense() const noexcept {
        return ros_point_cloud2_get_is_dense(handle());
    }
    /// @brief Whether field values are stored big-endian.
    /// @return `true` if big-endian.
    [[nodiscard]] bool is_bigendian() const noexcept {
        return ros_point_cloud2_get_is_bigendian(handle());
    }
    /// @brief Number of point fields declared in the PointField array.
    /// @return The fields array length.
    [[nodiscard]] std::uint32_t fields_len() const noexcept {
        return ros_point_cloud2_get_fields_len(handle());
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - RadarCube (view-only: no ros_radar_cube_encode in C API)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over an `edgefirst_msgs::RadarCube`.
 *
 * Wraps a raw multidimensional radar data cube (range × Doppler × angle
 * or similar) plus its serialized layout descriptor. View-only.
 *
 * @warning Backing CDR buffer must outlive this view and any borrowed
 *          references returned by `frame_id()`, `layout()`, `cube_raw()`.
 * @note Move-only.
 */
class RadarCubeView
    : public detail::ViewBase<RadarCubeView, detail::RadarCubeTraits> {
    using Base = detail::ViewBase<RadarCubeView, detail::RadarCubeTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    /// @return The header stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_radar_cube_get_stamp_sec(handle()),
                ros_radar_cube_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_radar_cube_get_frame_id(handle());
    }
    /// @brief Hardware-supplied timestamp counter (vendor specific).
    /// @return The raw 64-bit timestamp value.
    [[nodiscard]] std::uint64_t timestamp() const noexcept {
        return ros_radar_cube_get_timestamp(handle());
    }
    /// @brief Serialized layout descriptor for the cube's shape/dtype.
    /// @return A `span<const std::uint8_t>` borrowed from the CDR buffer.
    /// @warning Invalidated when this view or its backing buffer is
    ///          destroyed.
    [[nodiscard]] span<const std::uint8_t> layout() const noexcept {
        std::size_t n = 0;
        auto* p = ros_radar_cube_get_layout(handle(), &n);
        return {p, n};
    }
    /// @brief Raw cube bytes as described by `layout()`.
    /// @return A `span<const std::uint8_t>` borrowed from the CDR buffer.
    /// @warning Invalidated when this view or its backing buffer is
    ///          destroyed.
    [[nodiscard]] span<const std::uint8_t> cube_raw() const noexcept {
        std::size_t n = 0;
        auto* p = ros_radar_cube_get_cube_raw(handle(), &n);
        return {p, n};
    }
    /// @brief Total number of cube samples (not bytes).
    /// @return The sample count from the handle.
    [[nodiscard]] std::uint32_t cube_len() const noexcept {
        return ros_radar_cube_get_cube_len(handle());
    }
    /// @brief Whether cube samples are complex (I/Q) values.
    /// @return `true` if complex-valued.
    [[nodiscard]] bool is_complex() const noexcept {
        return ros_radar_cube_get_is_complex(handle());
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - RadarInfo (view-only: no ros_radar_info_encode in C API)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over an `edgefirst_msgs::RadarInfo`
 *        configuration / capabilities message.
 *
 * View-only (no encode function in the C API).
 *
 * @warning Backing CDR buffer must outlive this view and any borrowed
 *          string references.
 * @note Move-only.
 */
class RadarInfoView
    : public detail::ViewBase<RadarInfoView, detail::RadarInfoTraits> {
    using Base = detail::ViewBase<RadarInfoView, detail::RadarInfoTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    /// @return The header stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_radar_info_get_stamp_sec(handle()),
                ros_radar_info_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_radar_info_get_frame_id(handle());
    }
    /// @brief Currently configured center frequency (vendor-defined
    ///        label, e.g. "77GHz").
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view center_frequency() const noexcept {
        return ros_radar_info_get_center_frequency(handle());
    }
    /// @brief Current frequency-sweep configuration label.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view frequency_sweep() const noexcept {
        return ros_radar_info_get_frequency_sweep(handle());
    }
    /// @brief Current range-toggle configuration label.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view range_toggle() const noexcept {
        return ros_radar_info_get_range_toggle(handle());
    }
    /// @brief Current detection-sensitivity configuration label.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view detection_sensitivity() const noexcept {
        return ros_radar_info_get_detection_sensitivity(handle());
    }
    /// @brief Whether the radar is currently publishing raw cube data.
    /// @return `true` if cube publishing is enabled.
    [[nodiscard]] bool cube() const noexcept {
        return ros_radar_info_get_cube(handle());
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - BoxView
//
// Note: ros_box_as_cdr was removed in the Task 4 refactor, so BoxView uses
// ViewBaseNoCdr (no as_cdr accessor).  There is also no ros_box_encode in
// the C API, so no owning Box type is provided.
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over a standalone `edgefirst_msgs::Box`
 *        message.
 *
 * Unlike `detail::BorrowedBoxView` (yielded by iteration over
 * `DetectView::boxes()`), this class wraps a top-level box message
 * decoded from its own CDR buffer. View-only: neither
 * `ros_box_as_cdr` nor `ros_box_encode` exist in the C API, so no
 * `as_cdr()` accessor and no owning `Box` counterpart are provided.
 *
 * @warning Backing CDR buffer must outlive this view and any borrowed
 *          string references returned by `label()` / `track_id()`.
 * @note Move-only.
 */
class BoxView : public detail::ViewBaseNoCdr<BoxView, detail::BoxTraits> {
    using Base = detail::ViewBaseNoCdr<BoxView, detail::BoxTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;

    /// @brief Box center X coordinate.
    /// @return The center X value from the handle.
    [[nodiscard]] float            center_x()       const noexcept { return ros_box_get_center_x(handle()); }
    /// @brief Box center Y coordinate.
    /// @return The center Y value from the handle.
    [[nodiscard]] float            center_y()       const noexcept { return ros_box_get_center_y(handle()); }
    /// @brief Box width.
    /// @return The width value from the handle.
    [[nodiscard]] float            width()          const noexcept { return ros_box_get_width(handle()); }
    /// @brief Box height.
    /// @return The height value from the handle.
    [[nodiscard]] float            height()         const noexcept { return ros_box_get_height(handle()); }
    /// @brief Class label of the detected object.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view label()          const noexcept { return ros_box_get_label(handle()); }
    /// @brief Detection confidence score in [0, 1].
    /// @return The score value from the handle.
    [[nodiscard]] float            score()          const noexcept { return ros_box_get_score(handle()); }
    /// @brief Estimated distance to the detected object (meters).
    /// @return The distance value from the handle.
    [[nodiscard]] float            distance()       const noexcept { return ros_box_get_distance(handle()); }
    /// @brief Estimated object speed (m/s).
    /// @return The speed value from the handle.
    [[nodiscard]] float            speed()          const noexcept { return ros_box_get_speed(handle()); }
    /// @brief Optional tracker identifier string.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view track_id()       const noexcept { return ros_box_get_track_id(handle()); }
    /// @brief Number of frames this track has been alive.
    /// @return The lifetime value from the handle.
    [[nodiscard]] std::int32_t     track_lifetime() const noexcept { return ros_box_get_track_lifetime(handle()); }
    /// @brief Time at which the track was first created.
    /// @return A Time value assembled from the handle.
    [[nodiscard]] Time             track_created()  const noexcept {
        return Time{ros_box_get_track_created_sec(handle()),
                    ros_box_get_track_created_nanosec(handle())};
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - DetectView (view-only: no ros_detect_encode in C API)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over an `edgefirst_msgs::Detect`
 *        message — a header plus an array of detection boxes.
 *
 * Detections are exposed via the `boxes()` range adaptor, which yields
 * `detail::BorrowedBoxView` values on iteration. Each yielded child
 * borrows into the parent's CDR buffer and becomes invalid when this
 * DetectView is destroyed.
 *
 * View-only (no encode function in the C API).
 *
 * @warning Backing CDR buffer must outlive this view, every yielded box
 *          child, and any borrowed references returned by accessors.
 * @note Move-only.
 *
 * @code{.cpp}
 * namespace ef = edgefirst::schemas;
 * auto det = ef::DetectView::from_cdr(payload);
 * if (!det) return;
 * for (auto box : det->boxes()) {
 *     std::cout << box.label() << " score=" << box.score() << "\n";
 * }
 * @endcode
 */
class DetectView : public detail::ViewBase<DetectView, detail::DetectTraits> {
    using Base = detail::ViewBase<DetectView, detail::DetectTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    /// @return The header stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_detect_get_stamp_sec(handle()),
                ros_detect_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_detect_get_frame_id(handle());
    }
    /// @brief Number of detection boxes in this message.
    /// @return The box array length.
    [[nodiscard]] std::uint32_t boxes_len() const noexcept {
        return ros_detect_get_boxes_len(handle());
    }

    /// @brief Range adaptor over the detection boxes.
    /// @return A `detail::ChildRange` yielding `detail::BorrowedBoxView`
    ///         elements; each yielded child borrows into this view's CDR
    ///         buffer and is only valid while this DetectView is alive.
    [[nodiscard]] auto boxes() const noexcept {
        return detail::ChildRange<ros_detect_t,
                                  detail::BorrowedBoxView,
                                  decltype(&ros_detect_get_box)>{
            handle(),
            ros_detect_get_boxes_len(handle()),
            ros_detect_get_box
        };
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - ModelView (view-only: no ros_model_encode in C API)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over an `edgefirst_msgs::Model`
 *        combined detection + segmentation message.
 *
 * Contains both an array of detection boxes and an array of masks. Both
 * are exposed via range adaptors (`boxes()` and `masks()`) yielding
 * parent-borrowed child views.
 *
 * View-only (no encode function in the C API).
 *
 * @warning Backing CDR buffer must outlive this view, every yielded child
 *          from `boxes()` / `masks()`, and any borrowed references.
 * @note Move-only.
 *
 * @code{.cpp}
 * namespace ef = edgefirst::schemas;
 * auto m = ef::ModelView::from_cdr(payload);
 * if (!m) return;
 * for (auto box : m->boxes())  { use(box);  }
 * for (auto mask : m->masks()) { use(mask); }
 * @endcode
 */
class ModelView : public detail::ViewBase<ModelView, detail::ModelTraits> {
    using Base = detail::ViewBase<ModelView, detail::ModelTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    /// @return The header stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_model_get_stamp_sec(handle()),
                ros_model_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_model_get_frame_id(handle());
    }
    /// @brief Number of detection boxes in this message.
    /// @return The box array length.
    [[nodiscard]] std::uint32_t boxes_len() const noexcept {
        return ros_model_get_boxes_len(handle());
    }
    /// @brief Number of segmentation masks in this message.
    /// @return The mask array length.
    [[nodiscard]] std::uint32_t masks_len() const noexcept {
        return ros_model_get_masks_len(handle());
    }

    /// @brief Range adaptor over the detection boxes.
    /// @return A `detail::ChildRange` yielding `detail::BorrowedBoxView`
    ///         children whose lifetime is tied to this ModelView.
    [[nodiscard]] auto boxes() const noexcept {
        return detail::ChildRange<ros_model_t,
                                  detail::BorrowedBoxView,
                                  decltype(&ros_model_get_box)>{
            handle(),
            ros_model_get_boxes_len(handle()),
            ros_model_get_box
        };
    }

    /// @brief Range adaptor over the segmentation masks.
    /// @return A `detail::ChildRange` yielding `detail::BorrowedMaskView`
    ///         children whose lifetime is tied to this ModelView.
    [[nodiscard]] auto masks() const noexcept {
        return detail::ChildRange<ros_model_t,
                                  detail::BorrowedMaskView,
                                  decltype(&ros_model_get_mask)>{
            handle(),
            ros_model_get_masks_len(handle()),
            ros_model_get_mask
        };
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - ModelInfoView (view-only: no ros_model_info_encode in C API)
// ---------------------------------------------------------------------------

/**
 * @brief Non-owning, move-only view over an `edgefirst_msgs::ModelInfo`
 *        message describing a deployed inference model.
 *
 * Exposes model type/format/name, input and output shapes, and the set
 * of class labels the model was trained on. View-only (no encode
 * function in the C API).
 *
 * @warning Backing CDR buffer must outlive this view and every borrowed
 *          reference — string accessors and `input_shape()`,
 *          `output_shape()`, `label(i)` all borrow directly into the CDR
 *          buffer.
 * @note Move-only.
 */
class ModelInfoView
    : public detail::ViewBase<ModelInfoView, detail::ModelInfoTraits> {
    using Base = detail::ViewBase<ModelInfoView, detail::ModelInfoTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    /// @brief Message timestamp.
    /// @return The header stamp as a Time value.
    [[nodiscard]] Time stamp() const noexcept {
        return {ros_model_info_get_stamp_sec(handle()),
                ros_model_info_get_stamp_nanosec(handle())};
    }
    /// @brief Coordinate frame identifier.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_model_info_get_frame_id(handle());
    }
    /// @brief Model architecture type (e.g. "detection", "segmentation").
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view model_type() const noexcept {
        return ros_model_info_get_model_type(handle());
    }
    /// @brief Model serialization format (e.g. "tflite", "onnx").
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view model_format() const noexcept {
        return ros_model_info_get_model_format(handle());
    }
    /// @brief Human-readable model name.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view model_name() const noexcept {
        return ros_model_info_get_model_name(handle());
    }
    /// @brief Numeric input tensor dtype identifier (model-specific).
    /// @return The input type code from the handle.
    [[nodiscard]] std::uint8_t input_type() const noexcept {
        return ros_model_info_get_input_type(handle());
    }
    /// @brief Numeric output tensor dtype identifier (model-specific).
    /// @return The output type code from the handle.
    [[nodiscard]] std::uint8_t output_type() const noexcept {
        return ros_model_info_get_output_type(handle());
    }
    /// @brief Input tensor shape as an array of dimensions.
    /// @return A `span<const std::uint32_t>` borrowed from the CDR
    ///         buffer.
    /// @warning Invalidated when this view or its backing buffer is
    ///          destroyed.
    [[nodiscard]] span<const std::uint32_t> input_shape() const noexcept {
        std::size_t n = 0;
        auto* p = ros_model_info_get_input_shape(handle(), &n);
        return {p, n};
    }
    /// @brief Output tensor shape as an array of dimensions.
    /// @return A `span<const std::uint32_t>` borrowed from the CDR
    ///         buffer.
    /// @warning Invalidated when this view or its backing buffer is
    ///          destroyed.
    [[nodiscard]] span<const std::uint32_t> output_shape() const noexcept {
        std::size_t n = 0;
        auto* p = ros_model_info_get_output_shape(handle(), &n);
        return {p, n};
    }
    /// @brief Number of class labels known to the model.
    /// @return The labels array length.
    [[nodiscard]] std::uint32_t labels_len() const noexcept {
        return ros_model_info_get_labels_len(handle());
    }
    /// @brief Look up the class label at a given index.
    /// @param index Zero-based label index; must be less than
    ///        `labels_len()`.
    /// @return A `std::string_view` borrowed from the CDR buffer.
    /// @note Valid only while this view and its backing buffer are alive.
    [[nodiscard]] std::string_view label(std::uint32_t index) const noexcept {
        return ros_model_info_get_label(handle(), index);
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - MagneticField (view-only)
// ---------------------------------------------------------------------------

/// @brief Non-owning, move-only view over a `sensor_msgs::MagneticField`.
class MagneticFieldView
    : public detail::ViewBase<MagneticFieldView, detail::MagneticFieldTraits> {
    using Base = detail::ViewBase<MagneticFieldView, detail::MagneticFieldTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    [[nodiscard]] Time stamp() const noexcept {
        return {ros_magnetic_field_get_stamp_sec(handle()),
                ros_magnetic_field_get_stamp_nanosec(handle())};
    }
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_magnetic_field_get_frame_id(handle());
    }
    [[nodiscard]] Vector3 magnetic_field() const noexcept {
        Vector3 v;
        ros_magnetic_field_get_magnetic_field(handle(), &v.x, &v.y, &v.z);
        return v;
    }
    [[nodiscard]] std::array<double, 9> magnetic_field_covariance() const noexcept {
        std::array<double, 9> cov{};
        ros_magnetic_field_get_magnetic_field_covariance(handle(), cov.data());
        return cov;
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - FluidPressure (view-only)
// ---------------------------------------------------------------------------

class FluidPressureView
    : public detail::ViewBase<FluidPressureView, detail::FluidPressureTraits> {
    using Base = detail::ViewBase<FluidPressureView, detail::FluidPressureTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    [[nodiscard]] Time stamp() const noexcept {
        return {ros_fluid_pressure_get_stamp_sec(handle()),
                ros_fluid_pressure_get_stamp_nanosec(handle())};
    }
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_fluid_pressure_get_frame_id(handle());
    }
    [[nodiscard]] double fluid_pressure() const noexcept {
        return ros_fluid_pressure_get_fluid_pressure(handle());
    }
    [[nodiscard]] double variance() const noexcept {
        return ros_fluid_pressure_get_variance(handle());
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - Temperature (view-only)
// ---------------------------------------------------------------------------

class TemperatureView
    : public detail::ViewBase<TemperatureView, detail::TemperatureTraits> {
    using Base = detail::ViewBase<TemperatureView, detail::TemperatureTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    [[nodiscard]] Time stamp() const noexcept {
        return {ros_temperature_get_stamp_sec(handle()),
                ros_temperature_get_stamp_nanosec(handle())};
    }
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_temperature_get_frame_id(handle());
    }
    [[nodiscard]] double temperature() const noexcept {
        return ros_temperature_get_temperature(handle());
    }
    [[nodiscard]] double variance() const noexcept {
        return ros_temperature_get_variance(handle());
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - BatteryState (view-only)
// ---------------------------------------------------------------------------

class BatteryStateView
    : public detail::ViewBase<BatteryStateView, detail::BatteryStateTraits> {
    using Base = detail::ViewBase<BatteryStateView, detail::BatteryStateTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    // power_supply_status
    static constexpr std::uint8_t POWER_SUPPLY_STATUS_UNKNOWN      = 0;
    static constexpr std::uint8_t POWER_SUPPLY_STATUS_CHARGING     = 1;
    static constexpr std::uint8_t POWER_SUPPLY_STATUS_DISCHARGING  = 2;
    static constexpr std::uint8_t POWER_SUPPLY_STATUS_NOT_CHARGING = 3;
    static constexpr std::uint8_t POWER_SUPPLY_STATUS_FULL         = 4;

    // power_supply_health
    static constexpr std::uint8_t POWER_SUPPLY_HEALTH_UNKNOWN               = 0;
    static constexpr std::uint8_t POWER_SUPPLY_HEALTH_GOOD                  = 1;
    static constexpr std::uint8_t POWER_SUPPLY_HEALTH_OVERHEAT              = 2;
    static constexpr std::uint8_t POWER_SUPPLY_HEALTH_DEAD                  = 3;
    static constexpr std::uint8_t POWER_SUPPLY_HEALTH_OVERVOLTAGE           = 4;
    static constexpr std::uint8_t POWER_SUPPLY_HEALTH_UNSPEC_FAILURE        = 5;
    static constexpr std::uint8_t POWER_SUPPLY_HEALTH_COLD                  = 6;
    static constexpr std::uint8_t POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7;
    static constexpr std::uint8_t POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE   = 8;

    // power_supply_technology
    static constexpr std::uint8_t POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0;
    static constexpr std::uint8_t POWER_SUPPLY_TECHNOLOGY_NIMH    = 1;
    static constexpr std::uint8_t POWER_SUPPLY_TECHNOLOGY_LION    = 2;
    static constexpr std::uint8_t POWER_SUPPLY_TECHNOLOGY_LIPO    = 3;
    static constexpr std::uint8_t POWER_SUPPLY_TECHNOLOGY_LIFE    = 4;
    static constexpr std::uint8_t POWER_SUPPLY_TECHNOLOGY_NICD    = 5;
    static constexpr std::uint8_t POWER_SUPPLY_TECHNOLOGY_LIMN    = 6;

    [[nodiscard]] Time stamp() const noexcept {
        return {ros_battery_state_get_stamp_sec(handle()),
                ros_battery_state_get_stamp_nanosec(handle())};
    }
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_battery_state_get_frame_id(handle());
    }
    [[nodiscard]] float voltage() const noexcept {
        return ros_battery_state_get_voltage(handle());
    }
    [[nodiscard]] float temperature() const noexcept {
        return ros_battery_state_get_temperature(handle());
    }
    [[nodiscard]] float current() const noexcept {
        return ros_battery_state_get_current(handle());
    }
    [[nodiscard]] float charge() const noexcept {
        return ros_battery_state_get_charge(handle());
    }
    [[nodiscard]] float capacity() const noexcept {
        return ros_battery_state_get_capacity(handle());
    }
    [[nodiscard]] float design_capacity() const noexcept {
        return ros_battery_state_get_design_capacity(handle());
    }
    [[nodiscard]] float percentage() const noexcept {
        return ros_battery_state_get_percentage(handle());
    }
    [[nodiscard]] std::uint8_t power_supply_status() const noexcept {
        return ros_battery_state_get_power_supply_status(handle());
    }
    [[nodiscard]] std::uint8_t power_supply_health() const noexcept {
        return ros_battery_state_get_power_supply_health(handle());
    }
    [[nodiscard]] std::uint8_t power_supply_technology() const noexcept {
        return ros_battery_state_get_power_supply_technology(handle());
    }
    [[nodiscard]] bool present() const noexcept {
        return ros_battery_state_get_present(handle());
    }
    [[nodiscard]] std::uint32_t cell_voltage_len() const noexcept {
        return ros_battery_state_get_cell_voltage_len(handle());
    }
    /// @brief Copy up to `out.size()` cell voltages; returns total count.
    std::uint32_t cell_voltage(span<float> out) const noexcept {
        return ros_battery_state_get_cell_voltage(handle(), out.data(), out.size());
    }
    [[nodiscard]] std::uint32_t cell_temperature_len() const noexcept {
        return ros_battery_state_get_cell_temperature_len(handle());
    }
    std::uint32_t cell_temperature(span<float> out) const noexcept {
        return ros_battery_state_get_cell_temperature(handle(), out.data(), out.size());
    }
    [[nodiscard]] std::string_view location() const noexcept {
        return ros_battery_state_get_location(handle());
    }
    [[nodiscard]] std::string_view serial_number() const noexcept {
        return ros_battery_state_get_serial_number(handle());
    }
};

// ---------------------------------------------------------------------------
// nav_msgs - Odometry (view-only)
// ---------------------------------------------------------------------------

class OdometryView : public detail::ViewBase<OdometryView, detail::OdometryTraits> {
    using Base = detail::ViewBase<OdometryView, detail::OdometryTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    [[nodiscard]] Time stamp() const noexcept {
        return {ros_odometry_get_stamp_sec(handle()),
                ros_odometry_get_stamp_nanosec(handle())};
    }
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_odometry_get_frame_id(handle());
    }
    [[nodiscard]] std::string_view child_frame_id() const noexcept {
        return ros_odometry_get_child_frame_id(handle());
    }
    [[nodiscard]] PoseWithCovariance pose() const noexcept {
        PoseWithCovariance p;
        ros_odometry_get_pose(handle(),
                              &p.pose.px, &p.pose.py, &p.pose.pz,
                              &p.pose.ox, &p.pose.oy, &p.pose.oz, &p.pose.ow);
        ros_odometry_get_pose_covariance(handle(), p.covariance.data());
        return p;
    }
    [[nodiscard]] TwistWithCovariance twist() const noexcept {
        TwistWithCovariance t;
        ros_odometry_get_twist(handle(),
                               &t.twist.lx, &t.twist.ly, &t.twist.lz,
                               &t.twist.ax, &t.twist.ay, &t.twist.az);
        ros_odometry_get_twist_covariance(handle(), t.covariance.data());
        return t;
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - Vibration (view-only)
// ---------------------------------------------------------------------------

class VibrationView : public detail::ViewBase<VibrationView, detail::VibrationTraits> {
    using Base = detail::ViewBase<VibrationView, detail::VibrationTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    // measurement_type
    static constexpr std::uint8_t MEASUREMENT_UNKNOWN      = 0;
    static constexpr std::uint8_t MEASUREMENT_RMS          = 1;
    static constexpr std::uint8_t MEASUREMENT_PEAK         = 2;
    static constexpr std::uint8_t MEASUREMENT_PEAK_TO_PEAK = 3;

    // unit
    static constexpr std::uint8_t UNIT_UNKNOWN           = 0;
    static constexpr std::uint8_t UNIT_ACCEL_M_PER_S2    = 1;
    static constexpr std::uint8_t UNIT_ACCEL_G           = 2;
    static constexpr std::uint8_t UNIT_VELOCITY_MM_PER_S = 3;
    static constexpr std::uint8_t UNIT_DISPLACEMENT_UM   = 4;
    static constexpr std::uint8_t UNIT_VELOCITY_IN_PER_S = 5;
    static constexpr std::uint8_t UNIT_DISPLACEMENT_MIL  = 6;

    [[nodiscard]] Time stamp() const noexcept {
        return {ros_vibration_get_stamp_sec(handle()),
                ros_vibration_get_stamp_nanosec(handle())};
    }
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_vibration_get_frame_id(handle());
    }
    [[nodiscard]] std::uint8_t measurement_type() const noexcept {
        return ros_vibration_get_measurement_type(handle());
    }
    [[nodiscard]] std::uint8_t unit() const noexcept {
        return ros_vibration_get_unit(handle());
    }
    [[nodiscard]] float band_lower_hz() const noexcept {
        return ros_vibration_get_band_lower_hz(handle());
    }
    [[nodiscard]] float band_upper_hz() const noexcept {
        return ros_vibration_get_band_upper_hz(handle());
    }
    [[nodiscard]] Vector3 vibration() const noexcept {
        Vector3 v;
        ros_vibration_get_vibration(handle(), &v.x, &v.y, &v.z);
        return v;
    }
    [[nodiscard]] std::uint32_t clipping_len() const noexcept {
        return ros_vibration_get_clipping_len(handle());
    }
    /// @brief Copy up to `out.size()` clipping counters; returns total count.
    std::uint32_t clipping(span<std::uint32_t> out) const noexcept {
        return ros_vibration_get_clipping(handle(), out.data(), out.size());
    }
};

// ============================================================================
// Builder types — fluent RAII wrappers around the C builder API
//
// Each builder wraps an opaque C builder handle with RAII lifecycle, fluent
// setter chaining, and two build paths:
//   - build()       → expected<Released, Error>   (allocating)
//   - encode_into() → expected<std::size_t, Error> (caller buffer)
//
// Builders are move-only; create via the static `create()` factory.
//
// String arguments (const char*) must be valid null-terminated UTF-8.
// This is the same contract as the existing encode() factories.
// ============================================================================

// ---------------------------------------------------------------------------
// std_msgs - HeaderBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `std_msgs::Header` messages.
///
/// @code{.cpp}
/// auto b = ef::HeaderBuilder::create();
/// if (!b) { /* handle error */ }
/// b->stamp({10, 0}).frame_id("camera");
/// auto r = b->build();
/// @endcode
class HeaderBuilder
    : public detail::BuilderBase<HeaderBuilder, detail::HeaderBuilderTraits> {
    using Base = detail::BuilderBase<HeaderBuilder, detail::HeaderBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    /// @brief Set the stamp field.
    HeaderBuilder& stamp(Time t) noexcept {
        ros_header_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    /// @brief Set the frame_id field (string copied into builder).
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_header_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_header_builder_set_frame_id"));
        return {};
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - ImageBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `sensor_msgs::Image` messages.
class ImageBuilder
    : public detail::BuilderBase<ImageBuilder, detail::ImageBuilderTraits> {
    using Base = detail::BuilderBase<ImageBuilder, detail::ImageBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    ImageBuilder& stamp(Time t) noexcept {
        ros_image_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_image_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_image_builder_set_frame_id"));
        return {};
    }
    ImageBuilder& height(std::uint32_t v) noexcept {
        ros_image_builder_set_height(ptr(), v); return *this;
    }
    ImageBuilder& width(std::uint32_t v) noexcept {
        ros_image_builder_set_width(ptr(), v); return *this;
    }
    [[nodiscard]] expected<void, Error> encoding(const char* s) noexcept {
        if (ros_image_builder_set_encoding(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_image_builder_set_encoding"));
        return {};
    }
    ImageBuilder& is_bigendian(std::uint8_t v) noexcept {
        ros_image_builder_set_is_bigendian(ptr(), v); return *this;
    }
    ImageBuilder& step(std::uint32_t v) noexcept {
        ros_image_builder_set_step(ptr(), v); return *this;
    }
    /// @brief Set the pixel data (BORROWED — must remain valid until
    ///        next setter, build, encode_into, or destruction).
    [[nodiscard]] expected<void, Error>
    data(span<const std::uint8_t> d) noexcept {
        if (ros_image_builder_set_data(ptr(), d.data(), d.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_image_builder_set_data"));
        return {};
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - CompressedImageBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `sensor_msgs::CompressedImage` messages.
class CompressedImageBuilder
    : public detail::BuilderBase<CompressedImageBuilder,
                                 detail::CompressedImageBuilderTraits> {
    using Base = detail::BuilderBase<CompressedImageBuilder,
                                     detail::CompressedImageBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    CompressedImageBuilder& stamp(Time t) noexcept {
        ros_compressed_image_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_compressed_image_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_compressed_image_builder_set_frame_id"));
        return {};
    }
    [[nodiscard]] expected<void, Error> format(const char* s) noexcept {
        if (ros_compressed_image_builder_set_format(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_compressed_image_builder_set_format"));
        return {};
    }
    [[nodiscard]] expected<void, Error>
    data(span<const std::uint8_t> d) noexcept {
        if (ros_compressed_image_builder_set_data(ptr(), d.data(), d.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_compressed_image_builder_set_data"));
        return {};
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - ImuBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `sensor_msgs::Imu` messages.
class ImuBuilder
    : public detail::BuilderBase<ImuBuilder, detail::ImuBuilderTraits> {
    using Base = detail::BuilderBase<ImuBuilder, detail::ImuBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    ImuBuilder& stamp(Time t) noexcept {
        ros_imu_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_imu_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_imu_builder_set_frame_id"));
        return {};
    }
    ImuBuilder& orientation(Quaternion q) noexcept {
        ros_imu_builder_set_orientation(ptr(), q.x, q.y, q.z, q.w);
        return *this;
    }
    [[nodiscard]] expected<void, Error>
    orientation_covariance(const std::array<double, 9>& cov) noexcept {
        if (ros_imu_builder_set_orientation_covariance(ptr(), cov.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_imu_builder_set_orientation_covariance"));
        return {};
    }
    ImuBuilder& angular_velocity(Vector3 v) noexcept {
        ros_imu_builder_set_angular_velocity(ptr(), v.x, v.y, v.z);
        return *this;
    }
    [[nodiscard]] expected<void, Error>
    angular_velocity_covariance(const std::array<double, 9>& cov) noexcept {
        if (ros_imu_builder_set_angular_velocity_covariance(ptr(), cov.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_imu_builder_set_angular_velocity_covariance"));
        return {};
    }
    ImuBuilder& linear_acceleration(Vector3 v) noexcept {
        ros_imu_builder_set_linear_acceleration(ptr(), v.x, v.y, v.z);
        return *this;
    }
    [[nodiscard]] expected<void, Error>
    linear_acceleration_covariance(const std::array<double, 9>& cov) noexcept {
        if (ros_imu_builder_set_linear_acceleration_covariance(ptr(), cov.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_imu_builder_set_linear_acceleration_covariance"));
        return {};
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - NavSatFixBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `sensor_msgs::NavSatFix` messages.
class NavSatFixBuilder
    : public detail::BuilderBase<NavSatFixBuilder,
                                 detail::NavSatFixBuilderTraits> {
    using Base = detail::BuilderBase<NavSatFixBuilder,
                                     detail::NavSatFixBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    NavSatFixBuilder& stamp(Time t) noexcept {
        ros_nav_sat_fix_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_nav_sat_fix_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_nav_sat_fix_builder_set_frame_id"));
        return {};
    }
    NavSatFixBuilder& status(std::int8_t st, std::uint16_t svc) noexcept {
        ros_nav_sat_fix_builder_set_status(ptr(), st, svc);
        return *this;
    }
    NavSatFixBuilder& latitude(double v) noexcept {
        ros_nav_sat_fix_builder_set_latitude(ptr(), v); return *this;
    }
    NavSatFixBuilder& longitude(double v) noexcept {
        ros_nav_sat_fix_builder_set_longitude(ptr(), v); return *this;
    }
    NavSatFixBuilder& altitude(double v) noexcept {
        ros_nav_sat_fix_builder_set_altitude(ptr(), v); return *this;
    }
    [[nodiscard]] expected<void, Error>
    position_covariance(const std::array<double, 9>& cov) noexcept {
        if (ros_nav_sat_fix_builder_set_position_covariance(ptr(), cov.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_nav_sat_fix_builder_set_position_covariance"));
        return {};
    }
    NavSatFixBuilder& position_covariance_type(std::uint8_t v) noexcept {
        ros_nav_sat_fix_builder_set_position_covariance_type(ptr(), v);
        return *this;
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - PointFieldBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `sensor_msgs::PointField` messages.
class PointFieldBuilder
    : public detail::BuilderBase<PointFieldBuilder,
                                 detail::PointFieldBuilderTraits> {
    using Base = detail::BuilderBase<PointFieldBuilder,
                                     detail::PointFieldBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    [[nodiscard]] expected<void, Error> name(const char* s) noexcept {
        if (ros_point_field_builder_set_name(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_point_field_builder_set_name"));
        return {};
    }
    PointFieldBuilder& offset(std::uint32_t v) noexcept {
        ros_point_field_builder_set_offset(ptr(), v); return *this;
    }
    PointFieldBuilder& datatype(std::uint8_t v) noexcept {
        ros_point_field_builder_set_datatype(ptr(), v); return *this;
    }
    PointFieldBuilder& count(std::uint32_t v) noexcept {
        ros_point_field_builder_set_count(ptr(), v); return *this;
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - PointCloud2Builder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `sensor_msgs::PointCloud2` messages.
class PointCloud2Builder
    : public detail::BuilderBase<PointCloud2Builder,
                                 detail::PointCloud2BuilderTraits> {
    using Base = detail::BuilderBase<PointCloud2Builder,
                                     detail::PointCloud2BuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    PointCloud2Builder& stamp(Time t) noexcept {
        ros_point_cloud2_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_point_cloud2_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_point_cloud2_builder_set_frame_id"));
        return {};
    }
    PointCloud2Builder& height(std::uint32_t v) noexcept {
        ros_point_cloud2_builder_set_height(ptr(), v); return *this;
    }
    PointCloud2Builder& width(std::uint32_t v) noexcept {
        ros_point_cloud2_builder_set_width(ptr(), v); return *this;
    }
    /// @brief Set the field descriptors (BORROWED — array and each
    ///        element's `name` must remain valid until next setter/build/free).
    [[nodiscard]] expected<void, Error>
    fields(span<const ros_point_field_elem_t> f) noexcept {
        if (ros_point_cloud2_builder_set_fields(ptr(), f.data(), f.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_point_cloud2_builder_set_fields"));
        return {};
    }
    PointCloud2Builder& is_bigendian(bool v) noexcept {
        ros_point_cloud2_builder_set_is_bigendian(ptr(), v); return *this;
    }
    PointCloud2Builder& point_step(std::uint32_t v) noexcept {
        ros_point_cloud2_builder_set_point_step(ptr(), v); return *this;
    }
    PointCloud2Builder& row_step(std::uint32_t v) noexcept {
        ros_point_cloud2_builder_set_row_step(ptr(), v); return *this;
    }
    /// @brief Set the data bulk (BORROWED).
    [[nodiscard]] expected<void, Error>
    data(span<const std::uint8_t> d) noexcept {
        if (ros_point_cloud2_builder_set_data(ptr(), d.data(), d.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_point_cloud2_builder_set_data"));
        return {};
    }
    PointCloud2Builder& is_dense(bool v) noexcept {
        ros_point_cloud2_builder_set_is_dense(ptr(), v); return *this;
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - CameraInfoBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `sensor_msgs::CameraInfo` messages.
class CameraInfoBuilder
    : public detail::BuilderBase<CameraInfoBuilder,
                                 detail::CameraInfoBuilderTraits> {
    using Base = detail::BuilderBase<CameraInfoBuilder,
                                     detail::CameraInfoBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    CameraInfoBuilder& stamp(Time t) noexcept {
        ros_camera_info_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_camera_info_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_camera_info_builder_set_frame_id"));
        return {};
    }
    CameraInfoBuilder& height(std::uint32_t v) noexcept {
        ros_camera_info_builder_set_height(ptr(), v); return *this;
    }
    CameraInfoBuilder& width(std::uint32_t v) noexcept {
        ros_camera_info_builder_set_width(ptr(), v); return *this;
    }
    [[nodiscard]] expected<void, Error>
    distortion_model(const char* s) noexcept {
        if (ros_camera_info_builder_set_distortion_model(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_camera_info_builder_set_distortion_model"));
        return {};
    }
    [[nodiscard]] expected<void, Error>
    d(span<const double> v) noexcept {
        if (ros_camera_info_builder_set_d(ptr(), v.data(), v.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_camera_info_builder_set_d"));
        return {};
    }
    [[nodiscard]] expected<void, Error>
    k(const std::array<double, 9>& v) noexcept {
        if (ros_camera_info_builder_set_k(ptr(), v.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_camera_info_builder_set_k"));
        return {};
    }
    [[nodiscard]] expected<void, Error>
    r(const std::array<double, 9>& v) noexcept {
        if (ros_camera_info_builder_set_r(ptr(), v.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_camera_info_builder_set_r"));
        return {};
    }
    [[nodiscard]] expected<void, Error>
    p(const std::array<double, 12>& v) noexcept {
        if (ros_camera_info_builder_set_p(ptr(), v.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_camera_info_builder_set_p"));
        return {};
    }
    CameraInfoBuilder& binning_x(std::uint32_t v) noexcept {
        ros_camera_info_builder_set_binning_x(ptr(), v); return *this;
    }
    CameraInfoBuilder& binning_y(std::uint32_t v) noexcept {
        ros_camera_info_builder_set_binning_y(ptr(), v); return *this;
    }
    /// @brief Set the RegionOfInterest.
    CameraInfoBuilder& roi(std::uint32_t x_offset, std::uint32_t y_offset,
                           std::uint32_t h, std::uint32_t w,
                           bool do_rectify) noexcept {
        ros_camera_info_builder_set_roi(ptr(), x_offset, y_offset, h, w,
                                        static_cast<std::uint8_t>(do_rectify));
        return *this;
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - MagneticFieldBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `sensor_msgs::MagneticField` messages.
class MagneticFieldBuilder
    : public detail::BuilderBase<MagneticFieldBuilder,
                                 detail::MagneticFieldBuilderTraits> {
    using Base = detail::BuilderBase<MagneticFieldBuilder,
                                     detail::MagneticFieldBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    MagneticFieldBuilder& stamp(Time t) noexcept {
        ros_magnetic_field_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_magnetic_field_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_magnetic_field_builder_set_frame_id"));
        return {};
    }
    MagneticFieldBuilder& magnetic_field(Vector3 v) noexcept {
        ros_magnetic_field_builder_set_magnetic_field(ptr(), v.x, v.y, v.z);
        return *this;
    }
    [[nodiscard]] expected<void, Error>
    magnetic_field_covariance(const std::array<double, 9>& cov) noexcept {
        if (ros_magnetic_field_builder_set_magnetic_field_covariance(ptr(), cov.data()) != 0)
            return unexpected<Error>(Error::from_errno("ros_magnetic_field_builder_set_magnetic_field_covariance"));
        return {};
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - FluidPressureBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `sensor_msgs::FluidPressure` messages.
class FluidPressureBuilder
    : public detail::BuilderBase<FluidPressureBuilder,
                                 detail::FluidPressureBuilderTraits> {
    using Base = detail::BuilderBase<FluidPressureBuilder,
                                     detail::FluidPressureBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    FluidPressureBuilder& stamp(Time t) noexcept {
        ros_fluid_pressure_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_fluid_pressure_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_fluid_pressure_builder_set_frame_id"));
        return {};
    }
    FluidPressureBuilder& fluid_pressure(double v) noexcept {
        ros_fluid_pressure_builder_set_fluid_pressure(ptr(), v); return *this;
    }
    FluidPressureBuilder& variance(double v) noexcept {
        ros_fluid_pressure_builder_set_variance(ptr(), v); return *this;
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - TemperatureBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `sensor_msgs::Temperature` messages.
class TemperatureBuilder
    : public detail::BuilderBase<TemperatureBuilder,
                                 detail::TemperatureBuilderTraits> {
    using Base = detail::BuilderBase<TemperatureBuilder,
                                     detail::TemperatureBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    TemperatureBuilder& stamp(Time t) noexcept {
        ros_temperature_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_temperature_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_temperature_builder_set_frame_id"));
        return {};
    }
    TemperatureBuilder& temperature(double v) noexcept {
        ros_temperature_builder_set_temperature(ptr(), v); return *this;
    }
    TemperatureBuilder& variance(double v) noexcept {
        ros_temperature_builder_set_variance(ptr(), v); return *this;
    }
};

// ---------------------------------------------------------------------------
// sensor_msgs - BatteryStateBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `sensor_msgs::BatteryState` messages.
class BatteryStateBuilder
    : public detail::BuilderBase<BatteryStateBuilder,
                                 detail::BatteryStateBuilderTraits> {
    using Base = detail::BuilderBase<BatteryStateBuilder,
                                     detail::BatteryStateBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    BatteryStateBuilder& stamp(Time t) noexcept {
        ros_battery_state_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_battery_state_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_battery_state_builder_set_frame_id"));
        return {};
    }
    BatteryStateBuilder& voltage(float v) noexcept {
        ros_battery_state_builder_set_voltage(ptr(), v); return *this;
    }
    BatteryStateBuilder& temperature(float v) noexcept {
        ros_battery_state_builder_set_temperature(ptr(), v); return *this;
    }
    BatteryStateBuilder& current(float v) noexcept {
        ros_battery_state_builder_set_current(ptr(), v); return *this;
    }
    BatteryStateBuilder& charge(float v) noexcept {
        ros_battery_state_builder_set_charge(ptr(), v); return *this;
    }
    BatteryStateBuilder& capacity(float v) noexcept {
        ros_battery_state_builder_set_capacity(ptr(), v); return *this;
    }
    BatteryStateBuilder& design_capacity(float v) noexcept {
        ros_battery_state_builder_set_design_capacity(ptr(), v); return *this;
    }
    BatteryStateBuilder& percentage(float v) noexcept {
        ros_battery_state_builder_set_percentage(ptr(), v); return *this;
    }
    BatteryStateBuilder& power_supply_status(std::uint8_t v) noexcept {
        ros_battery_state_builder_set_power_supply_status(ptr(), v); return *this;
    }
    BatteryStateBuilder& power_supply_health(std::uint8_t v) noexcept {
        ros_battery_state_builder_set_power_supply_health(ptr(), v); return *this;
    }
    BatteryStateBuilder& power_supply_technology(std::uint8_t v) noexcept {
        ros_battery_state_builder_set_power_supply_technology(ptr(), v); return *this;
    }
    BatteryStateBuilder& present(bool v) noexcept {
        ros_battery_state_builder_set_present(ptr(), v); return *this;
    }
    [[nodiscard]] expected<void, Error>
    cell_voltage(span<const float> v) noexcept {
        if (ros_battery_state_builder_set_cell_voltage(ptr(), v.data(), v.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_battery_state_builder_set_cell_voltage"));
        return {};
    }
    [[nodiscard]] expected<void, Error>
    cell_temperature(span<const float> v) noexcept {
        if (ros_battery_state_builder_set_cell_temperature(ptr(), v.data(), v.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_battery_state_builder_set_cell_temperature"));
        return {};
    }
    [[nodiscard]] expected<void, Error> location(const char* s) noexcept {
        if (ros_battery_state_builder_set_location(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_battery_state_builder_set_location"));
        return {};
    }
    [[nodiscard]] expected<void, Error> serial_number(const char* s) noexcept {
        if (ros_battery_state_builder_set_serial_number(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_battery_state_builder_set_serial_number"));
        return {};
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - MaskBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `edgefirst_msgs::Mask` messages.
class MaskBuilder
    : public detail::BuilderBase<MaskBuilder, detail::MaskBuilderTraits> {
    using Base = detail::BuilderBase<MaskBuilder, detail::MaskBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    MaskBuilder& height(std::uint32_t v) noexcept {
        ros_mask_builder_set_height(ptr(), v); return *this;
    }
    MaskBuilder& width(std::uint32_t v) noexcept {
        ros_mask_builder_set_width(ptr(), v); return *this;
    }
    MaskBuilder& length(std::uint32_t v) noexcept {
        ros_mask_builder_set_length(ptr(), v); return *this;
    }
    [[nodiscard]] expected<void, Error> encoding(const char* s) noexcept {
        if (ros_mask_builder_set_encoding(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_mask_builder_set_encoding"));
        return {};
    }
    /// @brief Set mask payload (BORROWED).
    [[nodiscard]] expected<void, Error>
    mask(span<const std::uint8_t> d) noexcept {
        if (ros_mask_builder_set_mask(ptr(), d.data(), d.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_mask_builder_set_mask"));
        return {};
    }
    MaskBuilder& boxed(bool v) noexcept {
        ros_mask_builder_set_boxed(ptr(), v); return *this;
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - LocalTimeBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `edgefirst_msgs::LocalTime` messages.
class LocalTimeBuilder
    : public detail::BuilderBase<LocalTimeBuilder,
                                 detail::LocalTimeBuilderTraits> {
    using Base = detail::BuilderBase<LocalTimeBuilder,
                                     detail::LocalTimeBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    LocalTimeBuilder& stamp(Time t) noexcept {
        ros_local_time_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_local_time_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_local_time_builder_set_frame_id"));
        return {};
    }
    LocalTimeBuilder& date(std::uint16_t year, std::uint8_t month,
                           std::uint8_t day) noexcept {
        ros_local_time_builder_set_date(ptr(), year, month, day);
        return *this;
    }
    LocalTimeBuilder& time(std::int32_t sec, std::uint32_t nsec) noexcept {
        ros_local_time_builder_set_time(ptr(), sec, nsec);
        return *this;
    }
    LocalTimeBuilder& timezone(std::int16_t v) noexcept {
        ros_local_time_builder_set_timezone(ptr(), v); return *this;
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - RadarCubeBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `edgefirst_msgs::RadarCube` messages.
class RadarCubeBuilder
    : public detail::BuilderBase<RadarCubeBuilder,
                                 detail::RadarCubeBuilderTraits> {
    using Base = detail::BuilderBase<RadarCubeBuilder,
                                     detail::RadarCubeBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    RadarCubeBuilder& stamp(Time t) noexcept {
        ros_radar_cube_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_radar_cube_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_radar_cube_builder_set_frame_id"));
        return {};
    }
    RadarCubeBuilder& timestamp(std::uint64_t v) noexcept {
        ros_radar_cube_builder_set_timestamp(ptr(), v); return *this;
    }
    /// @brief Set layout descriptor (BORROWED).
    [[nodiscard]] expected<void, Error>
    layout(span<const std::uint8_t> d) noexcept {
        if (ros_radar_cube_builder_set_layout(ptr(), d.data(), d.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_radar_cube_builder_set_layout"));
        return {};
    }
    /// @brief Set shape array (BORROWED).
    [[nodiscard]] expected<void, Error>
    shape(span<const std::uint16_t> d) noexcept {
        if (ros_radar_cube_builder_set_shape(ptr(), d.data(), d.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_radar_cube_builder_set_shape"));
        return {};
    }
    /// @brief Set scales array (BORROWED).
    [[nodiscard]] expected<void, Error>
    scales(span<const float> d) noexcept {
        if (ros_radar_cube_builder_set_scales(ptr(), d.data(), d.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_radar_cube_builder_set_scales"));
        return {};
    }
    /// @brief Set cube samples (BORROWED).
    [[nodiscard]] expected<void, Error>
    cube(span<const std::int16_t> d) noexcept {
        if (ros_radar_cube_builder_set_cube(ptr(), d.data(), d.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_radar_cube_builder_set_cube"));
        return {};
    }
    RadarCubeBuilder& is_complex(bool v) noexcept {
        ros_radar_cube_builder_set_is_complex(ptr(), v); return *this;
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - RadarInfoBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `edgefirst_msgs::RadarInfo` messages.
class RadarInfoBuilder
    : public detail::BuilderBase<RadarInfoBuilder,
                                 detail::RadarInfoBuilderTraits> {
    using Base = detail::BuilderBase<RadarInfoBuilder,
                                     detail::RadarInfoBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    RadarInfoBuilder& stamp(Time t) noexcept {
        ros_radar_info_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_radar_info_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_radar_info_builder_set_frame_id"));
        return {};
    }
    [[nodiscard]] expected<void, Error>
    center_frequency(const char* s) noexcept {
        if (ros_radar_info_builder_set_center_frequency(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_radar_info_builder_set_center_frequency"));
        return {};
    }
    [[nodiscard]] expected<void, Error>
    frequency_sweep(const char* s) noexcept {
        if (ros_radar_info_builder_set_frequency_sweep(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_radar_info_builder_set_frequency_sweep"));
        return {};
    }
    [[nodiscard]] expected<void, Error>
    range_toggle(const char* s) noexcept {
        if (ros_radar_info_builder_set_range_toggle(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_radar_info_builder_set_range_toggle"));
        return {};
    }
    [[nodiscard]] expected<void, Error>
    detection_sensitivity(const char* s) noexcept {
        if (ros_radar_info_builder_set_detection_sensitivity(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_radar_info_builder_set_detection_sensitivity"));
        return {};
    }
    RadarInfoBuilder& cube(bool v) noexcept {
        ros_radar_info_builder_set_cube(ptr(), v); return *this;
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - TrackBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `edgefirst_msgs::Track` messages.
class TrackBuilder
    : public detail::BuilderBase<TrackBuilder, detail::TrackBuilderTraits> {
    using Base = detail::BuilderBase<TrackBuilder, detail::TrackBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    [[nodiscard]] expected<void, Error> id(const char* s) noexcept {
        if (ros_track_builder_set_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_track_builder_set_id"));
        return {};
    }
    TrackBuilder& lifetime(std::int32_t v) noexcept {
        ros_track_builder_set_lifetime(ptr(), v); return *this;
    }
    TrackBuilder& created(Time t) noexcept {
        ros_track_builder_set_created(ptr(), t.sec, t.nanosec);
        return *this;
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - DetectBoxBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for a standalone `edgefirst_msgs::DetectBox`.
class DetectBoxBuilder
    : public detail::BuilderBase<DetectBoxBuilder,
                                 detail::DetectBoxBuilderTraits> {
    using Base = detail::BuilderBase<DetectBoxBuilder,
                                     detail::DetectBoxBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    DetectBoxBuilder& center_x(float v) noexcept {
        ros_detect_box_builder_set_center_x(ptr(), v); return *this;
    }
    DetectBoxBuilder& center_y(float v) noexcept {
        ros_detect_box_builder_set_center_y(ptr(), v); return *this;
    }
    DetectBoxBuilder& width(float v) noexcept {
        ros_detect_box_builder_set_width(ptr(), v); return *this;
    }
    DetectBoxBuilder& height(float v) noexcept {
        ros_detect_box_builder_set_height(ptr(), v); return *this;
    }
    [[nodiscard]] expected<void, Error> label(const char* s) noexcept {
        if (ros_detect_box_builder_set_label(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_detect_box_builder_set_label"));
        return {};
    }
    DetectBoxBuilder& score(float v) noexcept {
        ros_detect_box_builder_set_score(ptr(), v); return *this;
    }
    DetectBoxBuilder& distance(float v) noexcept {
        ros_detect_box_builder_set_distance(ptr(), v); return *this;
    }
    DetectBoxBuilder& speed(float v) noexcept {
        ros_detect_box_builder_set_speed(ptr(), v); return *this;
    }
    [[nodiscard]] expected<void, Error> track_id(const char* s) noexcept {
        if (ros_detect_box_builder_set_track_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_detect_box_builder_set_track_id"));
        return {};
    }
    DetectBoxBuilder& track_lifetime(std::int32_t v) noexcept {
        ros_detect_box_builder_set_track_lifetime(ptr(), v); return *this;
    }
    DetectBoxBuilder& track_created(Time t) noexcept {
        ros_detect_box_builder_set_track_created(ptr(), t.sec, t.nanosec);
        return *this;
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - DetectBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `edgefirst_msgs::Detect` messages.
class DetectBuilder
    : public detail::BuilderBase<DetectBuilder, detail::DetectBuilderTraits> {
    using Base = detail::BuilderBase<DetectBuilder, detail::DetectBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    DetectBuilder& stamp(Time t) noexcept {
        ros_detect_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_detect_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_detect_builder_set_frame_id"));
        return {};
    }
    DetectBuilder& input_timestamp(Time t) noexcept {
        ros_detect_builder_set_input_timestamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    DetectBuilder& model_time(Time t) noexcept {
        ros_detect_builder_set_model_time(ptr(), t.sec, t.nanosec);
        return *this;
    }
    DetectBuilder& output_time(Time t) noexcept {
        ros_detect_builder_set_output_time(ptr(), t.sec, t.nanosec);
        return *this;
    }
    /// @brief Set the boxes array (BORROWED — each element's label and
    ///        track_id pointers must remain valid until next setter/build/free).
    [[nodiscard]] expected<void, Error>
    boxes(span<const ros_detect_box_elem_t> b) noexcept {
        if (ros_detect_builder_set_boxes(ptr(), b.data(), b.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_detect_builder_set_boxes"));
        return {};
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - CameraFrameBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `edgefirst_msgs::CameraFrame` messages.
class CameraFrameBuilder
    : public detail::BuilderBase<CameraFrameBuilder,
                                 detail::CameraFrameBuilderTraits> {
    using Base = detail::BuilderBase<CameraFrameBuilder,
                                     detail::CameraFrameBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    CameraFrameBuilder& stamp(Time t) noexcept {
        ros_camera_frame_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_camera_frame_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_camera_frame_builder_set_frame_id"));
        return {};
    }
    CameraFrameBuilder& seq(std::uint64_t v) noexcept {
        ros_camera_frame_builder_set_seq(ptr(), v); return *this;
    }
    CameraFrameBuilder& pid(std::uint32_t v) noexcept {
        ros_camera_frame_builder_set_pid(ptr(), v); return *this;
    }
    CameraFrameBuilder& width(std::uint32_t v) noexcept {
        ros_camera_frame_builder_set_width(ptr(), v); return *this;
    }
    CameraFrameBuilder& height(std::uint32_t v) noexcept {
        ros_camera_frame_builder_set_height(ptr(), v); return *this;
    }
    [[nodiscard]] expected<void, Error> format(const char* s) noexcept {
        if (ros_camera_frame_builder_set_format(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_camera_frame_builder_set_format"));
        return {};
    }
    [[nodiscard]] expected<void, Error> color_space(const char* s) noexcept {
        if (ros_camera_frame_builder_set_color_space(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_camera_frame_builder_set_color_space"));
        return {};
    }
    [[nodiscard]] expected<void, Error>
    color_transfer(const char* s) noexcept {
        if (ros_camera_frame_builder_set_color_transfer(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_camera_frame_builder_set_color_transfer"));
        return {};
    }
    [[nodiscard]] expected<void, Error>
    color_encoding(const char* s) noexcept {
        if (ros_camera_frame_builder_set_color_encoding(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_camera_frame_builder_set_color_encoding"));
        return {};
    }
    [[nodiscard]] expected<void, Error> color_range(const char* s) noexcept {
        if (ros_camera_frame_builder_set_color_range(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_camera_frame_builder_set_color_range"));
        return {};
    }
    CameraFrameBuilder& fence_fd(std::int32_t v) noexcept {
        ros_camera_frame_builder_set_fence_fd(ptr(), v); return *this;
    }
    /// @brief Set planes (BORROWED — each element's data must remain valid).
    [[nodiscard]] expected<void, Error>
    planes(span<const ros_camera_plane_elem_t> p) noexcept {
        if (ros_camera_frame_builder_set_planes(ptr(), p.data(), p.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_camera_frame_builder_set_planes"));
        return {};
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - ModelBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `edgefirst_msgs::Model` messages.
class ModelBuilder
    : public detail::BuilderBase<ModelBuilder, detail::ModelBuilderTraits> {
    using Base = detail::BuilderBase<ModelBuilder, detail::ModelBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    ModelBuilder& stamp(Time t) noexcept {
        ros_model_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_model_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_model_builder_set_frame_id"));
        return {};
    }
    ModelBuilder& input_time(Time t) noexcept {
        ros_model_builder_set_input_time(ptr(), t.sec, t.nanosec);
        return *this;
    }
    ModelBuilder& model_time(Time t) noexcept {
        ros_model_builder_set_model_time(ptr(), t.sec, t.nanosec);
        return *this;
    }
    ModelBuilder& output_time(Time t) noexcept {
        ros_model_builder_set_output_time(ptr(), t.sec, t.nanosec);
        return *this;
    }
    ModelBuilder& decode_time(Time t) noexcept {
        ros_model_builder_set_decode_time(ptr(), t.sec, t.nanosec);
        return *this;
    }
    /// @brief Set boxes (BORROWED — see DetectBuilder::boxes).
    [[nodiscard]] expected<void, Error>
    boxes(span<const ros_detect_box_elem_t> b) noexcept {
        if (ros_model_builder_set_boxes(ptr(), b.data(), b.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_model_builder_set_boxes"));
        return {};
    }
    /// @brief Set masks (BORROWED — each element's encoding/mask must
    ///        remain valid until next setter/build/free).
    [[nodiscard]] expected<void, Error>
    masks(span<const ros_mask_elem_t> m) noexcept {
        if (ros_model_builder_set_masks(ptr(), m.data(), m.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_model_builder_set_masks"));
        return {};
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - ModelInfoBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `edgefirst_msgs::ModelInfo` messages.
class ModelInfoBuilder
    : public detail::BuilderBase<ModelInfoBuilder,
                                 detail::ModelInfoBuilderTraits> {
    using Base = detail::BuilderBase<ModelInfoBuilder,
                                     detail::ModelInfoBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    ModelInfoBuilder& stamp(Time t) noexcept {
        ros_model_info_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_model_info_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_model_info_builder_set_frame_id"));
        return {};
    }
    /// @brief Set input shape (BORROWED).
    [[nodiscard]] expected<void, Error>
    input_shape(span<const std::uint32_t> d) noexcept {
        if (ros_model_info_builder_set_input_shape(ptr(), d.data(), d.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_model_info_builder_set_input_shape"));
        return {};
    }
    ModelInfoBuilder& input_type(std::uint8_t v) noexcept {
        ros_model_info_builder_set_input_type(ptr(), v); return *this;
    }
    /// @brief Set output shape (BORROWED).
    [[nodiscard]] expected<void, Error>
    output_shape(span<const std::uint32_t> d) noexcept {
        if (ros_model_info_builder_set_output_shape(ptr(), d.data(), d.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_model_info_builder_set_output_shape"));
        return {};
    }
    ModelInfoBuilder& output_type(std::uint8_t v) noexcept {
        ros_model_info_builder_set_output_type(ptr(), v); return *this;
    }
    /// @brief Set labels (strings are copied into the builder).
    [[nodiscard]] expected<void, Error>
    labels(const char* const* lbl, std::size_t count) noexcept {
        if (ros_model_info_builder_set_labels(ptr(), lbl, count) != 0)
            return unexpected<Error>(Error::from_errno("ros_model_info_builder_set_labels"));
        return {};
    }
    [[nodiscard]] expected<void, Error>
    model_type(const char* s) noexcept {
        if (ros_model_info_builder_set_model_type(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_model_info_builder_set_model_type"));
        return {};
    }
    [[nodiscard]] expected<void, Error>
    model_format(const char* s) noexcept {
        if (ros_model_info_builder_set_model_format(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_model_info_builder_set_model_format"));
        return {};
    }
    [[nodiscard]] expected<void, Error>
    model_name(const char* s) noexcept {
        if (ros_model_info_builder_set_model_name(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_model_info_builder_set_model_name"));
        return {};
    }
};

// ---------------------------------------------------------------------------
// edgefirst_msgs - VibrationBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `edgefirst_msgs::Vibration` messages.
class VibrationBuilder
    : public detail::BuilderBase<VibrationBuilder,
                                 detail::VibrationBuilderTraits> {
    using Base = detail::BuilderBase<VibrationBuilder,
                                     detail::VibrationBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    VibrationBuilder& stamp(Time t) noexcept {
        ros_vibration_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_vibration_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_vibration_builder_set_frame_id"));
        return {};
    }
    VibrationBuilder& vibration(Vector3 v) noexcept {
        ros_vibration_builder_set_vibration(ptr(), v.x, v.y, v.z);
        return *this;
    }
    VibrationBuilder& band_lower_hz(float v) noexcept {
        ros_vibration_builder_set_band_lower_hz(ptr(), v); return *this;
    }
    VibrationBuilder& band_upper_hz(float v) noexcept {
        ros_vibration_builder_set_band_upper_hz(ptr(), v); return *this;
    }
    VibrationBuilder& measurement_type(std::uint8_t v) noexcept {
        ros_vibration_builder_set_measurement_type(ptr(), v); return *this;
    }
    VibrationBuilder& unit(std::uint8_t v) noexcept {
        ros_vibration_builder_set_unit(ptr(), v); return *this;
    }
    [[nodiscard]] expected<void, Error>
    clipping(span<const std::uint32_t> d) noexcept {
        if (ros_vibration_builder_set_clipping(ptr(), d.data(), d.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_vibration_builder_set_clipping"));
        return {};
    }
};

// ---------------------------------------------------------------------------
// foxglove_msgs - FoxgloveCompressedVideoBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `foxglove_msgs::CompressedVideo` messages.
class FoxgloveCompressedVideoBuilder
    : public detail::BuilderBase<FoxgloveCompressedVideoBuilder,
                                 detail::FoxgloveCompressedVideoBuilderTraits> {
    using Base = detail::BuilderBase<FoxgloveCompressedVideoBuilder,
                                     detail::FoxgloveCompressedVideoBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    FoxgloveCompressedVideoBuilder& stamp(Time t) noexcept {
        ros_foxglove_compressed_video_builder_set_stamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    /// Alias for stamp(); matches the Foxglove schema field name.
    FoxgloveCompressedVideoBuilder& timestamp(Time t) noexcept { return stamp(t); }
    [[nodiscard]] expected<void, Error> frame_id(const char* s) noexcept {
        if (ros_foxglove_compressed_video_builder_set_frame_id(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_foxglove_compressed_video_builder_set_frame_id"));
        return {};
    }
    [[nodiscard]] expected<void, Error>
    data(span<const std::uint8_t> d) noexcept {
        if (ros_foxglove_compressed_video_builder_set_data(ptr(), d.data(), d.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_foxglove_compressed_video_builder_set_data"));
        return {};
    }
    [[nodiscard]] expected<void, Error> format(const char* s) noexcept {
        if (ros_foxglove_compressed_video_builder_set_format(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_foxglove_compressed_video_builder_set_format"));
        return {};
    }
};

// ---------------------------------------------------------------------------
// foxglove_msgs - FoxgloveTextAnnotationBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `foxglove_msgs::TextAnnotation` messages.
class FoxgloveTextAnnotationBuilder
    : public detail::BuilderBase<FoxgloveTextAnnotationBuilder,
                                 detail::FoxgloveTextAnnotationBuilderTraits> {
    using Base = detail::BuilderBase<FoxgloveTextAnnotationBuilder,
                                     detail::FoxgloveTextAnnotationBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    FoxgloveTextAnnotationBuilder& timestamp(Time t) noexcept {
        ros_foxglove_text_annotation_builder_set_timestamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    FoxgloveTextAnnotationBuilder& position(double x, double y) noexcept {
        ros_foxglove_text_annotation_builder_set_position(ptr(), x, y);
        return *this;
    }
    [[nodiscard]] expected<void, Error> text(const char* s) noexcept {
        if (ros_foxglove_text_annotation_builder_set_text(ptr(), s) != 0)
            return unexpected<Error>(Error::from_errno("ros_foxglove_text_annotation_builder_set_text"));
        return {};
    }
    FoxgloveTextAnnotationBuilder& font_size(double v) noexcept {
        ros_foxglove_text_annotation_builder_set_font_size(ptr(), v);
        return *this;
    }
    FoxgloveTextAnnotationBuilder& text_color(double r, double g,
                                              double b, double a) noexcept {
        ros_foxglove_text_annotation_builder_set_text_color(ptr(), r, g, b, a);
        return *this;
    }
    FoxgloveTextAnnotationBuilder& background_color(double r, double g,
                                                    double b, double a) noexcept {
        ros_foxglove_text_annotation_builder_set_background_color(ptr(), r, g, b, a);
        return *this;
    }
};

// ---------------------------------------------------------------------------
// foxglove_msgs - FoxglovePointAnnotationBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `foxglove_msgs::PointsAnnotation` messages.
class FoxglovePointAnnotationBuilder
    : public detail::BuilderBase<FoxglovePointAnnotationBuilder,
                                 detail::FoxglovePointAnnotationBuilderTraits> {
    using Base = detail::BuilderBase<FoxglovePointAnnotationBuilder,
                                     detail::FoxglovePointAnnotationBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    FoxglovePointAnnotationBuilder& timestamp(Time t) noexcept {
        ros_foxglove_point_annotation_builder_set_timestamp(ptr(), t.sec, t.nanosec);
        return *this;
    }
    FoxglovePointAnnotationBuilder& type(std::uint8_t v) noexcept {
        ros_foxglove_point_annotation_builder_set_type(ptr(), v);
        return *this;
    }
    /// @brief Set points array (BORROWED).
    [[nodiscard]] expected<void, Error>
    points(span<const ros_foxglove_point2_elem_t> pts) noexcept {
        if (ros_foxglove_point_annotation_builder_set_points(ptr(), pts.data(), pts.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_foxglove_point_annotation_builder_set_points"));
        return {};
    }
    FoxglovePointAnnotationBuilder& outline_color(double r, double g,
                                                  double b, double a) noexcept {
        ros_foxglove_point_annotation_builder_set_outline_color(ptr(), r, g, b, a);
        return *this;
    }
    /// @brief Set per-point outline colors (BORROWED).
    [[nodiscard]] expected<void, Error>
    outline_colors(span<const ros_foxglove_color_elem_t> colors) noexcept {
        if (ros_foxglove_point_annotation_builder_set_outline_colors(ptr(), colors.data(), colors.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_foxglove_point_annotation_builder_set_outline_colors"));
        return {};
    }
    FoxglovePointAnnotationBuilder& fill_color(double r, double g,
                                               double b, double a) noexcept {
        ros_foxglove_point_annotation_builder_set_fill_color(ptr(), r, g, b, a);
        return *this;
    }
    FoxglovePointAnnotationBuilder& thickness(double v) noexcept {
        ros_foxglove_point_annotation_builder_set_thickness(ptr(), v);
        return *this;
    }
};

// ---------------------------------------------------------------------------
// foxglove_msgs - FoxgloveImageAnnotationBuilder
// ---------------------------------------------------------------------------

/// @brief Fluent builder for `foxglove_msgs::ImageAnnotations` messages.
class FoxgloveImageAnnotationBuilder
    : public detail::BuilderBase<FoxgloveImageAnnotationBuilder,
                                 detail::FoxgloveImageAnnotationBuilderTraits> {
    using Base = detail::BuilderBase<FoxgloveImageAnnotationBuilder,
                                     detail::FoxgloveImageAnnotationBuilderTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::create;
    using Base::build;
    using Base::encode_into;

    /// @brief Set circle annotations (BORROWED).
    [[nodiscard]] expected<void, Error>
    circles(span<const ros_foxglove_circle_annotation_elem_t> c) noexcept {
        if (ros_foxglove_image_annotation_builder_set_circles(ptr(), c.data(), c.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_foxglove_image_annotation_builder_set_circles"));
        return {};
    }
    /// @brief Set point annotations (BORROWED — inner arrays also borrowed).
    [[nodiscard]] expected<void, Error>
    points(span<const ros_foxglove_point_annotation_elem_t> p) noexcept {
        if (ros_foxglove_image_annotation_builder_set_points(ptr(), p.data(), p.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_foxglove_image_annotation_builder_set_points"));
        return {};
    }
    /// @brief Set text annotations (BORROWED — each element's text must
    ///        remain valid).
    [[nodiscard]] expected<void, Error>
    texts(span<const ros_foxglove_text_annotation_elem_t> t) noexcept {
        if (ros_foxglove_image_annotation_builder_set_texts(ptr(), t.data(), t.size()) != 0)
            return unexpected<Error>(Error::from_errno("ros_foxglove_image_annotation_builder_set_texts"));
        return {};
    }
};

// =============================================================================
// mavros_msgs View classes
// =============================================================================

/// @brief Zero-copy view of a mavros_msgs/Altitude CDR buffer.
class MavrosAltitudeView : public detail::ViewBase<MavrosAltitudeView, detail::MavrosAltitudeTraits> {
    using Base = detail::ViewBase<MavrosAltitudeView, detail::MavrosAltitudeTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    [[nodiscard]] Time stamp() const noexcept {
        return {ros_mavros_altitude_get_stamp_sec(handle()),
                ros_mavros_altitude_get_stamp_nanosec(handle())};
    }
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_mavros_altitude_get_frame_id(handle());
    }
    [[nodiscard]] float monotonic() const noexcept {
        return ros_mavros_altitude_get_monotonic(handle());
    }
    [[nodiscard]] float amsl() const noexcept {
        return ros_mavros_altitude_get_amsl(handle());
    }
    [[nodiscard]] float local() const noexcept {
        return ros_mavros_altitude_get_local(handle());
    }
    [[nodiscard]] float relative() const noexcept {
        return ros_mavros_altitude_get_relative(handle());
    }
    [[nodiscard]] float terrain() const noexcept {
        return ros_mavros_altitude_get_terrain(handle());
    }
    [[nodiscard]] float bottom_clearance() const noexcept {
        return ros_mavros_altitude_get_bottom_clearance(handle());
    }
};

/// @brief Zero-copy view of a mavros_msgs/VfrHud CDR buffer.
class MavrosVfrHudView : public detail::ViewBase<MavrosVfrHudView, detail::MavrosVfrHudTraits> {
    using Base = detail::ViewBase<MavrosVfrHudView, detail::MavrosVfrHudTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    [[nodiscard]] Time stamp() const noexcept {
        return {ros_mavros_vfrhud_get_stamp_sec(handle()),
                ros_mavros_vfrhud_get_stamp_nanosec(handle())};
    }
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_mavros_vfrhud_get_frame_id(handle());
    }
    [[nodiscard]] float airspeed() const noexcept {
        return ros_mavros_vfrhud_get_airspeed(handle());
    }
    [[nodiscard]] float groundspeed() const noexcept {
        return ros_mavros_vfrhud_get_groundspeed(handle());
    }
    [[nodiscard]] std::int16_t heading() const noexcept {
        return ros_mavros_vfrhud_get_heading(handle());
    }
    [[nodiscard]] float throttle() const noexcept {
        return ros_mavros_vfrhud_get_throttle(handle());
    }
    [[nodiscard]] float altitude() const noexcept {
        return ros_mavros_vfrhud_get_altitude(handle());
    }
    [[nodiscard]] float climb() const noexcept {
        return ros_mavros_vfrhud_get_climb(handle());
    }
};

/// @brief Zero-copy view of a mavros_msgs/EstimatorStatus CDR buffer.
class MavrosEstimatorStatusView : public detail::ViewBase<MavrosEstimatorStatusView, detail::MavrosEstimatorStatusTraits> {
    using Base = detail::ViewBase<MavrosEstimatorStatusView, detail::MavrosEstimatorStatusTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    [[nodiscard]] Time stamp() const noexcept {
        return {ros_mavros_estimator_status_get_stamp_sec(handle()),
                ros_mavros_estimator_status_get_stamp_nanosec(handle())};
    }
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_mavros_estimator_status_get_frame_id(handle());
    }
    [[nodiscard]] bool attitude_status_flag() const noexcept {
        return ros_mavros_estimator_status_get_attitude_status_flag(handle());
    }
    [[nodiscard]] bool velocity_horiz_status_flag() const noexcept {
        return ros_mavros_estimator_status_get_velocity_horiz_status_flag(handle());
    }
    [[nodiscard]] bool velocity_vert_status_flag() const noexcept {
        return ros_mavros_estimator_status_get_velocity_vert_status_flag(handle());
    }
    [[nodiscard]] bool pos_horiz_rel_status_flag() const noexcept {
        return ros_mavros_estimator_status_get_pos_horiz_rel_status_flag(handle());
    }
    [[nodiscard]] bool pos_horiz_abs_status_flag() const noexcept {
        return ros_mavros_estimator_status_get_pos_horiz_abs_status_flag(handle());
    }
    [[nodiscard]] bool pos_vert_abs_status_flag() const noexcept {
        return ros_mavros_estimator_status_get_pos_vert_abs_status_flag(handle());
    }
    [[nodiscard]] bool pos_vert_agl_status_flag() const noexcept {
        return ros_mavros_estimator_status_get_pos_vert_agl_status_flag(handle());
    }
    [[nodiscard]] bool const_pos_mode_status_flag() const noexcept {
        return ros_mavros_estimator_status_get_const_pos_mode_status_flag(handle());
    }
    [[nodiscard]] bool pred_pos_horiz_rel_status_flag() const noexcept {
        return ros_mavros_estimator_status_get_pred_pos_horiz_rel_status_flag(handle());
    }
    [[nodiscard]] bool pred_pos_horiz_abs_status_flag() const noexcept {
        return ros_mavros_estimator_status_get_pred_pos_horiz_abs_status_flag(handle());
    }
    [[nodiscard]] bool gps_glitch_status_flag() const noexcept {
        return ros_mavros_estimator_status_get_gps_glitch_status_flag(handle());
    }
    [[nodiscard]] bool accel_error_status_flag() const noexcept {
        return ros_mavros_estimator_status_get_accel_error_status_flag(handle());
    }
};

/// @brief Zero-copy view of a mavros_msgs/ExtendedState CDR buffer.
class MavrosExtendedStateView : public detail::ViewBase<MavrosExtendedStateView, detail::MavrosExtendedStateTraits> {
    using Base = detail::ViewBase<MavrosExtendedStateView, detail::MavrosExtendedStateTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    static constexpr std::uint8_t VTOL_STATE_UNDEFINED        = 0;
    static constexpr std::uint8_t VTOL_STATE_TRANSITION_TO_FW = 1;
    static constexpr std::uint8_t VTOL_STATE_TRANSITION_TO_MC = 2;
    static constexpr std::uint8_t VTOL_STATE_MC               = 3;
    static constexpr std::uint8_t VTOL_STATE_FW               = 4;

    static constexpr std::uint8_t LANDED_STATE_UNDEFINED  = 0;
    static constexpr std::uint8_t LANDED_STATE_ON_GROUND  = 1;
    static constexpr std::uint8_t LANDED_STATE_IN_AIR     = 2;
    static constexpr std::uint8_t LANDED_STATE_TAKEOFF    = 3;
    static constexpr std::uint8_t LANDED_STATE_LANDING    = 4;

    [[nodiscard]] Time stamp() const noexcept {
        return {ros_mavros_extended_state_get_stamp_sec(handle()),
                ros_mavros_extended_state_get_stamp_nanosec(handle())};
    }
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_mavros_extended_state_get_frame_id(handle());
    }
    [[nodiscard]] std::uint8_t vtol_state() const noexcept {
        return ros_mavros_extended_state_get_vtol_state(handle());
    }
    [[nodiscard]] std::uint8_t landed_state() const noexcept {
        return ros_mavros_extended_state_get_landed_state(handle());
    }
};

/// @brief Zero-copy view of a mavros_msgs/SysStatus CDR buffer.
///
/// Field units follow the raw MAVLink SYS_STATUS message definition —
/// no conversions are applied.
/// @see https://mavlink.io/en/messages/common.html#SYS_STATUS
class MavrosSysStatusView : public detail::ViewBase<MavrosSysStatusView, detail::MavrosSysStatusTraits> {
    using Base = detail::ViewBase<MavrosSysStatusView, detail::MavrosSysStatusTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    [[nodiscard]] Time stamp() const noexcept {
        return {ros_mavros_sys_status_get_stamp_sec(handle()),
                ros_mavros_sys_status_get_stamp_nanosec(handle())};
    }
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_mavros_sys_status_get_frame_id(handle());
    }
    /// Bitmask of onboard sensors present.
    [[nodiscard]] std::uint32_t sensors_present() const noexcept {
        return ros_mavros_sys_status_get_sensors_present(handle());
    }
    /// Bitmask of onboard sensors enabled.
    [[nodiscard]] std::uint32_t sensors_enabled() const noexcept {
        return ros_mavros_sys_status_get_sensors_enabled(handle());
    }
    /// Bitmask of onboard sensors reporting healthy.
    [[nodiscard]] std::uint32_t sensors_health() const noexcept {
        return ros_mavros_sys_status_get_sensors_health(handle());
    }
    /// Maximum CPU/MCU usage in per-mille (0–1000).
    [[nodiscard]] std::uint16_t load() const noexcept {
        return ros_mavros_sys_status_get_load(handle());
    }
    /// Battery voltage in millivolts (mV). UINT16_MAX if unknown.
    [[nodiscard]] std::uint16_t voltage_battery() const noexcept {
        return ros_mavros_sys_status_get_voltage_battery(handle());
    }
    /// Battery current in centi-amperes (cA, 10 mA steps). -1 if unknown.
    [[nodiscard]] std::int16_t current_battery() const noexcept {
        return ros_mavros_sys_status_get_current_battery(handle());
    }
    /// Battery remaining, 0–100 (%). -1 if not estimated.
    [[nodiscard]] std::int8_t battery_remaining() const noexcept {
        return ros_mavros_sys_status_get_battery_remaining(handle());
    }
    /// Communication drop rate in per-mille.
    [[nodiscard]] std::uint16_t drop_rate_comm() const noexcept {
        return ros_mavros_sys_status_get_drop_rate_comm(handle());
    }
    /// Communication error count.
    [[nodiscard]] std::uint16_t errors_comm() const noexcept {
        return ros_mavros_sys_status_get_errors_comm(handle());
    }
    /// Autopilot-specific error count 1.
    [[nodiscard]] std::uint16_t errors_count1() const noexcept {
        return ros_mavros_sys_status_get_errors_count1(handle());
    }
    /// Autopilot-specific error count 2.
    [[nodiscard]] std::uint16_t errors_count2() const noexcept {
        return ros_mavros_sys_status_get_errors_count2(handle());
    }
    /// Autopilot-specific error count 3.
    [[nodiscard]] std::uint16_t errors_count3() const noexcept {
        return ros_mavros_sys_status_get_errors_count3(handle());
    }
    /// Autopilot-specific error count 4.
    [[nodiscard]] std::uint16_t errors_count4() const noexcept {
        return ros_mavros_sys_status_get_errors_count4(handle());
    }
};

/// @brief Zero-copy view of a mavros_msgs/State CDR buffer.
class MavrosStateView : public detail::ViewBase<MavrosStateView, detail::MavrosStateTraits> {
    using Base = detail::ViewBase<MavrosStateView, detail::MavrosStateTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    static constexpr std::uint8_t MAV_STATE_UNINIT             = 0;
    static constexpr std::uint8_t MAV_STATE_BOOT               = 1;
    static constexpr std::uint8_t MAV_STATE_CALIBRATING        = 2;
    static constexpr std::uint8_t MAV_STATE_STANDBY            = 3;
    static constexpr std::uint8_t MAV_STATE_ACTIVE             = 4;
    static constexpr std::uint8_t MAV_STATE_CRITICAL           = 5;
    static constexpr std::uint8_t MAV_STATE_EMERGENCY          = 6;
    static constexpr std::uint8_t MAV_STATE_POWEROFF           = 7;
    static constexpr std::uint8_t MAV_STATE_FLIGHT_TERMINATION = 8;

    [[nodiscard]] Time stamp() const noexcept {
        return {ros_mavros_state_get_stamp_sec(handle()),
                ros_mavros_state_get_stamp_nanosec(handle())};
    }
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_mavros_state_get_frame_id(handle());
    }
    [[nodiscard]] bool connected() const noexcept {
        return ros_mavros_state_get_connected(handle());
    }
    [[nodiscard]] bool armed() const noexcept {
        return ros_mavros_state_get_armed(handle());
    }
    [[nodiscard]] bool guided() const noexcept {
        return ros_mavros_state_get_guided(handle());
    }
    [[nodiscard]] bool manual_input() const noexcept {
        return ros_mavros_state_get_manual_input(handle());
    }
    [[nodiscard]] std::string_view mode() const noexcept {
        return ros_mavros_state_get_mode(handle());
    }
    [[nodiscard]] std::uint8_t system_status() const noexcept {
        return ros_mavros_state_get_system_status(handle());
    }
};

/// @brief Zero-copy view of a mavros_msgs/StatusText CDR buffer.
class MavrosStatusTextView : public detail::ViewBase<MavrosStatusTextView, detail::MavrosStatusTextTraits> {
    using Base = detail::ViewBase<MavrosStatusTextView, detail::MavrosStatusTextTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    static constexpr std::uint8_t SEVERITY_EMERGENCY = 0;
    static constexpr std::uint8_t SEVERITY_ALERT     = 1;
    static constexpr std::uint8_t SEVERITY_CRITICAL  = 2;
    static constexpr std::uint8_t SEVERITY_ERROR     = 3;
    static constexpr std::uint8_t SEVERITY_WARNING   = 4;
    static constexpr std::uint8_t SEVERITY_NOTICE    = 5;
    static constexpr std::uint8_t SEVERITY_INFO      = 6;
    static constexpr std::uint8_t SEVERITY_DEBUG     = 7;

    [[nodiscard]] Time stamp() const noexcept {
        return {ros_mavros_status_text_get_stamp_sec(handle()),
                ros_mavros_status_text_get_stamp_nanosec(handle())};
    }
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_mavros_status_text_get_frame_id(handle());
    }
    [[nodiscard]] std::uint8_t severity() const noexcept {
        return ros_mavros_status_text_get_severity(handle());
    }
    [[nodiscard]] std::string_view text() const noexcept {
        return ros_mavros_status_text_get_text(handle());
    }
};

/// @brief Zero-copy view of a mavros_msgs/GPSRAW CDR buffer.
///
/// Field units follow the raw MAVLink GPS_RAW_INT message definition —
/// no conversions are applied.
/// @see https://mavlink.io/en/messages/common.html#GPS_RAW_INT
class MavrosGpsRawView : public detail::ViewBase<MavrosGpsRawView, detail::MavrosGpsRawTraits> {
    using Base = detail::ViewBase<MavrosGpsRawView, detail::MavrosGpsRawTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    static constexpr std::uint8_t GPS_FIX_TYPE_NO_GPS    = 0;
    static constexpr std::uint8_t GPS_FIX_TYPE_NO_FIX    = 1;
    static constexpr std::uint8_t GPS_FIX_TYPE_2D_FIX    = 2;
    static constexpr std::uint8_t GPS_FIX_TYPE_3D_FIX    = 3;
    static constexpr std::uint8_t GPS_FIX_TYPE_DGPS      = 4;
    static constexpr std::uint8_t GPS_FIX_TYPE_RTK_FLOAT = 5;
    static constexpr std::uint8_t GPS_FIX_TYPE_RTK_FIXED = 6;
    static constexpr std::uint8_t GPS_FIX_TYPE_STATIC    = 7;
    static constexpr std::uint8_t GPS_FIX_TYPE_PPP       = 8;

    [[nodiscard]] Time stamp() const noexcept {
        return {ros_mavros_gps_raw_get_stamp_sec(handle()),
                ros_mavros_gps_raw_get_stamp_nanosec(handle())};
    }
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_mavros_gps_raw_get_frame_id(handle());
    }
    /// GPS fix type (see GPS_FIX_TYPE_* constants).
    [[nodiscard]] std::uint8_t fix_type() const noexcept {
        return ros_mavros_gps_raw_get_fix_type(handle());
    }
    /// Latitude in degrees * 1e7 (degE7).
    [[nodiscard]] std::int32_t lat() const noexcept {
        return ros_mavros_gps_raw_get_lat(handle());
    }
    /// Longitude in degrees * 1e7 (degE7).
    [[nodiscard]] std::int32_t lon() const noexcept {
        return ros_mavros_gps_raw_get_lon(handle());
    }
    /// Altitude above MSL in millimetres (mm).
    [[nodiscard]] std::int32_t alt() const noexcept {
        return ros_mavros_gps_raw_get_alt(handle());
    }
    /// GPS HDOP in cm. UINT16_MAX if unknown.
    [[nodiscard]] std::uint16_t eph() const noexcept {
        return ros_mavros_gps_raw_get_eph(handle());
    }
    /// GPS VDOP in cm. UINT16_MAX if unknown.
    [[nodiscard]] std::uint16_t epv() const noexcept {
        return ros_mavros_gps_raw_get_epv(handle());
    }
    /// GPS ground speed in cm/s. UINT16_MAX if unknown.
    [[nodiscard]] std::uint16_t vel() const noexcept {
        return ros_mavros_gps_raw_get_vel(handle());
    }
    /// Course over ground in centidegrees (0-35999). UINT16_MAX if unknown.
    [[nodiscard]] std::uint16_t cog() const noexcept {
        return ros_mavros_gps_raw_get_cog(handle());
    }
    /// Number of satellites visible. UINT8_MAX if unknown.
    [[nodiscard]] std::uint8_t satellites_visible() const noexcept {
        return ros_mavros_gps_raw_get_satellites_visible(handle());
    }
    /// Altitude above WGS84 ellipsoid in millimetres (mm).
    [[nodiscard]] std::int32_t alt_ellipsoid() const noexcept {
        return ros_mavros_gps_raw_get_alt_ellipsoid(handle());
    }
    /// Horizontal position uncertainty in millimetres (mm).
    [[nodiscard]] std::uint32_t h_acc() const noexcept {
        return ros_mavros_gps_raw_get_h_acc(handle());
    }
    /// Vertical position uncertainty in millimetres (mm).
    [[nodiscard]] std::uint32_t v_acc() const noexcept {
        return ros_mavros_gps_raw_get_v_acc(handle());
    }
    /// Speed uncertainty in mm/s.
    [[nodiscard]] std::uint32_t vel_acc() const noexcept {
        return ros_mavros_gps_raw_get_vel_acc(handle());
    }
    /// Heading uncertainty in centidegrees.
    [[nodiscard]] std::int32_t hdg_acc() const noexcept {
        return ros_mavros_gps_raw_get_hdg_acc(handle());
    }
    /// Yaw in centidegrees. 0 if unknown.
    [[nodiscard]] std::uint16_t yaw() const noexcept {
        return ros_mavros_gps_raw_get_yaw(handle());
    }
    /// Number of DGPS satellites.
    [[nodiscard]] std::uint8_t dgps_numch() const noexcept {
        return ros_mavros_gps_raw_get_dgps_numch(handle());
    }
    /// Age of DGPS correction in milliseconds (ms).
    [[nodiscard]] std::uint32_t dgps_age() const noexcept {
        return ros_mavros_gps_raw_get_dgps_age(handle());
    }
};

/// @brief Zero-copy view of a mavros_msgs/TimesyncStatus CDR buffer.
class MavrosTimesyncStatusView : public detail::ViewBase<MavrosTimesyncStatusView, detail::MavrosTimesyncStatusTraits> {
    using Base = detail::ViewBase<MavrosTimesyncStatusView, detail::MavrosTimesyncStatusTraits>;
    friend Base;
    using Base::Base;
public:
    using Base::from_cdr;
    using Base::as_cdr;

    [[nodiscard]] Time stamp() const noexcept {
        return {ros_mavros_timesync_status_get_stamp_sec(handle()),
                ros_mavros_timesync_status_get_stamp_nanosec(handle())};
    }
    [[nodiscard]] std::string_view frame_id() const noexcept {
        return ros_mavros_timesync_status_get_frame_id(handle());
    }
    [[nodiscard]] std::uint64_t remote_timestamp_ns() const noexcept {
        return ros_mavros_timesync_status_get_remote_timestamp_ns(handle());
    }
    [[nodiscard]] std::int64_t observed_offset_ns() const noexcept {
        return ros_mavros_timesync_status_get_observed_offset_ns(handle());
    }
    [[nodiscard]] std::int64_t estimated_offset_ns() const noexcept {
        return ros_mavros_timesync_status_get_estimated_offset_ns(handle());
    }
    [[nodiscard]] float round_trip_time_ms() const noexcept {
        return ros_mavros_timesync_status_get_round_trip_time_ms(handle());
    }
};

} // namespace edgefirst::schemas

#endif // EDGEFIRST_SCHEMAS_HPP
