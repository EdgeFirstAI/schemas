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
 * The wrapper exposes two type categories per buffer-backed message:
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
 * Not every type has an owning sibling: types without a corresponding
 * `ros_<type>_encode` function in the C API are view-only. These include
 * `ImuView`, `NavSatFixView`, `CameraInfoView`, `TransformStampedView`,
 * `PointCloud2View`, `RadarCubeView`, `RadarInfoView`, `DetectView`,
 * `ModelView`, `ModelInfoView`, `LocalTimeView`, `TrackView`, `BoxView`.
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
 * A future release may introduce an `encode_into(span<uint8_t>)` variant
 * that encodes into a caller-provided buffer, eliminating the
 * per-message allocation entirely. This is a C API change and is
 * tracked separately.
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

} // namespace edgefirst::schemas

#endif // EDGEFIRST_SCHEMAS_HPP
