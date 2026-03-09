# Rust Examples

Comprehensive examples demonstrating EdgeFirst Schemas Rust API usage.

## Running Examples

```bash
# From repository root
cargo run --example basic_types
```

## Example Files

### basic_types.rs

Demonstrates fundamental ROS2 message types:
- `builtin_interfaces::Time` and `Duration`
- `std_msgs::Header`, `ColorRGBA`
- `geometry_msgs::Vector3`, `Point`, `Quaternion`, `Pose`, `Transform`

**Key concepts:**
- Struct initialization with named fields
- Accessing nested fields
- CDR serialization using buffer-backed zero-copy types

## Building Blocks

### Creating Messages

```rust
use edgefirst_schemas::std_msgs::Header;
use edgefirst_schemas::builtin_interfaces::Time;

// Buffer-backed zero-copy construction
let stamp = Time { sec: 1234567890, nanosec: 123456789 };
let header = Header::new(stamp, "camera").unwrap();
```

### CDR Serialization

```rust
// Serialize to CDR bytes (ROS2-compatible)
let bytes = header.to_cdr();

// Deserialize from CDR bytes (zero-copy)
let decoded = Header::from_cdr(&bytes[..]).unwrap();

// Verify round-trip
assert_eq!(decoded.stamp(), stamp);
assert_eq!(decoded.frame_id(), "camera");
```

## Additional Resources

- **[Rust API Docs](https://docs.rs/edgefirst-schemas/)** - Complete API reference
- **[EdgeFirst Samples](https://github.com/EdgeFirstAI/samples)** - Complete workflows

---

**Questions?** support@au-zone.com
