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
- CDR serialization using `serde_cdr` module

## Building Blocks

### Creating Messages

```rust
use edgefirst_schemas::std_msgs::Header;
use edgefirst_schemas::builtin_interfaces::Time;

// Struct initialization
let header = Header {
    stamp: Time { sec: 1234567890, nanosec: 123456789 },
    frame_id: "camera".to_string(),
};
```

### CDR Serialization

```rust
use edgefirst_schemas::serde_cdr::{serialize, deserialize};

// Serialize to CDR bytes (ROS2-compatible)
let bytes = serialize(&header).expect("serialization failed");

// Deserialize from CDR bytes
let decoded: Header = deserialize(&bytes).expect("deserialization failed");

// Verify round-trip
assert_eq!(header, decoded);
```

## Additional Resources

- **[Rust API Docs](https://docs.rs/edgefirst-schemas/)** - Complete API reference
- **[EdgeFirst Samples](https://github.com/EdgeFirstAI/samples)** - Complete workflows

---

**Questions?** support@au-zone.com
