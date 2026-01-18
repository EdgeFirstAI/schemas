# Python Examples

Comprehensive examples demonstrating EdgeFirst Schemas Python API usage.

## Running Examples

```bash
# Install package in development mode (from repository root)
pip install -e .

# Run examples
python examples/python/basic_types.py
python examples/python/sensor_msgs.py
python examples/python/decode_pointcloud.py
```

## Example Files

### 1. basic_types.py

Demonstrates fundamental ROS2 message types:
- `builtin_interfaces.Time` and `Duration`
- `std_msgs.Header`, `ColorRGBA`
- `geometry_msgs.Vector3`, `Point`, `Quaternion`, `Pose`

**Key concepts:**
- Creating message instances with dataclasses
- Type hints for IDE support
- Accessing nested fields

### 2. sensor_msgs.py

Camera, LiDAR, IMU, and GPS message types with serialization examples.

### 3. decode_pointcloud.py

PointCloud2 decoding with the `decode_pcd` utility.

## Building Blocks

### Creating Messages

```python
from edgefirst.schemas.std_msgs import Header
from edgefirst.schemas.builtin_interfaces import Time

header = Header(
    stamp=Time(sec=1234567890, nanosec=123456789),
    frame_id="camera"
)
```

### Serialization

```python
# Serialize
bytes_data = header.serialize()

# Deserialize
decoded = Header.deserialize(bytes_data)
```

## Additional Resources

- **[EdgeFirst Samples](https://github.com/EdgeFirstAI/samples)** - Complete workflows
- **[Python Type Hints](https://docs.python.org/3/library/typing.html)** - PEP 484

---

**Questions?** support@au-zone.com
