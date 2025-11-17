# EdgeFirst Perception Schemas

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Rust](https://img.shields.io/badge/rust-1.70%2B-orange.svg)](https://www.rust-lang.org/)
[![Python](https://img.shields.io/badge/python-3.8%2B-blue.svg)](https://www.python.org/)

**Core message schemas and language bindings for the EdgeFirst Perception middleware**

EdgeFirst Perception Schemas provides the foundational message types used throughout the [EdgeFirst Perception](https://doc.edgefirst.ai/latest/perception/) middleware stack. It delivers high-performance Rust and Python bindings for consuming and producing messages in EdgeFirst Perception applications. The library implements [ROS2](https://www.ros.org/) Common Interfaces, [Foxglove](https://foxglove.dev/) schemas, and custom EdgeFirst message types with CDR (Common Data Representation) serialization over [Zenoh](https://zenoh.io/).

**No ROS2 Required:** EdgeFirst Perception applications work directly on Linux, Windows, and macOS without any ROS2 installation. For systems that do use ROS2, EdgeFirst Perception interoperates seamlessly through the [Zenoh ROS2 DDS Bridge](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds).

## Features

- **🔄 ROS2 Common Interfaces** - Full compatibility with standard [ROS2](https://www.ros.org/) message types (`geometry_msgs`, `sensor_msgs`, `std_msgs`, `nav_msgs`)
- **📊 Foxglove Schema Support** - Native visualization with [Foxglove Studio](https://foxglove.dev/)
- **⚡ Custom EdgeFirst Messages** - Specialized types for edge AI (detection, tracking, DMA buffers, radar)
- **🦀 High-Performance Rust Bindings** - Zero-copy serialization with CDR encoding
- **🐍 Python Bindings** - Efficient point cloud decoding and message handling
- **📡 Zenoh-Based Communication** - Modern pub/sub over [Zenoh](https://zenoh.io/) middleware
- **💻 Cross-Platform** - Linux, Windows, and macOS support
- **🚫 ROS2 Optional** - No ROS2 installation required for EdgeFirst Perception applications

## Quick Start

### Installation

**Rust** (via [crates.io](https://crates.io/)):

```bash
cargo add edgefirst-schemas
```

**Python** (via pip, when published):

```bash
pip install edgefirst-schemas
```

For detailed installation instructions and troubleshooting, see the [Developer Guide](https://doc.edgefirst.ai/latest/perception/dev/).

### Consuming Messages (Primary Use Case)

Most applications consume messages from EdgeFirst Perception services. Here's how to decode sensor data:

**Python Example - Consuming PointCloud2:**

```python
from edgefirst.schemas import PointCloud2, decode_pcd

# Receive a point cloud message from EdgeFirst Perception
# (via Zenoh subscriber - see samples for complete examples)
points = decode_pcd(point_cloud_msg)

# Access point data
for point in points:
    x, y, z = point.x, point.y, point.z
    # Process point data...
```

**Rust Example - Consuming Detection Results:**

```rust
use edgefirst_schemas::edgefirst_msgs::Detect;
use edgefirst_schemas::sensor_msgs::Image;

fn process_detections(detect_msg: Detect) {
    for bbox in detect_msg.boxes {
        println!("Class: {}, Confidence: {:.2}",
                 bbox.class_id, bbox.confidence);
        // Process detection...
    }
}
```

**Complete working examples:** See the [EdgeFirst Samples](https://github.com/EdgeFirstAI/samples) repository for full subscriber implementations, Zenoh configuration, and integration patterns.

### Producing Messages (Secondary Use Case)

Applications can also produce messages for custom perception pipelines:

**Python Example - Creating Custom Messages:**

```python
from edgefirst.schemas.geometry_msgs import Pose, Point, Quaternion
from edgefirst.schemas.std_msgs import Header

# Create a pose message
pose = Pose(
    position=Point(x=1.0, y=2.0, z=0.5),
    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
)
```

**Rust Example - Building Detection Messages:**

```rust
use edgefirst_schemas::edgefirst_msgs::{Detect, Box as BBox, Track};
use edgefirst_schemas::std_msgs::Header;

fn create_detection() -> Detect {
    Detect {
        header: Header::default(),
        boxes: vec![
            BBox {
                class_id: 1,
                confidence: 0.95,
                x: 100, y: 100, w: 50, h: 50,
                ..Default::default()
            }
        ],
        tracks: vec![],
        model_info: Default::default(),
    }
}
```

**Learn more:** The [Developer Guide](https://doc.edgefirst.ai/latest/perception/dev/) covers serialization, Zenoh publishing, and message lifecycle management.

### Building from Source

**Rust:**

```bash
cargo build --release
cargo test
```

**Python:**

```bash
python -m pip install -e .
```

**ROS2 Debian Package** (for ROS2 integration only):

```bash
cd edgefirst_msgs
source /opt/ros/humble/setup.bash
fakeroot debian/rules build
dpkg -i ../ros-humble-edgefirst-msgs_*.deb
```

For complete build instructions, see [CONTRIBUTING.md](CONTRIBUTING.md).

## Message Schemas

EdgeFirst Perception Schemas combines three sources of message definitions:

### 1. ROS2 Common Interfaces

Standard [ROS2](https://www.ros.org/) message types for broad interoperability:

- **`std_msgs`** - Basic primitive types (Header, String, etc.)
- **`geometry_msgs`** - Spatial messages (Pose, Transform, Twist, etc.)
- **`sensor_msgs`** - Sensor data (PointCloud2, Image, CameraInfo, Imu, NavSatFix, etc.)
- **`nav_msgs`** - Navigation (Odometry, Path)
- **`builtin_interfaces`** - Time and Duration
- **`rosgraph_msgs`** - Clock

Based on [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) LTS release.

### 2. Foxglove Schemas

Visualization-focused message types from [Foxglove Schemas](https://github.com/foxglove/schemas):

- Scene graph visualization - 3D rendering primitives
- Annotation types - Bounding boxes, markers, text
- Panel-specific messages - Optimized for [Foxglove Studio](https://foxglove.dev/)

### 3. EdgeFirst Custom Messages

Specialized types for edge AI perception workflows:

- **`Detect`** - Object detection results with bounding boxes and tracks
- **`Box`** - 2D bounding box with confidence and class
- **`Track`** - Object tracking information with unique IDs
- **`DmaBuffer`** - Zero-copy DMA buffer sharing for hardware accelerators
- **`RadarCube`** - Raw radar data cube for processing
- **`RadarInfo`** - Radar sensor calibration and metadata
- **`Model`** - Neural network model metadata
- **`ModelInfo`** - Inference performance instrumentation

Full message definitions and field descriptions are in the [API Reference](https://docs.rs/edgefirst-schemas/).

## Platform Support

EdgeFirst Perception Schemas works on:

- **Linux** - Primary development and deployment platform
- **Windows** - Full support for development and integration
- **macOS** - Development and testing support

**No ROS2 Required:** Applications can consume and produce EdgeFirst Perception messages on any supported platform without installing ROS2. ROS2 is only needed if you want to bridge EdgeFirst Perception data into an existing ROS2 ecosystem.

See the [EdgeFirst Samples](https://github.com/EdgeFirstAI/samples) repository for platform-specific examples and setup guides.

## Hardware Platforms

EdgeFirst Perception is optimized for Au-Zone edge AI platforms:

- **[Au-Zone Maivin](https://www.edgefirst.ai/edgefirstmodules)** - Edge AI development platform
- **[Au-Zone Raivin](https://www.edgefirst.ai/edgefirstmodules)** - Rugged edge AI computer for industrial deployment

These platforms provide hardware-accelerated inference and sensor integration. For custom hardware projects, contact Au-Zone for engineering services.

## Use Cases

EdgeFirst Perception Schemas enables:

- **Consuming Sensor Data** - Subscribe to camera, radar, lidar, IMU, GPS topics
- **Processing Detections** - Receive object detection and tracking results
- **Custom Perception Services** - Build new perception algorithms that integrate with EdgeFirst
- **Recording & Playback** - Use with MCAP for data recording and analysis
- **Visualization** - Connect [Foxglove Studio](https://foxglove.dev/) for real-time monitoring
- **ROS2 Integration** - Bridge to ROS2 systems when needed

**Example applications:** Explore the [EdgeFirst Samples](https://github.com/EdgeFirstAI/samples) for complete implementations including camera subscribers, detection visualizers, sensor fusion examples, and custom service templates.

## Documentation

- **[EdgeFirst Perception Documentation](https://doc.edgefirst.ai/latest/perception/)** - Main documentation hub
- **[Developer Guide](https://doc.edgefirst.ai/latest/perception/dev/)** - In-depth development guide
- **[EdgeFirst Samples](https://github.com/EdgeFirstAI/samples)** - Working code examples
- **[API Reference](https://docs.rs/edgefirst-schemas/)** - Rust API documentation
- **[ROS2 Message Reference](https://docs.ros.org/en/humble/p/common_interfaces/)** - ROS2 Common Interfaces
- **[Foxglove Schemas](https://github.com/foxglove/schemas)** - Foxglove message definitions

## Support

### Community Resources

- **[GitHub Discussions](https://github.com/EdgeFirstAI/schemas/discussions)** - Ask questions and share ideas
- **[Issue Tracker](https://github.com/EdgeFirstAI/schemas/issues)** - Report bugs and request features
- **[EdgeFirst Documentation](https://doc.edgefirst.ai/latest/perception/)** - Comprehensive guides and tutorials
- **[Sample Code](https://github.com/EdgeFirstAI/samples)** - Example applications and integrations

### EdgeFirst Ecosystem

**[EdgeFirst Studio](https://studio.edgefirst.ai)** - Complete MLOps platform for edge AI:
- Deploy models to devices running EdgeFirst Perception
- Monitor inference performance in real-time
- Manage fleets of edge devices
- Record and replay sensor data with MCAP
- Visualize messages with integrated Foxglove
- Free tier available for development

**EdgeFirst Hardware Platforms**:
- [Maivin and Raivin](https://www.edgefirst.ai/edgefirstmodules) edge AI computers
- Custom carrier board design services
- Rugged industrial enclosures for harsh environments

### Professional Services

Au-Zone Technologies offers commercial support for production deployments:

- **Training & Workshops** - Accelerate your team's development with EdgeFirst Perception
- **Custom Development** - Tailored perception solutions and algorithm integration
- **Integration Services** - Connect EdgeFirst Perception to your existing systems
- **Production Support** - SLA-backed support for mission-critical applications

**Contact:** support@au-zone.com | **Learn more:** [au-zone.com](https://au-zone.com)

## Architecture

For detailed information about message serialization, CDR encoding, Zenoh communication patterns, and system architecture, see the [ARCHITECTURE.md](ARCHITECTURE.md) document.

Quick overview:
- Messages are serialized using CDR (Common Data Representation)
- Communication happens over [Zenoh](https://zenoh.io/) pub/sub middleware
- ROS2 interoperability via [Zenoh ROS2 DDS Bridge](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds) when needed
- Zero-copy optimizations for embedded platforms using DMA buffers

## Contributing

We welcome contributions! Please see our [Contributing Guidelines](CONTRIBUTING.md) for details on:

- Development setup and build process
- Code style and testing requirements
- Pull request process
- Issue reporting guidelines

All contributors must follow our [Code of Conduct](CODE_OF_CONDUCT.md).

## Security

For reporting security vulnerabilities, please see our [Security Policy](SECURITY.md). Do not report security issues through public GitHub issues.

## License

Licensed under the Apache License, Version 2.0. See [LICENSE](LICENSE) for details.

```
Copyright © 2025 Au-Zone Technologies. All Rights Reserved.
```

### Third-Party Acknowledgments

This project incorporates schemas and code from:

- **[ROS2 Common Interfaces](https://github.com/ros2/common_interfaces)** - Apache-2.0 License
- **[Foxglove Schemas](https://github.com/foxglove/schemas)** - MIT License
- **[zenoh-ros-type](https://github.com/evshary/zenoh-ros-type)** - Apache-2.0 License (original Rust implementation basis)

See [NOTICE](NOTICE) file for complete third-party license information.

## Related Projects

- **[Zenoh](https://zenoh.io/)** - High-performance pub/sub middleware
- **[ROS2](https://www.ros.org/)** - Robot Operating System
- **[Foxglove Studio](https://foxglove.dev/)** - Robotics visualization and debugging
- **[EdgeFirst Samples](https://github.com/EdgeFirstAI/samples)** - Example applications

---

**EdgeFirst Perception** is a trademark of Au-Zone Technologies Inc.
