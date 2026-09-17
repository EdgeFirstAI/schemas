# edgefirst-schemas

[![Crates.io](https://img.shields.io/crates/v/edgefirst-schemas.svg)](https://crates.io/crates/edgefirst-schemas)
[![Documentation](https://docs.rs/edgefirst-schemas/badge.svg)](https://docs.rs/edgefirst-schemas)
[![License](https://img.shields.io/crates/l/edgefirst-schemas.svg)](LICENSE)

**EdgeFirst Perception Schemas** — zero-copy CDR1 Little-Endian message types for ROS 2 / Zenoh middleware.

This is the pure-Rust schema crate. It defines:

- **ROS 2 Common Interfaces** — `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `builtin_interfaces`, `rosgraph_msgs`
- **Foxglove visualization** types
- **EdgeFirst custom** perception messages (`Detect`, `Box` / `DetectBox`, `Track`, `Tensor`/`CameraFrame`, `RadarCube`, `RadarInfo`, `Model`, `ModelInfo`)

All messages use a custom zero-copy CDR implementation — no `serde`, no allocations in the hot path. See the workspace [`ARCHITECTURE.md`](../../ARCHITECTURE.md) for the full design rationale.

## Companion crates

- [`edgefirst-schemas-capi`](../capi) — C/C++ static and shared libraries (`libedgefirst_schemas.{a,so}`) exposing the same zero-copy view types via FFI. Built but **not** published to crates.io.
- [`edgefirst-schemas-python`](../python) — PyO3 wheel published to PyPI as `edgefirst-schemas`, importable as `edgefirst.schemas`.

## License

Licensed under Apache-2.0. See the workspace [LICENSE](../../LICENSE) and [NOTICE](../../NOTICE).

<img referrerpolicy="no-referrer-when-downgrade" src="https://px.edgefirst.ai/a.png?x-pxid=6b7d1551-16c8-4cd5-a983-f642b5af1b2b" alt="" width="1" height="1" style="position:absolute; width:1px; height:1px; opacity:0; pointer-events:none;" />
