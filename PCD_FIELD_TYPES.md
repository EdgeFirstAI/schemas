# DynPointCloud: Type-Coercing Field Access

## Problem

`DynPointCloud` and `DynPoint` provide typed field accessors (`read_f32`, `read_u32`, etc.) that enforce exact type matching — `read_f32("cluster_id")` returns `None` if the field is stored as `UINT32`. This forces every consumer to either:

1. Know the exact storage type at compile time (brittle — types vary by service and configuration)
2. Write a cascade of `read_f32().or_else(|| read_u32()...)` attempts (8 name lookups per call)

Real-world PointCloud2 fields have variable types across services:

| Field | lidarpub | fusion (late) | fusion (early) | fusion (grid) |
|-------|----------|---------------|----------------|---------------|
| `x`, `y`, `z` | FLOAT32 | FLOAT32 | FLOAT32 | FLOAT32 |
| `cluster_id` | UINT32 | — | — | UINT32 |
| `vision_class` | — | UINT16 | UINT8 | UINT8 |
| `fusion_class` | — | — | UINT8 | UINT8 |
| `instance_id` | — | UINT16 | UINT16 | UINT16 |
| `track_id` | — | — | — | UINT32 |
| `reflect` | UINT8 | — | — | — |

Consumers like the EdgeFirst samples need to visualize these fields regardless of their storage type. The current API makes this unnecessarily difficult.

## Requested Changes

### 1. Expose `cloud()` on `DynPoint`

`DynPoint.cloud` is currently private. Expose it so consumers can access field metadata from a point reference:

```rust
impl<'a, 'c> DynPoint<'a, 'c> {
    /// Access the parent point cloud for field metadata lookup.
    pub fn cloud(&self) -> &DynPointCloud<'a> {
        self.cloud
    }
}
```

This enables `point.cloud().field("name")` to get a `FieldDesc` and inspect `.field_type` without needing a separate reference to the cloud.

### 2. Type-coercing read methods on `FieldDesc`

Add methods to `FieldDesc` (or `DynPoint`) that coerce any stored numeric type to the requested output type. The `FieldDesc` already knows the `field_type`, so the match is a single branch — no cascading lookups.

The design should support the common coercion targets:

```rust
impl<'a> FieldDesc<'a> {
    /// Read this field from a point's data slice, coercing any numeric type to f64.
    pub fn read_as_f64(&self, point_data: &[u8]) -> Option<f64>;

    /// Read this field, coercing any numeric type to f32.
    /// Integer types wider than 24 bits may lose precision.
    pub fn read_as_f32(&self, point_data: &[u8]) -> Option<f32>;

    /// Read this field, coercing any numeric type to i64.
    /// FLOAT32/FLOAT64 values are truncated toward zero.
    pub fn read_as_i64(&self, point_data: &[u8]) -> Option<i64>;
}
```

Each method matches on `self.field_type`, reads the bytes at `self.byte_offset`, and converts:

```rust
pub fn read_as_f64(&self, point_data: &[u8]) -> Option<f64> {
    let off = self.byte_offset as usize;
    match self.field_type {
        PointFieldType::Float64 => Some(f64::from_le_bytes(point_data[off..off+8].try_into().ok()?)),
        PointFieldType::Float32 => Some(f32::from_le_bytes(point_data[off..off+4].try_into().ok()?) as f64),
        PointFieldType::Uint32  => Some(u32::from_le_bytes(point_data[off..off+4].try_into().ok()?) as f64),
        PointFieldType::Int32   => Some(i32::from_le_bytes(point_data[off..off+4].try_into().ok()?) as f64),
        PointFieldType::Uint16  => Some(u16::from_le_bytes(point_data[off..off+2].try_into().ok()?) as f64),
        PointFieldType::Int16   => Some(i16::from_le_bytes(point_data[off..off+2].try_into().ok()?) as f64),
        PointFieldType::Uint8   => Some(point_data[off] as f64),
        PointFieldType::Int8    => Some(point_data[off] as i8 as f64),
    }
}
```

### 3. Convenience methods on `DynPoint`

For the common case where you want name-based access with type coercion (not performance-critical, or the field is only read once):

```rust
impl<'a, 'c> DynPoint<'a, 'c> {
    /// Read a named field as f64, coercing from any stored numeric type.
    pub fn read_as_f64(&self, name: &str) -> Option<f64> {
        let desc = self.cloud.field(name)?;
        desc.read_as_f64(self.data)
    }

    pub fn read_as_f32(&self, name: &str) -> Option<f32> { ... }
    pub fn read_as_i64(&self, name: &str) -> Option<i64> { ... }
}
```

## Usage Patterns

### Hot-loop pattern (resolve once, read per-point)

```rust
let cloud = DynPointCloud::from_pointcloud2(&pcd)?;

// Resolve field descriptors once
let x_desc = cloud.field("x")?;
let y_desc = cloud.field("y")?;
let z_desc = cloud.field("z")?;
let class_desc = cloud.field("vision_class"); // may not exist

for point in cloud.iter() {
    let x = x_desc.read_as_f32(point.data())?;
    let y = y_desc.read_as_f32(point.data())?;
    let z = z_desc.read_as_f32(point.data())?;
    let class = class_desc.and_then(|d| d.read_as_f64(point.data()));
}
```

This requires exposing `DynPoint::data() -> &[u8]` (currently private).

### Simple pattern (name-based, auto-coercing)

```rust
for point in cloud.iter() {
    let x = point.read_as_f32("x").unwrap_or(0.0);
    let y = point.read_as_f32("y").unwrap_or(0.0);
    let z = point.read_as_f32("z").unwrap_or(0.0);
    let class = point.read_as_f64("vision_class").unwrap_or(0.0);
}
```

### Field discovery

```rust
let cloud = DynPointCloud::from_pointcloud2(&pcd)?;
for field in cloud.fields() {
    println!("{}: {:?} (offset={}, count={})",
        field.name, field.field_type, field.byte_offset, field.count);
}
```

## Summary of Required Visibility Changes

| Item | Status |
|------|--------|
| `DynPoint.cloud` | **Done** — `pub fn cloud()` accessor |
| `DynPoint.data` | **Done** — `pub fn data()` accessor |
| `PointFieldType::read_as_f64` | **Done** — core coercion on the type enum |
| `FieldDesc::read_as_f64` | **Done** — delegates to `PointFieldType::read_as_f64` |
| `FieldDesc::read_as_f32` | **Done** — delegates via f64 then narrows |
| `FieldDesc::read_as_i64` | **Deferred** — future release |
| `DynPoint::read_as_f64` | **Done** — convenience wrapper |
| `DynPoint::read_as_f32` | **Done** — convenience wrapper |
| `DynPoint::read_as_i64` | **Deferred** — future release |
| `DynPointCloud::gather_as_f64` | **Done** — bulk coercing gather |
| `DynPointCloud::gather_as_f32` | **Done** — delegates to `gather_as_f64` |

## Context

The EdgeFirst samples repository currently works around these gaps with a `read_field_as_f64` helper in `rust/lib.rs` that cascades through all 8 `read_*` methods. Once these APIs are available in edgefirst-schemas, the samples will switch to the native API and the workaround will be removed.
