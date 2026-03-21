// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! Zero-copy point cloud access layer over PointCloud2 data buffers.
//!
//! Provides two tiers of access:
//! - [`DynPointCloud`]: Runtime-defined field inspection with dynamic typed access.
//! - [`PointCloud`]: Compile-time typed access via the [`Point`] trait.
//!
//! Both modes are zero-copy views over the PointCloud2 data buffer.
//! Field reads use `from_le_bytes` on small byte-array copies (matching
//! the pattern used by [`CdrCursor`](crate::cdr::CdrCursor)).

use super::PointFieldView;

/// Maximum number of fields supported by [`DynPointCloud`].
///
/// 16 covers all practical sensor outputs. Clouds with more fields
/// than this should use the static [`PointCloud<P>`] tier instead.
pub const MAX_FIELDS: usize = 16;

// ── PointFieldType ──────────────────────────────────────────────────

/// Typed representation of PointField datatype constants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointFieldType {
    Int8 = 1,
    Uint8 = 2,
    Int16 = 3,
    Uint16 = 4,
    Int32 = 5,
    Uint32 = 6,
    Float32 = 7,
    Float64 = 8,
}

impl PointFieldType {
    /// Convert from the ROS2 PointField datatype constant.
    pub fn from_datatype(dt: u8) -> Option<Self> {
        match dt {
            1 => Some(Self::Int8),
            2 => Some(Self::Uint8),
            3 => Some(Self::Int16),
            4 => Some(Self::Uint16),
            5 => Some(Self::Int32),
            6 => Some(Self::Uint32),
            7 => Some(Self::Float32),
            8 => Some(Self::Float64),
            _ => None,
        }
    }

    /// Size of a single scalar of this type in bytes.
    pub fn size_bytes(self) -> usize {
        match self {
            Self::Int8 | Self::Uint8 => 1,
            Self::Int16 | Self::Uint16 => 2,
            Self::Int32 | Self::Uint32 | Self::Float32 => 4,
            Self::Float64 => 8,
        }
    }
}

// ── FieldDesc ───────────────────────────────────────────────────────

/// Resolved field descriptor with typed information.
#[derive(Debug, Clone, Copy)]
pub struct FieldDesc<'a> {
    pub name: &'a str,
    pub byte_offset: u32,
    pub field_type: PointFieldType,
    pub count: u32,
}

impl<'a> FieldDesc<'a> {
    /// Create from a PointFieldView, validating the datatype.
    pub fn from_view(view: &PointFieldView<'a>) -> Option<Self> {
        Some(FieldDesc {
            name: view.name,
            byte_offset: view.offset,
            field_type: PointFieldType::from_datatype(view.datatype)?,
            count: view.count,
        })
    }
}

// ── PointCloudError ─────────────────────────────────────────────────

/// Errors from pointcloud view construction or field access.
#[derive(Debug)]
pub enum PointCloudError {
    /// A required field was not found in the PointCloud2 field list.
    FieldNotFound { name: &'static str },
    /// A field was found but has mismatched type or offset.
    FieldMismatch {
        name: &'static str,
        reason: &'static str,
    },
    /// The cloud has more fields than MAX_FIELDS.
    TooManyFields { found: usize },
    /// An unrecognized PointField datatype constant was encountered.
    UnknownDatatype { field_name: String, datatype: u8 },
    /// Big-endian point data is not supported.
    BigEndianNotSupported,
    /// The point step is zero or inconsistent with the data length.
    InvalidLayout { reason: &'static str },
    /// A field descriptor's byte offset is out of range for the point data.
    FieldAccessOutOfBounds { byte_offset: u32 },
}

impl core::fmt::Display for PointCloudError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::FieldNotFound { name } => write!(f, "field not found: {name}"),
            Self::FieldMismatch { name, reason } => {
                write!(f, "field mismatch for '{name}': {reason}")
            }
            Self::TooManyFields { found } => {
                write!(f, "too many fields: {found} (max {MAX_FIELDS})")
            }
            Self::UnknownDatatype {
                field_name,
                datatype,
            } => write!(f, "unknown datatype {datatype} for field '{field_name}'"),
            Self::BigEndianNotSupported => write!(f, "big-endian point data not supported"),
            Self::InvalidLayout { reason } => write!(f, "invalid layout: {reason}"),
            Self::FieldAccessOutOfBounds { byte_offset } => {
                write!(f, "field access out of bounds at byte offset {byte_offset}")
            }
        }
    }
}

impl std::error::Error for PointCloudError {}

// ── PointScalar ─────────────────────────────────────────────────────

/// Maps a Rust primitive to its PointFieldType and provides LE byte reading.
///
/// This is an implementation detail of [`define_point!`].
pub trait PointScalar: Sized {
    const FIELD_TYPE: PointFieldType;
    fn read_le(data: &[u8], offset: usize) -> Self;
}

impl PointScalar for f32 {
    const FIELD_TYPE: PointFieldType = PointFieldType::Float32;
    #[inline(always)]
    fn read_le(data: &[u8], offset: usize) -> Self {
        let bytes: [u8; 4] = data[offset..offset + 4]
            .try_into()
            .expect("bounds checked by caller");
        f32::from_le_bytes(bytes)
    }
}

impl PointScalar for f64 {
    const FIELD_TYPE: PointFieldType = PointFieldType::Float64;
    #[inline(always)]
    fn read_le(data: &[u8], offset: usize) -> Self {
        let bytes: [u8; 8] = data[offset..offset + 8]
            .try_into()
            .expect("bounds checked by caller");
        f64::from_le_bytes(bytes)
    }
}

impl PointScalar for u8 {
    const FIELD_TYPE: PointFieldType = PointFieldType::Uint8;
    #[inline(always)]
    fn read_le(data: &[u8], offset: usize) -> Self {
        data[offset]
    }
}

impl PointScalar for i8 {
    const FIELD_TYPE: PointFieldType = PointFieldType::Int8;
    #[inline(always)]
    fn read_le(data: &[u8], offset: usize) -> Self {
        data[offset] as i8
    }
}

impl PointScalar for u16 {
    const FIELD_TYPE: PointFieldType = PointFieldType::Uint16;
    #[inline(always)]
    fn read_le(data: &[u8], offset: usize) -> Self {
        let bytes: [u8; 2] = data[offset..offset + 2]
            .try_into()
            .expect("bounds checked by caller");
        u16::from_le_bytes(bytes)
    }
}

impl PointScalar for i16 {
    const FIELD_TYPE: PointFieldType = PointFieldType::Int16;
    #[inline(always)]
    fn read_le(data: &[u8], offset: usize) -> Self {
        let bytes: [u8; 2] = data[offset..offset + 2]
            .try_into()
            .expect("bounds checked by caller");
        i16::from_le_bytes(bytes)
    }
}

impl PointScalar for u32 {
    const FIELD_TYPE: PointFieldType = PointFieldType::Uint32;
    #[inline(always)]
    fn read_le(data: &[u8], offset: usize) -> Self {
        let bytes: [u8; 4] = data[offset..offset + 4]
            .try_into()
            .expect("bounds checked by caller");
        u32::from_le_bytes(bytes)
    }
}

impl PointScalar for i32 {
    const FIELD_TYPE: PointFieldType = PointFieldType::Int32;
    #[inline(always)]
    fn read_le(data: &[u8], offset: usize) -> Self {
        let bytes: [u8; 4] = data[offset..offset + 4]
            .try_into()
            .expect("bounds checked by caller");
        i32::from_le_bytes(bytes)
    }
}

// ── Point trait ─────────────────────────────────────────────────────

/// Expected field descriptor for compile-time point validation.
pub struct ExpectedField {
    pub name: &'static str,
    pub byte_offset: u32,
    pub field_type: PointFieldType,
}

/// Trait for compile-time typed point structs.
///
/// Implement this via the [`define_point!`] macro. The trait provides
/// field layout metadata for validation and a `read_from` function
/// for reading a point from a data buffer at a given byte offset.
pub trait Point: Sized {
    /// Number of scalar fields.
    const FIELD_COUNT: usize;

    /// Expected field layout for runtime validation against PointCloud2.
    fn expected_fields() -> &'static [ExpectedField];

    /// Size of one point record in bytes (max offset + field size).
    fn point_size() -> u32;

    /// Read one point from a data buffer at the given byte offset.
    ///
    /// # Panics
    /// Panics if `base + point_size() > data.len()`.
    fn read_from(data: &[u8], base: usize) -> Self;
}

/// Define a named point struct implementing the [`Point`] trait.
///
/// # Example
/// ```
/// use edgefirst_schemas::define_point;
/// use edgefirst_schemas::sensor_msgs::pointcloud::{Point, PointFieldType};
///
/// define_point! {
///     pub struct XyzPoint {
///         x: f32 => 0,
///         y: f32 => 4,
///         z: f32 => 8,
///     }
/// }
///
/// assert_eq!(XyzPoint::point_size(), 12);
/// ```
#[macro_export]
macro_rules! define_point {
    (
        $(#[$meta:meta])*
        $vis:vis struct $Name:ident {
            $( $field:ident : $ty:ty => $offset:expr ),+ $(,)?
        }
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, Copy, PartialEq)]
        $vis struct $Name {
            $( pub $field: $ty ),+
        }

        impl $crate::sensor_msgs::pointcloud::Point for $Name {
            const FIELD_COUNT: usize =
                $crate::_point_field_count!($($field)+);

            fn expected_fields()
                -> &'static [$crate::sensor_msgs::pointcloud::ExpectedField]
            {
                &[
                    $(
                        $crate::sensor_msgs::pointcloud::ExpectedField {
                            name: stringify!($field),
                            byte_offset: $offset,
                            field_type:
                                <$ty as $crate::sensor_msgs::pointcloud::PointScalar>
                                    ::FIELD_TYPE,
                        }
                    ),+
                ]
            }

            fn point_size() -> u32 {
                let mut size = 0u32;
                $(
                    {
                        let end = $offset + (core::mem::size_of::<$ty>() as u32);
                        if end > size { size = end; }
                    }
                )+
                size
            }

            fn read_from(data: &[u8], base: usize) -> Self {
                $Name {
                    $(
                        $field:
                            <$ty as $crate::sensor_msgs::pointcloud::PointScalar>
                                ::read_le(data, base + ($offset as usize))
                    ),+
                }
            }
        }
    };
}

/// Helper macro to count fields. Not public API.
#[macro_export]
#[doc(hidden)]
macro_rules! _point_field_count {
    () => { 0usize };
    ($x:ident $($rest:ident)*) => {
        1usize + $crate::_point_field_count!($($rest)*)
    };
}

// Re-export macros under the pointcloud module path for discoverability.
pub use crate::_point_field_count;
pub use crate::define_point;

// ── DynPointCloud ───────────────────────────────────────────────────

/// Zero-copy dynamic point cloud view over PointCloud2 data.
///
/// Fields are resolved at construction time from the PointCloud2 field
/// descriptors. Access is by field name with explicit type. Supports up
/// to [`MAX_FIELDS`] fields per point; clouds with more fields should
/// use the compile-time [`PointCloud<P>`] tier instead.
///
/// The returned view borrows the PointCloud2's internal data buffer —
/// no data is copied. The `PointCloud2` must outlive this view.
///
/// # Example
/// ```ignore
/// let cloud = DynPointCloud::from_pointcloud2(&pcd2)?;
/// for point in cloud.iter() {
///     let x = point.read_f32("x");
///     let y = point.read_f32("y");
///     let z = point.read_f32("z");
/// }
/// ```
#[derive(Debug)]
pub struct DynPointCloud<'a> {
    data: &'a [u8],
    point_step: usize,
    row_step: usize,
    num_points: usize,
    fields: [Option<FieldDesc<'a>>; MAX_FIELDS],
    field_count: usize,
    height: u32,
    width: u32,
}

impl<'a> DynPointCloud<'a> {
    /// Create a dynamic point cloud view from a PointCloud2 message.
    ///
    /// Returns a zero-copy view that borrows the PointCloud2's internal
    /// data buffer. The `PointCloud2` must outlive the returned view.
    ///
    /// # Errors
    ///
    /// - [`PointCloudError::BigEndianNotSupported`] — big-endian point data.
    /// - [`PointCloudError::InvalidLayout`] — `point_step` is zero, data
    ///   buffer is shorter than `num_points × point_step`, or a field
    ///   extends beyond `point_step`.
    /// - [`PointCloudError::TooManyFields`] — more than [`MAX_FIELDS`] fields.
    /// - [`PointCloudError::UnknownDatatype`] — unrecognized PointField datatype.
    pub fn from_pointcloud2<B: AsRef<[u8]>>(
        pc: &'a super::PointCloud2<B>,
    ) -> Result<Self, PointCloudError> {
        if pc.is_bigendian() {
            return Err(PointCloudError::BigEndianNotSupported);
        }

        let point_step = pc.point_step() as usize;
        if point_step == 0 {
            return Err(PointCloudError::InvalidLayout {
                reason: "point_step is zero",
            });
        }

        let num_points = pc.point_count();
        let data = pc.data();
        let height = pc.height() as usize;
        let width = pc.width() as usize;
        let row_step = pc.row_step() as usize;

        if num_points > 0 {
            // row_step must accommodate at least width × point_step.
            let min_row_step =
                width
                    .checked_mul(point_step)
                    .ok_or(PointCloudError::InvalidLayout {
                        reason: "width × point_step overflows usize",
                    })?;
            if row_step < min_row_step {
                return Err(PointCloudError::InvalidLayout {
                    reason: "row_step smaller than width × point_step",
                });
            }

            // Data buffer must hold height × row_step bytes.
            let required_len =
                height
                    .checked_mul(row_step)
                    .ok_or(PointCloudError::InvalidLayout {
                        reason: "height × row_step overflows usize",
                    })?;
            if data.len() < required_len {
                return Err(PointCloudError::InvalidLayout {
                    reason: "data buffer shorter than height × row_step",
                });
            }
        }

        let mut fields = [const { None }; MAX_FIELDS];
        let mut field_count = 0;

        for view in pc.fields_iter() {
            if field_count >= MAX_FIELDS {
                return Err(PointCloudError::TooManyFields {
                    found: field_count + 1,
                });
            }
            let desc =
                FieldDesc::from_view(&view).ok_or_else(|| PointCloudError::UnknownDatatype {
                    field_name: view.name.to_string(),
                    datatype: view.datatype,
                })?;
            // Validate field fits within point_step (accounting for count).
            let field_size = desc
                .field_type
                .size_bytes()
                .checked_mul(desc.count as usize)
                .ok_or(PointCloudError::InvalidLayout {
                    reason: "field count × size overflows usize",
                })?;
            let field_end = (desc.byte_offset as usize).checked_add(field_size).ok_or(
                PointCloudError::InvalidLayout {
                    reason: "field offset + size overflows usize",
                },
            )?;
            if field_end > point_step {
                return Err(PointCloudError::InvalidLayout {
                    reason: "field extends beyond point_step",
                });
            }
            fields[field_count] = Some(desc);
            field_count += 1;
        }

        Ok(DynPointCloud {
            data,
            point_step,
            row_step: pc.row_step() as usize,
            num_points,
            fields,
            field_count,
            height: pc.height(),
            width: pc.width(),
        })
    }

    /// Number of points in the cloud.
    pub fn len(&self) -> usize {
        self.num_points
    }

    /// Whether the cloud is empty.
    pub fn is_empty(&self) -> bool {
        self.num_points == 0
    }

    /// Height (number of rows). 1 for unorganized clouds.
    pub fn height(&self) -> u32 {
        self.height
    }

    /// Width (number of columns per row).
    pub fn width(&self) -> u32 {
        self.width
    }

    /// Point step in bytes.
    pub fn point_step(&self) -> usize {
        self.point_step
    }

    /// Number of fields per point.
    pub fn field_count(&self) -> usize {
        self.field_count
    }

    /// Iterate over field descriptors.
    pub fn fields(&self) -> impl Iterator<Item = &FieldDesc<'a>> {
        self.fields[..self.field_count]
            .iter()
            .filter_map(|f| f.as_ref())
    }

    /// Look up a field by name.
    pub fn field(&self, name: &str) -> Option<&FieldDesc<'a>> {
        self.fields().find(|f| f.name == name)
    }

    /// Compute the byte offset of the i-th point, correctly handling
    /// row padding in organized clouds.
    #[inline]
    fn point_offset(&self, i: usize) -> usize {
        if self.row_step == (self.width as usize) * self.point_step {
            i * self.point_step
        } else {
            let w = self.width as usize;
            (i / w) * self.row_step + (i % w) * self.point_step
        }
    }

    /// Get a single point view by linear index.
    pub fn point(&self, index: usize) -> Option<DynPoint<'a, '_>> {
        if index >= self.num_points {
            return None;
        }
        let base = self.point_offset(index);
        let end = base + self.point_step;
        if end > self.data.len() {
            return None;
        }
        Some(DynPoint {
            data: &self.data[base..end],
            cloud: self,
        })
    }

    /// Get a point by (row, col) for organized clouds.
    ///
    /// Uses `row_step` to correctly handle row padding in organized clouds.
    pub fn point_at(&self, row: u32, col: u32) -> Option<DynPoint<'a, '_>> {
        if row >= self.height || col >= self.width {
            return None;
        }
        let base = (row as usize) * self.row_step + (col as usize) * self.point_step;
        let end = base + self.point_step;
        if end > self.data.len() {
            return None;
        }
        Some(DynPoint {
            data: &self.data[base..end],
            cloud: self,
        })
    }

    /// Iterate over all points.
    pub fn iter(&self) -> DynPointIter<'a, '_> {
        DynPointIter {
            cloud: self,
            index: 0,
        }
    }

    /// Gather a named f32 field into a Vec.
    ///
    /// **Note:** Allocates a `Vec` of `num_points` elements. For hot-path
    /// access without allocation, use [`DynPoint::read_f32_at`] with a
    /// pre-resolved descriptor instead.
    pub fn gather_f32(&self, name: &str) -> Option<Vec<f32>> {
        let desc = self.field(name)?;
        if desc.field_type != PointFieldType::Float32 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let mut out = Vec::with_capacity(self.num_points);
        for i in 0..self.num_points {
            let base = self.point_offset(i) + off;
            let bytes: [u8; 4] = self.data[base..base + 4].try_into().ok()?;
            out.push(f32::from_le_bytes(bytes));
        }
        Some(out)
    }

    /// Gather a named u32 field into a Vec.
    ///
    /// **Note:** Allocates a `Vec` of `num_points` elements. For hot-path
    /// access without allocation, use [`DynPoint::read_u32_at`] with a
    /// pre-resolved descriptor instead.
    pub fn gather_u32(&self, name: &str) -> Option<Vec<u32>> {
        let desc = self.field(name)?;
        if desc.field_type != PointFieldType::Uint32 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let mut out = Vec::with_capacity(self.num_points);
        for i in 0..self.num_points {
            let base = self.point_offset(i) + off;
            let bytes: [u8; 4] = self.data[base..base + 4].try_into().ok()?;
            out.push(u32::from_le_bytes(bytes));
        }
        Some(out)
    }

    /// Gather a named u16 field into a Vec.
    ///
    /// **Note:** Allocates a `Vec` of `num_points` elements. For hot-path
    /// access without allocation, use [`DynPoint::read_u16_at`] with a
    /// pre-resolved descriptor instead.
    pub fn gather_u16(&self, name: &str) -> Option<Vec<u16>> {
        let desc = self.field(name)?;
        if desc.field_type != PointFieldType::Uint16 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let mut out = Vec::with_capacity(self.num_points);
        for i in 0..self.num_points {
            let base = self.point_offset(i) + off;
            let bytes: [u8; 2] = self.data[base..base + 2].try_into().ok()?;
            out.push(u16::from_le_bytes(bytes));
        }
        Some(out)
    }

    /// Gather a named u8 field into a Vec.
    ///
    /// **Note:** Allocates a `Vec` of `num_points` elements. For hot-path
    /// access without allocation, use [`DynPoint::read_u8_at`] with a
    /// pre-resolved descriptor instead.
    pub fn gather_u8(&self, name: &str) -> Option<Vec<u8>> {
        let desc = self.field(name)?;
        if desc.field_type != PointFieldType::Uint8 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let mut out = Vec::with_capacity(self.num_points);
        for i in 0..self.num_points {
            out.push(self.data[self.point_offset(i) + off]);
        }
        Some(out)
    }

    /// Gather a named i8 field into a Vec.
    ///
    /// **Note:** Allocates a `Vec` of `num_points` elements. For hot-path
    /// access without allocation, use [`DynPoint::read_i8_at`] with a
    /// pre-resolved descriptor instead.
    pub fn gather_i8(&self, name: &str) -> Option<Vec<i8>> {
        let desc = self.field(name)?;
        if desc.field_type != PointFieldType::Int8 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let mut out = Vec::with_capacity(self.num_points);
        for i in 0..self.num_points {
            out.push(self.data[self.point_offset(i) + off] as i8);
        }
        Some(out)
    }

    /// Gather a named i16 field into a Vec.
    ///
    /// **Note:** Allocates a `Vec` of `num_points` elements. For hot-path
    /// access without allocation, use [`DynPoint::read_i16_at`] with a
    /// pre-resolved descriptor instead.
    pub fn gather_i16(&self, name: &str) -> Option<Vec<i16>> {
        let desc = self.field(name)?;
        if desc.field_type != PointFieldType::Int16 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let mut out = Vec::with_capacity(self.num_points);
        for i in 0..self.num_points {
            let base = self.point_offset(i) + off;
            let bytes: [u8; 2] = self.data[base..base + 2].try_into().ok()?;
            out.push(i16::from_le_bytes(bytes));
        }
        Some(out)
    }

    /// Gather a named i32 field into a Vec.
    ///
    /// **Note:** Allocates a `Vec` of `num_points` elements. For hot-path
    /// access without allocation, use [`DynPoint::read_i32_at`] with a
    /// pre-resolved descriptor instead.
    pub fn gather_i32(&self, name: &str) -> Option<Vec<i32>> {
        let desc = self.field(name)?;
        if desc.field_type != PointFieldType::Int32 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let mut out = Vec::with_capacity(self.num_points);
        for i in 0..self.num_points {
            let base = self.point_offset(i) + off;
            let bytes: [u8; 4] = self.data[base..base + 4].try_into().ok()?;
            out.push(i32::from_le_bytes(bytes));
        }
        Some(out)
    }

    /// Gather a named f64 field into a Vec.
    ///
    /// **Note:** Allocates a `Vec` of `num_points` elements. For hot-path
    /// access without allocation, use [`DynPoint::read_f64_at`] with a
    /// pre-resolved descriptor instead.
    pub fn gather_f64(&self, name: &str) -> Option<Vec<f64>> {
        let desc = self.field(name)?;
        if desc.field_type != PointFieldType::Float64 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let mut out = Vec::with_capacity(self.num_points);
        for i in 0..self.num_points {
            let base = self.point_offset(i) + off;
            let bytes: [u8; 8] = self.data[base..base + 8].try_into().ok()?;
            out.push(f64::from_le_bytes(bytes));
        }
        Some(out)
    }
}

// ── DynPoint ────────────────────────────────────────────────────────

/// Zero-copy view of a single point within a [`DynPointCloud`].
///
/// Provides typed field access by name (`read_f32("x")`) or by
/// pre-resolved [`FieldDesc`] (`read_f32_at(&desc)`) for hot-path
/// use where repeated name lookups would be wasteful.
///
/// Field access reads bytes from the point's data slice using
/// `from_le_bytes` — no unsafe code, no allocation, no alignment concerns.
pub struct DynPoint<'a, 'c> {
    data: &'a [u8],
    cloud: &'c DynPointCloud<'a>,
}

impl<'a, 'c> DynPoint<'a, 'c> {
    /// Read an f32 field by name. Returns None if field not found or wrong type.
    pub fn read_f32(&self, name: &str) -> Option<f32> {
        let desc = self.cloud.field(name)?;
        if desc.field_type != PointFieldType::Float32 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let bytes: [u8; 4] = self.data[off..off + 4].try_into().ok()?;
        Some(f32::from_le_bytes(bytes))
    }

    /// Read a u32 field by name.
    pub fn read_u32(&self, name: &str) -> Option<u32> {
        let desc = self.cloud.field(name)?;
        if desc.field_type != PointFieldType::Uint32 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let bytes: [u8; 4] = self.data[off..off + 4].try_into().ok()?;
        Some(u32::from_le_bytes(bytes))
    }

    /// Read a u16 field by name.
    pub fn read_u16(&self, name: &str) -> Option<u16> {
        let desc = self.cloud.field(name)?;
        if desc.field_type != PointFieldType::Uint16 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let bytes: [u8; 2] = self.data[off..off + 2].try_into().ok()?;
        Some(u16::from_le_bytes(bytes))
    }

    /// Read a u8 field by name.
    pub fn read_u8(&self, name: &str) -> Option<u8> {
        let desc = self.cloud.field(name)?;
        if desc.field_type != PointFieldType::Uint8 {
            return None;
        }
        Some(self.data[desc.byte_offset as usize])
    }

    /// Read an i8 field by name.
    pub fn read_i8(&self, name: &str) -> Option<i8> {
        let desc = self.cloud.field(name)?;
        if desc.field_type != PointFieldType::Int8 {
            return None;
        }
        Some(self.data[desc.byte_offset as usize] as i8)
    }

    /// Read an i16 field by name.
    pub fn read_i16(&self, name: &str) -> Option<i16> {
        let desc = self.cloud.field(name)?;
        if desc.field_type != PointFieldType::Int16 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let bytes: [u8; 2] = self.data[off..off + 2].try_into().ok()?;
        Some(i16::from_le_bytes(bytes))
    }

    /// Read an i32 field by name.
    pub fn read_i32(&self, name: &str) -> Option<i32> {
        let desc = self.cloud.field(name)?;
        if desc.field_type != PointFieldType::Int32 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let bytes: [u8; 4] = self.data[off..off + 4].try_into().ok()?;
        Some(i32::from_le_bytes(bytes))
    }

    /// Read an f64 field by name.
    pub fn read_f64(&self, name: &str) -> Option<f64> {
        let desc = self.cloud.field(name)?;
        if desc.field_type != PointFieldType::Float64 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let bytes: [u8; 8] = self.data[off..off + 8].try_into().ok()?;
        Some(f64::from_le_bytes(bytes))
    }

    /// Read an f32 field by pre-resolved descriptor (avoids name lookup).
    ///
    /// # Errors
    /// Returns [`PointCloudError::FieldAccessOutOfBounds`] if the descriptor's
    /// byte offset exceeds the point data slice.
    pub fn read_f32_at(&self, desc: &FieldDesc<'_>) -> Result<f32, PointCloudError> {
        let off = desc.byte_offset as usize;
        let bytes: [u8; 4] = self
            .data
            .get(off..off + 4)
            .ok_or(PointCloudError::FieldAccessOutOfBounds {
                byte_offset: desc.byte_offset,
            })?
            .try_into()
            .map_err(|_| PointCloudError::FieldAccessOutOfBounds {
                byte_offset: desc.byte_offset,
            })?;
        Ok(f32::from_le_bytes(bytes))
    }

    /// Read a u32 field by pre-resolved descriptor.
    pub fn read_u32_at(&self, desc: &FieldDesc<'_>) -> Result<u32, PointCloudError> {
        let off = desc.byte_offset as usize;
        let bytes: [u8; 4] = self
            .data
            .get(off..off + 4)
            .ok_or(PointCloudError::FieldAccessOutOfBounds {
                byte_offset: desc.byte_offset,
            })?
            .try_into()
            .map_err(|_| PointCloudError::FieldAccessOutOfBounds {
                byte_offset: desc.byte_offset,
            })?;
        Ok(u32::from_le_bytes(bytes))
    }

    /// Read a u16 field by pre-resolved descriptor.
    pub fn read_u16_at(&self, desc: &FieldDesc<'_>) -> Result<u16, PointCloudError> {
        let off = desc.byte_offset as usize;
        let bytes: [u8; 2] = self
            .data
            .get(off..off + 2)
            .ok_or(PointCloudError::FieldAccessOutOfBounds {
                byte_offset: desc.byte_offset,
            })?
            .try_into()
            .map_err(|_| PointCloudError::FieldAccessOutOfBounds {
                byte_offset: desc.byte_offset,
            })?;
        Ok(u16::from_le_bytes(bytes))
    }

    /// Read a u8 field by pre-resolved descriptor.
    pub fn read_u8_at(&self, desc: &FieldDesc<'_>) -> Result<u8, PointCloudError> {
        self.data.get(desc.byte_offset as usize).copied().ok_or(
            PointCloudError::FieldAccessOutOfBounds {
                byte_offset: desc.byte_offset,
            },
        )
    }

    /// Read an i8 field by pre-resolved descriptor.
    pub fn read_i8_at(&self, desc: &FieldDesc<'_>) -> Result<i8, PointCloudError> {
        self.data
            .get(desc.byte_offset as usize)
            .map(|&b| b as i8)
            .ok_or(PointCloudError::FieldAccessOutOfBounds {
                byte_offset: desc.byte_offset,
            })
    }

    /// Read an i16 field by pre-resolved descriptor.
    pub fn read_i16_at(&self, desc: &FieldDesc<'_>) -> Result<i16, PointCloudError> {
        let off = desc.byte_offset as usize;
        let bytes: [u8; 2] = self
            .data
            .get(off..off + 2)
            .ok_or(PointCloudError::FieldAccessOutOfBounds {
                byte_offset: desc.byte_offset,
            })?
            .try_into()
            .map_err(|_| PointCloudError::FieldAccessOutOfBounds {
                byte_offset: desc.byte_offset,
            })?;
        Ok(i16::from_le_bytes(bytes))
    }

    /// Read an i32 field by pre-resolved descriptor.
    pub fn read_i32_at(&self, desc: &FieldDesc<'_>) -> Result<i32, PointCloudError> {
        let off = desc.byte_offset as usize;
        let bytes: [u8; 4] = self
            .data
            .get(off..off + 4)
            .ok_or(PointCloudError::FieldAccessOutOfBounds {
                byte_offset: desc.byte_offset,
            })?
            .try_into()
            .map_err(|_| PointCloudError::FieldAccessOutOfBounds {
                byte_offset: desc.byte_offset,
            })?;
        Ok(i32::from_le_bytes(bytes))
    }

    /// Read an f64 field by pre-resolved descriptor.
    pub fn read_f64_at(&self, desc: &FieldDesc<'_>) -> Result<f64, PointCloudError> {
        let off = desc.byte_offset as usize;
        let bytes: [u8; 8] = self
            .data
            .get(off..off + 8)
            .ok_or(PointCloudError::FieldAccessOutOfBounds {
                byte_offset: desc.byte_offset,
            })?
            .try_into()
            .map_err(|_| PointCloudError::FieldAccessOutOfBounds {
                byte_offset: desc.byte_offset,
            })?;
        Ok(f64::from_le_bytes(bytes))
    }
}

// ── DynPointIter ────────────────────────────────────────────────────

/// Iterator over points in a DynPointCloud.
pub struct DynPointIter<'a, 'c> {
    cloud: &'c DynPointCloud<'a>,
    index: usize,
}

impl<'a, 'c> Iterator for DynPointIter<'a, 'c> {
    type Item = DynPoint<'a, 'c>;

    fn next(&mut self) -> Option<Self::Item> {
        let point = self.cloud.point(self.index)?;
        self.index += 1;
        Some(point)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.cloud.num_points.saturating_sub(self.index);
        (remaining, Some(remaining))
    }
}

impl ExactSizeIterator for DynPointIter<'_, '_> {}

// ── PointCloud<P> ───────────────────────────────────────────────────

/// Compile-time typed zero-copy point cloud view.
///
/// `P` defines the expected point layout. Construction validates that the
/// PointCloud2 field descriptors match `P`'s expectations.
///
/// # Example
/// ```ignore
/// define_point! {
///     pub struct XyzPoint { x: f32 => 0, y: f32 => 4, z: f32 => 8 }
/// }
/// let cloud = PointCloud::<XyzPoint>::from_pointcloud2(&pcd2)?;
/// for point in cloud.iter() {
///     println!("{}, {}, {}", point.x, point.y, point.z);
/// }
/// ```
#[derive(Debug)]
pub struct PointCloud<'a, P: Point> {
    data: &'a [u8],
    point_step: usize,
    row_step: usize,
    num_points: usize,
    height: u32,
    width: u32,
    _marker: core::marker::PhantomData<P>,
}

impl<'a, P: Point> PointCloud<'a, P> {
    /// Create a typed point cloud view, validating that the PointCloud2
    /// field layout matches `P`'s expectations.
    ///
    /// Returns a zero-copy view that borrows the PointCloud2's internal
    /// data buffer. The `PointCloud2` must outlive the returned view.
    ///
    /// # Errors
    ///
    /// - [`PointCloudError::BigEndianNotSupported`] — big-endian point data.
    /// - [`PointCloudError::FieldNotFound`] — a field expected by `P` is missing.
    /// - [`PointCloudError::FieldMismatch`] — a field has wrong offset or datatype.
    /// - [`PointCloudError::InvalidLayout`] — `point_step` smaller than
    ///   `P::point_size()`, or data buffer shorter than `num_points × point_step`.
    pub fn from_pointcloud2<B: AsRef<[u8]>>(
        pc: &'a super::PointCloud2<B>,
    ) -> Result<Self, PointCloudError> {
        if pc.is_bigendian() {
            return Err(PointCloudError::BigEndianNotSupported);
        }

        Self::validate(pc)?;

        let point_step = pc.point_step() as usize;
        let num_points = pc.point_count();
        let height = pc.height() as usize;
        let width = pc.width() as usize;
        let row_step = pc.row_step() as usize;

        // Bounds-check the data buffer (same check as DynPointCloud).
        if num_points > 0 {
            let min_row_step =
                width
                    .checked_mul(point_step)
                    .ok_or(PointCloudError::InvalidLayout {
                        reason: "width × point_step overflows usize",
                    })?;
            if row_step < min_row_step {
                return Err(PointCloudError::InvalidLayout {
                    reason: "row_step smaller than width × point_step",
                });
            }

            let required_len =
                height
                    .checked_mul(row_step)
                    .ok_or(PointCloudError::InvalidLayout {
                        reason: "height × row_step overflows usize",
                    })?;
            if pc.data().len() < required_len {
                return Err(PointCloudError::InvalidLayout {
                    reason: "data buffer shorter than height × row_step",
                });
            }
        }

        Ok(PointCloud {
            data: pc.data(),
            point_step,
            row_step,
            num_points,
            height: pc.height(),
            width: pc.width(),
            _marker: core::marker::PhantomData,
        })
    }

    /// Validate that the PointCloud2 field layout matches `P`'s expectations.
    pub fn validate<B: AsRef<[u8]>>(pc: &super::PointCloud2<B>) -> Result<(), PointCloudError> {
        let expected = P::expected_fields();

        // Scan fields without allocating — for each expected field, iterate
        // the PointCloud2 field descriptors to find a match.
        for exp in expected {
            let mut found = None;
            for f in pc.fields_iter() {
                if f.name == exp.name {
                    found = Some((f.offset, f.datatype));
                    break;
                }
            }
            match found {
                None => {
                    return Err(PointCloudError::FieldNotFound { name: exp.name });
                }
                Some((offset, datatype)) => {
                    if offset != exp.byte_offset {
                        return Err(PointCloudError::FieldMismatch {
                            name: exp.name,
                            reason: "byte offset mismatch",
                        });
                    }
                    let actual_type = PointFieldType::from_datatype(datatype);
                    if actual_type != Some(exp.field_type) {
                        return Err(PointCloudError::FieldMismatch {
                            name: exp.name,
                            reason: "datatype mismatch",
                        });
                    }
                }
            }
        }

        let point_step = pc.point_step();
        if point_step < P::point_size() {
            return Err(PointCloudError::InvalidLayout {
                reason: "point_step smaller than Point type size",
            });
        }

        Ok(())
    }

    /// Number of points.
    pub fn len(&self) -> usize {
        self.num_points
    }

    /// Whether the cloud is empty.
    pub fn is_empty(&self) -> bool {
        self.num_points == 0
    }

    /// Height of the cloud.
    pub fn height(&self) -> u32 {
        self.height
    }

    /// Width of the cloud.
    pub fn width(&self) -> u32 {
        self.width
    }

    /// Compute the byte offset of the i-th point, correctly handling
    /// row padding in organized clouds.
    #[inline]
    fn point_offset(&self, i: usize) -> usize {
        if self.row_step == (self.width as usize) * self.point_step {
            i * self.point_step
        } else {
            let w = self.width as usize;
            (i / w) * self.row_step + (i % w) * self.point_step
        }
    }

    /// Read a single point by linear index.
    pub fn get(&self, index: usize) -> Option<P> {
        if index >= self.num_points {
            return None;
        }
        let base = self.point_offset(index);
        if base + P::point_size() as usize > self.data.len() {
            return None;
        }
        Some(P::read_from(self.data, base))
    }

    /// Read a point by (row, col) for organized clouds.
    ///
    /// Uses `row_step` to correctly handle row padding in organized clouds.
    pub fn get_at(&self, row: u32, col: u32) -> Option<P> {
        if row >= self.height || col >= self.width {
            return None;
        }
        let base = (row as usize) * self.row_step + (col as usize) * self.point_step;
        if base + P::point_size() as usize > self.data.len() {
            return None;
        }
        Some(P::read_from(self.data, base))
    }

    /// Iterate over all points.
    pub fn iter(&self) -> PointIter<'a, P> {
        PointIter {
            data: self.data,
            point_step: self.point_step,
            row_step: self.row_step,
            width: self.width as usize,
            num_points: self.num_points,
            index: 0,
            _marker: core::marker::PhantomData,
        }
    }
}

/// Iterator over points in a typed PointCloud.
pub struct PointIter<'a, P: Point> {
    data: &'a [u8],
    point_step: usize,
    row_step: usize,
    width: usize,
    num_points: usize,
    index: usize,
    _marker: core::marker::PhantomData<P>,
}

impl<P: Point> PointIter<'_, P> {
    #[inline]
    fn point_offset(&self, i: usize) -> usize {
        if self.row_step == self.width * self.point_step {
            i * self.point_step
        } else {
            (i / self.width) * self.row_step + (i % self.width) * self.point_step
        }
    }
}

impl<P: Point> Iterator for PointIter<'_, P> {
    type Item = P;

    fn next(&mut self) -> Option<P> {
        if self.index >= self.num_points {
            return None;
        }
        let base = self.point_offset(self.index);
        if base + P::point_size() as usize > self.data.len() {
            return None;
        }
        self.index += 1;
        Some(P::read_from(self.data, base))
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.num_points.saturating_sub(self.index);
        (remaining, Some(remaining))
    }
}

impl<P: Point> ExactSizeIterator for PointIter<'_, P> {}

// ── Tests ───────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin_interfaces::Time;
    use crate::sensor_msgs::{PointCloud2, PointFieldView};

    /// Build a PointCloud2 with known xyz + intensity data.
    fn make_test_cloud() -> PointCloud2<Vec<u8>> {
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "intensity",
                offset: 12,
                datatype: 7,
                count: 1,
            },
        ];
        let point_step = 16u32;
        let num_points = 4u32;
        let mut data = vec![0u8; (point_step * num_points) as usize];

        // Write 4 test points: (1,2,3,10), (4,5,6,20), (7,8,9,30), (10,11,12,40)
        for i in 0..4u32 {
            let base = (i * point_step) as usize;
            let x = (i * 3 + 1) as f32;
            let y = (i * 3 + 2) as f32;
            let z = (i * 3 + 3) as f32;
            let intensity = ((i + 1) * 10) as f32;
            data[base..base + 4].copy_from_slice(&x.to_le_bytes());
            data[base + 4..base + 8].copy_from_slice(&y.to_le_bytes());
            data[base + 8..base + 12].copy_from_slice(&z.to_le_bytes());
            data[base + 12..base + 16].copy_from_slice(&intensity.to_le_bytes());
        }

        PointCloud2::new(
            Time::new(100, 0),
            "lidar",
            1,
            num_points,
            &fields,
            false,
            point_step,
            point_step * num_points,
            &data,
            true,
        )
        .unwrap()
    }

    // ── DynPointCloud tests ─────────────────────────────────────────

    #[test]
    fn dyn_cloud_from_pointcloud2() {
        let pc = make_test_cloud();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();

        assert_eq!(cloud.len(), 4);
        assert_eq!(cloud.field_count(), 4);
        assert!(cloud.field("x").is_some());
        assert!(cloud.field("intensity").is_some());
        assert!(cloud.field("nonexistent").is_none());
    }

    #[test]
    fn dyn_cloud_point_access() {
        let pc = make_test_cloud();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();

        let p0 = cloud.point(0).unwrap();
        assert_eq!(p0.read_f32("x"), Some(1.0));
        assert_eq!(p0.read_f32("y"), Some(2.0));
        assert_eq!(p0.read_f32("z"), Some(3.0));
        assert_eq!(p0.read_f32("intensity"), Some(10.0));

        let p3 = cloud.point(3).unwrap();
        assert_eq!(p3.read_f32("x"), Some(10.0));
        assert_eq!(p3.read_f32("z"), Some(12.0));

        // Out of bounds
        assert!(cloud.point(4).is_none());
    }

    #[test]
    fn dyn_cloud_descriptor_access() {
        let pc = make_test_cloud();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();

        // Pre-resolve field once, use for all points
        let x_desc = cloud.field("x").unwrap();
        let z_desc = cloud.field("z").unwrap();

        for (i, point) in cloud.iter().enumerate() {
            let expected_x = (i as f32) * 3.0 + 1.0;
            let expected_z = (i as f32) * 3.0 + 3.0;
            assert_eq!(point.read_f32_at(x_desc).unwrap(), expected_x);
            assert_eq!(point.read_f32_at(z_desc).unwrap(), expected_z);
        }
    }

    #[test]
    fn dyn_cloud_gather() {
        let pc = make_test_cloud();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();

        let xs = cloud.gather_f32("x").unwrap();
        assert_eq!(xs, vec![1.0, 4.0, 7.0, 10.0]);

        let zs = cloud.gather_f32("z").unwrap();
        assert_eq!(zs, vec![3.0, 6.0, 9.0, 12.0]);

        // Wrong type returns None
        assert!(cloud.gather_u32("x").is_none());
        // Missing field returns None
        assert!(cloud.gather_f32("nonexistent").is_none());
    }

    #[test]
    fn dyn_cloud_iterator_count() {
        let pc = make_test_cloud();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();

        assert_eq!(cloud.iter().count(), 4);
        assert_eq!(cloud.iter().len(), 4);
    }

    #[test]
    fn dyn_cloud_organized() {
        // 2×2 organized cloud
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ];
        let point_step = 12u32;
        let mut data = vec![0u8; 48]; // 4 points × 12 bytes
        for i in 0..4u32 {
            let base = (i * point_step) as usize;
            let val = (i + 1) as f32;
            data[base..base + 4].copy_from_slice(&val.to_le_bytes());
        }
        let pc = PointCloud2::new(
            Time::new(0, 0),
            "cam",
            2,
            2,
            &fields,
            false,
            point_step,
            24,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();

        assert_eq!(cloud.height(), 2);
        assert_eq!(cloud.width(), 2);
        assert_eq!(cloud.point_at(0, 0).unwrap().read_f32("x"), Some(1.0));
        assert_eq!(cloud.point_at(0, 1).unwrap().read_f32("x"), Some(2.0));
        assert_eq!(cloud.point_at(1, 0).unwrap().read_f32("x"), Some(3.0));
        assert_eq!(cloud.point_at(1, 1).unwrap().read_f32("x"), Some(4.0));
        assert!(cloud.point_at(2, 0).is_none());
    }

    #[test]
    fn dyn_cloud_rejects_bigendian() {
        let fields = [PointFieldView {
            name: "x",
            offset: 0,
            datatype: 7,
            count: 1,
        }];
        let data = vec![0u8; 4];
        let pc =
            PointCloud2::new(Time::new(0, 0), "f", 1, 1, &fields, true, 4, 4, &data, true).unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let err = DynPointCloud::from_pointcloud2(&decoded).unwrap_err();
        assert!(matches!(err, PointCloudError::BigEndianNotSupported));
    }

    #[test]
    fn dyn_cloud_mixed_types() {
        // Cloud with f32 xyz + u16 class + u8 flags
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "class",
                offset: 12,
                datatype: 4,
                count: 1,
            },
            PointFieldView {
                name: "flags",
                offset: 14,
                datatype: 2,
                count: 1,
            },
        ];
        let point_step = 16u32;
        let mut data = vec![0u8; 16];
        data[0..4].copy_from_slice(&1.0f32.to_le_bytes());
        data[4..8].copy_from_slice(&2.0f32.to_le_bytes());
        data[8..12].copy_from_slice(&3.0f32.to_le_bytes());
        data[12..14].copy_from_slice(&42u16.to_le_bytes());
        data[14] = 0xFF;

        let pc = PointCloud2::new(
            Time::new(0, 0),
            "f",
            1,
            1,
            &fields,
            false,
            point_step,
            16,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();
        let p = cloud.point(0).unwrap();

        assert_eq!(p.read_f32("x"), Some(1.0));
        assert_eq!(p.read_u16("class"), Some(42));
        assert_eq!(p.read_u8("flags"), Some(0xFF));
        // Wrong type access
        assert_eq!(p.read_f32("class"), None);
        assert_eq!(p.read_u32("flags"), None);

        // Gather typed columns
        assert_eq!(cloud.gather_u16("class"), Some(vec![42]));
        assert_eq!(cloud.gather_u8("flags"), Some(vec![0xFF]));
    }

    #[test]
    fn dyn_cloud_empty() {
        let fields = [PointFieldView {
            name: "x",
            offset: 0,
            datatype: 7,
            count: 1,
        }];
        let pc =
            PointCloud2::new(Time::new(0, 0), "f", 0, 0, &fields, false, 4, 0, &[], true).unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();

        assert!(cloud.is_empty());
        assert_eq!(cloud.len(), 0);
        assert_eq!(cloud.iter().count(), 0);
        assert!(cloud.gather_f32("x").unwrap().is_empty());
    }

    // ── Static PointCloud tests ─────────────────────────────────────

    define_point! {
        struct TestXyzPoint {
            x: f32 => 0,
            y: f32 => 4,
            z: f32 => 8,
        }
    }

    define_point! {
        struct TestXyzClassPoint {
            x: f32 => 0,
            y: f32 => 4,
            z: f32 => 8,
            class_id: u16 => 12,
            instance_id: u16 => 14,
        }
    }

    #[test]
    fn static_cloud_xyz() {
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ];
        let point_step = 12u32;
        let mut data = vec![0u8; 36];
        for i in 0..3u32 {
            let base = (i * point_step) as usize;
            data[base..base + 4].copy_from_slice(&(i as f32 + 1.0).to_le_bytes());
            data[base + 4..base + 8].copy_from_slice(&(i as f32 + 10.0).to_le_bytes());
            data[base + 8..base + 12].copy_from_slice(&(i as f32 + 100.0).to_le_bytes());
        }

        let pc = PointCloud2::new(
            Time::new(0, 0),
            "lidar",
            1,
            3,
            &fields,
            false,
            point_step,
            36,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = PointCloud::<TestXyzPoint>::from_pointcloud2(&decoded).unwrap();

        assert_eq!(cloud.len(), 3);
        let p0 = cloud.get(0).unwrap();
        assert_eq!(p0.x, 1.0);
        assert_eq!(p0.y, 10.0);
        assert_eq!(p0.z, 100.0);

        let p2 = cloud.get(2).unwrap();
        assert_eq!(p2.x, 3.0);
        assert_eq!(p2.y, 12.0);
        assert_eq!(p2.z, 102.0);
    }

    #[test]
    fn static_cloud_mixed_types() {
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "class_id",
                offset: 12,
                datatype: 4,
                count: 1,
            },
            PointFieldView {
                name: "instance_id",
                offset: 14,
                datatype: 4,
                count: 1,
            },
        ];
        let point_step = 16u32;
        let mut data = vec![0u8; 16];
        data[0..4].copy_from_slice(&1.5f32.to_le_bytes());
        data[4..8].copy_from_slice(&2.5f32.to_le_bytes());
        data[8..12].copy_from_slice(&3.5f32.to_le_bytes());
        data[12..14].copy_from_slice(&7u16.to_le_bytes());
        data[14..16].copy_from_slice(&42u16.to_le_bytes());

        let pc = PointCloud2::new(
            Time::new(0, 0),
            "f",
            1,
            1,
            &fields,
            false,
            point_step,
            16,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = PointCloud::<TestXyzClassPoint>::from_pointcloud2(&decoded).unwrap();

        let p = cloud.get(0).unwrap();
        assert_eq!(p.x, 1.5);
        assert_eq!(p.y, 2.5);
        assert_eq!(p.z, 3.5);
        assert_eq!(p.class_id, 7);
        assert_eq!(p.instance_id, 42);
    }

    #[test]
    fn static_cloud_validation_field_missing() {
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
        ];
        let data = vec![0u8; 8];
        let pc = PointCloud2::new(
            Time::new(0, 0),
            "f",
            1,
            1,
            &fields,
            false,
            8,
            8,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let err = PointCloud::<TestXyzPoint>::from_pointcloud2(&decoded).unwrap_err();
        assert!(matches!(err, PointCloudError::FieldNotFound { name: "z" }));
    }

    #[test]
    fn static_cloud_validation_type_mismatch() {
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 6, // u32 not f32
                count: 1,
            },
        ];
        let data = vec![0u8; 12];
        let pc = PointCloud2::new(
            Time::new(0, 0),
            "f",
            1,
            1,
            &fields,
            false,
            12,
            12,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let err = PointCloud::<TestXyzPoint>::from_pointcloud2(&decoded).unwrap_err();
        assert!(matches!(
            err,
            PointCloudError::FieldMismatch { name: "z", .. }
        ));
    }

    #[test]
    fn static_cloud_extra_fields_ok() {
        // Cloud has more fields than the Point type — that's fine,
        // we only check the fields the Point declares.
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "intensity",
                offset: 12,
                datatype: 7,
                count: 1,
            },
        ];
        let data = vec![0u8; 16];
        let pc = PointCloud2::new(
            Time::new(0, 0),
            "f",
            1,
            1,
            &fields,
            false,
            16,
            16,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = PointCloud::<TestXyzPoint>::from_pointcloud2(&decoded).unwrap();
        assert_eq!(cloud.len(), 1);
    }

    #[test]
    fn static_cloud_iterator() {
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ];
        let point_step = 12u32;
        let n = 100u32;
        let mut data = vec![0u8; (point_step * n) as usize];
        for i in 0..n {
            let base = (i * point_step) as usize;
            data[base..base + 4].copy_from_slice(&(i as f32).to_le_bytes());
            data[base + 4..base + 8].copy_from_slice(&(i as f32 * 2.0).to_le_bytes());
            data[base + 8..base + 12].copy_from_slice(&(i as f32 * 3.0).to_le_bytes());
        }

        let pc = PointCloud2::new(
            Time::new(0, 0),
            "f",
            1,
            n,
            &fields,
            false,
            point_step,
            point_step * n,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = PointCloud::<TestXyzPoint>::from_pointcloud2(&decoded).unwrap();

        assert_eq!(cloud.iter().len(), 100);
        let points: Vec<_> = cloud.iter().collect();
        assert_eq!(
            points[0],
            TestXyzPoint {
                x: 0.0,
                y: 0.0,
                z: 0.0
            }
        );
        assert_eq!(
            points[99],
            TestXyzPoint {
                x: 99.0,
                y: 198.0,
                z: 297.0
            }
        );
    }

    // ── Convenience method tests ────────────────────────────────────

    #[test]
    fn convenience_methods() {
        let pc = make_test_cloud();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();

        // Dynamic
        let dyn_cloud = decoded.as_dyn_cloud().unwrap();
        assert_eq!(dyn_cloud.len(), 4);

        // Static (using TestXyzPoint — ignores intensity field)
        let typed_cloud = decoded.as_typed_cloud::<TestXyzPoint>().unwrap();
        assert_eq!(typed_cloud.len(), 4);
        assert_eq!(typed_cloud.get(0).unwrap().x, 1.0);
    }

    // ── PointFieldType unit tests ───────────────────────────────────

    #[test]
    fn point_field_type_from_datatype() {
        assert_eq!(PointFieldType::from_datatype(1), Some(PointFieldType::Int8));
        assert_eq!(
            PointFieldType::from_datatype(2),
            Some(PointFieldType::Uint8)
        );
        assert_eq!(
            PointFieldType::from_datatype(3),
            Some(PointFieldType::Int16)
        );
        assert_eq!(
            PointFieldType::from_datatype(4),
            Some(PointFieldType::Uint16)
        );
        assert_eq!(
            PointFieldType::from_datatype(5),
            Some(PointFieldType::Int32)
        );
        assert_eq!(
            PointFieldType::from_datatype(6),
            Some(PointFieldType::Uint32)
        );
        assert_eq!(
            PointFieldType::from_datatype(7),
            Some(PointFieldType::Float32)
        );
        assert_eq!(
            PointFieldType::from_datatype(8),
            Some(PointFieldType::Float64)
        );
        // Invalid
        assert_eq!(PointFieldType::from_datatype(0), None);
        assert_eq!(PointFieldType::from_datatype(9), None);
        assert_eq!(PointFieldType::from_datatype(255), None);
    }

    #[test]
    fn point_field_type_size_bytes() {
        assert_eq!(PointFieldType::Int8.size_bytes(), 1);
        assert_eq!(PointFieldType::Uint8.size_bytes(), 1);
        assert_eq!(PointFieldType::Int16.size_bytes(), 2);
        assert_eq!(PointFieldType::Uint16.size_bytes(), 2);
        assert_eq!(PointFieldType::Int32.size_bytes(), 4);
        assert_eq!(PointFieldType::Uint32.size_bytes(), 4);
        assert_eq!(PointFieldType::Float32.size_bytes(), 4);
        assert_eq!(PointFieldType::Float64.size_bytes(), 8);
    }

    #[test]
    fn field_desc_from_view_unknown_datatype() {
        let view = PointFieldView {
            name: "bad",
            offset: 0,
            datatype: 99,
            count: 1,
        };
        assert!(FieldDesc::from_view(&view).is_none());
    }

    // ── Signed and f64 type tests ───────────────────────────────────

    #[test]
    fn dyn_cloud_signed_and_f64_types() {
        let fields = [
            PointFieldView {
                name: "i8_field",
                offset: 0,
                datatype: 1,
                count: 1,
            }, // Int8
            PointFieldView {
                name: "u8_field",
                offset: 1,
                datatype: 2,
                count: 1,
            }, // Uint8
            PointFieldView {
                name: "i16_field",
                offset: 2,
                datatype: 3,
                count: 1,
            }, // Int16
            PointFieldView {
                name: "u16_field",
                offset: 4,
                datatype: 4,
                count: 1,
            }, // Uint16
            PointFieldView {
                name: "i32_field",
                offset: 6,
                datatype: 5,
                count: 1,
            }, // Int32
            PointFieldView {
                name: "u32_field",
                offset: 10,
                datatype: 6,
                count: 1,
            }, // Uint32
            PointFieldView {
                name: "f32_field",
                offset: 14,
                datatype: 7,
                count: 1,
            }, // Float32
            PointFieldView {
                name: "f64_field",
                offset: 18,
                datatype: 8,
                count: 1,
            }, // Float64
        ];
        let point_step = 26u32; // 1+1+2+2+4+4+4+8
        let mut data = vec![0u8; point_step as usize];
        data[0] = 0xFE_u8; // i8 = -2
        data[1] = 42; // u8 = 42
        data[2..4].copy_from_slice(&(-300i16).to_le_bytes()); // i16
        data[4..6].copy_from_slice(&1000u16.to_le_bytes()); // u16
        data[6..10].copy_from_slice(&(-100_000i32).to_le_bytes()); // i32
        data[10..14].copy_from_slice(&3_000_000u32.to_le_bytes()); // u32
        data[14..18].copy_from_slice(&std::f32::consts::PI.to_le_bytes()); // f32
        data[18..26].copy_from_slice(&std::f64::consts::E.to_le_bytes()); // f64

        let pc = PointCloud2::new(
            Time::new(0, 0),
            "test",
            1,
            1,
            &fields,
            false,
            point_step,
            point_step,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();

        assert_eq!(cloud.field_count(), 8);

        // Verify all field types are correctly resolved
        let i8_desc = cloud.field("i8_field").unwrap();
        assert_eq!(i8_desc.field_type, PointFieldType::Int8);
        let f64_desc = cloud.field("f64_field").unwrap();
        assert_eq!(f64_desc.field_type, PointFieldType::Float64);

        // Verify gather for u8 and u16
        assert_eq!(cloud.gather_u8("u8_field"), Some(vec![42]));
        assert_eq!(cloud.gather_u16("u16_field"), Some(vec![1000]));
        assert_eq!(cloud.gather_u32("u32_field"), Some(vec![3_000_000]));

        // Verify point-level access
        let p = cloud.point(0).unwrap();
        assert_eq!(p.read_u8("u8_field"), Some(42));
        assert_eq!(p.read_u16("u16_field"), Some(1000));
        assert_eq!(p.read_u32("u32_field"), Some(3_000_000));
        assert_eq!(p.read_f32("f32_field"), Some(std::f32::consts::PI));

        // Verify descriptor-based access
        let u16_desc = cloud.field("u16_field").unwrap();
        assert_eq!(p.read_u16_at(u16_desc).unwrap(), 1000);
        let u8_desc = cloud.field("u8_field").unwrap();
        assert_eq!(p.read_u8_at(u8_desc).unwrap(), 42);
    }

    // ── define_point! macro metadata tests ──────────────────────────

    #[test]
    fn define_point_metadata() {
        assert_eq!(TestXyzPoint::FIELD_COUNT, 3);
        assert_eq!(TestXyzPoint::point_size(), 12);

        let fields = TestXyzPoint::expected_fields();
        assert_eq!(fields.len(), 3);
        assert_eq!(fields[0].name, "x");
        assert_eq!(fields[0].byte_offset, 0);
        assert_eq!(fields[0].field_type, PointFieldType::Float32);
        assert_eq!(fields[1].name, "y");
        assert_eq!(fields[2].name, "z");
        assert_eq!(fields[2].byte_offset, 8);
    }

    #[test]
    fn define_point_mixed_metadata() {
        assert_eq!(TestXyzClassPoint::FIELD_COUNT, 5);
        assert_eq!(TestXyzClassPoint::point_size(), 16); // max(14+2) = 16

        let fields = TestXyzClassPoint::expected_fields();
        assert_eq!(fields[3].name, "class_id");
        assert_eq!(fields[3].field_type, PointFieldType::Uint16);
        assert_eq!(fields[3].byte_offset, 12);
        assert_eq!(fields[4].name, "instance_id");
        assert_eq!(fields[4].byte_offset, 14);
    }

    #[test]
    fn define_point_read_from() {
        let mut data = vec![0u8; 12];
        data[0..4].copy_from_slice(&1.5f32.to_le_bytes());
        data[4..8].copy_from_slice(&2.5f32.to_le_bytes());
        data[8..12].copy_from_slice(&3.5f32.to_le_bytes());

        let p = TestXyzPoint::read_from(&data, 0);
        assert_eq!(p.x, 1.5);
        assert_eq!(p.y, 2.5);
        assert_eq!(p.z, 3.5);
    }

    // ── Static cloud validation edge cases ──────────────────────────

    #[test]
    fn static_cloud_validation_offset_mismatch() {
        // Field "y" at wrong offset (8 instead of 4)
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 8, // wrong — TestXyzPoint expects 4
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 12,
                datatype: 7,
                count: 1,
            },
        ];
        let data = vec![0u8; 16];
        let pc = PointCloud2::new(
            Time::new(0, 0),
            "f",
            1,
            1,
            &fields,
            false,
            16,
            16,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let err = PointCloud::<TestXyzPoint>::from_pointcloud2(&decoded).unwrap_err();
        assert!(matches!(
            err,
            PointCloudError::FieldMismatch {
                name: "y",
                reason: "byte offset mismatch"
            }
        ));
    }

    #[test]
    fn static_cloud_point_step_too_small() {
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ];
        // point_step=8 < TestXyzPoint::point_size()=12
        let data = vec![0u8; 8];
        let pc = PointCloud2::new(
            Time::new(0, 0),
            "f",
            1,
            1,
            &fields,
            false,
            8,
            8,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let err = PointCloud::<TestXyzPoint>::from_pointcloud2(&decoded).unwrap_err();
        assert!(matches!(err, PointCloudError::InvalidLayout { .. }));
    }

    // ── Realistic LiDAR layout with padding ─────────────────────────

    define_point! {
        /// Typical Ouster OS1 point layout (subset).
        struct TestOusterPoint {
            x: f32 => 0,
            y: f32 => 4,
            z: f32 => 8,
        }
    }

    #[test]
    fn static_cloud_with_padding() {
        // Simulates a sensor with 32-byte point_step but only xyz used
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "intensity",
                offset: 12,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "ring",
                offset: 16,
                datatype: 4,
                count: 1,
            },
            PointFieldView {
                name: "timestamp",
                offset: 24,
                datatype: 8,
                count: 1,
            },
        ];
        let point_step = 32u32;
        let n = 3u32;
        let mut data = vec![0u8; (point_step * n) as usize];
        for i in 0..n {
            let base = (i * point_step) as usize;
            data[base..base + 4].copy_from_slice(&(i as f32 * 10.0).to_le_bytes());
            data[base + 4..base + 8].copy_from_slice(&(i as f32 * 20.0).to_le_bytes());
            data[base + 8..base + 12].copy_from_slice(&(i as f32 * 30.0).to_le_bytes());
        }

        let pc = PointCloud2::new(
            Time::new(0, 0),
            "os1",
            1,
            n,
            &fields,
            false,
            point_step,
            point_step * n,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();

        // Static typed — only reads x,y,z, skips intensity/ring/timestamp
        let cloud = PointCloud::<TestOusterPoint>::from_pointcloud2(&decoded).unwrap();
        assert_eq!(cloud.len(), 3);
        let p1 = cloud.get(1).unwrap();
        assert_eq!(p1.x, 10.0);
        assert_eq!(p1.y, 20.0);
        assert_eq!(p1.z, 30.0);

        // Dynamic — can also read intensity and ring
        let dyn_cloud = decoded.as_dyn_cloud().unwrap();
        assert_eq!(dyn_cloud.field_count(), 6);
        assert_eq!(dyn_cloud.point_step(), 32);

        // Verify stride is respected in gather
        let xs = dyn_cloud.gather_f32("x").unwrap();
        assert_eq!(xs, vec![0.0, 10.0, 20.0]);
    }

    // ── DynPointCloud error paths ───────────────────────────────────

    #[test]
    fn dyn_cloud_unknown_datatype_rejected() {
        // Manually craft a PointCloud2 with an unknown datatype
        // We can't do this through the normal API, so test FieldDesc directly
        let view = PointFieldView {
            name: "bad",
            offset: 0,
            datatype: 99,
            count: 1,
        };
        assert!(FieldDesc::from_view(&view).is_none());
    }

    #[test]
    fn dyn_cloud_gather_wrong_type_returns_none() {
        let pc = make_test_cloud();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();

        // "x" is f32, not u32/u16/u8
        assert!(cloud.gather_u32("x").is_none());
        assert!(cloud.gather_u16("x").is_none());
        assert!(cloud.gather_u8("x").is_none());

        // Non-existent field
        assert!(cloud.gather_f32("nonexistent").is_none());
        assert!(cloud.gather_u32("nonexistent").is_none());
        assert!(cloud.gather_u16("nonexistent").is_none());
        assert!(cloud.gather_u8("nonexistent").is_none());
    }

    #[test]
    fn dyn_point_wrong_type_returns_none() {
        let pc = make_test_cloud();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();
        let p = cloud.point(0).unwrap();

        // "x" is f32 — all non-f32 reads should return None
        assert!(p.read_u32("x").is_none());
        assert!(p.read_u16("x").is_none());
        assert!(p.read_u8("x").is_none());

        // Non-existent field
        assert!(p.read_f32("nope").is_none());
        assert!(p.read_u32("nope").is_none());
        assert!(p.read_u16("nope").is_none());
        assert!(p.read_u8("nope").is_none());
    }

    // ── PointCloud2 error display ───────────────────────────────────

    #[test]
    fn point_cloud_error_display() {
        let e = PointCloudError::FieldNotFound { name: "x" };
        assert_eq!(format!("{e}"), "field not found: x");

        let e = PointCloudError::FieldMismatch {
            name: "y",
            reason: "byte offset mismatch",
        };
        assert_eq!(
            format!("{e}"),
            "field mismatch for 'y': byte offset mismatch"
        );

        let e = PointCloudError::TooManyFields { found: 20 };
        assert_eq!(format!("{e}"), "too many fields: 20 (max 16)");

        let e = PointCloudError::UnknownDatatype {
            field_name: "bad".into(),
            datatype: 99,
        };
        assert_eq!(format!("{e}"), "unknown datatype 99 for field 'bad'");

        let e = PointCloudError::BigEndianNotSupported;
        assert_eq!(format!("{e}"), "big-endian point data not supported");

        let e = PointCloudError::InvalidLayout {
            reason: "point_step is zero",
        };
        assert_eq!(format!("{e}"), "invalid layout: point_step is zero");
    }

    // ── Static cloud organized access ───────────────────────────────

    #[test]
    fn static_cloud_organized_access() {
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ];
        let point_step = 12u32;
        // 3×2 organized cloud
        let n = 6u32;
        let mut data = vec![0u8; (point_step * n) as usize];
        for i in 0..n {
            let base = (i * point_step) as usize;
            data[base..base + 4].copy_from_slice(&(i as f32).to_le_bytes());
        }

        let pc = PointCloud2::new(
            Time::new(0, 0),
            "depth",
            3,
            2,
            &fields,
            false,
            point_step,
            point_step * 2,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = PointCloud::<TestXyzPoint>::from_pointcloud2(&decoded).unwrap();

        assert_eq!(cloud.height(), 3);
        assert_eq!(cloud.width(), 2);

        // (row, col) → linear index = row * width + col
        assert_eq!(cloud.get_at(0, 0).unwrap().x, 0.0);
        assert_eq!(cloud.get_at(0, 1).unwrap().x, 1.0);
        assert_eq!(cloud.get_at(1, 0).unwrap().x, 2.0);
        assert_eq!(cloud.get_at(1, 1).unwrap().x, 3.0);
        assert_eq!(cloud.get_at(2, 0).unwrap().x, 4.0);
        assert_eq!(cloud.get_at(2, 1).unwrap().x, 5.0);

        // Out of bounds
        assert!(cloud.get_at(3, 0).is_none());
        assert!(cloud.get_at(0, 2).is_none());
    }

    #[test]
    fn static_cloud_organized_with_row_padding() {
        // Organized cloud where row_step > width * point_step (row padding).
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ];
        let point_step = 12u32;
        let width = 2u32;
        let height = 3u32;
        // 8 bytes of padding per row (row_step = 32 > width * point_step = 24).
        let row_step = 32u32;
        let total_bytes = (row_step * height) as usize;
        let mut data = vec![0xFFu8; total_bytes]; // fill with 0xFF to catch stale reads

        for row in 0..height {
            for col in 0..width {
                let offset = (row * row_step + col * point_step) as usize;
                let val = (row * width + col) as f32;
                data[offset..offset + 4].copy_from_slice(&val.to_le_bytes());
            }
        }

        let pc = PointCloud2::new(
            Time::new(0, 0),
            "padded",
            height,
            width,
            &fields,
            false,
            point_step,
            row_step,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();

        // Test both DynPointCloud and PointCloud<P>
        let dyn_cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();
        let typed_cloud = PointCloud::<TestXyzPoint>::from_pointcloud2(&decoded).unwrap();

        for row in 0..height {
            for col in 0..width {
                let expected = (row * width + col) as f32;
                let dp = dyn_cloud.point_at(row, col).unwrap();
                assert_eq!(dp.read_f32("x"), Some(expected), "dyn row={row} col={col}");
                let tp = typed_cloud.get_at(row, col).unwrap();
                assert_eq!(tp.x, expected, "typed row={row} col={col}");
            }
        }
    }

    // ── PointScalar trait coverage ──────────────────────────────────

    define_point! {
        /// Point with all supported scalar types.
        struct TestAllTypesPoint {
            a: i8 => 0,
            b: u8 => 1,
            c: i16 => 2,
            d: u16 => 4,
            e: i32 => 6,
            f: u32 => 10,
            g: f32 => 14,
            h: f64 => 18,
        }
    }

    #[test]
    fn static_cloud_all_scalar_types() {
        use crate::sensor_msgs::pointcloud::Point;

        assert_eq!(TestAllTypesPoint::FIELD_COUNT, 8);
        assert_eq!(TestAllTypesPoint::point_size(), 26); // 18 + 8

        let fields = TestAllTypesPoint::expected_fields();
        assert_eq!(fields[0].field_type, PointFieldType::Int8);
        assert_eq!(fields[1].field_type, PointFieldType::Uint8);
        assert_eq!(fields[2].field_type, PointFieldType::Int16);
        assert_eq!(fields[3].field_type, PointFieldType::Uint16);
        assert_eq!(fields[4].field_type, PointFieldType::Int32);
        assert_eq!(fields[5].field_type, PointFieldType::Uint32);
        assert_eq!(fields[6].field_type, PointFieldType::Float32);
        assert_eq!(fields[7].field_type, PointFieldType::Float64);

        // Read from raw bytes
        let mut data = vec![0u8; 26];
        data[0] = 0xFE; // i8 = -2
        data[1] = 200; // u8
        data[2..4].copy_from_slice(&(-500i16).to_le_bytes());
        data[4..6].copy_from_slice(&60000u16.to_le_bytes());
        data[6..10].copy_from_slice(&(-1_000_000i32).to_le_bytes());
        data[10..14].copy_from_slice(&4_000_000u32.to_le_bytes());
        data[14..18].copy_from_slice(&std::f32::consts::E.to_le_bytes());
        data[18..26].copy_from_slice(&std::f64::consts::PI.to_le_bytes());

        let p = TestAllTypesPoint::read_from(&data, 0);
        assert_eq!(p.a, -2);
        assert_eq!(p.b, 200);
        assert_eq!(p.c, -500);
        assert_eq!(p.d, 60000);
        assert_eq!(p.e, -1_000_000);
        assert_eq!(p.f, 4_000_000);
        assert_eq!(p.g, std::f32::consts::E);
        assert_eq!(p.h, std::f64::consts::PI);
    }

    // ── Coverage tests for signed/f64 accessors (PR #12 Comment 4) ──

    /// Helper: build the 8-field all-types cloud CDR used by multiple tests.
    fn make_all_types_cloud_cdr() -> Vec<u8> {
        let fields = [
            PointFieldView {
                name: "i8_field",
                offset: 0,
                datatype: 1,
                count: 1,
            },
            PointFieldView {
                name: "u8_field",
                offset: 1,
                datatype: 2,
                count: 1,
            },
            PointFieldView {
                name: "i16_field",
                offset: 2,
                datatype: 3,
                count: 1,
            },
            PointFieldView {
                name: "u16_field",
                offset: 4,
                datatype: 4,
                count: 1,
            },
            PointFieldView {
                name: "i32_field",
                offset: 6,
                datatype: 5,
                count: 1,
            },
            PointFieldView {
                name: "u32_field",
                offset: 10,
                datatype: 6,
                count: 1,
            },
            PointFieldView {
                name: "f32_field",
                offset: 14,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "f64_field",
                offset: 18,
                datatype: 8,
                count: 1,
            },
        ];
        let point_step = 26u32;
        let mut data = vec![0u8; point_step as usize];
        data[0] = 0xFE_u8; // i8 = -2
        data[1] = 42;
        data[2..4].copy_from_slice(&(-300i16).to_le_bytes());
        data[4..6].copy_from_slice(&1000u16.to_le_bytes());
        data[6..10].copy_from_slice(&(-100_000i32).to_le_bytes());
        data[10..14].copy_from_slice(&3_000_000u32.to_le_bytes());
        data[14..18].copy_from_slice(&std::f32::consts::PI.to_le_bytes());
        data[18..26].copy_from_slice(&std::f64::consts::E.to_le_bytes());

        PointCloud2::new(
            Time::new(0, 0),
            "test",
            1,
            1,
            &fields,
            false,
            point_step,
            point_step,
            &data,
            true,
        )
        .unwrap()
        .to_cdr()
    }

    #[test]
    fn dyn_point_signed_and_f64_by_name() {
        let cdr = make_all_types_cloud_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();
        let p = cloud.point(0).unwrap();

        assert_eq!(p.read_i8("i8_field"), Some(-2));
        assert_eq!(p.read_i16("i16_field"), Some(-300));
        assert_eq!(p.read_i32("i32_field"), Some(-100_000));
        assert_eq!(p.read_f64("f64_field"), Some(std::f64::consts::E));
    }

    #[test]
    fn dyn_point_signed_and_f64_by_descriptor() {
        let cdr = make_all_types_cloud_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();
        let p = cloud.point(0).unwrap();

        let i8_desc = cloud.field("i8_field").unwrap();
        let i16_desc = cloud.field("i16_field").unwrap();
        let i32_desc = cloud.field("i32_field").unwrap();
        let f64_desc = cloud.field("f64_field").unwrap();

        assert_eq!(p.read_i8_at(i8_desc).unwrap(), -2);
        assert_eq!(p.read_i16_at(i16_desc).unwrap(), -300);
        assert_eq!(p.read_i32_at(i32_desc).unwrap(), -100_000);
        assert_eq!(p.read_f64_at(f64_desc).unwrap(), std::f64::consts::E);
    }

    #[test]
    fn dyn_cloud_gather_signed_and_f64() {
        let cdr = make_all_types_cloud_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();

        assert_eq!(cloud.gather_i8("i8_field"), Some(vec![-2]));
        assert_eq!(cloud.gather_i16("i16_field"), Some(vec![-300]));
        assert_eq!(cloud.gather_i32("i32_field"), Some(vec![-100_000]));
        assert_eq!(
            cloud.gather_f64("f64_field"),
            Some(vec![std::f64::consts::E])
        );
    }

    #[test]
    fn dyn_point_signed_f64_wrong_type_returns_none() {
        let cdr = make_all_types_cloud_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();
        let p = cloud.point(0).unwrap();

        // read_i8 on non-i8 field
        assert!(p.read_i8("f32_field").is_none());
        assert!(p.read_i16("i8_field").is_none());
        assert!(p.read_i32("i16_field").is_none());
        assert!(p.read_f64("f32_field").is_none());
        assert!(p.read_f32("f64_field").is_none());

        // gather wrong type
        assert!(cloud.gather_i8("f64_field").is_none());
        assert!(cloud.gather_f64("i8_field").is_none());
        assert!(cloud.gather_i16("i32_field").is_none());
        assert!(cloud.gather_i32("i16_field").is_none());
    }

    #[test]
    fn dyn_point_at_invalid_descriptor_returns_error() {
        let cdr = make_all_types_cloud_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();
        let p = cloud.point(0).unwrap();

        let bad = FieldDesc {
            name: "fake",
            byte_offset: 9999,
            field_type: PointFieldType::Float32,
            count: 1,
        };
        assert!(matches!(
            p.read_f32_at(&bad),
            Err(PointCloudError::FieldAccessOutOfBounds { byte_offset: 9999 })
        ));
        assert!(p.read_u8_at(&bad).is_err());
        assert!(p.read_i8_at(&bad).is_err());
        assert!(p.read_u16_at(&bad).is_err());
        assert!(p.read_i16_at(&bad).is_err());
        assert!(p.read_u32_at(&bad).is_err());
        assert!(p.read_i32_at(&bad).is_err());
        assert!(p.read_f64_at(&bad).is_err());
    }

    #[test]
    fn dyn_cloud_gather_with_row_padding() {
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ];
        let point_step = 12u32;
        let width = 2u32;
        let height = 2u32;
        let row_step = 32u32; // 8 bytes padding per row
        let total = (row_step * height) as usize;
        let mut data = vec![0xFFu8; total];

        // Write known x values at correct padded offsets
        for row in 0..height {
            for col in 0..width {
                let off = (row * row_step + col * point_step) as usize;
                let val = (row * width + col) as f32;
                data[off..off + 4].copy_from_slice(&val.to_le_bytes());
            }
        }

        let pc = PointCloud2::new(
            Time::new(0, 0),
            "pad",
            height,
            width,
            &fields,
            false,
            point_step,
            row_step,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = DynPointCloud::from_pointcloud2(&decoded).unwrap();

        let gathered = cloud.gather_f32("x").unwrap();
        assert_eq!(gathered, vec![0.0, 1.0, 2.0, 3.0]);
    }

    #[test]
    fn dyn_cloud_max_fields_boundary() {
        // Build 16 u8 fields — should succeed
        let names: Vec<String> = (0..17).map(|i| format!("f{i}")).collect();
        let fields_16: Vec<PointFieldView<'_>> = (0..16)
            .map(|i| PointFieldView {
                name: &names[i],
                offset: i as u32,
                datatype: 2, // Uint8
                count: 1,
            })
            .collect();
        let data = vec![0u8; 16];
        let pc = PointCloud2::new(
            Time::new(0, 0),
            "max",
            1,
            1,
            &fields_16,
            false,
            16,
            16,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        assert!(DynPointCloud::from_pointcloud2(&decoded).is_ok());

        // Build 17 fields — should fail with TooManyFields
        let fields_17: Vec<PointFieldView<'_>> = (0..17)
            .map(|i| PointFieldView {
                name: &names[i],
                offset: i as u32,
                datatype: 2,
                count: 1,
            })
            .collect();
        let data = vec![0u8; 17];
        let pc = PointCloud2::new(
            Time::new(0, 0),
            "max",
            1,
            1,
            &fields_17,
            false,
            17,
            17,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        assert!(matches!(
            DynPointCloud::from_pointcloud2(&decoded),
            Err(PointCloudError::TooManyFields { found: 17 })
        ));
    }

    #[test]
    fn dyn_cloud_rejects_row_step_too_small() {
        let fields = [PointFieldView {
            name: "x",
            offset: 0,
            datatype: 7,
            count: 1,
        }];
        let data = vec![0u8; 48];
        // row_step = 2, but width * point_step = 2 * 4 = 8
        let pc = PointCloud2::new(
            Time::new(0, 0),
            "bad",
            3,
            2,
            &fields,
            false,
            4,
            2,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        assert!(matches!(
            DynPointCloud::from_pointcloud2(&decoded),
            Err(PointCloudError::InvalidLayout { .. })
        ));
    }

    #[test]
    fn static_cloud_rejects_row_step_too_small() {
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ];
        let data = vec![0u8; 48];
        let pc = PointCloud2::new(
            Time::new(0, 0),
            "bad",
            2,
            2,
            &fields,
            false,
            12,
            4,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        assert!(matches!(
            PointCloud::<TestXyzPoint>::from_pointcloud2(&decoded),
            Err(PointCloudError::InvalidLayout { .. })
        ));
    }

    #[test]
    fn static_cloud_iter_with_row_padding() {
        let fields = [
            PointFieldView {
                name: "x",
                offset: 0,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "y",
                offset: 4,
                datatype: 7,
                count: 1,
            },
            PointFieldView {
                name: "z",
                offset: 8,
                datatype: 7,
                count: 1,
            },
        ];
        let point_step = 12u32;
        let width = 2u32;
        let height = 2u32;
        let row_step = 32u32; // 8 bytes padding per row
        let total = (row_step * height) as usize;
        let mut data = vec![0xFFu8; total];

        for row in 0..height {
            for col in 0..width {
                let off = (row * row_step + col * point_step) as usize;
                let val = (row * width + col) as f32;
                data[off..off + 4].copy_from_slice(&val.to_le_bytes());
            }
        }

        let pc = PointCloud2::new(
            Time::new(0, 0),
            "pad",
            height,
            width,
            &fields,
            false,
            point_step,
            row_step,
            &data,
            true,
        )
        .unwrap();
        let cdr = pc.to_cdr();
        let decoded = PointCloud2::from_cdr(&cdr).unwrap();
        let cloud = PointCloud::<TestXyzPoint>::from_pointcloud2(&decoded).unwrap();

        // Test iter() correctness across row-padded boundaries
        let xs: Vec<f32> = cloud.iter().map(|p| p.x).collect();
        assert_eq!(xs, vec![0.0, 1.0, 2.0, 3.0]);

        // Test get() also uses correct offsets
        assert_eq!(cloud.get(2).unwrap().x, 2.0);
        assert_eq!(cloud.get(3).unwrap().x, 3.0);
    }
}
