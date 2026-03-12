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

/// Maximum number of fields in a DynPointCloud.
///
/// 16 covers all practical sensor outputs. Clouds with more fields
/// than this should use the static `PointCloud<P>` tier instead.
const MAX_FIELDS: usize = 16;

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
        }
    }
}

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
/// use edgefirst_schemas::sensor_msgs::pointcloud::PointFieldType;
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
/// descriptors. Access is by field name with explicit type.
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
    num_points: usize,
    fields: [Option<FieldDesc<'a>>; MAX_FIELDS],
    field_count: usize,
    height: u32,
    width: u32,
}

impl<'a> DynPointCloud<'a> {
    /// Create a dynamic point cloud view from a PointCloud2 message.
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

        if num_points > 0 && data.len() < num_points * point_step {
            return Err(PointCloudError::InvalidLayout {
                reason: "data buffer shorter than num_points × point_step",
            });
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
            fields[field_count] = Some(desc);
            field_count += 1;
        }

        Ok(DynPointCloud {
            data,
            point_step,
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

    /// Get a single point view by linear index.
    pub fn point(&self, index: usize) -> Option<DynPoint<'a, '_>> {
        if index >= self.num_points {
            return None;
        }
        let base = index * self.point_step;
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
    pub fn point_at(&self, row: u32, col: u32) -> Option<DynPoint<'a, '_>> {
        if row >= self.height || col >= self.width {
            return None;
        }
        self.point((row as usize) * (self.width as usize) + (col as usize))
    }

    /// Iterate over all points.
    pub fn iter(&self) -> DynPointIter<'a, '_> {
        DynPointIter {
            cloud: self,
            index: 0,
        }
    }

    /// Gather a named f32 field into a Vec.
    pub fn gather_f32(&self, name: &str) -> Option<Vec<f32>> {
        let desc = self.field(name)?;
        if desc.field_type != PointFieldType::Float32 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let mut out = Vec::with_capacity(self.num_points);
        for i in 0..self.num_points {
            let base = i * self.point_step + off;
            let bytes: [u8; 4] = self.data[base..base + 4].try_into().ok()?;
            out.push(f32::from_le_bytes(bytes));
        }
        Some(out)
    }

    /// Gather a named u32 field into a Vec.
    pub fn gather_u32(&self, name: &str) -> Option<Vec<u32>> {
        let desc = self.field(name)?;
        if desc.field_type != PointFieldType::Uint32 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let mut out = Vec::with_capacity(self.num_points);
        for i in 0..self.num_points {
            let base = i * self.point_step + off;
            let bytes: [u8; 4] = self.data[base..base + 4].try_into().ok()?;
            out.push(u32::from_le_bytes(bytes));
        }
        Some(out)
    }

    /// Gather a named u16 field into a Vec.
    pub fn gather_u16(&self, name: &str) -> Option<Vec<u16>> {
        let desc = self.field(name)?;
        if desc.field_type != PointFieldType::Uint16 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let mut out = Vec::with_capacity(self.num_points);
        for i in 0..self.num_points {
            let base = i * self.point_step + off;
            let bytes: [u8; 2] = self.data[base..base + 2].try_into().ok()?;
            out.push(u16::from_le_bytes(bytes));
        }
        Some(out)
    }

    /// Gather a named u8 field into a Vec.
    pub fn gather_u8(&self, name: &str) -> Option<Vec<u8>> {
        let desc = self.field(name)?;
        if desc.field_type != PointFieldType::Uint8 {
            return None;
        }
        let off = desc.byte_offset as usize;
        let mut out = Vec::with_capacity(self.num_points);
        for i in 0..self.num_points {
            out.push(self.data[i * self.point_step + off]);
        }
        Some(out)
    }
}

// ── DynPoint ────────────────────────────────────────────────────────

/// Zero-copy view of a single point within a DynPointCloud.
///
/// Field access reads bytes from the point's data slice using
/// `from_le_bytes` — no unsafe code, no alignment concerns.
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

    /// Read an f32 field by pre-resolved descriptor (avoids name lookup).
    pub fn read_f32_at(&self, desc: &FieldDesc<'_>) -> f32 {
        let off = desc.byte_offset as usize;
        let bytes: [u8; 4] = self.data[off..off + 4]
            .try_into()
            .expect("field offset within point bounds");
        f32::from_le_bytes(bytes)
    }

    /// Read a u32 field by pre-resolved descriptor.
    pub fn read_u32_at(&self, desc: &FieldDesc<'_>) -> u32 {
        let off = desc.byte_offset as usize;
        let bytes: [u8; 4] = self.data[off..off + 4]
            .try_into()
            .expect("field offset within point bounds");
        u32::from_le_bytes(bytes)
    }

    /// Read a u16 field by pre-resolved descriptor.
    pub fn read_u16_at(&self, desc: &FieldDesc<'_>) -> u16 {
        let off = desc.byte_offset as usize;
        let bytes: [u8; 2] = self.data[off..off + 2]
            .try_into()
            .expect("field offset within point bounds");
        u16::from_le_bytes(bytes)
    }

    /// Read a u8 field by pre-resolved descriptor.
    pub fn read_u8_at(&self, desc: &FieldDesc<'_>) -> u8 {
        self.data[desc.byte_offset as usize]
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
///     pub struct XyzPoint { x: f32 => 0, y: f32 => 4, z: f32 at 8 }
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
    num_points: usize,
    height: u32,
    width: u32,
    _marker: core::marker::PhantomData<P>,
}

impl<'a, P: Point> PointCloud<'a, P> {
    /// Create a typed point cloud view, validating field layout.
    pub fn from_pointcloud2<B: AsRef<[u8]>>(
        pc: &'a super::PointCloud2<B>,
    ) -> Result<Self, PointCloudError> {
        if pc.is_bigendian() {
            return Err(PointCloudError::BigEndianNotSupported);
        }

        Self::validate(pc)?;

        let point_step = pc.point_step() as usize;
        let num_points = pc.point_count();

        Ok(PointCloud {
            data: pc.data(),
            point_step,
            num_points,
            height: pc.height(),
            width: pc.width(),
            _marker: core::marker::PhantomData,
        })
    }

    /// Validate that the PointCloud2 field layout matches `P`'s expectations.
    pub fn validate<B: AsRef<[u8]>>(pc: &super::PointCloud2<B>) -> Result<(), PointCloudError> {
        let fields: Vec<_> = pc.fields_iter().collect();
        let expected = P::expected_fields();

        for exp in expected {
            let found = fields.iter().find(|f| f.name == exp.name);
            match found {
                None => {
                    return Err(PointCloudError::FieldNotFound { name: exp.name });
                }
                Some(f) => {
                    if f.offset != exp.byte_offset {
                        return Err(PointCloudError::FieldMismatch {
                            name: exp.name,
                            reason: "byte offset mismatch",
                        });
                    }
                    let actual_type = PointFieldType::from_datatype(f.datatype);
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

    /// Read a single point by linear index.
    pub fn get(&self, index: usize) -> Option<P> {
        if index >= self.num_points {
            return None;
        }
        let base = index * self.point_step;
        if base + P::point_size() as usize > self.data.len() {
            return None;
        }
        Some(P::read_from(self.data, base))
    }

    /// Read a point by (row, col) for organized clouds.
    pub fn get_at(&self, row: u32, col: u32) -> Option<P> {
        if row >= self.height || col >= self.width {
            return None;
        }
        self.get((row as usize) * (self.width as usize) + (col as usize))
    }

    /// Iterate over all points.
    pub fn iter(&self) -> PointIter<'a, P> {
        PointIter {
            data: self.data,
            point_step: self.point_step,
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
    num_points: usize,
    index: usize,
    _marker: core::marker::PhantomData<P>,
}

impl<P: Point> Iterator for PointIter<'_, P> {
    type Item = P;

    fn next(&mut self) -> Option<P> {
        if self.index >= self.num_points {
            return None;
        }
        let base = self.index * self.point_step;
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
            assert_eq!(point.read_f32_at(x_desc), expected_x);
            assert_eq!(point.read_f32_at(z_desc), expected_z);
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
}
