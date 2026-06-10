// SPDX-License-Identifier: Apache-2.0
// Copyright © 2026 Au-Zone Technologies. All Rights Reserved.

//! ROS 2 `nav_msgs` message types.
//!
//! CdrFixed: `MapMetaData`
//!
//! Buffer-backed: `GridCells`, `OccupancyGrid`, `Odometry`, `Path`

use crate::builtin_interfaces::Time;
use crate::cdr::*;
use crate::geometry_msgs::{
    scan_pose_stamped, size_pose_stamped, write_pose_stamped, Point, Pose, PoseWithCovariance,
    Quaternion, Twist, TwistWithCovariance, Vector3,
};
use crate::std_msgs::Header;

// Zero-valued constants for builder defaults. The covariance-bearing geometry
// types hold `[f64; 36]`, which has no `Default` impl, so the zeros are spelled
// out here once and shared by the builders below.
const ZERO_POSE: Pose = Pose {
    position: Point {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    },
    orientation: Quaternion {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        w: 0.0,
    },
};
const ZERO_MAP_META_DATA: MapMetaData = MapMetaData {
    map_load_time: Time { sec: 0, nanosec: 0 },
    resolution: 0.0,
    width: 0,
    height: 0,
    origin: ZERO_POSE,
};
const ZERO_POSE_WITH_COVARIANCE: PoseWithCovariance = PoseWithCovariance {
    pose: ZERO_POSE,
    covariance: [0.0; 36],
};
const ZERO_TWIST_WITH_COVARIANCE: TwistWithCovariance = TwistWithCovariance {
    twist: Twist {
        linear: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        angular: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
    },
    covariance: [0.0; 36],
};

// ── Odometry<B> ─────────────────────────────────────────────────────
//
// CDR layout: Header → pre, then:
//   string child_frame_id → offsets[0] (start of child_frame_id)
//   pad to 8 → offsets[1] (start of PoseWithCovariance)
//   PoseWithCovariance pose  → offsets[2] captured after pose decode,
//                              i.e. start of TwistWithCovariance
//   TwistWithCovariance twist
//
// offsets[1] is captured from the cursor after aligning to 8 because
// `child_frame_id` is a variable-length string; PoseWithCovariance
// starts with a Point (f64 first), so an explicit align-to-8 is needed.
// offsets[2] is captured from the cursor after pose decode so `twist()`
// doesn't depend on `PoseWithCovariance::CDR_SIZE` layout arithmetic.

pub struct Odometry<B> {
    buf: B,
    offsets: [usize; 3],
}

impl<B> Odometry<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> Odometry<C> {
        Odometry {
            buf: f(self.buf),
            offsets: self.offsets,
        }
    }
}

impl<B: AsRef<[u8]>> Odometry<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let pre = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), pre);
        let o0 = c.offset();
        let _ = c.read_string()?; // child_frame_id
        c.align(8);
        let o1 = c.offset();
        PoseWithCovariance::read_cdr(&mut c)?;
        let o2 = c.offset();
        TwistWithCovariance::read_cdr(&mut c)?;
        Ok(Odometry {
            offsets: [o0, o1, o2],
            buf,
        })
    }

    /// Returns a `Header` view by re-parsing the CDR buffer prefix.
    pub fn header(&self) -> Header<&[u8]> {
        Header::from_cdr(self.buf.as_ref()).expect("header bytes validated during from_cdr")
    }
    pub fn stamp(&self) -> Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }
    pub fn frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 8).0
    }
    pub fn child_frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), self.offsets[0]).0
    }
    pub fn pose(&self) -> PoseWithCovariance {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[1]);
        PoseWithCovariance::read_cdr(&mut c).expect("pose validated during from_cdr")
    }
    pub fn twist(&self) -> TwistWithCovariance {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[2]);
        TwistWithCovariance::read_cdr(&mut c).expect("twist validated during from_cdr")
    }
    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl Odometry<Vec<u8>> {
    pub fn new(
        stamp: Time,
        frame_id: &str,
        child_frame_id: &str,
        pose: PoseWithCovariance,
        twist: TwistWithCovariance,
    ) -> Result<Self, CdrError> {
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_string(child_frame_id);
        sizer.align(8);
        let o1 = sizer.offset();
        PoseWithCovariance::size_cdr(&mut sizer);
        let o2 = sizer.offset();
        TwistWithCovariance::size_cdr(&mut sizer);

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_string(child_frame_id);
        pose.write_cdr(&mut w);
        twist.write_cdr(&mut w);
        w.finish()?;

        Ok(Odometry {
            offsets: [o0, o1, o2],
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `OdometryBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> OdometryBuilder<'a> {
        OdometryBuilder::new()
    }
}

// ── OdometryBuilder<'a> ─────────────────────────────────────────────

/// Builder for `Odometry<Vec<u8>>` with buffer-reuse finalizers.
pub struct OdometryBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    child_frame_id: std::borrow::Cow<'a, str>,
    pose: PoseWithCovariance,
    twist: TwistWithCovariance,
}

impl<'a> Default for OdometryBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            child_frame_id: std::borrow::Cow::Borrowed(""),
            pose: ZERO_POSE_WITH_COVARIANCE,
            twist: ZERO_TWIST_WITH_COVARIANCE,
        }
    }
}

impl<'a> OdometryBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stamp(&mut self, t: Time) -> &mut Self {
        self.stamp = t;
        self
    }
    pub fn frame_id(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.frame_id = s.into();
        self
    }
    pub fn child_frame_id(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.child_frame_id = s.into();
        self
    }
    pub fn pose(&mut self, p: PoseWithCovariance) -> &mut Self {
        self.pose = p;
        self
    }
    pub fn twist(&mut self, t: TwistWithCovariance) -> &mut Self {
        self.twist = t;
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        s.size_string(&self.frame_id);
        s.size_string(&self.child_frame_id);
        s.align(8);
        PoseWithCovariance::size_cdr(&mut s);
        TwistWithCovariance::size_cdr(&mut s);
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.write_string(&self.child_frame_id);
        self.pose.write_cdr(&mut w);
        self.twist.write_cdr(&mut w);
        w.finish()
    }

    pub fn build(&self) -> Result<Odometry<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        Odometry::from_cdr(buf)
    }

    pub fn encode_into_vec(&self, buf: &mut Vec<u8>) -> Result<(), CdrError> {
        buf.resize(self.size(), 0);
        self.write_into(buf)
    }

    pub fn encode_into_slice(&self, buf: &mut [u8]) -> Result<usize, CdrError> {
        let need = self.size();
        if buf.len() < need {
            return Err(CdrError::BufferTooShort {
                need,
                have: buf.len(),
            });
        }
        self.write_into(&mut buf[..need])?;
        Ok(need)
    }
}

impl<B: AsRef<[u8]> + AsMut<[u8]>> Odometry<B> {
    /// In-place overwrite of the header `stamp` (no re-encode).
    pub fn set_stamp(&mut self, t: Time) -> Result<(), CdrError> {
        let b = self.buf.as_mut();
        wr_i32(b, CDR_HEADER_SIZE, t.sec)?;
        wr_u32(b, CDR_HEADER_SIZE + 4, t.nanosec)
    }
}

// ── MapMetaData ─────────────────────────────────────────────────────
//
// CDR layout (data-relative offsets, counting from start of payload):
//   map_load_time (Time, 8 bytes)  → data offset 0
//   resolution    (f32,  4 bytes)  → data offset 8
//   width         (u32,  4 bytes)  → data offset 12
//   height        (u32,  4 bytes)  → data offset 16
//   [4-byte pad to 8-align for Pose]→ data offset 20–23
//   origin        (Pose, 56 bytes) → data offset 24
//
// CDR_SIZE = 80 when the type starts at a data offset that is a
// multiple of 8 (e.g. standalone via encode_fixed/decode_fixed).  When
// embedded at a data offset ≡ 4 (mod 8) the Pose field falls naturally
// on an 8-byte boundary and no padding is inserted (size = 76).  Do NOT
// use offset arithmetic of the form `base + MapMetaData::CDR_SIZE` in
// buffer-backed composite types; capture the post-field cursor position
// instead.  See the `CdrFixed` trait docs (EDGEAI-1243 pattern).

#[derive(PartialEq, Clone, Copy, Debug)]
pub struct MapMetaData {
    pub map_load_time: Time,
    pub resolution: f32,
    pub width: u32,
    pub height: u32,
    pub origin: Pose,
}

impl CdrFixed for MapMetaData {
    /// Wire size when starting at a data offset divisible by 8.
    ///
    /// WARNING: position-dependent padding — only 76 bytes at a start ≡ 4
    /// (mod 8). Never use `base + MapMetaData::CDR_SIZE` offset arithmetic in
    /// composite types; capture the post-field cursor position instead (see the
    /// `CdrFixed` trait docs, EDGEAI-1243). `OccupancyGrid::from_cdr` does this.
    const CDR_SIZE: usize = 80;

    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError> {
        let map_load_time = Time::read_cdr(cursor)?;
        let resolution = cursor.read_f32()?;
        let width = cursor.read_u32()?;
        let height = cursor.read_u32()?;
        let origin = Pose::read_cdr(cursor)?; // auto-aligns to 8 before first f64
        Ok(MapMetaData {
            map_load_time,
            resolution,
            width,
            height,
            origin,
        })
    }

    fn write_cdr(&self, writer: &mut CdrWriter<'_>) {
        self.map_load_time.write_cdr(writer);
        writer.write_f32(self.resolution);
        writer.write_u32(self.width);
        writer.write_u32(self.height);
        self.origin.write_cdr(writer); // auto-aligns to 8 before first f64
    }

    fn size_cdr(sizer: &mut CdrSizer) {
        Time::size_cdr(sizer);
        sizer.size_f32();
        sizer.size_u32();
        sizer.size_u32();
        Pose::size_cdr(sizer); // auto-aligns to 8 before first f64
    }
}

use crate::schema_registry::SchemaType;

impl SchemaType for MapMetaData {
    const SCHEMA_NAME: &'static str = "nav_msgs/msg/MapMetaData";
}

// ── GridCells<B> ────────────────────────────────────────────────────
//
// CDR layout: Header → offsets[0] (cell_width), then:
//   f32 cell_width (4 bytes), f32 cell_height (4 bytes),
//   u32 seq_len → offsets[1] (seq start), Point[] (24 bytes each)

pub struct GridCells<B> {
    buf: B,
    offsets: [usize; 2], // [0] = cell_width start (4-aligned), [1] = start of Point data
    count: usize,
}

impl<B> GridCells<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> GridCells<C> {
        GridCells {
            buf: f(self.buf),
            offsets: self.offsets,
            count: self.count,
        }
    }
}

impl<B: AsRef<[u8]>> GridCells<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let pre = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), pre);
        c.align(4);
        let o0 = c.offset();
        c.read_f32()?; // cell_width
        c.read_f32()?; // cell_height
        let raw = c.read_seq_len()?;
        let count = c.check_seq_count(raw, Point::CDR_SIZE)?;
        let o1 = c.offset();
        for _ in 0..count {
            Point::read_cdr(&mut c)?;
        }
        Ok(GridCells {
            offsets: [o0, o1],
            count,
            buf,
        })
    }

    pub fn header(&self) -> Header<&[u8]> {
        Header::from_cdr(self.buf.as_ref()).expect("header bytes validated during from_cdr")
    }
    pub fn stamp(&self) -> Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }
    pub fn frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 8).0
    }
    pub fn cell_width(&self) -> f32 {
        rd_f32(self.buf.as_ref(), self.offsets[0])
    }
    pub fn cell_height(&self) -> f32 {
        rd_f32(self.buf.as_ref(), self.offsets[0] + 4)
    }

    /// Number of cells in the sequence.
    #[inline]
    pub fn len(&self) -> usize {
        self.count
    }

    /// Returns true if the sequence is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Get a single cell centre point by index.
    #[inline]
    pub fn cell(&self, index: usize) -> Option<Point> {
        if index >= self.count {
            return None;
        }
        let offset = self.offsets[1] + index * Point::CDR_SIZE;
        let mut c = CdrCursor::resume(self.buf.as_ref(), offset);
        Some(Point::read_cdr(&mut c).expect("cell point validated during from_cdr"))
    }

    /// Get all cell centre points as a Vec.
    pub fn cells(&self) -> Vec<Point> {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[1]);
        (0..self.count)
            .map(|_| Point::read_cdr(&mut c).expect("cell point validated during from_cdr"))
            .collect()
    }

    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl GridCells<Vec<u8>> {
    pub fn new(
        stamp: Time,
        frame_id: &str,
        cell_width: f32,
        cell_height: f32,
        cells: &[Point],
    ) -> Result<Self, CdrError> {
        let count = cells.len();
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        sizer.align(4);
        let o0 = sizer.offset();
        sizer.size_f32(); // cell_width
        sizer.size_f32(); // cell_height
        sizer.size_u32(); // seq_len
        let o1 = sizer.offset();
        for _ in 0..count {
            Point::size_cdr(&mut sizer);
        }

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_f32(cell_width);
        w.write_f32(cell_height);
        w.write_u32(count as u32);
        for p in cells {
            p.write_cdr(&mut w);
        }
        w.finish()?;

        Ok(GridCells {
            offsets: [o0, o1],
            count,
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `GridCellsBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> GridCellsBuilder<'a> {
        GridCellsBuilder::new()
    }
}

// ── GridCellsBuilder<'a> ────────────────────────────────────────────

/// Builder for `GridCells<Vec<u8>>` with buffer-reuse finalizers.
pub struct GridCellsBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    cell_width: f32,
    cell_height: f32,
    cells: &'a [Point],
}

impl<'a> Default for GridCellsBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            cell_width: 0.0,
            cell_height: 0.0,
            cells: &[],
        }
    }
}

impl<'a> GridCellsBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stamp(&mut self, t: Time) -> &mut Self {
        self.stamp = t;
        self
    }
    pub fn frame_id(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.frame_id = s.into();
        self
    }
    pub fn cell_width(&mut self, v: f32) -> &mut Self {
        self.cell_width = v;
        self
    }
    pub fn cell_height(&mut self, v: f32) -> &mut Self {
        self.cell_height = v;
        self
    }
    pub fn cells(&mut self, cells: &'a [Point]) -> &mut Self {
        self.cells = cells;
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        s.size_string(&self.frame_id);
        s.align(4);
        s.size_f32();
        s.size_f32();
        s.size_u32();
        for _ in self.cells {
            Point::size_cdr(&mut s);
        }
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.write_f32(self.cell_width);
        w.write_f32(self.cell_height);
        w.write_u32(self.cells.len() as u32);
        for p in self.cells {
            p.write_cdr(&mut w);
        }
        w.finish()
    }

    pub fn build(&self) -> Result<GridCells<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        GridCells::from_cdr(buf)
    }

    pub fn encode_into_vec(&self, buf: &mut Vec<u8>) -> Result<(), CdrError> {
        buf.resize(self.size(), 0);
        self.write_into(buf)
    }

    pub fn encode_into_slice(&self, buf: &mut [u8]) -> Result<usize, CdrError> {
        let need = self.size();
        if buf.len() < need {
            return Err(CdrError::BufferTooShort {
                need,
                have: buf.len(),
            });
        }
        self.write_into(&mut buf[..need])?;
        Ok(need)
    }
}

impl<B: AsRef<[u8]> + AsMut<[u8]>> GridCells<B> {
    /// In-place overwrite of the header `stamp` (no re-encode).
    pub fn set_stamp(&mut self, t: Time) -> Result<(), CdrError> {
        let b = self.buf.as_mut();
        wr_i32(b, CDR_HEADER_SIZE, t.sec)?;
        wr_u32(b, CDR_HEADER_SIZE + 4, t.nanosec)
    }

    /// In-place overwrite of `cell_width` (no re-encode).
    pub fn set_cell_width(&mut self, v: f32) -> Result<(), CdrError> {
        wr_f32(self.buf.as_mut(), self.offsets[0], v)
    }

    /// In-place overwrite of `cell_height` (no re-encode).
    pub fn set_cell_height(&mut self, v: f32) -> Result<(), CdrError> {
        wr_f32(self.buf.as_mut(), self.offsets[0] + 4, v)
    }
}

// ── OccupancyGrid<B> ────────────────────────────────────────────────
//
// CDR layout: Header → offsets[0] (MapMetaData start, cursor-captured),
//   MapMetaData info → offsets[1] (seq_len position),
//   u32 seq_len + i8[] data → offsets[2] (start of i8 data bytes)

pub struct OccupancyGrid<B> {
    buf: B,
    // [0]: absolute offset of first MapMetaData field (map_load_time.sec)
    // [1]: absolute offset of the seq_len u32 for `data`
    // [2]: absolute offset of the first i8 element in `data`
    offsets: [usize; 3],
    count: usize,
}

impl<B> OccupancyGrid<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> OccupancyGrid<C> {
        OccupancyGrid {
            buf: f(self.buf),
            offsets: self.offsets,
            count: self.count,
        }
    }
}

impl<B: AsRef<[u8]>> OccupancyGrid<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let pre = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), pre);
        let o0 = c.offset();
        MapMetaData::read_cdr(&mut c)?; // reads map_load_time, resolution, width, height, origin
        let o1 = c.offset();
        let raw = c.read_seq_len()?;
        let count = c.check_seq_count(raw, 1)?;
        let o2 = c.offset();
        c.skip(count)?;
        Ok(OccupancyGrid {
            offsets: [o0, o1, o2],
            count,
            buf,
        })
    }

    pub fn header(&self) -> Header<&[u8]> {
        Header::from_cdr(self.buf.as_ref()).expect("header bytes validated during from_cdr")
    }
    pub fn stamp(&self) -> Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }
    pub fn frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 8).0
    }

    /// Parse and return the map metadata.
    pub fn info(&self) -> MapMetaData {
        let mut c = CdrCursor::resume(self.buf.as_ref(), self.offsets[0]);
        MapMetaData::read_cdr(&mut c).expect("MapMetaData validated during from_cdr")
    }

    /// Number of cells in the occupancy grid data sequence.
    #[inline]
    pub fn len(&self) -> usize {
        self.count
    }

    /// Returns true if the data sequence is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Access the raw occupancy data bytes (one i8 per cell, zero-copy).
    pub fn data(&self) -> &[i8] {
        let bytes = &self.buf.as_ref()[self.offsets[2]..self.offsets[2] + self.count];
        // SAFETY: i8 and u8 have the same size/alignment; we reinterpret the slice.
        unsafe { std::slice::from_raw_parts(bytes.as_ptr() as *const i8, self.count) }
    }

    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

impl OccupancyGrid<Vec<u8>> {
    pub fn new(
        stamp: Time,
        frame_id: &str,
        info: MapMetaData,
        data: &[i8],
    ) -> Result<Self, CdrError> {
        let count = data.len();
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        MapMetaData::size_cdr(&mut sizer);
        let o1 = sizer.offset();
        sizer.size_u32(); // seq_len
        let o2 = sizer.offset();
        sizer.size_raw(count); // i8[] — no alignment needed (1-byte elements)

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        info.write_cdr(&mut w);
        w.write_u32(count as u32);
        for &v in data {
            w.write_i8(v);
        }
        w.finish()?;

        Ok(OccupancyGrid {
            offsets: [o0, o1, o2],
            count,
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `OccupancyGridBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> OccupancyGridBuilder<'a> {
        OccupancyGridBuilder::new()
    }
}

// ── OccupancyGridBuilder<'a> ────────────────────────────────────────

/// Builder for `OccupancyGrid<Vec<u8>>` with buffer-reuse finalizers.
pub struct OccupancyGridBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    info: MapMetaData,
    data: &'a [i8],
}

impl<'a> Default for OccupancyGridBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            info: ZERO_MAP_META_DATA,
            data: &[],
        }
    }
}

impl<'a> OccupancyGridBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stamp(&mut self, t: Time) -> &mut Self {
        self.stamp = t;
        self
    }
    pub fn frame_id(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.frame_id = s.into();
        self
    }
    pub fn info(&mut self, info: MapMetaData) -> &mut Self {
        self.info = info;
        self
    }
    pub fn data(&mut self, data: &'a [i8]) -> &mut Self {
        self.data = data;
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        s.size_string(&self.frame_id);
        MapMetaData::size_cdr(&mut s);
        s.size_u32();
        s.size_raw(self.data.len());
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        self.info.write_cdr(&mut w);
        w.write_u32(self.data.len() as u32);
        for &v in self.data {
            w.write_i8(v);
        }
        w.finish()
    }

    pub fn build(&self) -> Result<OccupancyGrid<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        OccupancyGrid::from_cdr(buf)
    }

    pub fn encode_into_vec(&self, buf: &mut Vec<u8>) -> Result<(), CdrError> {
        buf.resize(self.size(), 0);
        self.write_into(buf)
    }

    pub fn encode_into_slice(&self, buf: &mut [u8]) -> Result<usize, CdrError> {
        let need = self.size();
        if buf.len() < need {
            return Err(CdrError::BufferTooShort {
                need,
                have: buf.len(),
            });
        }
        self.write_into(&mut buf[..need])?;
        Ok(need)
    }
}

impl<B: AsRef<[u8]> + AsMut<[u8]>> OccupancyGrid<B> {
    /// In-place overwrite of the header `stamp` (no re-encode).
    pub fn set_stamp(&mut self, t: Time) -> Result<(), CdrError> {
        let b = self.buf.as_mut();
        wr_i32(b, CDR_HEADER_SIZE, t.sec)?;
        wr_u32(b, CDR_HEADER_SIZE + 4, t.nanosec)
    }
}

// ── Path<B> ─────────────────────────────────────────────────────────
//
// CDR layout: Header → offsets[0] (seq_len position),
//   u32 seq_len → offsets[1] (start of first PoseStamped data),
//   PoseStamped[] (variable-length: stamp + frame_id string + Pose each)
//
// PoseStamped elements inside the sequence do NOT carry their own
// CDR encapsulation header.  Each element is:
//   Time stamp (8 bytes) + string frame_id + Pose (56 bytes, auto-aligned).
// Access is sequential because frame_id string lengths vary.

pub struct Path<B> {
    buf: B,
    offsets: [usize; 2], // [0] = seq_len position, [1] = first PoseStamped data start
    count: usize,
}

impl<B> Path<B> {
    /// Convert the buffer type without re-parsing the offset table.
    #[inline]
    pub fn map_buffer<C>(self, f: impl FnOnce(B) -> C) -> Path<C> {
        Path {
            buf: f(self.buf),
            offsets: self.offsets,
            count: self.count,
        }
    }
}

impl<B: AsRef<[u8]>> Path<B> {
    pub fn from_cdr(buf: B) -> Result<Self, CdrError> {
        let header = Header::<&[u8]>::from_cdr(buf.as_ref())?;
        let o0 = header.end_offset();
        let mut c = CdrCursor::resume(buf.as_ref(), o0);
        // PoseStamped mandatory minimum per element: Time(8) + string length
        // prefix(4) + Pose(56) = 68 bytes. The frame_id content and the 8-byte
        // Pose alignment padding only add more, so 68 is a safe (never rejects a
        // valid buffer) anti-amplification floor — an empty frame_id encodes as a
        // 4-byte length of 0. Per-element `scan_pose_stamped` still fully validates.
        let raw = c.read_seq_len()?;
        let count = c.check_seq_count(raw, 68)?;
        let o1 = c.offset();
        for _ in 0..count {
            scan_pose_stamped(&mut c)?;
        }
        Ok(Path {
            offsets: [o0, o1],
            count,
            buf,
        })
    }

    pub fn header(&self) -> Header<&[u8]> {
        Header::from_cdr(self.buf.as_ref()).expect("header bytes validated during from_cdr")
    }
    pub fn stamp(&self) -> Time {
        rd_time(self.buf.as_ref(), CDR_HEADER_SIZE)
    }
    pub fn frame_id(&self) -> &str {
        rd_string(self.buf.as_ref(), CDR_HEADER_SIZE + 8).0
    }

    /// Number of poses in the path.
    #[inline]
    pub fn len(&self) -> usize {
        self.count
    }

    /// Returns true if the path has no poses.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Zero-copy forward iterator over the path's `PoseStamped` elements.
    ///
    /// A single O(n) pass that yields `(stamp, frame_id, pose)` per element,
    /// borrowing `frame_id` directly from the CDR buffer with no allocation.
    /// Prefer this over repeated [`pose_at`](Self::pose_at) (O(n) per call)
    /// when traversing the whole path — a `0..len()` loop over `pose_at` is
    /// O(n²).
    pub fn iter(&self) -> PoseStampedIter<'_> {
        PoseStampedIter {
            cursor: CdrCursor::resume(self.buf.as_ref(), self.offsets[1]),
            remaining: self.count,
        }
    }

    /// Zero-copy access to a single `PoseStamped` element by index.
    ///
    /// Scans the variable-length sequence to `index` (O(n)) without
    /// allocating. Returns `(stamp, frame_id, pose)` borrowing `frame_id`
    /// from the CDR buffer. For full traversal use [`iter`](Self::iter) to
    /// avoid the O(n²) cost of calling this in a `0..len()` loop.
    pub fn pose_at(&self, index: usize) -> Option<(Time, &str, Pose)> {
        self.iter().nth(index)
    }

    /// Scan through the pose sequence and return all elements.
    ///
    /// Each returned tuple is `(stamp, frame_id, pose)`.  The `frame_id`
    /// string is allocated because elements are variable-length; prefer
    /// [`iter`](Self::iter) for an allocation-free borrowing traversal.
    pub fn poses(&self) -> Vec<(Time, String, Pose)> {
        self.iter()
            .map(|(stamp, frame_id, pose)| (stamp, frame_id.to_owned(), pose))
            .collect()
    }

    pub fn as_cdr(&self) -> &[u8] {
        self.buf.as_ref()
    }
    pub fn to_cdr(&self) -> Vec<u8> {
        self.buf.as_ref().to_vec()
    }
}

/// Non-allocating forward iterator over the `PoseStamped` elements of a [`Path`].
///
/// Created by [`Path::iter`]. Each call to `next` parses one element from the
/// CDR buffer and yields `(stamp, frame_id, pose)`, where `frame_id` borrows
/// directly from the buffer (no heap allocation). Implements
/// [`ExactSizeIterator`], so `len()` returns the remaining element count.
pub struct PoseStampedIter<'a> {
    cursor: CdrCursor<'a>,
    remaining: usize,
}

impl<'a> Iterator for PoseStampedIter<'a> {
    type Item = (Time, &'a str, Pose);

    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 {
            return None;
        }
        self.remaining -= 1;
        let stamp = Time::read_cdr(&mut self.cursor).expect("stamp validated during from_cdr");
        let frame_id = self
            .cursor
            .read_string()
            .expect("frame_id validated during from_cdr");
        let pose = Pose::read_cdr(&mut self.cursor).expect("pose validated during from_cdr");
        Some((stamp, frame_id, pose))
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.remaining, Some(self.remaining))
    }
}

impl ExactSizeIterator for PoseStampedIter<'_> {}

impl Path<Vec<u8>> {
    /// Construct a Path from a slice of `(stamp, frame_id, pose)` tuples.
    pub fn new(
        stamp: Time,
        frame_id: &str,
        poses: &[(Time, &str, Pose)],
    ) -> Result<Self, CdrError> {
        let count = poses.len();
        let mut sizer = CdrSizer::new();
        Time::size_cdr(&mut sizer);
        sizer.size_string(frame_id);
        let o0 = sizer.offset();
        sizer.size_u32(); // seq_len
        let o1 = sizer.offset();
        for &(_, fid, _) in poses {
            size_pose_stamped(&mut sizer, fid);
        }

        let mut buf = vec![0u8; sizer.size()];
        let mut w = CdrWriter::new(&mut buf)?;
        stamp.write_cdr(&mut w);
        w.write_string(frame_id);
        w.write_u32(count as u32);
        for &(ps, fid, pose) in poses {
            write_pose_stamped(&mut w, ps, fid, pose);
        }
        w.finish()?;

        Ok(Path {
            offsets: [o0, o1],
            count,
            buf,
        })
    }

    pub fn into_cdr(self) -> Vec<u8> {
        self.buf
    }

    /// Start a new `PathBuilder` with zero-valued defaults.
    pub fn builder<'a>() -> PathBuilder<'a> {
        PathBuilder::new()
    }
}

// ── PathBuilder<'a> ─────────────────────────────────────────────────

/// Builder for `Path<Vec<u8>>` with buffer-reuse finalizers.
///
/// The `poses` are held as a borrowed slice of `(stamp, frame_id, pose)`
/// tuples; each `frame_id` borrows for the builder's lifetime.
pub struct PathBuilder<'a> {
    stamp: Time,
    frame_id: std::borrow::Cow<'a, str>,
    poses: &'a [(Time, &'a str, Pose)],
}

impl<'a> Default for PathBuilder<'a> {
    fn default() -> Self {
        Self {
            stamp: Time { sec: 0, nanosec: 0 },
            frame_id: std::borrow::Cow::Borrowed(""),
            poses: &[],
        }
    }
}

impl<'a> PathBuilder<'a> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stamp(&mut self, t: Time) -> &mut Self {
        self.stamp = t;
        self
    }
    pub fn frame_id(&mut self, s: impl Into<std::borrow::Cow<'a, str>>) -> &mut Self {
        self.frame_id = s.into();
        self
    }
    pub fn poses(&mut self, poses: &'a [(Time, &'a str, Pose)]) -> &mut Self {
        self.poses = poses;
        self
    }

    fn size(&self) -> usize {
        let mut s = CdrSizer::new();
        Time::size_cdr(&mut s);
        s.size_string(&self.frame_id);
        s.size_u32(); // seq_len
        for &(_, fid, _) in self.poses {
            size_pose_stamped(&mut s, fid);
        }
        s.size()
    }

    fn write_into(&self, buf: &mut [u8]) -> Result<(), CdrError> {
        let mut w = CdrWriter::new(buf)?;
        self.stamp.write_cdr(&mut w);
        w.write_string(&self.frame_id);
        w.write_u32(self.poses.len() as u32);
        for &(ps, fid, pose) in self.poses {
            write_pose_stamped(&mut w, ps, fid, pose);
        }
        w.finish()
    }

    pub fn build(&self) -> Result<Path<Vec<u8>>, CdrError> {
        let mut buf = vec![0u8; self.size()];
        self.write_into(&mut buf)?;
        Path::from_cdr(buf)
    }

    pub fn encode_into_vec(&self, buf: &mut Vec<u8>) -> Result<(), CdrError> {
        buf.resize(self.size(), 0);
        self.write_into(buf)
    }

    pub fn encode_into_slice(&self, buf: &mut [u8]) -> Result<usize, CdrError> {
        let need = self.size();
        if buf.len() < need {
            return Err(CdrError::BufferTooShort {
                need,
                have: buf.len(),
            });
        }
        self.write_into(&mut buf[..need])?;
        Ok(need)
    }
}

impl<B: AsRef<[u8]> + AsMut<[u8]>> Path<B> {
    /// In-place overwrite of the header `stamp` (no re-encode).
    pub fn set_stamp(&mut self, t: Time) -> Result<(), CdrError> {
        let b = self.buf.as_mut();
        wr_i32(b, CDR_HEADER_SIZE, t.sec)?;
        wr_u32(b, CDR_HEADER_SIZE + 4, t.nanosec)
    }
}

// ── Registry ────────────────────────────────────────────────────────

/// Check if a type name is supported by this module.
pub fn is_type_supported(type_name: &str) -> bool {
    matches!(
        type_name,
        "GridCells" | "MapMetaData" | "OccupancyGrid" | "Odometry" | "Path"
    )
}

/// List all type schema names in this module.
pub fn list_types() -> &'static [&'static str] {
    &[
        "nav_msgs/msg/GridCells",
        "nav_msgs/msg/MapMetaData",
        "nav_msgs/msg/OccupancyGrid",
        "nav_msgs/msg/Odometry",
        "nav_msgs/msg/Path",
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builtin_interfaces::Time;
    use crate::cdr::{decode_fixed, encode_fixed};
    use crate::geometry_msgs::{Point, Pose, Quaternion};

    #[test]
    fn map_meta_data_cdr_size() {
        // Verify standalone CDR_SIZE = 80 (data payload, no encapsulation header).
        assert_eq!(MapMetaData::CDR_SIZE, 80);
        let meta = MapMetaData {
            map_load_time: Time::new(0, 0),
            resolution: 0.05,
            width: 100,
            height: 100,
            origin: Pose {
                position: Point {
                    x: -2.5,
                    y: -2.5,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        };
        let bytes = encode_fixed(&meta).unwrap();
        // Total encoded = CDR_HEADER_SIZE (4) + CDR_SIZE (80) = 84 bytes
        assert_eq!(bytes.len(), 84);
        let decoded = decode_fixed::<MapMetaData>(&bytes).unwrap();
        assert!((decoded.resolution - 0.05_f32).abs() < 1e-6);
        assert_eq!(decoded.width, 100);
        assert_eq!(decoded.height, 100);
        assert!((decoded.origin.position.x - -2.5).abs() < 1e-10);
        assert!((decoded.origin.orientation.w - 1.0).abs() < 1e-10);
    }

    #[test]
    fn map_meta_data_roundtrip() {
        let meta = MapMetaData {
            map_load_time: Time::new(1234, 5678),
            resolution: 0.1,
            width: 200,
            height: 150,
            origin: Pose {
                position: Point {
                    x: 1.0,
                    y: 2.0,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.707,
                    w: 0.707,
                },
            },
        };
        let bytes = encode_fixed(&meta).unwrap();
        let decoded = decode_fixed::<MapMetaData>(&bytes).unwrap();
        assert_eq!(decoded.map_load_time, meta.map_load_time);
        assert!((decoded.resolution - meta.resolution).abs() < 1e-6);
        assert_eq!(decoded.width, meta.width);
        assert_eq!(decoded.height, meta.height);
        assert!((decoded.origin.position.y - 2.0).abs() < 1e-10);
    }

    #[test]
    fn grid_cells_roundtrip() {
        let cells = vec![
            Point {
                x: 1.0,
                y: 2.0,
                z: 0.0,
            },
            Point {
                x: 3.0,
                y: 4.0,
                z: 0.0,
            },
            Point {
                x: 5.0,
                y: 6.0,
                z: 0.0,
            },
        ];
        let gc = GridCells::new(Time::new(1, 0), "map", 0.5, 0.5, &cells).unwrap();
        assert_eq!(gc.len(), 3);
        assert!((gc.cell_width() - 0.5_f32).abs() < 1e-6);
        assert!((gc.cell_height() - 0.5_f32).abs() < 1e-6);

        let bytes = gc.to_cdr();
        let decoded = GridCells::from_cdr(bytes).unwrap();
        assert_eq!(decoded.frame_id(), "map");
        assert_eq!(decoded.len(), 3);
        assert!((decoded.cell(0).unwrap().x - 1.0).abs() < 1e-10);
        assert!((decoded.cell(2).unwrap().y - 6.0).abs() < 1e-10);
        assert!(decoded.cell(3).is_none());
    }

    #[test]
    fn grid_cells_empty() {
        let gc = GridCells::new(Time::new(0, 0), "map", 1.0, 1.0, &[]).unwrap();
        assert!(gc.is_empty());
        assert_eq!(gc.cells(), vec![]);
        let bytes = gc.to_cdr();
        let decoded = GridCells::from_cdr(bytes).unwrap();
        assert!(decoded.is_empty());
    }

    #[test]
    fn occupancy_grid_roundtrip() {
        let info = MapMetaData {
            map_load_time: Time::new(100, 0),
            resolution: 0.05,
            width: 4,
            height: 2,
            origin: Pose {
                position: Point {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        };
        let data: Vec<i8> = vec![0, 50, 100, -1, 0, 0, 0, 0];
        let og = OccupancyGrid::new(Time::new(1, 0), "map", info, &data).unwrap();
        assert_eq!(og.len(), 8);
        assert_eq!(og.data(), data.as_slice());

        let bytes = og.to_cdr();
        let decoded = OccupancyGrid::from_cdr(bytes).unwrap();
        assert_eq!(decoded.frame_id(), "map");
        assert_eq!(decoded.len(), 8);
        assert_eq!(decoded.data()[1], 50);
        assert_eq!(decoded.data()[2], 100);
        assert_eq!(decoded.data()[3], -1);
        let d_info = decoded.info();
        assert!((d_info.resolution - 0.05_f32).abs() < 1e-6);
        assert_eq!(d_info.width, 4);
        assert_eq!(d_info.height, 2);
    }

    #[test]
    fn occupancy_grid_empty_data() {
        let info = MapMetaData {
            map_load_time: Time::new(0, 0),
            resolution: 1.0,
            width: 0,
            height: 0,
            origin: Pose {
                position: Point {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        };
        let og = OccupancyGrid::new(Time::new(0, 0), "map", info, &[]).unwrap();
        assert!(og.is_empty());
        let bytes = og.to_cdr();
        let decoded = OccupancyGrid::from_cdr(bytes).unwrap();
        assert!(decoded.is_empty());
        assert_eq!(decoded.data(), &[]);
    }

    #[test]
    fn path_roundtrip() {
        let pose1 = Pose {
            position: Point {
                x: 1.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        };
        let pose2 = Pose {
            position: Point {
                x: 2.0,
                y: 1.0,
                z: 0.0,
            },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.707,
                w: 0.707,
            },
        };
        let poses = [
            (Time::new(1, 0), "map", pose1),
            (Time::new(2, 0), "map", pose2),
        ];
        let path = Path::new(Time::new(0, 0), "map", &poses).unwrap();
        assert_eq!(path.len(), 2);

        let bytes = path.to_cdr();
        let decoded = Path::from_cdr(bytes).unwrap();
        assert_eq!(decoded.frame_id(), "map");
        assert_eq!(decoded.len(), 2);

        let all = decoded.poses();
        assert_eq!(all.len(), 2);
        assert_eq!(all[0].0, Time::new(1, 0));
        assert_eq!(all[0].1, "map");
        assert!((all[0].2.position.x - 1.0).abs() < 1e-10);
        assert_eq!(all[1].0, Time::new(2, 0));
        assert!((all[1].2.orientation.z - 0.707).abs() < 1e-10);

        let (s, fid, p) = decoded.pose_at(0).unwrap();
        assert_eq!(s, Time::new(1, 0));
        assert_eq!(fid, "map");
        assert!((p.position.x - 1.0).abs() < 1e-10);
    }

    #[test]
    fn path_pose_at_distinct_frame_ids() {
        let pose = Pose {
            position: Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        };
        let poses = [
            (Time::new(1, 0), "base_link", pose),
            (Time::new(2, 0), "odom", pose),
        ];
        let path = Path::new(Time::new(0, 0), "map", &poses).unwrap();
        let (_, fid0, _) = path.pose_at(0).unwrap();
        let (_, fid1, _) = path.pose_at(1).unwrap();
        assert_eq!(fid0, "base_link");
        assert_eq!(fid1, "odom");
    }

    #[test]
    fn path_empty() {
        let path = Path::new(Time::new(0, 0), "map", &[]).unwrap();
        assert!(path.is_empty());
        assert_eq!(path.poses(), vec![]);
        let bytes = path.to_cdr();
        let decoded = Path::from_cdr(bytes).unwrap();
        assert!(decoded.is_empty());
    }

    #[test]
    fn path_variable_frame_ids() {
        let pose = Pose {
            position: Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        };
        let poses = [
            (Time::new(1, 0), "short", pose),
            (Time::new(2, 0), "a_longer_frame_id", pose),
            (Time::new(3, 0), "x", pose),
        ];
        let path = Path::new(Time::new(10, 0), "world", &poses).unwrap();
        let bytes = path.to_cdr();
        let decoded = Path::from_cdr(bytes).unwrap();
        assert_eq!(decoded.len(), 3);
        let all = decoded.poses();
        assert_eq!(all[0].1, "short");
        assert_eq!(all[1].1, "a_longer_frame_id");
        assert_eq!(all[2].1, "x");
    }

    fn sample_pose(x: f64) -> Pose {
        Pose {
            position: Point { x, y: 0.0, z: 0.0 },
            orientation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            },
        }
    }

    #[test]
    fn grid_cells_builder_matches_new() {
        let cells = [
            Point {
                x: 1.0,
                y: 2.0,
                z: 0.0,
            },
            Point {
                x: 3.0,
                y: 4.0,
                z: 0.0,
            },
        ];
        let direct = GridCells::new(Time::new(1, 0), "map", 0.5, 0.25, &cells).unwrap();
        let built = GridCells::builder()
            .stamp(Time::new(1, 0))
            .frame_id("map")
            .cell_width(0.5)
            .cell_height(0.25)
            .cells(&cells)
            .build()
            .unwrap();
        assert_eq!(built.to_cdr(), direct.to_cdr());

        // In-place scalar setters.
        let mut gc = built;
        gc.set_stamp(Time::new(9, 9)).unwrap();
        gc.set_cell_width(2.0).unwrap();
        gc.set_cell_height(3.0).unwrap();
        assert_eq!(gc.stamp(), Time::new(9, 9));
        assert!((gc.cell_width() - 2.0).abs() < 1e-6);
        assert!((gc.cell_height() - 3.0).abs() < 1e-6);
        assert_eq!(gc.len(), 2);
    }

    #[test]
    fn occupancy_grid_builder_matches_new() {
        let info = MapMetaData {
            map_load_time: Time::new(100, 0),
            resolution: 0.05,
            width: 4,
            height: 2,
            origin: sample_pose(0.0),
        };
        let data: [i8; 8] = [0, 50, 100, -1, 0, 50, 100, -1];
        let direct = OccupancyGrid::new(Time::new(1, 0), "map", info, &data).unwrap();
        let built = OccupancyGrid::builder()
            .stamp(Time::new(1, 0))
            .frame_id("map")
            .info(info)
            .data(&data)
            .build()
            .unwrap();
        assert_eq!(built.to_cdr(), direct.to_cdr());
        assert_eq!(built.data(), &data[..]);

        let mut og = built;
        og.set_stamp(Time::new(5, 5)).unwrap();
        assert_eq!(og.stamp(), Time::new(5, 5));
        assert_eq!(og.info().width, 4);
    }

    #[test]
    fn path_builder_matches_new() {
        let poses = [
            (Time::new(1, 0), "a", sample_pose(1.0)),
            (Time::new(2, 0), "longer_frame", sample_pose(2.0)),
        ];
        let direct = Path::new(Time::new(10, 0), "world", &poses).unwrap();
        let built = Path::builder()
            .stamp(Time::new(10, 0))
            .frame_id("world")
            .poses(&poses)
            .build()
            .unwrap();
        assert_eq!(built.to_cdr(), direct.to_cdr());
    }

    #[test]
    fn odometry_builder_matches_new() {
        let pose = PoseWithCovariance {
            pose: sample_pose(1.5),
            covariance: [0.1; 36],
        };
        let twist = TwistWithCovariance {
            twist: Twist {
                linear: Vector3 {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                },
                angular: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.5,
                },
            },
            covariance: [0.2; 36],
        };
        let direct = Odometry::new(Time::new(1, 0), "odom", "base_link", pose, twist).unwrap();
        let built = Odometry::builder()
            .stamp(Time::new(1, 0))
            .frame_id("odom")
            .child_frame_id("base_link")
            .pose(pose)
            .twist(twist)
            .build()
            .unwrap();
        assert_eq!(built.to_cdr(), direct.to_cdr());
        assert_eq!(built.child_frame_id(), "base_link");
        assert_eq!(built.pose().pose.position.x, 1.5);
    }

    #[test]
    fn path_iter_matches_pose_at_and_is_zero_alloc() {
        let poses = [
            (Time::new(1, 0), "a", sample_pose(1.0)),
            (Time::new(2, 0), "bb", sample_pose(2.0)),
            (Time::new(3, 0), "ccc", sample_pose(3.0)),
        ];
        let path = Path::new(Time::new(0, 0), "world", &poses).unwrap();
        let decoded = Path::from_cdr(path.to_cdr()).unwrap();

        // ExactSizeIterator reports the right length.
        assert_eq!(decoded.iter().len(), 3);

        // iter() yields the same tuples as pose_at() for every index.
        for (i, (stamp, frame_id, pose)) in decoded.iter().enumerate() {
            let (s2, f2, p2) = decoded.pose_at(i).unwrap();
            assert_eq!((stamp, frame_id, pose), (s2, f2, p2));
            assert_eq!(frame_id, poses[i].1);
        }
        assert!(decoded.pose_at(3).is_none());
    }
}
