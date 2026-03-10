// SPDX-License-Identifier: Apache-2.0
// Copyright © 2025 Au-Zone Technologies. All Rights Reserved.

//! Zero-copy CDR (Common Data Representation) infrastructure.
//!
//! This module implements CDR1 Little-Endian serialization as used by ROS 2 DDS.
//! Every CDR buffer starts with a 4-byte encapsulation header (`00 01 00 00`)
//! followed by payload fields aligned to their natural boundaries (u32 → 4,
//! f64 → 8, etc.). Strings are length-prefixed and NUL-terminated.
//!
//! ## Type categories
//!
//! **`CdrFixed` types** — fixed-size, `Copy` structs (Time, Vector3, Pose, …).
//! Use `encode_fixed` / `decode_fixed` for serialization.
//!
//! **Buffer-backed types** — generic `Type<B: AsRef<[u8]>>` wrappers around a CDR
//! byte buffer. On construction (`from_cdr`), a single scan builds a small
//! offset table for O(1) field access. These types live in their respective
//! package modules (e.g. `sensor_msgs::Image`).
//!
//! ## Internal helpers
//!
//! The `rd_*` / `wr_*` functions provide unchecked reads/writes at known absolute
//! offsets, used by buffer-backed types after validation. These are `pub(crate)`
//! and not part of the public API.

use std::fmt;

/// CDR1 Little-Endian encapsulation header.
pub const CDR_LE_HEADER: [u8; 4] = [0x00, 0x01, 0x00, 0x00];

/// Size of the CDR encapsulation header in bytes.
pub const CDR_HEADER_SIZE: usize = 4;

// ── Error ────────────────────────────────────────────────────────────

/// Errors that can occur during CDR read/write operations.
#[derive(Debug)]
pub enum CdrError {
    /// Buffer is too short for the requested operation.
    BufferTooShort { need: usize, have: usize },
    /// A CDR string contained invalid UTF-8.
    InvalidUtf8,
    /// A CDR string was missing its NUL terminator.
    MissingNul,
    /// The CDR encapsulation header was invalid.
    InvalidHeader,
    /// A boolean value was not 0 or 1.
    InvalidBool(u8),
}

impl fmt::Display for CdrError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CdrError::BufferTooShort { need, have } => {
                write!(
                    f,
                    "CDR buffer too short: need {} bytes, have {}",
                    need, have
                )
            }
            CdrError::InvalidUtf8 => write!(f, "CDR string contains invalid UTF-8"),
            CdrError::MissingNul => write!(f, "CDR string missing NUL terminator"),
            CdrError::InvalidHeader => write!(f, "invalid CDR encapsulation header"),
            CdrError::InvalidBool(v) => write!(f, "invalid CDR bool value: {}", v),
        }
    }
}

impl std::error::Error for CdrError {}

// ── Alignment ────────────────────────────────────────────────────────

/// Align `pos` up to the next multiple of `n` (power of two).
#[inline(always)]
pub const fn align(pos: usize, n: usize) -> usize {
    (pos + n - 1) & !(n - 1)
}

/// Align an absolute buffer position to `n` bytes relative to the CDR data
/// start (after the 4-byte encapsulation header).
///
/// Use this instead of [`align`] when computing field positions in accessors
/// that need >4-byte alignment (e.g. f64 fields need 8-byte alignment).
/// For 4-byte alignment, `cdr_align` and `align` produce the same result
/// because `CDR_HEADER_SIZE` (4) is itself 4-byte aligned.
#[inline(always)]
pub const fn cdr_align(pos: usize, n: usize) -> usize {
    CDR_HEADER_SIZE + align(pos - CDR_HEADER_SIZE, n)
}

// ── CdrCursor — zero-copy reader ─────────────────────────────────────

/// A zero-copy cursor for reading CDR-encoded data from a byte buffer.
pub struct CdrCursor<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> CdrCursor<'a> {
    /// Create a new cursor over `buf`, starting after the 4-byte CDR header.
    pub fn new(buf: &'a [u8]) -> Result<Self, CdrError> {
        if buf.len() < CDR_HEADER_SIZE {
            return Err(CdrError::BufferTooShort {
                need: CDR_HEADER_SIZE,
                have: buf.len(),
            });
        }
        if buf[0..4] != CDR_LE_HEADER {
            return Err(CdrError::InvalidHeader);
        }
        Ok(CdrCursor {
            buf,
            pos: CDR_HEADER_SIZE,
        })
    }

    /// Resume scanning from a known-valid buffer at the given offset.
    pub fn resume(buf: &'a [u8], offset: usize) -> Self {
        CdrCursor { buf, pos: offset }
    }

    /// Current byte offset (including the CDR header).
    #[inline(always)]
    pub fn offset(&self) -> usize {
        self.pos
    }

    /// Remaining bytes in the buffer.
    #[inline(always)]
    pub fn remaining(&self) -> usize {
        self.buf.len() - self.pos
    }

    /// Align the cursor position to `n` bytes relative to the data start.
    ///
    /// CDR alignment is computed from the beginning of the serialized data
    /// (after the 4-byte encapsulation header), not from the buffer start.
    #[inline(always)]
    pub fn align(&mut self, n: usize) {
        self.pos = CDR_HEADER_SIZE + align(self.pos - CDR_HEADER_SIZE, n);
    }

    /// Skip `n` bytes, checking bounds.
    #[inline(always)]
    pub fn skip(&mut self, n: usize) -> Result<(), CdrError> {
        self.ensure(n)?;
        self.pos += n;
        Ok(())
    }

    /// Set position to an absolute offset.
    #[inline(always)]
    pub fn set_pos(&mut self, pos: usize) {
        self.pos = pos;
    }

    #[inline(always)]
    fn ensure(&self, n: usize) -> Result<(), CdrError> {
        if self.pos + n > self.buf.len() {
            Err(CdrError::BufferTooShort {
                need: self.pos + n,
                have: self.buf.len(),
            })
        } else {
            Ok(())
        }
    }

    // ── Primitive readers ────────────────────────────────────────────

    pub fn read_u8(&mut self) -> Result<u8, CdrError> {
        self.ensure(1)?;
        let v = self.buf[self.pos];
        self.pos += 1;
        Ok(v)
    }

    pub fn read_i8(&mut self) -> Result<i8, CdrError> {
        Ok(self.read_u8()? as i8)
    }

    pub fn read_bool(&mut self) -> Result<bool, CdrError> {
        let v = self.read_u8()?;
        match v {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(CdrError::InvalidBool(v)),
        }
    }

    pub fn read_u16(&mut self) -> Result<u16, CdrError> {
        self.align(2);
        self.ensure(2)?;
        let v = u16::from_le_bytes([self.buf[self.pos], self.buf[self.pos + 1]]);
        self.pos += 2;
        Ok(v)
    }

    pub fn read_i16(&mut self) -> Result<i16, CdrError> {
        Ok(self.read_u16()? as i16)
    }

    pub fn read_u32(&mut self) -> Result<u32, CdrError> {
        self.align(4);
        self.ensure(4)?;
        let v = u32::from_le_bytes(
            self.buf[self.pos..self.pos + 4]
                .try_into()
                .expect("slice is exactly 4 bytes after bounds check"),
        );
        self.pos += 4;
        Ok(v)
    }

    pub fn read_i32(&mut self) -> Result<i32, CdrError> {
        Ok(self.read_u32()? as i32)
    }

    pub fn read_u64(&mut self) -> Result<u64, CdrError> {
        self.align(8);
        self.ensure(8)?;
        let v = u64::from_le_bytes(
            self.buf[self.pos..self.pos + 8]
                .try_into()
                .expect("slice is exactly 8 bytes after bounds check"),
        );
        self.pos += 8;
        Ok(v)
    }

    pub fn read_i64(&mut self) -> Result<i64, CdrError> {
        Ok(self.read_u64()? as i64)
    }

    pub fn read_f32(&mut self) -> Result<f32, CdrError> {
        Ok(f32::from_bits(self.read_u32()?))
    }

    pub fn read_f64(&mut self) -> Result<f64, CdrError> {
        Ok(f64::from_bits(self.read_u64()?))
    }

    // ── Variable-length readers ──────────────────────────────────────

    /// Read a CDR sequence length (u32).
    pub fn read_seq_len(&mut self) -> Result<u32, CdrError> {
        self.read_u32()
    }

    /// Read a CDR string as a zero-copy `&str`.
    ///
    /// CDR strings are encoded as: u32 length (including NUL), UTF-8 bytes, NUL.
    pub fn read_string(&mut self) -> Result<&'a str, CdrError> {
        let len = self.read_u32()? as usize;
        if len == 0 {
            return Ok("");
        }
        self.ensure(len)?;
        let bytes = &self.buf[self.pos..self.pos + len];
        self.pos += len;
        // CDR strings include the NUL terminator in the length
        if bytes[len - 1] != 0 {
            return Err(CdrError::MissingNul);
        }
        let s = std::str::from_utf8(&bytes[..len - 1]).map_err(|_| CdrError::InvalidUtf8)?;
        Ok(s)
    }

    /// Read a CDR byte sequence as a zero-copy `&[u8]`.
    pub fn read_bytes(&mut self) -> Result<&'a [u8], CdrError> {
        let len = self.read_u32()? as usize;
        self.ensure(len)?;
        let bytes = &self.buf[self.pos..self.pos + len];
        self.pos += len;
        Ok(bytes)
    }

    /// Skip over `count` elements of size `elem_size`, aligning to `elem_size`.
    ///
    /// On little-endian targets, CDR1-LE primitive arrays are already in native
    /// byte order, so the `from_cdr` scan only needs to advance past them.
    fn skip_typed_seq(&mut self, count: usize, elem_size: usize) -> Result<(), CdrError> {
        if count > 0 {
            self.align(elem_size);
            let byte_len = count * elem_size;
            self.ensure(byte_len)?;
            self.pos += byte_len;
        }
        Ok(())
    }

    /// Skip over `count` u16/i16 values (2-byte aligned).
    pub fn skip_seq_2(&mut self, count: usize) -> Result<(), CdrError> {
        self.skip_typed_seq(count, 2)
    }

    /// Skip over `count` u32/i32/f32 values (4-byte aligned).
    pub fn skip_seq_4(&mut self, count: usize) -> Result<(), CdrError> {
        self.skip_typed_seq(count, 4)
    }

    /// Skip over `count` u64/i64/f64 values (8-byte aligned).
    pub fn skip_seq_8(&mut self, count: usize) -> Result<(), CdrError> {
        self.skip_typed_seq(count, 8)
    }

    /// Read `count` raw bytes without a length prefix (zero-copy).
    pub fn read_raw(&mut self, count: usize) -> Result<&'a [u8], CdrError> {
        self.ensure(count)?;
        let bytes = &self.buf[self.pos..self.pos + count];
        self.pos += count;
        Ok(bytes)
    }
}

// ── CdrWriter — buffer writer ────────────────────────────────────────

/// A cursor for writing CDR-encoded data into a mutable byte buffer.
///
/// Write methods use deferred error handling: on buffer overflow, the error
/// is recorded internally and all subsequent writes become no-ops. Call
/// [`finish()`](CdrWriter::finish) after all writes to check for errors.
pub struct CdrWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
    err: Option<CdrError>,
}

impl<'a> CdrWriter<'a> {
    /// Create a new writer, writing the CDR LE header at the start.
    pub fn new(buf: &'a mut [u8]) -> Result<Self, CdrError> {
        if buf.len() < CDR_HEADER_SIZE {
            return Err(CdrError::BufferTooShort {
                need: CDR_HEADER_SIZE,
                have: buf.len(),
            });
        }
        buf[0..4].copy_from_slice(&CDR_LE_HEADER);
        Ok(CdrWriter {
            buf,
            pos: CDR_HEADER_SIZE,
            err: None,
        })
    }

    /// Current byte offset (including CDR header).
    #[inline(always)]
    pub fn offset(&self) -> usize {
        self.pos
    }

    /// Align position to `n` bytes relative to the data start, writing zeros into padding.
    ///
    /// CDR alignment is computed from the beginning of the serialized data
    /// (after the 4-byte encapsulation header), not from the buffer start.
    #[inline(always)]
    pub fn align(&mut self, n: usize) {
        if self.err.is_some() {
            return;
        }
        let aligned = CDR_HEADER_SIZE + align(self.pos - CDR_HEADER_SIZE, n);
        if aligned > self.buf.len() {
            self.err = Some(CdrError::BufferTooShort {
                need: aligned,
                have: self.buf.len(),
            });
            return;
        }
        for i in self.pos..aligned {
            self.buf[i] = 0;
        }
        self.pos = aligned;
    }

    /// Check bounds for `need` bytes at the current position.
    /// Returns `false` and records an error on overflow.
    #[inline(always)]
    fn check(&mut self, need: usize) -> bool {
        if self.err.is_some() {
            return false;
        }
        if self.pos + need > self.buf.len() {
            self.err = Some(CdrError::BufferTooShort {
                need: self.pos + need,
                have: self.buf.len(),
            });
            return false;
        }
        true
    }

    /// Check for deferred write errors. Call after all writes are complete.
    pub fn finish(self) -> Result<(), CdrError> {
        match self.err {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    // ── Primitive writers ────────────────────────────────────────────

    pub fn write_u8(&mut self, v: u8) {
        if !self.check(1) {
            return;
        }
        self.buf[self.pos] = v;
        self.pos += 1;
    }

    pub fn write_i8(&mut self, v: i8) {
        self.write_u8(v as u8);
    }

    pub fn write_bool(&mut self, v: bool) {
        self.write_u8(v as u8);
    }

    pub fn write_u16(&mut self, v: u16) {
        self.align(2);
        if !self.check(2) {
            return;
        }
        self.buf[self.pos..self.pos + 2].copy_from_slice(&v.to_le_bytes());
        self.pos += 2;
    }

    pub fn write_i16(&mut self, v: i16) {
        self.write_u16(v as u16);
    }

    pub fn write_u32(&mut self, v: u32) {
        self.align(4);
        if !self.check(4) {
            return;
        }
        self.buf[self.pos..self.pos + 4].copy_from_slice(&v.to_le_bytes());
        self.pos += 4;
    }

    pub fn write_i32(&mut self, v: i32) {
        self.write_u32(v as u32);
    }

    pub fn write_u64(&mut self, v: u64) {
        self.align(8);
        if !self.check(8) {
            return;
        }
        self.buf[self.pos..self.pos + 8].copy_from_slice(&v.to_le_bytes());
        self.pos += 8;
    }

    pub fn write_i64(&mut self, v: i64) {
        self.write_u64(v as u64);
    }

    pub fn write_f32(&mut self, v: f32) {
        self.write_u32(v.to_bits());
    }

    pub fn write_f64(&mut self, v: f64) {
        self.write_u64(v.to_bits());
    }

    // ── Variable-length writers ──────────────────────────────────────

    /// Write a CDR string: u32(len+1) + UTF-8 bytes + NUL.
    pub fn write_string(&mut self, s: &str) {
        let len = s.len() + 1; // include NUL
        self.write_u32(len as u32);
        if !self.check(s.len() + 1) {
            return;
        }
        self.buf[self.pos..self.pos + s.len()].copy_from_slice(s.as_bytes());
        self.pos += s.len();
        self.buf[self.pos] = 0; // NUL
        self.pos += 1;
    }

    /// Write a CDR byte sequence: u32(len) + raw bytes.
    pub fn write_bytes(&mut self, data: &[u8]) {
        self.write_u32(data.len() as u32);
        if !self.check(data.len()) {
            return;
        }
        self.buf[self.pos..self.pos + data.len()].copy_from_slice(data);
        self.pos += data.len();
    }

    /// Write raw bytes without a length prefix.
    pub fn write_raw(&mut self, data: &[u8]) {
        if !self.check(data.len()) {
            return;
        }
        self.buf[self.pos..self.pos + data.len()].copy_from_slice(data);
        self.pos += data.len();
    }

    /// Write a typed primitive slice as raw bytes with proper alignment.
    ///
    /// On little-endian targets, CDR1-LE primitives share native byte order,
    /// so a `&[u16]`, `&[i16]`, `&[u32]`, `&[f32]`, `&[f64]` etc. can be
    /// written as a single `memcpy` after aligning to `elem_size`.
    fn write_typed_slice(&mut self, data: &[u8], elem_size: usize) {
        if !data.is_empty() {
            self.align(elem_size);
            if !self.check(data.len()) {
                return;
            }
            self.buf[self.pos..self.pos + data.len()].copy_from_slice(data);
            self.pos += data.len();
        }
    }

    /// Write a `&[u16]` (or `&[i16]`) slice as bulk bytes.
    pub fn write_slice_u16(&mut self, data: &[u16]) {
        let bytes =
            unsafe { std::slice::from_raw_parts(data.as_ptr() as *const u8, data.len() * 2) };
        self.write_typed_slice(bytes, 2);
    }

    /// Write a `&[i16]` slice as bulk bytes.
    pub fn write_slice_i16(&mut self, data: &[i16]) {
        let bytes =
            unsafe { std::slice::from_raw_parts(data.as_ptr() as *const u8, data.len() * 2) };
        self.write_typed_slice(bytes, 2);
    }

    /// Write a `&[u32]` (or `&[i32]`) slice as bulk bytes.
    pub fn write_slice_u32(&mut self, data: &[u32]) {
        let bytes =
            unsafe { std::slice::from_raw_parts(data.as_ptr() as *const u8, data.len() * 4) };
        self.write_typed_slice(bytes, 4);
    }

    /// Write a `&[f32]` slice as bulk bytes.
    pub fn write_slice_f32(&mut self, data: &[f32]) {
        let bytes =
            unsafe { std::slice::from_raw_parts(data.as_ptr() as *const u8, data.len() * 4) };
        self.write_typed_slice(bytes, 4);
    }

    /// Write a `&[f64]` slice as bulk bytes.
    pub fn write_slice_f64(&mut self, data: &[f64]) {
        let bytes =
            unsafe { std::slice::from_raw_parts(data.as_ptr() as *const u8, data.len() * 8) };
        self.write_typed_slice(bytes, 8);
    }
}

// ── CdrSizer — dry-run for size calculation ──────────────────────────

/// A dry-run writer that only tracks the byte offset (no buffer).
///
/// Used to compute the exact buffer size needed before writing.
pub struct CdrSizer {
    pos: usize,
}

impl Default for CdrSizer {
    fn default() -> Self {
        Self::new()
    }
}

impl CdrSizer {
    /// Create a new sizer starting after the CDR header.
    pub fn new() -> Self {
        CdrSizer {
            pos: CDR_HEADER_SIZE,
        }
    }

    /// Current byte offset (including CDR header).
    #[inline(always)]
    pub fn offset(&self) -> usize {
        self.pos
    }

    /// Total size needed for the CDR buffer.
    #[inline(always)]
    pub fn size(&self) -> usize {
        self.pos
    }

    /// Align position to `n` bytes relative to the data start.
    #[inline(always)]
    pub fn align(&mut self, n: usize) {
        self.pos = CDR_HEADER_SIZE + align(self.pos - CDR_HEADER_SIZE, n);
    }

    // ── Primitive sizers ─────────────────────────────────────────────

    pub fn size_u8(&mut self) {
        self.pos += 1;
    }

    pub fn size_i8(&mut self) {
        self.pos += 1;
    }

    pub fn size_bool(&mut self) {
        self.pos += 1;
    }

    pub fn size_u16(&mut self) {
        self.align(2);
        self.pos += 2;
    }

    pub fn size_i16(&mut self) {
        self.size_u16();
    }

    pub fn size_u32(&mut self) {
        self.align(4);
        self.pos += 4;
    }

    pub fn size_i32(&mut self) {
        self.size_u32();
    }

    pub fn size_u64(&mut self) {
        self.align(8);
        self.pos += 8;
    }

    pub fn size_i64(&mut self) {
        self.size_u64();
    }

    pub fn size_f32(&mut self) {
        self.size_u32();
    }

    pub fn size_f64(&mut self) {
        self.size_u64();
    }

    // ── Variable-length sizers ───────────────────────────────────────

    pub fn size_string(&mut self, s: &str) {
        self.size_u32(); // length prefix
        self.pos += s.len() + 1; // string + NUL
    }

    pub fn size_bytes(&mut self, len: usize) {
        self.size_u32(); // length prefix
        self.pos += len;
    }

    pub fn size_raw(&mut self, len: usize) {
        self.pos += len;
    }

    /// Size a sequence of `count` elements of `elem_size` bytes with alignment.
    fn size_typed_seq(&mut self, count: usize, elem_size: usize) {
        if count > 0 {
            self.align(elem_size);
            self.pos += count * elem_size;
        }
    }

    /// Size `count` u16/i16 elements (2 bytes each, 2-byte aligned).
    pub fn size_seq_2(&mut self, count: usize) {
        self.size_typed_seq(count, 2);
    }

    /// Size `count` u32/i32/f32 elements (4 bytes each, 4-byte aligned).
    pub fn size_seq_4(&mut self, count: usize) {
        self.size_typed_seq(count, 4);
    }

    /// Size `count` u64/i64/f64 elements (8 bytes each, 8-byte aligned).
    pub fn size_seq_8(&mut self, count: usize) {
        self.size_typed_seq(count, 8);
    }
}

// ── CdrFixed trait ───────────────────────────────────────────────────

/// Trait for fully-fixed-size CDR types (no strings, no Vecs).
///
/// Types implementing this trait are `Copy` and have a constant CDR wire size.
/// They can be embedded in buffer-backed composite types without needing
/// offset tables.
pub trait CdrFixed: Copy + Sized {
    /// The wire size of this type in CDR (not counting any encapsulation header).
    const CDR_SIZE: usize;

    /// Read this type from the cursor (cursor is already past the CDR header).
    fn read_cdr(cursor: &mut CdrCursor<'_>) -> Result<Self, CdrError>;

    /// Write this type to the writer (writer is already past the CDR header).
    fn write_cdr(&self, writer: &mut CdrWriter<'_>);

    /// Advance the sizer by this type's CDR size.
    fn size_cdr(sizer: &mut CdrSizer);
}

// ── Inline helpers: read/write primitives at known absolute offsets ──

#[inline(always)]
pub(crate) fn rd_u8(b: &[u8], pos: usize) -> u8 {
    b[pos]
}

#[inline(always)]
pub(crate) fn rd_bool(b: &[u8], pos: usize) -> bool {
    b[pos] != 0
}

#[inline(always)]
pub(crate) fn rd_u32(b: &[u8], pos: usize) -> u32 {
    u32::from_le_bytes(
        b[pos..pos + 4]
            .try_into()
            .expect("slice is exactly 4 bytes"),
    )
}

#[inline(always)]
pub(crate) fn rd_i32(b: &[u8], pos: usize) -> i32 {
    rd_u32(b, pos) as i32
}

#[inline(always)]
pub(crate) fn rd_u64(b: &[u8], pos: usize) -> u64 {
    u64::from_le_bytes(
        b[pos..pos + 8]
            .try_into()
            .expect("slice is exactly 8 bytes"),
    )
}

#[inline(always)]
pub(crate) fn rd_f32(b: &[u8], pos: usize) -> f32 {
    f32::from_bits(rd_u32(b, pos))
}

#[inline(always)]
pub(crate) fn rd_f64(b: &[u8], pos: usize) -> f64 {
    f64::from_bits(rd_u64(b, pos))
}

#[inline(always)]
pub(crate) fn wr_u32(b: &mut [u8], pos: usize, v: u32) -> Result<(), CdrError> {
    if pos + 4 > b.len() {
        return Err(CdrError::BufferTooShort {
            need: pos + 4,
            have: b.len(),
        });
    }
    b[pos..pos + 4].copy_from_slice(&v.to_le_bytes());
    Ok(())
}

#[inline(always)]
pub(crate) fn wr_i32(b: &mut [u8], pos: usize, v: i32) -> Result<(), CdrError> {
    wr_u32(b, pos, v as u32)
}

#[inline(always)]
pub(crate) fn wr_u8(b: &mut [u8], pos: usize, v: u8) -> Result<(), CdrError> {
    if pos >= b.len() {
        return Err(CdrError::BufferTooShort {
            need: pos + 1,
            have: b.len(),
        });
    }
    b[pos] = v;
    Ok(())
}

/// Read a CDR string at `pos`: u32 len (already aligned), then bytes.
/// Returns the string slice and the byte position AFTER the string (including NUL).
#[inline]
pub(crate) fn rd_string(b: &[u8], pos: usize) -> (&str, usize) {
    let p = align(pos, 4);
    let len = rd_u32(b, p) as usize;
    if len <= 1 {
        ("", p + 4 + len)
    } else {
        let start = p + 4;
        let s = std::str::from_utf8(&b[start..start + len - 1]).unwrap_or("");
        (s, start + len)
    }
}

/// Read a CDR byte sequence at `pos`: u32 len (aligned), then raw bytes.
/// Returns the slice and byte position AFTER the data.
#[inline]
pub(crate) fn rd_bytes(b: &[u8], pos: usize) -> (&[u8], usize) {
    let p = align(pos, 4);
    let len = rd_u32(b, p) as usize;
    let start = p + 4;
    (&b[start..start + len], start + len)
}

/// Read a Time (i32 + u32) at the given position (must be 4-byte aligned).
#[inline(always)]
pub(crate) fn rd_time(b: &[u8], pos: usize) -> crate::builtin_interfaces::Time {
    crate::builtin_interfaces::Time {
        sec: rd_i32(b, pos),
        nanosec: rd_u32(b, pos + 4),
    }
}

/// Read a Duration (i32 + u32) at the given position (must be 4-byte aligned).
#[inline(always)]
pub(crate) fn rd_duration(b: &[u8], pos: usize) -> crate::builtin_interfaces::Duration {
    crate::builtin_interfaces::Duration {
        sec: rd_i32(b, pos),
        nanosec: rd_u32(b, pos + 4),
    }
}

// ── Zero-copy typed slice views ──────────────────────────────────────
//
// On little-endian targets, CDR1-LE primitive arrays share native byte order.
// These helpers reinterpret a `&[u8]` region as a typed `&[T]` slice with
// no copying or per-element conversion.
//
// Safety: the caller must ensure `pos` is properly aligned for the element
// type and `pos + count * size_of::<T>()` is within bounds. These invariants
// are guaranteed by the `from_cdr` scan that validated alignment and length.
//
// Alignment note: `Vec<u8>` technically has align_of::<u8>() = 1, but all
// real-world allocators (glibc, jemalloc, mimalloc, Windows HeapAlloc)
// return memory aligned to at least 8 or 16 bytes, so
// `b.as_ptr().add(pos)` is properly aligned when `pos` satisfies CDR
// alignment rules. The debug_assert! below catches any violation during
// testing, ensuring these functions are only called at valid offsets.

/// View a region of `b` as `&[u16]` (zero-copy on LE targets).
#[inline(always)]
pub(crate) fn rd_slice_u16(b: &[u8], pos: usize, count: usize) -> &[u16] {
    let ptr = b[pos..].as_ptr();
    debug_assert!(
        (ptr as usize).is_multiple_of(std::mem::align_of::<u16>()),
        "rd_slice_u16: misaligned pointer"
    );
    unsafe { std::slice::from_raw_parts(ptr as *const u16, count) }
}

/// View a region of `b` as `&[i16]` (zero-copy on LE targets).
#[inline(always)]
pub(crate) fn rd_slice_i16(b: &[u8], pos: usize, count: usize) -> &[i16] {
    let ptr = b[pos..].as_ptr();
    debug_assert!(
        (ptr as usize).is_multiple_of(std::mem::align_of::<i16>()),
        "rd_slice_i16: misaligned pointer"
    );
    unsafe { std::slice::from_raw_parts(ptr as *const i16, count) }
}

/// View a region of `b` as `&[u32]` (zero-copy on LE targets).
#[inline(always)]
pub(crate) fn rd_slice_u32(b: &[u8], pos: usize, count: usize) -> &[u32] {
    let ptr = b[pos..].as_ptr();
    debug_assert!(
        (ptr as usize).is_multiple_of(std::mem::align_of::<u32>()),
        "rd_slice_u32: misaligned pointer"
    );
    unsafe { std::slice::from_raw_parts(ptr as *const u32, count) }
}

/// View a region of `b` as `&[f32]` (zero-copy on LE targets).
#[inline(always)]
pub(crate) fn rd_slice_f32(b: &[u8], pos: usize, count: usize) -> &[f32] {
    let ptr = b[pos..].as_ptr();
    debug_assert!(
        (ptr as usize).is_multiple_of(std::mem::align_of::<f32>()),
        "rd_slice_f32: misaligned pointer"
    );
    unsafe { std::slice::from_raw_parts(ptr as *const f32, count) }
}

/// Encode a CdrFixed type to a new CDR buffer (with header).
///
/// The buffer is pre-sized by [`CdrSizer`], so write errors cannot occur
/// under normal conditions. Returns `Err` only if the sizer and writer
/// logic diverge (indicates a library bug).
pub fn encode_fixed<T: CdrFixed>(val: &T) -> Result<Vec<u8>, CdrError> {
    let mut sizer = CdrSizer::new();
    T::size_cdr(&mut sizer);
    let mut buf = vec![0u8; sizer.size()];
    let mut writer = CdrWriter::new(&mut buf)?;
    val.write_cdr(&mut writer);
    writer.finish()?;
    Ok(buf)
}

/// Helper to decode a CdrFixed type from a CDR buffer (with header).
pub fn decode_fixed<T: CdrFixed>(buf: &[u8]) -> Result<T, CdrError> {
    let mut cursor = CdrCursor::new(buf)?;
    T::read_cdr(&mut cursor)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn align_basic() {
        assert_eq!(align(0, 4), 0);
        assert_eq!(align(1, 4), 4);
        assert_eq!(align(4, 4), 4);
        assert_eq!(align(5, 4), 8);
        assert_eq!(align(0, 8), 0);
        assert_eq!(align(1, 8), 8);
        assert_eq!(align(7, 8), 8);
        assert_eq!(align(8, 8), 8);
        assert_eq!(align(0, 2), 0);
        assert_eq!(align(1, 2), 2);
        assert_eq!(align(2, 2), 2);
        assert_eq!(align(3, 2), 4);
    }

    #[test]
    fn cursor_invalid_header() {
        let buf = [0x00, 0x00, 0x00, 0x00, 0x42];
        assert!(CdrCursor::new(&buf).is_err());
    }

    #[test]
    fn cursor_too_short() {
        let buf = [0x00, 0x01, 0x00];
        assert!(CdrCursor::new(&buf).is_err());
    }

    #[test]
    fn cursor_read_primitives() {
        // Build a known CDR buffer manually.
        // Alignment is relative to data start (after the 4-byte CDR header).
        let mut buf = vec![0x00, 0x01, 0x00, 0x00]; // CDR header
        buf.push(42u8); // u8 at data offset 0, buf offset 4
        buf.push(0); // padding for u16 alignment (data offset 1 → 2)
        buf.extend_from_slice(&1000u16.to_le_bytes()); // u16 at data offset 2, buf offset 6
        buf.extend_from_slice(&123456u32.to_le_bytes()); // u32 at data offset 4, buf offset 8
                                                         // u64 at data offset 8, already 8-byte aligned — no padding needed
        buf.extend_from_slice(&9876543210u64.to_le_bytes()); // u64 at data offset 8, buf offset 12

        let mut cursor = CdrCursor::new(&buf).unwrap();
        assert_eq!(cursor.read_u8().unwrap(), 42);
        assert_eq!(cursor.read_u16().unwrap(), 1000);
        assert_eq!(cursor.read_u32().unwrap(), 123456);
        assert_eq!(cursor.read_u64().unwrap(), 9876543210);
    }

    #[test]
    fn cursor_read_i32_negative() {
        let mut buf = vec![0x00, 0x01, 0x00, 0x00];
        buf.extend_from_slice(&(-42i32).to_le_bytes());
        let mut cursor = CdrCursor::new(&buf).unwrap();
        assert_eq!(cursor.read_i32().unwrap(), -42);
    }

    #[test]
    fn cursor_read_f32_f64() {
        // f32 at data offset 0 (buf 4..8), f64 at data offset 8 (buf 12..20)
        // f64 needs data-relative alignment: data offset 4 → 8 (4 bytes padding)
        let mut buf = vec![0x00, 0x01, 0x00, 0x00]; // CDR header (offset 0..4)
        buf.extend_from_slice(&std::f32::consts::PI.to_le_bytes()); // f32 at buf 4..8
        buf.extend_from_slice(&[0; 4]); // padding for f64 alignment (data offset 4 → 8)
        buf.extend_from_slice(&std::f64::consts::E.to_le_bytes()); // f64 at buf 12..20

        let mut cursor = CdrCursor::new(&buf).unwrap();
        assert!((cursor.read_f32().unwrap() - std::f32::consts::PI).abs() < 1e-7);
        assert!((cursor.read_f64().unwrap() - std::f64::consts::E).abs() < 1e-15);
    }

    #[test]
    fn cursor_read_bool() {
        let buf = vec![0x00, 0x01, 0x00, 0x00, 0, 1, 2];
        let mut cursor = CdrCursor::new(&buf).unwrap();
        assert!(!cursor.read_bool().unwrap());
        assert!(cursor.read_bool().unwrap());
        assert!(cursor.read_bool().is_err()); // 2 is invalid
    }

    #[test]
    fn cursor_read_string() {
        let mut buf = vec![0x00, 0x01, 0x00, 0x00];
        // "hello" → len=6 (including NUL)
        buf.extend_from_slice(&6u32.to_le_bytes());
        buf.extend_from_slice(b"hello\0");

        let mut cursor = CdrCursor::new(&buf).unwrap();
        assert_eq!(cursor.read_string().unwrap(), "hello");
    }

    #[test]
    fn cursor_read_empty_string() {
        let mut buf = vec![0x00, 0x01, 0x00, 0x00];
        // Empty CDR string: len=1, just NUL
        buf.extend_from_slice(&1u32.to_le_bytes());
        buf.push(0);

        let mut cursor = CdrCursor::new(&buf).unwrap();
        assert_eq!(cursor.read_string().unwrap(), "");
    }

    #[test]
    fn cursor_read_zero_len_string() {
        let mut buf = vec![0x00, 0x01, 0x00, 0x00];
        buf.extend_from_slice(&0u32.to_le_bytes());

        let mut cursor = CdrCursor::new(&buf).unwrap();
        assert_eq!(cursor.read_string().unwrap(), "");
    }

    #[test]
    fn cursor_read_bytes() {
        let mut buf = vec![0x00, 0x01, 0x00, 0x00];
        buf.extend_from_slice(&3u32.to_le_bytes());
        buf.extend_from_slice(&[10, 20, 30]);

        let mut cursor = CdrCursor::new(&buf).unwrap();
        assert_eq!(cursor.read_bytes().unwrap(), &[10, 20, 30]);
    }

    #[test]
    fn writer_primitives_roundtrip() {
        let mut buf = [0u8; 64];
        {
            let mut w = CdrWriter::new(&mut buf).unwrap();
            w.write_u8(42);
            w.write_u16(1000);
            w.write_u32(123456);
            w.write_u64(9876543210);
            w.write_i32(-42);
            w.write_f32(std::f32::consts::PI);
            w.write_f64(std::f64::consts::E);
            w.write_bool(true);
            w.write_bool(false);
        }

        let mut cursor = CdrCursor::new(&buf).unwrap();
        assert_eq!(cursor.read_u8().unwrap(), 42);
        assert_eq!(cursor.read_u16().unwrap(), 1000);
        assert_eq!(cursor.read_u32().unwrap(), 123456);
        assert_eq!(cursor.read_u64().unwrap(), 9876543210);
        assert_eq!(cursor.read_i32().unwrap(), -42);
        assert!((cursor.read_f32().unwrap() - std::f32::consts::PI).abs() < 1e-7);
        assert!((cursor.read_f64().unwrap() - std::f64::consts::E).abs() < 1e-15);
        assert!(cursor.read_bool().unwrap());
        assert!(!cursor.read_bool().unwrap());
    }

    #[test]
    fn writer_string_roundtrip() {
        let mut buf = [0u8; 64];
        {
            let mut w = CdrWriter::new(&mut buf).unwrap();
            w.write_string("hello");
            w.write_string("");
            w.write_string("world");
        }

        let mut cursor = CdrCursor::new(&buf).unwrap();
        assert_eq!(cursor.read_string().unwrap(), "hello");
        assert_eq!(cursor.read_string().unwrap(), "");
        assert_eq!(cursor.read_string().unwrap(), "world");
    }

    #[test]
    fn writer_bytes_roundtrip() {
        let mut buf = [0u8; 32];
        {
            let mut w = CdrWriter::new(&mut buf).unwrap();
            w.write_bytes(&[1, 2, 3, 4, 5]);
        }

        let mut cursor = CdrCursor::new(&buf).unwrap();
        assert_eq!(cursor.read_bytes().unwrap(), &[1, 2, 3, 4, 5]);
    }

    #[test]
    fn sizer_matches_writer() {
        let test_string = "hello world";
        let test_bytes = [1u8, 2, 3, 4, 5];

        let mut sizer = CdrSizer::new();
        sizer.size_u8();
        sizer.size_u16();
        sizer.size_u32();
        sizer.size_u64();
        sizer.size_i32();
        sizer.size_f32();
        sizer.size_f64();
        sizer.size_bool();
        sizer.size_string(test_string);
        sizer.size_bytes(test_bytes.len());

        let mut buf = vec![0u8; sizer.size()];
        let pos = {
            let mut w = CdrWriter::new(&mut buf).unwrap();
            w.write_u8(1);
            w.write_u16(2);
            w.write_u32(3);
            w.write_u64(4);
            w.write_i32(-5);
            w.write_f32(6.0);
            w.write_f64(7.0);
            w.write_bool(true);
            w.write_string(test_string);
            w.write_bytes(&test_bytes);
            w.offset()
        };

        assert_eq!(sizer.size(), pos, "sizer and writer disagree on total size");
    }

    // ── Wire-compatibility tests: CdrFixed roundtrip ───────────────

    /// Verify CdrFixed encode → decode roundtrip preserves values.
    fn assert_roundtrip<T: CdrFixed + PartialEq + std::fmt::Debug>(val: &T, name: &str) {
        let bytes = encode_fixed(val).unwrap();
        let decoded = decode_fixed::<T>(&bytes).unwrap();
        assert_eq!(*val, decoded, "{}: CdrFixed roundtrip failed", name);
    }

    #[test]
    fn roundtrip_time() {
        use crate::builtin_interfaces::Time;
        assert_roundtrip(&Time::new(0, 0), "zero");
        assert_roundtrip(&Time::new(42, 123456789), "typical");
        assert_roundtrip(&Time::new(-100, 500_000_000), "negative");
        assert_roundtrip(&Time::new(i32::MAX, 999_999_999), "max");
    }

    #[test]
    fn roundtrip_duration() {
        use crate::builtin_interfaces::Duration;
        assert_roundtrip(&Duration { sec: 0, nanosec: 0 }, "zero");
        assert_roundtrip(
            &Duration {
                sec: 5,
                nanosec: 500_000_000,
            },
            "typical",
        );
    }

    #[test]
    fn roundtrip_vector3() {
        use crate::geometry_msgs::Vector3;
        assert_roundtrip(
            &Vector3 {
                x: 1.5,
                y: -2.5,
                z: 3.0,
            },
            "vec3",
        );
    }

    #[test]
    fn roundtrip_quaternion() {
        use crate::geometry_msgs::Quaternion;
        assert_roundtrip(
            &Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.707,
                w: 0.707,
            },
            "quat",
        );
    }

    #[test]
    fn roundtrip_pose() {
        use crate::geometry_msgs::{Point, Pose, Quaternion};
        assert_roundtrip(
            &Pose {
                position: Point {
                    x: 1.0,
                    y: 2.0,
                    z: 3.0,
                },
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
            "pose",
        );
    }

    #[test]
    fn roundtrip_color_rgba() {
        use crate::std_msgs::ColorRGBA;
        assert_roundtrip(
            &ColorRGBA {
                r: 1.0,
                g: 0.5,
                b: 0.0,
                a: 1.0,
            },
            "color",
        );
    }

    #[test]
    fn roundtrip_service_header() {
        use crate::service::ServiceHeader;
        assert_roundtrip(
            &ServiceHeader {
                guid: 12345678901234567i64,
                seq: 42,
            },
            "svc_header",
        );
    }

    #[test]
    fn roundtrip_clock() {
        use crate::builtin_interfaces::Time;
        use crate::rosgraph_msgs::Clock;
        assert_roundtrip(
            &Clock {
                clock: Time::new(100, 500_000_000),
            },
            "clock",
        );
    }

    #[test]
    fn roundtrip_nav_sat_status() {
        use crate::sensor_msgs::NavSatStatus;
        assert_roundtrip(
            &NavSatStatus {
                status: 0,
                service: 1,
            },
            "nav_sat_status",
        );
    }

    #[test]
    fn roundtrip_region_of_interest() {
        use crate::sensor_msgs::RegionOfInterest;
        assert_roundtrip(
            &RegionOfInterest {
                x_offset: 10,
                y_offset: 20,
                height: 100,
                width: 200,
                do_rectify: true,
            },
            "roi",
        );
    }

    #[test]
    fn roundtrip_date() {
        use crate::edgefirst_msgs::Date;
        assert_roundtrip(
            &Date {
                year: 2025,
                month: 6,
                day: 15,
            },
            "date",
        );
    }

    #[test]
    fn roundtrip_inertia() {
        use crate::geometry_msgs::{Inertia, Vector3};
        assert_roundtrip(
            &Inertia {
                m: 10.0,
                com: Vector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                ixx: 1.0,
                ixy: 0.0,
                ixz: 0.0,
                iyy: 1.0,
                iyz: 0.0,
                izz: 1.0,
            },
            "inertia",
        );
    }

    #[test]
    fn roundtrip_foxglove_circle() {
        use crate::builtin_interfaces::Time;
        use crate::foxglove_msgs::{FoxgloveCircleAnnotations, FoxgloveColor, FoxglovePoint2};
        assert_roundtrip(
            &FoxgloveCircleAnnotations {
                timestamp: Time::new(100, 0),
                position: FoxglovePoint2 { x: 320.0, y: 240.0 },
                diameter: 50.0,
                thickness: 2.0,
                fill_color: FoxgloveColor {
                    r: 1.0,
                    g: 0.0,
                    b: 0.0,
                    a: 0.5,
                },
                outline_color: FoxgloveColor {
                    r: 0.0,
                    g: 1.0,
                    b: 0.0,
                    a: 1.0,
                },
            },
            "foxglove_circle",
        );
    }
}
