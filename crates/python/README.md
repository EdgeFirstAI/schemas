# edgefirst-schemas (Python)

Zero-copy CDR message types for EdgeFirst Perception. Wraps the
[`edgefirst-schemas`](https://github.com/EdgeFirstAI/schemas) Rust crate via PyO3.

```bash
pip install edgefirst-schemas
```

The wheel ships as `cp311-abi3` (Python 3.11+, single wheel per OS/arch).
For an older Python floor, build from source with `--no-default-features
--features abi3-py38` — see [Build from source](#build-from-source).

## Quick start

```python
from edgefirst.schemas.builtin_interfaces import Time
from edgefirst.schemas.std_msgs import Header
from edgefirst.schemas.sensor_msgs import Image
import numpy as np

pixels = np.zeros((720, 1280, 3), dtype=np.uint8)
img = Image(
    header=Header(stamp=Time(sec=1, nanosec=0), frame_id="cam"),
    height=720, width=1280, encoding="rgb8",
    is_bigendian=0, step=1280 * 3,
    data=pixels,  # any contiguous buffer-protocol object
)

# Zero-copy view of the pixel data — `np.frombuffer` aliases the same
# bytes the message was constructed with, no copy.
arr = np.frombuffer(img.data, dtype=np.uint8).reshape(720, 1280, 3)

# Wire bytes for transport:
buf = img.to_bytes()
img2 = Image.from_cdr(buf)
assert img2.width == 1280
```

### Forwarding raw CDR bytes without a copy

For publish paths where you want to hand the wire bytes straight to a
transport (Zenoh, raw socket, mcap writer), `cdr_view()` exposes the
full CDR buffer (header + payload) as a zero-copy `BorrowedBuf`:

```python
transport.publish(memoryview(img.cdr_view()))   # zero-copy
```

`to_bytes()` is the explicit-copy alternative when the consumer wants
an owned `bytes` value.

### Zero-copy contract — `BorrowedBuf`

Every bulk byte payload (`Image.data`, `Mask.mask`, `RadarCube.cube`,
`PointCloud2.data`, `CompressedVideo.data`, `CompressedImage.data`)
returns a `BorrowedBuf` view that aliases the parent message's bytes —
not a copy. The `BorrowedBuf` holds a strong reference to the parent,
so it's safe to keep it (or a `memoryview` derived from it) live after
the original message reference is dropped.

| Method | Returns | Cost |
|---|---|---|
| `borrowed_buf` itself | wraps the bytes | zero-copy, O(1) |
| `np.frombuffer(borrowed_buf, dtype=...)` | numpy ndarray aliasing the bytes | zero-copy, O(1) (Py 3.11+) |
| `memoryview(borrowed_buf)` | parent-anchored memoryview | zero-copy, O(1) (Py 3.11+) |
| `borrowed_buf.tobytes()` | owned `bytes` | one memcpy |
| `borrowed_buf.view()` | memoryview (Py 3.11+) / bytes (abi3-py38) | zero-copy / one memcpy |

## Migrating from the pycdr2-backed `edgefirst.schemas` (3.x → 4.x)

Version 4 replaces the pure-Python pycdr2 codec with a Rust-backed pyo3
binding. Wire-format bytes are unchanged — anything encoded under 3.x
decodes under 4.x and vice-versa — but the Python API surface is
narrower and stricter.

### What changed at the call site

```python
# 3.x (pycdr2-backed)                    # 4.x (pyo3-backed)
img = Image()                            img = Image(
img.height = 720                             header=Header(stamp=Time(1, 0)),
img.width = 1280                             height=720, width=1280,
img.encoding = "rgb8"                        encoding="rgb8", is_bigendian=0,
img.data = pixels                            step=1280 * 3, data=pixels,
buf = img.serialize()                    )
img2 = Image.deserialize(buf)            buf = img.to_bytes()
                                         img2 = Image.from_cdr(buf)
```

| 3.x pattern | 4.x replacement |
|---|---|
| `Foo()` then field assignment | `Foo(field=value, …)` constructor only — pyclasses are frozen |
| `.serialize()` | `.to_bytes()` |
| `Foo.deserialize(buf)` | `Foo.from_cdr(buf)` |
| `msg.data` returning `bytes` | `msg.data` returning `BorrowedBuf` (zero-copy view); use `.tobytes()` for the old shape |
| `from_schema()`, `decode_pcd()`, `colormap()`, registry helpers | **removed** — the legacy module survives only at `benches/python/legacy/` for benchmark parity |
| `std_msgs.{String, Int32, Float64, …}` primitive wrappers | **removed** — these were pycdr2-generated single-value wrappers; pass raw Python values instead |
| Mutable dataclass-style instances | Frozen pyclasses; rebuild instead of mutate |

### What stays the same

- All field names and types match the ROS 2 / Foxglove / EdgeFirst
  schemas verbatim.
- Wire-format CDR1 LE bytes are byte-equivalent across versions.
- The `edgefirst.schemas.<submodule>` import paths
  (`sensor_msgs.Image`, `std_msgs.Header`, etc.) are preserved.

### abi3-py38 builds — typed numpy caveats

The default wheel is `cp311-abi3`, where the buffer protocol is in the
limited API and typed numpy arrays (`np.uint16`, `np.float32`, …) work
zero-copy as constructor inputs. On the opt-in `abi3-py38` build the
buffer protocol isn't available; pass `arr.tobytes()` for typed arrays,
and `BorrowedBuf.view()` returns `bytes` (one copy) instead of a
parent-anchored memoryview. Plain `bytes` / `bytearray` / `np.uint8`
arrays work on either build.

```python
shape = np.array([2, 128, 12, 128], dtype=np.uint16)

# Default cp311-abi3 wheel — works directly:
RadarCube(..., shape=shape, ...)

# abi3-py38 build — pass bytes:
RadarCube(..., shape=shape.tobytes(), ...)
```

## Build from source

Default (cp311-abi3 wheel for Python 3.11+):

```bash
maturin develop --release --manifest-path crates/python/Cargo.toml
```

abi3-py38 wheel (for embedded targets pinning an older Python):

```bash
maturin build --release \
  --manifest-path crates/python/Cargo.toml \
  --no-default-features --features abi3-py38
```

Cross-compile manylinux2014 wheels via zig:

```bash
maturin build --release --zig --compatibility manylinux2014 \
  --target aarch64-unknown-linux-gnu \
  --manifest-path crates/python/Cargo.toml
```
