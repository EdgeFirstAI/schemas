# EdgeFirst Schemas C Test Suite

Comprehensive C test suite using the Criterion testing framework.

## Prerequisites

**macOS:**
```bash
brew install criterion
```

**Ubuntu/Debian:**
```bash
sudo apt-get install libcriterion-dev
```

**Build from source (if not in package manager):**
```bash
git clone https://github.com/Snaipe/Criterion
cd Criterion
meson build
ninja -C build
sudo ninja -C build install
```

## Building and Running Tests

**Build library and compile tests:**
```bash
make all
```

**Run all tests:**
```bash
make test
```

**Run individual test suite:**
```bash
./test_builtin_interfaces
./test_std_msgs
./test_geometry_msgs
./test_edgefirst_msgs
./test_sensor_msgs
./test_errno
```

**Verbose output for debugging:**
```bash
./test_builtin_interfaces --verbose
```

**Filter tests by pattern:**
```bash
./test_builtin_interfaces --filter time_serialize
```

**Clean build artifacts:**
```bash
make clean
```

## Test Coverage

### test_builtin_interfaces.c (17 tests)
- Time: create, set, serialize, deserialize, zero/negative values, errno
- Duration: create, set, serialize, deserialize, zero/negative values, errno

### test_std_msgs.c (14 tests)
- Header: create, set frame_id, set stamp, long strings, special chars, serialization, errno
- ColorRGBA: create, set RGBA values, serialization, errno

### test_geometry_msgs.c (11 tests)
- Vector3: create, set, serialize
- Point: create, serialize
- Quaternion: create, identity validation, serialize
- Pose: create, set position/orientation, serialize
- Transform: create, serialize

### test_edgefirst_msgs.c (17 tests)
- DetectTrack: create, set, serialize
- DetectBox2D: create, normalized coords, set, serialize
- Detect: create, set label/score, serialize with/without boxes
- Mask: create, set dimensions/encoding/data, serialize with/without data
- DmaBuf: create, set dimensions/fourcc/fd, serialize

### test_sensor_msgs.c (16 tests)
- PointField: create, set name/values, datatype constants, serialize
- PointCloud2: create, set dimensions, set data, serialize
- NavSatStatus: create, service/status constants, set, serialize
- NavSatFix: create, set position/covariance, serialize

### test_errno.c (22 tests)
- Serialization errno: NULL pointer, NULL output parameters
- Deserialization errno: NULL buffer, zero length, invalid data, truncated data
- Setter errno: NULL pointers, NULL strings, NULL arrays
- Getter errno: NULL pointer safety
- Sequential errors and error recovery
- Destructor NULL safety

**Total: 97 tests covering all message types and error handling**

## Test Organization

Each test file follows the pattern:
1. **Create and destroy tests** - Basic object lifecycle
2. **Setter/getter tests** - Field manipulation
3. **Serialization round-trip tests** - CDR serialization correctness
4. **Edge case tests** - Zero values, empty strings, boundary conditions
5. **Errno tests** - Error handling validation

## Criterion Features Used

- `cr_assert()` - Basic assertions
- `cr_assert_eq()` - Equality assertions
- `cr_assert_neq()` - Inequality assertions
- `cr_assert_null()` - NULL pointer assertions
- `cr_assert_not_null()` - Non-NULL pointer assertions
- `cr_assert_str_eq()` - String equality assertions
- `cr_assert_float_eq()` - Floating point equality with tolerance
- `cr_assert_gt()` - Greater than assertions

## Memory Safety

All tests properly clean up resources:
- Call `_destroy()` for all created objects
- Call `edgefirst_buffer_destroy()` for serialization buffers
- Verify NULL safety of all functions

## Continuous Integration

These tests are designed to run in CI/CD pipelines:
- Exit code 0 on success, non-zero on failure
- Verbose output for debugging failures
- Fast execution (< 1 second total)

## Extending Tests

To add new tests:

1. Add test function with `Test(suite_name, test_name)` macro
2. Use `cr_assert_*()` macros for validation
3. Clean up all allocated resources
4. Add test source to `TEST_SOURCES` in Makefile
5. Run `make test` to verify

Example:
```c
Test(my_suite, my_test) {
    MyType *obj = my_type_create();
    cr_assert_not_null(obj);
    
    // Test code here
    
    my_type_destroy(obj);
}
```

## Troubleshooting

**Criterion not found:**
- Ensure Criterion is installed (`brew install criterion` or `apt-get install libcriterion-dev`)
- Check include path: `-I/usr/local/include`
- Check library path: `-L/usr/local/lib`

**Library not found at runtime:**
- Check rpath: `-Wl,-rpath,../../target/release`
- Or set `LD_LIBRARY_PATH`: `export LD_LIBRARY_PATH=../../target/release:$LD_LIBRARY_PATH`

**Tests fail to compile:**
- Ensure Rust library is built: `cd ../.. && cargo build --release`
- Check header is up to date: `include/edgefirst/schemas.h`

**Tests crash:**
- Run with verbose output: `./test_name --verbose`
- Use debugger: `lldb ./test_name` or `gdb ./test_name`
- Check for NULL pointer dereferences
- Verify memory is properly freed

## License

Apache-2.0 - See LICENSE file in repository root.
