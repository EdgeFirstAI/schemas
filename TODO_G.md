# TODO Phase G: TypeScript/JavaScript Support

**Phase:** G
**Status:** Not Started
**Estimate:** 20-30 hours
**Dependencies:** None (can proceed in parallel with other phases)
**Blocks:** Phase F completion (cross-language validation must include TypeScript)

---

## Objective

Create a comprehensive TypeScript/JavaScript schemas package with:
- Clean, modern TypeScript implementation (ES2022+)
- CDR serialization/deserialization matching Rust and Python
- All message types with strong typing
- Unit tests running in browsers (Chrome, Firefox, Safari) and Node.js
- Performance benchmarks across browser engines
- npm package `@edgefirst/schemas`

---

## Reference Implementation

The existing webui at `~/Software/Maivin/webui` contains a working but monolithic JavaScript implementation. This serves as a **requirements reference only** - the schemas package will be a clean-room implementation following modern TypeScript best practices.

**What exists in webui (reference):**
- `src/js/Cdr.js` - CDR reader/writer (866 lines, functional)
- `src/js/boxes.js`, `boxes3d.js` - Detection message parsing
- `src/js/pcd.js` - PointCloud2 parsing
- `src/js/imu.js` - IMU message parsing
- `src/js/stream.js` - H.264 video handling

**Problems with webui implementation:**
- Code duplication (parseTime, parseHeader duplicated across files)
- No TypeScript / no type safety
- Inline hardcoded message schemas
- No tests or benchmarks
- Mixed concerns (parsing + rendering)

---

## Directory Structure

```
schemas/
└── typescript/
    ├── src/
    │   ├── cdr/
    │   │   ├── reader.ts           # CDR deserialization
    │   │   ├── writer.ts           # CDR serialization
    │   │   ├── size-calculator.ts  # Pre-calculate serialized size
    │   │   ├── types.ts            # EncapsulationKind, alignment types
    │   │   └── index.ts            # CDR module exports
    │   ├── types/
    │   │   ├── builtin_interfaces/
    │   │   │   ├── time.ts         # Time message
    │   │   │   ├── duration.ts     # Duration message
    │   │   │   └── index.ts
    │   │   ├── std_msgs/
    │   │   │   ├── header.ts       # Header message
    │   │   │   ├── color-rgba.ts   # ColorRGBA message
    │   │   │   └── index.ts
    │   │   ├── geometry_msgs/
    │   │   │   ├── vector3.ts
    │   │   │   ├── point.ts
    │   │   │   ├── quaternion.ts
    │   │   │   ├── pose.ts
    │   │   │   ├── transform.ts
    │   │   │   ├── twist.ts
    │   │   │   └── index.ts        # All 15 types
    │   │   ├── sensor_msgs/
    │   │   │   ├── image.ts
    │   │   │   ├── point-cloud2.ts
    │   │   │   ├── imu.ts
    │   │   │   ├── camera-info.ts
    │   │   │   ├── nav-sat-fix.ts
    │   │   │   └── index.ts
    │   │   ├── nav_msgs/
    │   │   │   ├── odometry.ts
    │   │   │   ├── path.ts
    │   │   │   └── index.ts
    │   │   ├── foxglove_msgs/
    │   │   │   ├── compressed-video.ts
    │   │   │   ├── image-annotations.ts
    │   │   │   └── index.ts
    │   │   └── edgefirst_msgs/
    │   │       ├── detect.ts
    │   │       ├── detect-box2d.ts
    │   │       ├── detect-track.ts
    │   │       ├── radar-cube.ts
    │   │       ├── radar-info.ts
    │   │       ├── dma-buf.ts
    │   │       ├── mask.ts
    │   │       ├── model.ts
    │   │       ├── model-info.ts
    │   │       └── index.ts
    │   ├── registry.ts             # Message type registry
    │   ├── validation.ts           # Runtime schema validation
    │   └── index.ts                # Main entry point / barrel exports
    ├── tests/
    │   ├── unit/
    │   │   ├── cdr/
    │   │   │   ├── reader.test.ts
    │   │   │   └── writer.test.ts
    │   │   └── types/
    │   │       ├── builtin_interfaces.test.ts
    │   │       ├── std_msgs.test.ts
    │   │       ├── geometry_msgs.test.ts
    │   │       ├── sensor_msgs.test.ts
    │   │       ├── edgefirst_msgs.test.ts
    │   │       └── foxglove_msgs.test.ts
    │   ├── integration/
    │   │   └── roundtrip.test.ts   # Serialize → deserialize roundtrips
    │   └── interop/
    │       └── cross-language.test.ts  # Validate against Rust/Python CDR
    ├── benchmarks/
    │   ├── node/
    │   │   ├── serialization.bench.ts
    │   │   └── deserialization.bench.ts
    │   ├── browser/
    │   │   ├── index.html          # Browser benchmark harness
    │   │   ├── runner.ts           # Benchmark runner
    │   │   └── serialization.bench.ts
    │   └── fixtures/
    │       └── test-data.ts        # Shared test data generators
    ├── package.json
    ├── tsconfig.json
    ├── tsup.config.ts              # Build configuration
    ├── vitest.config.ts            # Test configuration
    └── README.md
```

---

## Phase G.1: Core CDR Implementation (8-10 hrs)

### G.1.1 CdrReader

**File:** `src/cdr/reader.ts`

```typescript
export enum EncapsulationKind {
  CDR_BE = 0x0000,
  CDR_LE = 0x0001,
  PL_CDR_BE = 0x0002,
  PL_CDR_LE = 0x0003,
  CDR2_BE = 0x0006,
  CDR2_LE = 0x0007,
  // ... other variants
}

export class CdrReader {
  private view: DataView;
  private offset: number;
  private littleEndian: boolean;
  private encapsulationKind: EncapsulationKind;

  constructor(data: ArrayBufferView | ArrayBuffer);

  // Primitive readers
  int8(): number;
  uint8(): number;
  int16(): number;
  uint16(): number;
  int32(): number;
  uint32(): number;
  int64(): bigint;
  uint64(): bigint;
  float32(): number;
  float64(): number;

  // Complex readers
  string(): string;
  uint8Array(): Uint8Array;
  int8Array(): Int8Array;
  int16Array(): Int16Array;
  uint16Array(): Uint16Array;
  int32Array(): Int32Array;
  uint32Array(): Uint32Array;
  float32Array(): Float32Array;
  float64Array(): Float64Array;
  stringArray(): string[];

  // Utilities
  align(size: number): void;
  seek(offset: number): void;
  get position(): number;
  get remaining(): number;
}
```

### G.1.2 CdrWriter

**File:** `src/cdr/writer.ts`

```typescript
export class CdrWriter {
  private buffer: ArrayBuffer;
  private view: DataView;
  private offset: number;
  private littleEndian: boolean;

  constructor(options?: { kind?: EncapsulationKind; initialCapacity?: number });

  // Primitive writers
  int8(value: number): void;
  uint8(value: number): void;
  int16(value: number): void;
  uint16(value: number): void;
  int32(value: number): void;
  uint32(value: number): void;
  int64(value: bigint): void;
  uint64(value: bigint): void;
  float32(value: number): void;
  float64(value: number): void;

  // Complex writers
  string(value: string): void;
  uint8Array(value: Uint8Array): void;
  // ... other array writers

  // Utilities
  align(size: number): void;
  toBytes(): Uint8Array;
  get size(): number;
}
```

### G.1.3 Size Calculator

**File:** `src/cdr/size-calculator.ts`

Pre-calculate serialized size without allocating buffers (useful for buffer pre-allocation).

**Acceptance Criteria:**
- [ ] CdrReader handles all CDR encapsulation kinds
- [ ] CdrWriter produces valid CDR bytes
- [ ] Alignment is correctly handled for all primitive types
- [ ] Big-endian and little-endian support
- [ ] 100% test coverage for CDR module

---

## Phase G.2: Message Type Definitions (8-12 hrs)

### Message Type Pattern

Each message type follows this pattern:

```typescript
// src/types/std_msgs/header.ts
import { CdrReader, CdrWriter } from '../../cdr';
import { Time } from '../builtin_interfaces';

export interface Header {
  stamp: Time;
  frame_id: string;
}

export const Header = {
  /**
   * Creates a Header with default values.
   */
  create(partial?: Partial<Header>): Header {
    return {
      stamp: Time.create(),
      frame_id: '',
      ...partial,
    };
  },

  /**
   * Serializes a Header to CDR bytes.
   */
  serialize(msg: Header): Uint8Array {
    const writer = new CdrWriter();
    Time.write(writer, msg.stamp);
    writer.string(msg.frame_id);
    return writer.toBytes();
  },

  /**
   * Deserializes a Header from CDR bytes.
   */
  deserialize(data: ArrayBufferView | ArrayBuffer): Header {
    const reader = new CdrReader(data);
    return Header.read(reader);
  },

  /**
   * Reads a Header from an existing CdrReader (for nested types).
   */
  read(reader: CdrReader): Header {
    return {
      stamp: Time.read(reader),
      frame_id: reader.string(),
    };
  },

  /**
   * Writes a Header to an existing CdrWriter (for nested types).
   */
  write(writer: CdrWriter, msg: Header): void {
    Time.write(writer, msg.stamp);
    writer.string(msg.frame_id);
  },
};
```

### Priority Message Types

**Priority 1 - Heavy messages (benchmark targets):**
- FoxgloveCompressedVideo
- RadarCube
- PointCloud2
- Image
- Mask

**Priority 2 - Common robotics types:**
- IMU
- CameraInfo
- Pose, PoseStamped
- Transform, TransformStamped
- Twist, TwistStamped

**Priority 3 - EdgeFirst custom:**
- Detect, DetectBox2D, DetectTrack
- Model, ModelInfo
- RadarInfo, DmaBuf

**Acceptance Criteria:**
- [ ] All message types from Rust library implemented
- [ ] TypeScript interfaces match Rust struct fields exactly
- [ ] Nested types properly composed
- [ ] serialize/deserialize roundtrip works for all types

---

## Phase G.3: Unit Testing (4-6 hrs)

### Test Framework: Vitest

Vitest is chosen for:
- Native TypeScript support
- Fast execution
- Browser mode support
- Compatible with Jest API

**Configuration (`vitest.config.ts`):**
```typescript
import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    include: ['tests/**/*.test.ts'],
    environment: 'node',
    coverage: {
      provider: 'v8',
      reporter: ['text', 'json', 'html'],
      thresholds: {
        statements: 80,
        branches: 80,
        functions: 80,
        lines: 80,
      },
    },
  },
});
```

### Test Categories

**CDR Tests (`tests/unit/cdr/`):**
- Primitive type roundtrips (int8, uint8, ..., float64)
- String encoding/decoding (ASCII, UTF-8, empty, long)
- Array handling (empty, small, large)
- Alignment verification
- Encapsulation kind handling
- Error cases (truncated data, invalid UTF-8)

**Message Type Tests (`tests/unit/types/`):**
- Default value creation
- Field assignment
- Serialize/deserialize roundtrip
- Nested type handling
- Edge cases (empty arrays, max values, NaN/Inf)

**Integration Tests (`tests/integration/`):**
- Full message roundtrips
- Complex nested structures
- Large payload handling

### Browser Testing

**Configuration (`vitest.config.browser.ts`):**
```typescript
import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    include: ['tests/**/*.test.ts'],
    browser: {
      enabled: true,
      name: 'chromium', // Also: 'firefox', 'webkit'
      provider: 'playwright',
    },
  },
});
```

**npm scripts:**
```json
{
  "scripts": {
    "test": "vitest run",
    "test:watch": "vitest",
    "test:browser": "vitest run --config vitest.config.browser.ts",
    "test:browser:chrome": "vitest run --config vitest.config.browser.ts --browser.name=chromium",
    "test:browser:firefox": "vitest run --config vitest.config.browser.ts --browser.name=firefox",
    "test:browser:safari": "vitest run --config vitest.config.browser.ts --browser.name=webkit"
  }
}
```

**Acceptance Criteria:**
- [ ] All tests pass in Node.js
- [ ] All tests pass in Chrome (Chromium)
- [ ] All tests pass in Firefox
- [ ] All tests pass in Safari (WebKit)
- [ ] ≥80% code coverage

---

## Phase G.4: Performance Benchmarks (4-6 hrs)

### Benchmark Framework: Vitest Bench

Vitest includes built-in benchmarking support using Tinybench.

**Configuration (`vitest.config.bench.ts`):**
```typescript
import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    benchmark: {
      include: ['benchmarks/**/*.bench.ts'],
      reporters: ['default', 'json'],
      outputFile: 'benchmarks/results/results.json',
    },
  },
});
```

### Benchmark Targets

**Heavy Messages (mandatory benchmarks):**

| Message | Payload Sizes |
|---------|---------------|
| FoxgloveCompressedVideo | 10KB, 100KB, 1MB |
| RadarCube | 64x64x8, 128x128x16, 256x256x32 |
| PointCloud2 | 1K, 10K, 100K, 1M points |
| Image | 640x480, 1280x720, 1920x1080 |
| Mask | 640x480, 1280x720 |

**Benchmark Template:**
```typescript
// benchmarks/node/serialization.bench.ts
import { bench, describe } from 'vitest';
import { PointCloud2 } from '../../src/types/sensor_msgs';
import { generatePointCloud } from '../fixtures/test-data';

describe('PointCloud2 Serialization', () => {
  const pc1k = generatePointCloud(1000);
  const pc10k = generatePointCloud(10000);
  const pc100k = generatePointCloud(100000);

  bench('serialize 1K points', () => {
    PointCloud2.serialize(pc1k);
  });

  bench('serialize 10K points', () => {
    PointCloud2.serialize(pc10k);
  });

  bench('serialize 100K points', () => {
    PointCloud2.serialize(pc100k);
  });
});

describe('PointCloud2 Deserialization', () => {
  const bytes1k = PointCloud2.serialize(generatePointCloud(1000));
  const bytes10k = PointCloud2.serialize(generatePointCloud(10000));
  const bytes100k = PointCloud2.serialize(generatePointCloud(100000));

  bench('deserialize 1K points', () => {
    PointCloud2.deserialize(bytes1k);
  });

  bench('deserialize 10K points', () => {
    PointCloud2.deserialize(bytes10k);
  });

  bench('deserialize 100K points', () => {
    PointCloud2.deserialize(bytes100k);
  });
});
```

### Browser Benchmarks

**HTML Harness (`benchmarks/browser/index.html`):**
```html
<!DOCTYPE html>
<html>
<head>
  <title>EdgeFirst Schemas Benchmarks</title>
</head>
<body>
  <h1>EdgeFirst Schemas Browser Benchmarks</h1>
  <div id="results"></div>
  <script type="module" src="./runner.js"></script>
</body>
</html>
```

**npm scripts:**
```json
{
  "scripts": {
    "bench": "vitest bench",
    "bench:browser": "vitest bench --config vitest.config.browser.ts"
  }
}
```

**Acceptance Criteria:**
- [ ] Benchmarks run in Node.js
- [ ] Benchmarks run in Chrome, Firefox, Safari
- [ ] Results exported as JSON for comparison
- [ ] Baseline results documented for each engine

---

## Phase G.5: Build Tooling (2-3 hrs)

### tsup Configuration

**File:** `tsup.config.ts`
```typescript
import { defineConfig } from 'tsup';

export default defineConfig({
  entry: ['src/index.ts'],
  format: ['esm', 'cjs'],
  dts: true,
  splitting: true,
  treeshake: true,
  clean: true,
  minify: false,  // Minified browser bundle separate
  outDir: 'dist',
});
```

### Build Outputs

```
dist/
├── index.js          # ESM (for bundlers)
├── index.cjs         # CommonJS (for legacy Node)
├── index.d.ts        # TypeScript declarations
├── index.d.cts       # CommonJS declarations
└── browser/
    └── edgefirst-schemas.min.js  # Minified IIFE for <script>
```

### package.json

```json
{
  "name": "@edgefirst/schemas",
  "version": "1.0.0",
  "description": "EdgeFirst Perception message schemas with CDR serialization",
  "type": "module",
  "main": "./dist/index.cjs",
  "module": "./dist/index.js",
  "types": "./dist/index.d.ts",
  "exports": {
    ".": {
      "import": {
        "types": "./dist/index.d.ts",
        "default": "./dist/index.js"
      },
      "require": {
        "types": "./dist/index.d.cts",
        "default": "./dist/index.cjs"
      }
    }
  },
  "files": [
    "dist",
    "README.md"
  ],
  "scripts": {
    "build": "tsup",
    "build:browser": "tsup --format iife --globalName EdgeFirstSchemas --minify --outDir dist/browser",
    "test": "vitest run",
    "test:watch": "vitest",
    "test:browser": "vitest run --config vitest.config.browser.ts",
    "test:coverage": "vitest run --coverage",
    "bench": "vitest bench",
    "bench:browser": "vitest bench --config vitest.config.browser.ts",
    "lint": "eslint src tests",
    "typecheck": "tsc --noEmit",
    "prepublishOnly": "npm run build && npm run test"
  },
  "devDependencies": {
    "@playwright/test": "^1.40.0",
    "@types/node": "^20.10.0",
    "@vitest/browser": "^1.0.0",
    "@vitest/coverage-v8": "^1.0.0",
    "eslint": "^8.55.0",
    "playwright": "^1.40.0",
    "tsup": "^8.0.0",
    "typescript": "^5.3.0",
    "vitest": "^1.0.0"
  },
  "engines": {
    "node": ">=18.0.0"
  },
  "repository": {
    "type": "git",
    "url": "https://github.com/EdgeFirstAI/schemas.git",
    "directory": "typescript"
  },
  "license": "Apache-2.0",
  "keywords": [
    "edgefirst",
    "perception",
    "schemas",
    "cdr",
    "ros2",
    "robotics",
    "sensor-fusion"
  ]
}
```

**Acceptance Criteria:**
- [ ] `npm run build` produces ESM + CJS + .d.ts
- [ ] `npm run build:browser` produces minified IIFE bundle
- [ ] Package imports work in Node.js (ESM and CJS)
- [ ] Package imports work in browser bundlers (Vite, webpack)
- [ ] Package works via `<script>` tag

---

## Phase G.6: Cross-Language Validation (2-4 hrs)

### CDR Interoperability Tests

**Test Cases:**
1. TypeScript serialize → Rust deserialize
2. Rust serialize → TypeScript deserialize
3. TypeScript serialize → Python deserialize
4. Python serialize → TypeScript deserialize

**Test Data Generation:**
- Generate test fixtures in each language
- Export as binary CDR files
- Cross-validate in other languages

**File:** `tests/interop/cross-language.test.ts`
```typescript
import { describe, it, expect } from 'vitest';
import { readFileSync } from 'fs';
import { Time, Header, PointCloud2 } from '../../src';

describe('Cross-Language CDR Validation', () => {
  describe('Rust → TypeScript', () => {
    it('deserializes Time from Rust', () => {
      const rustBytes = readFileSync('fixtures/rust/time.cdr');
      const time = Time.deserialize(rustBytes);
      expect(time.sec).toBe(1234567890);
      expect(time.nanosec).toBe(123456789);
    });

    it('deserializes PointCloud2 from Rust', () => {
      const rustBytes = readFileSync('fixtures/rust/pointcloud2.cdr');
      const pc = PointCloud2.deserialize(rustBytes);
      expect(pc.width).toBe(1000);
      // ... validate all fields
    });
  });

  describe('TypeScript → Rust', () => {
    it('produces compatible Time bytes', () => {
      const time = Time.create({ sec: 1234567890, nanosec: 123456789 });
      const bytes = Time.serialize(time);
      // Compare with expected Rust output or use Rust test harness
    });
  });
});
```

**Acceptance Criteria:**
- [ ] All message types roundtrip correctly across TypeScript ↔ Rust
- [ ] All message types roundtrip correctly across TypeScript ↔ Python
- [ ] Test fixtures generated and committed to repository

---

## Validation Checklist

### Before Merging

- [ ] `npm run build` succeeds
- [ ] `npm run test` passes (Node.js)
- [ ] `npm run test:browser` passes (Chrome, Firefox, Safari)
- [ ] `npm run test:coverage` shows ≥80% coverage
- [ ] `npm run bench` completes with baseline metrics
- [ ] `npm run lint` passes
- [ ] `npm run typecheck` passes
- [ ] Cross-language validation passes
- [ ] README.md documents usage

### npm Publish Checklist

- [ ] Version bumped appropriately
- [ ] CHANGELOG.md updated
- [ ] `npm publish --dry-run` succeeds
- [ ] Package size is reasonable (<100KB minified)

---

## Success Criteria

| Criterion | Target |
|-----------|--------|
| All message types implemented | 100% |
| Node.js test pass rate | 100% |
| Chrome test pass rate | 100% |
| Firefox test pass rate | 100% |
| Safari test pass rate | 100% |
| Code coverage | ≥80% |
| Cross-language CDR compatibility | 100% |
| Benchmark baselines documented | Yes |

---

## File Changes Summary

| File | Action |
|------|--------|
| `typescript/` | Create directory structure |
| `typescript/package.json` | Create npm package config |
| `typescript/tsconfig.json` | Create TypeScript config |
| `typescript/tsup.config.ts` | Create build config |
| `typescript/vitest.config.ts` | Create test config |
| `typescript/src/**/*.ts` | Create CDR + message types |
| `typescript/tests/**/*.test.ts` | Create unit + integration tests |
| `typescript/benchmarks/**/*.bench.ts` | Create benchmarks |
| `typescript/README.md` | Create package documentation |

---

**Next Phase:** Integration with Phase F (Cross-Language Validation) to include TypeScript in the validation matrix.
