# Python CDR Benchmarks

## Summary

**Target Hardware:** rpi5-hailo  
**Python:** Python 3.13.5  
**Last updated:** 20260427T152927Z  
**Git revision:** `468cd76a0328`

EdgeFirst Python bindings (pyo3) vs. **pycdr2** (pure Python) and **cyclonedds-python** (C++-backed bindings). All three implementations encode/decode identical CDR1 LE messages with the fixture shapes defined in [`benches/python/shapes.py`](benches/python/shapes.py), which mirror [`benches/cpp/common.hpp`](benches/cpp/common.hpp) — the C++ comparison lives in [BENCHMARKS.md](BENCHMARKS.md).

> Lower is better. Each chart shows three bars per row — **blue = edgefirst (pyo3)**, **emerald = cyclone-dds-py**, **violet = pycdr2**. Charts use a logarithmic x-axis when the dynamic range exceeds ~100×; per-bar labels are suppressed in favour of the click-to-expand numeric tables. Regenerate locally with `./benches/python/benchmark.sh --target <ssh-host> --render`.

**Coverage:** 82 fixtures × 3 implementations (edgefirst (pyo3), cyclone-dds-py, pycdr2) = 246 measurements.

**Image: decode latency by resolution × encoding**

_Camera frame (RGB / YUYV / NV12)._

![Image/decode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22VGA%20RGB%22%2C%22VGA%20YUYV%22%2C%22VGA%20NV12%22%2C%22HD%20RGB%22%2C%22HD%20YUYV%22%2C%22HD%20NV12%22%2C%22FHD%20RGB%22%2C%22FHD%20YUYV%22%2C%22FHD%20NV12%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22edgefirst%20%28pyo3%29%22%2C%22data%22%3A%5B72852.0%2C55889.0%2C23500.0%2C405409.0%2C278167.5%2C186075.0%2C1372040.5%2C620001.0%2C454556.0%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22cyclone-dds-py%22%2C%22data%22%3A%5B7346898.5%2C4948402.0%2C3602047.0%2C22219410.0%2C14694122.0%2C11013751.0%2C60826714.0%2C40101956.0%2C25137065.5%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22pycdr2%22%2C%22data%22%3A%5B7255779.0%2C4759661.5%2C3614398.0%2C22267023.0%2C14668725.0%2C11001399.5%2C60364588.0%2C39758449.0%2C25118576.5%5D%2C%22backgroundColor%22%3A%22rgba%28139%2C92%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28139%2C92%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Image/decode%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=620&h=384)

<details>
<summary>Numeric values — Image decode (click to expand)</summary>

| Variant | edgefirst (pyo3) | cyclone-dds-py | pycdr2 | cdds / native | pycdr2 / native |
|---|---|---|---|---|---|
| VGA RGB | 72.9 µs | 7.3 ms | 7.3 ms | **101×** | **100×** |
| VGA YUYV | 55.9 µs | 4.9 ms | 4.8 ms | **89×** | **85×** |
| VGA NV12 | 23.5 µs | 3.6 ms | 3.6 ms | **153×** | **154×** |
| HD RGB | 405 µs | 22.2 ms | 22.3 ms | **55×** | **55×** |
| HD YUYV | 278 µs | 14.7 ms | 14.7 ms | **53×** | **53×** |
| HD NV12 | 186 µs | 11.0 ms | 11.0 ms | **59×** | **59×** |
| FHD RGB | 1.4 ms | 60.8 ms | 60.4 ms | **44×** | **44×** |
| FHD YUYV | 620 µs | 40.1 ms | 39.8 ms | **65×** | **64×** |
| FHD NV12 | 455 µs | 25.1 ms | 25.1 ms | **55×** | **55×** |

</details>

**Image: encode latency by resolution × encoding**

![Image/encode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22VGA%20RGB%22%2C%22VGA%20YUYV%22%2C%22VGA%20NV12%22%2C%22HD%20RGB%22%2C%22HD%20YUYV%22%2C%22HD%20NV12%22%2C%22FHD%20RGB%22%2C%22FHD%20YUYV%22%2C%22FHD%20NV12%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22edgefirst%20%28pyo3%29%22%2C%22data%22%3A%5B86463.5%2C39593.0%2C21963.0%2C380483.0%2C302168.0%2C137111.0%2C972373.0%2C608205.0%2C452427.0%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22cyclone-dds-py%22%2C%22data%22%3A%5B27815683.0%2C17709564.0%2C13937342.0%2C83016169.0%2C53497435.0%2C42119017.0%2C192911010.0%2C125835614.0%2C88812564.0%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22pycdr2%22%2C%22data%22%3A%5B27895565.0%2C17814556.5%2C13888824.0%2C82934897.0%2C52840827.0%2C41434018.0%2C188612189.0%2C123765294.0%2C90337390.0%5D%2C%22backgroundColor%22%3A%22rgba%28139%2C92%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28139%2C92%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Image/encode%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=620&h=384)

<details>
<summary>Numeric values — Image encode (click to expand)</summary>

| Variant | edgefirst (pyo3) | cyclone-dds-py | pycdr2 | cdds / native | pycdr2 / native |
|---|---|---|---|---|---|
| VGA RGB | 86.5 µs | 27.8 ms | 27.9 ms | **322×** | **323×** |
| VGA YUYV | 39.6 µs | 17.7 ms | 17.8 ms | **447×** | **450×** |
| VGA NV12 | 22.0 µs | 13.9 ms | 13.9 ms | **635×** | **632×** |
| HD RGB | 380 µs | 83.0 ms | 82.9 ms | **218×** | **218×** |
| HD YUYV | 302 µs | 53.5 ms | 52.8 ms | **177×** | **175×** |
| HD NV12 | 137 µs | 42.1 ms | 41.4 ms | **307×** | **302×** |
| FHD RGB | 972 µs | 193 ms | 189 ms | **198×** | **194×** |
| FHD YUYV | 608 µs | 126 ms | 124 ms | **207×** | **203×** |
| FHD NV12 | 452 µs | 88.8 ms | 90.3 ms | **196×** | **200×** |

</details>

**RadarCube: decode latency by DRVEGRD range mode**

_SmartMicro DRVEGRD radar cube tensors. DRVEGRD-169 = 12 virtual RX channels (3TX × 4RX); DRVEGRD-171 = 48 virtual RX (6TX × 8RX)._

![RadarCube/decode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22DRVEGRD-169%20Ultra-Short%22%2C%22DRVEGRD-169%20Short%22%2C%22DRVEGRD-169%20Long%22%2C%22DRVEGRD-171%20Short%22%2C%22DRVEGRD-171%20Extra-Long%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22edgefirst%20%28pyo3%29%22%2C%22data%22%3A%5B8130.0%2C87204.0%2C588215.0%2C611724.0%2C2734248.0%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22cyclone-dds-py%22%2C%22data%22%3A%5B822354.0%2C3732408.0%2C15908208.0%2C15836523.0%2C67233250.0%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22pycdr2%22%2C%22data%22%3A%5B819576.0%2C3749029.0%2C16061886.5%2C16085636.0%2C67911202.0%5D%2C%22backgroundColor%22%3A%22rgba%28139%2C92%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28139%2C92%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22RadarCube/decode%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=620&h=240)

<details>
<summary>Numeric values — RadarCube decode (click to expand)</summary>

| Variant | edgefirst (pyo3) | cyclone-dds-py | pycdr2 | cdds / native | pycdr2 / native |
|---|---|---|---|---|---|
| DRVEGRD-169 Ultra-Short | 8.1 µs | 822 µs | 820 µs | **101×** | **101×** |
| DRVEGRD-169 Short | 87.2 µs | 3.7 ms | 3.7 ms | **43×** | **43×** |
| DRVEGRD-169 Long | 588 µs | 15.9 ms | 16.1 ms | **27×** | **27×** |
| DRVEGRD-171 Short | 612 µs | 15.8 ms | 16.1 ms | **26×** | **26×** |
| DRVEGRD-171 Extra-Long | 2.7 ms | 67.2 ms | 67.9 ms | **25×** | **25×** |

</details>

**RadarCube: encode latency by DRVEGRD range mode**

![RadarCube/encode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22DRVEGRD-169%20Ultra-Short%22%2C%22DRVEGRD-169%20Short%22%2C%22DRVEGRD-169%20Long%22%2C%22DRVEGRD-171%20Short%22%2C%22DRVEGRD-171%20Extra-Long%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22edgefirst%20%28pyo3%29%22%2C%22data%22%3A%5B9833.0%2C96629.5%2C573326.0%2C581455.0%2C2233005.5%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22cyclone-dds-py%22%2C%22data%22%3A%5B2609729.0%2C11642502.5%2C48300275.0%2C48384237.0%2C212271759.0%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22pycdr2%22%2C%22data%22%3A%5B2597933.0%2C11599892.5%2C48303093.0%2C48307648.0%2C208590691.0%5D%2C%22backgroundColor%22%3A%22rgba%28139%2C92%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28139%2C92%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22RadarCube/encode%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=620&h=240)

<details>
<summary>Numeric values — RadarCube encode (click to expand)</summary>

| Variant | edgefirst (pyo3) | cyclone-dds-py | pycdr2 | cdds / native | pycdr2 / native |
|---|---|---|---|---|---|
| DRVEGRD-169 Ultra-Short | 9.8 µs | 2.6 ms | 2.6 ms | **265×** | **264×** |
| DRVEGRD-169 Short | 96.6 µs | 11.6 ms | 11.6 ms | **120×** | **120×** |
| DRVEGRD-169 Long | 573 µs | 48.3 ms | 48.3 ms | **84×** | **84×** |
| DRVEGRD-171 Short | 581 µs | 48.4 ms | 48.3 ms | **83×** | **83×** |
| DRVEGRD-171 Extra-Long | 2.2 ms | 212 ms | 209 ms | **95×** | **93×** |

</details>

**Mask: decode latency by resolution**

_Argmax-applied segmentation masks (1 u8 per pixel, regardless of class count)._

![Mask/decode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22160%5Cu00d7160%20proto%22%2C%22320%5Cu00d7320%20%28HD%20model%29%22%2C%22480%5Cu00d7480%20%28FHD%20model%29%22%2C%22640%5Cu00d7640%20%28instance%20mask%29%22%2C%221280%5Cu00d7720%20HD%22%2C%221920%5Cu00d71080%20FHD%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22edgefirst%20%28pyo3%29%22%2C%22data%22%3A%5B1204.0%2C3926.0%2C10555.0%2C23555.0%2C99315.0%2C262075.0%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22cyclone-dds-py%22%2C%22data%22%3A%5B178111.0%2C687465.0%2C1694523.0%2C3106212.0%2C7254602.5%2C16480359.0%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22pycdr2%22%2C%22data%22%3A%5B178037.0%2C687215.0%2C1648449.0%2C3164851.5%2C7316187.0%2C16485971.0%5D%2C%22backgroundColor%22%3A%22rgba%28139%2C92%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28139%2C92%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Mask/decode%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=620&h=276)

<details>
<summary>Numeric values — Mask decode (click to expand)</summary>

| Variant | edgefirst (pyo3) | cyclone-dds-py | pycdr2 | cdds / native | pycdr2 / native |
|---|---|---|---|---|---|
| 160×160 proto | 1.2 µs | 178 µs | 178 µs | **148×** | **148×** |
| 320×320 (HD model) | 3.9 µs | 687 µs | 687 µs | **175×** | **175×** |
| 480×480 (FHD model) | 10.6 µs | 1.7 ms | 1.6 ms | **161×** | **156×** |
| 640×640 (instance mask) | 23.6 µs | 3.1 ms | 3.2 ms | **132×** | **134×** |
| 1280×720 HD | 99.3 µs | 7.3 ms | 7.3 ms | **73×** | **74×** |
| 1920×1080 FHD | 262 µs | 16.5 ms | 16.5 ms | **63×** | **63×** |

</details>

**Mask: encode latency by resolution**

![Mask/encode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22160%5Cu00d7160%20proto%22%2C%22320%5Cu00d7320%20%28HD%20model%29%22%2C%22480%5Cu00d7480%20%28FHD%20model%29%22%2C%22640%5Cu00d7640%20%28instance%20mask%29%22%2C%221280%5Cu00d7720%20HD%22%2C%221920%5Cu00d71080%20FHD%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22edgefirst%20%28pyo3%29%22%2C%22data%22%3A%5B878.7%2C3296.0%2C7241.0%2C17500.0%2C124093.0%2C259167.0%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22cyclone-dds-py%22%2C%22data%22%3A%5B552529.0%2C2353440.5%2C5903839.0%2C10853529.0%2C23801488.5%2C53782104.5%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22pycdr2%22%2C%22data%22%3A%5B551834.5%2C2339988.0%2C5863849.0%2C10701399.0%2C23700703.0%2C53364033.0%5D%2C%22backgroundColor%22%3A%22rgba%28139%2C92%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28139%2C92%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Mask/encode%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=620&h=276)

<details>
<summary>Numeric values — Mask encode (click to expand)</summary>

| Variant | edgefirst (pyo3) | cyclone-dds-py | pycdr2 | cdds / native | pycdr2 / native |
|---|---|---|---|---|---|
| 160×160 proto | 879 ns | 553 µs | 552 µs | **629×** | **628×** |
| 320×320 (HD model) | 3.3 µs | 2.4 ms | 2.3 ms | **714×** | **710×** |
| 480×480 (FHD model) | 7.2 µs | 5.9 ms | 5.9 ms | **815×** | **810×** |
| 640×640 (instance mask) | 17.5 µs | 10.9 ms | 10.7 ms | **620×** | **612×** |
| 1280×720 HD | 124 µs | 23.8 ms | 23.7 ms | **192×** | **191×** |
| 1920×1080 FHD | 259 µs | 53.8 ms | 53.4 ms | **208×** | **206×** |

</details>

**PointCloud2: decode latency by sensor mode**

_LiDAR / radar point clouds — point_step=13 = x/y/z f32 + reflect u8; point_step=16 = + fusion_class/vision_class/instance_id._

![PointCloud2/decode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22Robosense%20E1R%20%2826K%2C%2013%20bpp%29%22%2C%22Ouster%201024%5Cu00d710%5Cu00d7128%20%28131K%2C%2013%20bpp%29%22%2C%22Ouster%202048%5Cu00d710%5Cu00d7128%20%28262K%2C%2013%20bpp%29%22%2C%22Fusion%20classes%20%28131K%2C%2016%20bpp%29%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22edgefirst%20%28pyo3%29%22%2C%22data%22%3A%5B18370.0%2C255732.0%2C650668.0%2C353001.0%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22cyclone-dds-py%22%2C%22data%22%3A%5B2526284.0%2C14304130.0%2C28888687.0%2C17698611.5%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22pycdr2%22%2C%22data%22%3A%5B2519914.0%2C14318743.0%2C28761337.0%2C17713288.5%5D%2C%22backgroundColor%22%3A%22rgba%28139%2C92%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28139%2C92%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22PointCloud2/decode%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=620&h=204)

<details>
<summary>Numeric values — PointCloud2 decode (click to expand)</summary>

| Variant | edgefirst (pyo3) | cyclone-dds-py | pycdr2 | cdds / native | pycdr2 / native |
|---|---|---|---|---|---|
| Robosense E1R (26K, 13 bpp) | 18.4 µs | 2.5 ms | 2.5 ms | **138×** | **137×** |
| Ouster 1024×10×128 (131K, 13 bpp) | 256 µs | 14.3 ms | 14.3 ms | **56×** | **56×** |
| Ouster 2048×10×128 (262K, 13 bpp) | 651 µs | 28.9 ms | 28.8 ms | **44×** | **44×** |
| Fusion classes (131K, 16 bpp) | 353 µs | 17.7 ms | 17.7 ms | **50×** | **50×** |

</details>

**PointCloud2: encode latency by sensor mode**

![PointCloud2/encode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22Robosense%20E1R%20%2826K%2C%2013%20bpp%29%22%2C%22Ouster%201024%5Cu00d710%5Cu00d7128%20%28131K%2C%2013%20bpp%29%22%2C%22Ouster%202048%5Cu00d710%5Cu00d7128%20%28262K%2C%2013%20bpp%29%22%2C%22Fusion%20classes%20%28131K%2C%2016%20bpp%29%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22edgefirst%20%28pyo3%29%22%2C%22data%22%3A%5B17592.0%2C281945.0%2C612706.0%2C359334.0%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22cyclone-dds-py%22%2C%22data%22%3A%5B8644041.0%2C46649243.5%2C99264003.0%2C58061931.0%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22pycdr2%22%2C%22data%22%3A%5B8499968.0%2C46872469.5%2C99908067.5%2C57286794.5%5D%2C%22backgroundColor%22%3A%22rgba%28139%2C92%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28139%2C92%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22PointCloud2/encode%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=620&h=204)

<details>
<summary>Numeric values — PointCloud2 encode (click to expand)</summary>

| Variant | edgefirst (pyo3) | cyclone-dds-py | pycdr2 | cdds / native | pycdr2 / native |
|---|---|---|---|---|---|
| Robosense E1R (26K, 13 bpp) | 17.6 µs | 8.6 ms | 8.5 ms | **491×** | **483×** |
| Ouster 1024×10×128 (131K, 13 bpp) | 282 µs | 46.6 ms | 46.9 ms | **165×** | **166×** |
| Ouster 2048×10×128 (262K, 13 bpp) | 613 µs | 99.3 ms | 99.9 ms | **162×** | **163×** |
| Fusion classes (131K, 16 bpp) | 359 µs | 58.1 ms | 57.3 ms | **162×** | **159×** |

</details>

**CompressedVideo: decode latency by payload size**

_Foxglove H.264/H.265 NAL-unit frames._

![CompressedVideo/decode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%2210%20KB%22%2C%22100%20KB%22%2C%22500%20KB%22%2C%221%20MB%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22edgefirst%20%28pyo3%29%22%2C%22data%22%3A%5B667.6%2C3667.0%2C39575.0%2C114639.0%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22cyclone-dds-py%22%2C%22data%22%3A%5B78390.0%2C688863.0%2C3947667.5%2C8621690.0%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22pycdr2%22%2C%22data%22%3A%5B78278.0%2C686575.0%2C3904158.5%2C8640014.0%5D%2C%22backgroundColor%22%3A%22rgba%28139%2C92%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28139%2C92%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22CompressedVideo/decode%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=620&h=204)

<details>
<summary>Numeric values — CompressedVideo decode (click to expand)</summary>

| Variant | edgefirst (pyo3) | cyclone-dds-py | pycdr2 | cdds / native | pycdr2 / native |
|---|---|---|---|---|---|
| 10 KB | 668 ns | 78.4 µs | 78.3 µs | **117×** | **117×** |
| 100 KB | 3.7 µs | 689 µs | 687 µs | **188×** | **187×** |
| 500 KB | 39.6 µs | 3.9 ms | 3.9 ms | **100×** | **99×** |
| 1 MB | 115 µs | 8.6 ms | 8.6 ms | **75×** | **75×** |

</details>

**CompressedVideo: encode latency by payload size**

![CompressedVideo/encode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%2210%20KB%22%2C%22100%20KB%22%2C%22500%20KB%22%2C%221%20MB%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22edgefirst%20%28pyo3%29%22%2C%22data%22%3A%5B466.36%2C4315.0%2C40760.0%2C131815.0%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22cyclone-dds-py%22%2C%22data%22%3A%5B229889.0%2C2356460.0%2C13247628.0%2C28880706.0%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22pycdr2%22%2C%22data%22%3A%5B229130.0%2C2384137.0%2C12895674.0%2C28844708.0%5D%2C%22backgroundColor%22%3A%22rgba%28139%2C92%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28139%2C92%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22CompressedVideo/encode%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=620&h=204)

<details>
<summary>Numeric values — CompressedVideo encode (click to expand)</summary>

| Variant | edgefirst (pyo3) | cyclone-dds-py | pycdr2 | cdds / native | pycdr2 / native |
|---|---|---|---|---|---|
| 10 KB | 466 ns | 230 µs | 229 µs | **493×** | **491×** |
| 100 KB | 4.3 µs | 2.4 ms | 2.4 ms | **546×** | **553×** |
| 500 KB | 40.8 µs | 13.2 ms | 12.9 ms | **325×** | **316×** |
| 1 MB | 132 µs | 28.9 ms | 28.8 ms | **219×** | **219×** |

</details>

**DmaBuffer: decode latency by fixture**

_DMA-buf reference (metadata only; no payload)._

![DmaBuffer/decode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22default%201280%5Cu00d7720%20RGBA%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22edgefirst%20%28pyo3%29%22%2C%22data%22%3A%5B336.1%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22cyclone-dds-py%22%2C%22data%22%3A%5B13537.0%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22pycdr2%22%2C%22data%22%3A%5B16167.0%5D%2C%22backgroundColor%22%3A%22rgba%28139%2C92%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28139%2C92%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22DmaBuffer/decode%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=620&h=180)

<details>
<summary>Numeric values — DmaBuffer decode (click to expand)</summary>

| Variant | edgefirst (pyo3) | cyclone-dds-py | pycdr2 | cdds / native | pycdr2 / native |
|---|---|---|---|---|---|
| default 1280×720 RGBA | 336 ns | 13.5 µs | 16.2 µs | **40×** | **48×** |

</details>

**DmaBuffer: encode latency by fixture**

![DmaBuffer/encode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22default%201280%5Cu00d7720%20RGBA%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22edgefirst%20%28pyo3%29%22%2C%22data%22%3A%5B100.56%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22cyclone-dds-py%22%2C%22data%22%3A%5B14500.0%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22pycdr2%22%2C%22data%22%3A%5B14260.0%5D%2C%22backgroundColor%22%3A%22rgba%28139%2C92%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28139%2C92%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22DmaBuffer/encode%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=620&h=180)

<details>
<summary>Numeric values — DmaBuffer encode (click to expand)</summary>

| Variant | edgefirst (pyo3) | cyclone-dds-py | pycdr2 | cdds / native | pycdr2 / native |
|---|---|---|---|---|---|
| default 1280×720 RGBA | 101 ns | 14.5 µs | 14.3 µs | **144×** | **142×** |

</details>

**Light message types: encode / decode round-trip**

_Small `CdrFixed` structs and `Header` (geometry primitives, time stamps, color). Costs are dominated by Python attribute access and per-call interpreter overhead, not the serialization work itself — speedup ratios are smaller than the heavy types and absolute times are sub-microsecond._

<details>
<summary>Numeric values — light types (click to expand)</summary>

| Message | Op | edgefirst (pyo3) | cyclone-dds-py | pycdr2 | cdds / native | pycdr2 / native |
|---|---|---|---|---|---|---|
| Time | encode | 122 ns | 6.1 µs | 6.1 µs | **50×** | **50×** |
| Time | decode | 289 ns | 4.9 µs | 5.0 µs | **17×** | **17×** |
| Duration | encode | 122 ns | 6.1 µs | 6.0 µs | **50×** | **49×** |
| Duration | decode | 269 ns | 5.0 µs | 4.9 µs | **18×** | **18×** |
| Header | encode | 100 ns | 8.3 µs | 8.3 µs | **83×** | **83×** |
| Header | decode | 325 ns | 7.4 µs | 7.5 µs | **23×** | **23×** |
| ColorRGBA | encode | 122 ns | 7.4 µs | 7.4 µs | **61×** | **61×** |
| ColorRGBA | decode | 282 ns | 6.3 µs | 6.3 µs | **22×** | **22×** |
| Vector3 | encode | 130 ns | 6.9 µs | 6.8 µs | **53×** | **52×** |
| Vector3 | decode | 273 ns | 5.8 µs | 5.7 µs | **21×** | **21×** |
| Point | encode | 130 ns | 6.9 µs | 6.8 µs | **53×** | **52×** |
| Point | decode | 281 ns | 5.8 µs | 5.7 µs | **20×** | **20×** |
| Point32 | encode | 125 ns | 6.7 µs | 6.8 µs | **54×** | **54×** |
| Point32 | decode | 276 ns | 5.6 µs | 5.6 µs | **20×** | **20×** |
| Quaternion | encode | 134 ns | 7.6 µs | 7.7 µs | **57×** | **57×** |
| Quaternion | decode | 280 ns | 6.4 µs | 6.4 µs | **23×** | **23×** |
| Pose | encode | 150 ns | 10.9 µs | 11.0 µs | **73×** | **73×** |
| Pose | decode | 280 ns | 10.1 µs | 10.2 µs | **36×** | **37×** |
| Pose2D | encode | 120 ns | 7.0 µs | 6.7 µs | **58×** | **56×** |
| Pose2D | decode | 274 ns | 5.7 µs | 5.7 µs | **21×** | **21×** |
| Transform | encode | 150 ns | 10.9 µs | 10.7 µs | **72×** | **71×** |
| Transform | decode | 277 ns | 10.1 µs | 10.2 µs | **36×** | **37×** |
| Twist | encode | 145 ns | 10.2 µs | 10.0 µs | **70×** | **69×** |
| Twist | decode | 282 ns | 9.4 µs | 9.4 µs | **34×** | **33×** |

</details>

---

<details>
<summary>Target system metadata</summary>

```
timestamp:      20260427T152927Z
git_rev:        468cd76a0328a6031a431a4fb730305a1f2b3da0
target_host:    rpi5-hailo
python_version: Python 3.13.5
uname:          Linux rpi5-hailo 6.12.75+rpt-rpi-2712 #1 SMP PREEMPT Debian 1:6.12.75-1+rpt1 (2026-03-11) aarch64 GNU/Linux
--- /proc/cpuinfo (head -30) ---
processor	: 0
BogoMIPS	: 108.00
Features	: fp asimd evtstrm aes pmull sha1 sha2 crc32 atomics fphp asimdhp cpuid asimdrdm lrcpc dcpop asimddp
CPU implementer	: 0x41
CPU architecture: 8
CPU variant	: 0x4
CPU part	: 0xd0b
CPU revision	: 1

processor	: 1
BogoMIPS	: 108.00
Features	: fp asimd evtstrm aes pmull sha1 sha2 crc32 atomics fphp asimdhp cpuid asimdrdm lrcpc dcpop asimddp
CPU implementer	: 0x41
CPU architecture: 8
CPU variant	: 0x4
CPU part	: 0xd0b
CPU revision	: 1

processor	: 2
BogoMIPS	: 108.00
Features	: fp asimd evtstrm aes pmull sha1 sha2 crc32 atomics fphp asimdhp cpuid asimdrdm lrcpc dcpop asimddp
CPU implementer	: 0x41
CPU architecture: 8
CPU variant	: 0x4
CPU part	: 0xd0b
CPU revision	: 1

processor	: 3
BogoMIPS	: 108.00
Features	: fp asimd evtstrm aes pmull sha1 sha2 crc32 atomics fphp asimdhp cpuid asimdrdm lrcpc dcpop asimddp
```

</details>
