# CDR Serialization Benchmarks

## Summary

**Target Hardware:** Raspberry Pi 5 Model B (Cortex-A76 @ 2.4 GHz, 4 cores), aarch64
**Toolchain:** zig 0.16.0 cross-compile from Ubuntu 25.10 host
**Last updated:** 2026-04-26

edgefirst-schemas C++ wrappers vs. **eProsima Fast-CDR** and **Eclipse
Cyclone DDS** — the codecs under the two major rmw vendors in current
ROS 2 releases. The deferred-cost design dominates the eager-materialize
strategy used by both DDS codecs on every non-trivial workload; the
charts below tell the story at a glance.

> Lower is better. Each chart shows three bars per row —
> **blue = edgefirst**, **amber = Fast-CDR**, **emerald = Cyclone DDS**.
> Charts use a logarithmic x-axis when the dynamic range exceeds ~1000×,
> otherwise linear. Per-bar labels carry a humanized unit
> (ns / µs / ms / s) so values stay legible across many orders of
> magnitude. Regenerate locally with
> `./benches/cpp/benchmark.sh --target <ssh-host> --render`.

**Image: decode latency by resolution × encoding**

![Image/decode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22VGA_rgb8%22%2C%22VGA_yuyv%22%2C%22VGA_nv12%22%2C%22HD_rgb8%22%2C%22HD_yuyv%22%2C%22HD_nv12%22%2C%22FHD_rgb8%22%2C%22FHD_yuyv%22%2C%22FHD_nv12%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B107290.8%2C54767.9%2C17268.7%2C611522.8%2C329548.2%2C106689.8%2C1609800.0%2C1181700.0%2C392788.1%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B55.89%2C55.97%2C55.93%2C55.95%2C56.16%2C55.96%2C55.96%2C55.9%2C55.93%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B98123.5%2C56156.1%2C18387.1%2C848834.7%2C589735.2%2C109301.0%2C1622000.0%2C1170800.0%2C398638.2%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Image/decode%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=540)

<details>
<summary>Numeric values (click to expand)</summary>

| Variant | edgefirst | Fast-CDR | Cyclone DDS | ef/Fast-CDR | ef/Cyclone |
|---|---|---|---|---|---|
| VGA_rgb8 | 55 ns | 104 µs | 114 µs | **1,883×** | **2,078×** |
| VGA_yuyv | 55 ns | 51.7 µs | 54.8 µs | **939×** | **996×** |
| VGA_nv12 | 55 ns | 16.7 µs | 17.3 µs | **303×** | **315×** |
| HD_rgb8 | 55 ns | 604 µs | 632 µs | **10,974×** | **11,478×** |
| HD_yuyv | 55 ns | 369 µs | 379 µs | **6,700×** | **6,885×** |
| HD_nv12 | 55 ns | 93.9 µs | 102 µs | **1,705×** | **1,844×** |
| FHD_rgb8 | 55 ns | 1.6 ms | 1.6 ms | **28,945×** | **29,692×** |
| FHD_yuyv | 55 ns | 977 µs | 988 µs | **17,748×** | **17,935×** |
| FHD_nv12 | 55 ns | 430 µs | 448 µs | **7,806×** | **8,144×** |

</details>

**Image HD_rgb8: workflow loops (per-iteration time)**

![Image/workflow](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22HD_rgb8%22%2C%22HD_rgb8%22%2C%22HD_rgb8%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B627331200.0%2C737685900.0%2C1319100.0%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B34617.0%2C73464.3%2C682516.7%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B618257700.0%2C720628000.0%2C1286900.0%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Image/workflow%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values (click to expand)</summary>

| Op | edgefirst | Fast-CDR | Cyclone DDS | ef/Fast-CDR | ef/Cyclone |
|---|---|---|---|---|---|
| `pub_loop_inplace` | 35 ns | 601 µs | 607 µs | **17,378×** | **17,533×** |
| `sub_loop` | 72 ns | 704 µs | 725 µs | **9,738×** | **10,028×** |
| `sub_modify_pub` | 656 µs | 1.3 ms | 1.3 ms | **1.9×** | **2.0×** |

</details>

**RadarCube: decode latency by DRVEGRD-169 range mode**

![RadarCube/decode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22DRVEGRD169_ultra_short%22%2C%22DRVEGRD169_short%22%2C%22DRVEGRD169_long%22%2C%22DRVEGRD171_short%22%2C%22DRVEGRD171_extra_long%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B10300.0%2C83657.3%2C696606.0%2C706256.6%2C3444700.0%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B57.54%2C57.55%2C57.55%2C57.56%2C57.55%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B10415.9%2C93015.0%2C720651.7%2C699875.9%2C3439800.0%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22RadarCube/decode%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=300)

<details>
<summary>Numeric values (click to expand)</summary>

| Variant | edgefirst | Fast-CDR | Cyclone DDS | ef/Fast-CDR | ef/Cyclone |
|---|---|---|---|---|---|
| DRVEGRD169 ultra short | 58 ns | 10.4 µs | 10.3 µs | **181×** | **179×** |
| DRVEGRD169 short | 58 ns | 93.0 µs | 83.7 µs | **1,616×** | **1,454×** |
| DRVEGRD169 long | 58 ns | 721 µs | 697 µs | **12,523×** | **12,105×** |
| DRVEGRD171 short | 58 ns | 700 µs | 706 µs | **12,159×** | **12,270×** |
| DRVEGRD171 extra long | 58 ns | 3.4 ms | 3.4 ms | **59,774×** | **59,858×** |

</details>

**Mask: decode latency by resolution × class count**

![Mask/decode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22160x160_proto%22%2C%22320x320_8class%22%2C%22480x480_9class%22%2C%22640x640_8class%22%2C%221280x720_hd%22%2C%221920x1080_fhd%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B1289.6%2C4867.7%2C12522.3%2C28535.0%2C90211.4%2C431348.2%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B53.72%2C53.67%2C53.79%2C53.51%2C54.11%2C53.59%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B1212.2%2C4917.6%2C12192.6%2C25470.5%2C107209.1%2C468762.1%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Mask/decode%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=360)

<details>
<summary>Numeric values (click to expand)</summary>

| Variant | edgefirst | Fast-CDR | Cyclone DDS | ef/Fast-CDR | ef/Cyclone |
|---|---|---|---|---|---|
| 160x160 proto | 53 ns | 1.2 µs | 1.3 µs | **23×** | **25×** |
| 320x320 8class | 53 ns | 4.7 µs | 4.7 µs | **89×** | **90×** |
| 480x480 9class | 53 ns | 11.6 µs | 11.8 µs | **219×** | **222×** |
| 640x640 8class | 53 ns | 28.1 µs | 26.8 µs | **530×** | **506×** |
| 1280x720 hd | 53 ns | 91.6 µs | 94.5 µs | **1,730×** | **1,786×** |
| 1920x1080 fhd | 53 ns | 504 µs | 451 µs | **9,520×** | **8,529×** |

</details>

**CompressedVideo: decode latency by payload size**

![CompressedVideo/decode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%2210KB%22%2C%22100KB%22%2C%22500KB%22%2C%221MB%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B715.5%2C4856.5%2C36381.0%2C137576.2%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B54.63%2C54.62%2C54.62%2C54.62%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B644.35%2C4768.6%2C40527.1%2C138520.5%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22CompressedVideo/decode%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=240)

<details>
<summary>Numeric values (click to expand)</summary>

| Variant | edgefirst | Fast-CDR | Cyclone DDS | ef/Fast-CDR | ef/Cyclone |
|---|---|---|---|---|---|
| 10KB | 53 ns | 637 ns | 717 ns | **12×** | **13×** |
| 100KB | 53 ns | 4.9 µs | 5.0 µs | **91×** | **94×** |
| 500KB | 53 ns | 41.3 µs | 47.6 µs | **774×** | **894×** |
| 1MB | 53 ns | 122 µs | 134 µs | **2,283×** | **2,506×** |

</details>

**PointCloud2: decode latency by sensor source**

![PointCloud2/decode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22robosense_e1r%22%2C%22ouster_1024x10_128beam%22%2C%22ouster_2048x10_128beam%22%2C%22fusion_classes_ouster%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B19674.8%2C322640.3%2C818061.1%2C430267.6%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B125.17%2C125.81%2C125.05%2C124.47%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B22985.3%2C288183.4%2C809203.2%2C435450.5%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22PointCloud2/decode%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=240)

<details>
<summary>Numeric values (click to expand)</summary>

| Variant | edgefirst | Fast-CDR | Cyclone DDS | ef/Fast-CDR | ef/Cyclone |
|---|---|---|---|---|---|
| robosense e1r | 123 ns | 19.6 µs | 25.2 µs | **159×** | **205×** |
| ouster 1024×10 128beam | 123 ns | 298 µs | 297 µs | **2,425×** | **2,413×** |
| ouster 2048×10 128beam | 123 ns | 784 µs | 819 µs | **6,377×** | **6,670×** |
| fusion classes ouster | 124 ns | 415 µs | 437 µs | **3,350×** | **3,527×** |

</details>

**Header: decode latency (small-message baseline)**

![Header/decode](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22decode%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B161.5523%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B44.7019%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B104.8665%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Header/decode%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values (click to expand)</summary>

| Variant | edgefirst | Fast-CDR | Cyclone DDS | ef/Fast-CDR | ef/Cyclone |
|---|---|---|---|---|---|
| default | 44 ns | 101 ns | 150 ns | **2.3×** | **3.4×** |

</details>

---

## Methodology

### What's measured

Three categories of operations are benchmarked for each C++ implementation:

#### Codec primitives (per-impl baseline)

| op | edgefirst | Fast-CDR | what it measures |
|----|-----------|----------|------------------|
| `encode/new` | yes | yes | build a fresh message buffer |
| `decode/decode` | yes (borrow) | yes (materialize) | receive bytes → ready-to-use state |

These are reported as per-impl baselines and are **not directly comparable
on their own.** Fast-CDR's `decode` number includes full materialization work
that edgefirst defers into field accessors; edgefirst's borrow scan records a
small offset table in O(variable-length fields) then stops. The honest
cross-impl comparison is in the access and workflow patterns below.

#### Access patterns (apples-to-apples comparison)

For each impl: "from these wire bytes, deliver these accessed values":

| op | what it measures |
|----|------------------|
| `access/one_field` | decode + read 1 small field (e.g., `header.stamp.sec`) |
| `access/half_fields` | decode + read the message's typical-consumer subset (~50%) |
| `access/all_fields` | decode + read every scalar / array length |
| `access/payload_iter` | decode + iterate the bulk payload |

Same end goal across impls; different cost curves. The crossover as access
count rises is the central narrative of the comparison.

#### Workflow patterns (real-world loops)

| op | what it measures |
|----|------------------|
| `workflow/sub_modify_pub` | receive → modify 1–2 fields → publish |
| `workflow/pub_loop_rebuild` | 1000-iter publisher rebuilding messages from scratch |
| `workflow/pub_loop_inplace` | 1000-iter publisher reusing buffer + in-place setters |
| `workflow/sub_loop` | 1000-iter subscriber decoding the same wire bytes |

`pub_loop_inplace` and `sub_loop` at low access count are where edgefirst's
buffer-reuse and deferred-cost design dominate.

### Toolchain matrix

| Backend | Framework | Config |
|---------|-----------|--------|
| C++ (edgefirst) | Google Benchmark 1.9.1 | ~1s per benchmark, auto warmup; `BENCH_FAST=1` → 0.05s |
| C++ (Fast-CDR) | Google Benchmark 1.9.1 | Fast-CDR 2.2.5, Foonathan-memory 0.7-3 |
| C++ (Cyclone DDS) | Google Benchmark 1.9.1 | Cyclone DDS 0.10.5 (CDR sources only, no full ddsc) |

C++ cross-compile: `zig c++` is the default (single-tool toolchain via
`benches/cpp/cmake/zig-wrappers/`). `gcc-aarch64-linux-gnu` is supported as
a fallback via `--toolchain gcc` if zig is unavailable.

### IDL provenance

Hand-written OMG IDL files are committed at `benches/cpp/idl/`. Generated C++
types for Fast-CDR are produced by `fastddsgen 4.3.0` and committed to
`benches/cpp/types/fastcdr/` (regenerable but not required for a normal build).

To regenerate:

```bash
export FASTDDS_PATH=$HOME/Software/ROS2/Fast-DDS-Gen
./benches/cpp/scripts/regen_fastcdr_types.sh
```

IDL fixes applied during initial authoring:
- `float64` → `double`, `float32` → `float` (IDL base types)
- Constants that appeared inside struct bodies hoisted to module scope with
  disambiguating prefixes (`RADAR_CUBE_*`, `POINT_FIELD_*`) — fastddsgen
  rejects const-inside-struct

### Wire-byte parity

`benches/cpp/tests/parity_test.cpp` asserts that edgefirst-schemas and Fast-CDR
produce byte-identical CDR for representative variants of every message type.
This test gates trust in the cross-impl comparison — a failing parity test means
the impls are not measuring the same wire format.

Parity verified for: `Header`, `Time`, `Vector3`, `Pose`, `DmaBuffer`,
`Image/HD_rgb8`, `RadarCube/DRVEGRD169_short`, `Mask/320x320_8class`,
`CompressedVideo/10KB`, `PointCloud2/robosense_e1r`.

### Sample / warmup / iteration counts

- **Google Benchmark** (C++): default ~1s per benchmark with automatic warmup.
  `BENCH_FAST=1` (via `--quick` flag) reduces to 0.05s for rapid iteration.

### Fixture definitions

Fixtures are declared in `benches/cpp/common.hpp`. Each message type has a set
of representative variants covering realistic workload shapes. The current set
(subject to ongoing review against real-world workloads):

- **Image**: VGA/HD/FHD × rgb8/yuyv/nv12
- **RadarCube**: DRVEGRD169 ultra-short / short / long (grounded in radarpub source)
- **Mask**: 160×160, 320×320, 480×480, 640×640, 1280×720, 1920×1080 (production model outputs)
- **CompressedVideo**: 10 KB, 100 KB, 500 KB, 1 MB payloads
- **PointCloud2**: Robosense E1R (26K pts), Ouster 128-beam 1024 (131K pts), Ouster 128-beam 2048 (262K pts), fusion-annotated (131K pts)
- **Small types**: `Header`, `DmaBuffer`, `Time`, `Vector3`, `Pose`

Each fixture also declares field subsets for the `access/*` ops so both
backends read identically the same fields.

---

## Building and Running

### Local development (Ubuntu 25.10)

**Prerequisites:**

```bash
# Build & cross-compile
sudo apt install -y cmake ninja-build openssh-client default-jre python3-pip

# Cross-compiler — zig is the default; gcc is supported as a fallback
sudo snap install zig --classic --beta
# (or, for the gcc fallback)
# sudo apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# Reporting
pip install --user requests
```

Fast-CDR, Foonathan-memory, and Google Benchmark are pulled automatically by
CMake `FetchContent` at pinned tags — no manual installs required.

`fastddsgen` is only needed if you regenerate the committed Fast-CDR types.
Install by cloning [eProsima/Fast-DDS-Gen](https://github.com/eProsima/Fast-DDS-Gen.git)
and building with:

```bash
JAVA_HOME=$(dirname $(dirname $(readlink -f $(which java)))) ./gradlew assemble
export FASTDDS_PATH=$HOME/Software/ROS2/Fast-DDS-Gen
```

**Common workflows:**

```bash
# Full flow: cross-compile → deploy → run on target → render report
./benches/cpp/benchmark.sh --target user@imx8mp.local --render

# Cross-compile only (no target required)
./benches/cpp/benchmark.sh --build-only

# Fast iteration on a single message type
./benches/cpp/benchmark.sh --quick --filter='Image' --target user@imx8mp.local

# Re-render from previously collected results
./benches/cpp/benchmark.sh --run-only --render --target user@imx8mp.local

# Try zig toolchain (may fail; see NOTE in --help)
./benches/cpp/benchmark.sh --toolchain zig --build-only
```

**Full `benchmark.sh` usage:**

```
Usage: benchmark.sh [OPTIONS]

Build, deploy, and run on-target C++ codec benchmarks.

Build:
  --toolchain {zig|gcc}     Cross-compile toolchain (default: zig)
  --build-dir DIR           CMake build directory (default: build/aarch64 under benches/cpp/)
  --clean                   Wipe build dir before configuring
  --build-only              Stop after build; --target is not required

Deploy & run:
  --target HOST             SSH destination, e.g. user@imx8mp.local
                            (env: BENCH_TARGET_HOST)
  --remote-path PATH        Remote directory for binaries (default: /tmp/edgefirst-bench)
                            (env: BENCH_REMOTE_PATH)
  --run-only                Skip build and deploy; re-run and collect from target
  --impl LIST               Comma-separated subset of: edgefirst,fastcdr
                            (default: all)
  --filter REGEX            Passed as --benchmark_filter=<REGEX> to each binary
  --quick                   Pass --benchmark_min_time=0.05s for fast feedback

Output:
  --output-dir DIR          Local results directory (default: results/<utc-timestamp>)
  --render                  Generate charts + Markdown report after collecting results

Environment variables (lowest precedence; CLI flags override):
  BENCH_TARGET_HOST         Same as --target
  BENCH_REMOTE_PATH         Same as --remote-path
  BENCH_TOOLCHAIN           Same as --toolchain
```

### CI

`benchmark.yml` is on-demand (`workflow_dispatch`). When dispatched:

1. `build-cpp-benchmarks` — cross-platform build on `ubuntu-22.04-arm`
   (native aarch64; no cross-compilation needed in CI).
2. `run-cpp-benchmarks` — downloads C++ binaries, runs each, captures JSON on
   the `nxp-imx8mp-latest` self-hosted runner.
3. `process-benchmarks` — calls `scripts/render_benchmarks.py results/` to
   produce a job summary with tables and QuickChart URLs.

CI is useful for regression detection; it is not the intended source of
headline numbers (run `benchmark.sh --render` locally for those).

### Adding a new backend

1. Drop `bench_<impl>.cpp` into `benches/cpp/`.
2. Add a 4-line block to `benches/cpp/CMakeLists.txt`:
   ```cmake
   add_benchmark_executable(bench_<impl>
       SOURCES bench_<impl>.cpp ${<IMPL>_GENERATED_SOURCES}
       LINK    <impl>::<impl>)
   ```
3. Add the impl name to the `IMPLS_ALL` array in `benchmark.sh`.
4. Generate types via a parallel script (e.g., `scripts/regen_cyclonedds_types.sh`)
   populating `types/<impl>/`, and commit the output.

No other files need to change.

---

## Detailed Results

Auto-generated by `scripts/render_benchmarks.py` from the most recent
on-target run (Pi 5, 2026-04-26). Each operation phase leads with the
chart and the corresponding numeric table is one click away.
Regenerate via `./benches/cpp/benchmark.sh --target <ssh-host> --render`.

*EdgeFirst zero-copy CDR vs. eProsima Fast-CDR comparison.*

### CompressedVideo

#### access

![CompressedVideo/access Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%2210KB%22%2C%22100KB%22%2C%22500KB%22%2C%221MB%22%2C%2210KB%22%2C%22100KB%22%2C%22500KB%22%2C%221MB%22%2C%2210KB%22%2C%22100KB%22%2C%22500KB%22%2C%221MB%22%2C%2210KB%22%2C%22100KB%22%2C%22500KB%22%2C%221MB%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B717.97%2C4760.0%2C37094.7%2C126297.6%2C716.63%2C4782.4%2C39539.7%2C165531.1%2C715.99%2C4754.0%2C39273.2%2C124945.3%2C2321.0%2C21201.5%2C116679.9%2C251881.6%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B101.73%2C101.71%2C102.05%2C101.56%2C97.32%2C97.32%2C97.78%2C97.32%2C62.89%2C62.9%2C62.89%2C62.9%2C1664.7%2C16204.7%2C80638.5%2C164319.1%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B646.19%2C4686.6%2C36313.2%2C131898.9%2C646.73%2C4665.6%2C39329.9%2C115964.8%2C645.13%2C4660.8%2C40659.8%2C125398.9%2C2251.1%2C21228.3%2C116228.3%2C252491.2%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22CompressedVideo/access%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=960)

<details>
<summary>Numeric values for CompressedVideo/access (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| 10KB | 717.9717 ns | 101.7316 ns | 646.1874 ns |
| 100KB | 4.7600 µs | 101.7128 ns | 4.6866 µs |
| 500KB | 37.0947 µs | 102.0508 ns | 36.3132 µs |
| 1MB | 126.2976 µs | 101.5647 ns | 131.8989 µs |
| 10KB | 716.6315 ns | 97.3230 ns | 646.7292 ns |
| 100KB | 4.7824 µs | 97.3217 ns | 4.6656 µs |
| 500KB | 39.5397 µs | 97.7824 ns | 39.3299 µs |
| 1MB | 165.5311 µs | 97.3220 ns | 115.9648 µs |
| 10KB | 715.9877 ns | 62.8949 ns | 645.1318 ns |
| 100KB | 4.7540 µs | 62.9031 ns | 4.6608 µs |
| 500KB | 39.2732 µs | 62.8903 ns | 40.6598 µs |
| 1MB | 124.9453 µs | 62.9005 ns | 125.3989 µs |
| 10KB | 2.3210 µs | 1.6647 µs | 2.2511 µs |
| 100KB | 21.2015 µs | 16.2047 µs | 21.2283 µs |
| 500KB | 116.6799 µs | 80.6385 µs | 116.2283 µs |
| 1MB | 251.8816 µs | 164.3191 µs | 252.4912 µs |

</details>

#### decode

![CompressedVideo/decode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%2210KB%22%2C%22100KB%22%2C%22500KB%22%2C%221MB%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B715.5%2C4856.5%2C36381.0%2C137576.2%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B54.63%2C54.62%2C54.62%2C54.62%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B644.35%2C4768.6%2C40527.1%2C138520.5%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22CompressedVideo/decode%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=240)

<details>
<summary>Numeric values for CompressedVideo/decode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| 10KB | 715.5017 ns | 54.6256 ns | 644.3470 ns |
| 100KB | 4.8565 µs | 54.6202 ns | 4.7686 µs |
| 500KB | 36.3810 µs | 54.6156 ns | 40.5271 µs |
| 1MB | 137.5762 µs | 54.6211 ns | 138.5205 µs |

</details>

#### encode

![CompressedVideo/encode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%2210KB%22%2C%22100KB%22%2C%22500KB%22%2C%221MB%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B0.8326%2C5.1622%2C35.6026%2C149.2077%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B0.638%2C4.8885%2C45.8937%2C159.595%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B0.7411%2C4.9743%2C39.7657%2C150.8413%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22CompressedVideo/encode%20Latency%20%28%5Cu00b5s%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28%5Cu00b5s%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=240)

<details>
<summary>Numeric values for CompressedVideo/encode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| 10KB | 832.6224 ns | 638.0310 ns | 741.0596 ns |
| 100KB | 5.1622 µs | 4.8885 µs | 4.9743 µs |
| 500KB | 35.6026 µs | 45.8937 µs | 39.7657 µs |
| 1MB | 149.2077 µs | 159.5950 µs | 150.8413 µs |

</details>

#### workflow

![CompressedVideo/workflow Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22100KB%22%2C%22100KB%22%2C%22100KB%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B4.9342%2C4.7325%2C0.0107%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B0.0279%2C0.0792%2C0.0051%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B4.9837%2C4.7626%2C0.0105%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22CompressedVideo/workflow%20Latency%20%28ms%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ms%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for CompressedVideo/workflow (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| 100KB | 4.9342 ms | 27.9461 µs | 4.9837 ms |
| 100KB | 4.9108 ms | 4.8274 ms | 4.9242 ms |
| 100KB | 4.7325 ms | 79.2096 µs | 4.7626 ms |
| 100KB | 10.7391 µs | 5.1109 µs | 10.5011 µs |

</details>

### DmaBuffer

#### access

![DmaBuffer/access Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22all_fields%22%2C%22half_fields%22%2C%22one_field%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B403.9463%2C398.5133%2C406.9796%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B145.3557%2C84.6149%2C62.6233%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B206.9908%2C202.7204%2C200.4965%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22DmaBuffer/access%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for DmaBuffer/access (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| all_fields | 403.9463 ns | 145.3557 ns | 206.9908 ns |
| half_fields | 398.5133 ns | 84.6149 ns | 202.7204 ns |
| one_field | 406.9796 ns | 62.6233 ns | 200.4965 ns |

</details>

#### decode

![DmaBuffer/decode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22decode%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B383.2188%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B54.6336%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B202.9405%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22DmaBuffer/decode%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for DmaBuffer/decode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| decode | 383.2188 ns | 54.6336 ns | 202.9405 ns |

</details>

#### encode

![DmaBuffer/encode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22new%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B591.7352%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B148.4517%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B433.5588%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22DmaBuffer/encode%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for DmaBuffer/encode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| new | 591.7352 ns | 148.4517 ns | 433.5588 ns |

</details>

### Header

#### access

![Header/access Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22all_fields%22%2C%22half_fields%22%2C%22one_field%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B174.3219%2C178.3196%2C171.044%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B78.5226%2C54.4228%2C54.7996%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B107.6652%2C104.2149%2C104.3393%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Header/access%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Header/access (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| all_fields | 174.3219 ns | 78.5226 ns | 107.6652 ns |
| half_fields | 178.3196 ns | 54.4228 ns | 104.2149 ns |
| one_field | 171.0440 ns | 54.7996 ns | 104.3393 ns |

</details>

#### decode

![Header/decode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22decode%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B161.5523%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B44.7019%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B104.8665%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Header/decode%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Header/decode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| decode | 161.5523 ns | 44.7019 ns | 104.8665 ns |

</details>

#### encode

![Header/encode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22new%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B267.9609%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B185.5847%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B217.6143%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Header/encode%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Header/encode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| new | 267.9609 ns | 185.5847 ns | 217.6143 ns |

</details>

#### workflow

![Header/workflow Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22pub_loop_inplace%22%2C%22sub_loop%22%2C%22sub_modify_pub%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B277670.8%2C155354.1%2C444.02%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B16685.5%2C78224.3%2C229.1%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B223984.9%2C105745.4%2C351.87%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Header/workflow%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Header/workflow (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| pub_loop_inplace | 277.6708 µs | 16.6855 µs | 223.9849 µs |
| pub_loop_rebuild | 275.6493 µs | 134.8912 µs | 223.1367 µs |
| sub_loop | 155.3541 µs | 78.2243 µs | 105.7454 µs |
| sub_modify_pub | 444.0155 ns | 229.1047 ns | 351.8736 ns |

</details>

### Image

#### access

![Image/access Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22VGA_rgb8%22%2C%22VGA_yuyv%22%2C%22VGA_nv12%22%2C%22HD_rgb8%22%2C%22HD_yuyv%22%2C%22HD_nv12%22%2C%22FHD_rgb8%22%2C%22FHD_yuyv%22%2C%22FHD_nv12%22%2C%22VGA_rgb8%22%2C%22VGA_yuyv%22%2C%22VGA_nv12%22%2C%22HD_rgb8%22%2C%22HD_yuyv%22%2C%22HD_nv12%22%2C%22FHD_rgb8%22%2C%22FHD_yuyv%22%2C%22FHD_nv12%22%2C%22VGA_rgb8%22%2C%22VGA_yuyv%22%2C%22VGA_nv12%22%2C%22HD_rgb8%22%2C%22HD_yuyv%22%2C%22HD_nv12%22%2C%22FHD_rgb8%22%2C%22FHD_yuyv%22%2C%22FHD_nv12%22%2C%22VGA_rgb8%22%2C%22VGA_yuyv%22%2C%22VGA_nv12%22%2C%22HD_rgb8%22%2C%22HD_yuyv%22%2C%22HD_nv12%22%2C%22FHD_rgb8%22%2C%22FHD_yuyv%22%2C%22FHD_nv12%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B101084.9%2C56231.7%2C18039.2%2C581085.8%2C317419.6%2C107193.7%2C1639000.0%2C1154400.0%2C376462.0%2C98932.4%2C57535.9%2C18388.3%2C617796.2%2C326271.9%2C109628.7%2C1593000.0%2C1144200.0%2C390362.3%2C94349.8%2C54692.1%2C18236.0%2C623804.5%2C320482.5%2C108417.9%2C1574600.0%2C1183800.0%2C390550.1%2C221708.1%2C143398.5%2C68393.2%2C845570.5%2C468099.6%2C219996.3%2C2459600.0%2C1606100.0%2C551638.8%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B125.17%2C120.39%2C120.39%2C120.45%2C120.37%2C120.36%2C120.41%2C120.36%2C120.6%2C90.66%2C90.68%2C90.66%2C90.67%2C90.67%2C90.68%2C90.66%2C90.66%2C90.64%2C64.61%2C64.61%2C64.59%2C64.61%2C64.61%2C64.59%2C64.59%2C64.61%2C66.65%2C144572.0%2C96400.6%2C48267.6%2C445012.7%2C290172.6%2C144295.1%2C1010900.0%2C672404.9%2C326575.6%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B105001.5%2C69418.9%2C26692.9%2C811637.1%2C569337.4%2C116440.7%2C1637800.0%2C1157400.0%2C381659.6%2C107497.5%2C51200.0%2C18307.8%2C811415.7%2C623252.9%2C96331.0%2C1615400.0%2C1309800.0%2C394899.2%2C103479.3%2C55509.3%2C18218.1%2C807263.1%2C591331.2%2C100634.6%2C1595700.0%2C1148200.0%2C403406.5%2C221941.5%2C143248.3%2C67377.4%2C984983.8%2C530942.0%2C220644.9%2C2467900.0%2C1605600.0%2C530996.6%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Image/access%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=2160)

<details>
<summary>Numeric values for Image/access (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| VGA_rgb8 | 101.0849 µs | 125.1650 ns | 105.0015 µs |
| VGA_yuyv | 56.2317 µs | 120.3872 ns | 69.4189 µs |
| VGA_nv12 | 18.0392 µs | 120.3904 ns | 26.6929 µs |
| HD_rgb8 | 581.0858 µs | 120.4474 ns | 811.6371 µs |
| HD_yuyv | 317.4196 µs | 120.3663 ns | 569.3374 µs |
| HD_nv12 | 107.1937 µs | 120.3634 ns | 116.4407 µs |
| FHD_rgb8 | 1.6390 ms | 120.4091 ns | 1.6378 ms |
| FHD_yuyv | 1.1544 ms | 120.3628 ns | 1.1574 ms |
| FHD_nv12 | 376.4620 µs | 120.6015 ns | 381.6596 µs |
| VGA_rgb8 | 98.9324 µs | 90.6639 ns | 107.4975 µs |
| VGA_yuyv | 57.5359 µs | 90.6789 ns | 51.2000 µs |
| VGA_nv12 | 18.3883 µs | 90.6578 ns | 18.3078 µs |
| HD_rgb8 | 617.7962 µs | 90.6677 ns | 811.4157 µs |
| HD_yuyv | 326.2719 µs | 90.6724 ns | 623.2529 µs |
| HD_nv12 | 109.6287 µs | 90.6773 ns | 96.3310 µs |
| FHD_rgb8 | 1.5930 ms | 90.6630 ns | 1.6154 ms |
| FHD_yuyv | 1.1442 ms | 90.6564 ns | 1.3098 ms |
| FHD_nv12 | 390.3623 µs | 90.6429 ns | 394.8992 µs |
| VGA_rgb8 | 94.3498 µs | 64.6098 ns | 103.4793 µs |
| VGA_yuyv | 54.6921 µs | 64.6086 ns | 55.5093 µs |
| VGA_nv12 | 18.2360 µs | 64.5890 ns | 18.2181 µs |
| HD_rgb8 | 623.8045 µs | 64.6136 ns | 807.2631 µs |
| HD_yuyv | 320.4825 µs | 64.6110 ns | 591.3312 µs |
| HD_nv12 | 108.4179 µs | 64.5944 ns | 100.6346 µs |
| FHD_rgb8 | 1.5746 ms | 64.5931 ns | 1.5957 ms |
| FHD_yuyv | 1.1838 ms | 64.6092 ns | 1.1482 ms |
| FHD_nv12 | 390.5501 µs | 66.6522 ns | 403.4065 µs |
| VGA_rgb8 | 221.7081 µs | 144.5720 µs | 221.9415 µs |
| VGA_yuyv | 143.3985 µs | 96.4006 µs | 143.2483 µs |
| VGA_nv12 | 68.3932 µs | 48.2676 µs | 67.3774 µs |
| HD_rgb8 | 845.5705 µs | 445.0127 µs | 984.9838 µs |
| HD_yuyv | 468.0996 µs | 290.1726 µs | 530.9420 µs |
| HD_nv12 | 219.9963 µs | 144.2951 µs | 220.6449 µs |
| FHD_rgb8 | 2.4596 ms | 1.0109 ms | 2.4679 ms |
| FHD_yuyv | 1.6061 ms | 672.4049 µs | 1.6056 ms |
| FHD_nv12 | 551.6388 µs | 326.5756 µs | 530.9966 µs |

</details>

#### decode

![Image/decode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22VGA_rgb8%22%2C%22VGA_yuyv%22%2C%22VGA_nv12%22%2C%22HD_rgb8%22%2C%22HD_yuyv%22%2C%22HD_nv12%22%2C%22FHD_rgb8%22%2C%22FHD_yuyv%22%2C%22FHD_nv12%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B107290.8%2C54767.9%2C17268.7%2C611522.8%2C329548.2%2C106689.8%2C1609800.0%2C1181700.0%2C392788.1%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B55.89%2C55.97%2C55.93%2C55.95%2C56.16%2C55.96%2C55.96%2C55.9%2C55.93%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B98123.5%2C56156.1%2C18387.1%2C848834.7%2C589735.2%2C109301.0%2C1622000.0%2C1170800.0%2C398638.2%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Image/decode%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=540)

<details>
<summary>Numeric values for Image/decode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| VGA_rgb8 | 107.2908 µs | 55.8922 ns | 98.1235 µs |
| VGA_yuyv | 54.7679 µs | 55.9682 ns | 56.1561 µs |
| VGA_nv12 | 17.2687 µs | 55.9260 ns | 18.3871 µs |
| HD_rgb8 | 611.5228 µs | 55.9456 ns | 848.8347 µs |
| HD_yuyv | 329.5482 µs | 56.1552 ns | 589.7352 µs |
| HD_nv12 | 106.6898 µs | 55.9567 ns | 109.3010 µs |
| FHD_rgb8 | 1.6098 ms | 55.9552 ns | 1.6220 ms |
| FHD_yuyv | 1.1817 ms | 55.9047 ns | 1.1708 ms |
| FHD_nv12 | 392.7881 µs | 55.9262 ns | 398.6382 µs |

</details>

#### encode

![Image/encode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22VGA_rgb8%22%2C%22VGA_yuyv%22%2C%22VGA_nv12%22%2C%22HD_rgb8%22%2C%22HD_yuyv%22%2C%22HD_nv12%22%2C%22FHD_rgb8%22%2C%22FHD_yuyv%22%2C%22FHD_nv12%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B0.1104%2C0.0531%2C0.017%2C0.62%2C0.3221%2C0.0904%2C1.662%2C1.0136%2C0.4453%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B0.1026%2C0.0501%2C0.0165%2C0.6031%2C0.2975%2C0.1101%2C1.66%2C1.0479%2C0.4027%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B0.1166%2C0.0528%2C0.017%2C0.8648%2C0.5547%2C0.1053%2C1.6417%2C1.022%2C0.4508%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Image/encode%20Latency%20%28ms%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ms%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=540)

<details>
<summary>Numeric values for Image/encode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| VGA_rgb8 | 110.3625 µs | 102.6380 µs | 116.6445 µs |
| VGA_yuyv | 53.1295 µs | 50.0681 µs | 52.7579 µs |
| VGA_nv12 | 17.0220 µs | 16.4952 µs | 17.0051 µs |
| HD_rgb8 | 619.9881 µs | 603.1329 µs | 864.7867 µs |
| HD_yuyv | 322.0708 µs | 297.5132 µs | 554.6582 µs |
| HD_nv12 | 90.4406 µs | 110.0875 µs | 105.3330 µs |
| FHD_rgb8 | 1.6620 ms | 1.6600 ms | 1.6417 ms |
| FHD_yuyv | 1.0136 ms | 1.0479 ms | 1.0220 ms |
| FHD_nv12 | 445.2709 µs | 402.7476 µs | 450.8223 µs |

</details>

#### workflow

![Image/workflow Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22HD_rgb8%22%2C%22HD_rgb8%22%2C%22HD_rgb8%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B627331200.0%2C737685900.0%2C1319100.0%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B34617.0%2C73464.3%2C682516.7%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B618257700.0%2C720628000.0%2C1286900.0%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Image/workflow%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Image/workflow (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| HD_rgb8 | 627.3312 ms | 34.6170 µs | 618.2577 ms |
| HD_rgb8 | 525.8234 ms | 633.3915 ms | 518.1643 ms |
| HD_rgb8 | 737.6859 ms | 73.4643 µs | 720.6280 ms |
| HD_rgb8 | 1.3191 ms | 682.5167 µs | 1.2869 ms |

</details>

### Mask

#### access

![Mask/access Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22160x160_proto%22%2C%22320x320_8class%22%2C%22480x480_9class%22%2C%22640x640_8class%22%2C%221280x720_hd%22%2C%221920x1080_fhd%22%2C%22160x160_proto%22%2C%22320x320_8class%22%2C%22480x480_9class%22%2C%22640x640_8class%22%2C%221280x720_hd%22%2C%221920x1080_fhd%22%2C%22160x160_proto%22%2C%22320x320_8class%22%2C%22480x480_9class%22%2C%22640x640_8class%22%2C%221280x720_hd%22%2C%221920x1080_fhd%22%2C%22160x160_proto%22%2C%22320x320_8class%22%2C%22480x480_9class%22%2C%22640x640_8class%22%2C%221280x720_hd%22%2C%221920x1080_fhd%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B1294.4%2C4746.7%2C11870.5%2C27890.7%2C100909.3%2C447328.5%2C1294.6%2C4700.5%2C11415.2%2C28099.2%2C100708.1%2C465935.0%2C1291.2%2C4727.6%2C11504.8%2C27994.2%2C98462.1%2C425495.9%2C5370.4%2C22178.2%2C49661.1%2C92609.2%2C219721.0%2C555514.9%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B78.97%2C81.77%2C81.94%2C78.78%2C78.94%2C81.49%2C73.23%2C73.14%2C72.32%2C73.71%2C73.56%2C73.24%2C59.41%2C59.29%2C58.91%2C58.71%2C58.71%2C58.9%2C4075.0%2C16147.4%2C36205.9%2C64167.7%2C144274.1%2C325051.9%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B1212.6%2C4629.1%2C11607.8%2C24274.8%2C91642.2%2C473000.9%2C1213.1%2C4777.8%2C11359.6%2C24423.7%2C100075.0%2C426138.4%2C1211.7%2C4948.8%2C11657.0%2C24611.7%2C101759.1%2C439832.1%2C5285.7%2C20934.6%2C48867.7%2C92821.4%2C219939.6%2C563819.0%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Mask/access%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=1440)

<details>
<summary>Numeric values for Mask/access (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| 160x160_proto | 1.2944 µs | 78.9651 ns | 1.2126 µs |
| 320x320_8class | 4.7467 µs | 81.7671 ns | 4.6291 µs |
| 480x480_9class | 11.8705 µs | 81.9368 ns | 11.6078 µs |
| 640x640_8class | 27.8907 µs | 78.7787 ns | 24.2748 µs |
| 1280x720_hd | 100.9093 µs | 78.9450 ns | 91.6422 µs |
| 1920x1080_fhd | 447.3285 µs | 81.4880 ns | 473.0009 µs |
| 160x160_proto | 1.2946 µs | 73.2316 ns | 1.2131 µs |
| 320x320_8class | 4.7005 µs | 73.1399 ns | 4.7778 µs |
| 480x480_9class | 11.4152 µs | 72.3158 ns | 11.3596 µs |
| 640x640_8class | 28.0992 µs | 73.7087 ns | 24.4237 µs |
| 1280x720_hd | 100.7081 µs | 73.5625 ns | 100.0750 µs |
| 1920x1080_fhd | 465.9350 µs | 73.2381 ns | 426.1384 µs |
| 160x160_proto | 1.2912 µs | 59.4115 ns | 1.2117 µs |
| 320x320_8class | 4.7276 µs | 59.2918 ns | 4.9488 µs |
| 480x480_9class | 11.5048 µs | 58.9056 ns | 11.6570 µs |
| 640x640_8class | 27.9942 µs | 58.7102 ns | 24.6117 µs |
| 1280x720_hd | 98.4621 µs | 58.7072 ns | 101.7591 µs |
| 1920x1080_fhd | 425.4959 µs | 58.9042 ns | 439.8321 µs |
| 160x160_proto | 5.3704 µs | 4.0750 µs | 5.2857 µs |
| 320x320_8class | 22.1782 µs | 16.1474 µs | 20.9346 µs |
| 480x480_9class | 49.6611 µs | 36.2059 µs | 48.8677 µs |
| 640x640_8class | 92.6092 µs | 64.1677 µs | 92.8214 µs |
| 1280x720_hd | 219.7210 µs | 144.2741 µs | 219.9396 µs |
| 1920x1080_fhd | 555.5149 µs | 325.0519 µs | 563.8190 µs |

</details>

#### decode

![Mask/decode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22160x160_proto%22%2C%22320x320_8class%22%2C%22480x480_9class%22%2C%22640x640_8class%22%2C%221280x720_hd%22%2C%221920x1080_fhd%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B1289.6%2C4867.7%2C12522.3%2C28535.0%2C90211.4%2C431348.2%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B53.72%2C53.67%2C53.79%2C53.51%2C54.11%2C53.59%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B1212.2%2C4917.6%2C12192.6%2C25470.5%2C107209.1%2C468762.1%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Mask/decode%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=360)

<details>
<summary>Numeric values for Mask/decode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| 160x160_proto | 1.2896 µs | 53.7176 ns | 1.2122 µs |
| 320x320_8class | 4.8677 µs | 53.6673 ns | 4.9176 µs |
| 480x480_9class | 12.5223 µs | 53.7862 ns | 12.1926 µs |
| 640x640_8class | 28.5350 µs | 53.5086 ns | 25.4705 µs |
| 1280x720_hd | 90.2114 µs | 54.1061 ns | 107.2091 µs |
| 1920x1080_fhd | 431.3482 µs | 53.5946 ns | 468.7621 µs |

</details>

#### encode

![Mask/encode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22160x160_proto%22%2C%22320x320_8class%22%2C%22480x480_9class%22%2C%22640x640_8class%22%2C%221280x720_hd%22%2C%221920x1080_fhd%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B1.4195%2C4.983%2C12.5538%2C26.3214%2C160.175%2C409.3522%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B1.2372%2C4.8711%2C12.266%2C27.5435%2C140.9371%2C418.9168%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B1.3136%2C5.2192%2C12.2519%2C22.8805%2C130.7927%2C365.4059%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Mask/encode%20Latency%20%28%5Cu00b5s%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28%5Cu00b5s%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=360)

<details>
<summary>Numeric values for Mask/encode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| 160x160_proto | 1.4195 µs | 1.2372 µs | 1.3136 µs |
| 320x320_8class | 4.9830 µs | 4.8711 µs | 5.2192 µs |
| 480x480_9class | 12.5538 µs | 12.2660 µs | 12.2519 µs |
| 640x640_8class | 26.3214 µs | 27.5435 µs | 22.8805 µs |
| 1280x720_hd | 160.1750 µs | 140.9371 µs | 130.7927 µs |
| 1920x1080_fhd | 409.3522 µs | 418.9168 µs | 365.4059 µs |

</details>

#### workflow

![Mask/workflow Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22640x640_8class%22%2C%22640x640_8class%22%2C%22640x640_8class%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B27376100.0%2C25882400.0%2C61386.9%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B47086.9%2C69232.9%2C24971.7%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B26845000.0%2C28013700.0%2C59151.2%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Mask/workflow%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Mask/workflow (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| 640x640_8class | 27.3761 ms | 47.0869 µs | 26.8450 ms |
| 640x640_8class | 28.8872 ms | 24.6893 ms | 27.6391 ms |
| 640x640_8class | 25.8824 ms | 69.2329 µs | 28.0137 ms |
| 640x640_8class | 61.3869 µs | 24.9717 µs | 59.1512 µs |

</details>

### PointCloud2

#### access

![PointCloud2/access Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22robosense_e1r%22%2C%22ouster_1024x10_128beam%22%2C%22ouster_2048x10_128beam%22%2C%22fusion_classes_ouster%22%2C%22robosense_e1r%22%2C%22ouster_1024x10_128beam%22%2C%22ouster_2048x10_128beam%22%2C%22fusion_classes_ouster%22%2C%22robosense_e1r%22%2C%22ouster_1024x10_128beam%22%2C%22ouster_2048x10_128beam%22%2C%22fusion_classes_ouster%22%2C%22robosense_e1r%22%2C%22ouster_1024x10_128beam%22%2C%22ouster_2048x10_128beam%22%2C%22fusion_classes_ouster%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B19758.7%2C317711.0%2C811800.0%2C406620.9%2C20122.6%2C313477.1%2C783377.6%2C433583.9%2C19946.3%2C317939.0%2C798699.4%2C435577.9%2C75496.3%2C429932.3%2C1174600.0%2C571103.8%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B206.89%2C212.3%2C208.23%2C207.19%2C157.58%2C156.5%2C157.14%2C155.52%2C136.06%2C136.29%2C136.62%2C134.78%2C53513.2%2C267265.7%2C554225.3%2C331125.4%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B22569.4%2C297948.8%2C791963.6%2C402602.8%2C22541.6%2C293758.0%2C775418.5%2C421787.5%2C22821.2%2C292347.9%2C800530.4%2C434643.7%2C76053.7%2C429258.2%2C1170100.0%2C550293.1%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22PointCloud2/access%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=960)

<details>
<summary>Numeric values for PointCloud2/access (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| robosense_e1r | 19.7587 µs | 206.8854 ns | 22.5694 µs |
| ouster_1024x10_128beam | 317.7110 µs | 212.3000 ns | 297.9488 µs |
| ouster_2048x10_128beam | 811.8000 µs | 208.2331 ns | 791.9636 µs |
| fusion_classes_ouster | 406.6209 µs | 207.1862 ns | 402.6028 µs |
| robosense_e1r | 20.1226 µs | 157.5845 ns | 22.5416 µs |
| ouster_1024x10_128beam | 313.4771 µs | 156.4957 ns | 293.7580 µs |
| ouster_2048x10_128beam | 783.3776 µs | 157.1411 ns | 775.4185 µs |
| fusion_classes_ouster | 433.5839 µs | 155.5243 ns | 421.7875 µs |
| robosense_e1r | 19.9463 µs | 136.0575 ns | 22.8212 µs |
| ouster_1024x10_128beam | 317.9390 µs | 136.2865 ns | 292.3479 µs |
| ouster_2048x10_128beam | 798.6994 µs | 136.6169 ns | 800.5304 µs |
| fusion_classes_ouster | 435.5779 µs | 134.7847 ns | 434.6437 µs |
| robosense_e1r | 75.4963 µs | 53.5132 µs | 76.0537 µs |
| ouster_1024x10_128beam | 429.9323 µs | 267.2657 µs | 429.2582 µs |
| ouster_2048x10_128beam | 1.1746 ms | 554.2253 µs | 1.1701 ms |
| fusion_classes_ouster | 571.1038 µs | 331.1254 µs | 550.2931 µs |

</details>

#### decode

![PointCloud2/decode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22robosense_e1r%22%2C%22ouster_1024x10_128beam%22%2C%22ouster_2048x10_128beam%22%2C%22fusion_classes_ouster%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B19674.8%2C322640.3%2C818061.1%2C430267.6%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B125.17%2C125.81%2C125.05%2C124.47%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B22985.3%2C288183.4%2C809203.2%2C435450.5%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22PointCloud2/decode%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=240)

<details>
<summary>Numeric values for PointCloud2/decode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| robosense_e1r | 19.6748 µs | 125.1708 ns | 22.9853 µs |
| ouster_1024x10_128beam | 322.6403 µs | 125.8109 ns | 288.1834 µs |
| ouster_2048x10_128beam | 818.0611 µs | 125.0510 ns | 809.2032 µs |
| fusion_classes_ouster | 430.2676 µs | 124.4719 ns | 435.4505 µs |

</details>

#### encode

![PointCloud2/encode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22robosense_e1r%22%2C%22ouster_1024x10_128beam%22%2C%22ouster_2048x10_128beam%22%2C%22fusion_classes_ouster%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B19.8388%2C338.8152%2C842.5911%2C470.0627%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B19.7481%2C312.0642%2C817.87%2C460.4982%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B19.8656%2C288.5611%2C804.9716%2C473.0711%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22PointCloud2/encode%20Latency%20%28%5Cu00b5s%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28%5Cu00b5s%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=240)

<details>
<summary>Numeric values for PointCloud2/encode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| robosense_e1r | 19.8388 µs | 19.7481 µs | 19.8656 µs |
| ouster_1024x10_128beam | 338.8152 µs | 312.0642 µs | 288.5611 µs |
| ouster_2048x10_128beam | 842.5911 µs | 817.8700 µs | 804.9716 µs |
| fusion_classes_ouster | 470.0627 µs | 460.4982 µs | 473.0711 µs |

</details>

#### workflow

![PointCloud2/workflow Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22robosense_e1r%22%2C%22robosense_e1r%22%2C%22robosense_e1r%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B20770800.0%2C20137200.0%2C55161.3%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B102040.9%2C142757.9%2C19458.3%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B20312400.0%2C22128000.0%2C47670.2%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22PointCloud2/workflow%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for PointCloud2/workflow (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| robosense_e1r | 20.7708 ms | 102.0409 µs | 20.3124 ms |
| robosense_e1r | 20.0935 ms | 19.9000 ms | 20.1324 ms |
| robosense_e1r | 20.1372 ms | 142.7579 µs | 22.1280 ms |
| robosense_e1r | 55.1613 µs | 19.4583 µs | 47.6702 µs |

</details>

### Pose

#### access

![Pose/access Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22all_fields%22%2C%22half_fields%22%2C%22one_field%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B262.5893%2C265.9587%2C289.551%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B32.6105%2C29.382%2C28.8569%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B169.7998%2C171.8289%2C171.5539%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Pose/access%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Pose/access (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| all_fields | 262.5893 ns | 32.6105 ns | 169.7998 ns |
| half_fields | 265.9587 ns | 29.3820 ns | 171.8289 ns |
| one_field | 289.5510 ns | 28.8569 ns | 171.5539 ns |

</details>

#### decode

![Pose/decode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22decode%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B281.4544%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B32.1142%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B170.6634%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Pose/decode%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Pose/decode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| decode | 281.4544 ns | 32.1142 ns | 170.6634 ns |

</details>

#### encode

![Pose/encode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22new%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B474.7823%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B144.5646%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B370.0437%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Pose/encode%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Pose/encode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| new | 474.7823 ns | 144.5646 ns | 370.0437 ns |

</details>

### RadarCube

#### access

![RadarCube/access Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22DRVEGRD169_ultra_short%22%2C%22DRVEGRD169_short%22%2C%22DRVEGRD169_long%22%2C%22DRVEGRD171_short%22%2C%22DRVEGRD171_extra_long%22%2C%22DRVEGRD169_ultra_short%22%2C%22DRVEGRD169_short%22%2C%22DRVEGRD169_long%22%2C%22DRVEGRD171_short%22%2C%22DRVEGRD171_extra_long%22%2C%22DRVEGRD169_ultra_short%22%2C%22DRVEGRD169_short%22%2C%22DRVEGRD169_long%22%2C%22DRVEGRD171_short%22%2C%22DRVEGRD171_extra_long%22%2C%22DRVEGRD169_ultra_short%22%2C%22DRVEGRD169_short%22%2C%22DRVEGRD169_long%22%2C%22DRVEGRD171_short%22%2C%22DRVEGRD171_extra_long%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B10428.2%2C78612.7%2C670178.6%2C676229.9%2C3408700.0%2C10298.8%2C76338.2%2C696653.8%2C671503.9%2C3418800.0%2C10161.9%2C80058.5%2C701669.8%2C692266.1%2C3424800.0%2C28864.6%2C131678.1%2C814847.3%2C815555.3%2C4553100.0%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B103.29%2C102.33%2C102.41%2C102.2%2C103.9%2C74.49%2C74.11%2C74.18%2C74.52%2C75.26%2C67.3%2C67.39%2C67.76%2C67.36%2C70.23%2C30865.0%2C123226.8%2C507926.0%2C506720.8%2C2043600.0%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B10360.9%2C88674.5%2C727819.3%2C725225.7%2C3456700.0%2C10378.9%2C89524.9%2C720140.5%2C726199.0%2C3449400.0%2C10442.0%2C109701.1%2C716173.2%2C707824.2%2C3458300.0%2C28018.5%2C130807.1%2C801649.2%2C800918.0%2C4510300.0%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22RadarCube/access%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=1200)

<details>
<summary>Numeric values for RadarCube/access (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| DRVEGRD169_ultra_short | 10.4282 µs | 103.2923 ns | 10.3609 µs |
| DRVEGRD169_short | 78.6127 µs | 102.3254 ns | 88.6745 µs |
| DRVEGRD169_long | 670.1786 µs | 102.4131 ns | 727.8193 µs |
| DRVEGRD171_short | 676.2299 µs | 102.2042 ns | 725.2257 µs |
| DRVEGRD171_extra_long | 3.4087 ms | 103.9000 ns | 3.4567 ms |
| DRVEGRD169_ultra_short | 10.2988 µs | 74.4898 ns | 10.3789 µs |
| DRVEGRD169_short | 76.3382 µs | 74.1132 ns | 89.5249 µs |
| DRVEGRD169_long | 696.6538 µs | 74.1801 ns | 720.1405 µs |
| DRVEGRD171_short | 671.5039 µs | 74.5231 ns | 726.1990 µs |
| DRVEGRD171_extra_long | 3.4188 ms | 75.2574 ns | 3.4494 ms |
| DRVEGRD169_ultra_short | 10.1619 µs | 67.3035 ns | 10.4420 µs |
| DRVEGRD169_short | 80.0585 µs | 67.3931 ns | 109.7011 µs |
| DRVEGRD169_long | 701.6698 µs | 67.7648 ns | 716.1732 µs |
| DRVEGRD171_short | 692.2661 µs | 67.3564 ns | 707.8242 µs |
| DRVEGRD171_extra_long | 3.4248 ms | 70.2322 ns | 3.4583 ms |
| DRVEGRD169_ultra_short | 28.8646 µs | 30.8650 µs | 28.0185 µs |
| DRVEGRD169_short | 131.6781 µs | 123.2268 µs | 130.8071 µs |
| DRVEGRD169_long | 814.8473 µs | 507.9260 µs | 801.6492 µs |
| DRVEGRD171_short | 815.5553 µs | 506.7208 µs | 800.9180 µs |
| DRVEGRD171_extra_long | 4.5531 ms | 2.0436 ms | 4.5103 ms |

</details>

#### decode

![RadarCube/decode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22DRVEGRD169_ultra_short%22%2C%22DRVEGRD169_short%22%2C%22DRVEGRD169_long%22%2C%22DRVEGRD171_short%22%2C%22DRVEGRD171_extra_long%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B10300.0%2C83657.3%2C696606.0%2C706256.6%2C3444700.0%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B57.54%2C57.55%2C57.55%2C57.56%2C57.55%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B10415.9%2C93015.0%2C720651.7%2C699875.9%2C3439800.0%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22RadarCube/decode%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=300)

<details>
<summary>Numeric values for RadarCube/decode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| DRVEGRD169_ultra_short | 10.3000 µs | 57.5405 ns | 10.4159 µs |
| DRVEGRD169_short | 83.6573 µs | 57.5486 ns | 93.0150 µs |
| DRVEGRD169_long | 696.6060 µs | 57.5476 ns | 720.6517 µs |
| DRVEGRD171_short | 706.2566 µs | 57.5595 ns | 699.8759 µs |
| DRVEGRD171_extra_long | 3.4447 ms | 57.5478 ns | 3.4398 ms |

</details>

#### encode

![RadarCube/encode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22DRVEGRD169_ultra_short%22%2C%22DRVEGRD169_short%22%2C%22DRVEGRD169_long%22%2C%22DRVEGRD171_short%22%2C%22DRVEGRD171_extra_long%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B0.0108%2C0.0865%2C0.7272%2C0.6848%2C3.4149%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B0.0103%2C0.0872%2C0.7555%2C0.7299%2C3.4292%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B0.0106%2C0.0879%2C0.7408%2C0.6864%2C3.3786%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22RadarCube/encode%20Latency%20%28ms%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ms%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=300)

<details>
<summary>Numeric values for RadarCube/encode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| DRVEGRD169_ultra_short | 10.8383 µs | 10.2840 µs | 10.5614 µs |
| DRVEGRD169_short | 86.5207 µs | 87.2061 µs | 87.8875 µs |
| DRVEGRD169_long | 727.2378 µs | 755.5119 µs | 740.8304 µs |
| DRVEGRD171_short | 684.8052 µs | 729.8604 µs | 686.3692 µs |
| DRVEGRD171_extra_long | 3.4149 ms | 3.4292 ms | 3.3786 ms |

</details>

#### workflow

![RadarCube/workflow Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22DRVEGRD169_short%22%2C%22DRVEGRD169_short%22%2C%22DRVEGRD169_short%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B123012600.0%2C76156000.0%2C163773.8%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B35348.6%2C76012.0%2C84473.0%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B81683000.0%2C83868600.0%2C144576.7%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22RadarCube/workflow%20Latency%20%28log%20scale%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22type%22%3A%22logarithmic%22%2C%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%2C%20log%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for RadarCube/workflow (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| DRVEGRD169_short | 123.0126 ms | 35.3486 µs | 81.6830 ms |
| DRVEGRD169_short | 65.1649 ms | 76.4399 ms | 76.2470 ms |
| DRVEGRD169_short | 76.1560 ms | 76.0120 µs | 83.8686 ms |
| DRVEGRD169_short | 163.7738 µs | 84.4730 µs | 144.5767 µs |

</details>

### Time

#### access

![Time/access Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22all_fields%22%2C%22half_fields%22%2C%22one_field%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B71.7603%2C71.9652%2C71.064%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B5.8371%2C5.4389%2C5.439%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B56.6965%2C57.1322%2C57.092%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Time/access%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Time/access (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| all_fields | 71.7603 ns | 5.8371 ns | 56.6965 ns |
| half_fields | 71.9652 ns | 5.4389 ns | 57.1322 ns |
| one_field | 71.0640 ns | 5.4390 ns | 57.0920 ns |

</details>

#### decode

![Time/decode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22decode%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B81.2822%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B5.8371%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B55.4262%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Time/decode%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Time/decode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| decode | 81.2822 ns | 5.8371 ns | 55.4262 ns |

</details>

#### encode

![Time/encode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22new%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B164.0575%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B83.9522%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B137.4111%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Time/encode%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Time/encode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| new | 164.0575 ns | 83.9522 ns | 137.4111 ns |

</details>

### Vector3

#### access

![Vector3/access Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22all_fields%22%2C%22half_fields%22%2C%22one_field%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B99.0574%2C100.1161%2C97.4196%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B14.8104%2C13.587%2C13.2448%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B72.4478%2C72.4062%2C71.8756%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Vector3/access%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Vector3/access (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| all_fields | 99.0574 ns | 14.8104 ns | 72.4478 ns |
| half_fields | 100.1161 ns | 13.5870 ns | 72.4062 ns |
| one_field | 97.4196 ns | 13.2448 ns | 71.8756 ns |

</details>

#### decode

![Vector3/decode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22decode%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B96.7536%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B14.3592%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B70.8787%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Vector3/decode%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Vector3/decode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| decode | 96.7536 ns | 14.3592 ns | 70.8787 ns |

</details>

#### encode

![Vector3/encode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22new%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22cyclonedds%22%2C%22data%22%3A%5B183.4232%5D%2C%22backgroundColor%22%3A%22rgba%2816%2C185%2C129%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2816%2C185%2C129%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B103.6186%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B168.598%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Vector3/encode%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Vector3/encode (click to expand)</summary>

| Variant | cyclonedds | edgefirst | fastcdr |
| ------- | ------- | ------- | ------- |
| new | 183.4232 ns | 103.6186 ns | 168.5980 ns |

</details>

### C++ Summary

- **cyclonedds:** 217 benchmarks
- **edgefirst:** 217 benchmarks
- **fastcdr:** 217 benchmarks
- **Total C++:** 651





## Appendix

### A. Per-impl codec primitive baselines (edgefirst vs. Fast-CDR vs. Cyclone DDS)

These tables report each impl's natural codec primitive cost — the
headline number for *that single library*. They are reported separately
from the cross-impl tables in Summary because the operations are not
directly comparable: Fast-CDR's and Cyclone DDS's `decode` include full
materialization; edgefirst's borrow records a small offset table and
stops. The honest comparison happens in the access and workflow patterns
above.

#### encode/new — build a fresh message buffer

| Message / variant | edgefirst | Fast-CDR | Cyclone DDS |
|---|---:|---:|---:|
| CompressedVideo (100KB) | 4.9 µs | 5.0 µs | 5.2 µs |
| CompressedVideo (10KB) | 638 ns | 741 ns | 833 ns |
| CompressedVideo (1MB) | 160 µs | 151 µs | 149 µs |
| CompressedVideo (500KB) | 45.9 µs | 39.8 µs | 35.6 µs |
| DmaBuffer | 148 ns | 434 ns | 592 ns |
| Header | 186 ns | 218 ns | 268 ns |
| Image (FHD_nv12) | 403 µs | 451 µs | 445 µs |
| Image (FHD_rgb8) | 1.7 ms | 1.6 ms | 1.7 ms |
| Image (FHD_yuyv) | 1.0 ms | 1.0 ms | 1.0 ms |
| Image (HD_nv12) | 110 µs | 105 µs | 90.4 µs |
| Image (HD_rgb8) | 603 µs | 865 µs | 620 µs |
| Image (HD_yuyv) | 298 µs | 555 µs | 322 µs |
| Image (VGA_nv12) | 16.5 µs | 17.0 µs | 17.0 µs |
| Image (VGA_rgb8) | 103 µs | 117 µs | 110 µs |
| Image (VGA_yuyv) | 50.1 µs | 52.8 µs | 53.1 µs |
| Mask (1280x720_hd) | 141 µs | 131 µs | 160 µs |
| Mask (160x160_proto) | 1.2 µs | 1.3 µs | 1.4 µs |
| Mask (1920x1080_fhd) | 419 µs | 365 µs | 409 µs |
| Mask (320x320_8class) | 4.9 µs | 5.2 µs | 5.0 µs |
| Mask (480x480_9class) | 12.3 µs | 12.3 µs | 12.6 µs |
| Mask (640x640_8class) | 27.5 µs | 22.9 µs | 26.3 µs |
| PointCloud2 (fusion_classes_ouster) | 460 µs | 473 µs | 470 µs |
| PointCloud2 (ouster_1024x10_128beam) | 312 µs | 289 µs | 339 µs |
| PointCloud2 (ouster_2048x10_128beam) | 818 µs | 805 µs | 843 µs |
| PointCloud2 (robosense_e1r) | 19.7 µs | 19.9 µs | 19.8 µs |
| Pose | 145 ns | 370 ns | 475 ns |
| RadarCube (DRVEGRD169_long) | 756 µs | 741 µs | 727 µs |
| RadarCube (DRVEGRD169_short) | 87.2 µs | 87.9 µs | 86.5 µs |
| RadarCube (DRVEGRD169_ultra_short) | 10.3 µs | 10.6 µs | 10.8 µs |
| RadarCube (DRVEGRD171_extra_long) | 3.4 ms | 3.4 ms | 3.4 ms |
| RadarCube (DRVEGRD171_short) | 730 µs | 686 µs | 685 µs |
| Time | 84 ns | 137 ns | 164 ns |
| Vector3 | 104 ns | 169 ns | 183 ns |

#### decode/decode — receive bytes → ready-to-use state

| Message / variant | edgefirst | Fast-CDR | Cyclone DDS |
|---|---:|---:|---:|
| CompressedVideo (100KB) | 55 ns | 4.8 µs | 4.9 µs |
| CompressedVideo (10KB) | 55 ns | 644 ns | 716 ns |
| CompressedVideo (1MB) | 55 ns | 139 µs | 138 µs |
| CompressedVideo (500KB) | 55 ns | 40.5 µs | 36.4 µs |
| DmaBuffer | 55 ns | 203 ns | 383 ns |
| Header | 45 ns | 105 ns | 162 ns |
| Image (FHD_nv12) | 56 ns | 399 µs | 393 µs |
| Image (FHD_rgb8) | 56 ns | 1.6 ms | 1.6 ms |
| Image (FHD_yuyv) | 56 ns | 1.2 ms | 1.2 ms |
| Image (HD_nv12) | 56 ns | 109 µs | 107 µs |
| Image (HD_rgb8) | 56 ns | 849 µs | 612 µs |
| Image (HD_yuyv) | 56 ns | 590 µs | 330 µs |
| Image (VGA_nv12) | 56 ns | 18.4 µs | 17.3 µs |
| Image (VGA_rgb8) | 56 ns | 98.1 µs | 107 µs |
| Image (VGA_yuyv) | 56 ns | 56.2 µs | 54.8 µs |
| Mask (1280x720_hd) | 54 ns | 107 µs | 90.2 µs |
| Mask (160x160_proto) | 54 ns | 1.2 µs | 1.3 µs |
| Mask (1920x1080_fhd) | 54 ns | 469 µs | 431 µs |
| Mask (320x320_8class) | 54 ns | 4.9 µs | 4.9 µs |
| Mask (480x480_9class) | 54 ns | 12.2 µs | 12.5 µs |
| Mask (640x640_8class) | 54 ns | 25.5 µs | 28.5 µs |
| PointCloud2 (fusion_classes_ouster) | 124 ns | 435 µs | 430 µs |
| PointCloud2 (ouster_1024x10_128beam) | 126 ns | 288 µs | 323 µs |
| PointCloud2 (ouster_2048x10_128beam) | 125 ns | 809 µs | 818 µs |
| PointCloud2 (robosense_e1r) | 125 ns | 23.0 µs | 19.7 µs |
| Pose | 32 ns | 171 ns | 281 ns |
| RadarCube (DRVEGRD169_long) | 58 ns | 721 µs | 697 µs |
| RadarCube (DRVEGRD169_short) | 58 ns | 93.0 µs | 83.7 µs |
| RadarCube (DRVEGRD169_ultra_short) | 58 ns | 10.4 µs | 10.3 µs |
| RadarCube (DRVEGRD171_extra_long) | 58 ns | 3.4 ms | 3.4 ms |
| RadarCube (DRVEGRD171_short) | 58 ns | 700 µs | 706 µs |
| Time | 6 ns | 55 ns | 81 ns |
| Vector3 | 14 ns | 71 ns | 97 ns |

### B. How zero-copy borrow works

`from_cdr(&[u8])` scans the buffer once to record a small offset table
(typically 1–7 `usize` values). Field accessors then read at known offsets
with no further validation. The scan itself is O(number of variable-length
fields) — fixed-size regions and typed arrays are skipped in constant time.

The buffer is never copied or reallocated. String fields return `&str` slices
into the buffer. Byte payloads return `&[u8]`. Typed numeric arrays return
`&[i16]`, `&[f32]`, etc. directly reinterpreted from the wire bytes.

`new()` computes the exact buffer size in a dry-run pass (`CdrSizer`), then
writes all fields in a single allocation. Bulk data (byte blobs, typed arrays)
is written via `memcpy` rather than element-by-element.

### C. Raw results

After running `benchmark.sh`, results are stored at:

```
results/<utc-timestamp>/
├── edgefirst.json      # Google Benchmark JSON for edgefirst backend
├── fastcdr.json        # Google Benchmark JSON for Fast-CDR backend
├── system.json         # uname, cpuinfo, freq governor, toolchain versions
└── BENCHMARKS-<ts>.md  # Rendered report (local-only, not committed)
```

