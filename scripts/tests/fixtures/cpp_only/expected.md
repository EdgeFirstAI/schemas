## 📊 On-Target Benchmark Results

**Target Hardware:** NXP i.MX 8M Plus (Cortex-A53 @ 1.8GHz)
**Architecture:** aarch64
**Total Benchmarks:** 10 (C++: 10)

## 🔷 C++ Benchmarks

*EdgeFirst zero-copy CDR vs. eProsima Fast-CDR comparison.*

### Header

#### access

![Header/access Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22all_fields%22%2C%22one_field%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B24.1%2C15.7%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B28.6%2C18.4%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Header/access%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Header/access (click to expand)</summary>

| Variant | edgefirst | fastcdr | Speedup |
| ------- | ------- | ------- | ------- |
| all_fields | 24.1000 ns | 28.6000 ns | 1.19x |
| one_field | 15.7000 ns | 18.4000 ns | 1.17x |

</details>

#### decode

![Header/decode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22decode%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B13.3%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B48.2%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Header/decode%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Header/decode (click to expand)</summary>

| Variant | edgefirst | fastcdr | Speedup |
| ------- | ------- | ------- | ------- |
| decode | 13.3000 ns | 48.2000 ns | 3.62x |

</details>

#### encode

![Header/encode Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22new%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B58.9%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B210.5%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Header/encode%20Latency%20%28ns%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28ns%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Header/encode (click to expand)</summary>

| Variant | edgefirst | fastcdr | Speedup |
| ------- | ------- | ------- | ------- |
| new | 58.9000 ns | 210.5000 ns | 3.57x |

</details>

#### workflow

![Header/workflow Chart](https://quickchart.io/chart?c=%7B%22type%22%3A%22horizontalBar%22%2C%22data%22%3A%7B%22labels%22%3A%5B%22sub_loop%22%5D%2C%22datasets%22%3A%5B%7B%22label%22%3A%22edgefirst%22%2C%22data%22%3A%5B24.0%5D%2C%22backgroundColor%22%3A%22rgba%2859%2C130%2C246%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%2859%2C130%2C246%2C1%29%22%2C%22borderWidth%22%3A1%7D%2C%7B%22label%22%3A%22fastcdr%22%2C%22data%22%3A%5B72.0%5D%2C%22backgroundColor%22%3A%22rgba%28245%2C158%2C11%2C0.85%29%22%2C%22borderColor%22%3A%22rgba%28245%2C158%2C11%2C1%29%22%2C%22borderWidth%22%3A1%7D%5D%7D%2C%22options%22%3A%7B%22title%22%3A%7B%22display%22%3Atrue%2C%22text%22%3A%22Header/workflow%20Latency%20%28%5Cu00b5s%29%22%2C%22fontSize%22%3A11%7D%2C%22legend%22%3A%7B%22labels%22%3A%7B%22fontSize%22%3A9%2C%22boxWidth%22%3A12%7D%7D%2C%22scales%22%3A%7B%22xAxes%22%3A%5B%7B%22scaleLabel%22%3A%7B%22display%22%3Atrue%2C%22labelString%22%3A%22Time%20%28%5Cu00b5s%29%22%2C%22fontSize%22%3A9%7D%2C%22ticks%22%3A%7B%22beginAtZero%22%3Atrue%2C%22fontSize%22%3A8%7D%7D%5D%2C%22yAxes%22%3A%5B%7B%22ticks%22%3A%7B%22fontSize%22%3A8%7D%7D%5D%7D%2C%22plugins%22%3A%7B%22datalabels%22%3A%7B%22display%22%3Afalse%7D%7D%7D%7D&w=600&h=200)

<details>
<summary>Numeric values for Header/workflow (click to expand)</summary>

| Variant | edgefirst | fastcdr | Speedup |
| ------- | ------- | ------- | ------- |
| sub_loop | 24.0000 µs | 72.0000 µs | 3.00x |

</details>

### C++ Summary

- **edgefirst:** 5 benchmarks
- **fastcdr:** 5 benchmarks
- **Total C++:** 10

