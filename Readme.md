# MicroSafe-RL

**Ultra-lightweight AI Safety Layer for Robotics & LLM-driven systems.**

535 ns deterministic latency · 24 bytes RAM · STM32 / Arduino / ESP32  
Hardware-validated · Gymnasium-compatible · SB3 / RLLib / CleanRL ready

---

## What it does

MicroSafe-RL sits between an AI model and physical execution, enforcing:

- **Bounded outputs** — hard action clipping at configurable safety limits
- **Anomaly suppression** — EMA + MAD + velocity coherence detection
- **Gravity modulation** — penalty-driven attenuation of unsafe actions
- **Lightweight CBF** — barrier function enforcement without a QP solver
- **Real-time risk scoring** — O(1) per-step, composable safety index

---

## Architecture

```
LLM / RL Policy
      ↓
 MicroSafeRL            ← EMA + MAD + coherence + gravity  (C++ / Python)
      ↓
 MicroSafeRL_CBF        ← barrier function, no QP solver required
      ↓
 MicroSafeController    ← unified safety score across both layers
      ↓
  Actuator / Robot
```

**Safety stack positioning:**

| Layer | What it checks | Latency | Where it runs |
|---|---|---|---|
| Shielding | Formal logic specifications | ms | Cloud / PC |
| Standard CBF | Kinematic safe sets (QP solver) | ms | PC / FPGA |
| **MicroSafe-RL** | Signal-level physics, runtime | **535 ns** | **Microcontroller** |

MicroSafe-RL is not a replacement for CBF — it is the layer that runs where standard CBF cannot.

---

## Core algorithm (MicroSafeRL)

```cpp
// Per-step safety enforcement — O(1), no heap allocation
ema_mean = λ * ema_mean + (1-λ) * sensor;
abs_dev  = |sensor - ema_mean|;
ema_mad  = λ * ema_mad  + (1-λ) * abs_dev;
velocity = |sensor - prev_sensor|;

coherence = 1 / (1 + abs_dev * β);
penalty   = min(κ * (ema_mad + α*(1-coherence) + 0.3*velocity), max_penalty);
gravity   = max(0, 1 - penalty * gravity_factor);
safe      = clamp(ai_action * gravity, min_limit, max_limit);
```

Combined with `MicroSafeRL_CBF` for barrier enforcement:

```cpp
float MicroSafeController::apply(float ai, float sensor) {
    float rl_safe = rl.apply_safe_control(ai, sensor);  // physics layer
    float final   = cbf.apply_safe_control(rl_safe);    // barrier layer
    return final;
}
```

---

## Benchmark results

Tested against Kalman filter and PLC baselines:

| Metric | MicroSafe-RL | Kalman | PLC |
|---|---|---|---|
| Detection margin (mean) | **19.2** | 11.0 | 8.1 |
| ROC — TPR | **1.00** | 1.00 | 1.00 |
| ROC — FPR | **≈ 0** | ≈ 0 | ≈ 0 |
| Latency | **535 ns** | ms-range | ms-range |
| RAM footprint | **24 bytes** | KB-range | KB-range |

Noise stress test: raw spike +6.5 → clipped output ±1.468, zero hardware impact.

Full benchmark figures: [`/figures`](./figures)

---

## File structure

```
├── include/
│   ├── MicroSafeRL.h           # Core safety layer (EMA + MAD + gravity)
│   ├── MicroSafeRL_CBF.h       # Lightweight barrier function
│   └── MicroSafeController.h   # Combined RL + CBF controller
├── python/
│   └── microsafe_llm_plugin.py # LLM ↔ actuator bridge (Ollama-compatible)
├── examples/
│   └── basic_demo/basic_demo.ino
├── data/
│   └── demo_log.csv
└── docs/
    └── safety_concept.md
```

---

## Quick start

**Embedded (Arduino / STM32):**

```cpp
#include "MicroSafeController.h"

MicroSafeController ctrl;

void loop() {
    float ai_cmd = get_ai_action();
    float sensor = read_sensor();
    float safe   = ctrl.apply(ai_cmd, sensor);
    actuator.set(safe);
}
```

**Python + LLM (Ollama):**

```bash
python python/microsafe_llm_plugin.py
```

Closed-loop pipeline:
```
User command → LLM (gemma2 / llama3) → MicroSafeRL → CBF → Actuator → feedback → LLM
```

**Gymnasium wrapper:**

```python
from microsafe_rl import MicroSafeWrapper
env = MicroSafeWrapper(your_env, mode="SAFE")
```

---

## Use cases

- Autonomous robotics and drones
- LLM-driven physical agents (local inference)
- Industrial edge control systems
- Autonomous surface vessels (USV)
- Safety-critical embedded AI

---

## Publications

- Zenodo: [doi link]
- arXiv: cs.RO — pending
- Hardware validation: STM32F401, two deterministic latency tiers

---

## License

Copyright © 2025 Dimitar Kretski. All rights reserved.

**Non-commercial use** (research, education, personal projects):
Free to use, modify, and share with attribution.

**Commercial use** (products, services, integration into commercial systems):
Requires a separate commercial license.
Contact: kretski1@gmail.com

This repository is **not** MIT-licensed.
Commercial use without a license agreement is prohibited.
