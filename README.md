<div align="center">

# 🔄 Inverted Pendulum on Cart — Control Systems Case Study

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023a%2B-orange?logo=mathworks&logoColor=white)](https://www.mathworks.com/products/matlab.html)
[![Simulink](https://img.shields.io/badge/Simulink-R2023a%2B-blue?logo=mathworks&logoColor=white)](https://www.mathworks.com/products/simulink.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Control](https://img.shields.io/badge/Control-PID%20%7C%20LQR-blueviolet)](https://en.wikipedia.org/wiki/Control_theory)

**A comprehensive study of mathematical modeling, classical PID control, and optimal LQR control applied to the inverted pendulum on cart — an underactuated benchmark system in control engineering.**

</div>

---

## 📋 Table of Contents

1. [Project Overview](#-project-overview)
2. [System Diagram](#-system-diagram)
3. [Key Features](#-key-features)
4. [System Architecture](#-system-architecture)
5. [Performance Requirements](#-performance-requirements)
6. [Modeling Approach](#-modeling-approach)
7. [Control Strategies](#-control-strategies)
8. [Multi-Fidelity Simulation](#-multi-fidelity-simulation)
9. [Results & Insights](#-results--insights)
10. [System Limitations](#-system-limitations)
11. [Project Structure](#-project-structure)
12. [Installation & Setup](#-installation--setup)
13. [Usage](#-usage)
14. [Contributing](#-contributing)
15. [License](#-license)
16. [Contact & Support](#-contact--support)

---

## 🎯 Project Overview

The **inverted pendulum on a cart** is one of the most studied benchmark problems in control theory. It is inherently unstable — a small perturbation will cause the pendulum to fall — making it an ideal testbed for evaluating and comparing control strategies.

This project provides a complete, end-to-end control systems case study including:

- **Mathematical derivation** of the nonlinear and linearized equations of motion
- **Open-loop analysis** demonstrating instability and the need for active control
- **Classical PID control** with interactive gain tuning
- **Optimal LQR control** based on minimizing a quadratic cost function
- **Multi-fidelity simulation** pipeline: MATLAB analytical → Simulink linear model → Simscape Multibody (3-D physics)

The project is structured to support both academic learning and professional reference.

---

## 🖼 System Diagram

```
           F (control force →)
           ↓
  ┌──────────────────────────┐
  │          CART            │ ← Mass M, position x
  └──────────────────────────┘
            │
            │ pivot
            │  ╲  θ (pendulum angle from vertical)
            │   ╲
            │    ● ← Mass m, length l from pivot to center of mass lc = l/2
```

**Physical variables:**

| Symbol | Description | Value |
|--------|-------------|-------|
| `M` | Cart mass | 0.5 kg |
| `m` | Pendulum mass | 0.2 kg |
| `l` | Pendulum rod length | 0.1 m |
| `lc` | Pivot to center of mass (`l/2`) | 0.05 m |
| `J` | Moment of inertia of rod (`ml²/12`) | 1.67 × 10⁻⁴ kg·m² |
| `g` | Gravitational acceleration | 9.81 m/s² |
| `x` | Cart position | state variable |
| `θ` | Pendulum angle from vertical | state variable |
| `F` | Horizontal force on cart | control input |

---

## ✨ Key Features

- 📐 **State-space representation** of linearized pendulum dynamics
- 📊 **Open-loop analysis** — instability proof, controllability & observability checks
- 🎛️ **Interactive PID tuner** — real-time gain adjustment with live plots and pole-zero map
- ⚡ **LQR optimal control** — automatic gain synthesis via MATLAB `lqr()` solving the Algebraic Riccati Equation
- 🔬 **Three simulation fidelity levels**: pure MATLAB, Simulink, and Simscape Multibody
- 📈 **Full performance metrics** — settling time, overshoot, steady-state error, control energy
- 🏗️ **Modular, well-commented MATLAB code** suitable for education and research

---

## 🏗 System Architecture

### State Vector

The system state is defined as:

$$\mathbf{x} = \begin{bmatrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{bmatrix}$$

where `x` is the cart position, `ẋ` is cart velocity, `θ` is the pendulum angle from vertical, and `θ̇` is the angular velocity.

### Linearized State-Space Equations

After linearization around the upright equilibrium (`θ = 0`, `θ̇ = 0`), the system is described by:

$$\dot{\mathbf{x}} = A\mathbf{x} + B u$$

$$y = C\mathbf{x} + D u$$

where the determinant Δ = `(M + m)(mlc² + J) − (mlc)²` and:

$$A = \begin{bmatrix} 0 & 1 & 0 & 0 \\ 0 & 0 & \dfrac{-m^2 g l_c^2}{\Delta} & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & \dfrac{(M+m)mgl_c}{\Delta} & 0 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ \dfrac{ml_c^2+J}{\Delta} \\ 0 \\ \dfrac{-ml_c}{\Delta} \end{bmatrix}$$

$$C = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0 \end{bmatrix}, \quad D = \mathbf{0}$$

The outputs are cart position `x` and pendulum angle `θ`.

---

## 📏 Performance Requirements

The control design targets the following specifications (initial condition: `θ₀ = 0.1 rad`, `θ̇₀ = 1 rad/s`):

| Metric | Requirement |
|--------|-------------|
| **Rise Time** | < 2 s |
| **Settling Time (2% criterion)** | < 5 s |
| **Overshoot** | < 10% |
| **Steady-State Error (θ)** | < 0.01 rad |
| **Steady-State Error (x)** | Bounded (LQR) |

---

## 🔬 Modeling Approach

### Nonlinear Equations of Motion

The full nonlinear dynamics are derived from the Euler-Lagrange equations. The coupled equations are:

$$(M + m)\ddot{x} + ml_c\ddot{\theta}\cos\theta - ml_c\dot{\theta}^2\sin\theta = F$$

$$(ml_c^2 + J)\ddot{\theta} + ml_c\ddot{x}\cos\theta - mgl_c\sin\theta = 0$$

### Linearization

Around the upright equilibrium point `(x=0, ẋ=0, θ=0, θ̇=0)`, the small-angle approximation is applied:

$$\sin\theta \approx \theta, \quad \cos\theta \approx 1, \quad \dot{\theta}^2\theta \approx 0$$

This yields the linear state-space model given in [System Architecture](#-system-architecture), which enables classical linear control design and analysis.

---

## 🎮 Control Strategies

### PID Control (Classical)

The PID controller applies a feedback force based on the angle error:

$$u(t) = K_p\,\theta(t) + K_i\int\theta\,dt + K_d\,\dot{\theta}(t)$$

The state-feedback gain vector is `K_pid = [0, 0, Kp, Kd]`, controlling only the pendulum angle (without cart position feedback in the basic implementation).

**Tuning**: An interactive MATLAB GUI (`inverted_pendulum_pid_tuner.m`) provides real-time slider-based gain tuning with live closed-loop response plots and a pole-zero map.

---

### LQR Control (Optimal)

The LQR controller minimizes the infinite-horizon quadratic cost function:

$$J = \int_0^{\infty} \left( \mathbf{x}^T Q\,\mathbf{x} + u^T R\,u \right) dt$$

MATLAB's `lqr()` function solves the Algebraic Riccati Equation to obtain the optimal full-state feedback gain `K_lqr`:

$$u = -K_\text{lqr}\,\mathbf{x}$$

**Weighting matrices used:**

$$Q = \text{diag}(1,\ 1,\ 10,\ 1), \quad R = 1$$

The high weight on `θ` (`Q(3,3) = 10`) prioritizes pendulum angle stabilization.

---

### Comparison Table

| Feature | PID | LQR |
|---------|-----|-----|
| **Design approach** | Trial-and-error / empirical | Systematic / mathematical optimization |
| **States controlled** | Angle `θ` only (basic) | Full state `[x, ẋ, θ, θ̇]` |
| **Optimality** | Not guaranteed | Optimal w.r.t. cost function J |
| **Cart position** | Drifts freely | Regulated to zero |
| **Tuning parameters** | Kp, Ki, Kd | Weighting matrices Q, R |
| **Robustness** | Moderate | Good (guaranteed gain margin ≥ 6 dB) |
| **Implementation** | Simple | Requires full state measurement/estimation |
| **Settling time** | ~2–4 s (tuned) | < 2 s (typical) |
| **Overshoot** | Depends on tuning | Low (optimized) |

---

## 🔭 Multi-Fidelity Simulation

This project uses a three-tier simulation approach, progressively increasing physical fidelity:

```
Level 1 — MATLAB Analytical
   ↓  (linearized state-space, fast, ideal for control design)
Level 2 — Simulink Linear Model  (Simulink_linear_model.slx)
   ↓  (block-diagram integration, interfaces with MATLAB workspace)
Level 3 — Simscape Multibody    (Simscape_Multibody_simulation.slx)
         (3-D rigid-body physics, nonlinear, closest to hardware)
```

| Level | Model | Fidelity | Use Case |
|-------|-------|----------|----------|
| 1 | `inverted_pendulum_openloop.m` / `*_PID.m` / `LQR_for_simulink.m` | Low–Medium | Control design, gain tuning |
| 2 | `Simulink_linear_model.slx` | Medium | Verification, timing analysis |
| 3 | `Simscape_Multibody_simulation.slx` | High | Pre-hardware validation |

The LQR gains designed at Level 1 are directly exported to Level 2 and Level 3 via MATLAB workspace variables, enabling a seamless design-to-validation workflow.

---

## 📊 Results & Insights

### Open-Loop Behavior

The open-loop system is **unstable** — a small initial angle perturbation grows exponentially. The system has one unstable pole (positive real part) at approximately `+5.9`, confirming the need for active feedback control.

### PID vs. LQR Performance

| Metric | PID (tuned) | LQR (Q=diag[1,1,10,1], R=1) |
|--------|-------------|------------------------------|
| Settling time (θ) | ~2–4 s | < 2 s |
| Max overshoot (θ) | ~5–15% | < 5% |
| Steady-state θ error | ≈ 0 rad | ≈ 0 rad |
| Steady-state x drift | Yes (uncontrolled) | ≈ 0 m |
| Max control force | Moderate | Moderate–High (initial transient) |
| Control energy ∫F²dt | Higher | Lower (optimal by design) |

### Key Takeaways

- **LQR outperforms PID** on all metrics for this system because it simultaneously controls both cart position and pendulum angle with a mathematically optimal gain.
- **PID can achieve angle stabilization** but the cart drifts without additional position feedback loop.
- **Multi-fidelity simulation** confirms that the linear-model-designed LQR gains transfer successfully to the nonlinear Simscape model, validating the linearization assumption for small angles.

---

## ⚠️ System Limitations

| Limitation | Description |
|------------|-------------|
| **Small-angle assumption** | The linearized model is only valid for `|θ| ≲ 15°` (~0.26 rad). Large angle perturbations require the full nonlinear model. |
| **No disturbance rejection** | The current designs do not include explicit disturbance observer or integral action on cart position. |
| **Parameter sensitivity** | LQR performance degrades if physical parameters (M, m, l) deviate significantly from the values used in design. |
| **Actuator limits** | The linear models assume unlimited force input; real implementations are saturated. |
| **Full-state feedback** | LQR requires all four states to be measured or estimated (e.g., via a Luenberger observer or Kalman filter). |
| **Friction neglected** | Cart and pivot friction are not modeled, which may cause steady-state error on hardware. |

---

## 📁 Project Structure

```
inverted-pendulum-case-study/
│
├── 📄 README.md                          # This file
│
├── 📂 parameters/
│   └── system_parameters.m               # Shared physical parameters (M, m, l, g, …)
│
├── 📂 models/
│   ├── 📂 Open_loop_Matlab/
│   │   └── inverted_pendulum_openloop.m  # Open-loop simulation & stability analysis
│   │
│   ├── 📂 PID_Matlab/
│   │   ├── inverted_pendulum_PID.m       # PID closed-loop simulation (fixed gains)
│   │   └── inverted_pendulum_pid_tuner.m # Interactive GUI PID tuner with live plots
│   │
│   └── 📂 LQR_Matlab+simulink/
│       ├── LQR_for_simulink.m            # LQR design + post-processing for Simulink output
│       ├── LQR_for_simscape_multibody_simulation.m  # LQR design for Simscape
│       ├── Simulink_linear_model.slx     # Simulink linear state-space model
│       └── Simscape_Multibody_simulation.slx        # 3-D Simscape Multibody model
│
└── 📂 results/
    ├── 📂 Open_loop_results/
    │   ├── Open_loop_results.fig         # Open-loop state response figure
    │   └── Open_loop_stability-and-controllability.png
    ├── 📂 PID_results/
    │   ├── PID_results.fig               # PID closed-loop response
    │   └── poles.png                     # Pole-zero map
    └── 📂 LQR_results/
        ├── angle.fig                     # Pendulum angle response
        ├── x.fig                         # Cart position response
        └── control_Force.fig             # Control force over time
```

---

## 🛠 Installation & Setup

### Prerequisites

| Tool | Version | Notes |
|------|---------|-------|
| MATLAB | R2023a or later | Core requirement |
| Control System Toolbox | ≥ R2023a | Required for `ss()`, `lqr()`, `initial()` |
| Simulink | ≥ R2023a | Required for `.slx` models |
| Simscape & Simscape Multibody | ≥ R2023a | Required for 3-D physics simulation |

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/AmiraMerzougui/inverted-pendulum-case-study.git
   cd inverted-pendulum-case-study
   ```

2. **Open MATLAB** and set the working directory to the cloned folder:
   ```matlab
   cd('/path/to/inverted-pendulum-case-study')
   ```

3. **Add the project to the MATLAB path:**
   ```matlab
   addpath(genpath('.'))
   ```

4. **Verify toolbox availability:**
   ```matlab
   ver('control')    % Control System Toolbox
   ver('simulink')   % Simulink
   ver('physmod')    % Simscape
   ```

---

## 🚀 Usage

### 1. Open-Loop Simulation

Verify the open-loop instability and check controllability/observability:

```matlab
cd models/Open_loop_Matlab
run('inverted_pendulum_openloop.m')
```

Expected output: state trajectories diverge; console confirms the system is unstable but controllable and observable.

---

### 2. PID Control — Fixed Gains

Run a closed-loop PID simulation with pre-set gains:

```matlab
cd models/PID_Matlab
run('inverted_pendulum_PID.m')
```

Generates three plots: pendulum angle, cart position, and control force.

---

### 3. PID Control — Interactive Tuner

Launch the interactive GUI to tune PID gains in real time:

```matlab
cd models/PID_Matlab
inverted_pendulum_pid_tuner()
```

A GUI window opens with Kp, Ki, Kd sliders and live updates of the response, pole map, and performance metrics.

---

### 4. LQR Control — MATLAB (analytical)

Design the optimal LQR controller and simulate via MATLAB `initial()`:

```matlab
cd models/LQR_Matlab+simulink
run('LQR_for_simulink.m')    % designs K_lqr and sets up workspace for Simulink
```

---

### 5. LQR Control — Simulink

1. Run `LQR_for_simulink.m` first (populates workspace variables).
2. Open and run the Simulink model:
   ```matlab
   open_system('Simulink_linear_model.slx')
   sim('Simulink_linear_model')
   ```
3. The script post-processes the output and generates Figures 1–5 with performance metrics.

---

### 6. LQR Control — Simscape Multibody (highest fidelity)

1. Run `LQR_for_simscape_multibody_simulation.m` to compute gains.
2. Open and simulate:
   ```matlab
   open_system('Simscape_Multibody_simulation.slx')
   sim('Simscape_Multibody_simulation')
   ```

---

## 🤝 Contributing

Contributions are welcome! To contribute:

1. **Fork** the repository on GitHub.
2. **Create a feature branch:**
   ```bash
   git checkout -b feature/my-improvement
   ```
3. **Make your changes** — keep MATLAB code style consistent (comments, section headers, variable naming).
4. **Test your changes** by running the relevant scripts and verifying plots.
5. **Commit** with a clear message:
   ```bash
   git commit -m "Add observer-based LQR with Kalman filter"
   ```
6. **Push** and **open a Pull Request** against `main`.

### Ideas for Contributions

- 🔍 State observer (Luenberger / Kalman filter) for output-feedback LQR
- 🌀 Nonlinear MPC controller
- 🔊 Disturbance rejection and robustness analysis
- 📱 Real-time hardware implementation (Arduino / Raspberry Pi)
- 🧪 Additional unit tests for parameter edge cases

---

## 📄 License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

```
MIT License — Copyright (c) 2024 Amira Merzougui
Permission is granted, free of charge, to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software.
```

---

## 📬 Contact & Support

**Author:** Amira Merzougui

- 🐙 **GitHub:** [@AmiraMerzougui](https://github.com/AmiraMerzougui)
- 📧 **Issues:** Please use the [GitHub Issues](https://github.com/AmiraMerzougui/inverted-pendulum-case-study/issues) tracker for bug reports and feature requests.

---

<div align="center">

*Built with ❤️ for control systems engineering education and research.*

</div>