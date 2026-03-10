# OTBU — Online Terrain Barrier Update

> Python simulation of the **OTBU** algorithm from *"Robust and Near-Fuel-Optimal Landing Guidance with Online Terrain Avoidance"* (Basar & Ghosh, IEEE RA-L). Implements real-time terrain barrier generation integrated with the MSS-OTALG guidance law for autonomous precision soft landing on Mars.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [System Architecture](#2-system-architecture)
3. [Mathematical Foundations](#3-mathematical-foundations)
   - 3.1 [Equations of Motion](#31-equations-of-motion)
   - 3.2 [Time-to-Go Estimation](#32-time-to-go-estimation)
   - 3.3 [MSS-OTALG Guidance Law](#33-mss-otalg-guidance-law)
   - 3.4 [Sliding Mode Control](#34-sliding-mode-control)
   - 3.5 [OTBU Algorithm — Terrain Barrier Update](#35-otbu-algorithm--terrain-barrier-update)
   - 3.6 [Divert Repulsion Field](#36-divert-repulsion-field)
   - 3.7 [Composite Acceleration Command (Full)](#37-composite-acceleration-command-full)
4. [Module Reference](#4-module-reference)
   - [`main.py`](#mainpy)
   - [`rk45.py`](#rk45py)
   - [`tgo_min.py`](#tgo_minpy)
   - [`a0.py`](#a0py)
   - [`barriers.py`](#barrierspy)
   - [`divert.py`](#divertpy)
   - [`sliding.py`](#slidingpy)
   - [`saturation.py`](#saturationpy)
   - [`show.py`](#showpy)
   - [`TerrainMap/unityTerrainController.py`](#terrainmapunityterraincontrollerpy)
   - [`TerrainDataExporter.cs`](#terraindataexportercs)
5. [Unity Integration](#5-unity-integration)
6. [Repository Structure & Missing Files](#6-repository-structure--missing-files)
7. [Dependencies & Setup](#7-dependencies--setup)
8. [Running the Simulation](#8-running-the-simulation)
9. [Parameters Reference](#9-parameters-reference)
10. [Known Limitations & Future Work](#10-known-limitations--future-work)
11. [References](#11-references)

---

## 1. Project Overview

This repository is a Python/Unity simulation implementation of the **Online Terrain Barrier Update (OTBU)** algorithm, presented in:

> **S. Z. Basar and S. Ghosh**, *"Robust and Near-Fuel-Optimal Landing Guidance with Online Terrain Avoidance,"* IEEE Robotics and Automation Letters (RA-L). [[Paper]](#11-references)

OTBU generates terrain barrier functions on the fly using onboard elevation data (a world-coordinate-registered grid map from a downward-facing camera), and feeds them into the **Multiple Sliding Surfaces – Optimal Terrain Avoidance Landing Guidance (MSS-OTALG)** law from [[1]](#11-references) to steer the spacecraft away from hazardous terrain while achieving precision soft landing.

The key components are:

- **MSS-OTALG** [[1]](#11-references): the base guidance law, combining ZEM/ZEV optimal guidance, a terrain divert term, and a Multiple Sliding Surfaces (MSS) robust correction term
- **OTBU (Algorithm 1)**: online update of polynomial barrier functions $\rho_x, \rho_y, \rho_z$ at adaptive intervals, based on real-time terrain peak detection via H-maxima transform
- **Unity terrain simulation**: a Unity scene provides live elevation map data over TCP, substituting for an onboard depth camera in simulation
- **Adaptive RK45 integration**: trajectory propagation with variable step size

The vehicle dynamics follow a 3-DOF model in a local East-North-Up (ENU) frame centred on the landing site. Terrain is validated against real Martian elevation maps from NASA's HiRISE database (DTM: *"Possible Vent in Ceraunius Fossae"*, [[15]](#11-references)).

---

## 2. System Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                          main.py                               │
│   Sets ICs, calls tgo_init, initAcc, then drives rk45 loop     │
└────────────┬───────────────────────────────────────────────────┘
             │  f(t, y)  called at each integration step
             ▼
┌────────────────────────────────────────────────────────────────┐
│                        ODE RHS (in main.py)                    │
│                                                                │
│  ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌───────────┐  │
│  │ barriers │   │  divert  │   │ sliding  │   │saturation │  │
│  │ (terrain │──▶│(repulsion│   │  (SMC    │──▶│ (boundary │  │
│  │  query)  │   │ gradient)│   │ surface) │   │  layer)   │  │
│  └────┬─────┘   └────┬─────┘   └────┬─────┘   └─────┬─────┘  │
│       │              │              │                │        │
│       └──────────────┴──────────────┴────────────────┘        │
│                             │                                  │
│                    a = a_ogl + a_divert + a_smc                │
│                             │                                  │
│                    ┌────────▼────────┐                         │
│                    │  Thrust model   │                         │
│                    │ (T = a·m, capped│                         │
│                    │  + noise band)  │                         │
│                    └────────┬────────┘                         │
│                             │                                  │
│              ẋ = [v, g + T/m, ṁ, ȧ]                           │
└─────────────────────────────┬──────────────────────────────────┘
                              │
              ┌───────────────▼───────────────┐
              │         rk45.py               │
              │  Adaptive RK45 integrator     │
              └───────────────┬───────────────┘
                              │
              ┌───────────────▼───────────────┐
              │      rk45_results.npz          │
              └───────────────┬───────────────┘
                              │
              ┌───────────────▼───────────────┐
              │           show.py             │
              │  Load & visualise + Unity cam │
              └───────────────────────────────┘

Unity TCP Server (port 5555)
  TerrainDataExporter.cs  ◄──────────────►  TerrainMap/unityTerrainController.py
  (C# TCP listener,                  (Python TCP client,
   camera + terrain export)           terrain data loader)
```

---

## 3. Mathematical Foundations

The mathematical basis of this implementation follows the paper [[OTBU]](#11-references) and the base guidance law from [[1]](#11-references). Notation is preserved from those works.

### 3.1 Equations of Motion

The landing site is taken as the origin of a local **East-North-Up (ENU)** coordinate frame. The lander is modelled as a 3-DOF point mass with variable mass and first-order actuator dynamics (added in this implementation):

$$\dot{\mathbf{r}} = \mathbf{v}$$

$$\dot{\mathbf{v}} = \mathbf{a}_c + \mathbf{g} + \mathbf{a}_p$$

$$\dot{m} = -\frac{\|\mathbf{T}\|}{I_{sp}\,g_e}$$

$$\dot{\mathbf{a}}_c = \frac{\mathbf{a}_{cmd} - \mathbf{a}_c}{\tau}$$

where $\mathbf{r}, \mathbf{v} \in \mathbb{R}^3$ are position and velocity in the ENU frame, $\mathbf{g} = [0,\, 0,\, -g]^T$ is Mars surface gravity (valid for the low-altitude powered descent stage), $\mathbf{a}_c = \mathbf{T}/m$ is the guidance acceleration command, $\mathbf{a}_p$ is bounded perturbation acceleration, $I_{sp}$ is specific impulse, $g_e$ is Earth standard gravity, and $\tau$ is the actuator lag time constant.

The thrust vector is capped at $T_{max}$ and a ±5% uniform noise band is applied per component to model actuator uncertainty:

$$T_i \sim \mathcal{U}(0.95\,T_{cmd,i},\; 1.05\,T_{cmd,i})$$

---

### 3.2 Time-to-Go Estimation

The optimal time-to-go $t_{go}$ is computed using the method of [[14]](#11-references), which solves for the energy-optimal terminal time for a constrained terminal velocity guidance law. This yields a quartic polynomial in $t_{go}$:

$$D \cdot t_{go}^4 + A \cdot t_{go}^2 + B \cdot t_{go} + C = 0$$

where:

$$A = -2\left(\|\mathbf{v}_0\|^2 + \mathbf{v}_f \cdot \mathbf{v}_0 + \|\mathbf{v}_f\|^2\right), \quad B = 12\,(\mathbf{r}_f - \mathbf{r}_0) \cdot (\mathbf{v}_0 + \mathbf{v}_f)$$

$$C = -18\,\|\mathbf{r}_f - \mathbf{r}_0\|^2, \quad D = \tfrac{1}{2}\,\|\mathbf{g}\|^2$$

All real, non-negative roots are extracted and the minimum is taken as $t_{go,opt}$. As noted in the paper, this method does not account for time lost due to divert manoeuvres, so a **20 s buffer** is added:

```python
tf = tgo_init(r0, v0, rf, vf) + 20
```

---

### 3.3 MSS-OTALG Guidance Law

The base guidance law is **MSS-OTALG** [[1]](#11-references). The full guidance acceleration command is (Eq. (2) of [[OTBU]](#11-references)):

$$\mathbf{a}_c = \frac{6}{t_{go}^2}\,\text{ZEM} - \frac{2}{t_{go}}\,\text{ZEV} + \mathbf{p}\,\frac{t_{go}^2}{12} - \mathbf{\Phi}\,\text{sgn}(\mathbf{s}_2)$$

where $t_{go} \triangleq t_f - t$, and:

- **ZEM** (Zero Effort Miss): $\text{ZEM} = \mathbf{r}_f^d - \left[\mathbf{r}(t) + \mathbf{v}\,t_{go} + \tfrac{1}{2}\mathbf{g}\,t_{go}^2\right]$  
- **ZEV** (Zero Effort Velocity): $\text{ZEV} = \mathbf{v}_f^d - \left[\mathbf{v}(t) + \mathbf{g}\,t_{go}\right]$
- $\mathbf{p}\,t_{go}^2/12$: the **divert manoeuvre term**, where $\mathbf{p} = [\dot{p}_{r_x}, \dot{p}_{r_y}, \dot{p}_{r_z}]^T$ is the barrier repulsion gradient (see §3.6)
- $-\mathbf{\Phi}\,\text{sgn}(\mathbf{s}_2)$: the **MSS robust term** (see §3.4)

> **Implementation note**: The paper uses the hard sign function $\text{sgn}(\mathbf{s}_2)$. This implementation substitutes a **saturation function** with boundary layer width $\varepsilon = 0.1$ (via `saturation.py`) to eliminate high-frequency chattering in the numerical simulation:
> $$\text{sgn}(s_2) \;\approx\; \text{sat}(s_2,\,\varepsilon) = \begin{cases} s_2/\varepsilon & |s_2| < \varepsilon \\ \text{sgn}(s_2) & |s_2| \geq \varepsilon \end{cases}$$

---

### 3.4 Sliding Mode Control

The MSS robust term requires two sliding surfaces. **Surface 1** is the position error:

$$\mathbf{s}_1 = \mathbf{r} - \mathbf{r}_f^d, \qquad \dot{\mathbf{s}}_1 = \mathbf{v} - \mathbf{v}_f^d$$

**Surface 2** is the composite surface with time-varying gain (virtual controller gain $\Lambda = 2$):

$$\mathbf{s}_2 = \dot{\mathbf{s}}_1 + \frac{\Lambda}{t_{go}}\,\mathbf{s}_1$$

The gain $\Lambda/t_{go}$ contracts to infinity as $t_{go} \to 0$, ensuring finite-time convergence to the sliding surface.

The **switching gain matrix** $\mathbf{\Phi}$ is chosen to dominate the divert term and perturbation bounds:

$$\mathbf{\Phi} = k_1\,a_{p,max}\,\mathbf{I} + k_2\,\frac{|\mathbf{p}|}{12}\,t_{go}^2$$

where $k_1 = 0.2$, $k_2 = 0.8$, and $a_{p,max}$ is the maximum perturbation acceleration.

---

### 3.5 OTBU Algorithm — Terrain Barrier Update

**Algorithm 1** (from [[OTBU]](#11-references)) generates the polynomial barrier functions $\rho_i$ online. It is initialised with: estimated highest point $\mathbf{r}_H$, desired landing site $\mathbf{r}_f^d$, last update time $t_{last}$, safety margin $\delta$, update intervals $0 < \Delta t_1 < \Delta t_2$, switching distance $\Theta$, peak suppression level $\alpha$, and barrier order $m$.

```
Algorithm 1: Online Terrain Barrier Update (OTBU)
Initialise: rH, rf, tlast, δ, Δt1, Δt2, Θ, α, m

procedure OTBU(F, r, t):
  # 1. Set update interval based on distance to landing site
  if ‖r − rf‖ < Θ:  Δt ← Δt1
  else:              Δt ← Δt2

  if t − tlast ≥ Δt or t == 0:
    tlast ← t

    # 2. Peak detection on terrain height map F
    im     ← ExtendedMaxima(F, α)     # H-maxima transform, threshold α
    s      ← GetCentroids(im)         # centroids of regional maxima
    rpeak  ← max(s)                   # highest centroid

    # 3. Vertical barrier
    if rz − rHz > δ:
      ρz ← rHz + δ;   b ← rH         # use pre-estimated highest point
    else:
      ρz ← rpeakz + δ;  b ← rpeak    # switch to local peak from camera

    # 4. Lateral barriers (power-law curves anchored at b and rf)
    for ν = x, y:
      if bν > rfν:
        iν ← (bν − rfν)^m / (bz − rfz)
        jν ← (bν − rfν)^m − iν·bz
        kν ← rfν
        ρν ← (iν·rz + jν)^(1/m) + kν
      elif bν < rfν:
        pν ← (−bν + rfν)^m / (bz − rfz)
        qν ← (−bν + rfν)^m − pν·bz
        Tν ← −rfν
        ρν ← −((pν·rz + qν)^(1/m) + Tν)
      else:
        ρν ← Inf        # canyon case: no lateral barrier in this axis
```

**Notes on the algorithm:**

- The H-maxima transform (`ExtendedMaxima`) filters out noise peaks below threshold $\alpha$, retaining only significant terrain features. Implemented in Python via `skimage.morphology.extrema.h_maxima` (paper uses MATLAB `imextendedmax`).
- The vertical barrier switches from $\mathbf{r}_H$ (pre-estimated highest point from orbital images) to $\mathbf{r}_{peak}$ (onboard camera) once the spacecraft descends below $r_{H_z} + \delta$. This minimises guidance switching.
- The lateral barriers are polynomial curves of order $m = 0.5$ in $r_z$, shaped so they pass through both $\mathbf{b}$ and $\mathbf{r}_f^d$.
- **Corner case (canyon)**: if $b_\nu = r_{f\nu}^d$, $\rho_\nu = \infty$. The remaining defined barrier(s) plus $\rho_z$ still prevent a crash. In this implementation, `np.inf` is set for that axis.


---

### 3.6 Divert Repulsion Field

For each axis $i \in \{x, y, z\}$, given the signed distance from the barrier $d_i \triangleq r_i - \rho_i$, the repulsion gradient component is (Eq. (3) of [[OTBU]](#11-references)):

$$\psi_i = \frac{\ell_{2,i}}{d_i^2 + \ell_{1,i}}, \qquad \dot{p}_{r_i} = \frac{d_i\,\ell_{2,i}\,\ell_{3,i}\,e^{-\psi_i}}{(d_i^2 + \ell_{1,i})^2}$$

with constants $\ell_{1,i} = 1$, $\ell_{2,i} = 9500$, $\ell_{3,i} = 500$ for all $i$. This field is smooth, vanishes both on the barrier ($d_i \to 0$) and far from it ($d_i \to \infty$), and produces a repulsion lobe at intermediate distances.

---

### 3.7 Composite Acceleration Command (Full)

The complete MSS-OTALG guidance command, as implemented, is:

$$\mathbf{a}_c = \underbrace{\frac{6\,\text{ZEM}}{t_{go}^2} - \frac{2\,\text{ZEV}}{t_{go}}}_{\text{ZEM/ZEV optimal term}} + \underbrace{\mathbf{p}\,\frac{t_{go}^2}{12}}_{\text{OTBU divert term}} + \underbrace{-\mathbf{\Phi}\,\text{sat}(\mathbf{s}_2,\varepsilon)}_{\text{MSS robust term}}$$

This is passed through a first-order actuator lag (time constant $\tau$) and converted to thrust via $\mathbf{T} = \mathbf{a}_c \cdot m$.

---

## 4. Module Reference

### `main.py`

**Entry point.** Defines initial and terminal conditions, instantiates all guidance parameters, defines the ODE right-hand side `f(t, y)`, and calls the integrator.

| Symbol | Value | Description |
|---|---|---|
| `r0` | `[704, 145, 721]` m | Initial position (ENU-like frame) |
| `v0` | `[-55.9, 22.9, -83.8]` m/s | Initial velocity |
| `m0` | `1905` kg | Initial mass |
| `rf` | `[453, 786, 145]` m | Target landing site |
| `vf` | `[0, 0, 0.5]` m/s | Terminal velocity |
| `Isp` | `225` s | Engine specific impulse |
| `Tmax` | `8 × 3100 / √3` N | Maximum thrust magnitude |
| `m_min` | `1455` kg | Minimum (dry) mass |
| `tau` | `5/90` s | Actuator time constant |
| `eps` | `0.1` | SMC boundary layer width |

**State mutation note**: `state` dict is used as a closure to persist `t_last` and `rho_last` across ODE evaluations without making them global. This is required because RK45 sub-steps must not trigger unnecessary Unity queries.

---

### `rk45.py`

**Adaptive Runge-Kutta-Fehlberg (RK45) integrator.**

Implements the classic Dormand-Prince / Fehlberg Butcher tableau. At each step, both a 4th-order (`y4`) and 5th-order (`y5`) solution are computed. The local truncation error estimate is:

$$E = \max_i |y5_i - y4_i|$$

Step size is updated by:

$$h_{new} = 0.9 \cdot h \cdot \left(\frac{\text{tol}}{E}\right)^{0.2}$$

**Early termination** conditions:
- `‖r - rf‖ < 0.1` m (landing convergence)
- Any state element becomes non-finite

Default tolerance: `tol = 1e-6`, initial step size `h = 0.001` s.

---

### `tgo_min.py`

**Time-to-go initialisation via polynomial root finding.**

Solves the quartic $D t^4 + A t^2 + B t + C = 0$ using `np.roots`. Extracts the minimum real non-negative root as the energy-optimal $t_{go}$.

```python
tgo_init(r0, v0, rf, vf) -> tf  # final time = t_go,opt + 20s buffer
```

---

### `a0.py`

**Initial acceleration command computation.**

Evaluates the full composite guidance law at $t=0$ to provide a consistent initial condition for the actuator state $\mathbf{a}_c(0)$. This avoids a step discontinuity at $t=0^+$ and ensures the actuator dynamics start from the correct equilibrium.

```python
initAcc(r0, v0, rf, vf, eps, tf, highest, t=0) -> (a0, t_last, rho_init)
```

---

### `barriers.py`

**Real-time terrain barrier computation.**

Queries Unity terrain data via `UnityTerrainController`, applies h-maxima peak detection, and computes the 3D barrier surface $\boldsymbol{\rho}(r_z)$ as described in §3.5.

```python
barriers(r, rf, highest, rho_last, t_last, t) -> (rho, t_last)
```

**Caching**: If $|t - t_{last}| < \Delta t$, the previous `rho_last` is returned without a Unity query, saving TCP round-trips during RK45 sub-steps.


---

### `divert.py`

**Barrier repulsion field computation.**

```python
divert(r, rho) -> (d, p)
```

- `d = r - rho`: signed distance from barrier
- `p`: repulsion gradient vector (see §3.6)

---

### `sliding.py`

**Sliding mode surface computation.**

```python
sliding(r, rf, v, vf, tgo, lam=2) -> (s1, s2)
```

Returns $\mathbf{s}_1$ (position error) and $\mathbf{s}_2$ (composite sliding surface).

---

### `saturation.py`

**Vectorised saturation function with boundary layer.**

```python
saturation(x, width=1.0) -> np.ndarray
```

Implemented via `np.where` for efficient elementwise evaluation over the 3-vector sliding surface.

---

### `show.py`

**Post-processing visualisation.**

Loads `rk45_results.npz`, replays the trajectory through Unity's camera controller, and plots mass history and acceleration history using matplotlib.

Coordinate remapping: Unity uses a Y-up convention, so `r[i,1]` (Python z) maps to Unity Y and `r[i,2]` (Python y) maps to Unity Z:

```python
controller.move_camera(r[i,0], r[i,2], r[i,1])
```

---

### `TerrainMap/unityTerrainController.py`

**TCP client for the Unity terrain interface.** Located at `./TerrainMap/unityTerrainController.py`. Sends JSON-encoded commands to the `TerrainDataExporter` TCP server and deserialises the returned terrain height map.

| Method | Description |
|---|---|
| `send_command(command_dict)` | Low-level: opens a TCP connection, sends JSON, returns parsed JSON response |
| `move_camera(x, y, z)` | Moves the Unity orthographic camera to world position |
| `move_and_export(x, y, z)` | Atomic move + terrain export command |
| `load_terrain_data()` | Reads `terrain_data.json` from disk and returns `{x, y, z}` arrays |

The data schema returned by `load_terrain_data()`:

```json
{
  "x": [float, ...],   // 1D, length = resolution (world X coords)
  "y": [float, ...],   // 1D, length = resolution (world Z coords, Unity convention)
  "z": [[float, ...]]  // 2D (resolution × resolution), height values
}
```

Each call to `send_command` opens and closes a fresh TCP socket — there is no persistent connection. This keeps the protocol simple but adds per-call overhead; for high-frequency querying, connection pooling would reduce latency.

---

### `TerrainMap/Assets/TerrainDataExporter.cs`

**Unity-side TCP server.** Attached to a `GameObject` in the Unity scene. Runs a background `Thread` listening on `127.0.0.1:5555`. Also provides real-time **trajectory visualisation** via a `LineRenderer` component.

#### TCP Commands

| Command | Behaviour |
|---|---|
| `move_camera` | Queues camera position update on main thread; appends point to `LineRenderer` if `showTrajectory` is enabled |
| `move_and_export` | Queues camera move + triggers `ExportTerrainData()` |
| `clear_trajectory` | Clears the `LineRenderer` trajectory buffer |

#### Trajectory Visualisation

On `Start()`, a child `GameObject` named `"CameraTrajectory"` is created and a `LineRenderer` is attached to it. As the camera moves, each new position is appended to `trajectoryPoints` and immediately reflected in the renderer. This gives a live 3D trace of the descent path inside the Unity scene.

Inspector-exposed parameters:

| Field | Type | Default | Description |
|---|---|---|---|
| `showTrajectory` | `bool` | `true` | Toggle trajectory rendering on/off |
| `trajectoryColor` | `Color` | Red | Line colour |
| `trajectoryWidth` | `float` | `0.5` | Line width in world units |
| `trajectoryMaterial` | `Material` | `null` | Override material; falls back to `Sprites/Default` |

**Thread safety**: Camera movement and terrain export are deferred to `Update()` (main thread) via flag fields (`pendingCameraPosition`, `shouldExportData`) — required because Unity scene objects must only be accessed from the main thread.

> **Note**: `get_camera_position` and `get_highest_point` commands are implemented in `GetHighestPoint()` internally but are not yet exposed as TCP commands in `ProcessCommand`. They can be added by extending the switch block.

---

## 5. Unity Integration

The Python guidance loop and the Unity terrain environment communicate over a **local TCP socket** (loopback, port 5555):

```
Python (barriers.py)                        Unity (TerrainDataExporter.cs)
─────────────────────                        ──────────────────────
move_and_export(x, y, z)  ──── JSON ──────►  ProcessCommand()
                                              │
                          ◄── {"status":"success"} ──
                                              │
                                           Update() fires:
                                           - Camera moved
                                           - ExportTerrainData() called
                                           - terrain_data.json written
                                              │
load_terrain_data()  ─── reads file ────────►  terrain_data.json
```

**Coordinate convention**: Python uses `[x, y, z]` = `[East, North, Up]`. Unity uses `[x, y, z]` = `[East, Up, North]`. The mapping applied throughout the codebase is:

| Python | Unity |
|---|---|
| `r[0]` (x) | `x` |
| `r[1]` (y, North) | `z` |
| `r[2]` (z, Up) | `y` |

The TCP message protocol is JSON with UTF-8 encoding. Maximum buffer size is currently 1024 bytes. There is a known race condition between `Thread.Sleep(1)` in `move_and_export` and `Update()` execution — see §10.

---

## 6. Repository Structure & Missing Files

```
project-root/
├── main.py
├── rk45.py
├── tgo_min.py
├── a0.py
├── barriers.py
├── divert.py
├── sliding.py
├── saturation.py
├── show.py
├── rk45_results.npz                    ← generated at runtime
│
└── TerrainMap/
    ├── __init__.py
    ├── unityTerrainController.py       ← Python TCP client
    └── Assets/
        ├── TerrainDataExporter.cs      ← attach to a GameObject in scene
        └── terrain_data.json           ← generated at runtime by Unity
```

Update `self.data_path` inside `TerrainMap/unityTerrainController.py` to point to your `TerrainMap/Assets/` folder before running.

---

## 7. Dependencies & Setup

### Python

```
Python >= 3.10
numpy
scipy
matplotlib
scikit-image   # for extrema.h_maxima, regionprops, label
```

Install:

```bash
pip install numpy scipy matplotlib scikit-image
```

### Unity

- Unity **2021.3 LTS** or later (tested configuration)
- A Terrain object must be assigned to the `terrain` field on the `TerrainDataExporter` component
- An Orthographic Camera must be assigned to `orthographicCamera`
- `TerrainDataExporter.cs` must be attached to an active `GameObject` in the scene
- The scene must be in **Play mode** before running `main.py`

### File path configuration

Update the following hardcoded paths before running:

| File | Variable | Description |
|---|---|---|
| `TerrainMap/unityTerrainController.py` | `self.data_path` | Path to `TerrainMap/Assets/terrain_data.json` |
| `show.py` | `output_path` | Path to `rk45_results.npz` |

---

## 8. Running the Simulation

1. Open the Unity project and enter **Play mode**. Confirm the TCP server is running (check Console for `"Server started on port 5555"`).

2. Ensure the `TerrainMap/` package is set up (§6).

3. From the project root:

```bash
python main.py
```

This will integrate the trajectory and save results to `rk45_results.npz`.

4. To replay and visualise:

```bash
python show.py
```

This streams the trajectory back through the Unity camera and plots mass and acceleration time histories.

---

## 9. Parameters Reference

Values from paper Table I [[OTBU]](#11-references), with code discrepancies noted.

| Parameter | Symbol | Paper Value | Code Value | Units | Source |
|---|---|---|---|---|---|
| Mars gravity | $\mathbf{g}$ | $[0,0,-g]^T$ | `[0,0,-3.7114]` | m/s² | `main.py` |
| Specific impulse | $I_{sp}$ | 225 | 225 | s | `main.py` |
| Earth gravity (mass flow) | $g_e$ | — | 9.807 | m/s² | `main.py` |
| Max thrust | $T_{max}$ | 31,000 | 31,000 (ODE) / ≈14,318 (ap_max calc) | N | `main.py` |
| Actuator lag time constant | $\tau$ | 0.0556 | 5/90 ≈ 0.0556 | s | `main.py` |
| Initial mass | $m_0$ | 1905 | 1905 | kg | `main.py` |
| SMC gain | $k_1$ | 0.2 | 0.2 ✓ | — | `main.py` |
| SMC gain | $k_2$ | 0.8 | 0.8 ✓ | — | `main.py` |
| Virtual controller gain | $\Lambda$ | 2 | 2 (`lam`) | — | `sliding.py` |
| Safety margin | $\delta$ | 95.5 | 95.5 (`delta_r`) | m | `barriers.py` |
| Switching distance | $\Theta$ | 1000 | 1000 ✓ | m | `barriers.py` |
| Update interval (near) | $\Delta t_1$ | 0.05 | 0.05 | s | `barriers.py` |
| Update interval (far) | $\Delta t_2$ | 0.1 | 0.1 | s | `barriers.py` |
| ExtendedMaxima threshold | $\alpha$ | 10 | 10 | m | `barriers.py` |
| Barrier polynomial order | $m$ | 0.5 | 0.5 | — | `barriers.py` |
| Repulsion constant | $\ell_{1,i}$ | 1 | 1 | — | `divert.py` |
| Repulsion constant | $\ell_{2,i}$ | 9500 | 9500 | — | `divert.py` |
| Repulsion constant | $\ell_{3,i}$ | 500 | 500 | — | `divert.py` |
| Boundary layer width | $\varepsilon$ | — (uses sgn) | 0.1 | — | `main.py` |
| Estimated highest point | $\mathbf{r}_H$ | `[480.39, 751.74, 858.82]`$^T$ | ⚠️ `[725, 581, 224]` (hardcoded estimate) | m | `main.py` |
| RK45 tolerance | — | — | 1e-6 | — | `rk45.py` |
| RK45 initial step | — | — | 0.001 | s | `rk45.py` |
| Terrain grid resolution | — | — | 100 × 100 | px | `TerrainDataExporter.cs` |

---

## 10. Known Limitations & Future Work

### Bugs & Discrepancies with the Paper

- **T_max inconsistency** (`main.py`): The ODE thrust cap uses `T_max = 10*3100 = 31000 N` (matches paper Table I), but the $a_{p,max}$ computation uses `Tmax = 8*3100/√3 ≈ 14318 N`. Only the ODE cap value matches the paper.
- **Highest point hardcoded**: `main.py` sets `highest = np.array([725, 581, 224])` with a comment acknowledging it is a guess. Paper Algorithm 1 takes $\mathbf{r}_H$ from pre-processed orbital images; it should be queried from Unity at startup via `get_highest_point()`.
- **Race condition** (`TerrainDataExporter.cs`): `Thread.Sleep(1)` in `move_and_export` does not guarantee the camera has moved before `shouldExportData = true` is processed. A `ManualResetEvent` handshake is needed.
- **Unity TCP buffer**: The 1024-byte receive buffer in `HandleClient` will truncate large JSON payloads.

### Limitations

- Single dominant peak assumed per terrain patch (`regionProps[0]` only) — multi-peak handling is deferred
- `sgn(s2)` replaced with `sat(s2, ε)` to avoid chattering; this widens the convergence boundary layer
- No fuel constraint enforcement in the guidance law
- Sequential (blocking) Unity TCP calls add latency per integration step

### Planned Extensions

- Multi-peak obstacle handling
- Successive convexification (SCvx) for fuel-optimal re-planning
- Hardware-in-the-loop testing with IMU noise injection
- Replace file-based terrain transfer with shared memory or UDP

---

## 11. References

**Primary paper (this implementation):**

> **[OTBU]** S. Z. Basar and S. Ghosh, *"Robust and Near-Fuel-Optimal Landing Guidance with Online Terrain Avoidance,"* IEEE Robotics and Automation Letters (RA-L), 2025.

**References from the paper:**

> **[1]** Z. B. Sheikh and S. Ghosh, *"Robust near-optimal landing guidance for hazardous terrain using multiple sliding surfaces,"* Advances in Space Research, Jan. 2025.

> **[11]** P. Soille, *Morphological Image Analysis.* Springer Berlin Heidelberg, 1999. *(H-maxima transform)*

> **[13]** S. Z. Basar and S. Ghosh, *"Fuel-optimal powered descent guidance for hazardous terrain,"* IFAC-PapersOnLine, vol. 56, no. 2, pp. 6018–6023, 2023, 22nd IFAC World Congress.

> **[14]** Y. Guo, M. Hawkins, and B. Wie, *"Optimal feedback guidance algorithms for planetary landing and asteroid intercept,"* AAS/AIAA Astrodynamics Specialist Conference, 2011. *(Time-to-go estimation method)*

> **[15]** NASA/JPL-Caltech/UArizona, *"Possible Vent in Ceraunius Fossae (ESP\_029122\_2065),"* HiRISE DTM. [https://www.uahirise.org/dtm/ESP_029122_2065](https://www.uahirise.org/dtm/ESP_029122_2065) *(Martian terrain data used in simulations)*
