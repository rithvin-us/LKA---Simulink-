# Lane Keeping Assist (LKA) - MATLAB & Simulink
## SAE / aBAJA (Autonomous Baja) Competition

This repository contains the simulation, control algorithms, vehicle dynamics modeling, and verification framework for the **Lane Keeping Assist (LKA)** system developed for the SAE / aBAJA autonomous challenge.

The repository follows **MathWorks Automotive Advisory Board (MAAB)** guidelines and the **ISO 11270** standard for LKAS verification.

---

## 1. Quickstart

1. Launch MATLAB (R2022b or later recommended).
2. Set your MATLAB working directory to the root of this repository (`LKA - Matlab/`).
3. Run the startup script in the MATLAB Command Window:
   ```matlab
   run('config/startup.m')
   ```
   *This automatically registers all subfolders to the MATLAB path, sets `work/` as the Simulink cache directory, and loads vehicle, controller, and sensor parameters into the base workspace.*
4. Run the baseline simulation and view performance KPIs:
   ```matlab
   run('scripts/run_baseline_simulation.m')
   ```
5. When finished, clean up the workspace and paths:
   ```matlab
   run('config/shutdown.m')
   ```

---

## 2. Repository Layout

```plaintext
LKA - Matlab/
├── .gitignore                      # Tailored ignore rules for MATLAB & Simulink
├── README.md                       # Main project documentation (this file)
│
├── config/                         # Environment & lifecycle configuration
│   ├── startup.m                   # Adds paths, sets cache folders, loads parameters
│   └── shutdown.m                  # Clears paths, closes models, cleans up cache
│
├── data/                           # Inputs, calibration parameters, and buses
│   ├── bus_definitions/            # Simulink Bus object definitions
│   │   └── create_lka_buses.m
│   ├── parameters/                 # Physical parameters & controller gains
│   │   ├── vehicle_params.m        # Vehicle inertia, mass, geometry, tire stiffness
│   │   ├── lka_controller_params.m # Stanley/MPC gains, saturation limits
│   │   └── sensor_params.m         # Camera field-of-view, latency, noise
│   └── scenarios/                  # Test track geometry (.mat / generators)
│       └── generate_test_scenarios.m
│
├── models/                         # Simulink models & component libraries
│   └── README.md                   # Modeling architecture & model referencing guide
│
├── src/                            # MATLAB algorithmic source code
│   ├── controllers/                # Steering control laws (Stanley, Pure Pursuit)
│   │   ├── stanley_steer_control.m
│   │   └── pure_pursuit_steer.m
│   ├── perception/                 # Lateral error & heading angle computation
│   │   └── compute_lateral_error.m
│   └── utilities/                  # Lowpass filters and rate limiters
│       └── lowpass_steer_filter.m
│
├── scripts/                        # Automation & analysis workflows
│   ├── run_baseline_simulation.m   # Standalone runner and visualization
│   └── evaluate_kpis.m             # Computes ISO 11270 pass/fail performance KPIs
│
├── tests/                          # Verification and Validation (V&V)
│   ├── unit_tests/                 # MATLAB unit test suites
│   │   └── test_stanley_controller.m
│   └── scenario_tests/             # Track compliance test suites
│       └── test_iso_11270_compliance.m
│
├── docs/                           # Engineering reports, standards, and sources
│   ├── mathematical_models.md      # Mathematical derivation of vehicle dynamics & control
│   └── references/                 # Academic and regulatory citations
│       ├── references.bib          # BibTeX citations
│       └── iso_11270_lka_compliance.md # ISO 11270 requirements summary
│
└── work/                           # Simulink cache folder (ignored by git)
```

---

## 3. Required Toolboxes

The models and scripts in this repository use:
- **MATLAB & Simulink**
- **Automated Driving Toolbox** (Scenario generation, coordinate transforms, lane detection)
- **Vehicle Dynamics Blockset** / **Simscape Driveline** (Vehicle plant representation)
- **Control System Toolbox** / **Model Predictive Control Toolbox** (Controller synthesis)

---

## 4. Key Sources and References

1. **Rajamani, Rajesh (2012)**. *Vehicle Dynamics and Control*. 2nd Edition, Springer. *(Used for 2-DOF bicycle model and lateral deviation kinematics).*
2. **Snider, Jarrod M. (2009)**. *Automatic Steering Methods for Autonomous Automobile Path Tracking*. Carnegie Mellon University, CMU-RI-TR-09-08. *(Used for the Stanley and Pure Pursuit steering laws).*
3. **ISO 11270:2014**. *Intelligent transport systems — Lane keeping assistance systems (LKAS) — Performance requirements and test procedures*. *(Used for pass/fail KPI validation).*
4. **MathWorks Automotive Advisory Board (MAAB)**. *Control Algorithm Modeling Guidelines Using MATLAB, Simulink, and Stateflow*.

For complete BibTeX citations, see [`docs/references/references.bib`](file:///e:/Abaja%20Labs/2026/LKA%20-%20Matlab/docs/references/references.bib).
