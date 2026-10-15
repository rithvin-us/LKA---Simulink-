# ISO 11270:2014 Standard Overview for Lane Keeping Assistance Systems (LKAS)

This document extracts the essential performance metrics, operating boundaries, and test procedures required for LKAS validation as outlined in **ISO 11270:2014** (*Intelligent transport systems — Lane keeping assistance systems — Performance requirements and test procedures*).

---

## 1. System Classification

Under ISO 11270 and SAE J3016:
- **Automation Level**: Level 1 (Lateral control assist) or Level 2 (when combined with Longitudinal ACC).
- **Driver Override**: The driver must always be capable of overpowering or disengaging the assist torque with intentional steering effort ($\ge 3.0\text{ to }5.0\text{ N}\cdot\text{m}$).

---

## 2. Dynamic Performance Boundaries

| Parameter | ISO 11270 Requirement | Implementation in Repository |
| :--- | :--- | :--- |
| **Max Lateral Acceleration ($a_{y}$)** | $\le 3.0\text{ m/s}^2$ (standard comfort/safety limit) | `controller.maxLateralAccel = 3.0` |
| **Max Lateral Jerk ($\dot{a}_y$)** | $\le 5.0\text{ m/s}^3$ | Handled via `lowpass_steer_filter.m` |
| **Departure Boundary Margin** | Front wheel outer edge shall not cross lane boundary by $> 0.3\text{ m}$ | Checked in `evaluate_kpis.m` |
| **Operating Velocity Range** | Typical highway: $60\text{--}130\text{ km/h}$; Off-road/Baja adapted: $15\text{--}40\text{ km/h}$ | `vehicle.nominalSpeed = 8.0 m/s` (28.8 km/h) |

---

## 3. Test Scenarios for Validation

### Test Case 1: Straight Road Drift (Initial Lateral Departure)
- **Objective**: Verify that the vehicle stably returns to lane center without excessive overshoot.
- **Conditions**: Initial lateral offset $e_0 = 0.8\text{ m}$; lateral overshoot after recovery must damp within 2 oscillations.

### Test Case 2: Constant Curvature Curve
- **Objective**: Maintain lane centering along a fixed radius curve ($R = 50\text{ m}$ or $R = 100\text{ m}$).
- **Evaluation**: Steady-state cross-track error $e_{\text{ss}} \le 0.35\text{ m}$ without tire lift-off or actuator saturation.

### Test Case 3: S-Curve / Double Lane Change
- **Objective**: Transition between opposite curvature corners while keeping lateral acceleration below $3.0\text{ m/s}^2$.

---

## 4. Citation

```bibtex
@standard{iso11270_2014,
  title     = {Intelligent transport systems — Lane keeping assistance systems (LKAS) — Performance requirements and test procedures},
  number    = {ISO 11270:2014},
  year      = {2014},
  publisher = {International Organization for Standardization}
}
```
