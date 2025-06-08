# Robot Constraints Simulation
Implementation based on the Bi-Level Performance-Safety Consideration paper.
Constraints are defined according to equations (7), (8), (9), (10).

## Current Implementation Status

### Completed Features ✅
- Distance Constraint (Equation 7) with improved angular region detection
- Yielding Constraint (Equation 8) for approach rate control  
- Speed Constraint (Equation 9) with distance-dependent limits
- Acceleration Constraint (Equation 10) - under review
- Multiple human support
- Real-time simulation with animation
- Constraint visualization plots

### Recent Improvements
- Modified front/side region detection using sine-based angular comparison
- Enhanced boundary condition handling (h = 0 on safety boundaries)
- Support for multiple human obstacles

## Constraint Functions
Implementation Summary:

1. DistanceConstraint - Equation (7): Maintains safe distance with different thresholds
   for front (ρ_0=3.0m) and side (ρ_1=1.0m) regions

2. YieldingConstraint - Equation (8): Enforces yielding behavior when approaching humans
   Returns approach rate (dρ/dt) in yielding zones

3. SpeedConstraint - Equation (9): Limits speed based on distance to humans
   Uses ν_M(ρ_hi) = V_M · tanh(ρ_hi) for speed limits

4. AccelConstraint - Equation (10): Constrains acceleration near humans
   Uses distance-dependent acceleration limits

All Constraints implement the condition: C_ji = ḣ_ji + α·h²_ji ≥ 0 (Equation 11)

### Equation (7) - Distance Constraint
```
h_d,i(ρ_hi, θ_hi) = {
    ρ_hi - ρ_0,  if |θ_hi| < θ_0 (front region)
    ρ_hi - ρ_1,  if |θ_hi| ≥ θ_0 (side region)
}
```
Where:
- ρ_hi = distance to human i
- θ_hi = angular deviation between robot trajectory and human position
- ρ_0 = larger safety threshold for front region
- ρ_1 = smaller safety threshold for side region
- θ_0 = critical angular range

### Equation (8) - Yielding Constraint
```
h_y,i(dρ_hi/dt, ρ_hi, θ_hi) = {
    dρ_hi/dt,  if ρ_hi ≤ ρ_0 and |θ_hi| < θ_0
    dρ_hi/dt,  if ρ_hi ≤ ρ_1 and |θ_hi| ≥ θ_0
}
```
Where:
- dρ_hi/dt = rate of change of distance (negative when approaching)

### Equation (9) - Speed Constraint
```
h_s,i(|v|, ν_M(ρ_hi)) = ν_M(ρ_hi) - |v|
```
Where:
- |v| = robot speed magnitude
- ν_M(ρ_hi) = V_M · tanh(ρ_hi) = maximum permissible speed
- V_M = robot's absolute maximum speed

### Equation (10) - Acceleration Constraint
```
h_a,i(d|v|/dt, a_max(ρ_hi)) = a_max(ρ_hi) - d|v|/dt
```
Where:
- d|v|/dt = magnitude of robot acceleration
- a_max(ρ_hi) = maximum allowable acceleration as function of distance

### Equation (11) - CBF Condition
```
C_ji = ḣ_ji + α·h²_ji ≥ 0
```
Where:
- j ∈ {d, y, s, a} (distance, yielding, speed, acceleration)
- i = human index
- α > 0 = tuning parameter controlling barrier strength

## Parameters
- ρ_0 = 3.0m (front region safety threshold)
- ρ_1 = 1.2m (side region safety threshold)  
- θ_0 = π/4 (45°, critical angular range)
- V_M = 1.0 m/s (maximum speed)
- A_M = 1.0 m/s² (maximum acceleration)

## Usage

### Run Simulation
```bash
python simulation.py
```

### Generate Constraint Plots
```bash
python constraints_plots.py
```

## Output Files
- `plots/figure0_full_region_multiple_humans.png` - All constraint functions
- `plots/figure1_minimum_constraint_multiple_humans.png` - Minimum constraint
- `plots/figure2_constraint_condition_multiple_humans.png` - CBF conditions (deprecated)
- `plots/figure3_individual_constraints_multiple_humans.png` - Individual constraints
- `plots/simulation_result_*.png` - Animation results

## Pending Improvements
- [ ] Analysis of constraint redundancy (h1, h3 vs h2)
- [ ] Review/replacement of acceleration constraint
- [ ] Implementation of new constraint functions