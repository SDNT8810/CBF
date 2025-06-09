# Robot Constraints Simulation
Implementation based on the Bi-Level Performance-Safety Consideration paper.
Constraints are defined according to equations (7), (8), (9), (10).

### Recent Improvements
- Modified front/side region detection using sine-based angular comparison
- Enhanced boundary condition handling (h = 0 on safety boundaries)
- Support for multiple human obstacles

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



# Constraint Functions and Detailed Derivatives for All Constraints

## 1. Distance Constraint – Equation (7)

### Constraint Function
```math
h_{d,i}(\rho_{hi}, \theta_{hi}) = \begin{cases}
\rho_{hi} - \rho_0, & \text{if } |\theta_{hi}| < \theta_0 \text{ (front region)} \\
\rho_{hi} - \rho_1, & \text{if } |\theta_{hi}| \geq \theta_0 \text{ (side region)}
\end{cases}
```

Where:
- $\rho_{hi} = |\mathbf{p}_r - \mathbf{p}_{hi}|$ = distance to human i
- $\theta_{hi}$ = angular deviation between robot trajectory and human position
- $\rho_0$ = larger safety threshold for front region (3.0m)
- $\rho_1$ = smaller safety threshold for side region (1.2m)
- $\theta_0 = \pi/4$ = critical angular range (45°)

### Time Derivative
```math
\dot{h}_{d,i} = \frac{d\rho_{hi}}{dt} = \frac{(\mathbf{p}_r - \mathbf{p}_{hi})^\top (\mathbf{v}_r - \mathbf{v}_{hi})}{\rho_{hi}}
```

### Spatial Derivatives
```math
\frac{\partial h_{d,i}}{\partial x_r} = \frac{x_r - x_{hi}}{\rho_{hi}}, \quad
\frac{\partial h_{d,i}}{\partial y_r} = \frac{y_r - y_{hi}}{\rho_{hi}}, \quad
\frac{\partial h_{d,i}}{\partial \theta_r} = 0
```

---

## 2. Yielding Constraint – Equation (8)

### Constraint Function
```math
h_{y,i}(\rho_{hi}, \theta_{hi}) = \begin{cases}
\frac{d\rho_{hi}}{dt}, & \text{if } \rho_{hi} \leq \rho_0 \text{ and } |\theta_{hi}| < \theta_0 \\
\frac{d\rho_{hi}}{dt}, & \text{if } \rho_{hi} \leq \rho_1 \text{ and } |\theta_{hi}| \geq \theta_0
\end{cases}
```

Where:
- $\frac{d\rho_{hi}}{dt} = \frac{(\mathbf{p}_r - \mathbf{p}_{hi})^\top (\mathbf{v}_r - \mathbf{v}_{hi})}{\rho_{hi}}$ = rate of change of distance
- Negative when approaching, positive when moving away
- Active only in yielding zones (close proximity regions)

### Time Derivative
```math
\dot{h}_{y,i} = \frac{d^2\rho_{hi}}{dt^2} = \frac{1}{\rho_{hi}} \left[ 
\|\mathbf{v}_r - \mathbf{v}_{hi}\|^2 + (\mathbf{p}_r - \mathbf{p}_{hi})^\top (\mathbf{a}_r - \mathbf{a}_{hi}) - \frac{[(\mathbf{p}_r - \mathbf{p}_{hi})^\top (\mathbf{v}_r - \mathbf{v}_{hi})]^2}{\rho_{hi}^2} 
\right]
```

### Spatial Derivatives
```math
\frac{\partial h_{y,i}}{\partial x_r} = \frac{\partial h_{d,i}}{\partial x_r}, \quad
\frac{\partial h_{y,i}}{\partial y_r} = \frac{\partial h_{d,i}}{\partial y_r}, \quad
\frac{\partial h_{y,i}}{\partial \theta_r} = 0
```

---

## 3. Speed Constraint – Equation (9)

### Constraint Function
```math
h_{s,i}(|\mathbf{v}_r|, \rho_{hi}) = \nu_M(\rho_{hi}) - |\mathbf{v}_r|
```

Where:
- $|\mathbf{v}_r|$ = robot speed magnitude
- $\nu_M(\rho_{hi}) = V_M \cdot \tanh(\rho_{hi})$ = maximum permissible speed
- $V_M = 1.0$ m/s = robot's absolute maximum speed
- Speed limit decreases as robot approaches humans

### Time Derivative
```math
\dot{h}_{s,i} = V_M \cdot \text{sech}^2(\rho_{hi}) \cdot \frac{(\mathbf{p}_r - \mathbf{p}_{hi})^\top (\mathbf{v}_r - \mathbf{v}_{hi})}{\rho_{hi}} - \frac{\mathbf{v}_r^\top \mathbf{a}_r}{\|\mathbf{v}_r\|}
```

### Spatial Derivatives
```math
\frac{\partial h_{s,i}}{\partial x_r} = V_M \cdot \text{sech}^2(\rho_{hi}) \cdot \frac{x_r - x_{hi}}{\rho_{hi}}, \quad
\frac{\partial h_{s,i}}{\partial y_r} = V_M \cdot \text{sech}^2(\rho_{hi}) \cdot \frac{y_r - y_{hi}}{\rho_{hi}}, \quad
\frac{\partial h_{s,i}}{\partial \theta_r} = 0
```

---

## 4. Acceleration Constraint – Equation (10)

### Constraint Function
```math
h_{a,i}(|\mathbf{a}_r|, \rho_{hi}) = a_{\text{max}}(\rho_{hi}) - |\mathbf{a}_r|
```

Where:
- $|\mathbf{a}_r| = \frac{d|\mathbf{v}_r|}{dt}$ = magnitude of robot acceleration
- $a_{\text{max}}(\rho_{hi}) = A_M \cdot \tanh(\rho_{hi})$ = maximum allowable acceleration
- $A_M = 1.0$ m/s² = robot's absolute maximum acceleration
- Acceleration limit decreases as robot approaches humans

### Time Derivative
```math
\dot{h}_{a,i} = A_M \cdot \text{sech}^2(\rho_{hi}) \cdot \frac{(\mathbf{p}_r - \mathbf{p}_{hi})^\top (\mathbf{v}_r - \mathbf{v}_{hi})}{\rho_{hi}} - \frac{\mathbf{a}_r^\top \dot{\mathbf{a}}_r}{\|\mathbf{a}_r\|}
```

### Spatial Derivatives
```math
\frac{\partial h_{a,i}}{\partial x_r} = A_M \cdot \text{sech}^2(\rho_{hi}) \cdot \frac{x_r - x_{hi}}{\rho_{hi}}, \quad
\frac{\partial h_{a,i}}{\partial y_r} = A_M \cdot \text{sech}^2(\rho_{hi}) \cdot \frac{y_r - y_{hi}}{\rho_{hi}}, \quad
\frac{\partial h_{a,i}}{\partial \theta_r} = 0
```

---

## Summary Table

| Constraint | Complete Constraint Function | Time Derivative |
|------------|----------------------------|-----------------|
| **Distance** | $h_{d,i}(\rho_{hi}, \theta_{hi}) = \begin{cases} \rho_{hi} - \rho_0, & \text{if } \|\theta_{hi}\| < \theta_0 \\ \rho_{hi} - \rho_1, & \text{if } \|\theta_{hi}\| \geq \theta_0 \end{cases}$ | $\dot{h}_{d,i} = \frac{(\mathbf{p}_r - \mathbf{p}_{hi})^\top (\mathbf{v}_r - \mathbf{v}_{hi})}{\rho_{hi}}$ |
| **Yielding** | $h_{y,i}(\rho_{hi}, \theta_{hi}) = \begin{cases} \frac{d\rho_{hi}}{dt}, & \text{if } \rho_{hi} \leq \rho_0 \text{ and } \|\theta_{hi}\| < \theta_0 \\ \frac{d\rho_{hi}}{dt}, & \text{if } \rho_{hi} \leq \rho_1 \text{ and } \|\theta_{hi}\| \geq \theta_0 \end{cases}$ | $\dot{h}_{y,i} = \frac{1}{\rho_{hi}} \left[ \|\mathbf{v}_r - \mathbf{v}_{hi}\|^2 + (\mathbf{p}_r - \mathbf{p}_{hi})^\top (\mathbf{a}_r - \mathbf{a}_{hi}) - \frac{[(\mathbf{p}_r - \mathbf{p}_{hi})^\top (\mathbf{v}_r - \mathbf{v}_{hi})]^2}{\rho_{hi}^2} \right]$ |
| **Speed** | $h_{s,i}(\|\mathbf{v}_r\|, \rho_{hi}) = V_M \cdot \tanh(\rho_{hi}) - \|\mathbf{v}_r\|$ | $\dot{h}_{s,i} = V_M \cdot \text{sech}^2(\rho_{hi}) \cdot \frac{(\mathbf{p}_r - \mathbf{p}_{hi})^\top (\mathbf{v}_r - \mathbf{v}_{hi})}{\rho_{hi}} - \frac{\mathbf{v}_r^\top \mathbf{a}_r}{\|\mathbf{v}_r\|}$ |
| **Acceleration** | $h_{a,i}(\|\mathbf{a}_r\|, \rho_{hi}) = A_M \cdot \tanh(\rho_{hi}) - \|\mathbf{a}_r\|$ | $\dot{h}_{a,i} = A_M \cdot \text{sech}^2(\rho_{hi}) \cdot \frac{(\mathbf{p}_r - \mathbf{p}_{hi})^\top (\mathbf{v}_r - \mathbf{v}_{hi})}{\rho_{hi}} - \frac{\mathbf{a}_r^\top \dot{\mathbf{a}}_r}{\|\mathbf{a}_r\|}$ |
