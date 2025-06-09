## Task Summary

# phase 1: Constraints for human avoidance

✅ Figure 0: Plot h_d, h_y, h_a, h_s over (x, y) ∈ [-10, 10] × [-10, 10]
    - 4 × 2 subplots: 2D + 3D views of each barrier function
    - Generated: plots/figure0_full_region.png
  
FRONT REGION (|θ_hi| < θ_0):
✅ Figure 1: Plot h_d, h_y, h_a, h_s over (x, y) ∈ [-10, 10] × [-10, 10]
    - 4 × 2 subplots: 2D + 3D views of each barrier function
    - Generated: plots/figure_1_cbf_front_region.png

SIDE REGION (|θ_hi| ≥ θ_0):
✅ Figure 2: Plot h_d, h_y, h_a, h_s over (x, y) ∈ [-10, 10] × [-10, 10]
    - 4 × 2 subplots: 2D + 3D views of each barrier function
    - Generated: plots/figure_2_cbf_side_region.png

✅ Figure 3: Plot minimum{h_d, h_y, h_a, h_s} over (x, y)
    - 2 subplots: 2D and 3D view
    - Generated: plots/figure_3_minimum_cbf.png

✅ Figure 4: Plot constraint condition C_ji = ḣ_ji + α·h²_ji ≥ 0 (Eq. 11)
    - 2 subplots: 2D and 3D view
    - Generated: plots/figure_4_constraint_condition.png

# phase 2: some improvements and modifications

✅ [1]- remove eq 11 and all CBF's --> done
✅ [2]- show a plane that when h = 0 (probably on edges of inner region) --> done
✅ [3]- remove function 4 (or add new one) removed
✅ [4]- line 32 (-y , -x) --> no need to change (instead change in constraints_plots.py)
✅ [5]- all h functions should be zero on the boundary --> done
✅ [6]- new constraint function for h_d: --> done

```python
# Check if human is in front or side region
    old version:     is_front_region = np.abs(relative_angle) < theta_0
    proposed   :     is_front_region = np.abs(np.sin(relative_angle)) < np.abs(np.sin(theta_0))
```


# phase 3: new constraints

[1]- Check if h1 and h2 is covering h3
[2]- Implementation of derivative functions for h1, h2, h3, h4
[3]- Implementation of new constraint functions

