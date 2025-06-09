## Task Summary

# phase 1: Constraints for human avoidance

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

