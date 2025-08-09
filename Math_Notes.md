# Math Notes – Quick Reference

## 1. Calculus (Motion & Control)
- Velocity: v(t) = dx/dt  
- Acceleration: a(t) = dv/dt = d²x/dt²  
- From acceleration to position:  
  v(t) = a*t + v0  
  x(t) = 0.5*a*t² + v0*t + x0  

- Common derivatives:  
  d/dt [tⁿ] = n*t^(n-1)  
  d/dt [sin t] = cos t  
  d/dt [cos t] = -sin t  

- Common integrals:  
  ∫ tⁿ dt = t^(n+1)/(n+1) + C  
  ∫ sin t dt = -cos t + C  
  ∫ cos t dt = sin t + C  

---

## 2. Linear Algebra (Transforms & Projections)
- Dot product: a·b = |a||b|cos(θ)  
- Matrix-vector multiplication transforms coordinates: y = R*x  
- 2D rotation matrix:  
  [cosθ  -sinθ]  
  [sinθ   cosθ]  

Example: 90° rotation of (1,0) → (0,1)  

---

## 3. Coordinate Frame Changes
- BEV projection: p_BEV = H * p_cam  
  (H is homography matrix from perspective transform)

---

## 4. Quick Python Check
```python
import numpy as np
theta = np.pi / 2
R = np.array([[np.cos(theta), -np.sin(theta)],
              [np.sin(theta),  np.cos(theta)]])
print(R @ np.array([1, 0]))  # -> [0. 1.]
