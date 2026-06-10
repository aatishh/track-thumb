# Track-Thumb Model (build123d)

## What This Code Does

Reconstructs a complex 3D CAD model in Python using the **build123d** library. The final output is an **STL file** ready for 3D printing or visualization.

---

## Methodology

### 1. **Sketch Definition on Inclined Planes**
- 40+ 2D sketches defined on **custom 3D planes** (not just XY):
  - Each plane has origin point, X-direction, Z-direction vectors
  - Sketches contain line segments, circular arcs, and splines
  
Example:
```python
plane = Plane(origin=Vector(-18.57, -18.47, 47.57),
              x_dir=Vector(0.705, -0.709, 0.0),
              z_dir=Vector(-0.342, -0.340, 0.876))
sketch = BuildSketch(plane)
```

### 2. **Stacking Features into 3D**
Build model using sequential CAD operations:

| Operation | Count | Purpose |
|-----------|-------|---------|
| **Extrude (ADD)** | 12 | Add material (push sketch into 3D) |
| **Extrude (CUT)** | 15 | Remove material (pockets, holes) |
| **Loft** | 6 | Blend between two sketches smoothly |
| **Sweep** | 2 | Drag profile along a curved path |
| **Revolve** | 1 | Rotate profile around an axis |

### 3. **Lofting Non-Parallel Sketches**
**Problem:** Lofting between sketches on different tilted planes causes twisting.

**Solution:**
- Resample both sketches to **64 equal points each**
- Keep sharp corners by detecting curvature changes
- Loft between point-lists (instead of raw curves)
- **Result:** Smooth, twist-free surfaces

### 4. **Boolean Operations**
- **Fuse:** Merge two solids together
- **Cut:** Remove one solid from another
- **Intersect:** Keep only overlapping volume

### 5. **Sweep with Fallback Strategy**
When dragging a profile along a curved path:
1. Try standard **pipe-shell** operation
2. If fails → Try **reversed wire direction**
3. If still fails → Use **Shell + capping algorithm**:
   - Create shell from sweep
   - Detect open boundary edges
   - Fit planes to cap the edges
   - Sew shell + caps together into solid

### 6. **Export to STL**
Final solid → binary STL mesh (~300K triangles) for 3D printing/viewing.

---

## Code Structure
Main Python Script (2634 lines)
├── Boolean helpers (fuse, cut, intersect)
├── 40+ sketches on inclined planes
├── Resampled loft profiles (64-point lists)
└── Build sequence:
├── 12× Extrude (ADD)
├── 15× Extrude (CUT)
├── 6× Loft
├── 2× Sweep
├── 1× Revolve
└── Export to STL
# track-thumb
Source volume : 12281.72 mm³
Output volume : 11382.73 mm³
Difference    : 7.32%
genrated stl image <img width="2032" height="1564" alt="image" src="https://github.com/user-attachments/assets/74ebe18e-eaf7-4d40-a3c2-e85f5a39e519" />
