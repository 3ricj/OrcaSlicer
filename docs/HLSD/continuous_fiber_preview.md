# Continuous-fiber preview

`GCodeProcessor` treats a G1 with positive U and XY motion inside an
`M1001`..`M1002` window as a fiber deposit. The time estimator still classifies
that move as travel (U/V are not E), so print-time and statistic totals stay
unchanged. The stored vertex is then tagged `erFiber` / `EGCodeExtrusionRole::Fiber`
with the configured fiber bead width so the 3D viewer draws a tube.

The Feature Type list exposes a "Continuous fiber" row. Fiber is included in
the extrusion bounding box used to fit the preview. libvgcode's
`extrusion_roles_visibility` default includes Fiber, so the role is on unless
the user hides it.

The cooling buffer leaves U/V lines untouched when `fs_fiber_enabled` is on:
those axes are invisible to its length parser, and rewriting F on them would
corrupt the lifecycle.
