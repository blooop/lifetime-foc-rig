"""Characterize Genesis CPU per-step cost for the 1-DOF rig plant, so the plan
can pick a control/physics rate that stays real-time. Per-step cost is roughly
fixed regardless of timestep, so real-time factor = steps_per_sec * dt.
"""
import tempfile, time
import genesis as gs

MJCF = """
<mujoco model="shaft">
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="rotor"><joint name="shaft" type="hinge" axis="0 0 1" damping="0.0005"/>
    <geom type="box" size="0.06 0.012 0.012" mass="0.12"/></body>
  </worldbody>
</mujoco>
"""

gs.init(backend=gs.cpu, logging_level="error")
with tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False) as f:
    f.write(MJCF); path = f.name

scene = gs.Scene(show_viewer=False)
rotor = scene.add_entity(gs.morphs.MJCF(file=path))
scene.build()
dof = rotor.get_joint("shaft").dof_idx_local

# warm up (exclude any first-call JIT)
for _ in range(50):
    rotor.control_dofs_force([0.01], [dof]); scene.step()

N = 5000
t0 = time.perf_counter()
for _ in range(N):
    rotor.control_dofs_force([0.01], [dof])
    scene.step()
wall = time.perf_counter() - t0

sps = N / wall
print(f"bare stepping: {N} steps in {wall:.2f}s -> {sps:,.0f} steps/s, {wall/N*1e3:.3f} ms/step")
print("real-time factor by control rate:")
for hz in (2000, 1000, 500, 250, 100):
    print(f"  {hz:5d} Hz physics -> {sps/hz:5.2f}x real-time")
