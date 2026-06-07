"""Phase-0 smoke test: prove Genesis runs on CPU and can model the FOC rig's
plant contract — one torque-controlled revolute DOF (the motor shaft), stepped
deterministically, reading back angle/velocity.

This is the gate for GENESIS_SIM_PLAN.md. Run:  pixi run -e sim python panel/sim/smoke_test.py
"""
import os
import sys
import tempfile
import time

# A single hinge ("shaft") rotor, no gravity — the minimal analogue of the
# motor shaft DOF the SoftFirmware would drive with a torque.
MJCF = """
<mujoco model="foc_rig_shaft">
  <option gravity="0 0 0" timestep="0.001"/>
  <worldbody>
    <body name="rotor" pos="0 0 0">
      <joint name="shaft" type="hinge" axis="0 0 1" damping="0.0005"/>
      <geom type="box" size="0.06 0.012 0.012" mass="0.12"/>
    </body>
  </worldbody>
</mujoco>
"""


def main():
    print("== importing genesis ==")
    import genesis as gs
    print(f"genesis {getattr(gs, '__version__', '?')}")

    print("== gs.init(backend=cpu) ==")
    gs.init(backend=gs.cpu, logging_level="warning")

    with tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False) as f:
        f.write(MJCF)
        mjcf_path = f.name

    print("== build scene (1 hinge DOF) ==")
    scene = gs.Scene(show_viewer=False)
    rotor = scene.add_entity(gs.morphs.MJCF(file=mjcf_path))
    scene.build()

    dof = rotor.get_joint("shaft").dof_idx_local
    print(f"shaft dof_idx_local = {dof}")

    # Apply a constant torque and confirm the shaft accelerates (torque-in ->
    # state-out, the plant contract), and measure the real-time factor.
    tau = 0.02  # N*m
    n = 2000    # 2 s at 1 ms
    t0 = time.perf_counter()
    vels = []
    for i in range(n):
        rotor.control_dofs_force([tau], [dof])
        scene.step()
        if i % 400 == 0 or i == n - 1:
            pos = float(rotor.get_dofs_position([dof])[0])
            vel = float(rotor.get_dofs_velocity([dof])[0])
            vels.append(vel)
            print(f"  step {i:4d}: angle={pos:8.4f} rad  vel={vel:8.4f} rad/s")
    wall = time.perf_counter() - t0
    sim_t = n * 0.001
    print(f"== stepped {n} x 1ms = {sim_t:.1f}s sim in {wall:.2f}s wall "
          f"-> {sim_t / wall:.1f}x real-time ==")

    ok = vels[-1] > vels[0]  # constant torque, ~no friction -> speed must rise
    print("RESULT:", "PASS — torque accelerates the shaft, CPU stepping works"
          if ok else "FAIL — shaft did not accelerate under torque")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        import traceback
        traceback.print_exc()
        print("RESULT: FAIL —", type(e).__name__, str(e))
        sys.exit(2)
