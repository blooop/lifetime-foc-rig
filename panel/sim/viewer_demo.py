"""Standalone 3D visualization: watch the simulated carriage in the Genesis
viewer as the firmware homes it (MIN -> MAX -> center) and then cycles hall-to-
hall. The GL viewer needs the MAIN thread, so this runs the whole sim loop here
(no PyQt, no background thread) — unlike `gui-sim`, which is the interactive
control panel without a 3D window.

Run:  pixi run -e sim python -m sim.viewer_demo            (from panel/)
      pixi run -e sim viewer                               (pixi task)
"""
import time

from sim.plant import GenesisPlant, PlantConfig
from sim.soft_firmware import SoftFirmware


def main(v_cycle=60.0, hz=1000.0):
    cfg = PlantConfig()
    plant = GenesisPlant(cfg, show_viewer=True, timestep=1.0 / hz)
    fw = SoftFirmware(plant, lambda s: None)        # telemetry not needed for the view
    dt = 1.0 / hz
    us = 0
    direction = -1                                  # start toward MAX after homing
    cycling = False

    fw.handle("ME1", 0)                             # enable + auto-home
    fw.handle("EH", 0)
    print("Genesis viewer: homing (MIN -> MAX -> center), then cycling. Ctrl-C to quit.")

    wall0 = time.perf_counter()
    try:
        while True:
            if us % 1_000_000 == 0:
                fw.handle("EK", us // 1000)         # watchdog keepalive
            # once homed + settled at center, start cycling and reverse on each hall
            if fw.homed and fw.home_phase == 0:
                if not cycling:
                    fw.handle("MC1", us // 1000)
                    fw.handle(f"M{direction * v_cycle}", us // 1000)
                    cycling = True
                elif fw.esMin.triggered and direction > 0:
                    direction = -1
                    fw.handle(f"M{direction * v_cycle}", us // 1000)
                elif fw.esMax.triggered and direction < 0:
                    direction = 1
                    fw.handle(f"M{direction * v_cycle}", us // 1000)
            fw.loop(us)
            plant.step(dt)
            us += int(dt * 1e6)
            # pace to wall clock so the motion plays at real speed
            slack = wall0 + us * 1e-6 - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            elif slack < -0.5:
                wall0 = time.perf_counter() - us * 1e-6
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
