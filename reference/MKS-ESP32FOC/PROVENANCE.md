# MKS-ESP32FOC — vendored reference copy

Read-only reference for the **MKS ESP32 FOC V2.0** board, vendored into this repo
so the schematics and makerbase example sketches are always at hand.

- **Upstream:** https://github.com/makerbase-motor/MKS-ESP32FOC.git
- **Branch:** `MKS-ESP32-FOC-V2.0`
- **Commit:** `729e303`

## What's here
- `Hardware/` — board schematics (V1.0, V2.0, Mega) as PDF. `MKS ESP32 FOC V2.0_SCH.pdf`
  is the one that matches our board.
- `Test Code/` — makerbase's stock SimpleFOC example sketches. Notably
  `14_dual_inline_current_sense_test/` documents the inline current-sense pins:
  **Motor 0 = ADC 39 (phase A), 36 (phase B); Motor 1 = 35, 34**, 10 mΩ shunt, gain 50
  (INA240A2). This is the source for our `InlineCurrentSense(0.01f, 50.0f, 39, 36)`.
- `image/`, `README.md` — board overview.

## Intentionally omitted
- `Test Code/13_SimpleFOCStudio_M0_bluetooth/simpleFOCStudio_dist.7z.00{1,2,3}` —
  a ~69 MB Windows build of the SimpleFOCStudio BLE GUI. Not relevant to this
  Linux/Python rig and far too large to commit to git history. Grab it from upstream
  if ever needed. The `.ino` for that example is kept.
- A few 1-byte placeholder stubs that were empty in upstream (`User Manual/User Manual`,
  `image/image`, `Test Code/8_hallsensor_test/8_hallsensor_test`).
