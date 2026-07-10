# 2023-ChargedUp-Rewrite-Summer2026-Akash

A personal, from-scratch rewrite/learning project. This is **not an official BobcatRobotics (FRC
Team 177) repo** — it's Akash's own synthesis of the team's 2023 "Charged Up" competition robot
code, rebuilt to run on 2026-era WPILib and vendor libraries.

It combines:
- **Functionality/architecture** from the team's 2023 Charged Up robot (swerve drivetrain, arm,
  elevator, wrist, intake, LEDs) — `BobcatRobotics/2023-ChargedUp-177-Bot-2` @ tag `DCMP`
- **Code formatting and logging conventions** (the AdvantageKit IO-interface pattern) from the
  2024 Crescendo season — `BobcatRobotics/2024-Crescendo` @ tag `BATB`
- **Vision pose-estimation filtering ("throwout") logic** validated during the 2024 and 2025
  seasons — 2024-Crescendo's fiducial-ID allowlist and 2025 Reefscape's continuous
  distance/tag-count standard-deviation formula (`BobcatRobotics/2025-Reefscape` @ branch
  `cki_fixes`)
- **Current 2026 WPILib/vendor library APIs**, verified against each library's own current docs
  rather than copied forward from old usage — using `BobcatRobotics/177-Rebuilt`'s `comp/2026` as
  a reference for the current AdvantageKit swerve/vision template, and a freshly vendored copy of
  Limelight's own `LimelightHelpers.java` (v1.14) rather than any team's older copy

No BobcatLib dependency and no cross-repo monorepo tricks — this is a normal, standalone,
self-contained single-module Gradle project.

## Architecture

Standard AdvantageKit IO-interface pattern throughout: each subsystem has an `XyzIO` interface
(`@AutoLog` inputs, no-op default methods), an `XyzIOTalonFX`/`XyzIOSpark` real-hardware
implementation, and an `XyzIOSim` physics-sim implementation, selected in `RobotContainer` based on
`Constants.currentMode` (REAL/SIM/REPLAY).

| Subsystem | Package | Notes |
|---|---|---|
| Drivetrain | `subsystems.drive` | Phoenix6 swerve, adapted from the current AdvantageKit TalonFX swerve template |
| Elevator / Arm / Wrist | `subsystems.elevator/arm/wrist` | Single Motion Magic closed loop per motor (fixes a 2023 bug where Motion Magic gains were configured but never used — see below) |
| Intake | `subsystems.intake` | Roller with stator-current game-piece detection |
| LEDs | `subsystems.leds` | REV Blinkin over PWM |
| Vision | `subsystems.vision` | Dual Limelight, MegaTag2, restored 2025-validated throwout logic |
| Commands | `commands` | Flat factory-style (`DriveCommands`, `ScoringCommands`), replacing the 2023 robot's deeply nested subclassed-`Command` folders |

## Known bugs fixed (not carried forward)

- **Arm/Wrist duplicated control loop**: the 2023 robot configured Motion Magic gains on the
  TalonFX but drove the motor in `PercentOutput` with a separate hand-rolled WPILib
  `PIDController` computing the setpoint error every loop instead. This rewrite uses a single
  Motion Magic Voltage request running entirely onboard the motor controller.
- **`distanceFormula` bug**: `PoseEstimator.distanceFormula` computed `sqrt(dx² − dy²)` instead of
  `sqrt(dx² + dy²)` — not the Pythagorean theorem, and `NaN` whenever `dy > dx`. Fixed in
  `frc.robot.util.FieldGeometry`, with a regression test.
- **`closestGrid` min-finding bug**: compared each candidate distance against the literal constant
  `4` instead of the running minimum, so it could return the wrong grid. Also fixed in
  `FieldGeometry`, also tested.
- **LEDs `turnOff()`**: cut power to a PDH switchable channel instead of using the Blinkin's own
  documented "black" PWM value (which every other color-setting method already sent correctly).
  Now just uses the PWM value like every other color, with no PDH involvement.
- Renamed the CAN bus off its joke name (`"CANt_open_file"`).

## Known placeholders — verify on hardware before relying on these

Ported from 2023 source that didn't record enough information to convert exactly, or that
genuinely needs re-measurement/re-tuning on the physical 2026 robot:

- **Elevator setpoints** (`Constants.ElevatorConstants`): 2023 used raw, gear-ratio-unknown
  Phoenix v5 sensor ticks. The current values preserve the *relative spacing* between the original
  four setpoints on a placeholder rotation scale — re-measure the real positions on hardware.
- **Arm/Wrist `rotorToSensorRatio`** (gearbox ratio between the TalonFX rotor and the remote
  CANcoder): never stated in the 2023 source. Set from CAD/gearbox spec before trusting closed-loop
  behavior.
- **Drivetrain steer gains, slip current, coupling ratio** (`generated/TunerConstants`): the 2023
  robot's Phoenix v5 native-unit gains don't translate to Phoenix6's units. Drive kS/kV/kA *were*
  converted from the 2023 robot's actual SysId results; steer gains are placeholders. SysId/wheel
  radius characterization routines are already wired into the auto chooser for re-tuning.
- **Vision camera mount offsets** (`VisionConstants.cameras`): the 2023 robot read Limelight
  botpose directly without a client-side camera-to-robot transform, so this data was never
  captured. Measure and set before relying on MegaTag2 accuracy.
- **Elevator/Arm/Wrist mechanism geometry in sim** (drum radius, arm length/mass, etc.): reasonable
  stand-ins, not the real 2023 CAD.
- **PathPlanner paths/autos**: intentionally not fabricated — see
  `src/main/deploy/pathplanner/README.md`. Author these in the PathPlanner GUI app once the
  drivetrain is on real hardware.

## Building / testing

Standard GradleRIO project — requires the WPILib 2026 toolchain.

```
./gradlew build       # compile + Checkstyle + unit tests
./gradlew test         # unit tests only
```

CI (`.github/workflows/build.yml`) runs the same build on every push/PR.
