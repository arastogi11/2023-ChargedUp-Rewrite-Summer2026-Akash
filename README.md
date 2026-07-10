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

## Dashboard: Elastic only

This project targets [Elastic](https://github.com/Gold872/elastic-dashboard) exclusively — no
SmartDashboard or Shuffleboard usage anywhere in the codebase. All telemetry goes through
AdvantageKit's `Logger` (NT4 + on-robot logging) or `LoggedNetworkBoolean`/`LoggedNetworkNumber`
for dashboard-writable values, both of which publish plain NT4 topics that Elastic can bind
directly — no legacy dashboard Java APIs involved.

The physical 2023 robot currently has its **Limelights and LEDs removed**. Rather than relying on
those subsystems' existing NT-based null/zero-tag safety (which already prevents crashes with no
hardware present), both are **dashboard-toggleable and default OFF**:

- `/DriverDashboard/VisionEnabled` (`Vision.java`) — while off, cameras aren't polled and their
  "disconnected" alerts are suppressed, so the dashboard doesn't nag about hardware that's
  intentionally absent.
- `/DriverDashboard/LedsEnabled` (`Leds.java`) — while off, every color command is a no-op and the
  Blinkin PWM output is forced to "black" every loop.

Add a toggle-switch widget bound to each of those two NT4 boolean topics in your Elastic layout
(drag them in from the NT4 topic tree — Elastic doesn't require any special widget config or
project-side layout file for this). Flip them on once the hardware is reinstalled.

## roboRIO 1 memory

The real 2023 robot runs on a **roboRIO 1** (256MB total RAM — much less than a roboRIO 2), which
can run low on memory if logging volume gets too high. Two things address this:

- **`build.gradle`'s JVM args no longer force a fixed 100MB heap with `-XX:+AlwaysPreTouch`.** That
  block was copied forward from `BobcatRobotics/177-Rebuilt`'s `comp/2026` template (a roboRIO 2
  project) during the initial rewrite, and its own comment said "should only be enabled on the RIO
  2" — `AlwaysPreTouch` force-commits the whole heap immediately at boot, which on a 256MB system
  leaves very little room for the OS, NetworkTables, and camera streaming. Heap sizing is now left
  unset, so the JVM sizes itself against whatever RAM is actually available.
- **`/DriverDashboard/LoggingEnabled`** (`frc.robot.util.LoggingControl`, default **on**) — a
  dashboard kill switch for detailed AdvantageKit logging. While off, every subsystem still reads
  its sensors and runs its control logic completely normally (nothing about robot *behavior*
  changes), but the expensive part — serializing the full sensor/output struct to the USB log file
  and the live NT4 publisher every loop, for every subsystem — is skipped. Bind it to a
  toggle-switch widget in Elastic like the other two toggles above; flip it off mid-match if
  logging ever seems to be contributing to instability. (Only takes effect on the real robot —
  sim/replay runs on a desktop with vastly more RAM and always logs in full, since replay
  specifically depends on every input being recorded to work at all.)

Vision's per-loop allocations were also tightened (`VisionIOLimelight`'s tag-ID collection no
longer allocates a `HashSet` + boxes every ID; `Vision`'s accepted/rejected pose bookkeeping uses
`ArrayList` instead of `LinkedList` and is skipped entirely while the logging toggle above is off,
since those lists exist purely to feed the log).

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
