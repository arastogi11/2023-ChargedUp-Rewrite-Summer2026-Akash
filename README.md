# 2023-ChargedUp-Rewrite-Summer2026-Akash

A personal, from-scratch rewrite/learning project. This is **not an official BobcatRobotics (FRC Team 177) repo** — it's Akash's own synthesis of the team's 2023 "Charged Up" competition robot code, rebuilt to run on 2026-era WPILib and vendor libraries.

It combines:
- **Functionality/architecture** from the team's 2023 Charged Up robot (swerve drivetrain, arm, elevator, wrist, intake, LEDs)
- **Code formatting and logging conventions** (the AdvantageKit IO-interface pattern) from the team's 2024 Crescendo season
- **Vision pose-estimation filtering logic** validated during the 2024 and 2025 seasons
- **Current 2026 WPILib/vendor library APIs**, understood from each library's own documentation rather than copied forward from old usage

Progress and design decisions are tracked in commits, phase by phase (drivetrain, mechanisms, LEDs, vision, controls, autonomous).

## Building

Standard GradleRIO project — requires the WPILib 2026 toolchain.

```
./gradlew build
```
