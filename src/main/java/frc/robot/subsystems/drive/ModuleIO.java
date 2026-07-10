// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware-agnostic interface for a single swerve module's motors/sensors -- the "IO" (input/
 * output) half of the IO-interface pattern used throughout every subsystem in this codebase, so
 * it's worth explaining fully once here.
 *
 * <p>The pattern splits each subsystem into two pieces:
 *
 * <ul>
 *   <li>An {@code XyzIO} interface (this file) that declares every sensor value the subsystem
 *       cares about (as an {@code @AutoLog}-annotated {@code XyzIOInputs} class) and every command
 *       it can be given (as methods), with every method defaulting to a no-op.
 *   <li>One or more concrete implementations of that interface -- here, {@link ModuleIOTalonFX}
 *       (talks to real Phoenix6 hardware) and {@link ModuleIOSim} (runs a physics simulation
 *       instead). {@code Module.java} (the actual subsystem-facing class) is written entirely
 *       against the {@code ModuleIO} interface and has no idea which one it's actually using.
 * </ul>
 *
 * <p>This buys three things: (1) the exact same command-based robot code can run against real
 * hardware or a simulator just by swapping which implementation gets constructed in {@code
 * RobotContainer}; (2) {@code @AutoLog} auto-generates a matching {@code
 * ModuleIOInputsAutoLogged} class (via an annotation processor at build time -- you won't find it
 * in this source tree, but Module.java constructs and uses one) that logs and replays every field
 * automatically; and (3) in REPLAY mode, a plain {@code new ModuleIO() {}} (using only the no-op
 * defaults below) is a legitimate, harmless "do nothing" implementation, since replayed inputs
 * come from the log file instead of any of these methods actually being called meaningfully.
 */
public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public boolean turnEncoderConnected = false;
    // The steer axis's raw absolute position straight from the CANcoder (survives power cycles),
    // as opposed to turnPosition below (the TalonFX's own tracked position, which is what's
    // actually used for closed-loop control -- see ModuleIOTalonFX for how the two get tied
    // together via remote/fused CANcoder feedback).
    public Rotation2d turnAbsolutePosition = Rotation2d.kZero;
    public Rotation2d turnPosition = Rotation2d.kZero;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;

    // High-frequency odometry samples: potentially several per robot loop (see Drive's
    // ODOMETRY_FREQUENCY), collected on PhoenixOdometryThread's own schedule rather than the
    // normal 50Hz loop, for smoother/more accurate position tracking.
    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Rotation2d rotation) {}
}
