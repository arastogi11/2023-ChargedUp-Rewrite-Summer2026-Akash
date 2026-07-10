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
 * Hardware-agnostic interface for the drivetrain's gyroscope -- see {@link ModuleIO}'s javadoc
 * for a full explanation of the IO-interface pattern this follows. "Yaw" is the rotation you
 * intuitively think of as a robot's heading: rotation about the vertical axis, as if looking down
 * at the field from above (as opposed to pitch/roll, which are tilting forward/back or side to
 * side and aren't tracked here since a swerve drivetrain doesn't need them for pose estimation).
 */
public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = Rotation2d.kZero;
    public double yawVelocityRadPerSec = 0.0;
    // High-frequency odometry samples, same idea as ModuleIOInputs' odometry arrays -- multiple
    // gyro readings can arrive between one robot loop and the next.
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
