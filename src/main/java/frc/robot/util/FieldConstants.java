// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * 2023 Charged Up field-relative scoring positions, carried over unchanged from the 2023 robot's
 * {@code Constants.PoseEstimation} (same game, so the geometry itself doesn't need updating --
 * only the selection logic in {@link FieldGeometry} did, see its javadoc).
 */
public final class FieldConstants {
  private FieldConstants() {}

  public static final Pose2d[] scoringPositions = new Pose2d[9];

  static {
    for (int i = 0; i < scoringPositions.length; i++) {
      scoringPositions[i] =
          new Pose2d(1.95, Units.inchesToMeters(20.19 + 22.0 * i), new Rotation2d(Math.PI));
    }
  }

  /** Dirty-side grid (nearest the loading zone / human player side edge). */
  public static final Pose2d[] grid1 = {scoringPositions[0], scoringPositions[1], scoringPositions[2]};

  /** Center grid. */
  public static final Pose2d[] grid2 = {scoringPositions[3], scoringPositions[4], scoringPositions[5]};

  /** Clean-side grid. */
  public static final Pose2d[] grid3 = {scoringPositions[6], scoringPositions[7], scoringPositions[8]};

  public static final Pose2d hpStation = new Pose2d(15.61, 7.34, new Rotation2d());
}
