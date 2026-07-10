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

  // The 9 scoring node positions across all three grids, evenly spaced 22 inches apart along the
  // field's Y axis starting from a fixed offset. Rotation2d(Math.PI) means each position faces
  // 180 degrees -- i.e. the robot should be backed up to the grid, facing away from the field,
  // when scoring (a common orientation for 2023's front-facing scoring mechanisms).
  public static final Pose2d[] scoringPositions = new Pose2d[9];

  // A static initializer block: runs exactly once, the first time this class is loaded, before
  // anything else touches scoringPositions -- used here instead of a giant list literal because
  // the 9 positions follow a simple repeating pattern (each 22 inches over from the last) that's
  // clearer expressed as a loop than typed out 9 times by hand.
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
