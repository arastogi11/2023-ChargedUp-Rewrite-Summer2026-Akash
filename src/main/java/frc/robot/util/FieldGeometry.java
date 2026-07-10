// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Pure field-geometry math, pulled out of the 2023 robot's {@code PoseEstimator.closestGrid}
 * specifically so it's unit-testable. That method had two real bugs:
 *
 * <ol>
 *   <li>{@code distanceFormula} computed {@code sqrt(dx^2 - dy^2)} instead of {@code sqrt(dx^2 +
 *       dy^2)} -- not the Pythagorean theorem, and for {@code dy > dx} produces {@code NaN}
 *       (square root of a negative number).
 *   <li>{@code closestGrid}'s min-finding loop compared each candidate distance against the
 *       literal constant {@code 4} ({@code if (vals[i] < minIndex)}, where {@code minIndex} had
 *       just been initialized to {@code 4} and was never updated to track the smallest distance
 *       found so far) instead of against the running minimum -- so it returned the index of the
 *       *last* grid within an arbitrary 4-unit distance, not the actually-closest grid.
 * </ol>
 */
public final class FieldGeometry {
  private FieldGeometry() {}

  /** Euclidean distance between two points. */
  public static double distance(double x1, double y1, double x2, double y2) {
    return Math.hypot(x2 - x1, y2 - y1);
  }

  /**
   * Returns which scoring grid (1 = dirty side, 2 = center, 3 = clean side) the given pose is
   * closest to, measured against each grid's middle column.
   */
  public static int closestGrid(Pose2d pose) {
    double[] distances = {
      distance(pose.getX(), pose.getY(), FieldConstants.grid1[1].getX(), FieldConstants.grid1[1].getY()),
      distance(pose.getX(), pose.getY(), FieldConstants.grid2[1].getX(), FieldConstants.grid2[1].getY()),
      distance(pose.getX(), pose.getY(), FieldConstants.grid3[1].getX(), FieldConstants.grid3[1].getY()),
    };

    int closestIndex = 0;
    for (int i = 1; i < distances.length; i++) {
      if (distances[i] < distances[closestIndex]) {
        closestIndex = i;
      }
    }
    return closestIndex + 1;
  }
}
