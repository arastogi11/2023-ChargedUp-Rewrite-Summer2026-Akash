// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class FieldGeometryTest {
  @Test
  void distanceIsPythagorean() {
    // 3-4-5 triangle -- the 2023 robot's original `sqrt(dx^2 - dy^2)` formula would return
    // sqrt(9 - 16) = sqrt(-7), i.e. NaN, for this exact case.
    assertEquals(5.0, FieldGeometry.distance(0, 0, 3, 4), 1e-9);
  }

  @Test
  void distanceIsZeroForSamePoint() {
    assertEquals(0.0, FieldGeometry.distance(2.5, 1.5, 2.5, 1.5), 1e-9);
  }

  @Test
  void closestGridPicksTheNearestGridsMiddleColumn() {
    // Pose directly on top of grid1's middle column should resolve to grid 1.
    Pose2d nearGrid1 = new Pose2d(FieldConstants.grid1[1].getX(), FieldConstants.grid1[1].getY(), new Rotation2d());
    assertEquals(1, FieldGeometry.closestGrid(nearGrid1));

    Pose2d nearGrid2 = new Pose2d(FieldConstants.grid2[1].getX(), FieldConstants.grid2[1].getY(), new Rotation2d());
    assertEquals(2, FieldGeometry.closestGrid(nearGrid2));

    Pose2d nearGrid3 = new Pose2d(FieldConstants.grid3[1].getX(), FieldConstants.grid3[1].getY(), new Rotation2d());
    assertEquals(3, FieldGeometry.closestGrid(nearGrid3));
  }

  @Test
  void closestGridTracksTrueMinimumAcrossAllThreeCandidates() {
    // Regression case for the 2023 robot's second bug: its min-finding loop compared each
    // distance against the constant 4 (`if (vals[i] < minIndex)` where minIndex was initialized
    // to 4 and never updated) instead of the running minimum, so whichever candidate it checked
    // *last* that happened to be under 4 would win -- not the actually-closest one. Here grid1 is
    // closest (pose sits right on top of it), but grid2 and grid3 are also within the old
    // threshold's range and are checked after it, which would have overwritten the correct
    // answer under the old logic. The fixed implementation correctly returns grid 1.
    Pose2d pose =
        new Pose2d(FieldConstants.grid1[1].getX() + 0.1, FieldConstants.grid1[1].getY(), new Rotation2d());
    assertEquals(1, FieldGeometry.closestGrid(pose));
  }
}
