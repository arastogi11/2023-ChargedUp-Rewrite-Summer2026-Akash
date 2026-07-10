// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import org.junit.jupiter.api.Test;

/**
 * Tests the reject-pose "throwout" logic and standard-deviation formula independent of any
 * NetworkTables/hardware -- see {@link VisionConstants} for what each threshold does.
 */
class VisionConstantsTest {
  private static final Pose2d midField = new Pose2d(8.0, 4.0, new Rotation2d());

  private static PoseObservation observation(int tagCount, double ambiguity, double avgTagDist, Pose2d pose) {
    return new PoseObservation(0.0, pose, ambiguity, tagCount, avgTagDist);
  }

  @Test
  void rejectsZeroTagObservations() {
    assertTrue(VisionConstants.shouldRejectPose(observation(0, 0.0, 1.0, midField)));
  }

  @Test
  void rejectsHighAmbiguitySingleTagObservations() {
    assertTrue(
        VisionConstants.shouldRejectPose(
            observation(1, VisionConstants.maxAmbiguity + 0.01, 1.0, midField)));
  }

  @Test
  void acceptsLowAmbiguitySingleTagObservations() {
    assertFalse(
        VisionConstants.shouldRejectPose(
            observation(1, VisionConstants.maxAmbiguity - 0.01, 1.0, midField)));
  }

  @Test
  void ambiguityIsIgnoredForMultiTagObservations() {
    // Multi-tag solves are well-constrained regardless of the (unused) ambiguity value -- only
    // the single-tag case gates on it, matching the 2024 robot's approach.
    assertFalse(VisionConstants.shouldRejectPose(observation(2, 1.0, 1.0, midField)));
  }

  @Test
  void rejectsObservationsBeyondMaxTagDistance() {
    assertTrue(
        VisionConstants.shouldRejectPose(
            observation(2, 0.0, VisionConstants.maxTagDistanceMeters + 0.01, midField)));
  }

  @Test
  void rejectsPosesOutsideFieldBounds() {
    Pose2d beyondFieldLength =
        new Pose2d(VisionConstants.aprilTagLayout.getFieldLength() + 1.0, 4.0, new Rotation2d());
    assertTrue(VisionConstants.shouldRejectPose(observation(2, 0.0, 1.0, beyondFieldLength)));

    Pose2d negativeY = new Pose2d(8.0, -1.0, new Rotation2d());
    assertTrue(VisionConstants.shouldRejectPose(observation(2, 0.0, 1.0, negativeY)));
  }

  @Test
  void acceptsAWellFormedObservation() {
    assertFalse(VisionConstants.shouldRejectPose(observation(2, 0.0, 2.0, midField)));
  }

  @Test
  void xyStdDevScalesWithDistanceAndShrinksWithMoreTags() {
    var oneTagFar = observation(1, 0.0, 4.0, midField);
    var twoTagsFar = observation(2, 0.0, 4.0, midField);
    var oneTagNear = observation(1, 0.0, 1.0, midField);

    // More tags at the same distance -> lower stddev (more trust).
    assertTrue(VisionConstants.xyStdDev(twoTagsFar) < VisionConstants.xyStdDev(oneTagFar));
    // Farther away at the same tag count -> higher stddev (less trust).
    assertTrue(VisionConstants.xyStdDev(oneTagFar) > VisionConstants.xyStdDev(oneTagNear));

    assertEquals(
        VisionConstants.xyStdDevCoefficient * 4.0 / Math.sqrt(2), VisionConstants.xyStdDev(twoTagsFar), 1e-9);
  }
}
