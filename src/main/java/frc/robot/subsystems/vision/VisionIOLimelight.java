// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Real hardware IO for a single Limelight, using MegaTag2 (gyro-seeded) pose estimation via the
 * official LimelightHelpers API rather than parsing raw NetworkTables arrays by hand.
 */
public class VisionIOLimelight implements VisionIO {
  private final String name;
  private final Supplier<Rotation2d> gyroYawSupplier;

  public VisionIOLimelight(String name, Supplier<Rotation2d> gyroYawSupplier) {
    this.name = name;
    this.gyroYawSupplier = gyroYawSupplier;

    LimelightHelpers.SetFiducialIDFiltersOverride(name, VisionConstants.validFiducialIds);
    var camera = camera();
    LimelightHelpers.setCameraPose_RobotSpace(
        name, camera.forward(), camera.side(), camera.up(), camera.roll(), camera.pitch(), camera.yaw());
  }

  private VisionConstants.CameraConfig camera() {
    for (var c : VisionConstants.cameras) {
      if (c.name().equals(name)) {
        return c;
      }
    }
    throw new IllegalArgumentException("No VisionConstants.CameraConfig for camera: " + name);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // MegaTag2 requires the current gyro yaw to be pushed to the Limelight every loop.
    LimelightHelpers.SetRobotOrientation(
        name, gyroYawSupplier.get().getDegrees(), 0, 0, 0, 0, 0);

    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    inputs.connected = estimate != null && LimelightHelpers.getTV(name);

    if (estimate == null || estimate.tagCount == 0) {
      inputs.tagIds = new int[] {};
      inputs.poseObservations = new PoseObservation[] {};
      return;
    }

    Set<Integer> tagIds = new HashSet<>();
    for (var fiducial : estimate.rawFiducials) {
      tagIds.add(fiducial.id);
    }
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();

    // Ambiguity is only meaningful for single-tag observations (multi-tag solves are already
    // well-constrained); MegaTag2 doesn't disambiguate per-tag the way MegaTag1 does, so fall
    // back to the first raw fiducial's reported ambiguity, matching the 2024 robot's approach.
    double ambiguity =
        estimate.tagCount == 1 && estimate.rawFiducials.length > 0
            ? estimate.rawFiducials[0].ambiguity
            : 0.0;

    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              estimate.timestampSeconds,
              estimate.pose,
              ambiguity,
              estimate.tagCount,
              estimate.avgTagDist)
        };
  }
}
