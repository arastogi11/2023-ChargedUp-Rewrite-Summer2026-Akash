// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import java.util.function.Supplier;

/**
 * Real hardware IO for a single Limelight, using MegaTag2 (gyro-seeded) pose estimation via the
 * official LimelightHelpers API rather than parsing raw NetworkTables arrays by hand.
 */
public class VisionIOLimelight implements VisionIO {
  private final String name;
  private final Supplier<Rotation2d> gyroYawSupplier;

  /**
   * @param name the Limelight's configured hostname/NetworkTables table name (e.g.
   *     "limelight-front") -- must match what's set in the Limelight's own web UI.
   * @param gyroYawSupplier where to read the current gyro heading from (usually {@code
   *     Drive::getRotation}) -- MegaTag2 needs this pushed to the camera every loop, see {@link
   *     #updateInputs}.
   */
  public VisionIOLimelight(String name, Supplier<Rotation2d> gyroYawSupplier) {
    this.name = name;
    this.gyroYawSupplier = gyroYawSupplier;

    // One-time setup, done once at construction rather than every loop: tell this specific
    // Limelight which AprilTag IDs are worth reporting at all (see VisionConstants javadoc for
    // why this allowlist idea is carried over from 2024), and where on the robot it's physically
    // mounted (used by the Limelight's own onboard pose-solving math).
    LimelightHelpers.SetFiducialIDFiltersOverride(name, VisionConstants.validFiducialIds);
    var camera = camera();
    LimelightHelpers.setCameraPose_RobotSpace(
        name, camera.forward(), camera.side(), camera.up(), camera.roll(), camera.pitch(), camera.yaw());
  }

  /** Looks up this camera's mount-offset config from VisionConstants by name. */
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
    // MegaTag2 is Limelight's gyro-assisted pose-solving algorithm: rather than computing
    // rotation purely from what it sees (which is noisier), it trusts the robot's own gyro for
    // heading and only solves for position -- which is why the current heading has to be pushed
    // to the camera every single loop, before asking it for a pose estimate.
    LimelightHelpers.SetRobotOrientation(
        name, gyroYawSupplier.get().getDegrees(), 0, 0, 0, 0, 0);

    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    // getTV ("target valid") is Limelight's own basic "do I currently see any AprilTag at all"
    // flag; combined with a non-null estimate, this is as close as NetworkTables-based vision gets
    // to a real "is this camera actually connected and working" signal -- there's no persistent
    // TCP-style connection to check the state of the way there is for a CAN device.
    inputs.connected = estimate != null && LimelightHelpers.getTV(name);

    if (estimate == null || estimate.tagCount == 0) {
      inputs.tagIds = new int[] {};
      inputs.poseObservations = new PoseObservation[] {};
      return;
    }

    // Each raw fiducial in a single Limelight frame represents one distinct detected tag, so a
    // direct primitive-int copy is enough here -- no need for a HashSet to deduplicate (which
    // would cost a hash table plus one Integer box per tag, all just discarded every loop) or a
    // stream (which boxes/unboxes internally). This runs every single loop this camera is
    // enabled, so avoiding needless per-loop garbage matters more here than almost anywhere else
    // in the codebase.
    int[] tagIds = new int[estimate.rawFiducials.length];
    for (int i = 0; i < estimate.rawFiducials.length; i++) {
      tagIds[i] = estimate.rawFiducials[i].id;
    }
    inputs.tagIds = tagIds;

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
