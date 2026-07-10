// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * Vision tuning constants. The reject-pose thresholds and the continuous standard-deviation
 * formula are the "throwout logic" the user specifically wanted carried forward from the 2024 and
 * 2025 seasons:
 *
 * <ul>
 *   <li>Tag-count / ambiguity / field-bounds rejection matches both the 2024 robot's {@code
 *       Vision.getPoseValidMG2} and the current (177-Rebuilt comp/2026) canonical AdvantageKit
 *       vision template.
 *   <li>The fiducial-ID allowlist (only accept tags the current game actually uses) is the 2024
 *       robot's idea -- for 2023 Charged Up all 8 field tags are scoring-relevant, so this mostly
 *       acts as a sanity filter against garbage IDs rather than meaningfully narrowing anything,
 *       but it costs nothing to keep.
 *   <li>The continuous distance/tag-count standard-deviation formula ({@code xyStdDev = k *
 *       avgTagDist / sqrt(tagCount)}) is the 2025 robot's validated approach -- more accurate than
 *       2024's binary tag-count step, and more accurate than the flat baseline currently active
 *       (with the real formula commented out) in 177-Rebuilt's 2026 template.
 * </ul>
 */
public class VisionConstants {
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2023ChargedUp);

  // 2023 Charged Up used all 8 field tags for scoring-grid/loading-zone alignment.
  public static final int[] validFiducialIds = {1, 2, 3, 4, 5, 6, 7, 8};

  // Basic filtering thresholds (2024/2026-template convention).
  public static final double maxAmbiguity = 0.4; // 2024's poseAmbiguityThreshold
  public static final double maxTagDistanceMeters = 5.5; // 2024's live-tuned throwoutDist

  // Continuous stddev formula constant (2025 Reefscape's validated xyStdDev = k * avgTagDist /
  // sqrt(tagCount)).
  public static final double xyStdDevCoefficient = 0.05;
  // MegaTag2 is gyro-locked for rotation, so we always trust the gyro over vision for theta.
  public static final double thetaStdDev = Double.POSITIVE_INFINITY;

  /** Per-camera identity and its mount offset in robot space (forward/side/up meters, degrees). */
  public static record CameraConfig(
      String name, double forward, double side, double up, double roll, double pitch, double yaw) {}

  // 2023 robot had a front and a back Limelight. Mount offsets weren't used in the 2023 code (it
  // read botpose directly without a client-side camera-to-robot transform), so these are
  // placeholders -- TODO: measure and set from CAD/hardware before relying on MegaTag2 accuracy.
  public static final CameraConfig[] cameras = {
    new CameraConfig("limelight-front", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    new CameraConfig("limelight-back", 0.0, 0.0, 0.0, 0.0, 0.0, 180.0),
  };

  /**
   * Pure reject-pose function, pulled out of {@link Vision#periodic()} so it's unit-testable
   * without NetworkTables/hardware -- see class javadoc for what each threshold does and where it
   * came from.
   */
  public static boolean shouldRejectPose(VisionIO.PoseObservation observation) {
    return observation.tagCount() == 0 // No tags visible -- nothing to compute a pose from.
        // "Ambiguity" is how confident the camera's solver is in a single-tag pose solve (a lone
        // flat AprilTag can sometimes be explained almost equally well by two different camera
        // positions/angles -- classic "pose ambiguity" in computer vision). Multiple tags at once
        // resolve that ambiguity geometrically, so this check only applies with exactly one tag
        // visible.
        || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
        // Farther-away tags produce noisier pose estimates (small pixel errors translate to larger
        // real-world position errors), so reject anything past a sanity-check distance outright
        // rather than just trusting it less (see xyStdDev below for the "trust it less" half of
        // this idea, applied to everything that isn't rejected).
        || observation.avgTagDistMeters() > maxTagDistanceMeters
        // A correct pose can never fall outside the physical field -- if the camera's solve says
        // otherwise, something went wrong (misidentified tag, reflection, etc.) and the reading
        // should be thrown out rather than corrupting the pose estimator.
        || observation.pose().getX() < 0.0
        || observation.pose().getX() > aprilTagLayout.getFieldLength()
        || observation.pose().getY() < 0.0
        || observation.pose().getY() > aprilTagLayout.getFieldWidth();
  }

  /** Continuous distance/tag-count standard-deviation formula (2025 Reefscape). */
  public static double xyStdDev(VisionIO.PoseObservation observation) {
    return xyStdDevCoefficient * observation.avgTagDistMeters() / Math.sqrt(observation.tagCount());
  }
}
