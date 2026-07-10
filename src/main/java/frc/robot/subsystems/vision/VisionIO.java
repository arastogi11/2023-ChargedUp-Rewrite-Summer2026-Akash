// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware-agnostic interface for a single camera's AprilTag-based pose estimation -- see {@code
 * frc.robot.subsystems.drive.ModuleIO}'s javadoc for the full explanation of the IO-interface
 * pattern. {@link Vision} owns one of these per physical camera and decides, using {@link
 * VisionConstants#shouldRejectPose}, whether to actually trust each {@link PoseObservation} it
 * produces -- this interface's job is only to report what the camera saw, not to judge it.
 */
public interface VisionIO {
  /**
   * A single accepted-or-rejected pose observation from one camera update. This gets fed straight
   * into an {@code @AutoLog} array below, and (per {@code VisionIOLimelight}) is itself
   * WPILib-struct-serializable via its {@link Pose2d} field, so AdvantageKit can log/replay it with
   * no extra work.
   */
  public static record PoseObservation(
      double timestamp, Pose2d pose, double ambiguity, int tagCount, double avgTagDistMeters) {}

  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    // Every AprilTag ID currently visible to this camera, whether or not it ended up contributing
    // to an accepted pose observation -- useful for debugging what the camera can actually see.
    public int[] tagIds = new int[] {};
    public PoseObservation[] poseObservations = new PoseObservation[] {};
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
