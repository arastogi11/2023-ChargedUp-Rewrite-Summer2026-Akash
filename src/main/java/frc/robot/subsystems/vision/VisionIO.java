// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  /** A single accepted-or-rejected pose observation from one camera update. */
  public static record PoseObservation(
      double timestamp, Pose2d pose, double ambiguity, int tagCount, double avgTagDistMeters) {}

  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public int[] tagIds = new int[] {};
    public PoseObservation[] poseObservations = new PoseObservation[] {};
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
