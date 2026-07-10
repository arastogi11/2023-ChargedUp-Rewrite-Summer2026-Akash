// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Fuses one or more Limelights' pose observations into the drivetrain's pose estimator, gated by
 * the reject-pose "throwout" logic the user wanted carried forward from 2024/2025 -- see {@link
 * VisionConstants} javadoc for exactly which idea came from which season.
 */
public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    inputs = new VisionIOInputsAutoLogged[io.length];
    disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      disconnectedAlerts[i] =
          new Alert("Vision camera " + i + " (" + VisionConstants.cameras[i].name() + ") is disconnected.", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    List<Pose2d> allPosesAccepted = new LinkedList<>();
    List<Pose2d> allPosesRejected = new LinkedList<>();

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      io[cameraIndex].updateInputs(inputs[cameraIndex]);
      Logger.processInputs("Vision/Camera" + cameraIndex, inputs[cameraIndex]);
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      List<Pose2d> posesAccepted = new LinkedList<>();
      List<Pose2d> posesRejected = new LinkedList<>();

      for (var observation : inputs[cameraIndex].poseObservations) {
        boolean reject =
            observation.tagCount() == 0
                || (observation.tagCount() == 1 && observation.ambiguity() > VisionConstants.maxAmbiguity)
                || observation.avgTagDistMeters() > VisionConstants.maxTagDistanceMeters
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > VisionConstants.aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > VisionConstants.aprilTagLayout.getFieldWidth();

        if (reject) {
          posesRejected.add(observation.pose());
          continue;
        }
        posesAccepted.add(observation.pose());

        // Continuous distance/tag-count standard-deviation formula (2025 Reefscape, restored
        // here vs. the flat baseline currently active in 177-Rebuilt's 2026 template).
        double xyStdDev =
            VisionConstants.xyStdDevCoefficient
                * observation.avgTagDistMeters()
                / Math.sqrt(observation.tagCount());
        Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, VisionConstants.thetaStdDev);

        consumer.accept(observation.pose(), observation.timestamp(), stdDevs);
      }

      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/PosesAccepted", posesAccepted.toArray(new Pose2d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/PosesRejected", posesRejected.toArray(new Pose2d[0]));

      allPosesAccepted.addAll(posesAccepted);
      allPosesRejected.addAll(posesRejected);
    }

    Logger.recordOutput("Vision/Summary/PosesAccepted", allPosesAccepted.toArray(new Pose2d[0]));
    Logger.recordOutput("Vision/Summary/PosesRejected", allPosesRejected.toArray(new Pose2d[0]));
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> stdDevs);
  }
}
