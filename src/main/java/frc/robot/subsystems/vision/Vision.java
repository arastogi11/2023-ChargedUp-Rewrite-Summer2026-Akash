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
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * Fuses one or more Limelights' pose observations into the drivetrain's pose estimator, gated by
 * the reject-pose "throwout" logic the user wanted carried forward from 2024/2025 -- see {@link
 * VisionConstants} javadoc for exactly which idea came from which season.
 *
 * <p>The physical 2023 robot currently has its Limelights removed, so this subsystem is
 * dashboard-toggleable (default OFF) rather than always-on -- see {@link #enabled}. Camera IO is
 * NetworkTables-based and already null/zero-tag-safe with no cameras present, so nothing here
 * would actually crash without the toggle; it exists so the driver has explicit, visible control
 * over whether vision is contributing to the pose estimate, and so the "disconnected" alerts don't
 * spam the dashboard for hardware that's intentionally absent.
 */
public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  // Published at a plain NetworkTables path (not "/SmartDashboard/...") so it shows up as a
  // regular NT4 topic in Elastic without depending on the legacy SmartDashboard/Shuffleboard
  // Java APIs. Bind it to a toggle-switch widget in the Elastic layout.
  private final LoggedNetworkBoolean enabled = new LoggedNetworkBoolean("/DriverDashboard/VisionEnabled", false);

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
    Logger.recordOutput("Vision/Enabled", enabled.get());
    if (!enabled.get()) {
      // Don't poll cameras or show "disconnected" warnings for hardware that's intentionally off.
      for (Alert alert : disconnectedAlerts) {
        alert.set(false);
      }
      return;
    }

    List<Pose2d> allPosesAccepted = new LinkedList<>();
    List<Pose2d> allPosesRejected = new LinkedList<>();

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      io[cameraIndex].updateInputs(inputs[cameraIndex]);
      Logger.processInputs("Vision/Camera" + cameraIndex, inputs[cameraIndex]);
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      List<Pose2d> posesAccepted = new LinkedList<>();
      List<Pose2d> posesRejected = new LinkedList<>();

      for (var observation : inputs[cameraIndex].poseObservations) {
        if (VisionConstants.shouldRejectPose(observation)) {
          posesRejected.add(observation.pose());
          continue;
        }
        posesAccepted.add(observation.pose());

        double xyStdDev = VisionConstants.xyStdDev(observation);
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
