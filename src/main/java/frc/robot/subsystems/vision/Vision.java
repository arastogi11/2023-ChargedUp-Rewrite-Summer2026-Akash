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
import frc.robot.util.LoggingControl;
import java.util.ArrayList;
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
 *
 * <p>Every AdvantageKit recording call below is separately gated by {@link LoggingControl} (the
 * roboRIO 1 memory kill switch), but the pose-estimation math itself (reading camera inputs,
 * running the reject-pose check, and calling {@code consumer.accept}) is never gated -- vision
 * correction has to keep working even with detailed logging turned off. Building the
 * accepted/rejected pose lists purely for logging is skipped entirely while logging is off, rather
 * than built and then discarded, since those lists (and the array conversions/allocations that
 * come with them) exist only to feed {@code Logger.recordOutput} and have no other purpose.
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

  /**
   * @param consumer where accepted vision measurements get sent -- in practice always {@code
   *     Drive::addVisionMeasurement}, but taking a functional interface instead of a direct {@code
   *     Drive} reference means this class doesn't need to import or know about {@code Drive} at
   *     all, which keeps the two subsystems decoupled (Vision could feed any pose consumer, not
   *     specifically a swerve drivetrain).
   * @param io one {@link VisionIO} per physical camera (varargs -- pass as many as you have).
   */
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
    // Read once per loop rather than calling LoggingControl.enabled() repeatedly below -- it's
    // cheap either way, but this makes the gating below easier to follow.
    boolean logging = LoggingControl.enabled();
    if (logging) {
      Logger.recordOutput("Vision/Enabled", enabled.get());
    }

    if (!enabled.get()) {
      // Don't poll cameras or show "disconnected" warnings for hardware that's intentionally off.
      for (Alert alert : disconnectedAlerts) {
        alert.set(false);
      }
      return;
    }

    // null while logging is disabled -- nothing below should read these unless `logging` is true.
    List<Pose2d> allPosesAccepted = logging ? new ArrayList<>() : null;
    List<Pose2d> allPosesRejected = logging ? new ArrayList<>() : null;

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Always runs -- this is what actually reads the camera and feeds pose corrections into
      // the drivetrain, not just logging.
      io[cameraIndex].updateInputs(inputs[cameraIndex]);
      if (logging) {
        Logger.processInputs("Vision/Camera" + cameraIndex, inputs[cameraIndex]);
      }
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      List<Pose2d> posesAccepted = logging ? new ArrayList<>() : null;
      List<Pose2d> posesRejected = logging ? new ArrayList<>() : null;

      for (var observation : inputs[cameraIndex].poseObservations) {
        if (VisionConstants.shouldRejectPose(observation)) {
          if (logging) {
            posesRejected.add(observation.pose());
          }
          continue;
        }
        if (logging) {
          posesAccepted.add(observation.pose());
        }

        double xyStdDev = VisionConstants.xyStdDev(observation);
        Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, VisionConstants.thetaStdDev);

        // Always runs, regardless of the logging toggle -- this is the actual vision-to-pose-
        // estimator correction, not something logging-related.
        consumer.accept(observation.pose(), observation.timestamp(), stdDevs);
      }

      if (logging) {
        // Logging accepted and rejected poses separately (rather than just the final fused pose)
        // means you can open a match log in AdvantageScope afterward and actually see which
        // specific vision readings were thrown out and why -- invaluable for diagnosing "why did
        // the pose estimate jump" after the fact instead of only while it's happening live.
        Logger.recordOutput(
            "Vision/Camera" + cameraIndex + "/PosesAccepted", posesAccepted.toArray(new Pose2d[0]));
        Logger.recordOutput(
            "Vision/Camera" + cameraIndex + "/PosesRejected", posesRejected.toArray(new Pose2d[0]));

        allPosesAccepted.addAll(posesAccepted);
        allPosesRejected.addAll(posesRejected);
      }
    }

    if (logging) {
      Logger.recordOutput("Vision/Summary/PosesAccepted", allPosesAccepted.toArray(new Pose2d[0]));
      Logger.recordOutput("Vision/Summary/PosesRejected", allPosesRejected.toArray(new Pose2d[0]));
    }
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> stdDevs);
  }
}
