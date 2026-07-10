// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.LoggingControl;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * The swerve drivetrain subsystem: owns the four {@link Module}s (drive + steer motor pairs) and
 * the gyro, combines them into a single robot-wide velocity/pose interface, and keeps a running
 * estimate of the robot's position on the field.
 *
 * <p>Two different position estimates get fused together here, which is worth understanding since
 * it's the core idea behind almost every FRC drivetrain:
 *
 * <ul>
 *   <li><b>Odometry</b>: every loop, each module reports how far its wheel has spun since last
 *       time and which way it's pointed. Combined with the gyro's heading, that's enough geometry
 *       (see {@link SwerveDriveKinematics}) to calculate exactly how far the whole robot must have
 *       moved. This is precise moment-to-moment, but small errors (wheel slip, tiny gyro drift)
 *       accumulate over a match, so the estimate slowly drifts from reality.
 *   <li><b>Vision</b>: {@link frc.robot.subsystems.vision.Vision} periodically supplies an
 *       absolute pose reading from AprilTags, which doesn't drift but only updates a few times a
 *       second and is noisier per-sample.
 * </ul>
 *
 * <p>{@link SwerveDrivePoseEstimator} (see {@code poseEstimator} below) is a Kalman-filter-like
 * WPILib class that blends both sources automatically, weighted by how much each measurement's
 * reported "standard deviation" (uncertainty) trusts it -- odometry runs every loop and is
 * generally trusted a lot, vision corrects the accumulated drift periodically and is trusted only
 * as much as {@link frc.robot.subsystems.vision.VisionConstants#xyStdDev} says to.
 */
public class Drive extends SubsystemBase {
  // TunerConstants doesn't include these constants, so they are declared locally.
  // Odometry runs much faster than the normal 50Hz robot loop when the modules are on a CAN FD
  // ("CAN with Flexible Data-rate", the newer, higher-bandwidth CAN bus standard) bus like a
  // CANivore -- 250 samples/sec instead of 100 -- because more, more-frequent position samples
  // make the pose estimate smoother and reduce the odometry drift described above.
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;

  // The distance from the robot's center to its farthest-out swerve module -- needed to convert
  // between the robot's overall rotational speed and each individual wheel's linear speed
  // (a point farther from the center has to move faster to achieve the same rotation rate).
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants -- placeholder mass/MOI, re-measure/tune on the real robot.
  private static final double ROBOT_MASS_KG = 55.0;
  private static final double ROBOT_MOI = 6.0;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  // A lock (mutex) shared with PhoenixOdometryThread: that background thread reads high-frequency
  // sensor samples on its own schedule, independent of the normal 50Hz robot loop, so this lock
  // prevents this class's periodic() from reading module data mid-update and seeing a
  // half-written, inconsistent set of samples.
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  // An Alert shows up automatically on the Driver Station and any connected dashboard whenever
  // .set(true) is called -- no manual dashboard wiring needed. Used throughout this codebase for
  // "hey, something's wrong" conditions a driver should notice mid-match.
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  // Kinematics converts between "how is the whole robot moving" (a single ChassisSpeeds: forward,
  // sideways, and rotational velocity) and "how is each individual swerve module moving" (four
  // separate SwerveModuleStates: each module's own speed and steering angle) -- the core geometry
  // problem every swerve drivetrain has to solve, in both directions.
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  // The module positions read on the previous odometry update, kept around so periodic() can
  // compute how far each wheel moved *this* update (the delta) rather than its absolute position.
  private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  // The two VecBuilder.fill(...) arguments are the estimator's starting trust levels: how much to
  // trust odometry-only tracking (x meters, y meters, theta radians of expected error) vs. how
  // much to trust a vision measurement by default before Vision.java supplies its own
  // per-measurement values (see Drive class javadoc above for the odometry/vision fusion idea).
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          kinematics,
          rawGyroRotation,
          lastModulePositions,
          Pose2d.kZero,
          VecBuilder.fill(0.2, 0.2, 0.2),
          VecBuilder.fill(0.1, 0.1, 99));

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Tells the WPILib usage-reporting system this robot uses an AdvantageKit-style swerve
    // drivetrain -- purely telemetry FIRST collects, has no effect on robot behavior.
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // The odometry thread (see PhoenixOdometryThread) runs on its own background thread at
    // ODOMETRY_FREQUENCY, independent of the normal 50Hz loop, so position samples are as
    // frequent and evenly-spaced as the CAN bus allows rather than being limited to once per robot
    // loop.
    PhoenixOdometryThread.getInstance().start();

    // AutoBuilder is PathPlannerLib's entry point: give it function references for reading the
    // current pose, resetting it, reading current chassis speeds, and commanding a new chassis
    // speed, plus a PPHolonomicDriveController (the PID controller PathPlanner uses internally to
    // follow a path -- separate PID gains for translation vs. rotation), and it can then
    // autonomously drive any path/auto authored in the PathPlanner GUI without this class needing
    // to know anything about path-following itself. The alliance-color supplier flips
    // blue-origin-authored paths to red automatically.
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(2, 0.0, 0.0), new PIDConstants(10, 0.0, 0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    // LocalADStarAK lets PathPlanner dynamically route around obstacles at runtime (rather than
    // only following pre-authored paths exactly), with its internal state made replay-safe by
    // routing it through AdvantageKit's logger -- see the class itself for details.
    Pathfinding.setPathfinder(new LocalADStarAK());
    // These two callbacks just forward PathPlanner's internal state (the path it's currently
    // following, and where it wants the robot right now) into AdvantageKit's logger, so you can
    // see the planned trajectory overlaid on the actual robot path in AdvantageScope.
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // SysIdRoutine drives the robot through WPILib's standard system-identification test
    // sequence and logs voltage/velocity/position -- see RobotContainer's auto chooser comment
    // for what this data is used for (deriving real kS/kV/kA feedforward gains).
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // Hold the odometry lock while reading gyro/module inputs, since the background odometry
    // thread (PhoenixOdometryThread) writes to these same queues concurrently -- without the
    // lock we could read a queue mid-write and see inconsistent data.
    odometryLock.lock();
    // updateInputs always runs -- the gyro-connected fallback logic below and pose estimation
    // depend on it -- but processInputs (the expensive part: serializing to every log receiver)
    // is skipped when LoggingControl says to cut back, see its javadoc for why.
    gyroIO.updateInputs(gyroInputs);
    if (LoggingControl.enabled()) {
      Logger.processInputs("Drive/Gyro", gyroInputs);
    }
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled() && LoggingControl.enabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry. Because the background thread samples faster than this periodic() method
    // runs (see ODOMETRY_FREQUENCY), there can be multiple new samples queued up since last loop
    // -- this loop processes every one of them in order, rather than just the latest, so no
    // position data is thrown away even at high sample rates.
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Gyro disconnected (see the Alert below) -- fall back to estimating rotation purely from
        // how much each wheel moved relative to the others. Kinematics.toTwist2d converts a set
        // of module deltas into a single robot-wide "twist" (small linear + rotational motion);
        // its rotational component is the best rotation estimate available without a gyro, though
        // it's less accurate (wheel slip corrupts it directly, where the gyro is immune to that).
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Feed this sample into the pose estimator. updateWithTime (rather than a plain update)
      // matters because we're processing possibly-several-loops'-worth of queued samples at once
      // here -- passing each one's real timestamp keeps the estimator's internal time-ordering
      // correct even though they're all being applied within a single call to periodic().
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert. Suppressed in SIM, since GyroIO's simulation stub never reports
    // "connected" (there's no real gyro to connect to) but that's expected, not a fault.
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    // Publish the latest pose to the shared RobotState singleton so other subsystems (or code
    // that doesn't otherwise have a reference to Drive) can read the robot's current position.
    RobotState.getInstance().robotPose = getPose();
  }

  /**
   * Runs the drive at the desired velocity. This is the one method basically everything else in
   * the codebase eventually calls to actually move the robot -- {@link
   * frc.robot.commands.DriveCommands#joystickDrive}, PathPlanner autos (via {@code
   * AutoBuilder.configure} above), and {@link #stop()} all funnel through here.
   *
   * @param speeds Speeds in meters/sec (and radians/sec for rotation)
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // ChassisSpeeds.discretize compensates for a subtle swerve-specific error: commanding
    // simultaneous linear + rotational velocity for a full 20ms loop actually traces a curved
    // path, not a straight line, so a naive implementation "skews" sideways slightly during
    // combined translate+rotate. Discretizing corrects the commanded speeds so the resulting
    // *discrete* 20ms step ends up at the intended pose instead.
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    // Convert the single robot-wide velocity into each module's individual speed + angle.
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    // If satisfying the requested motion would require any one module to spin faster than the
    // robot's actual top speed, scale ALL four modules' speeds down proportionally rather than
    // clamping that one module alone -- otherwise the robot would drive in the wrong direction
    // (imagine commanding straight-line motion where one corner module needs to go faster to
    // also satisfy a rotation component; clamping only that module bends the actual path).
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // This runs every loop the drive is being commanded (i.e. almost every loop of a match), so
    // it's one of the higher-value places to respect the logging kill switch -- see
    // LoggingControl's javadoc.
    if (LoggingControl.enabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
      Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);
    }

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    if (LoggingControl.enabled()) {
      Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules
   * will return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a new timestamped vision measurement -- called by {@link
   * frc.robot.subsystems.vision.Vision} once per accepted camera observation. {@code
   * visionMeasurementStdDevs} is how the vision subsystem tells the pose estimator how much to
   * trust this particular reading (smaller = more trusted; see {@link
   * frc.robot.subsystems.vision.VisionConstants#xyStdDev} for how that number is computed).
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
