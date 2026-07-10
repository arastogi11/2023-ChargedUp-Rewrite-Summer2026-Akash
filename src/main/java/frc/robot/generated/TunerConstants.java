package frc.robot.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

/**
 * Swerve module/drivetrain constants for the rebuilt 2023 Charged Up robot.
 *
 * <p>CAN IDs, module offsets, track width, and gearing (SDS MK4i L3) are carried over from the
 * original 2023 robot's {@code Constants.Swerve} (Team 364 BaseFalconSwerve template, Phoenix v5).
 * The drive characterization (kS/kV/kA) below is derived from the 2023 robot's SysId results,
 * converted from volts-per-(meter/sec) to volts-per-(wheel rotation/sec) using the SDS MK4i wheel
 * circumference. The steer gains and slip current are placeholder starting values (the original
 * robot never SysId-characterized the turn axis) -- both should be re-tuned with the SysId
 * routines wired up in {@code Drive} once this runs on real hardware.
 */
public class TunerConstants {
  // Steer motor gains -- PLACEHOLDER, re-tune via SysId on hardware (2023 robot's angle PID
  // was configured in Phoenix v5 native units, which don't translate to Phoenix6 Slot0Configs).
  private static final Slot0Configs steerGains =
      new Slot0Configs()
          .withKP(225)
          .withKI(0)
          .withKD(0.5)
          .withKS(0.26056)
          .withKV(0.81715)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

  // Drive motor gains -- kS/kV converted from the 2023 robot's SysId characterization
  // (driveKS=0.15565 V, driveKV=2.0206 V*s/m, driveKA=0.94648 V*s^2/m) into per-wheel-rotation
  // units using the MK4i wheel circumference (kWheelRadius below): kV_rot = kV_m * circumference.
  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(0.2).withKI(0).withKD(0).withKS(0.15565).withKV(0.6450).withKA(0.3021);

  private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
  private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

  // Remote CANcoder feedback for the steer axis, matching the 2023 robot's module wiring.
  private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

  // Slip current -- PLACEHOLDER (2023's v5 current limits don't map directly); re-tune on
  // hardware.
  private static final Current kSlipCurrent = Amps.of(60);

  // "Initial configs" are applied to each module's motors once, at startup, before the
  // gains/gear-ratio/etc. settings above are layered on top -- this is where you'd set anything
  // Phoenix6 doesn't have a dedicated factory method for. Here it's just current limiting for the
  // steer motor (below), so the small, low-torque steer axis can't accidentally brown out the
  // robot the way an unlimited drive motor could.
  private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
  private static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(40))
                  .withStatorCurrentLimitEnable(true));
  private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
  private static final Pigeon2Configuration pigeonConfigs = null;

  // CAN bus name -- renamed off the 2023 robot's joke name ("CANt_open_file").
  public static final CANBus kCANBus = new CANBus("canivore", "./logs/example.hoot");

  // 2023 robot's tuned max speed (Constants.Swerve.maxSpeed).
  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.5);

  // SDS MK4i modules do have real steer/drive coupling internally, but the 2023 robot's v5-era
  // code never characterized or modeled it -- left at 0 (no compensation) rather than guessing a
  // wrong value; tune on hardware if position tracking drift is observed after steering.
  private static final double kCoupleRatio = 0.0;

  // SDS MK4i L3 gearing (Constants.Swerve.chosenModule = COTSFalconSwerveConstants.SDSMK4i L3).
  private static final double kDriveGearRatio = 6.75;
  private static final double kSteerGearRatio = 150.0 / 7.0;
  private static final Distance kWheelRadius = Inches.of(2.0);

  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;

  // Constants.Swerve.pigeonID
  private static final int kPigeonId = 1;

  // Simulation-only inertia/friction placeholders.
  private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
  private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);
  private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
  private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants()
          .withCANBusName(kCANBus.getName())
          .withPigeon2Id(kPigeonId)
          .withPigeon2Configs(pigeonConfigs);

  // A factory that bundles every setting shared by all four swerve modules (gearing, gains,
  // current limits, etc. -- everything defined above this point) so each module below only needs
  // to supply what's actually different about it: CAN IDs, its CANcoder's mounting offset, and its
  // physical position on the chassis.
  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withCouplingGearRatio(kCoupleRatio)
              .withWheelRadius(kWheelRadius)
              .withSteerMotorGains(steerGains)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
              .withSlipCurrent(kSlipCurrent)
              .withSpeedAt12Volts(kSpeedAt12Volts)
              .withDriveMotorType(kDriveMotorType)
              .withSteerMotorType(kSteerMotorType)
              .withFeedbackSource(kSteerFeedbackType)
              .withDriveMotorInitialConfigs(driveInitialConfigs)
              .withSteerMotorInitialConfigs(steerInitialConfigs)
              .withEncoderInitialConfigs(encoderInitialConfigs)
              .withSteerInertia(kSteerInertia)
              .withDriveInertia(kDriveInertia)
              .withSteerFrictionVoltage(kSteerFrictionVoltage)
              .withDriveFrictionVoltage(kDriveFrictionVoltage);

  // Each module's CANcoder is bolted to the swerve module in whatever rotational position the
  // mechanical assembly happened to land in -- there's no way to physically align it to exactly
  // zero. So "encoder offset" is the CANcoder reading, in rotations, at the module's true
  // mechanical zero (wheel pointed straight forward); the encoder config subtracts this offset so
  // that a *reported* position of zero always means "pointed forward", regardless of how the
  // sensor happened to be mounted. These four numbers are specific to this physical robot and
  // would need to be re-measured (spin each wheel to point forward, read the raw CANcoder value)
  // if a module is ever swapped or the CANcoder is ever removed and reinstalled.

  // Front Left -- Constants.Swerve.Mod0 (2023 robot)
  private static final int kFrontLeftDriveMotorId = 1;
  private static final int kFrontLeftSteerMotorId = 2;
  private static final int kFrontLeftEncoderId = 1;
  private static final Angle kFrontLeftEncoderOffset = Rotations.of(110.654296875 / 360.0);
  private static final boolean kFrontLeftSteerMotorInverted = false;
  private static final boolean kFrontLeftEncoderInverted = false;

  // 2023 robot: trackWidth = wheelBase = 0.521 m (square chassis)
  private static final Distance kFrontLeftXPos = Meters.of(0.2605);
  private static final Distance kFrontLeftYPos = Meters.of(0.2605);

  // Front Right -- Constants.Swerve.Mod1
  private static final int kFrontRightDriveMotorId = 3;
  private static final int kFrontRightSteerMotorId = 4;
  private static final int kFrontRightEncoderId = 2;
  private static final Angle kFrontRightEncoderOffset = Rotations.of(215.15625 / 360.0);
  private static final boolean kFrontRightSteerMotorInverted = false;
  private static final boolean kFrontRightEncoderInverted = false;

  private static final Distance kFrontRightXPos = Meters.of(0.2605);
  private static final Distance kFrontRightYPos = Meters.of(-0.2605);

  // Back Left -- Constants.Swerve.Mod2
  private static final int kBackLeftDriveMotorId = 5;
  private static final int kBackLeftSteerMotorId = 6;
  private static final int kBackLeftEncoderId = 3;
  private static final Angle kBackLeftEncoderOffset = Rotations.of(203.37890625 / 360.0);
  private static final boolean kBackLeftSteerMotorInverted = true;
  private static final boolean kBackLeftEncoderInverted = false;

  private static final Distance kBackLeftXPos = Meters.of(-0.2605);
  private static final Distance kBackLeftYPos = Meters.of(0.2605);

  // Back Right -- Constants.Swerve.Mod3
  private static final int kBackRightDriveMotorId = 7;
  private static final int kBackRightSteerMotorId = 8;
  private static final int kBackRightEncoderId = 4;
  private static final Angle kBackRightEncoderOffset = Rotations.of(52.646484375 / 360.0);
  private static final boolean kBackRightSteerMotorInverted = true;
  private static final boolean kBackRightEncoderInverted = false;

  private static final Distance kBackRightXPos = Meters.of(-0.2605);
  private static final Distance kBackRightYPos = Meters.of(-0.2605);

  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft =
          ConstantCreator.createModuleConstants(
              kFrontLeftSteerMotorId,
              kFrontLeftDriveMotorId,
              kFrontLeftEncoderId,
              kFrontLeftEncoderOffset,
              kFrontLeftXPos,
              kFrontLeftYPos,
              kInvertLeftSide,
              kFrontLeftSteerMotorInverted,
              kFrontLeftEncoderInverted);
  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight =
          ConstantCreator.createModuleConstants(
              kFrontRightSteerMotorId,
              kFrontRightDriveMotorId,
              kFrontRightEncoderId,
              kFrontRightEncoderOffset,
              kFrontRightXPos,
              kFrontRightYPos,
              kInvertRightSide,
              kFrontRightSteerMotorInverted,
              kFrontRightEncoderInverted);
  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft =
          ConstantCreator.createModuleConstants(
              kBackLeftSteerMotorId,
              kBackLeftDriveMotorId,
              kBackLeftEncoderId,
              kBackLeftEncoderOffset,
              kBackLeftXPos,
              kBackLeftYPos,
              kInvertLeftSide,
              kBackLeftSteerMotorInverted,
              kBackLeftEncoderInverted);
  public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight =
          ConstantCreator.createModuleConstants(
              kBackRightSteerMotorId,
              kBackRightDriveMotorId,
              kBackRightEncoderId,
              kBackRightEncoderOffset,
              kBackRightXPos,
              kBackRightYPos,
              kInvertRightSide,
              kBackRightSteerMotorInverted,
              kBackRightEncoderInverted);
}
