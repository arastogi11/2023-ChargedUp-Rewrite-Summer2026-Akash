// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Robot-wide numerical/boolean constants live here, grouped into a nested class per subsystem.
 * Do not put anything functional in this class.
 */
public final class Constants {
  private Constants() {}

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /**
   * CAN IDs and limit-switch ports carried over from the 2023 robot's {@code
   * Constants.ElevatorConstants}. The four named setpoints there (bottomPos=0, midPos=-62256,
   * highPos=-104446, shelfPos=-76050) were raw Phoenix v5 integrated-sensor ticks with no stated
   * gear ratio or drum radius, so they don't convert to Phoenix6 rotations directly. The values
   * below preserve the *relative spacing* between the 2023 setpoints (same gearing presumably
   * applies) scaled onto a placeholder rotation range -- re-measure on hardware once built.
   */
  public static final class ElevatorConstants {
    private ElevatorConstants() {}

    public static final int motorId = 9;
    public static final int bottomLimitPort = 0;
    public static final int topLimitPort = 1;

    public static final double statorCurrentLimitAmps = 60;
    public static final double motionMagicCruiseVelocityRotPerSec = 20;
    public static final double motionMagicAccelerationRotPerSecSq = 40;

    public static final Slot0Configs gains =
        new Slot0Configs().withKP(4.0).withKI(0).withKD(0).withKS(0.0).withKV(0.0).withKA(0.0);

    // Placeholder rotation setpoints -- see class javadoc.
    public static final double bottomPositionRotations = 0.0;
    public static final double midPositionRotations = 5.0;
    public static final double highPositionRotations = 8.39;
    public static final double shelfPositionRotations = 6.11;
  }

  /**
   * Carried over from the 2023 robot's {@code Constants.ArmConstants} / {@code
   * Constants.armCanCoderID}. Unlike the elevator, the arm's CANcoder reported real physical
   * degrees directly (the 2023 code read/compared {@code armEncoder.getPosition()} against
   * degree-valued constants), so those setpoints convert cleanly to rotations (degrees / 360).
   * {@code rotorToSensorRatio} (the gearbox ratio between the TalonFX rotor and the CANcoder) was
   * never stated in the 2023 source -- it's a placeholder here and must be set from the actual
   * gearbox/CAD before this closed loop will behave correctly on hardware.
   */
  public static final class ArmConstants {
    private ArmConstants() {}

    public static final int motorId = 10;
    public static final int cancoderId = 13;
    public static final boolean motorInverted = true;

    // TODO: set from CAD/gearbox spec -- not captured in the 2023 source.
    public static final double rotorToSensorRatio = 100.0;

    public static final double statorCurrentLimitAmps = 50;
    public static final double motionMagicCruiseVelocityRotPerSec = 2.0;
    public static final double motionMagicAccelerationRotPerSecSq = 4.0;

    public static final double magnetOffsetRotations = 205.0 / 360.0;

    public static final Slot0Configs gains =
        new Slot0Configs().withKP(60.0).withKI(0).withKD(0).withKS(0.0).withKV(0.0).withKA(0.0);

    public static final double startingPositionRotations = 10.0 / 360.0;
    public static final double groundPickupPositionRotations = 82.5 / 360.0;
    public static final double chutePositionRotations = 35.0 / 360.0;
    public static final double shelfPositionRotations = 17.0 / 360.0;
    public static final double autoCarryPositionRotations = 38.5 / 360.0;
    public static final double midScorePositionRotations = 28.65 / 360.0;
    public static final double highScorePositionRotations = 57.21 / 360.0;
    public static final double topSuckPositionRotations = 70.9 / 360.0;
  }

  /**
   * Carried over from the 2023 robot's {@code Constants.WristConstants} / {@code
   * Constants.wristMotorID}/{@code wristCanCoderID}. Same CANcoder-in-real-degrees setup as the
   * arm -- see {@link ArmConstants} javadoc for the caveat on {@code rotorToSensorRatio}.
   */
  public static final class WristConstants {
    private WristConstants() {}

    public static final int motorId = 12;
    public static final int cancoderId = 14;

    // TODO: set from CAD/gearbox spec -- not captured in the 2023 source.
    public static final double rotorToSensorRatio = 100.0;

    public static final double statorCurrentLimitAmps = 20;
    public static final double motionMagicCruiseVelocityRotPerSec = 2.0;
    public static final double motionMagicAccelerationRotPerSecSq = 4.0;

    public static final double magnetOffsetRotations = 160.66 / 360.0;

    public static final Slot0Configs gains =
        new Slot0Configs().withKP(45.0).withKI(0).withKD(0).withKS(0.0).withKV(0.0).withKA(0.0);

    public static final double startingPositionRotations = 320.0 / 360.0;
    public static final double groundPickupPositionRotations = 275.0 / 360.0;
    public static final double babyBirdPositionRotations = 255.0 / 360.0;
    public static final double midScorePositionRotations = 345.76 / 360.0;
    public static final double highScorePositionRotations = 304.6 / 360.0;
    public static final double shelfPickupPositionRotations = 359.0 / 360.0;
    public static final double topSuckPositionRotations = 296.0 / 360.0;
    public static final double autoCarryPositionRotations = 322.0 / 360.0;
  }

  /** Carried over from the 2023 robot's {@code Constants.intakeMotorID} and {@code Intake.java}. */
  public static final class IntakeConstants {
    private IntakeConstants() {}

    public static final int motorId = 11;
    public static final double statorCurrentLimitAmps = 60;

    // 2023's cube/cone current-sensing threshold (Intake.cubeThreshold / coneThreshold).
    public static final double gamePieceCurrentThresholdAmps = 20;

    public static final double intakePercent = 0.9;
    public static final double intakeSlowPercent = 0.06;
    public static final double outtakePercent = 0.4;
    public static final double outtakeFullPercent = 1.0;
  }
}
