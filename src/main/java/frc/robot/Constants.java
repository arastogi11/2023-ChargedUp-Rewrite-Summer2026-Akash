// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Robot-wide numerical/boolean constants live here, grouped into a nested class per subsystem.
 * Do not put anything functional in this class -- no motor objects, no logic, just numbers and
 * flags that other classes read.
 */
public final class Constants {
  private Constants() {}

  // What mode to run in when we're NOT on a real robot (RobotBase.isReal() is false). Almost
  // always SIM; switch this to Mode.REPLAY only temporarily, while replaying a saved log.
  public static final Mode simMode = Mode.SIM;

  // The actual mode used everywhere else in the codebase. RobotBase.isReal() is true only when
  // the code is actually running on a roboRIO -- so this line means "REAL on the robot, otherwise
  // whatever simMode says." Every subsystem's constructor in RobotContainer switches on this to
  // decide whether to build real-hardware IO or simulated IO.
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

    // CAN ID of the elevator's single TalonFX (Kraken/Falcon) motor controller.
    public static final int motorId = 9;
    // roboRIO DIO port numbers for the two hard-stop limit switches.
    public static final int bottomLimitPort = 0;
    public static final int topLimitPort = 1;

    // Stator current limit in amps -- the motor controller will clamp output to avoid drawing
    // more than this from the battery through the motor windings, protecting the motor/breaker.
    public static final double statorCurrentLimitAmps = 60;
    // Motion Magic is Phoenix6's built-in trapezoidal motion profiler: instead of jumping straight
    // to a target and letting the PID loop fight the resulting huge error, it ramps velocity up to
    // this cruise speed, holds it, then ramps down as it approaches the goal. Units here are
    // rotations/sec (cruise) and rotations/sec^2 (acceleration) of the mechanism (post-gearbox).
    public static final double motionMagicCruiseVelocityRotPerSec = 20;
    public static final double motionMagicAccelerationRotPerSecSq = 40;

    // Slot0Configs holds the closed-loop gains Phoenix6 uses onboard the motor controller itself
    // (not in robot code) to drive toward a Motion Magic setpoint: kP/kI/kD are standard PID terms
    // (proportional/integral/derivative -- kP alone is often enough for a well-tuned mechanism),
    // and kS/kV/kA are feedforward terms (kS = voltage to overcome static friction before the
    // mechanism starts moving at all, kV = voltage per unit velocity, kA = voltage per unit
    // acceleration) that let the controller anticipate the needed output instead of only reacting
    // to error after the fact. All zero here except kP -- untuned, a real starting point for
    // hardware tuning, not finished values.
    public static final Slot0Configs gains =
        new Slot0Configs().withKP(4.0).withKI(0).withKD(0).withKS(0.0).withKV(0.0).withKA(0.0);

    // Placeholder rotation setpoints -- see class javadoc. These four values are what
    // Elevator.Position.BOTTOM/MID/HIGH/SHELF resolve to.
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
    // CAN ID of the CANcoder magnetic absolute encoder mounted on the arm's output shaft, used as
    // the TalonFX's *remote* feedback sensor instead of its own internal rotor sensor -- see
    // ArmIOTalonFX's javadoc for why that matters (it reads real arm angle directly, immune to
    // belt/chain slip that would throw off a rotor-based position estimate).
    public static final int cancoderId = 13;
    public static final boolean motorInverted = true;

    // How many rotor rotations correspond to one rotation of the CANcoder (i.e. the gearbox ratio
    // between the motor and the arm's output shaft). TODO: set from CAD/gearbox spec -- not
    // captured in the 2023 source, so this is a guess, not a measured value.
    public static final double rotorToSensorRatio = 100.0;

    public static final double statorCurrentLimitAmps = 50;
    // See ElevatorConstants above for what Motion Magic cruise/acceleration and Slot0Configs mean.
    public static final double motionMagicCruiseVelocityRotPerSec = 2.0;
    public static final double motionMagicAccelerationRotPerSecSq = 4.0;

    // CANcoder "magnet offset": the raw magnet reading doesn't start at zero at the arm's
    // mechanical zero point, so this offset (2023's ArmConstants.armOffset, in degrees here
    // converted to rotations) gets subtracted in the encoder config to make position readings
    // physically meaningful (0 = the arm's defined zero position).
    public static final double magnetOffsetRotations = 205.0 / 360.0;

    public static final Slot0Configs gains =
        new Slot0Configs().withKP(60.0).withKI(0).withKD(0).withKS(0.0).withKV(0.0).withKA(0.0);

    // Named setpoints, all originally specified in degrees (0-360) by the 2023 code and converted
    // here to rotations (Phoenix6's native unit) by dividing by 360.
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

    // See ArmConstants.magnetOffsetRotations above for what this is.
    public static final double magnetOffsetRotations = 160.66 / 360.0;

    public static final Slot0Configs gains =
        new Slot0Configs().withKP(45.0).withKI(0).withKD(0).withKS(0.0).withKV(0.0).withKA(0.0);

    // Named setpoints, converted from the 2023 code's degree values to rotations (/360).
    public static final double startingPositionRotations = 320.0 / 360.0;
    public static final double groundPickupPositionRotations = 275.0 / 360.0;
    public static final double babyBirdPositionRotations = 255.0 / 360.0;
    public static final double midScorePositionRotations = 345.76 / 360.0;
    public static final double highScorePositionRotations = 304.6 / 360.0;
    public static final double shelfPickupPositionRotations = 359.0 / 360.0;
    public static final double topSuckPositionRotations = 296.0 / 360.0;
    public static final double autoCarryPositionRotations = 322.0 / 360.0;
  }

  /**
   * Carried over from the 2023 robot's {@code Constants.LEDConstants} / {@code
   * BlinkinLEDs.java}. See {@link frc.robot.subsystems.leds.Leds} javadoc for why {@code
   * turnOff()} no longer touches the PDH.
   */
  public static final class LedsConstants {
    private LedsConstants() {}

    // roboRIO PWM port the Blinkin LED controller is wired to.
    public static final int pwmPort = 0;

    // REV Blinkin controllers pick a color/pattern based purely on what PWM signal (-1.0 to 1.0)
    // you send them -- there's no separate "set color" API, just these magic numbers from REV's
    // published pattern table. All six values below are carried over unchanged from 2023.
    public static final double blackPwm = 0.99; // "Solid Colors: Black" -- i.e. off
    public static final double yellowPwm = 0.69; // solid yellow (cone indicator)
    public static final double purplePwm = 0.91; // solid violet/purple (cube indicator)
    public static final double greenPwm = -0.05; // a pattern in Blinkin's negative/dynamic range
    public static final double blinkPurplePwm = 0.15; // a pattern in Blinkin's positive range
    public static final double blinkYellowPwm = -0.07; // a pattern in Blinkin's negative range
  }

  /** Carried over from the 2023 robot's {@code Constants.intakeMotorID} and {@code Intake.java}. */
  public static final class IntakeConstants {
    private IntakeConstants() {}

    public static final int motorId = 11;
    public static final double statorCurrentLimitAmps = 60;

    // How much stator current (amps) the roller draws once it's gripped a game piece and stalled
    // against it -- used as a crude "do we have something?" sensor instead of a dedicated beam
    // break, carried over from 2023's Intake.cubeThreshold/coneThreshold (which were identical
    // values, so the 2023 code never actually distinguished cone vs. cube by current either).
    public static final double gamePieceCurrentThresholdAmps = 20;

    // Open-loop percent-output speeds (-1.0 to 1.0) for each named intake action.
    public static final double intakePercent = 0.9;
    public static final double intakeSlowPercent = 0.06;
    public static final double outtakePercent = 0.4;
    public static final double outtakeFullPercent = 1.0;
  }
}
