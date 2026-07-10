// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ArmConstants;

/**
 * Real hardware IO for the arm: TalonFX with a remote CANcoder as the feedback sensor (same
 * physical sensing setup as the 2023 robot), driven via a single Motion Magic Voltage request.
 * The 2023 robot configured Motion Magic gains on the TalonFX but never actually used them --
 * instead it drove the motor in PercentOutput with a separate WPILib PIDController computing the
 * output every loop. That duplicated control loop is the bug this rewrite fixes: there is exactly
 * one closed loop here, running onboard the TalonFX.
 */
public class ArmIOTalonFX implements ArmIO {
  private final TalonFX motor = new TalonFX(ArmConstants.motorId);
  private final CANcoder cancoder = new CANcoder(ArmConstants.cancoderId);

  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getStatorCurrent();

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ArmIOTalonFX() {
    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = -ArmConstants.magnetOffsetRotations;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        ArmConstants.motorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.Slot0 = ArmConstants.gains;
    config.Feedback.FeedbackRemoteSensorID = ArmConstants.cancoderId;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.RotorToSensorRatio = ArmConstants.rotorToSensorRatio;
    config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.motionMagicCruiseVelocityRotPerSec;
    config.MotionMagic.MotionMagicAcceleration = ArmConstants.motionMagicAccelerationRotPerSecSq;
    config.CurrentLimits.StatorCurrentLimit = ArmConstants.statorCurrentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);

    inputs.connected = connectedDebounce.calculate(status.isOK());
    inputs.positionRotations = position.getValueAsDouble();
    inputs.velocityRotPerSec = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double percentOutput) {
    motor.setControl(dutyCycleRequest.withOutput(percentOutput));
  }

  @Override
  public void setPosition(double positionRotations) {
    motor.setControl(positionRequest.withPosition(positionRotations));
  }
}
