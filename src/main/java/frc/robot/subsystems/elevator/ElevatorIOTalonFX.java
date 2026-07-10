// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;

/**
 * Real hardware IO for the elevator: a single TalonFX using its integrated rotor sensor, driven
 * via a single Motion Magic Voltage request (Slot0 gains) -- unlike the 2023 robot, there is no
 * separate WPILib PIDController wrapping the motor's own closed loop.
 */
public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX motor = new TalonFX(ElevatorConstants.motorId);
  private final DigitalInput bottomLimit = new DigitalInput(ElevatorConstants.bottomLimitPort);
  private final DigitalInput topLimit = new DigitalInput(ElevatorConstants.topLimitPort);

  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

  private final StatusSignal<Angle> position = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getStatorCurrent();

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ElevatorIOTalonFX() {
    var config = new TalonFXConfiguration();
    // Brake mode so the elevator doesn't free-fall under gravity the instant it's disabled.
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = ElevatorConstants.gains;
    config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.motionMagicCruiseVelocityRotPerSec;
    config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.motionMagicAccelerationRotPerSecSq;
    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.statorCurrentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    // No high-frequency odometry needed here (that's a drivetrain-only concern -- position
    // tracking for pose estimation), so a plain 50Hz update rate for every signal is enough.
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);

    inputs.connected = connectedDebounce.calculate(status.isOK());
    inputs.positionRotations = position.getValueAsDouble();
    inputs.velocityRotPerSec = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    // 2023's limit switches were wired active-low (DigitalInput.get() == false at the limit).
    inputs.atBottomLimit = !bottomLimit.get();
    inputs.atTopLimit = !topLimit.get();
  }

  @Override
  public void setOpenLoop(double percentOutput) {
    motor.setControl(dutyCycleRequest.withOutput(percentOutput));
  }

  @Override
  public void setPosition(double positionRotations) {
    motor.setControl(positionRequest.withPosition(positionRotations));
  }

  @Override
  public void resetPosition() {
    tryUntilOk(5, () -> motor.setPosition(0.0, 0.25));
  }
}
