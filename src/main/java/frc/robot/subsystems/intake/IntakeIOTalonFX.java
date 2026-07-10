// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;

/** Real hardware IO for the intake roller: a single TalonFX, run open-loop. */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX motor = new TalonFX(IntakeConstants.motorId);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0.0);

  private final StatusSignal<AngularVelocity> velocity = motor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = motor.getMotorVoltage();
  private final StatusSignal<Current> current = motor.getStatorCurrent();

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public IntakeIOTalonFX() {
    var config = new TalonFXConfiguration();
    // Brake mode keeps a held game piece pinched between the rollers instead of it potentially
    // rolling free the instant the intake stops being actively driven.
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Chosen so that IntakeConstants.intakePercent (positive) actually pulls a game piece in --
    // see Constants.IntakeConstants' comment on why the sign convention was flipped relative to
    // the 2023 source during the rewrite.
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.statorCurrentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, current);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(velocity, appliedVolts, current);

    inputs.connected = connectedDebounce.calculate(status.isOK());
    inputs.velocityRotPerSec = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double percentOutput) {
    motor.setControl(dutyCycleRequest.withOutput(percentOutput));
  }
}
