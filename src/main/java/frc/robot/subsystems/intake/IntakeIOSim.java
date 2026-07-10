// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics-sim IO for the intake roller, backed by WPILib's {@link DCMotorSim} -- a simpler model
 * than the elevator/arm/wrist's sims since a roller has no gravity/position dependence, just
 * inertia. There's no simulated "detects a game piece" behavior, since the real detection
 * ({@code Intake.gamePieceSecured()}) works off stall current, which this sim doesn't model (a
 * simulated roller never actually stalls against anything).
 */
public class IntakeIOSim implements IntakeIO {
  private static final double GEARING = 3.0;
  private static final double MOI_KG_M2 = 0.001;

  private final DCMotorSim sim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), MOI_KG_M2, GEARING),
          DCMotor.getKrakenX60Foc(1));

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.connected = true;
    inputs.velocityRotPerSec = Units.radiansToRotations(sim.getAngularVelocityRadPerSec());
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double percentOutput) {
    appliedVolts = percentOutput * 12.0;
  }
}
