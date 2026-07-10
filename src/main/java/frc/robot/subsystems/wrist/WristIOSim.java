// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.WristConstants;

/** Physics-sim IO for the wrist, backed by WPILib's SingleJointedArmSim. */
public class WristIOSim implements WristIO {
  // Placeholder mechanism geometry -- not captured in the 2023 robot's Constants.java.
  private static final double WRIST_LENGTH_METERS = Units.inchesToMeters(10);
  private static final double WRIST_MASS_KG = 1.5;

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          WristConstants.rotorToSensorRatio,
          SingleJointedArmSim.estimateMOI(WRIST_LENGTH_METERS, WRIST_MASS_KG),
          WRIST_LENGTH_METERS,
          0.0,
          Units.rotationsToRadians(1.0),
          true,
          0.0);

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          25.0,
          0,
          0,
          new TrapezoidProfile.Constraints(
              WristConstants.motionMagicCruiseVelocityRotPerSec,
              WristConstants.motionMagicAccelerationRotPerSecSq));
  private final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.2, 0.8);

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(WristIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          controller.calculate(Units.radiansToRotations(sim.getAngleRads()))
              + feedforward.calculate(
                  Units.rotationsToRadians(controller.getSetpoint().position),
                  Units.rotationsToRadians(controller.getSetpoint().velocity));
    }

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.connected = true;
    inputs.positionRotations = Units.radiansToRotations(sim.getAngleRads());
    inputs.velocityRotPerSec = Units.radiansToRotations(sim.getVelocityRadPerSec());
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double percentOutput) {
    closedLoop = false;
    appliedVolts = percentOutput * 12.0;
  }

  @Override
  public void setPosition(double positionRotations) {
    closedLoop = true;
    controller.setGoal(positionRotations);
  }
}
