// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ArmConstants;

/**
 * Physics-sim IO for the arm, backed by WPILib's {@link SingleJointedArmSim} -- models a rigid arm
 * pivoting around a single motor-driven joint, including the effect of gravity torque changing
 * with angle (which is why this needs an {@link ArmFeedforward}, with its own gravity term,
 * instead of the simpler {@code ElevatorFeedforward} used for the linear-motion elevator). Same
 * software-closed-loop-instead-of-real-onboard-loop idea as {@code ElevatorIOSim} -- see its
 * javadoc.
 */
public class ArmIOSim implements ArmIO {
  // Placeholder mechanism geometry -- arm length/moment of inertia weren't captured in the 2023
  // robot's Constants.java; reasonable stand-ins for a single-jointed FRC scoring arm.
  private static final double ARM_LENGTH_METERS = Units.inchesToMeters(24);
  private static final double ARM_MASS_KG = 4.0;

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          ArmConstants.rotorToSensorRatio,
          SingleJointedArmSim.estimateMOI(ARM_LENGTH_METERS, ARM_MASS_KG),
          ARM_LENGTH_METERS,
          0.0,
          Units.rotationsToRadians(1.0),
          true,
          0.0);

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          30.0,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ArmConstants.motionMagicCruiseVelocityRotPerSec,
              ArmConstants.motionMagicAccelerationRotPerSecSq));
  private final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.4, 1.5);

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ArmIOInputs inputs) {
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
