// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;

/**
 * Physics-sim IO for the elevator, backed by WPILib's {@link ElevatorSim}.
 *
 * <p>Like {@code ModuleIOSim}, this runs its own software closed loop (a {@link
 * ProfiledPIDController}, WPILib's equivalent of Motion Magic -- a PID controller wrapped around a
 * trapezoidal motion profile, same shape of behavior as the real motor's onboard Motion Magic) plus
 * an {@link ElevatorFeedforward} (the elevator-specific feedforward model: kS for static friction,
 * kG for the constant force needed just to hold the elevator up against gravity, kV for velocity)
 * since there's no real motor controller here to delegate the closed loop to.
 */
public class ElevatorIOSim implements ElevatorIO {
  // Placeholder mechanism geometry -- the 2023 robot's CAD (drum radius, carriage mass) wasn't
  // captured in its Constants.java, so these are reasonable stand-ins for a single-stage FRC
  // elevator; tune once real numbers are available.
  private static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.0);
  private static final double CARRIAGE_MASS_KG = 4.5;
  private static final double GEARING = 12.0;
  private static final double MIN_HEIGHT_METERS = 0.0;
  private static final double MAX_HEIGHT_METERS = 1.3;

  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          GEARING,
          CARRIAGE_MASS_KG,
          DRUM_RADIUS_METERS,
          MIN_HEIGHT_METERS,
          MAX_HEIGHT_METERS,
          true,
          MIN_HEIGHT_METERS);

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          40.0,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ElevatorConstants.motionMagicCruiseVelocityRotPerSec,
              ElevatorConstants.motionMagicAccelerationRotPerSecSq));
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 0.3, 4.0);

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  private double positionRotations() {
    return sim.getPositionMeters() / (2 * Math.PI * DRUM_RADIUS_METERS);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          controller.calculate(positionRotations()) + feedforward.calculate(controller.getSetpoint().velocity);
    }

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.connected = true;
    inputs.positionRotations = positionRotations();
    inputs.velocityRotPerSec = sim.getVelocityMetersPerSecond() / (2 * Math.PI * DRUM_RADIUS_METERS);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.atBottomLimit = sim.hasHitLowerLimit();
    inputs.atTopLimit = sim.hasHitUpperLimit();
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

  @Override
  public void resetPosition() {
    sim.setState(MIN_HEIGHT_METERS, 0.0);
  }
}
