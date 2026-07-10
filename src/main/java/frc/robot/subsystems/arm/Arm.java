// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.Logger;

/**
 * The arm subsystem: a single-jointed pivoting arm that extends the intake out from the robot for
 * pickups and scoring. See {@code ArmIOTalonFX}'s javadoc for the control-loop bug this rewrite
 * fixes relative to the 2023 robot.
 */
public class Arm extends SubsystemBase {
  /** Named arm angles for each pickup/scoring/carry position. */
  public enum Position {
    STARTING(ArmConstants.startingPositionRotations),
    GROUND_PICKUP(ArmConstants.groundPickupPositionRotations),
    CHUTE(ArmConstants.chutePositionRotations),
    SHELF(ArmConstants.shelfPositionRotations),
    AUTO_CARRY(ArmConstants.autoCarryPositionRotations),
    MID_SCORE(ArmConstants.midScorePositionRotations),
    HIGH_SCORE(ArmConstants.highScorePositionRotations),
    TOP_SUCK(ArmConstants.topSuckPositionRotations);

    final double rotations;

    Position(double rotations) {
      this.rotations = rotations;
    }
  }

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public void setGoal(Position goal) {
    io.setPosition(goal.rotations);
  }

  public void setOpenLoop(double percentOutput) {
    io.setOpenLoop(percentOutput);
  }

  public double getPositionRotations() {
    return inputs.positionRotations;
  }
}
