// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public enum Position {
    BOTTOM(ElevatorConstants.bottomPositionRotations),
    MID(ElevatorConstants.midPositionRotations),
    HIGH(ElevatorConstants.highPositionRotations),
    SHELF(ElevatorConstants.shelfPositionRotations);

    final double rotations;

    Position(double rotations) {
      this.rotations = rotations;
    }
  }

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Re-zero automatically at the bottom hard stop, same behavior as the 2023 robot.
    if (inputs.atBottomLimit) {
      io.resetPosition();
    }
  }

  public void setGoal(Position goal) {
    io.setPosition(goal.rotations);
  }

  public void setOpenLoop(double percentOutput) {
    io.setOpenLoop(percentOutput);
  }

  public boolean atBottomLimit() {
    return inputs.atBottomLimit;
  }

  public boolean atTopLimit() {
    return inputs.atTopLimit;
  }
}
