// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

/**
 * The elevator subsystem: a single linear-motion mechanism the arm/wrist/intake ride on, used to
 * change scoring height. Written entirely against the {@link ElevatorIO} interface, so it works
 * identically whether {@code io} is talking to real hardware or a physics simulation -- see {@code
 * frc.robot.subsystems.drive.ModuleIO}'s javadoc for the full explanation of that pattern.
 */
public class Elevator extends SubsystemBase {
  /** Named heights the elevator can be commanded to, each backed by a tuned rotation count. */
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
  // AdvantageKit's annotation processor generates this class at build time from
  // ElevatorIO.ElevatorIOInputs (the @AutoLog-annotated class) -- it's an automatically-loggable,
  // automatically-replayable version of the plain inputs struct. You won't find
  // ElevatorIOInputsAutoLogged.java anywhere in this source tree; it's generated fresh on every
  // build.
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
