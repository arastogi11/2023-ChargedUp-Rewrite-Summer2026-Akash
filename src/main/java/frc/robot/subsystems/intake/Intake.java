// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

/**
 * The intake subsystem: a simple roller mechanism (no closed-loop position control) that grabs
 * and releases game pieces. "Detecting" a held game piece is done crudely, via stator current --
 * see {@link #gamePieceSecured()}.
 */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void runIn() {
    io.setOpenLoop(IntakeConstants.intakePercent);
  }

  public void runInSlow() {
    io.setOpenLoop(IntakeConstants.intakeSlowPercent);
  }

  public void runOut() {
    io.setOpenLoop(IntakeConstants.outtakePercent);
  }

  public void runOutFull() {
    io.setOpenLoop(IntakeConstants.outtakeFullPercent);
  }

  public void stop() {
    io.setOpenLoop(0.0);
  }

  /** True once stator current exceeds the game-piece-secured threshold. */
  public boolean gamePieceSecured() {
    return inputs.currentAmps >= IntakeConstants.gamePieceCurrentThresholdAmps;
  }
}
