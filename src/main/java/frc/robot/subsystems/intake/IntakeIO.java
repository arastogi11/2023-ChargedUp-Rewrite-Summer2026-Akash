// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware-agnostic interface for the intake roller -- see {@code
 * frc.robot.subsystems.drive.ModuleIO}'s javadoc for the full explanation of the IO-interface
 * pattern. Unlike the other mechanisms, the intake has no closed-loop position control (it's just
 * a roller run open-loop in or out) and no CANcoder -- {@code currentAmps} below doubles as its
 * only "sensor," used by {@code Intake.gamePieceSecured()} to detect a held game piece.
 */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = false;
    public double velocityRotPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the intake roller at the specified open-loop percent output. */
  public default void setOpenLoop(double percentOutput) {}
}
