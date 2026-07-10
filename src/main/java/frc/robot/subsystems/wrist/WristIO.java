// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware-agnostic interface for the wrist -- see {@code
 * frc.robot.subsystems.drive.ModuleIO}'s javadoc for the full explanation of the IO-interface
 * pattern.
 */
public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean connected = false;
    public double positionRotations = 0.0;
    public double velocityRotPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  /** Run the wrist motor at the specified open-loop percent output. */
  public default void setOpenLoop(double percentOutput) {}

  /** Run the wrist to the specified position (in rotations) via Motion Magic. */
  public default void setPosition(double positionRotations) {}
}
