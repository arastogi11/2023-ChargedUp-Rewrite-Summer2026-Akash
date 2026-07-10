// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean connected = false;
    public double positionRotations = 0.0;
    public double velocityRotPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean atBottomLimit = false;
    public boolean atTopLimit = false;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run the elevator motor at the specified open-loop percent output. */
  public default void setOpenLoop(double percentOutput) {}

  /** Run the elevator to the specified position (in rotations) via Motion Magic. */
  public default void setPosition(double positionRotations) {}

  /** Zero the encoder at the current position. */
  public default void resetPosition() {}
}
