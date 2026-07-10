// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware-agnostic interface for the Blinkin LED controller -- see {@code
 * frc.robot.subsystems.drive.ModuleIO}'s javadoc for the full explanation of the IO-interface
 * pattern. Much simpler than the other IO interfaces since a REV Blinkin has exactly one control
 * input (a single PWM signal) and reports nothing back -- there's no sensor to read, only a value
 * to set, so {@code LedsIOInputs} exists purely so the currently-commanded PWM value shows up in
 * logs/replay like everything else.
 */
public interface LedsIO {
  @AutoLog
  public static class LedsIOInputs {
    public double pwmValue = 0.0;
  }

  public default void updateInputs(LedsIOInputs inputs) {}

  /** Sets the Blinkin's PWM control signal directly (-1.0 to 1.0, per REV's pattern table). */
  public default void setPwm(double value) {}
}
