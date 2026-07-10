// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

public interface LedsIO {
  @AutoLog
  public static class LedsIOInputs {
    public double pwmValue = 0.0;
  }

  public default void updateInputs(LedsIOInputs inputs) {}

  /** Sets the Blinkin's PWM control signal directly (-1.0 to 1.0, per REV's pattern table). */
  public default void setPwm(double value) {}
}
