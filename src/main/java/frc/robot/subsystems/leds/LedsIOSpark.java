// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.LedsConstants;

/** Real hardware IO for a REV Blinkin LED controller driven over PWM. */
public class LedsIOSpark implements LedsIO {
  private final Spark leds = new Spark(LedsConstants.pwmPort);
  private double pwmValue = 0.0;

  @Override
  public void updateInputs(LedsIOInputs inputs) {
    inputs.pwmValue = pwmValue;
  }

  @Override
  public void setPwm(double value) {
    pwmValue = value;
    leds.set(value);
  }
}
