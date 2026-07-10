// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

/** Sim IO for the LEDs -- just tracks the commanded PWM value for logging, no physical effect. */
public class LedsIOSim implements LedsIO {
  private double pwmValue = 0.0;

  @Override
  public void updateInputs(LedsIOInputs inputs) {
    inputs.pwmValue = pwmValue;
  }

  @Override
  public void setPwm(double value) {
    pwmValue = value;
  }
}
