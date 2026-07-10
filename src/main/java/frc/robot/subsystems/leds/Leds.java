// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedsConstants;
import org.littletonrobotics.junction.Logger;

/**
 * REV Blinkin LED controller. PWM values below are carried over unchanged from the 2023 robot --
 * they already match REV's documented Blinkin pattern table (e.g. 0.99 is "Solid Colors: Black",
 * i.e. off).
 *
 * <p>The 2023 robot's {@code turnOff()} didn't actually use that "black" PWM value -- it instead
 * cut power to a PDH switchable channel, while every color-setting method separately re-enabled
 * that same channel on every call. That's redundant (a Blinkin doesn't need power-cycling to
 * change its PWM signal) and inconsistent with {@code setBlack()}, which already sent the correct
 * "off" pattern over PWM. This version just uses the PWM "black" value for off, like every other
 * color, with no PDH involvement at all.
 */
public class Leds extends SubsystemBase {
  private final LedsIO io;
  private final LedsIOInputsAutoLogged inputs = new LedsIOInputsAutoLogged();

  public Leds(LedsIO io) {
    this.io = io;
    io.setPwm(LedsConstants.blackPwm);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Leds", inputs);
  }

  public void setYellow() {
    io.setPwm(LedsConstants.yellowPwm);
  }

  public void blinkPurple() {
    io.setPwm(LedsConstants.blinkPurplePwm);
  }

  public void blinkYellow() {
    io.setPwm(LedsConstants.blinkYellowPwm);
  }

  public void setPurple() {
    io.setPwm(LedsConstants.purplePwm);
  }

  public void setGreen() {
    io.setPwm(LedsConstants.greenPwm);
  }

  public void off() {
    io.setPwm(LedsConstants.blackPwm);
  }
}
