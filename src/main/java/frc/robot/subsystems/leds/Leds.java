// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedsConstants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

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
 *
 * <p>The physical 2023 robot currently has its LEDs removed, so this subsystem is
 * dashboard-toggleable (default OFF, see {@link #enabled}) rather than always-on. Driving a PWM
 * port with nothing physically connected wouldn't crash anything on its own, but the toggle gives
 * the driver explicit control and keeps every color command a deliberate no-op rather than a
 * silently-ignored one while the hardware is absent.
 */
public class Leds extends SubsystemBase {
  private final LedsIO io;
  private final LedsIOInputsAutoLogged inputs = new LedsIOInputsAutoLogged();

  // Plain NT4 path (not "/SmartDashboard/...") so it shows up in Elastic without depending on the
  // legacy SmartDashboard/Shuffleboard Java APIs. Bind it to a toggle-switch widget in Elastic.
  private final LoggedNetworkBoolean enabled = new LoggedNetworkBoolean("/DriverDashboard/LedsEnabled", false);

  public Leds(LedsIO io) {
    this.io = io;
    io.setPwm(LedsConstants.blackPwm);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Leds", inputs);
    Logger.recordOutput("Leds/Enabled", enabled.get());
    if (!enabled.get()) {
      io.setPwm(LedsConstants.blackPwm);
    }
  }

  private void set(double pwmValue) {
    if (enabled.get()) {
      io.setPwm(pwmValue);
    }
  }

  public void setYellow() {
    set(LedsConstants.yellowPwm);
  }

  public void blinkPurple() {
    set(LedsConstants.blinkPurplePwm);
  }

  public void blinkYellow() {
    set(LedsConstants.blinkYellowPwm);
  }

  public void setPurple() {
    set(LedsConstants.purplePwm);
  }

  public void setGreen() {
    set(LedsConstants.greenPwm);
  }

  public void off() {
    set(LedsConstants.blackPwm);
  }
}
