// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The JVM's actual entry point (this is the class whose {@code main} method the roboRIO's JRE
 * calls when the robot code starts). It exists only to hand control to {@link RobotBase}, which
 * then constructs {@link Robot} and drives the mode-based callback loop (disabledInit,
 * autonomousPeriodic, etc.) -- all the actual robot logic lives in Robot.java and beyond, never
 * here.
 *
 * <p>Do NOT add any static variables to this class, or any initialization at all. Unless you know
 * what you are doing, do not modify this file except to change the parameter class to the
 * startRobot call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
