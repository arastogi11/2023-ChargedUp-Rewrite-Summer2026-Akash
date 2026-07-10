// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * A small singleton (a class with exactly one shared instance, accessed via {@link
 * #getInstance()} instead of being passed around everywhere) that holds a couple of values other
 * parts of the robot need to read without being directly wired to the subsystem that produces
 * them.
 *
 * <p>Right now that's just the robot's current field pose (written by {@link
 * frc.robot.subsystems.drive.Drive#periodic()} every loop, since {@code Drive} is the one
 * subsystem that actually knows where the robot is) and the current alliance color. Keep this
 * class small and deliberate -- it's convenient, but overusing shared global state instead of
 * passing values through constructors/suppliers makes a codebase much harder to reason about as
 * it grows.
 */
public class RobotState {
  // The single shared instance. Starts null and is created the first time getInstance() is
  // called (this pattern is called "lazy initialization").
  private static RobotState instance;

  /** Returns the one shared RobotState instance, creating it on first use. */
  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  // Private constructor -- the only way to get a RobotState is through getInstance() above,
  // which guarantees there's ever only one.
  private RobotState() {}

  /** The robot's current estimated field pose, updated every loop by {@code Drive}. */
  public Pose2d robotPose = new Pose2d();

  /** The current alliance color, if known (null until the driver station reports one). */
  public Alliance alliance;
}
