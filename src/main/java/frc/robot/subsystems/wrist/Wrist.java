// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.util.LoggingControl;
import org.littletonrobotics.junction.Logger;

/**
 * The wrist subsystem: a single-jointed pivot mounted at the end of the arm that angles the
 * intake to line up with the game piece or scoring target. See {@code ArmIOTalonFX}'s javadoc for
 * the control-loop bug this rewrite fixes (same fix applies here).
 */
public class Wrist extends SubsystemBase {
  /** Named wrist angles for each pickup/scoring/carry position. */
  public enum Position {
    STARTING(WristConstants.startingPositionRotations),
    GROUND_PICKUP(WristConstants.groundPickupPositionRotations),
    BABY_BIRD(WristConstants.babyBirdPositionRotations),
    MID_SCORE(WristConstants.midScorePositionRotations),
    HIGH_SCORE(WristConstants.highScorePositionRotations),
    SHELF_PICKUP(WristConstants.shelfPickupPositionRotations),
    TOP_SUCK(WristConstants.topSuckPositionRotations),
    AUTO_CARRY(WristConstants.autoCarryPositionRotations);

    final double rotations;

    Position(double rotations) {
      this.rotations = rotations;
    }
  }

  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // updateInputs always runs; recording is skippable -- see LoggingControl's javadoc.
    io.updateInputs(inputs);
    if (LoggingControl.enabled()) {
      Logger.processInputs("Wrist", inputs);
    }
  }

  public void setGoal(Position goal) {
    io.setPosition(goal.rotations);
  }

  public void setOpenLoop(double percentOutput) {
    io.setOpenLoop(percentOutput);
  }

  public double getPositionRotations() {
    return inputs.positionRotations;
  }
}
