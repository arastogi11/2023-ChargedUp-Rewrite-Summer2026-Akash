// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;

/**
 * Named scoring/pickup sequences for the 2023 game, combining elevator + arm + wrist + intake.
 *
 * <p>Flattened, factory-style replacement for the 2023 robot's deeply nested {@code
 * commands/Presets/}, {@code commands/Presets/Procedures/} class hierarchy -- one subclassed
 * {@code Command} per action there, one static factory method here.
 */
public final class ScoringCommands {
  private ScoringCommands() {}

  /**
   * Commands all three mechanisms to a named position simultaneously and returns immediately (each
   * {@code setGoal} call just hands a new Motion Magic target to the motor controller -- the motors
   * then move toward it on their own over the following loops, independent of this command, which
   * is why this can be a quick {@code Commands.parallel} of {@code runOnce}s rather than something
   * that waits for the mechanisms to actually arrive).
   */
  private static Command moveTo(
      Elevator elevator, Arm arm, Wrist wrist, Elevator.Position e, Arm.Position a, Wrist.Position w) {
    return Commands.parallel(
        Commands.runOnce(() -> elevator.setGoal(e), elevator),
        Commands.runOnce(() -> arm.setGoal(a), arm),
        Commands.runOnce(() -> wrist.setGoal(w), wrist));
  }

  /** Stow position for driving around with a held game piece. */
  public static Command stow(Elevator elevator, Arm arm, Wrist wrist) {
    return moveTo(
        elevator, arm, wrist, Elevator.Position.BOTTOM, Arm.Position.STARTING, Wrist.Position.STARTING);
  }

  /** Move to the mid scoring position and hold (release is a separate, driver-triggered step). */
  public static Command prepScoreMid(Elevator elevator, Arm arm, Wrist wrist) {
    return moveTo(
        elevator, arm, wrist, Elevator.Position.MID, Arm.Position.MID_SCORE, Wrist.Position.MID_SCORE);
  }

  /** Move to the high scoring position and hold. */
  public static Command prepScoreHigh(Elevator elevator, Arm arm, Wrist wrist) {
    return moveTo(
        elevator, arm, wrist, Elevator.Position.HIGH, Arm.Position.HIGH_SCORE, Wrist.Position.HIGH_SCORE);
  }

  /**
   * Move to ground-pickup position and run the intake until a game piece is secured.
   *
   * <p>The shape every pickup command below follows: {@code moveTo(...)} (returns instantly, see
   * its javadoc) {@code .andThen(...)} a command that keeps running the intake until {@code
   * gamePieceSecured()} goes true, then {@code .finallyDo(intake::stop)} to guarantee the rollers
   * stop -- {@code finallyDo} runs whether the command finished normally (piece secured) or was
   * interrupted early (e.g. the driver switches to a different action mid-pickup), so the intake
   * never gets left running by accident.
   */
  public static Command groundPickup(Elevator elevator, Arm arm, Wrist wrist, Intake intake) {
    return moveTo(
            elevator,
            arm,
            wrist,
            Elevator.Position.BOTTOM,
            Arm.Position.GROUND_PICKUP,
            Wrist.Position.GROUND_PICKUP)
        .andThen(Commands.run(intake::runIn, intake).until(intake::gamePieceSecured))
        .finallyDo(intake::stop);
  }

  /** Move to double-substation chute-pickup position and run the intake until secured. */
  public static Command chutePickup(Elevator elevator, Arm arm, Wrist wrist, Intake intake) {
    return moveTo(
            elevator, arm, wrist, Elevator.Position.SHELF, Arm.Position.CHUTE, Wrist.Position.AUTO_CARRY)
        .andThen(Commands.run(intake::runIn, intake).until(intake::gamePieceSecured))
        .finallyDo(intake::stop);
  }

  /** Move to single-substation shelf-pickup position and run the intake until secured. */
  public static Command shelfPickup(Elevator elevator, Arm arm, Wrist wrist, Intake intake) {
    return moveTo(
            elevator,
            arm,
            wrist,
            Elevator.Position.SHELF,
            Arm.Position.SHELF,
            Wrist.Position.SHELF_PICKUP)
        .andThen(Commands.run(intake::runIn, intake).until(intake::gamePieceSecured))
        .finallyDo(intake::stop);
  }

  /** Releases the held game piece (run the rollers outward briefly). */
  public static Command release(Intake intake) {
    return Commands.run(intake::runOut, intake).withTimeout(0.5).andThen(Commands.runOnce(intake::stop, intake));
  }
}
