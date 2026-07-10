// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.leds.LedsIO;
import frc.robot.subsystems.leds.LedsIOSim;
import frc.robot.subsystems.leds.LedsIOSpark;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOTalonFX;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 *
 * <p>Mechanisms, vision, and autonomous selection get wired in as later phases of the rewrite
 * land.
 */
public class RobotContainer {
  private final Drive drive;
  private final Elevator elevator;
  private final Arm arm;
  private final Wrist wrist;
  private final Intake intake;
  private final Leds leds;

  // Controllers (team now uses Xbox controllers, replacing the 2023 robot's raw Joysticks +
  // button board)
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        elevator = new Elevator(new ElevatorIOTalonFX());
        arm = new Arm(new ArmIOTalonFX());
        wrist = new Wrist(new WristIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());
        leds = new Leds(new LedsIOSpark());
      }
      case SIM -> {
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        elevator = new Elevator(new ElevatorIOSim());
        arm = new Arm(new ArmIOSim());
        wrist = new Wrist(new WristIOSim());
        intake = new Intake(new IntakeIOSim());
        leds = new Leds(new LedsIOSim());
      }
      default -> {
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        arm = new Arm(new ArmIO() {});
        wrist = new Wrist(new WristIO() {});
        intake = new Intake(new IntakeIO() {});
        leds = new Leds(new LedsIO() {});
      }
    }

    configureBindings();
  }

  /** Button -> command mappings go here. */
  private void configureBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    driverController.start().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Placeholder mechanism bindings -- expanded into full scoring sequences in a later phase.
    operatorController
        .a()
        .onTrue(
            Commands.runOnce(() -> elevator.setGoal(Elevator.Position.BOTTOM), elevator)
                .alongWith(Commands.runOnce(() -> arm.setGoal(Arm.Position.STARTING), arm))
                .alongWith(Commands.runOnce(() -> wrist.setGoal(Wrist.Position.STARTING), wrist)));
    operatorController
        .x()
        .onTrue(
            Commands.runOnce(() -> elevator.setGoal(Elevator.Position.MID), elevator)
                .alongWith(Commands.runOnce(() -> arm.setGoal(Arm.Position.MID_SCORE), arm))
                .alongWith(Commands.runOnce(() -> wrist.setGoal(Wrist.Position.MID_SCORE), wrist)));
    operatorController
        .y()
        .onTrue(
            Commands.runOnce(() -> elevator.setGoal(Elevator.Position.HIGH), elevator)
                .alongWith(Commands.runOnce(() -> arm.setGoal(Arm.Position.HIGH_SCORE), arm))
                .alongWith(
                    Commands.runOnce(() -> wrist.setGoal(Wrist.Position.HIGH_SCORE), wrist)));

    operatorController.rightTrigger().whileTrue(Commands.startEnd(intake::runIn, intake::stop, intake));
    operatorController.leftTrigger().whileTrue(Commands.startEnd(intake::runOut, intake::stop, intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
