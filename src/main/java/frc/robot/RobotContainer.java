// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScoringCommands;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
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
  private final Vision vision;

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
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight-front", drive::getRotation),
                new VisionIOLimelight("limelight-back", drive::getRotation));
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
        // No camera simulation yet -- stub IOs keep the AdvantageKit logging structure intact.
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
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
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
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

    // Scoring positions
    operatorController.a().onTrue(ScoringCommands.stow(elevator, arm, wrist));
    operatorController.x().onTrue(ScoringCommands.prepScoreMid(elevator, arm, wrist));
    operatorController.y().onTrue(ScoringCommands.prepScoreHigh(elevator, arm, wrist));

    // Pickup sequences -- move to position and intake until a game piece is secured
    operatorController.b().onTrue(ScoringCommands.groundPickup(elevator, arm, wrist, intake));
    operatorController.leftBumper().onTrue(ScoringCommands.chutePickup(elevator, arm, wrist, intake));
    operatorController.rightBumper().onTrue(ScoringCommands.shelfPickup(elevator, arm, wrist, intake));

    // Release the held game piece at whatever scoring position is currently active
    operatorController.rightTrigger().onTrue(ScoringCommands.release(intake));
    // Manual intake override, for de-jamming or off-nominal pickups
    operatorController.leftTrigger().whileTrue(Commands.startEnd(intake::runIn, intake::stop, intake));

    // Held game-piece type indicator -- 2023's intake current thresholds never actually
    // distinguished cone vs. cube (both were 20A), so the operator declares it explicitly, same
    // as the original driver-facing behavior.
    operatorController.povUp().onTrue(Commands.runOnce(leds::setYellow)); // cone
    operatorController.povDown().onTrue(Commands.runOnce(leds::setPurple)); // cube
    operatorController.back().onTrue(Commands.runOnce(leds::off));
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
