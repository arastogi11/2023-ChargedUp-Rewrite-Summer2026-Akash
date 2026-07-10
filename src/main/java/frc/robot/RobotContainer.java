// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 *
 * <p>Every subsystem below is constructed with a different {@code XyzIO} implementation depending
 * on {@link Constants#currentMode} -- the subsystem class itself (e.g. {@code Elevator}) never
 * knows or cares whether it's talking to a real TalonFX, a physics simulation, or nothing at all
 * (REPLAY). That indirection is the whole point of the IO-interface pattern used throughout {@code
 * subsystems/}: it's what lets the exact same command-based robot code run unmodified in three
 * very different situations.
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
  // button board). Port numbers here (0, 1) must match the order the controllers are plugged in
  // on the Driver Station's USB tab.
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Populated in the constructor with every autonomous routine the driver can pick from on the
  // dashboard, plus a handful of drivetrain characterization routines (see below).
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    // Build every subsystem with the IO implementation appropriate for how the code is currently
    // running. This switch is the ONLY place in the whole codebase that branches on
    // Constants.currentMode to pick hardware vs. sim vs. nothing -- everywhere else (commands,
    // RobotContainer's own button bindings, subsystem periodic() methods) is written against the
    // subsystem's public API and doesn't know or care which IO backs it.
    switch (Constants.currentMode) {
      case REAL -> {
        // Real robot: real hardware IO implementations, one per motor controller/sensor.
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
        // Simulation: same subsystem classes, but every IO is a physics model (ElevatorSim,
        // SingleJointedArmSim, etc.) instead of real motor controller calls, so the exact same
        // command-based code can be exercised and tuned before ever touching real hardware.
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
        // REPLAY: `new XyzIO() {}` constructs an anonymous instance of the interface using only
        // its no-op default methods -- no hardware calls, no sim math, nothing. That's correct
        // here because in replay every recorded @AutoLog input is fed back in from the saved log
        // file instead of being produced live, so nothing should be generating new values.
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

    registerNamedCommands();

    // Auto-populates from any .auto files authored in the PathPlanner GUI under
    // src/main/deploy/pathplanner/autos/ -- none exist yet, so this starts empty ("None" only)
    // until paths/autos are drawn in the PathPlanner app against the team's actual field
    // strategy, which isn't something to fabricate here.
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // SysId ("System Identification") drives the robot through a scripted quasistatic (slow,
    // near-constant velocity) then dynamic (step voltage) test and logs the response, which
    // WPILib's SysId analysis tool can turn into real kS/kV/kA feedforward gains -- run these from
    // the dashboard once on a real robot to replace the placeholder gains flagged in
    // generated/TunerConstants.java. Wheel radius characterization spins the robot in a circle to
    // solve for the actual effective wheel radius vs. the value currently guessed in
    // TunerConstants.
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));

    configureBindings();
  }

  /**
   * Registers named commands PathPlanner-authored autos can trigger via event markers -- i.e. once
   * a path is drawn in the PathPlanner GUI, you can drop an event marker on it named e.g.
   * "GroundPickup" and it'll run the exact command registered here under that name, with no extra
   * code needed per-auto.
   */
  private void registerNamedCommands() {
    NamedCommands.registerCommand("Stow", ScoringCommands.stow(elevator, arm, wrist));
    NamedCommands.registerCommand("PrepScoreMid", ScoringCommands.prepScoreMid(elevator, arm, wrist));
    NamedCommands.registerCommand("PrepScoreHigh", ScoringCommands.prepScoreHigh(elevator, arm, wrist));
    NamedCommands.registerCommand("GroundPickup", ScoringCommands.groundPickup(elevator, arm, wrist, intake));
    NamedCommands.registerCommand("ChutePickup", ScoringCommands.chutePickup(elevator, arm, wrist, intake));
    NamedCommands.registerCommand("ShelfPickup", ScoringCommands.shelfPickup(elevator, arm, wrist, intake));
    NamedCommands.registerCommand("Release", ScoringCommands.release(intake));
  }

  /**
   * Button -> command mappings go here.
   *
   * <p>Driver controller: left stick translates, right stick X rotates (both field-relative), Start
   * button X-locks the wheels to resist being pushed. Right trigger is a continuous precision/slow
   * mode -- see the comment below.
   *
   * <p>Operator controller: A/X/Y move to stow/mid-score/high-score position. B and the two
   * bumpers run the three pickup sequences (ground/chute/shelf), each of which drives the intake
   * until a game piece is detected. Right trigger releases the held piece; left trigger is a manual
   * intake override for de-jamming. D-pad up/down and Back set the LED cone/cube/off indicator.
   */
  private void configureBindings() {
    // Right trigger scales speed down continuously for fine positioning (e.g. lining up to
    // score) -- the Xbox-native equivalent of the 2023 robot's Ruffy joysticks, each of which had
    // a twist axis providing fine translation/strafe control at a fixed 20% speed whenever the
    // main stick was centered. Released = full speed, fully pressed = slowest (see
    // DriveCommands.PRECISION_MIN_SCALAR).
    //
    // setDefaultCommand means this command runs continuously any time nothing else that requires
    // the `drive` subsystem is scheduled -- exactly the behavior you want for "drive around with
    // the sticks" as a background activity that specific button-triggered commands can interrupt.
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> 1.0 - driverController.getRightTriggerAxis()));

    // onTrue fires the command once, the instant the button transitions from unpressed to pressed
    // (as opposed to whileTrue below, which restarts/continues running for as long as it's held).
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
    // Manual intake override, for de-jamming or off-nominal pickups. whileTrue means this only
    // runs while the trigger is physically held down, and Commands.startEnd wires runIn as the
    // "start" action and stop as the "end" action run when the trigger is released.
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
    return autoChooser.get();
  }
}
