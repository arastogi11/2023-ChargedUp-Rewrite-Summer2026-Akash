// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding
 * to each mode, as described in the TimedRobot documentation.
 *
 * <p>This class extends AdvantageKit's {@code LoggedRobot} instead of WPILib's plain {@code
 * TimedRobot}. {@code LoggedRobot} is a drop-in replacement that, on every 20ms loop, timestamps
 * and records every sensor input the robot reads (through the {@code XyzIO}/{@code
 * updateInputs(...)} pattern used throughout {@code subsystems/}) into a log file. Because every
 * input is captured, a saved log can later be "replayed": re-run through the exact same robot
 * code with the exact same sensor readings, reproducing exactly what the robot did without needing
 * the physical robot present. That's what {@code Constants.Mode.REPLAY} below is for -- it's
 * enormously useful for debugging something that happened at a competition after the fact.
 */
public class Robot extends LoggedRobot {
  // The currently-running autonomous command, so teleopInit() can cancel it if it's still going.
  private Command autonomousCommand;

  // Owns every subsystem and defines all button bindings -- see RobotContainer.java.
  private RobotContainer robotContainer;

  public Robot() {
    // Tag every log file with build/git info, so months from now you can tell exactly which
    // version of the code produced a given log. BuildConstants.java is generated automatically
    // at build time (see the `gversion` plugin in build.gradle) -- don't edit it by hand, and
    // don't worry that it's missing from git (it's in .gitignore since it's regenerated every
    // build).
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Decide where log data goes, based on which of the three modes we're running in
    // (Constants.currentMode picks this automatically -- REAL when deployed to an actual roboRIO,
    // SIM otherwise, unless you've manually forced REPLAY to play back a saved log).
    switch (Constants.currentMode) {
      case REAL:
        // On the real robot: write a .wpilog file to a USB stick plugged into the roboRIO ("/U/"
        // is the roboRIO's mount point for it) for permanent storage, AND publish live over
        // NetworkTables (NT4Publisher) so you can watch values in real time in AdvantageScope or
        // Elastic while the robot is running.
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // In simulation there's no USB stick, so just publish over NT for live viewing.
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replay mode: instead of reading real sensors, feed the logger a previously-recorded
        // .wpilog file as its input source. The robot code runs completely normally -- every IO
        // class still calls updateInputs() -- but the values it receives are whatever was
        // recorded that day, not live hardware. setUseTiming(false) skips the normal 20ms
        // real-time wait between loops so the whole match replays as fast as your computer can
        // process it instead of taking the full 2:15. The replayed run gets written back out to
        // its own new log file (with a "_sim" suffix) so you can diff logging output between the
        // original run and the replay if you're debugging a logging change itself.
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Actually start the logger now that all the receivers/replay source above are configured.
    // Nothing gets logged before this call.
    Logger.start();

    // Build every subsystem and wire up all button bindings. Must happen after Logger.start() so
    // that any @AutoLog inputs created during subsystem construction are captured correctly.
    robotContainer = new RobotContainer();
  }

  /**
   * Called once per 20ms loop, in every mode (disabled, autonomous, teleop, test). This is the
   * only method that runs unconditionally every loop -- everything else below only fires during
   * its specific mode.
   */
  @Override
  public void robotPeriodic() {
    // Briefly bump this thread to real-time priority so the critical command-scheduling/logging
    // work below isn't preempted by less important background threads, which keeps loop timing
    // consistent. Restored to normal priority at the end of the method -- don't remove the
    // restore call, or every other thread on the RIO gets starved.
    Threads.setCurrentThreadPriority(true, 99);

    // This is the heart of WPILib's command-based framework: every loop, the scheduler checks
    // each Trigger (button binding) for state changes, starts/stops commands accordingly, calls
    // execute() on every currently-running command, and calls periodic() on every registered
    // Subsystem (which is where each XyzIO's updateInputs()/logging happens). If this line is
    // ever removed, buttons stop working and no subsystem periodic() methods run at all.
    CommandScheduler.getInstance().run();

    Threads.setCurrentThreadPriority(false, 10);
  }

  /** Called once, the instant the robot becomes disabled (including at power-on). */
  @Override
  public void disabledInit() {}

  /** Called every loop while disabled. */
  @Override
  public void disabledPeriodic() {}

  /**
   * Called once at the start of autonomous. Grabs whichever command is currently selected in
   * {@link RobotContainer}'s auto chooser and schedules it to actually run.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** Called every loop during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** Called once at the start of teleop (driver control). */
  @Override
  public void teleopInit() {
    // If autonomous was still running a command when the match transitioned to teleop, cancel it
    // so it doesn't keep fighting the driver for control of the same subsystems. If you ever want
    // an autonomous routine to keep running into teleop (e.g. a slow closing sequence), remove
    // this cancel -- but that's an unusual choice, most teams want a clean handoff to the driver.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** Called every loop during teleop. */
  @Override
  public void teleopPeriodic() {}

  /** Called once at the start of test mode. */
  @Override
  public void testInit() {
    // Test mode is meant to start from a clean slate -- cancel absolutely everything (including
    // default commands) so test-mode-specific commands aren't fighting anything left over.
    CommandScheduler.getInstance().cancelAll();
  }

  /** Called every loop during test mode. */
  @Override
  public void testPeriodic() {}

  /** Called once when the simulator starts (SIM mode only, never on real hardware). */
  @Override
  public void simulationInit() {}

  /** Called every loop while running in the simulator (SIM mode only). */
  @Override
  public void simulationPeriodic() {}
}
