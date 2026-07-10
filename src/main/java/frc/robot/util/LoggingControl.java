// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * A single dashboard-controllable "kill switch" for detailed AdvantageKit logging, added because
 * the real 2023 robot's roboRIO 1 (256MB total RAM, much less than a roboRIO 2) can run low on
 * memory if logging volume gets too high -- this gives the driver a way to cut that load
 * immediately from Elastic mid-match if it's ever contributing to instability, without needing to
 * redeploy code.
 *
 * <p>Every subsystem's {@code periodic()} follows the same shape: {@code io.updateInputs(inputs)}
 * always runs unconditionally (sensor reading and the control logic that depends on it, like the
 * elevator's bottom-limit auto-zero, must never be affected by this toggle), but the AdvantageKit
 * recording calls that follow it ({@code Logger.processInputs(...)} and any extra {@code
 * Logger.recordOutput(...)} calls) are wrapped in {@code if (LoggingControl.enabled())}.
 * {@code Logger.processInputs} is the expensive part -- it's what serializes the entire inputs
 * struct and hands it to every registered data receiver (the USB log file writer and the live NT4
 * publisher) every single loop, for every subsystem, which is the dominant cost this toggle
 * actually reduces.
 *
 * <p>{@link #enabled()} always returns {@code true} outside of {@link Mode#REAL} -- disabling
 * {@code processInputs} during {@link Mode#REPLAY} would mean recorded values never get replayed
 * back into a subsystem's inputs at all, breaking replay entirely for as long as the toggle was
 * off; since the memory constraint this exists for is specific to the real roboRIO 1 hardware, sim
 * and replay (which run on a desktop with vastly more RAM) simply aren't affected by it.
 */
public final class LoggingControl {
  private LoggingControl() {}

  // Plain NT4 path (not "/SmartDashboard/...") so it shows up in Elastic without depending on the
  // legacy SmartDashboard/Shuffleboard Java APIs -- same convention as Vision/Leds' toggles. Bind
  // to a toggle-switch widget in the Elastic layout. Defaults to true (full logging) since the
  // memory issue this addresses is intermittent, not constant -- you want detailed logs by
  // default and only cut them if a match actually starts showing trouble.
  private static final LoggedNetworkBoolean detailedLoggingEnabled =
      new LoggedNetworkBoolean("/DriverDashboard/LoggingEnabled", true);

  /** Whether subsystems should record detailed AdvantageKit inputs/outputs this loop. */
  public static boolean enabled() {
    return Constants.currentMode != Mode.REAL || detailedLoggingEnabled.get();
  }
}
