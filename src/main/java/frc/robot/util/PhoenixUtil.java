// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

/** Small shared helper used by every TalonFX-based IO implementation in this codebase. */
public class PhoenixUtil {
  /**
   * Attempts to run the command until no error is produced.
   *
   * <p>Every motor controller config call in this codebase goes through here rather than being
   * called directly, because Phoenix6 config writes go out over the CAN bus and occasionally get
   * dropped (bus contention, timing, etc.) -- silently accepting a failed config would leave a
   * motor running with the wrong gains/limits/settings and no obvious sign anything went wrong.
   * Retrying a few times catches those transient failures cheaply at startup.
   */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) {
        break;
      }
    }
  }
}
