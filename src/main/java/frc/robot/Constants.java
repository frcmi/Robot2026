// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // These variables affect only the REAL runmode (on an actual robot).
  // They allow you to prevent subsystems that don't exist from being initialized,
  // skipping over looking for hardware that doesn't exist.
  public static boolean driveHardwareExists = true;
  public static boolean climbHardwareExists = false;
  public static boolean intakeHardwareExists = true;
  public static boolean shooterHardwareExists = true;
  public static boolean visionHardwareExists = true;
  public static boolean ledHardwareExists = true;

  // Set true during tuning sessions to allow dashboard-driven PID/table edits.
  // MUST be false at competition: every LoggedTunableNumber.get() and
  // LoggedInterpolatingTable.get() is a NetworkTables read, and getTableFromDashboard()
  // allocates a new TreeMap on every call — causing GC spikes inside the aiming loop.
  public static boolean kTuningMode = false;

  // Enable per-subsystem loop-time logging. Useful for diagnosing spikes; keep false
  // in normal operation to avoid the overhead of ~20 extra Timer.getFPGATimestamp()
  // calls per loop.
  public static boolean kEnableLoopTimingLogs = false;
}
