package frc.robot.lib;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.subsystem.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Virtual subsystem that automatically updates all LoggedInterpolatingTable instances. Instantiate
 * this once in RobotContainer to enable automatic table updates.
 */
public class LoggedInterpolatingTableManager extends VirtualSubsystem {
  private static LoggedInterpolatingTableManager instance;

  public LoggedInterpolatingTableManager() {
    if (instance != null) {
      throw new IllegalStateException(
          "LoggedInterpolatingTableManager already exists! Only create one instance.");
    }
    instance = this;
  }

  @Override
  public void periodic() {
    double startTime = Timer.getFPGATimestamp();

    // Update all registered tables
    LoggedInterpolatingTable.periodicAll();

    double endTime = Timer.getFPGATimestamp();
    Logger.recordOutput("Timing/LoggedInterpolatingTableManagerMS", (endTime - startTime) * 1e3);
  }

  public static LoggedInterpolatingTableManager getInstance() {
    return instance;
  }
}
