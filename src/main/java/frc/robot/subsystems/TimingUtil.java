package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

// Timing
public class TimingUtil {
  public static long angularTime = 0;
  public static void resetTime() {
    angularTime = 0;
  }
  public static void logTime() {
    Logger.recordOutput("Timing/AngularSubsystemMS", angularTime / 1e6);
  }
  public static void addTime(long time) {
    angularTime += time;
  }
}