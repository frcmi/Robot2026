package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

// Timing
public class TimingUtil {
  public static double angularTime = 0;
  public static double angularTimeLogging = 0;

  public static void resetTime() {
    angularTime = 0;
    angularTimeLogging = 0;
  }

  public static void logTime() {
    Logger.recordOutput("Timing/AngularSubsystemMS", angularTime * 1e3);
    Logger.recordOutput("Timing/AngularSubsystemLoggingMS", angularTimeLogging * 1e3);
  }

  public static void addTimeAngular(double time) {
    angularTime += time;
  }

  public static void addTimeAngularLogging(double time) {
    angularTimeLogging += time;
  }
}
