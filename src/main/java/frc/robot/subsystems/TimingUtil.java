package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;

// Timing
public class TimingUtil {
  public static double angularTime = 0;
  public static void resetTime() {
    angularTime = 0;
  }
  public static void logTime() {
    Logger.recordOutput("Timing/AngularSubsystemMS", angularTime * 1e3);
  }
  public static void addTime(double time) {
    angularTime += time;
    Timer.getFPGATimestamp();
  }
}