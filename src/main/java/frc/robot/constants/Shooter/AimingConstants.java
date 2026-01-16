package frc.robot.constants.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.LoggedInterpolatingTable;

public class AimingConstants {
  public static final Translation2d kHubPositionBlue = new Translation2d(4.03, 4.03); // Meters
  public static final Translation2d kHubPositionRed = new Translation2d(12.51, 4.03); // Meters

  // TODO: Tune, right now it's from my launch angle calculator
  public static LoggedInterpolatingTable kHoodAngleTable =
      new LoggedInterpolatingTable(
          "Shooter/HoodAngleM_Deg",
          new double[][] {
            {0.0, 0.0},
            {0.37, 15.0},
            {0.81, 60.0},
            {1.4, 45.0}
          });

  // No idea where these numbers even came from, copilot just made them up but whatever :shrug:
  public static LoggedInterpolatingTable kFlywheelSpeedTable =
      new LoggedInterpolatingTable(
          "Shooter/FlywheelSpeedM_RPS",
          new double[][] {
            {0.0, 0.0},
            {0.37, 20.0},
            {0.81, 35.0},
            {1.4, 50.0},
            {1.8, 65.0},
            {2.2, 80.0},
            {3.0, 100.0}
          });
  public static LoggedInterpolatingTable kAirtimeTable =
      new LoggedInterpolatingTable(
          "Shooter/AirtimeM_s",
          new double[][] {
            {0.0, 0.0},
            {0.37, 0.5},
            {0.81, 0.8},
            {1.4, 1.2},
            {1.8, 1.5},
            {2.2, 1.8},
            {3.0, 2.2}
          });
}
