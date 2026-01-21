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
            {0.2, 5.0}, // at hub
            {1, 15.0},
            {1.2, 20.0},
            {2.2, 25.0} // tower, corner
          });

  // No idea where these numbers even came from, copilot just made them up but whatever :shrug:
  public static LoggedInterpolatingTable kFlywheelSpeedTable =
      new LoggedInterpolatingTable(
          "Shooter/FlywheelSpeedM_RPS",
          new double[][] {
            {0.0, 0.0},
            {0.2, 40.0}, // at hub
            {1.0, 40.0},
            {2.2, 44.0}, // tower
            {2.5, 44.0},
            {2.8, 47.0},
            {4.8, 1000.0} // corner
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
