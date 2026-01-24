package frc.robot.constants.Shooter;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.LoggedInterpolatingTable;

public class AimingConstants {
  public static final Translation2d kHubPositionBlue = new Translation2d(4.03, 4.03); // Meters
  public static final Translation2d kHubPositionRed = new Translation2d(12.1, 4.07); // Meters

  public static LoggedInterpolatingTable kHoodAngleTable =
      new LoggedInterpolatingTable(
          "Shooter/HoodAngleM_Deg",
          new double[][] {
            {0.0, 0.0},
            {0.63, 5.0}, // at hub
            {2, 17.0}, // random data point
            {2.7, 20.0}, // tower
            {5.3, 30.0} // far corner
          });

  public static LoggedInterpolatingTable kFlywheelSpeedTable =
      new LoggedInterpolatingTable(
          "Shooter/FlywheelSpeedM_RPS",
          new double[][] {
            {0.0, 0.0},
            {0.63, 40.0}, // at hub
            {2.0, 44.0}, // random data point
            {2.7, 47.0}, // tower
            {5.3, 55.0} // far corner
          });
  public static LoggedInterpolatingTable kAirtimeTable =
      new LoggedInterpolatingTable(
          "Shooter/AirtimeM_s",
          new double[][] {
            {0.0, 0.0},
            {0.57, 0.9}, // at hub
            {0.91, 1.1}, // random data point
            {2.66, 1.22}, // tower
            {5.375, 1.36}, // far corner
          });
}
