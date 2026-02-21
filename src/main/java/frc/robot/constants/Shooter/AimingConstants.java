package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.LoggedInterpolatingTable;
import frc.robot.lib.LoggedTunableNumber;

public class AimingConstants {
  public static final Translation2d kHubPositionBlue = new Translation2d(4.03, 4.03); // Meters
  public static final Translation2d kHubPositionRed = new Translation2d(12, 4.03); // Meters
  public static final double trenchOffsetY = 2.75; // Meters
  public static final double trenchOffsetX = 0.5; // Meters

  public static LoggedTunableNumber kTrenchOffsetY =
      new LoggedTunableNumber("Shooter/TrenchOffset/Y", trenchOffsetY);
  public static LoggedTunableNumber kTrenchOffsetX =
      new LoggedTunableNumber("Shooter/TrenchOffset/X", trenchOffsetX);
  public static LoggedTunableNumber kTurretMinAngle =
      new LoggedTunableNumber(
          "Shooter/Turret/MinAngleDeg", TurretConstants.kTurretPhysicalMinAngle.in(Degrees));
  public static LoggedTunableNumber kTurretMaxAngle =
      new LoggedTunableNumber(
          "Shooter/Turret/MaxAngleDeg", TurretConstants.kTurretPhysicalMaxAngle.in(Degrees));

  // TODO: Tune, right now it's from my launch angle calculator
  public static LoggedInterpolatingTable kHoodAngleTable =
      new LoggedInterpolatingTable(
          "Shooter/HoodAngleM_Deg",
          new double[][] {
            {0.0, 10},
            {0.9, 10.1},
            {1, 11},
            {2, 12},
            {3, 10},
            {4, 14},
            {5, 15}
          });

  // No idea where these numbers even came from, copilot just made them up but whatever :shrug:
  public static LoggedInterpolatingTable kFlywheelSpeedTable =
      new LoggedInterpolatingTable(
          "Shooter/FlywheelSpeedM_RPS",

          new double[][] {{0, 0}, {0.9, 0.1}, {1, 25}, {2, 26}, {3, 30}, {4, 35}, {5, 40}});
  public static LoggedInterpolatingTable kAirtimeTable =
      new LoggedInterpolatingTable(
          "Shooter/AirtimeM_s",
          new double[][] {
            {1.6, 0.72},
            {3.184, 1.07},
          });
}
