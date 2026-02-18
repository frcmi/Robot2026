package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.LoggedInterpolatingTable;
import frc.robot.lib.LoggedTunableNumber;

public class AimingConstants {
  public static final Translation2d kHubPositionBlue = new Translation2d(4.03, 4.03); // Meters
  public static final Translation2d kHubPositionRed = new Translation2d(12.51, 4.03); // Meters

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
            {0.0, 0.0},
            {0.2, 5.0},
            {0.37, 15.0},
            {0.81, 60.0},
            {1.4, 45.0}
          });

  // No idea where these numbers even came from, copilot just made them up but whatever :shrug:
  public static LoggedInterpolatingTable kFlywheelSpeedTable =
      new LoggedInterpolatingTable("Shooter/FlywheelSpeedM_RPS", new double[][] {{0.0, 0.0}});
  public static LoggedInterpolatingTable kAirtimeTable =
      new LoggedInterpolatingTable(
          "Shooter/AirtimeM_s",
          new double[][] {
            {0.0, 0.2},
          });
}
