package frc.robot.constants.shooter;

import static edu.wpi.first.units.Units.Degrees;

import frc.robot.lib.LoggedInterpolatingTable;
import frc.robot.lib.LoggedTunableNumber;

public class AimingConstants {
  public static LoggedTunableNumber kTurretMinAngle =
      new LoggedTunableNumber(
          "Shooter/Turret/MinAngleDeg", TurretConstants.kTurretPhysicalMinAngle.in(Degrees));
  public static LoggedTunableNumber kTurretMaxAngle =
      new LoggedTunableNumber(
          "Shooter/Turret/MaxAngleDeg", TurretConstants.kTurretPhysicalMaxAngle.in(Degrees));

  // Regular aiming
  public static LoggedInterpolatingTable kHoodAngleTable =
      new LoggedInterpolatingTable(
          "Shooter/HoodAngleM_Deg",
          new double[][] {
            {0, 28},
            {0.9, 28},
            {1, 28},
            {1.5, 28.5},
            {2, 28.5},
            {2.5, 28.5},
            {3, 30.5},
            {3.5, 31},
            {4, 32},
            {4.5, 32},
            {5, 35.5},
          });
  public static LoggedInterpolatingTable kFlywheelSpeedTable =
      new LoggedInterpolatingTable(
          "Shooter/FlywheelSpeedM_RPS",
          new double[][] {
            {0, 22},
            {2, 24},
            {2.5, 26},
            {3, 27.5},
            {3.5, 32},
            {4, 36.5},
            {4.5, 49},
            {5, 49},
            {6, 52},
          });
  public static LoggedInterpolatingTable kAirtimeTable =
      new LoggedInterpolatingTable(
          "Shooter/AirtimeM_s",
          new double[][] {
            {2, 1.0},
            {3, 1.05},
            {4, 1.1},
            {5.0, 1.15},
          });

  // Neutral zone aiming
  public static LoggedInterpolatingTable kHoodAngleTableNeutral =
      new LoggedInterpolatingTable(
          "ShooterNeutral/HoodAngleM_Deg",
          new double[][] {
            {0, 43.2},
          });
  public static LoggedInterpolatingTable kFlywheelSpeedTableNeutral =
      new LoggedInterpolatingTable(
          "ShooterNeutral/FlywheelSpeedM_RPS",
          new double[][] {
            {2, 20},
            {4, 25},
            {4.5, 30},
            {5, 35},
            {7, 55},
          });
  public static LoggedInterpolatingTable kAirtimeTableNeutral =
      new LoggedInterpolatingTable(
          "ShooterNeutral/AirtimeM_s",
          new double[][] {
            {5.0, 1.15},
          });
}
