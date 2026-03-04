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
            {0, 33.2},
            {0.9, 33.3},
            {1, 34.2},
            {1.5, 34.7},
            {2, 35.2},
            {2.5, 34.2},
            {3, 33.2},
            {3.5, 34},
            {4, 38},
            {4.5, 39},
            {5, 40},
          });
  public static LoggedInterpolatingTable kFlywheelSpeedTable =
      new LoggedInterpolatingTable(
          "Shooter/FlywheelSpeedM_RPS",
          new double[][] {
            {2, 22},
            {2.5, 22},
            {3, 26},
            {3.5, 29},
            {4, 32},
            {4.5, 51},
            {5, 52},
          });
  public static LoggedInterpolatingTable kAirtimeTable =
      new LoggedInterpolatingTable(
          "Shooter/AirtimeM_s",
          new double[][] {
            {2, 0.95},
            {3, 1.0},
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
          });
  public static LoggedInterpolatingTable kAirtimeTableNeutral =
      new LoggedInterpolatingTable(
          "ShooterNeutral/AirtimeM_s",
          new double[][] {
            {5.0, 1.15},
          });
}
