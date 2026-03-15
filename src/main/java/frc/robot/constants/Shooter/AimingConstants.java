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
            {0, 27.8},
            {0.9, 27.8},
            {1, 27.8},
            {1.5, 27.8},
            {2, 27.8},
            {2.5, 30},
            {3, 31},
            {3.5, 32.5},
            {4, 32.5},
            {4.5, 32.8},
            {5, 35.5},
          });
  public static LoggedInterpolatingTable kFlywheelSpeedTable =
      new LoggedInterpolatingTable(
          "Shooter/FlywheelSpeedM_RPS",
          new double[][] {
            {0, 19},
            {2, 21.5},
            {2.5, 24},
            {3, 26.3},
            {3.5, 30.5},
            {4, 33},
            {4.5, 35.5},
            {5, 36},
            {6, 37},
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
