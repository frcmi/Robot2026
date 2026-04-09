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

  public static LoggedTunableNumber kAirtimeMultiplier =
      new LoggedTunableNumber("Shooter/AirtimeMultiplier", 1);

  // Regular aiming
  public static LoggedInterpolatingTable kHoodAngleTable =
      new LoggedInterpolatingTable(
          "Shooter/HoodAngleM_Deg",
          new double[][] {
            {2, 26},
            {2.5, 26},
            {3, 28},
            {3.5, 31},
            {4, 32},
            {4.5, 35},
            {5, 36},
            {6, 37},
          });
  public static LoggedInterpolatingTable kFlywheelSpeedTable =
      new LoggedInterpolatingTable(
          "Shooter/FlywheelSpeedM_RPS",
          new double[][] {
            {2, 25},
            {2.5, 27},
            {3, 31},
            {3.5, 32},
            {4, 34},
            {4.5, 38.5},
            {5, 41},
            {6, 45}
          });
  public static LoggedInterpolatingTable kAirtimeTable =
      new LoggedInterpolatingTable(
          "Shooter/AirtimeM_s",
          new double[][] {
            {2, 0.912},
            {3, 1.33},
            {4.5, 1.3},
            {6.0, 1.3},
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
            {5, 35},
            {7, 55},
            {8, 65},
            {9, 75},
            {13, 75},
          });
  public static LoggedInterpolatingTable kAirtimeTableNeutral =
      new LoggedInterpolatingTable(
          "ShooterNeutral/AirtimeM_s",
          new double[][] {
            {5.0, 1.15},
          });
}
