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
            {2, 26},
            {2.5, 26},
            {3, 28},
            {3.5, 32},
            {4, 33.8},
            {4.5, 34.5},
            {5, 35},
            {6, 36},
          });
  public static LoggedInterpolatingTable kFlywheelSpeedTable =
      new LoggedInterpolatingTable(
          "Shooter/FlywheelSpeedM_RPS",
          new double[][] {
            {2, 27},
            {2.5, 29},
            {3, 33},
            {3.5, 34},
            {4, 37},
            {4.5, 44},
            {5, 47},
            {6, 57}
          });
  public static LoggedInterpolatingTable kAirtimeTable =
      new LoggedInterpolatingTable(
          "Shooter/AirtimeM_s",
          new double[][] {
            {2, 0.76},
            {3, 1.11},
            {4.5, 1.15},
            {6.0, 1.1},
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
