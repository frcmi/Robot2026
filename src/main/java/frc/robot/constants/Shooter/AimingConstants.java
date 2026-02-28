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

  public static LoggedInterpolatingTable kHoodAngleTable =
      new LoggedInterpolatingTable(
          "Shooter/HoodAngleM_Deg",
          new double[][] {
            {0, 25.2},
            {0.9, 25.3},
            {1, 26.2},
            {1.5, 26.7},
            {2, 27.2},
            {2.5, 26.2},
            {3, 25.2},
            {3.5, 26},
            {4, 30},
            {4.5, 31},
            {5, 32},
          });

  // No idea where these numbers even came from, copilot just made them up but whatever :shrug:
  public static LoggedInterpolatingTable kFlywheelSpeedTable =
      new LoggedInterpolatingTable(
          "Shooter/FlywheelSpeedM_RPS",
          new double[][] {
            {2, 26},
            {2.5, 26},
            {3, 30},
            {3.5, 33},
            {4, 36},
            {4.5, 55},
            {5, 60},
          });

  public static LoggedInterpolatingTable kAirtimeTable =
      new LoggedInterpolatingTable(
          "Shooter/AirtimeM_s",
          new double[][] {
            {1.6, 0.72},
            {3.184, 1.07},
          });
}
