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
            {2, 27.2},
            {3, 25.2},
            {4, 29},
            {5, 30.2},
            {3.5, 26},
            {3.75, 27},
            {4.5, 33}
          });

  // No idea where these numbers even came from, copilot just made them up but whatever :shrug:
  public static LoggedInterpolatingTable kFlywheelSpeedTable =
      new LoggedInterpolatingTable(
          "Shooter/FlywheelSpeedM_RPS",
          new double[][] {
            {0, 0},
            {0.9, 0.1},
            {1, 25},
            {2, 26},
            {3, 30},
            {4, 36},
            {5, 40},
            {3.5, 33},
            {3.75, 34},
            {4.5, 41}
          });
  
  public static LoggedInterpolatingTable kAirtimeTable =
      new LoggedInterpolatingTable(
          "Shooter/AirtimeM_s",
          new double[][] {
            {1.6, 0.72},
            {3.184, 1.07},
          });
}
