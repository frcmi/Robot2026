package frc.robot.constants.shooter;

import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
  public static final Translation2d kHubPositionBlue =
      new Translation2d(4.62534, 4.03479); // Meters
  public static final Translation2d kHubPositionRed =
      new Translation2d(11.91641, 4.03479); // Meters
  public static final double trenchWidthX = 2; // Meters, from hub center
  public static final double trenchWidthY = 1.6; // Meters
  public static final double hubWidthY = 1.2; // Meters
  public static final double bumpWidthY = 2.5; // Meters
  public static final double fieldWidthY = 8.06958; // Meters

  public static final double allianceZoneXBlue = 2.4; // Meters, where we shoot when in neutral zone
  public static final double allianceZoneXRed = 16.54175 - allianceZoneXBlue; // Meters
  public static final double allianceZoneYBottom =
      1.7; // Meters, where we shoot when in neutral zone and "below" the hub
  public static final double allianceZoneYTop = fieldWidthY - allianceZoneYBottom;

  // Tolerances for aiming
  public static final double hubWidth =
      1.1 / 2; // Diameter of hub (meters), this is tolerance of the aiming so lower is more strict
  public static final double bumpWidth =
      3.5; // This is just the width that we can shoot over correctly when in the neutral zone
}
