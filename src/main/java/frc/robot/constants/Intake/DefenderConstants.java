package frc.robot.constants.intake;

import frc.robot.lib.LoggedTunableNumber;

public class DefenderConstants {
  public static final int DEFENDER_PORT = 2;
  public static LoggedTunableNumber defenderClosed =
      new LoggedTunableNumber("Defender/DefenderClosed", 0.5);
  public static LoggedTunableNumber defenderOpen =
      new LoggedTunableNumber("Defender/DefenderOpen", 0.5);
}
