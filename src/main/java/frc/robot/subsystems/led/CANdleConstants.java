package frc.robot.subsystems.led;

import static frc.robot.constants.RobotConstants.kRioBus;

import com.ctre.phoenix6.CANBus;
import frc.robot.lib.LoggedTunableNumber;

public class CANdleConstants {
  public static final CANBus m_candleCanBus = kRioBus;
  public static final int m_candleCanId = 1;
  public static LoggedTunableNumber m_candleBrightnessScalar =
      new LoggedTunableNumber("CANdle/BrightnessScalar", 0.5);
  public static final int k_candleNumLEDs = 40;
}
