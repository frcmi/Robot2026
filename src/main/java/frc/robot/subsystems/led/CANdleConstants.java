package frc.robot.subsystems.led;

import static frc.robot.constants.RobotConstants.kCanivoreBus;

import com.ctre.phoenix6.CANBus;
import frc.robot.lib.LoggedTunableNumber;

public class CANdleConstants {
  public static final CANBus m_candleCanBus = kCanivoreBus;
  public static final int m_candleCanId = 0;
  public static LoggedTunableNumber m_candleBrightnessScalar =
      new LoggedTunableNumber("CANdle/BrightnessScalar", 1.0);
  public static final int k_candleNumLEDs = 40;
}
