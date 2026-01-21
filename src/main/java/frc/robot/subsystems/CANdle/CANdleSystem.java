package frc.robot.subsystems.CANdle;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleSystem extends SubsystemBase {

  private static final int SlotStartIdx = 8;
  private static final int SlotEndIdx = 37;

  private final CANdle m_candle = new CANdle(1, "rio");

  public CANdleSystem() {
    var candleconfig = new CANdleConfiguration();

    candleconfig.LED.StripType = StripTypeValue.GRB;
    candleconfig.LED.BrightnessScalar = 0.5;

    candleconfig.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

    m_candle.getConfigurator().apply(candleconfig);

    for (int i = 0; i < 24; ++i) {
      m_candle.setControl(new EmptyAnimation(i));
    }
  }

  private Command setAnimFlow() {
    return run(() -> {
          m_candle.setControl(
              new ColorFlowAnimation(SlotStartIdx, SlotEndIdx)
                  .withSlot(0)
                  .withColor(new RGBWColor(0, 217, 0, 0)));
        });
  }
}
