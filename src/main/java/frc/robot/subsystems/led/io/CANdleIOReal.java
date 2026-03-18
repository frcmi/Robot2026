package frc.robot.subsystems.led.io;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import frc.robot.subsystems.led.CANdleConstants;

public class CANdleIOReal implements CANdleIO {

  private final CANdle m_candle =
      new CANdle(CANdleConstants.m_candleCanId, CANdleConstants.m_candleCanBus);
  private ControlRequest request;
  private CANdleConfiguration candleconfig = new CANdleConfiguration();

  public CANdleIOReal() {
    candleconfig.LED.StripType = StripTypeValue.GRB;
    candleconfig.LED.BrightnessScalar = CANdleConstants.m_candleBrightnessScalar.get();
    candleconfig.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

    m_candle.getConfigurator().apply(candleconfig);

    for (int i = 0; i < 8; i++) { // Clear all 8 CANDle slots
      m_candle.setControl(new EmptyAnimation(i));
    }
    request = new EmptyAnimation(0);
  }

  @Override
  public void setControl(ControlRequest request) {
    m_candle.setControl(request);
    this.request = request;
  }

  @Override
  public void updateInputs(CANdleIOInputs inputs) {
    if (request == null) {
      return;
    }
    inputs.animationName = request.getName();
    inputs.animation = request.toString();

    if (CANdleConstants.m_candleBrightnessScalar.hasChanged(
        CANdleConstants.m_candleBrightnessScalar.hashCode())) {
      candleconfig.LED.BrightnessScalar = CANdleConstants.m_candleBrightnessScalar.get();
      m_candle.getConfigurator().apply(candleconfig);
    }
  }
}
