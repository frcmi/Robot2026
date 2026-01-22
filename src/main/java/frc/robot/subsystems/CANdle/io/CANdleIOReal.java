package frc.robot.subsystems.CANdle.io;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

public class CANdleIOReal implements CANdleIO {

  private final CANdle m_candle = new CANdle(1, "rio");
  private ControlRequest request;

  public CANdleIOReal() {
    var candleconfig = new CANdleConfiguration();

    candleconfig.LED.StripType = StripTypeValue.GRB;
    candleconfig.LED.BrightnessScalar = 0.5;

    candleconfig.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

    m_candle.getConfigurator().apply(candleconfig);

    for (int i = 0; i < 24; ++i) {
      m_candle.setControl(new EmptyAnimation(i));
    }
    request = new EmptyAnimation(0);
  }

  @Override
  public void setControl(ControlRequest request) {
    m_candle.setControl(request);
    this.request = request;
  }
  ;

  @Override
  public void updateInputs(CANdleIOInputs inputs) {
    inputs.animationName = request.getName();
    inputs.animation = request.toString();
  }
  ;
}
