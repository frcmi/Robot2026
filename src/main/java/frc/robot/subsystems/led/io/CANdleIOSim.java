package frc.robot.subsystems.led.io;

import com.ctre.phoenix6.controls.ControlRequest;

public class CANdleIOSim implements CANdleIO {

  public CANdleIOSim() {}

  private ControlRequest request;

  @Override
  public void setControl(ControlRequest request) {
    this.request = request;
  }

  @Override
  public void updateInputs(CANdleIOInputs inputs) {
    if (request == null) {
      return;
    }
    inputs.animationName = request.getName();
    inputs.animation = request.toString();
  }
}
