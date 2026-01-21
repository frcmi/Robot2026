package frc.robot.subsystems.CANdle.io;

import com.ctre.phoenix6.controls.ControlRequest;

public class CANdleIOSim implements CANdleIO {

  private ControlRequest request;

  @Override
  public void setControl(ControlRequest request) {
    this.request = request;
  }
  ;

  @Override
  public void updateInputs(CANdleIOInputs inputs) {
    inputs.animationName = request.getName();
  }
  ;
}
