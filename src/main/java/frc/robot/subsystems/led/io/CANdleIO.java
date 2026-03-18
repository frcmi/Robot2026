package frc.robot.subsystems.led.io;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;

import org.littletonrobotics.junction.AutoLog;

public interface CANdleIO {

  @AutoLog
  class CANdleIOInputs {
    public String animationName = new EmptyAnimation(0).getName();
    public String animation = new EmptyAnimation(0).toString();
  }

  default void setControl(ControlRequest request) {}
  ;

  default void updateInputs(CANdleIOInputs inputs) {}
  ;
}
