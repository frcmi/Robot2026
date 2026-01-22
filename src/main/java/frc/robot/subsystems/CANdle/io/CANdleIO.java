package frc.robot.subsystems.CANdle.io;

import com.ctre.phoenix6.controls.ControlRequest;
import org.littletonrobotics.junction.AutoLog;

public interface CANdleIO {

  @AutoLog
  class CANdleIOInputs {
    public String animationName;
    public String animation;
  }

  default void setControl(ControlRequest request) {}
  ;

  default void updateInputs(CANdleIOInputs inputs) {}
  ;
}
